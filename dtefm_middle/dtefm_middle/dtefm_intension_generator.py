import rclpy
import sys
import os
import ast
import time
import torch
import numpy as np
from rclpy.node import Node
from dtefm_interfaces.msg import SRStateRobot, PlaceMsg, TransitionMsg, ArcMsg, PetriNet
from dtefm_interfaces.srv import SRTcpCommunication, SRState, PNCommand, IntensionControlSrv, GCPNSrv
from ament_index_python.packages import get_package_share_directory

package_share_directory = get_package_share_directory('dtefm_middle')
library_path = os.path.join(package_share_directory, 'resource/pntk')
sys.path.append(library_path)
from elements import Place, Transition, Arc
from petri_net import ColoredPetriNet
from example_nets import PlainNet

package_share_directory = get_package_share_directory('dtefm_middle')
library_path = os.path.join(package_share_directory, 'resource/rltk')
sys.path.append(library_path)
from pn_agent import Agent_PPO

class IntensionGenerator(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info(f"Intension Core node {name} initialized..")
        self.pn_client_ = self.create_client(PNCommand, '/identity/pn_srv')
        self.intension_control_client_ = self.create_client(IntensionControlSrv, '/identity/agent/intension_control_srv')

        self.intension_generate_server_ = self.create_service(GCPNSrv, '/identity/agent/intension_srv', self.intension_generate_callback)
        self.actor_lr = 5e-5
        self.critic_lr = 1e-3
        self.num_episodes = 500
        self.hidden_dim = 128
        self.gamma = 0.99
        self.lmbda = 0.95
        self.epochs = 20
        self.epsilon = 0.2      # greedy
        self.eps = 0.1
        self.device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
        self.pn_adj = None
        self.intension_agent = None
        self.update_pn()
        
    def update_pn(self):
        request = PNCommand.Request()
        request.command = 'RPRN'
        self.pn_client_.call_async(request).add_done_callback(self.update_pn_adj)
    
    def update_pn_adj(self, result):
        response = result.result()
        places = response.places
        transitions = response.transitions
        arcs = response.arcs
        
        self.pn_adj = np.zeros((len(places), len(transitions)+1))
        
        for arc in arcs:
            if arc.direction == 'PtoT':
                i = self.find_node_index(arc.node_in, places)
                j = self.find_node_index(arc.node_out, transitions)
                self.pn_adj[i][j] = -1
            elif arc.direction == 'TtoP':
                i = self.find_node_index(arc.node_out, places)
                j = self.find_node_index(arc.node_in, transitions)
                self.pn_adj[i][j] = 1
        self.get_logger().info(f"pn_adj updated {self.pn_adj}")
    
    def intension_control(self, target_state):
        request = IntensionControlSrv.Request()
        request.state = target_state
        self.intension_control_client_.call_async(request)
    
    def find_node_index(self, node_id, node_list):
        for i in range(len(node_list)):
            if node_id == node_list[i].id:
                return i
        
    def agent_ppo_initialize(self, actor_lr, critic_lr, lmbda, epochs, epsilon, eps, gamma, device):
        self.update_pn()
        time.sleep(0.5)
        self.action_dim = self.pn_adj.shape[1]
        self.intension_agent = Agent_PPO(self.lp, self.lt, self.action_dim, self.pn_adj, actor_lr, critic_lr, lmbda, epochs, epsilon, eps, gamma, device)
        package_share_directory = get_package_share_directory('dtefm_middle')
        actor_state_dict_path = os.path.join(package_share_directory, 'trained_memory/PPO_actor_lock.pth')
        critic_state_dict_path = os.path.join(package_share_directory, 'trained_memory/PPO_critic_lock.pth')
        self.load_trained_model(actor_state_dict_path, critic_state_dict_path)
        
    
    def load_trained_model(self, actor_state_dict_path, critic_state_dict_path):
        actor_state_dict = torch.load(actor_state_dict_path)
        critic_state_dict = torch.load(critic_state_dict_path)
        
        self.intension_agent.actor.load_state_dict(actor_state_dict)
        self.intension_agent.critic.load_state_dict(critic_state_dict)
        
    def intension_generate_callback(self, request, response):
        self.dim_p = request.state.dim_p
        self.dim_t = request.state.dim_t
        self.lp = request.state.lp
        self.lt = request.state.lt
        
        p_state = np.array(request.state.p_state).reshape(self.dim_p, self.lp)
        t_state = np.array(request.state.t_state).reshape(self.dim_t, self.lt)
        
        if self.intension_agent == None:
            self.agent_ppo_initialize(self.actor_lr, self.critic_lr, self.lmbda, self.epochs, self.epsilon, self.eps, self.gamma, self.device)
        action = self.intension_agent.actor(p_state, t_state)
        
        response.action_index = action
        return response
    

def main(args=None):
    rclpy.init(args=args)
    
    node = IntensionGenerator('identity')
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()