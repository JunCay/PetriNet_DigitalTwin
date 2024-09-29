import rclpy
import sys
import os
import ast
from rclpy.node import Node
from dtefm_interfaces.msg import SRStateRobot, PlaceMsg, TransitionMsg, ArcMsg, PetriNet, GCPNStateMsg
from dtefm_interfaces.srv import SRTcpCommunication, SRState, PNCommand, GCPNSrv, IntensionGateControlSrv, IntensionExpressControlSrv
from ament_index_python.packages import get_package_share_directory
# from src.dtefm_middle.resource.pntk.example_nets import PlainNet
# from src.dtefm_middle.resource.sr_state.sr_state import SR_State

package_share_directory = get_package_share_directory('dtefm_middle')
library_path = os.path.join(package_share_directory, 'resource/pntk')
sys.path.append(library_path)
from elements import Place, Transition, Arc
from petri_net import ColoredPetriNet
from example_nets import PlainNet

library_path = os.path.join(package_share_directory, 'resource/srtk')
sys.path.append(library_path)
print(library_path)
from sr_state import SR_State

class Identity(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info(f"Identity node {name} initialized..")
        self.sr_state_p = SR_State()
        self.sr_state_s = SR_State()
        self.sr_state_robot_subscriber_ = self.create_subscription(SRStateRobot, '/sr/robot/state/physical', self.sr_state_robot_physical_callback, 10)
        self.sr_state_server_ = self.create_service(SRState, '/identity/sr/state_srv', self.sr_state_srv_callback)
        
        self.intension_generator_publisher_ = self.create_publisher(GCPNStateMsg, '/identity/agent/state', 10)
        self.intension_generator_clinent_ = self.create_client(GCPNSrv, '/identity/agent/intension_srv')
        self.intension_control_server_ = self.create_service(IntensionExpressControlSrv, '/identity/agent/intension_control_srv', self.intensiton_control_callback)
        
        self.identity_pn = PlainNet('identity_pn')
        self.pn_server_ = self.create_service(PNCommand, '/identity/pn_srv/core', self.pn_server_callback)
        self.pn_updator_ = self.create_publisher(PetriNet, 'identity/pn/update', 10)
        # self.pn_decision_client = self.create_client()
        self.dt = 0.5
        self.pn_timer = self.create_timer(self.dt, self.pn_timer_callback)
        self.pn_create_command_log = []
        self.control_state = {'express': 0, 'ego_env': 1}
        
    def pn_timer_callback(self):
        if self.control_state['ego_env'] == 1:
            changed = self.identity_pn.tick(self.dt)
            if len(changed) > 0:
                self.get_logger().info(f"transition {changed} finished")
                self.pn_generate_response()
        
        if self.control_state['express'] == 1:
            state_msg = GCPNStateMsg()
            state_msg.dim_p = self.identity_pn.get_state_space()[0][0]
            state_msg.dim_t = self.identity_pn.get_state_space()[1][0]
            state_msg.lp = self.identity_pn.get_state_space()[0][1]
            state_msg.lt = self.identity_pn.get_state_space()[1][1]
            p_state = self.identity_pn.get_state()[0]
            t_state = self.identity_pn.get_state()[1]
            state_msg.p_state = [v for row in p_state for v in row]
            state_msg.t_state = [v for row in t_state for v in row]
            
            state_msg.timestamp = self.get_clock().now().to_msg()
            
            # self.intension_generator_publisher_.publish(state_msg)
            request = GCPNSrv.Request()
            request.state = state_msg
            self.intension_generator_clinent_.call_async(request)
            # self.get_logger().info(f"state passed")
    
    def intensiton_control_callback(self, request, response):
        self.control_state['express'] = request.express
        self.control_state['ego_env'] = request.ego_env
        response = IntensionExpressControlSrv.Response()
        response.express = self.control_state['express']
        response.ego_env = self.control_state['ego_env']
        return response
        
    def pn_server_callback(self, request, response):
        command = request.command
        args = request.args
        self.get_logger().info(f'received command: {command}')
        self.get_logger().info(f'received args: {args}')
        
        if command == 'MADP':
            try:
                new_name = args[0]
                if new_name == None or new_name == '':
                    raise TypeError
                if new_name in self.identity_pn.name_node.keys():
                    self.get_logger().error(f'Duplicated place name: {new_name}')
                    raise
                new_mark = ast.literal_eval(args[1])
                new_place = Place(new_name, new_mark)
                self.identity_pn.add_node(new_place)
            except TypeError:
                self.get_logger().error(f'error args type for command {command}')
            except:
                self.get_logger().error(f'error args for command {command}')
        elif command == 'MDLP':
            pass
        elif command == 'MADT':
            try:
                new_name = args[0]
                try:
                    time = float(args[1])
                except:
                    time = 0.0
                if new_name == None or new_name == '':
                    raise TypeError
                if new_name in self.identity_pn.name_node.keys():
                    self.get_logger().error(f'Duplicated transition name: {new_name}')
                    raise
                new_transition = Transition(new_name, time=time)
                self.identity_pn.add_node(new_transition)
            except TypeError:
                self.get_logger().error(f'error args type for command {command}')
            except:
                self.get_logger().error(f'error args for command {command}: {args}')
        elif command == 'MDLT':
            pass
        elif command == 'MADA':
            try:
                direction = args[0]
                node_in_name = args[1]
                node_out_name = args[2]
                if direction == 'PtoT':
                    # node_in = self.identity_pn.places[self.identity_pn.name_node[node_in_name].id] 
                    # node_out = self.identity_pn.transitions[self.identity_pn.name_node[node_out_name].id]
                    node_in = self.identity_pn.name_node[node_in_name]
                    node_out = self.identity_pn.name_node[node_out_name]
                elif direction == 'TtoP':
                    # node_in = self.identity_pn.transitions[self.identity_pn.name_node[node_in_name].id] 
                    # node_out = self.identity_pn.places[self.identity_pn.name_node[node_out_name].id]
                    node_in = self.identity_pn.name_node[node_in_name]
                    node_out = self.identity_pn.name_node[node_out_name]
                else:
                    raise TypeError
                if f'{node_in.name}->{node_out.name}' in self.identity_pn.name_node.keys():
                    self.get_logger().error(f'Duplicated arc from {node_in.name} to {node_out.name}')
                    raise
                self.identity_pn.add_arc(node_in, node_out)
            except TypeError:
                self.get_logger().error(f'Unknown direction for command {command}, use PtoT or TtoP instead')
            except:
                self.get_logger().error(f'Error args for command {command}: {args}')
        elif command == 'MDLA':
            pass
        elif command == 'MDLN':
            self.identity_pn = PlainNet('identity_pn')
            
        elif command == 'MAST':
            target_place = args[0]
            target_marking = ast.literal_eval(args[1])
            self.identity_pn.name_node[target_place].set_target_marking(target_marking)
            
        elif command == 'MFRT':
            try:
                target_trans = args[0]
                if target_trans not in self.identity_pn.name_node.keys():
                    self.get_logger().error(f'Unknown Transition for command {command}')
                else:
                    if self.identity_pn.name_node[target_trans].status == 'ready':
                        self.identity_pn.on_fire_transition(self.identity_pn.name_node[target_trans])
                        self.get_logger().info(f'{target_trans} started, rest time: {self.identity_pn.name_node[target_trans].time}')
                    else:
                        self.get_logger().warn(f'Firing unready Transition {target_trans}')
            except:
                self.get_logger().error(f'Error args for command {command}: {args}')
        elif command == 'SRSN':
            self.identity_pn.reset_net()
        elif command == 'SPMK':
            place_name = args[0]
            new_mark = eval(args[1])
            self.identity_pn.name_node[place_name].set_mark(new_mark)
            self.get_logger().info(f'set place [{place_name}] marking to {self.identity_pn.name_node[place_name].marking}')
        elif command == 'RSTS':
            pass
        elif command == 'RNET':
            pass
        elif command == 'RPRN':
            res = []
            res.append(['\\'])
            for i, key_from in enumerate(self.identity_pn.name_node.keys()):
                if isinstance(self.identity_pn.name_node[key_from], Place) or isinstance(self.identity_pn.name_node[key_from], Transition):
                    res[0].append(key_from)
                    res.append([key_from])
                    for j, key_to in enumerate(self.identity_pn.name_node.keys()):
                        if isinstance(self.identity_pn.name_node[key_to], Place) or isinstance(self.identity_pn.name_node[key_to], Transition):
                            if ((self.identity_pn.name_node[key_from].id, self.identity_pn.name_node[key_to].id) in self.identity_pn.adj.keys()):
                                res[i+1].append(1)
                            else:
                                res[i+1].append(0)
            mat_str = 'identity petri net adjMatrix: \n'
            for i in range(len(res)):
                for j in range(len(res[0])):
                    mat_str += str(res[i][j]) + '\t'
                mat_str += '\n'
            self.get_logger().info(mat_str)
        elif command == 'TEST':
            # self.get_logger().info(f"{self.identity_pn.name_node['t1'].time}")
            pass
        else:
            self.get_logger().error(f"Unknown Command: {command}")
            
        self.identity_pn.update_ready_transition()
        response = self.pn_generate_response()
        
        return response
    
    def pn_generate_response(self):     # include send state to topic /identity/pn/update
        places = []
        transitions = []
        arcs = []
        for p in self.identity_pn.places.values():
            p_msg = PlaceMsg()
            p_msg.id = str(p.id)
            p_msg.name = p.name
            for i in p.ins.values():
                p_msg.ins.append(str(i.id))
            for o in p.outs.values():
                p_msg.outs.append(str(o.id))
            for ia in p.in_arcs.values():
                p_msg.in_arcs.append(str(ia.id))
            for oa in p.out_arcs.values():
                p_msg.out_arcs.append(str(oa.id))
            for k in p.marking.keys():
                p_msg.marking_types.append(k)
                p_msg.marking.append(p.marking[k])
            p_msg.tokens = p.tokens
            places.append(p_msg)
        
        for t in self.identity_pn.transitions.values():
            t_msg = TransitionMsg()
            t_msg.id = str(t.id)
            t_msg.name = t.name
            for i in t.ins.values():
                t_msg.ins.append(str(i.id))
            for o in t.outs.values():
                t_msg.outs.append(str(o.id))
            for ia in t.in_arcs.values():
                t_msg.in_arcs.append(str(ia.id))
            for oa in t.out_arcs.values():
                t_msg.out_arcs.append(str(oa.id))
            t_msg.time = t.time
            t_msg.priority = t.priority
            t_msg.status = t.status
            t_msg.work_status = t.work_status
            transitions.append(t_msg)
            
        for arc in self.identity_pn.arcs.values():
            arc_msg = ArcMsg()
            arc_msg.id = str(arc.id)
            arc_msg.direction = arc.direction
            arc_msg.name = arc.name
            arc_msg.node_in = str(arc.node_in.id)
            arc_msg.node_out = str(arc.node_out.id)
            for k in arc.annotation.keys():
                arc_msg.annotation_key.append(k)
                arc_msg.annotation_value.append(arc.annotation[k])
            arcs.append(arc_msg)
            
        response = PNCommand.Response()
        response.places = places
        response.transitions = transitions
        response.arcs = arcs
        
        update_msg = PetriNet()
        update_msg.places = places
        update_msg.transitions = transitions
        update_msg.arcs = arcs
        self.pn_updator_.publish(update_msg)
        return response
        
            
    
    def sr_state_robot_physical_callback(self, msg):
        self.sr_state_robot.set_pos(msg.th, msg.ex, msg.h1, msg.h2, msg.z)
    
    # TODO: Command Manual
    # RCSP-->Reference to Current State Physical
    # RCSS-->Reference to Current State Simulate
    def sr_state_srv_callback(self, request, response):
        command = request.data
        if command == 'RCSP':
            response.robot.th = self.sr_state_p.robot.th
            response.robot.ex = self.sr_state_p.robot.ex
            response.robot.h1 = self.sr_state_p.robot.h1
            response.robot.h2 = self.sr_state_p.robot.h2
            response.robot.z  = self.sr_state_p.robot.z
            response.pa = None
            return response
        if command == 'RCSS':
            response.pa.th_pa = self.sr_state_p.pa.th_pa
        
        
def main(args=None):
    rclpy.init(args=args)
    
    node = Identity('identity')
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()