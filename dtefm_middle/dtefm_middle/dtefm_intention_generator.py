import rclpy
import sys
import os
import ast
import numpy as np
from rclpy.node import Node
from dtefm_interfaces.msg import SRStateRobot, PlaceMsg, TransitionMsg, ArcMsg, PetriNet
from dtefm_interfaces.srv import SRTcpCommunication, SRState, PNCommand
from ament_index_python.packages import get_package_share_directory

class PNAgent():
    def __init__(self):
        pass
    
    def take_action(self, state):
        state_p = state[0]
        state_t = state[1]
        
        choix = np.where((state_t[:, 0] == 1) & (state_t[:, 1] == 0))[0]

        if choix.size > 1:
            return choix[0].item()+1
        else:
            return 0
    
    def update(self):
        pass

class IntensionCore(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info(f"Intension Core node {name} initialized..")
        self.decision_agent = PNAgent()
        self.