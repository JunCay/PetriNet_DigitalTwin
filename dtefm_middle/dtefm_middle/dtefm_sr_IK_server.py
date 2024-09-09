import os
import numpy as np
import rclpy
import math
import time
import copy
import transforms3d as tfs
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from tf2_ros import TransformListener
from geometry_msgs.msg import Pose, Quaternion
from dtefm_interfaces.msg import SRStateRobot
from dtefm_interfaces.srv import SRRobotIKSrv, SRRobotInfoSrv
from dtefm_interfaces.action import SRRobotIKMove

def delta_angle(theta):
    theta = theta % 360  
    if theta > 180:
        theta -= 360
    return theta

class SRRobot():
    def __init__(self, name):
        self.name = name
        self.link_dict = dict()
        self.current_pose = Pose()
        self.current_state = SRStateRobot()
        self.target_pose = Pose()
        self.target_claw = 1
        self.target_state = SRStateRobot()
        self.move_status = SRRobotIKMove.Feedback.STATUS_STOP
        self.speed_level = 1
        self.speeds = {'th': 45, 'ex': 0.5, 'h1': 45, 'h2': 45, 'z': 0.5, 'lin': 0.1, 'ang': 10}
        self.precisions = {'th': 0.11, 'ex': 0.001, 'h1': 0.11, 'h2': 0.11, 'z': 0.001, 'lin': 0.001, 'ang':0.05}
        self.dt = 0.0011
    
    def get_status(self):
        return self.move_status    
    
    def get_states(self):
        return self.current_state
    
    def set_dt(self, dt):
        self.dt = dt
        
    def set_speed_level(self, speed_level):
        self.speed_level = speed_level
        
    def update_link_dict(self, new_link_dict:dict):
        for key in new_link_dict.keys():
            self.link_dict[key] = new_link_dict[key]
    
    def set_current_state(self, current_state:SRStateRobot):
        self.current_state.th = current_state.th
        self.current_state.ex = current_state.ex
        self.current_state.h1 = current_state.h1
        self.current_state.h2 = current_state.h2
        self.current_state.z  = current_state.z
        self.cal_sr_fk()
    
    def set_goal(self, goal_pose:Pose, target_claw):
        self.target_pose = copy.deepcopy(goal_pose)
        self.target_claw = target_claw
        
    def set_traj_point(self, traj_pose:Pose, target_claw):
        self.target_pose_ = copy.deepcopy(traj_pose)
        self.target_claw_ = target_claw
        
    def cal_sr_ik(self, override='target'):
        if override == 'target':
            x_t = np.float32(self.target_pose.position.x)
            y_t = np.float32(self.target_pose.position.y)
            z_t = np.float32(self.target_pose.position.z)
            quat = [1.0, 0.0, 0.0, 0.0]
            quat[0] = self.target_pose.orientation.w
            quat[1] = self.target_pose.orientation.x
            quat[2] = self.target_pose.orientation.y
            quat[3] = self.target_pose.orientation.z
        elif override == 'target_':
            x_t = np.float32(self.target_pose_.position.x)
            y_t = np.float32(self.target_pose_.position.y)
            z_t = np.float32(self.target_pose_.position.z)
            quat = [1.0, 0.0, 0.0, 0.0]
            quat[0] = self.target_pose_.orientation.w
            quat[1] = self.target_pose_.orientation.x
            quat[2] = self.target_pose_.orientation.y
            quat[3] = self.target_pose_.orientation.z
        elif override == 'current':
            x_t = np.float32(self.current_pose.position.x)
            y_t = np.float32(self.current_pose.position.y)
            z_t = np.float32(self.current_pose.position.z)
            quat = [1.0, 0.0, 0.0, 0.0]
            quat[0] = self.current_pose.orientation.w
            quat[1] = self.current_pose.orientation.x
            quat[2] = self.current_pose.orientation.y
            quat[3] = self.current_pose.orientation.z
        
        th_ = np.float32(self.current_state.th)
        ex_ = np.float32(self.current_state.ex)
        h1_ = np.float32(self.current_state.h1)
        h2_ = np.float32(self.current_state.h2)
        z_  = np.float32(self.current_state.z)
        
        # rel = quat[2] - 1.0
        # print(f"quat: , {quat}, quat[2] - 1.0: {rel:.8f}")
        
        if abs(abs(quat[2]) - 1.0) < 0.0001:
            theta_ = np.float32(180)
            # print(f"thetas: 0.0, {theta_:.2f}, 0.0")
        else:
            thetas = tfs.euler.quat2euler(quat, "sxyz")      # in radias
            theta_ = np.mod(np.rad2deg(thetas[1]), np.float32(360))
            # print(f"thetas: {thetas[0]}, {thetas[1]}, {thetas[2]}")
        theta = np.float32(delta_angle(-theta_ + 90.0))
        # print(f"theta: {theta}, original: {theta_}")
        xh = x_t - self.link_dict['l4'] * np.cos(np.radians(theta))
        zh = z_t - self.link_dict['l4'] * np.sin(np.radians(theta))
        
        if xh >= 0:
            sgn = np.float32(-1)
            th = -np.float32(np.degrees(np.arctan2(zh, xh)))
        else:
            sgn = np.float32(1)
            th = -np.mod((np.float32(np.degrees(np.arctan2(zh, xh))) + np.float32(360.0)), np.float32(360.0)) + np.float32(180.0)
        l4 = self.link_dict['l4']
        print(f"x_t: {x_t}, z_t: {z_t}, theta: {theta}, l4: {l4}")
        test = x_t*x_t + z_t*z_t + self.link_dict['l4']*self.link_dict['l4'] - 2*self.link_dict['l4']*(x_t*np.cos(np.radians(theta)) + z_t*np.sin(np.radians(theta)) )
        print(test)
        ex = sgn * np.sqrt(np.abs(x_t*x_t + z_t*z_t + self.link_dict['l4']*self.link_dict['l4'] - 2*self.link_dict['l4']*(x_t*np.cos(np.radians(theta)) + z_t*np.sin(np.radians(theta)) )))
        
        alpha = np.float32(np.degrees(np.arccos(np.abs(ex)/(2*self.link_dict['l2']))))
        
        d2_u = y_t - self.link_dict['d1_u']
        d2_l = y_t - self.link_dict['d1_l']
        
        theta2 = th + sgn * (alpha - np.float32(90))
        if ex < 0:
            theta3 = 2 * alpha
        else:
            theta3 = np.float32(360) - 2 * alpha
        
        if self.target_claw == 2:
            if ex < 0:
                theta5 = np.float32(360) - th - alpha - theta
            else:
                theta5 = np.float32(180) - th + alpha - theta 
            theta4 = h1_
            z = d2_u

        else:
            if ex < 0:
                theta4 = np.float32(360) - th - alpha - theta
            else:
                theta4 = np.float32(180) - th + alpha - theta 
            theta5 = h2_
            z = d2_l
        
        if ex <= 0:
            if self.target_claw == 1:
                h1 = np.float32(360) - theta4 - alpha
                h2 = h2_
            else:
                h2 = np.float32(360) - theta5 - alpha
                h1 = h1_
        else:
            if self.target_claw == 1:
                h1 = np.float32(180) - theta4 + alpha
                h2 = h2_
            else:
                h2 = np.float32(180) - theta5 + alpha
                h1 = h1_
                
        if override == 'target':
            self.target_state = SRStateRobot()
            self.target_state.th = float(th)
            self.target_state.ex = float(ex)
            self.target_state.h1 = float(h1)
            self.target_state.h2 = float(h2)
            self.target_state.z  = float(z)
        
        elif override == 'target_':
            self.target_state_ = SRStateRobot()
            self.target_state_.th = float(th)
            self.target_state_.ex = float(ex)
            self.target_state_.h1 = float(h1)
            self.target_state_.h2 = float(h2)
            self.target_state_.z  = float(z)
        
        elif override == 'current':
            self.current_state = SRStateRobot()
            self.current_state.th = float(th)
            self.current_state.ex = float(ex)
            self.current_state.h1 = float(h1)
            self.current_state.h2 = float(h2)
            self.current_state.z  = float(z)
        
    def cal_sr_fk(self):        # current_state to current_pose
        if self.target_claw == 1:
            theta = -self.current_state.h1 + self.current_state.th
            self.current_pose.position.y = self.link_dict['d1_u'] + self.current_state.z
            
        else:
            theta = self.current_state.h2 - self.current_state.th
            self.current_pose.position.y = self.link_dict['d1_u'] + self.current_state.z
            
        x = np.abs(self.current_state.ex) * np.cos(self.current_state.th) + np.float32(self.link_dict['l4']) * np.cos(theta)
        z = -np.abs(self.current_state.ex) * np.sin(self.current_state.th) - np.float32(self.link_dict['l4']) * np.sin(theta)
            
        quat = tfs.euler.euler2quat(0.0, theta, 0.0)
        self.current_pose.orientation.w = quat[0]
        self.current_pose.orientation.x = quat[1]
        self.current_pose.orientation.y = quat[2]
        self.current_pose.orientation.z = quat[3]
        
        self.current_pose.position.x = x
        self.current_pose.position.z = z
        
        
    def start_move_ik(self):
        self.move_status = SRRobotIKMove.Feedback.STATUS_MOVING
        self.directions = dict()
        self.directions['th'] = math.copysign(1, self.target_state.th-self.current_state.th)
        self.directions['ex'] = math.copysign(1, self.target_state.ex-self.current_state.ex)
        
        self.directions['h1'] = self.find_rotate_direction(self.target_state.h1, self.current_state.h1)
        self.directions['h2'] = self.find_rotate_direction(self.target_state.h2, self.current_state.h2)
        
        self.directions['z']  = math.copysign(1, self.target_state.z -self.current_state.z)
        
    def find_rotate_direction(self, target_angle, current_angle) -> float:
        target_angle = delta_angle(target_angle)
        current_angle = delta_angle(current_angle)
        if target_angle - current_angle > 0:
            if target_angle - current_angle > 180:
                direction = -1
            else:
                direction = 1
        else:
           if target_angle - current_angle < -180:
                direction = 1
           else: 
                direction = -1
        print(f"target: {target_angle}, current: {current_angle}, direction: {direction}")
        return direction
    
    def angular_rotate(self, angle, delta_angle):
        angle += delta_angle
        if angle > 180:
            angle -= 360
        elif angle < -180:
            angle += 360
        return angle
    
    def move_ik_step(self, move_type='syn'):
        if self.move_status == SRRobotIKMove.Feedback.STATUS_MOVING:
            if move_type == 'syn':
                if not self.close_goal(self.target_state.th, self.current_state.th, self.precisions['th']):    
                    self.current_state.th += self.directions['th'] * self.speeds['th'] * self.dt
                if not self.close_goal(self.target_state.ex, self.current_state.ex, self.precisions['ex']):
                    self.current_state.ex += self.directions['ex'] * self.speeds['ex'] * self.dt
                if not self.close_goal(self.target_state.h1, self.current_state.h1, self.precisions['h1']):
                    self.current_state.h1 = self.angular_rotate(self.current_state.h1, self.directions['h1'] * self.speeds['h1'] * self.dt)
                if not self.close_goal(self.target_state.h2, self.current_state.h2, self.precisions['h2']):
                    self.current_state.h2 = self.angular_rotate(self.current_state.h2, self.directions['h2'] * self.speeds['h2'] * self.dt)
                if not self.close_goal(self.target_state.z, self.current_state.z, self.precisions['z']):
                    self.current_state.z  += self.directions['z']  * self.speeds['z']  * self.dt
                    
            elif move_type == 'seq':
                if not self.close_goal(self.target_state.z, self.current_state.z, self.precisions['z']):
                    self.current_state.z += self.directions['z'] * self.speeds['z'] * self.dt
                else:
                    if not self.close_goal(self.target_state.th, self.current_state.th, self.precisions['th']):
                        self.current_state.th += self.directions['th'] * self.speeds['th'] * self.dt
                    else:
                        if not self.close_goal(self.target_state.ex, self.current_state.ex, self.precisions['ex']):
                            self.current_state.ex += self.directions['ex'] * self.speeds['ex'] * self.dt
                        else:
                            if not self.close_goal(self.target_state.h1, self.current_state.h1, self.precisions['h1']):
                                self.current_state.h1 += self.directions['h1'] * self.speeds['h1'] * self.dt
                                self.current_state.h2 += self.directions['h2'] * self.speeds['h2'] * self.dt
            elif move_type == 'lin':
                if not self.close_goals(type='current'):
                    
                    pass
            else:
                pass
            self.cal_sr_fk()
            return self.current_state
        else:
            pass
        
    def close_goal(self, a, b, how_close=0.01):
        if math.fabs(a-b) < how_close:
            return True
        else:
            return False
        
    def close_goals(self, type='joints'):
        if type == 'joints':
            if self.close_goal(self.target_state.th, self.current_state.th, self.precisions['th']) and self.close_goal(self.target_state.ex, self.current_state.ex, self.precisions['ex']) and self.close_goal(self.target_state.h1, self.current_state.h1, self.precisions['h1']) and self.close_goal(self.target_state.h2, self.current_state.h2, self.precisions['h2']) and self.close_goal(self.target_state.z, self.current_state.z, self.precisions['z']):
                self.move_status = SRRobotIKMove.Feedback.STATUS_STOP
                return True
            else:
                return False
        elif type == 'target':
            q1 = np.array([self.target_pose.orientation.w, self.target_pose.orientation.x,self.target_pose.orientation.y,self.target_pose.orientation.z])
            q2 = np.array([self.current_pose.orientation.w, self.current_pose.orientation.x, self.current_pose.orientation.y, self.current_pose.orientation.z])
            q_close = np.abs(np.dot(q1, q2))
            p1 = np.array([self.target_pose.position.x, self.target_pose.position.y, self.target_pose.position.z])
            p2 = np.array([self.current_pose.position.x, self.current_pose.position.y, self.current_pose.position.z])
            p_close = np.array(np.dot(p1, p2))
            if self.close_goal(p_close, 1, self.precisions['lin']) and self.close_goal(q_close, 1, self.precisions['ang']):
                self.move_status = SRRobotIKMove.Feedback.STATUS_STOP
                return True
            else:
                return False
        
    def print_state(self):
        res = f"current state: th: {self.current_state.th:.2f}\t| ex: {self.current_state.ex:.2f}\t| h1: {self.current_state.h1:.2f}\t| h2: {self.current_state.h2:.2f}\t| z: {self.current_state.z:.2f}"
        return res
    
    def print_target_state(self):
        res = f"target state: th: {self.target_state.th:.2f}\t| ex: {self.target_state.ex:.2f}\t| h1: {self.target_state.h1:.2f}\t| h2: {self.target_state.h2:.2f}\t| z: {self.target_state.z:.2f}"
        return res
    
    def print_target_pose(self):
        res = f"target pose: pos: [{self.target_pose.position.x:.2f}, {self.target_pose.position.y:.2f}, {self.target_pose.position.z:.2f}], ori: [{self.target_pose.orientation.w:.2f}, {self.target_pose.orientation.x:.2f}, {self.target_pose.orientation.y:.2f}, {self.target_pose.orientation.z:.2f}]"
        return res
    
class RobotIKServer(Node):
    def __init__(self, name):
        super().__init__(name)
        self.name = name
        self.get_logger().info(f'node {name} created.')
        self.sr_robot = SRRobot(f'{name}_robot')
        self.dt = 0.0011
        self.topic_freq = 0.01
        self.sr_robot.set_dt(self.dt)
        
        self.link_dict = dict()
        self.ik_server_ = self.create_service(SRRobotIKSrv, '/sr/robot/ik_srv', self.sr_ik_callback_)
        self.ik_sync_publisher_ = self.create_publisher(SRStateRobot, '/sr/robot/state/simulate', 10)
        self.sr_robot_info_client_ = self.create_client(SRRobotInfoSrv, '/sr/robot/info_srv')
        self.ik_action_server_ = ActionServer(self, SRRobotIKMove, '/sr/robot/ik_move', self.sr_ik_move_callback)
        
        time.sleep(0.5)
        self.robot_initialize(sync=True)
        
        self.current_joint_state = None
        self.target_joint_state = None          # SRStateRobot()
        
    def robot_initialize(self, sync=False):
        if not sync:
            self.link_dict['d1_l'] = np.float32(0.6348001)
            self.link_dict['d1_u'] = np.float32(0.6434001)
            self.link_dict['l2'] = np.float32(0.44)
            self.link_dict['l3'] = np.float32(0.44)
            self.link_dict['l4'] = np.float32(0.3400001)
            self.link_dict['l5'] = np.float32(0.3400001)
            
            self.sr_robot.link_dict['d1_l'] = np.float32(0.6348001)
            self.sr_robot.link_dict['d1_u'] = np.float32(0.6434001)
            self.sr_robot.link_dict['l2'] = np.float32(0.44)
            self.sr_robot.link_dict['l3'] = np.float32(0.44)
            self.sr_robot.link_dict['l4'] = np.float32(0.3400001)
            self.sr_robot.link_dict['l5'] = np.float32(0.3400001)
            
        else:
            request = SRRobotInfoSrv.Request()
            request.request = 'robot_info_sync_request'
            self.sr_robot_info_client_.call_async(request).add_done_callback(self.robot_initialize_)
    
    def robot_initialize_(self, result):
        response = result.result()
        self.link_dict['d1_l'] = np.float32(response.sr_robot_info.d1_l)
        self.link_dict['d1_u'] = np.float32(response.sr_robot_info.d1_u)
        self.link_dict['l2'] = np.float32(response.sr_robot_info.l2)
        self.link_dict['l3'] = np.float32(response.sr_robot_info.l3)
        self.link_dict['l4'] = np.float32(response.sr_robot_info.l4)
        self.link_dict['l5'] = np.float32(response.sr_robot_info.l5)
        
        self.sr_robot.link_dict['d1_l'] = np.float32(response.sr_robot_info.d1_l)
        self.sr_robot.link_dict['d1_u'] = np.float32(response.sr_robot_info.d1_u)
        self.sr_robot.link_dict['l2'] = np.float32(response.sr_robot_info.l2)
        self.sr_robot.link_dict['l3'] = np.float32(response.sr_robot_info.l3)
        self.sr_robot.link_dict['l4'] = np.float32(response.sr_robot_info.l4)
        self.sr_robot.link_dict['l5'] = np.float32(response.sr_robot_info.l5)
        
        self.get_logger().info("robot initialized")
        print(self.sr_robot.link_dict)
    
    def sr_ik_callback_(self, request, response):
        self.sr_robot.set_goal(request.target_pose, request.target_claw)
        self.sr_robot.set_current_state(request.current_state)
        self.sr_robot.cal_sr_ik()
        self.get_logger().info(self.sr_robot.print_target_pose())
        self.get_logger().info(self.sr_robot.print_target_state())
        self.sr_robot.start_move_ik()
        self.last_time = self.get_clock().now()
        
        while rclpy.ok() and not self.sr_robot.close_goals():
            self.sr_robot.move_ik_step()
            current_time = self.get_clock().now()
            duration = current_time - self.last_time
            
            if duration.nanoseconds/1e9 > self.topic_freq:
                sync_msg = SRStateRobot()
                sync_msg = self.sr_robot.get_states()
                self.ik_sync_publisher_.publish(sync_msg)
                self.get_logger().info(self.sr_robot.print_state())
                self.last_time = current_time
                
            time.sleep(self.dt)
        self.sr_robot.set_current_state(self.sr_robot.target_state)
        
        response.target_state = self.sr_robot.get_states()
        self.get_logger().info(self.sr_robot.print_state())
        self.get_logger().info(f"IK finished")
        
        return response
        
        
    
    # Aborted
    def sr_ik_callback(self, request, response):
        self.robot_initialize(sync=True)
        targetPose = request.target_pose
        x_t = np.float32(targetPose.position.x)
        y_t = np.float32(targetPose.position.y)
        z_t = np.float32(targetPose.position.z)
        quat = [1.0, 0.0, 0.0, 0.0]
        quat[0] = targetPose.orientation.w
        quat[1] = targetPose.orientation.x
        quat[2] = targetPose.orientation.y
        quat[3] = targetPose.orientation.z
        
        th_ = np.float32(request.current_state.th)
        ex_ = np.float32(request.current_state.ex)
        h1_ = np.float32(request.current_state.h1)
        h2_ = np.float32(request.current_state.h2)
        z_  = np.float32(request.current_state.z)
        
        self.current_joint_state = SRStateRobot()
        self.current_joint_state.th = float(th)
        self.current_joint_state.ex = float(ex)
        self.current_joint_state.h1 = float(h1)
        self.current_joint_state.h2 = float(h2)
        self.current_joint_state.z = float(z)
        
        thetas = tfs.euler.quat2euler(quat, "sxyz")      # in radias
        theta = np.float32(-np.rad2deg(thetas[1])) + np.float32(90.0)
        # theta = [e/3.14*180 for e in theta]
        # self.get_logger().info(f"theta: {theta[0]}, {theta[1]}, {theta[2]}")
        
        xh = x_t - self.link_dict['l4'] * np.cos(np.radians(theta))
        zh = z_t - self.link_dict['l4'] * np.sin(np.radians(theta))
        
        if xh >= 0:
            sgn = np.float32(-1)
            th = -np.float32(np.degrees(np.arctan2(zh, xh)))
        else:
            sgn = np.float32(1)
            th = -np.mod((np.float32(np.degrees(np.arctan2(zh, xh))) + np.float32(360.0)), np.float32(360.0)) + np.float32(180.0)
        
        ex = sgn * np.sqrt(x_t*x_t + z_t*z_t + self.link_dict['l4']*self.link_dict['l4'] - 2*self.link_dict['l4']*(x_t*np.cos(np.radians(theta)) + z_t*np.sin(np.radians(theta)) ))
        
        alpha = np.float32(np.degrees(np.arccos(np.abs(ex)/(2*self.link_dict['l2']))))
        
        d2_u = y_t - self.link_dict['d1_u']
        d2_l = y_t - self.link_dict['d1_l']
        
        theta2 = th + sgn * (alpha - np.float32(90))
        theta3 = sgn * (np.float32(180) - 2 * alpha)
        
        if request.target_claw == 1:
            theta5 = np.float32(90) - theta - theta3 - theta2
            theta4 = h1_
            z = d2_u

        else:
            theta4 = np.float32(90) - theta - theta3 - theta2
            theta5 = h2_
            z = d2_l

        h1 = theta4
        h2 = theta5
        # self.get_logger().info(f"thetas: {thetas[0]}, {thetas[1]}, {thetas[2]}")
        # self.get_logger().info(f"x_t: {x_t}, z_t: {z_t}, sgn: {sgn}, theta: {theta}")
        # self.get_logger().info(f"xh: {xh}, zh: {zh}, alpha: {alpha}")
        # self.get_logger().info(f"theta2: {theta2}, theta3: {theta3}, theta4: {theta4}, theta5: {theta5}")
        # self.get_logger().info(f"th: {th}, ex: {ex}, h1: {h1}, h2: {h2}, z: {z}")
        response.target_state = SRStateRobot()
        response.target_state.th = float(th)
        response.target_state.ex = float(ex)
        response.target_state.h1 = float(h1)
        response.target_state.h2 = float(h2)
        response.target_state.z = float(z)
        
        self.target_joint_state = SRStateRobot()
        self.target_joint_state.th = float(th)
        self.target_joint_state.ex = float(ex)
        self.target_joint_state.h1 = float(h1)
        self.target_joint_state.h2 = float(h2)
        self.target_joint_state.z = float(z)
        return response
    
    # Aborted
    def sr_ik_move_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info(f"sr robot ik moving")
        feedback_msg = SRRobotIKMove.Feedback()
        self.sr_robot.set_current_state(goal_handle.request.current_state)
        self.sr_robot.set_goal(goal_handle.request.target_pose, goal_handle.request.target_claw)

        while rclpy.ok() and not self.sr_robot.close_goals():
            self.sr_robot.move_ik_step()
            feedback_msg.feedback_state = self.sr_robot.current_pose
            feedback_msg.status = self.sr_robot.get_status()
            goal_handle.publish_feedback(feedback_msg)
            if goal_handle.is_cancel_requested:
                result = SRRobotIKMove.Result()
                result.final_state = self.sr_robot.current_pose
                return result
            time.sleep(self.dt)
        
        goal_handle.succeed()
        result = SRRobotIKMove.Result()
        result.final_state = self.sr_robot.current_pose
        return result
    # def trajectory_planing(self):
        

def main(args=None):
    rclpy.init(args=args)
    node = RobotIKServer('sr_robot_ik_server')
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()