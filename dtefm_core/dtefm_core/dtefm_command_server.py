import rclpy
import re
import os
from rclpy.node import Node
from std_msgs.msg import String
from dtefm_interfaces.msg import EAPCommand
from dtefm_interfaces.srv import EAPCommandAnalysis, EAPCommandState
from ament_index_python.packages import get_package_share_directory

class CommandNode():
    def __init__(self, length, head, content, parent=None):
        self.length = length
        self.head = head
        self.content = content
        self.parent = parent
        self.children = []
        
    def add_child(self, child):
        self.children.append(child)
    
    def children_num(self):
        return len(self.children)
    
    def print_info(self):
        parent_str = self.parent.content if self.parent is not None else "None"
        print(f'head: {self.head}\t| parent: {parent_str}\t| child_num: {self.children_num()}\t|content: {self.content}')
    
    def print_tree(self):
        self._print_tree(self)
        
    def _print_tree(self, node):
        if node is None:
            return
        # node.print_info()
        for child in node.children:
            self._print_tree(child)
       
class CommandTree():
    def __init__(self):
        self.root = None
        
    def add_node(self, node: CommandNode):
        if self.root is None:
            self.root = node
        else:
            node.parent.add_child(node)
        return node
            
    def print_tree(self):
        self._print_tree(self.root)
        
    def _print_tree(self, node):
        if node is None:
            return
        # print(node.head, ': ', node.content)
        node.print_info()
        for child in node.children:
            self._print_tree(child)
            
    def look_up(self, content):
        return self._look_up(self.root, content)
    
    def _look_up(self, node, content):
        if node is None:
            return None
        
        if node.content == content:    
            return node.parent
        else:
            for child in node.children:
                res = self._look_up(child, content)
                if res is not None:
                    return res
            return None

class CommandServer(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info(f'node {name} created.')
        self.command_analysis_server_ = self.create_service(EAPCommandAnalysis, '/command/eap/analysis_srv', self.command_analysis_callback)
        self.command_state_server_ = self.create_service(EAPCommandState, '/command/eap/state_srv', self.command_state_callback)
        # self.load_configs()
        self.recipe_name = None
        self.slot_pick_seq = None
        self.slot_place_seq = None
        self.src_port = None
        self.dst_port = None
        self.car_id = None
    
    def load_configs(self):
        sorter_gem_model_path = os.path.join(get_package_share_directory('dtefm_interfaces'), 'src', 'SorterGemModel.xml')
        if os.path.exists(sorter_gem_model_path):
            with open(sorter_gem_model_path, 'r') as f:
                content = f.read()
        else:
            self.get_logger().error(f'File {sorter_gem_model_path} does not exist')
        
    def parse_cmd(cmd):
        cmd = re.sub(r'\s+', '', cmd)
        head_pattern = r'<([LAUBJIF]\d?)\[(\d+)\]'
        L_pattern = r'L'
        A_pattern = r'[A]'
        U_pattern = r'[IUF]\d?'        
        command_tree = None
        current_parent = None
        pos=0
        layer = 0
        while pos < len(cmd):
            if cmd[pos] == '>':
                if layer > 0:
                    layer -= 1
                    current_parent = current_parent.parent
                else:
                    break
                pos += 1
                continue
            head = re.search(head_pattern, cmd[pos:]).group()
            head_type = re.search(head_pattern, cmd[pos:]).group(1)
            head_num = re.search(head_pattern, cmd[pos:]).group(2)
            
            if re.search(L_pattern, head_type):
                if command_tree is None:
                    command_tree = CommandTree()
                    cn = CommandNode(int(head_num), cmd[pos+1:pos+len(head)], head[1:])
                    command_tree.add_node(cn)
                    current_parent = cn
                else:
                    cn = CommandNode(int(head_num), cmd[pos+1:pos+len(head)], head[1:], parent=current_parent)
                    current_parent.add_child(cn)
                    current_parent = cn
                layer += 1
                pos += len(head)
            elif re.search(A_pattern, head_type):
                cn = CommandNode(int(head_num), cmd[pos+1:pos+len(head)], cmd[pos+len(head):pos+len(head)+int(head_num)+2], parent=current_parent)
                current_parent.add_child(cn)
                pos += len(head)+int(head_num)+2+1
            elif re.search(U_pattern, head_type):
                cn = CommandNode(int(head_num), cmd[pos+1:pos+len(head)], cmd[pos+len(head):pos+len(head)+int(head_num)], parent=current_parent)
                current_parent.add_child(cn)
                pos += len(head)+int(head_num)+1
                
        return command_tree
    
    def command_analysis_callback(self, request, response):
        snfn = request.snfn
        cmd = request.item
        rsp = self.command_analysis(snfn, cmd)
        response.ack = rsp
        
        return response
        
    def command_state_callback(self):
        pass
    
    def command_analysis(self, snfn, cmd):
        response = 'invalid request to dtefm_command_analysis server'
        if snfn == 'S2F49' or snfn == 'S2F41':
            response = 'started'
        
        return response
            
        
def main(args=None):
    rclpy.init(args=args)
    
    node = CommandServer('EAP_Command_Server')
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()