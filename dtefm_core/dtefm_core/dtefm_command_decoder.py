import rclpy
import re
from rclpy.node import Node
from std_msgs.msg import String
from dtefm_interfaces.msg import EAPCommand

# dtefm_command_decode:
#   listen to eap command in String type from the topic of '/command/eap/encoded_string'
#   publish the command decoded into EAPCommand type to topic '/command/eap/decoded'


class CommandDecoder(Node):
    def __init__(self, name='command_gate_eap'):
        super().__init__(name)
        self.get_logger().info(f'node {name} created.')
        self.in_command_subscriber_ = self.create_subscription(String, '/command/eap/encoded_string', self.listener_callback_in_command, 10)
        self.out_command_publisher_ = self.create_publisher(EAPCommand, '/command/eap/decoded', 10)
        self.in_command_subscriber_
    
    def listener_callback_in_command(self, msg:String):
        snfn, seq, W, item = self.string_command_parser(msg.data)
        msg_ = EAPCommand()
        msg_.snfn = snfn
        msg_.seq = seq
        msg_.w = W
        msg_.item = item
        self.out_command_publisher_.publish(msg_)
    
    def string_command_parser(self, str):
        snfn_pattern = r'S\d+F\d+'
        seq_pattern = r'\[\d{8}\]'
        W_pattern = r'\s+W\s+'
        
        
        snfn_ = re.search(snfn_pattern, str)
        seq_s_= re.search(seq_pattern, str)
        snfn = snfn_.group() if snfn_ else ''
        seq_s = seq_s_.group() if seq_s_ else ''
        
        if seq_s != '':
            seq = int(seq_s[1:-1])
        else:
            seq = -1
        W = 'E->H' if re.search(W_pattern, str) else 'H->E'
        try:
            item = self.extract_bracket(str)
        except:
            item = ''
        
        return snfn, seq, W, item
        
    def extract_bracket(self, input_string):
        bracket_n = 0
        res = []
        for ch in input_string:
            if ch == '<':
                bracket_n += 1
            elif ch == '>':
                bracket_n -= 1

            if bracket_n > 0:
                res.append(ch)

        res.append('>')
        return ''.join(res)
        

def main(args=None):
    rclpy.init(args=args)
    
    node = CommandDecoder('EAP_Command_Decoder')
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()