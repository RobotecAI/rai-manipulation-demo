import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from transformers import AutoModelForVision2Seq, AutoProcessor
from PIL import Image as PILImage
import torch

from std_msgs.msg import Float32MultiArray

class VLARobotNode(Node):
    def __init__(self):
        super().__init__('vla_robot_node')
        
        self.bridge = CvBridge()
        
        self.image_subscription = self.create_subscription(
            Image,
            '/vla_image',
            self.camera_callback,
            10)

        self.action_publisher = self.create_publisher(
            Float32MultiArray,
            '/vla_action',
            10
        )

        VLA_PATH = "openvla/openvla-7b"
        
        self.processor = AutoProcessor.from_pretrained(VLA_PATH, trust_remote_code=True)
        self.vla = AutoModelForVision2Seq.from_pretrained(
            VLA_PATH,
            attn_implementation="flash_attention_2",  # [Optional] Requires `flash_attn`
            torch_dtype=torch.bfloat16, 
            low_cpu_mem_usage=True, 
            trust_remote_code=True,
        ).to("cuda:0")

        self.get_logger().info('VLA Robot Node initialized')

    def camera_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        pil_image = PILImage.fromarray(cv_image)
        
        action = self.process_image(pil_image)
        
        self.execute_action(action)

    def process_image(self, image: PILImage.Image):
        prompt = "In: What action should the robot take to pickup the red cube?\nOut:"
        
        inputs = self.processor(prompt, image).to("cuda:0", dtype=torch.bfloat16)
        action = self.vla.predict_action(**inputs, unnorm_key="bridge_orig", do_sample=False)
        
        return action

    def execute_action(self, action):
        self.get_logger().info(f'Executing action:')
        for i in action:
            self.get_logger().info(f'{i}')
        msg = Float32MultiArray()
        msg.data = list(action)
        self.action_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    vla_robot_node = VLARobotNode()
    rclpy.spin(vla_robot_node)
    vla_robot_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()