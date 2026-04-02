import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


class JointStateBridge(Node):
    def __init__(self):
        super().__init__('joint_state_bridge')

        self.subscription = self.create_subscription(
            Float64MultiArray,
            'joint_ref',
            self.callback,
            10
        )

        self.publisher = self.create_publisher(
            JointState,
            'joint_states',
            10
        )

        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

    def callback(self, msg):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        js.position = msg.data

        self.publisher.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
