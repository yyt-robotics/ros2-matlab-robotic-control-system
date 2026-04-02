import threading
from flask import Flask, jsonify

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


# Shared buffer storing the latest joint reference received from ROS2
latest_joint_ref = [0.0] * 6
latest_lock = threading.Lock()


class JointRefBridge(Node):
    def __init__(self):
        super().__init__('joint_ref_bridge')

        self.subscription = self.create_subscription(
            Float64MultiArray,
            'joint_ref',
            self.joint_ref_callback,
            10
        )

    def joint_ref_callback(self, msg):
        global latest_joint_ref
        with latest_lock:
            latest_joint_ref = list(msg.data)


rclpy.init()
ros_node = JointRefBridge()


def ros_spin():
    rclpy.spin(ros_node)


ros_thread = threading.Thread(target=ros_spin, daemon=True)
ros_thread.start()

app = Flask(__name__)


@app.route('/', methods=['GET'])
def index():
    return jsonify({
        'service': 'joint_ref_bridge',
        'endpoint': '/joint_ref',
        'status': 'running'
    })


@app.route('/joint_ref', methods=['GET'])
def get_joint_ref():
    with latest_lock:
        return jsonify({
            'joints': latest_joint_ref
        })


def main():
    app.run(host='0.0.0.0', port=5002, debug=False)


if __name__ == '__main__':
    main()
