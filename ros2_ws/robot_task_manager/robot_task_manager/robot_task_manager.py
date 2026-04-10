import time
import csv
import os
import numpy as np

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from robot_interfaces.action import MoveToPose
from std_msgs.msg import String

from .ik_solver_opt import Gluon6L3IKLS


class RobotTaskManager(Node):
    def __init__(self):
        super().__init__('robot_task_manager')

        self.robot_status = 'idle'

        # Current joint state used as the start point for the next trajectory
        self.current_joints_state = [0.0] * 6
        # Joint velocity limits (rad/s)
        self.joint_vel_limits = [0.20, 0.18, 0.18, 0.25, 0.25, 0.35]

        # Trajectory sampling period (s)
        self.dt = 0.05
        
        # Python IK solver
        self.ik_solver = Gluon6L3IKLS()

        # Status publisher
        self.status_publisher = self.create_publisher(
            String,
            'robot_status',
            10
        )
        self.status_timer = self.create_timer(1.0, self.publish_status)
        self.joint_ref_publisher = self.create_publisher(
            Float64MultiArray,
            'joint_ref',
            10
        )
        
        # Action Server
        self._callback_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            MoveToPose,
            'execute_task',
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._callback_group
        )
        
        # CSV path for offline trajectory export
        # Can be overridden by environment variable TRAJECTORY_CSV_PATH
        # Project root inside Docker container
        project_root = "/root/ros2_study/workspace"
        
        # CSV path for offline trajectory export
        self.trajectory_file = os.environ.get(
            "TRAJECTORY_FILE",
            os.path.join(project_root, "matlab", "trajectory", "trajectory_log_6dof.csv")
        )
        
        os.makedirs(os.path.dirname(self.trajectory_file), exist_ok=True)
        self.get_logger().info(f'Offline trajectory CSV path: {self.trajectory_file}')

    def publish_status(self):
        msg = String()
        msg.data = self.robot_status
        self.status_publisher.publish(msg)

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def publish_joint_ref(self, joints):
        msg = Float64MultiArray()
        msg.data = [float(q) for q in joints]
        self.joint_ref_publisher.publish(msg)
    
    def call_python_ik(self, x, y, z, roll, pitch, yaw):
        try:
            success, q_sol, msg, pos_err, rot_err = self.ik_solver.solve(
                x, y, z, roll, pitch, yaw,
                q_init=np.array(self.current_joints_state, dtype=float)
            )

            self.get_logger().info(
                f'Python IK: success={success}, pos_err={pos_err:.6f}, rot_err={rot_err:.6f}, msg={msg}'
            )

            lower = self.ik_solver.lower
            upper = self.ik_solver.upper

            if np.any(q_sol < lower) or np.any(q_sol > upper):
                self.get_logger().error('Python IK returned joints outside limits.')
                return None
            
            if success or pos_err < 0.02:
                return q_sol.tolist()
            else:
                self.get_logger().error(
                    f'Python IK failed. pos_err={pos_err:.6f}, rot_err={rot_err:.6f}, msg={msg}'
                )
                return None

        except Exception as e:
            self.get_logger().error(f'Python IK exception: {e}')
            return None

    def is_pose_in_workspace(self, x, y, z):
        # Coarse workspace constraint for engineering use
        r = (x**2 + y**2 + z**2) ** 0.5

        # Based on empirical tests, z=0.7 m is clearly unreachable
        max_reach = 0.55   
        min_reach = 0.05   

        z_min = 0.02
        z_max = 0.50

        if r < min_reach or r > max_reach:
            return False

        if z < z_min or z > z_max:
            return False

        return True

    def compute_trajectory_timing(self, start_joints, goal_joints):
        dq = [abs(goal_joints[i] - start_joints[i]) for i in range(6)]

        times_needed = []
        for i in range(6):
            vmax = self.joint_vel_limits[i]
            if vmax > 1e-6:
                times_needed.append(dq[i] / vmax)
            else:
                times_needed.append(0.0)

        T_total = max(times_needed)

        # Minimum duration to avoid aggressive reference jumps
        T_total = max(T_total, 1.5)

        # Maximum duration to avoid overly slow motion
        T_total = min(T_total, 12.0)

        total_steps = max(5, int(np.ceil(T_total / self.dt)))

        return T_total, total_steps

    def execute_callback(self, goal_handle):
        x = goal_handle.request.x
        y = goal_handle.request.y
        z = goal_handle.request.z
        roll = goal_handle.request.roll
        pitch = goal_handle.request.pitch
        yaw = goal_handle.request.yaw

        self.get_logger().info(
            f'Received target pose: x={x}, y={y}, z={z}, roll={roll}, pitch={pitch}, yaw={yaw}'
        )

        if not self.is_pose_in_workspace(x, y, z):
            self.get_logger().error('Target unreachable: outside workspace')
            self.robot_status = 'idle'
            goal_handle.abort()

            result = MoveToPose.Result()
            result.success = False
            result.final_joints = self.current_joints_state.copy()
            result.message = 'Target unreachable'
            return result

        self.robot_status = 'busy'

        target_joints = self.call_python_ik(x, y, z, roll, pitch, yaw)

        if target_joints is None:
            self.robot_status = 'idle'
            goal_handle.abort()

            result = MoveToPose.Result()
            result.success = False
            result.final_joints = self.current_joints_state.copy()
            result.message = 'Target unreachable or IK failed'
            return result

        self.get_logger().info(f'Current stored state BEFORE planning: {self.current_joints_state}')
        self.get_logger().info(f'New IK goal joints: {target_joints}')

        # Start point = final state of previous task
        start_joints = self.current_joints_state.copy()
        goal_joints = target_joints.copy()

        self.get_logger().info(f'Start joints for this trajectory: {start_joints}')
        self.get_logger().info(f'Goal joints for this trajectory: {goal_joints}')

        T_total, total_steps = self.compute_trajectory_timing(start_joints, goal_joints)
        dt = self.dt

        self.get_logger().info(
            f'Trajectory timing: T_total={T_total:.3f}s, dt={dt:.3f}s, total_steps={total_steps}'
        )

        feedback_msg = MoveToPose.Feedback()
        feedback_msg.total_steps = total_steps

        with open(self.trajectory_file, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['time', 'q1_ref', 'q2_ref', 'q3_ref', 'q4_ref', 'q5_ref', 'q6_ref'])

            current_joints = start_joints.copy()

            for step in range(total_steps + 1):
                if goal_handle.is_cancel_requested:
                    self.get_logger().info('Task canceled!')
                    self.robot_status = 'canceled'
                    goal_handle.canceled()

                    self.current_joints_state = current_joints.copy()

                    result = MoveToPose.Result()
                    result.success = False
                    result.final_joints = current_joints
                    result.message = 'Canceled'
                    return result

                t = step * dt
                s = step / total_steps

                current_joints = [
                    start_joints[i] + s * (goal_joints[i] - start_joints[i])
                    for i in range(6)
                ]
                self.publish_joint_ref(current_joints)

                writer.writerow([t] + current_joints)

                feedback_msg.current_step = step
                feedback_msg.current_joints = current_joints

                self.get_logger().info(
                    f'Step {step}/{total_steps}: joints={current_joints}'
                )

                goal_handle.publish_feedback(feedback_msg)
                time.sleep(dt)

        goal_handle.succeed()
        self.robot_status = 'completed'

        # Store final state as the next start point
        self.get_logger().info(f'Updating current_joints_state to: {goal_joints}')
        self.current_joints_state = goal_joints.copy()

        result = MoveToPose.Result()
        result.success = True
        result.final_joints = goal_joints
        result.message = 'Trajectory executed from previous final state'
        return result


def main(args=None):
    rclpy.init(args=args)
    node = RobotTaskManager()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
