import time
import csv
import os
import numpy as np
import json
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

        # Waypoint container for future sequence execution
        self.waypoints = []

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
        
        # ===== Resolve trajectory CSV path =====
        data_dir = os.path.expanduser('~/ros2_project_data')
        external_trajectory_file = os.path.join(
            data_dir, 'matlab', 'trajectory', 'trajectory_log_6dof.csv'
        )

        # Project root fallback: go up from this file to repository root
        current_file_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.abspath(os.path.join(current_file_dir, '..', '..', '..'))
        repo_trajectory_file = os.path.join(
            project_root, 'matlab', 'trajectory', 'trajectory_log_6dof.csv'
        )

        # Prefer external data dir if it exists, otherwise fallback to repo path
        if os.path.isdir(data_dir):
            default_trajectory_file = external_trajectory_file
        else:
            default_trajectory_file = repo_trajectory_file

        self.declare_parameter('trajectory_file', default_trajectory_file)
        self.trajectory_file = (
            self.get_parameter('trajectory_file')
            .get_parameter_value()
            .string_value
        )

        os.makedirs(os.path.dirname(self.trajectory_file), exist_ok=True)
        self.get_logger().info(f'Offline trajectory CSV path: {self.trajectory_file}')

    def make_result(self, success, final_joints, message):
        result = MoveToPose.Result()
        result.success = bool(success)
        result.final_joints = [float(q) for q in final_joints]
        result.message = str(message)
        return result
    
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

    def solve_pose_to_joint_target(self, x, y, z, roll, pitch, yaw):
        if not self.is_pose_in_workspace(x, y, z):
            return None, 'Target unreachable: outside workspace'

        target_joints = self.call_python_ik(x, y, z, roll, pitch, yaw)
        if target_joints is None:
            return None, 'Target unreachable or IK failed'

        return target_joints, 'OK'

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

    def generate_cubic_joint_trajectory(self, start_joints, goal_joints):
        T_total, total_steps = self.compute_trajectory_timing(start_joints, goal_joints)
        dt = self.dt

        trajectory = []

        for step in range(total_steps + 1):
            t = min(step * dt, T_total)

            q = []
            qd = []
            qdd = []

            for i in range(6):
                q0 = float(start_joints[i])
                qf = float(goal_joints[i])
                dq = qf - q0

                if T_total < 1e-9:
                    qi = qf
                    qdi = 0.0
                    qddi = 0.0
                else:
                    a0 = q0
                    a1 = 0.0
                    a2 = 3.0 * dq / (T_total ** 2)
                    a3 = -2.0 * dq / (T_total ** 3)

                    qi = a0 + a1 * t + a2 * (t ** 2) + a3 * (t ** 3)
                    qdi = a1 + 2.0 * a2 * t + 3.0 * a3 * (t ** 2)
                    qddi = 2.0 * a2 + 6.0 * a3 * t

                q.append(float(qi))
                qd.append(float(qdi))
                qdd.append(float(qddi))

            trajectory.append({
                't': float(t),
                'q': q,
                'qd': qd,
                'qdd': qdd
            })

        return trajectory, T_total, total_steps

    def generate_linear_pose_waypoints(self, start_pose, goal_pose, num_steps=20):
        pose_points = []

        for step in range(num_steps + 1):
            s = step / num_steps

            pose = {
                'x': start_pose['x'] + s * (goal_pose['x'] - start_pose['x']),
                'y': start_pose['y'] + s * (goal_pose['y'] - start_pose['y']),
                'z': start_pose['z'] + s * (goal_pose['z'] - start_pose['z']),
                'roll': start_pose['roll'] + s * (goal_pose['roll'] - start_pose['roll']),
                'pitch': start_pose['pitch'] + s * (goal_pose['pitch'] - start_pose['pitch']),
                'yaw': start_pose['yaw'] + s * (goal_pose['yaw'] - start_pose['yaw'])
            }

            pose_points.append(pose)

        return pose_points

    def execute_trajectory(self, goal_handle, trajectory, verbose=True):
        feedback_msg = MoveToPose.Feedback()
        feedback_msg.total_steps = len(trajectory) - 1
        current_joints = self.current_joints_state.copy()

        for step, sample in enumerate(trajectory):
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Task canceled!')
                self.robot_status = 'canceled'
                goal_handle.canceled()
                self.current_joints_state = current_joints.copy()
                return False, current_joints, 'Canceled'

            current_joints = sample['q']
            self.publish_joint_ref(current_joints)

            feedback_msg.current_step = step
            feedback_msg.current_joints = [float(q) for q in current_joints]
            goal_handle.publish_feedback(feedback_msg)

            if verbose:
                self.get_logger().info(f"Step {step}/{len(trajectory)-1}: t={sample['t']:.3f}")

            if step < len(trajectory) - 1:
                time.sleep(self.dt)

        self.current_joints_state = trajectory[-1]['q'].copy()
        return True, self.current_joints_state.copy(), 'Trajectory executed successfully'

    def execute_linear_pose_motion(self, goal_handle, start_pose, goal_pose):
 
        num_steps = goal_pose.get('num_steps', 20)
        total_time = goal_pose.get('duration', num_steps * self.dt)

        pose_points = self.generate_linear_pose_waypoints(start_pose, goal_pose, num_steps=num_steps)
        current_joints = self.current_joints_state.copy()

        for idx, pose in enumerate(pose_points):
            target_joints, msg = self.solve_pose_to_joint_target(
                pose['x'], pose['y'], pose['z'],
                pose['roll'], pose['pitch'], pose['yaw']
            )
            if target_joints is None:
                return False, current_joints, f"L motion IK failed at point {idx}: {msg}"

            steps_per_segment = max(int(total_time / self.dt / num_steps), 1)

            trajectory, _, _ = self.generate_cubic_joint_trajectory(current_joints, target_joints)

            ok, current_joints, exec_msg = self.execute_trajectory(goal_handle, trajectory, verbose=False)
            if not ok:
                return False, current_joints, f"L motion failed at point {idx}: {exec_msg}"

            self.get_logger().info(f"L motion point {idx}/{num_steps} executed successfully.")

        self.current_joints_state = current_joints.copy()
        return True, current_joints, "L motion executed successfully"

    def execute_waypoint_sequence(self, goal_handle, waypoint_list):
        self.get_logger().info(f"Executing waypoint sequence: {len(waypoint_list)} points")

        current_joints = self.current_joints_state.copy()
        previous_pose_wp = None

        for idx, wp in enumerate(waypoint_list):
            self.get_logger().info(f"Waypoint {idx}: {wp}")

            if wp['type'] == 'pose':
                motion = wp.get('motion', 'R')

                if motion == 'R':
                    target_joints, msg = self.solve_pose_to_joint_target(
                        wp['x'], wp['y'], wp['z'],
                        wp['roll'], wp['pitch'], wp['yaw']
                    )
                    if target_joints is None:
                        return False, current_joints, f"Waypoint {idx} failed: {msg}"

                    trajectory, _, _ = self.generate_cubic_joint_trajectory(current_joints, target_joints)
                    ok, current_joints, msg = self.execute_trajectory(goal_handle, trajectory)

                    if not ok:
                        return False, current_joints, f"Waypoint {idx} execution failed: {msg}"

                elif motion == 'L':
                    if previous_pose_wp is None:
                        return False, current_joints, f"Waypoint {idx} is L but has no previous pose waypoint"

                    ok, current_joints, msg = self.execute_linear_pose_motion(goal_handle, previous_pose_wp, wp)

                    if not ok:
                        return False, current_joints, f"Waypoint {idx} execution failed: {msg}"

                else:
                    return False, current_joints, f"Unknown motion type: {motion}"

                previous_pose_wp = {
                    'x': wp['x'],
                    'y': wp['y'],
                    'z': wp['z'],
                    'roll': wp['roll'],
                    'pitch': wp['pitch'],
                    'yaw': wp['yaw']
                }

            elif wp['type'] == 'joint':
                target_joints = wp['joints']
                trajectory, _, _ = self.generate_cubic_joint_trajectory(current_joints, target_joints)
                ok, current_joints, msg = self.execute_trajectory(goal_handle, trajectory)

                if not ok:
                    return False, current_joints, f"Waypoint {idx} execution failed: {msg}"

                previous_pose_wp = None

            else:
                return False, current_joints, f"Unknown waypoint type: {wp['type']}"

        return True, current_joints, "Waypoint sequence executed"

    def export_trajectory_to_csv(self, trajectory):
        with open(self.trajectory_file, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['time', 'q1_ref', 'q2_ref', 'q3_ref', 'q4_ref', 'q5_ref', 'q6_ref'])

            for sample in trajectory:
                writer.writerow([sample['t']] + sample['q'])

    def execute_callback(self, goal_handle):
        if goal_handle.request.waypoints_json:
            try:
                waypoint_list = json.loads(goal_handle.request.waypoints_json)
            except Exception as e:
                goal_handle.abort()
                return self.make_result(False, self.current_joints_state, f"JSON parse error: {e}")

            ok, final_joints, msg = self.execute_waypoint_sequence(goal_handle, waypoint_list)

            if not ok:
                self.robot_status = 'idle'
                goal_handle.abort()
                return self.make_result(False, final_joints, msg)

            goal_handle.succeed()
            self.robot_status = 'completed'
            return self.make_result(True, final_joints, msg)

        x = goal_handle.request.x
        y = goal_handle.request.y
        z = goal_handle.request.z
        roll = goal_handle.request.roll
        pitch = goal_handle.request.pitch
        yaw = goal_handle.request.yaw

        self.get_logger().info(
            f'Received target pose: x={x}, y={y}, z={z}, roll={roll}, pitch={pitch}, yaw={yaw}'
        )

        self.robot_status = 'busy'

        use_waypoint_test = False

        if use_waypoint_test:
            waypoint_list = [
                {
                    'type': 'pose',
                    'x': x,
                    'y': y,
                    'z': z,
                    'roll': roll,
                    'pitch': pitch,
                    'yaw': yaw
                },
                {
                    'type': 'joint',
                    'joints': self.current_joints_state.copy()
                }
            ]

            ok, final_joints, exec_msg = self.execute_waypoint_sequence(goal_handle, waypoint_list)

            if not ok:
                self.robot_status = 'idle'
                goal_handle.abort()
                return self.make_result(False, final_joints, exec_msg)

            goal_handle.succeed()
            self.robot_status = 'completed'
            return self.make_result(True, final_joints, exec_msg)

        target_joints, msg = self.solve_pose_to_joint_target(x, y, z, roll, pitch, yaw)
        if target_joints is None:
            self.robot_status = 'idle'
            goal_handle.abort()
            return self.make_result(False, self.current_joints_state.copy(), msg)

        start_joints = self.current_joints_state.copy()
        goal_joints = target_joints.copy()

        self.get_logger().info(f'Start joints for this trajectory: {start_joints}')
        self.get_logger().info(f'Goal joints for this trajectory: {goal_joints}')

        trajectory, T_total, total_steps = self.generate_cubic_joint_trajectory(start_joints, goal_joints)

        self.get_logger().info(
            f'Cubic trajectory generated: T_total={T_total:.3f}s, dt={self.dt:.3f}s, total_steps={total_steps}'
        )

        self.export_trajectory_to_csv(trajectory)

        ok, final_joints, exec_msg = self.execute_trajectory(goal_handle, trajectory)

        if not ok:
            self.robot_status = 'idle'
            return self.make_result(False, final_joints, exec_msg)

        goal_handle.succeed()
        self.robot_status = 'completed'
        return self.make_result(True, final_joints, exec_msg)


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
