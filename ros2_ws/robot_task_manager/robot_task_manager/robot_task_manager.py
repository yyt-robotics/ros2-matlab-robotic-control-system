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
        if not goal_handle.request.waypoints_json:
            goal_handle.abort()
            return self.make_result(False, self.current_joints_state, "No waypoints provided")

        try:
            waypoint_list = json.loads(goal_handle.request.waypoints_json)
        except Exception as e:
            goal_handle.abort()
            return self.make_result(False, self.current_joints_state, f"JSON parse error: {e}")

        try:
            # ===== 生成完整 trajectory =====
            full_trajectory = self.generate_full_sequence_trajectory(waypoint_list)

            # ===== 写入 CSV =====
            if full_trajectory:
                self.export_trajectory_to_csv(full_trajectory)

            # ===== 更新 ROS2 Action =====
            goal_handle.succeed()
            self.robot_status = 'completed'
            self.current_joints_state = full_trajectory[-1]['q'].copy() if full_trajectory else self.current_joints_state.copy()

            return self.make_result(True, self.current_joints_state, f"Trajectory executed. {len(full_trajectory)} points saved.")

        except Exception as e:
            goal_handle.abort()
            self.robot_status = 'idle'
            return self.make_result(False, self.current_joints_state, f"Execution failed: {e}")
        
    def generate_r_motion(self, current_joints, target_pose):
        """
        R-motion: cubic trajectory from current_joints to target_pose
        Returns trajectory list and total duration
        """
        target_joints, msg = self.solve_pose_to_joint_target(
            target_pose['x'], target_pose['y'], target_pose['z'],
            target_pose['roll'], target_pose['pitch'], target_pose['yaw']
        )
        if target_joints is None:
            raise RuntimeError(f"R motion IK failed: {msg}")

        # 计算时间与步数
        dq = [abs(target_joints[i] - current_joints[i]) for i in range(6)]
        times_needed = [dq[i]/self.joint_vel_limits[i] for i in range(6)]
        T_total = max(times_needed)
        T_total = max(2.0, min(T_total, 12.0))  # 限制 2~12s
        total_steps = min(max(int(np.ceil(T_total / self.dt)), 5), 240)

        trajectory = []
        for step in range(total_steps + 1):
            t = step * self.dt
            q = []
            qd = []
            qdd = []
            for i in range(6):
                q0 = current_joints[i]
                qf = target_joints[i]
                dq_i = qf - q0
                a0 = q0
                a1 = 0
                a2 = 3*dq_i/(T_total**2)
                a3 = -2*dq_i/(T_total**3)
                qi = a0 + a1*t + a2*t**2 + a3*t**3
                qdi = a1 + 2*a2*t + 3*a3*t**2
                qddi = 2*a2 + 6*a3*t
                q.append(float(qi))
                qd.append(float(qdi))
                qdd.append(float(qddi))
            trajectory.append({'t': float(t), 'q': q, 'qd': qd, 'qdd': qdd})

        return trajectory, T_total, target_joints.copy()


    def generate_l_motion(self, start_pose, goal_pose, current_joints, t_offset=0.0):
        """
        L-motion: linear end-effector motion from start_pose to goal_pose
        current_joints: joint angles at start
        t_offset: cumulative time offset
        Returns:
            traj_full: list of trajectory points [{'t','q','qd','qdd'}, ...]
            current_joints: last joint state after this motion
            t_offset: updated cumulative time
        """
        num_steps = goal_pose.get('num_steps', 20)
        # 每小段线性插点
        pose_points = self.generate_linear_pose_waypoints(start_pose, goal_pose, num_steps=num_steps)

        traj_full = []

        for idx, pose in enumerate(pose_points):
            # 用 R-motion 生成关节轨迹，每小段最多 12s / 240 步
            traj, T_total, new_joints = self.generate_r_motion(current_joints, pose)

            # 时间累加
            for pt in traj:
                pt['t'] += t_offset
            t_offset += T_total

            # 保存轨迹
            traj_full.extend(traj)
            current_joints = new_joints.copy()

        return traj_full, current_joints, t_offset


    def generate_full_sequence_trajectory(self, waypoint_list):
        """
        Generate full joint trajectory for a sequence of waypoints (R/L/Joint)
        Ensures continuous joints and cumulative time
        """
        full_trajectory = []
        current_joints = self.current_joints_state.copy()
        t_offset = 0.0
        previous_pose_wp = None

        for idx, wp in enumerate(waypoint_list):
            wp_type = wp.get('type', 'pose')

            if wp_type == 'pose':
                motion = wp.get('motion', 'R').upper()

                if motion == 'R':
                    traj, T_total, current_joints = self.generate_r_motion(current_joints, wp)
                    # 累加时间
                    for pt in traj:
                        pt['t'] += t_offset
                    t_offset += T_total
                    full_trajectory.extend(traj)
                    previous_pose_wp = wp.copy()

                elif motion == 'L':
                    if previous_pose_wp is None:
                        raise RuntimeError(f"L motion at waypoint {idx} has no previous pose.")
                    traj, current_joints, t_offset = self.generate_l_motion(previous_pose_wp, wp, current_joints, t_offset)
                    full_trajectory.extend(traj)
                    previous_pose_wp = wp.copy()

                else:
                    raise RuntimeError(f"Unknown motion type '{motion}' at waypoint {idx}")

            elif wp_type == 'joint':
                target_joints = wp['joints']
                traj, T_total, _ = self.generate_cubic_joint_trajectory(current_joints, target_joints)
                # 累加时间
                for pt in traj:
                    pt['t'] += t_offset
                t_offset += T_total
                full_trajectory.extend(traj)
                current_joints = target_joints.copy()
                previous_pose_wp = None

            else:
                raise RuntimeError(f"Unknown waypoint type '{wp_type}' at index {idx}")

        # 写 CSV
        self.export_trajectory_to_csv(full_trajectory)

        # 更新末尾关节状态
        if full_trajectory:
            self.current_joints_state = full_trajectory[-1]['q'].copy()

        self.get_logger().info(f"Full trajectory generated: {len(full_trajectory)} points. CSV saved.")
        return full_trajectory
        
    def execute_linear_pose_motion_trajectory(self, start_pose, goal_pose, current_joints, t_offset=0.0):
        """
        Generate joint trajectory for L-type motion using previous segment's last joint state.
        Returns:
            full_traj: list of dicts [{'t':..., 'q':[q1..q6], 'qd':..., 'qdd':...}]
            t_offset: updated time offset after this segment
        """
        num_steps = goal_pose.get('num_steps', 20)
        total_time = goal_pose.get('duration', num_steps * self.dt)

        # 生成线性插值姿态
        pose_points = self.generate_linear_pose_waypoints(start_pose, goal_pose, num_steps=num_steps)

        full_traj = []

        for idx, pose in enumerate(pose_points):
            target_joints, msg = self.solve_pose_to_joint_target(
                pose['x'], pose['y'], pose['z'], pose['roll'], pose['pitch'], pose['yaw']
            )
            if target_joints is None:
                raise RuntimeError(f"L motion IK failed at point {idx}: {msg}")

            # cubic trajectory 连接
            traj, T_total, _ = self.generate_cubic_joint_trajectory(current_joints, target_joints)

            # 累加时间
            for pt in traj:
                pt['t'] += t_offset

            # 更新时间偏移
            t_offset += traj[-1]['t'] if traj else 0.0

            # 保存 trajectory
            full_traj.extend(traj)

            # 更新当前关节状态，为下一小段使用
            current_joints = traj[-1]['q'].copy()

        return full_traj, current_joints, t_offset

    def export_trajectory_to_csv(self, trajectory, filename=None):
        if filename is None:
            filename = os.path.expanduser(
                '~/ros2_project_data/matlab/trajectory/trajectory_log_6dof.csv'
            )
        os.makedirs(os.path.dirname(filename), exist_ok=True)

        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            # 写表头
            writer.writerow(['time','q1_ref','q2_ref','q3_ref','q4_ref','q5_ref','q6_ref'])
            for pt in trajectory:
                writer.writerow([pt['t'], *pt['q']])

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
