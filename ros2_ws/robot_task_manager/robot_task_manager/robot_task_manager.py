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
    """
    ROS2 action server for robotic task execution.

    This node receives Cartesian pose goals or waypoint sequences,
    solves inverse kinematics, generates joint-space trajectories,
    publishes joint references, and exports offline trajectory CSV files.
    """
    def __init__(self):
        super().__init__('robot_task_manager')

        self.robot_status = 'idle'

        # Current joint state used as the start point for the next trajectory
        self.current_joints_state = [0.0] * 6

        # Internal waypoint buffer reserved for future direct node-side sequence handling
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
        
        # Action server
        self._callback_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            MoveToPose,
            'execute_task',
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._callback_group
        )
        
        # Resolve trajectory CSV path
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
        """
        Create a MoveToPose action result message.
        """
        result = MoveToPose.Result()
        result.success = bool(success)
        result.final_joints = [float(q) for q in final_joints]
        result.message = str(message)
        return result
    
    def publish_status(self):
        """
        Publish the current robot task status.
        """
        msg = String()
        msg.data = self.robot_status
        self.status_publisher.publish(msg)

    def cancel_callback(self, goal_handle):
        """
        Accept incoming action cancel requests.
        """
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def publish_joint_ref(self, joints):
        """
        Publish the current joint reference to the /joint_ref topic.
        """
        msg = Float64MultiArray()
        msg.data = [float(q) for q in joints]
        self.joint_ref_publisher.publish(msg)
    
    def call_python_ik(self, x, y, z, roll, pitch, yaw):
        """
        Solve IK using the multi-start IK solver.

        This function is mainly kept for general pose solving and
        legacy trajectory experiments. The continuous L-motion pipeline
        uses solve_pose_to_joint_target_seed() instead.
        """
        try:
            success, q_sol, msg, pos_err, rot_err = self.ik_solver.solve(
                x, y, z, roll, pitch, yaw,
                q_init=np.array(self.current_joints_state, dtype=float)
            )

            self.get_logger().info(
                f'IK: success={success}, pos_err={pos_err:.6f}, rot_err={rot_err:.6f}, msg={msg}'
            )

            lower = self.ik_solver.lower
            upper = self.ik_solver.upper

            if np.any(q_sol < lower) or np.any(q_sol > upper):
                self.get_logger().error('IK solution outside joint limits.')
                return None
            
            if success or pos_err < 0.02:
                return q_sol.tolist()
            else:
                self.get_logger().error(
                    f'IK failed. pos_err={pos_err:.6f}, rot_err={rot_err:.6f}, msg={msg}'
                )
                return None

        except Exception as e:
            self.get_logger().error(f'IK exception: {e}')
            return None

    def solve_pose_to_joint_target(self, x, y, z, roll, pitch, yaw):
        """
        Solve a Cartesian pose into a joint target using multi-start IK.

        This is a general pose-solving helper. The main waypoint sequence
        pipeline uses solve_pose_to_joint_target_seed() for better IK
        continuity during Cartesian L-motion.
        """
        if not self.is_pose_in_workspace(x, y, z):
            return None, 'Target unreachable: outside workspace'

        target_joints = self.call_python_ik(x, y, z, roll, pitch, yaw)
        if target_joints is None:
            return None, 'Target unreachable or IK failed'

        return target_joints, 'OK'

    def is_pose_in_workspace(self, x, y, z):
        """
        Check whether a Cartesian pose is inside the coarse workspace bounds.

        The current bounds are conservative engineering limits rather than
        a fully sampled or analytically derived workspace.
        """
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
        """
        Legacy timing helper for direct cubic joint interpolation.

        This function is kept for future trajectory experiments.
        It is not used by the current v1.1 waypoint sequence pipeline.
        """
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
        """
        Legacy cubic joint trajectory generator.

        This function generates a direct cubic trajectory from one joint
        configuration to another. It is kept for future trajectory experiments
        and is not used by the current v1.1 waypoint sequence pipeline.
        """
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
        """
        Generate linearly interpolated Cartesian pose waypoints.

        Args:
            start_pose: Starting Cartesian pose dictionary.
            goal_pose: Target Cartesian pose dictionary.
            num_steps: Number of interpolation intervals.

        Returns:
            A list of Cartesian pose dictionaries.
        """
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
        """
        Publish a trajectory sample by sample through /joint_ref.

        This function is mainly used by real-time execution experiments.
        The offline sequence mode primarily exports the generated trajectory
        to CSV for MATLAB/Simulink replay.
        """
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

    def execute_callback(self, goal_handle):
        """
        Handle MoveToPose action requests.

        Supports two modes:
        1. Waypoint sequence: JSON array of pose waypoints.
        2. Single-goal: Cartesian pose with x, y, z, roll, pitch, yaw fields.
        """
        self.robot_status = 'busy'

       	# Case 1: Waypoint sequence execution
        if goal_handle.request.waypoints_json:
            try:
                waypoint_list = json.loads(goal_handle.request.waypoints_json)
                full_trajectory = self.generate_full_sequence_trajectory(waypoint_list)

                goal_handle.succeed()
                self.robot_status = 'completed'

                return self.make_result(
                    True,
                    self.current_joints_state,
                    f"Waypoint sequence trajectory generated. {len(full_trajectory)} points saved."
                )

            except Exception as e:
                goal_handle.abort()
                self.robot_status = 'idle'
                return self.make_result(
                    False,
                    self.current_joints_state,
                    f"Waypoint sequence failed: {e}"
                )

        # Case 2: Single goal execution
        try:
            target_pose = {
                'type': 'pose',
                'motion': 'R',
                'name': 'single_goal',
                'x': goal_handle.request.x,
                'y': goal_handle.request.y,
                'z': goal_handle.request.z,
                'roll': goal_handle.request.roll,
                'pitch': goal_handle.request.pitch,
                'yaw': goal_handle.request.yaw
            }

            traj, final_joints, _ = self.generate_r_motion_v2(
                self.current_joints_state.copy(),
                target_pose,
                t_offset=0.0
            )

            self.export_trajectory_to_csv(traj)
            self.current_joints_state = final_joints.copy()

            goal_handle.succeed()
            self.robot_status = 'completed'

            return self.make_result(
                True,
                self.current_joints_state,
                f"Single-goal trajectory generated. {len(traj)} points saved."
            )

        except Exception as e:
            goal_handle.abort()
            self.robot_status = 'idle'
            return self.make_result(
                False,
                self.current_joints_state,
                f"Single goal failed: {e}"
            )
    
    def solve_pose_to_joint_target_seed(self, x, y, z, roll, pitch, yaw, q_seed):
        """
        Solve IK with a specified seed joint state.

        This function uses the continuous IK mode and is mainly intended
        for Cartesian L-motion. By using the previous joint solution as
        the optimization seed, it reduces IK branch switching and wrist flips.
        """
        if not self.is_pose_in_workspace(x, y, z):
            return None, 'Target unreachable: outside workspace'

        try:
            success, q_sol, msg, pos_err, rot_err = self.ik_solver.solve_continuous(
                x, y, z, roll, pitch, yaw,
                q_init=np.array(q_seed, dtype=float)
            )

            self.get_logger().info(
                f'IK Continuous: success={success}, '
                f'pos_err={pos_err:.6f}, rot_err={rot_err:.6f}, msg={msg}'
            )

            lower = self.ik_solver.lower
            upper = self.ik_solver.upper

            if np.any(q_sol < lower) or np.any(q_sol > upper):
                return None, 'IK returned joints outside limits'

            if success or pos_err < 0.02:
                return q_sol.tolist(), 'OK'

            return None, f'Continuous IK failed. pos_err={pos_err:.6f}, rot_err={rot_err:.6f}, msg={msg}'

        except Exception as e:
            return None, f'Continuous IK exception: {e}'


    def generate_joint_path_trajectory(self, joint_path, t_offset=0.0):
        """
        Generate a continuous trajectory along a joint-space path.

        The input joint path may contain two or more joint configurations.
        A cubic time-scaling function is used to move along the path smoothly,
        while the global trajectory time is shifted by t_offset.

        Args:
            joint_path: List of joint vectors.
            t_offset: Global time offset for trajectory concatenation.

        Returns:
            trajectory: List of trajectory samples.
            T_total: Total duration of this trajectory segment.
        """
        if len(joint_path) < 2:
            raise RuntimeError("joint_path must contain at least two points")

        joint_path = [np.array(q, dtype=float) for q in joint_path]

        # Compute cumulative path length per joint and determine max displacement for timing
        total_abs_dq = np.zeros(6)
        for i in range(len(joint_path) - 1):
            total_abs_dq += np.abs(joint_path[i + 1] - joint_path[i])

        times_needed = []
        for i in range(6):
            vmax = self.joint_vel_limits[i]
            times_needed.append(total_abs_dq[i] / vmax if vmax > 1e-6 else 0.0)

        T_total = max(times_needed)
        T_total = max(2.0, min(T_total, 12.0))

        total_steps = min(max(int(np.ceil(T_total / self.dt)), 5), 240)

        # Build cumulative path length in joint space
        seg_lengths = []
        cumulative = [0.0]

        for i in range(len(joint_path) - 1):
            length = float(np.linalg.norm(joint_path[i + 1] - joint_path[i]))
            seg_lengths.append(length)
            cumulative.append(cumulative[-1] + length)

        total_length = cumulative[-1]

        trajectory = []

        for step in range(total_steps + 1):
            t_local = T_total * step / total_steps
            u = t_local / T_total if T_total > 1e-9 else 1.0

            # Smooth cubic time scaling
            s_ratio = 3.0 * u**2 - 2.0 * u**3

            if total_length < 1e-9:
                q = joint_path[-1].copy()
            else:
                s = s_ratio * total_length

                seg_idx = 0
                while seg_idx < len(seg_lengths) - 1 and cumulative[seg_idx + 1] < s:
                    seg_idx += 1

                seg_start = cumulative[seg_idx]
                seg_len = seg_lengths[seg_idx]

                if seg_len < 1e-9:
                    alpha = 0.0
                else:
                    alpha = (s - seg_start) / seg_len

                q = (1.0 - alpha) * joint_path[seg_idx] + alpha * joint_path[seg_idx + 1]

            trajectory.append({
                't': float(t_offset + t_local),
                'q': [float(v) for v in q],
                'qd': [0.0] * 6,
                'qdd': [0.0] * 6
            })

        return trajectory, T_total


    def generate_r_motion_v2(self, current_joints, target_pose, t_offset=0.0):
        """
        Generate an R-motion segment.

        R-motion is defined as a joint-space motion from the current joint
        configuration to the IK solution of the target Cartesian pose.
        """
        target_joints, msg = self.solve_pose_to_joint_target_seed(
            target_pose['x'], target_pose['y'], target_pose['z'],
            target_pose['roll'], target_pose['pitch'], target_pose['yaw'],
            current_joints
        )

        if target_joints is None:
            raise RuntimeError(f"R motion IK failed: {msg}")

        traj, T_total = self.generate_joint_path_trajectory(
            [current_joints, target_joints],
            t_offset=t_offset
        )

        return traj, target_joints.copy(), t_offset + T_total


    def generate_l_motion_single_segment(self, start_pose, goal_pose, current_joints, t_offset=0.0):
        """
        Generate an L-motion segment as one continuous trajectory.

        L-motion is defined as Cartesian straight-line motion between two
        poses. The Cartesian path is first interpolated, then each pose is
        solved using continuous IK, and finally the resulting joint path is
        time-parameterized as a single trajectory segment.

        If any intermediate IK solution fails or shows a large branch jump,
        the entire L-motion segment is rejected.
        """
        num_steps = goal_pose.get('num_steps', 50)
        pose_points = self.generate_linear_pose_waypoints(
            start_pose,
            goal_pose,
            num_steps=num_steps
        )

        joint_path = [list(current_joints)]
        q_seed = list(current_joints)

        # Skip the first interpolated pose because it corresponds to the current configuration.
        for idx, pose in enumerate(pose_points[1:], start=1):
            q_sol, msg = self.solve_pose_to_joint_target_seed(
                pose['x'], pose['y'], pose['z'],
                pose['roll'], pose['pitch'], pose['yaw'],
                q_seed
            )

            if q_sol is None:
                raise RuntimeError(
                    f"L motion failed at interpolation point {idx}/{num_steps}: {msg}"
                )

            joint_jump = [abs(q_sol[i] - q_seed[i]) for i in range(6)]
            max_joint_jump = max(joint_jump)

            if max_joint_jump > 0.8:
                raise RuntimeError(
                    f"L motion IK branch jump at point {idx}/{num_steps}: "
                    f"max_joint_jump={max_joint_jump:.3f} rad, "
                    f"joint_jump={joint_jump}"
                )

            joint_path.append(q_sol)
            q_seed = q_sol

        traj, T_total = self.generate_joint_path_trajectory(
            joint_path,
            t_offset=t_offset
        )

        final_joints = joint_path[-1]
        return traj, final_joints.copy(), t_offset + T_total

    def generate_full_sequence_trajectory(self, waypoint_list):
        """
        Generate a full trajectory for a sequence of waypoints.

        Supported waypoint types:
        - pose with R motion: joint-space motion to the target pose
        - pose with L motion: Cartesian straight-line motion from the previous pose
        - joint: direct joint-space motion to the target joint configuration

        The generated segments are concatenated with continuous global time
        and exported as a single trajectory CSV.
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
                    traj, current_joints, t_offset = self.generate_r_motion_v2(
                        current_joints,
                        wp,
                        t_offset
                    )

                elif motion == 'L':
                    if previous_pose_wp is None:
                        raise RuntimeError(
                            f"L motion at waypoint {idx} has no previous pose waypoint"
                        )

                    traj, current_joints, t_offset = self.generate_l_motion_single_segment(
                        previous_pose_wp,
                        wp,
                        current_joints,
                        t_offset
                    )

                else:
                    raise RuntimeError(f"Unknown motion type '{motion}' at waypoint {idx}")

                # Avoid duplicate time/sample at segment boundary
                if full_trajectory and traj:
                    full_trajectory.extend(traj[1:])
                else:
                    full_trajectory.extend(traj)

                previous_pose_wp = wp.copy()

            elif wp_type == 'joint':
                target_joints = wp['joints']

                traj, T_total = self.generate_joint_path_trajectory(
                    [current_joints, target_joints],
                    t_offset=t_offset
                )

                t_offset += T_total

                if full_trajectory and traj:
                    full_trajectory.extend(traj[1:])
                else:
                    full_trajectory.extend(traj)

                current_joints = target_joints.copy()
                previous_pose_wp = None

            else:
                raise RuntimeError(f"Unknown waypoint type '{wp_type}' at index {idx}")

        if full_trajectory:
            self.export_trajectory_to_csv(full_trajectory)
            self.current_joints_state = full_trajectory[-1]['q'].copy()

        self.get_logger().info(
            f"Full trajectory generated: {len(full_trajectory)} points. CSV saved."
        )

        return full_trajectory

    def export_trajectory_to_csv(self, trajectory, filename=None):
        """
        Export a generated joint trajectory to a CSV file.

        The CSV file is used by MATLAB/Simulink for offline trajectory replay.
        """
        if filename is None:
            filename = os.path.expanduser(
                '~/ros2_project_data/matlab/trajectory/trajectory_log_6dof.csv'
            )
        os.makedirs(os.path.dirname(filename), exist_ok=True)

        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            # Write CSV header
            writer.writerow(['time', 'q1_ref', 'q2_ref', 'q3_ref', 'q4_ref', 'q5_ref', 'q6_ref'])
            for pt in trajectory:
                writer.writerow([pt['t'], *pt['q']])

def main(args=None):
    """
    Start the RobotTaskManager node with a multi-threaded executor.
    """
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
