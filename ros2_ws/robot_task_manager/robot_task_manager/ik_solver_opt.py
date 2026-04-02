"""
Numerical inverse kinematics solver for the Gluon 6L3 manipulator
using constrained nonlinear least-squares optimization.
"""

import math
import numpy as np
from scipy.optimize import least_squares


def rot_x(alpha: float) -> np.ndarray:
    c = math.cos(alpha)
    s = math.sin(alpha)
    return np.array([
        [1, 0, 0],
        [0, c, -s],
        [0, s, c]
    ], dtype=float)

def rot_y(beta: float) -> np.ndarray:
    c = math.cos(beta)
    s = math.sin(beta)
    return np.array([
        [c, 0, s],
        [0, 1, 0],
        [-s, 0, c]
    ], dtype=float)


def rot_z(theta: float) -> np.ndarray:
    c = math.cos(theta)
    s = math.sin(theta)
    return np.array([
        [c, -s, 0],
        [s,  c, 0],
        [0,  0, 1]
    ], dtype=float)


def pose_to_transform(x: float, y: float, z: float,
                      roll: float, pitch: float, yaw: float) -> np.ndarray:
    R = rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)
    T = np.eye(4, dtype=float)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T


class Gluon6L3IKLS:
    def __init__(self):
        self.d = np.array([105.03, 0.0, 0.0, 75.66, 80.09, 44.36], dtype=float) / 1000.0
        self.a = np.array([0.0, -174.42, -174.42, 0.0, 0.0, 0.0], dtype=float) / 1000.0
        self.alpha = np.array([math.pi / 2, 0.0, 0.0, math.pi / 2, -math.pi / 2, 0.0], dtype=float)
        self.offset = np.array([0.0, -math.pi / 2, 0.0, -math.pi / 2, 0.0, 0.0], dtype=float)

        self.lower = np.array([
            math.radians(-160),
            math.radians(-90),
            math.radians(-150),
            math.radians(-150),
            math.radians(-150),
            math.radians(-360)
        ], dtype=float)

        self.upper = np.array([
            math.radians(160),
            math.radians(90),
            math.radians(150),
            math.radians(150),
            math.radians(150),
            math.radians(360)
        ], dtype=float)

    def candidate_initial_guesses(self, q_init: np.ndarray | None = None):
        guesses = []

        if q_init is not None:
            guesses.append(np.array(q_init, dtype=float))

        guesses.extend([
            np.zeros(6, dtype=float),
            np.array([0.0, -0.5, 0.5, 0.0, 0.0, 0.0], dtype=float),
            np.array([0.5, -0.8, 0.3, 0.0, 0.5, 0.0], dtype=float),
            np.array([-0.5, -0.8, 0.3, 0.0, -0.5, 0.0], dtype=float),
            np.array([0.3, 0.0, -0.8, 0.0, 0.3, 0.0], dtype=float),
            np.array([0.0, 0.3, -0.8, 0.0, 0.0, 0.0], dtype=float),
            np.array([0.0, -1.0, 1.0, 0.0, 0.0, 0.0], dtype=float),
        ])

        return guesses

    def joint_limit_cost(self, q: np.ndarray) -> float:
        q_mid = 0.5 * (self.lower + self.upper)
        q_half = 0.5 * (self.upper - self.lower)

        normalized = (q - q_mid) / q_half
        return float(np.sum(normalized ** 2))
    
    def dh_transform(self, theta: float, d: float, a: float, alpha: float) -> np.ndarray:
        ct = math.cos(theta)
        st = math.sin(theta)
        ca = math.cos(alpha)
        sa = math.sin(alpha)

        return np.array([
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0.0,      sa,      ca,      d],
            [0.0,     0.0,     0.0,    1.0]
        ], dtype=float)

    def fk(self, q: np.ndarray) -> np.ndarray:
        T = np.eye(4, dtype=float)
        for i in range(6):
            theta = q[i] + self.offset[i]
            T = T @ self.dh_transform(theta, self.d[i], self.a[i], self.alpha[i])
        return T

    def position_residual(self, q: np.ndarray, p_target: np.ndarray, w_pos: float = 1.0) -> np.ndarray:
        T_current = self.fk(q)
        p_current = T_current[:3, 3]
        dp = p_target - p_current
        return w_pos * dp

    def pose_residual(self, q: np.ndarray, T_target: np.ndarray,
        w_pos: float = 5.0, w_rot: float = 0.05) -> np.ndarray:
        T_current = self.fk(q)

        p_current = T_current[:3, 3]
        p_target = T_target[:3, 3]
        dp = p_target - p_current

        R_current = T_current[:3, :3]
        R_target = T_target[:3, :3]

        drot = 0.5 * (
            np.cross(R_current[:, 0], R_target[:, 0]) +
            np.cross(R_current[:, 1], R_target[:, 1]) +
            np.cross(R_current[:, 2], R_target[:, 2])
        )

        q_mid = 0.5 * (self.lower + self.upper)
        q_half = 0.5 * (self.upper - self.lower)
        limit_penalty = 0.01 * ((q - q_mid) / q_half)
        
        return np.concatenate([w_pos * dp, w_rot * drot, limit_penalty])

    def solve_position_only(self, x: float, y: float, z: float,
                            q_init: np.ndarray | None = None) -> np.ndarray:
        p_target = np.array([x, y, z], dtype=float)

        if q_init is None:
            q0 = np.zeros(6, dtype=float)
        else:
            q0 = np.array(q_init, dtype=float).copy()

        result = least_squares(
            fun=lambda q: self.position_residual(q, p_target, w_pos=1.0),
            x0=q0,
            bounds=(self.lower, self.upper),
            method='trf',
            ftol=1e-12,
            xtol=1e-12,
            gtol=1e-12,
            max_nfev=2000
        )

        return result.x

    def solve_once(self,
                   x: float, y: float, z: float,
                   roll: float, pitch: float, yaw: float,
                   q_init: np.ndarray | None = None) -> tuple[np.ndarray, str, float, float]:
        T_target = pose_to_transform(x, y, z, roll, pitch, yaw)

        # stage 1: position only
        q_stage1 = self.solve_position_only(x, y, z, q_init=q_init)

        # stage 2: weak orientation refinement
        result = least_squares(
            fun=lambda q: self.pose_residual(q, T_target, w_pos=5.0, w_rot=0.05),
            x0=q_stage1,
            bounds=(self.lower, self.upper),
            method='trf',
            ftol=1e-10,
            xtol=1e-10,
            gtol=1e-10,
            max_nfev=500
        )

        q_sol = result.x
        T_sol = self.fk(q_sol)

        p_sol = T_sol[:3, 3]
        p_target = T_target[:3, 3]
        pos_err_norm = float(np.linalg.norm(p_target - p_sol))

        R_sol = T_sol[:3, :3]
        R_target = T_target[:3, :3]
        rot_err = 0.5 * (
            np.cross(R_sol[:, 0], R_target[:, 0]) +
            np.cross(R_sol[:, 1], R_target[:, 1]) +
            np.cross(R_sol[:, 2], R_target[:, 2])
        )
        rot_err_norm = float(np.linalg.norm(rot_err))

        return q_sol, str(result.message), pos_err_norm, rot_err_norm

    def solve(self,
              x: float, y: float, z: float,
              roll: float, pitch: float, yaw: float,
              q_init: np.ndarray | None = None) -> tuple[bool, np.ndarray, str, float, float]:

        candidates = []

        for guess in self.candidate_initial_guesses(q_init):
            try:
                q_sol, msg, pos_err_norm, rot_err_norm = self.solve_once(
                    x, y, z, roll, pitch, yaw, q_init=guess
                )

                limit_cost = self.joint_limit_cost(q_sol)

                candidates.append({
                    "q": q_sol,
                    "msg": msg,
                    "pos_err": pos_err_norm,
                    "rot_err": rot_err_norm,
                    "limit_cost": limit_cost
                })

            except Exception as e:
                print(f"IK solve_once failed for one initial guess: {e}")

        if not candidates:
            return False, np.zeros(6), "All IK attempts failed", float('inf'), float('inf')

        # 先筛掉位置太差的
        good_pos_candidates = [c for c in candidates if c["pos_err"] < 2e-2]

        if good_pos_candidates:
            # 在位置合格的前提下，综合比较姿态和限位
            best = min(
                good_pos_candidates,
                key=lambda c: (c["rot_err"] / 0.1) ** 2 + 0.05 * c["limit_cost"]
            )
            success = best["rot_err"] < 0.1
        else:
            # 没有位置合格的，就选位置最好的
            best = min(candidates, key=lambda c: c["pos_err"])
            success = False

        q_sol = best["q"]
        pos_err_norm = best["pos_err"]
        rot_err_norm = best["rot_err"]
        msg = best["msg"]

        at_lower = np.isclose(q_sol, self.lower, atol=1e-3)
        at_upper = np.isclose(q_sol, self.upper, atol=1e-3)

        if np.any(at_lower) or np.any(at_upper):
            print("Warning: IK solution is close to joint limits.")
            print("at_lower:", at_lower.tolist())
            print("at_upper:", at_upper.tolist())

        return success, q_sol, msg, pos_err_norm, rot_err_norm

if __name__ == "__main__":
    solver = Gluon6L3IKLS()

    tests = [
        # name, x, y, z, roll, pitch, yaw, q_guess
        ("easy",   0.20, 0.10, 0.25, 0.0, 0.0, math.pi / 4,
         np.array([0.5, -0.8, 0.3, 0.0, 0.5, 0.0], dtype=float)),

        ("medium", 0.10, 0.10, 0.10, 0.0, 0.0, math.pi / 4,
         np.array([0.5, -0.8, 0.3, 0.0, 0.5, 0.0], dtype=float)),

        ("hard",   0.10, 0.10, 0.70, 0.0, 1.17, 0.0,
         np.array([0.5, -0.8, 0.3, 0.0, 0.5, 0.0], dtype=float)),
    ]

    for name, x, y, z, roll, pitch, yaw, q_guess in tests:
        print("\n" + "=" * 60)
        print(f"TEST: {name}")
        print(f"target pose = x={x}, y={y}, z={z}, roll={roll}, pitch={pitch}, yaw={yaw}")

        success, q, msg, pos_err, rot_err = solver.solve(
            x, y, z, roll, pitch, yaw,
            q_init=q_guess
        )

        print("success:", success)
        print("q:", q.tolist())
        print("msg:", msg)
        print("position error norm:", pos_err)
        print("rotation error norm:", rot_err)

        T = solver.fk(q)
        print("fk position:", T[:3, 3].tolist())
        print("fk rotation:")
        print(T[:3, :3])
