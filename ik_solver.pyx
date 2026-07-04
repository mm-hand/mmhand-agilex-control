# cython: boundscheck=False, wraparound=False, cdivision=True, language_level=3
#
# Build before running:
#   /usr/bin/python3 setup.py build_ext --inplace
#
# Public runtime API used by Python code:
#   NeroParams, ContinuityParams, ContinuityRuntimeState
#   fk, fk_all, pose_error, solve_pose_continuous_with_state
#
# Do not edit generated ik_solver.c or compiled .so files; rebuild from this .pyx.
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np
cimport numpy as np

cdef double PI = math.pi
cdef double TWO_PI = 2.0 * math.pi

# ── C-level 4×4 matrix helpers (double[16], row-major) ──────────────

cdef inline void _mat4_eye(double* o):
    o[0]=1; o[1]=0; o[2]=0; o[3]=0
    o[4]=0; o[5]=1; o[6]=0; o[7]=0
    o[8]=0; o[9]=0; o[10]=1; o[11]=0
    o[12]=0; o[13]=0; o[14]=0; o[15]=1

cdef void _mat4_mul(double* A, double* B, double* o):
    cdef double t[16]
    t[0]=A[0]*B[0]+A[1]*B[4]+A[2]*B[8]+A[3]*B[12]
    t[1]=A[0]*B[1]+A[1]*B[5]+A[2]*B[9]+A[3]*B[13]
    t[2]=A[0]*B[2]+A[1]*B[6]+A[2]*B[10]+A[3]*B[14]
    t[3]=A[0]*B[3]+A[1]*B[7]+A[2]*B[11]+A[3]*B[15]
    t[4]=A[4]*B[0]+A[5]*B[4]+A[6]*B[8]+A[7]*B[12]
    t[5]=A[4]*B[1]+A[5]*B[5]+A[6]*B[9]+A[7]*B[13]
    t[6]=A[4]*B[2]+A[5]*B[6]+A[6]*B[10]+A[7]*B[14]
    t[7]=A[4]*B[3]+A[5]*B[7]+A[6]*B[11]+A[7]*B[15]
    t[8]=A[8]*B[0]+A[9]*B[4]+A[10]*B[8]+A[11]*B[12]
    t[9]=A[8]*B[1]+A[9]*B[5]+A[10]*B[9]+A[11]*B[13]
    t[10]=A[8]*B[2]+A[9]*B[6]+A[10]*B[10]+A[11]*B[14]
    t[11]=A[8]*B[3]+A[9]*B[7]+A[10]*B[11]+A[11]*B[15]
    t[12]=A[12]*B[0]+A[13]*B[4]+A[14]*B[8]+A[15]*B[12]
    t[13]=A[12]*B[1]+A[13]*B[5]+A[14]*B[9]+A[15]*B[13]
    t[14]=A[12]*B[2]+A[13]*B[6]+A[14]*B[10]+A[15]*B[14]
    t[15]=A[12]*B[3]+A[13]*B[7]+A[14]*B[11]+A[15]*B[15]
    cdef int i
    for i in range(16):
        o[i] = t[i]

cdef inline void _dh_A_into(double theta, double d, double a_prev, double alpha_prev, double* o):
    cdef double ca = math.cos(alpha_prev)
    cdef double sa = math.sin(alpha_prev)
    cdef double ct = math.cos(theta)
    cdef double st = math.sin(theta)
    o[0]=ct;      o[1]=-st;     o[2]=0.0;    o[3]=a_prev
    o[4]=ca*st;   o[5]=ca*ct;   o[6]=-sa;    o[7]=-sa*d
    o[8]=sa*st;   o[9]=sa*ct;   o[10]=ca;    o[11]=ca*d
    o[12]=0.0;    o[13]=0.0;    o[14]=0.0;   o[15]=1.0

cdef inline void _invert_rigid_into(double* T, double* o):
    o[0]=T[0]; o[1]=T[4]; o[2]=T[8]
    o[4]=T[1]; o[5]=T[5]; o[6]=T[9]
    o[8]=T[2]; o[9]=T[6]; o[10]=T[10]
    o[3]=-(o[0]*T[3]+o[1]*T[7]+o[2]*T[11])
    o[7]=-(o[4]*T[3]+o[5]*T[7]+o[6]*T[11])
    o[11]=-(o[8]*T[3]+o[9]*T[7]+o[10]*T[11])
    o[12]=0.0; o[13]=0.0; o[14]=0.0; o[15]=1.0

cdef inline void _np4_to_c(np.ndarray src, double* dst):
    dst[0]=src[0,0]; dst[1]=src[0,1]; dst[2]=src[0,2]; dst[3]=src[0,3]
    dst[4]=src[1,0]; dst[5]=src[1,1]; dst[6]=src[1,2]; dst[7]=src[1,3]
    dst[8]=src[2,0]; dst[9]=src[2,1]; dst[10]=src[2,2]; dst[11]=src[2,3]
    dst[12]=src[3,0]; dst[13]=src[3,1]; dst[14]=src[3,2]; dst[15]=src[3,3]

cdef inline void _c_to_np4(double* src, np.ndarray dst):
    dst[0,0]=src[0]; dst[0,1]=src[1]; dst[0,2]=src[2]; dst[0,3]=src[3]
    dst[1,0]=src[4]; dst[1,1]=src[5]; dst[1,2]=src[6]; dst[1,3]=src[7]
    dst[2,0]=src[8]; dst[2,1]=src[9]; dst[2,2]=src[10]; dst[2,3]=src[11]
    dst[3,0]=src[12]; dst[3,1]=src[13]; dst[3,2]=src[14]; dst[3,3]=src[15]

# ── Scalar helpers for dedup and scoring ────────────────────────────

cdef inline double _wrap_to_pi_scalar(double x):
    return (x + PI) % TWO_PI - PI

cdef bint _is_duplicate(np.ndarray q, list unique_sols):
    cdef int i, j, n = len(unique_sols)
    cdef np.ndarray u
    cdef double d, diff
    for i in range(n):
        u = unique_sols[i]
        d = 0.0
        for j in range(7):
            diff = q[j] - u[j]
            if diff > PI: diff -= TWO_PI
            elif diff < -PI: diff += TWO_PI
            d += diff * diff
        if d < 1e-8:
            return True
    return False

cdef double _wrap_diff_norm7(np.ndarray a, np.ndarray b):
    cdef double s = 0.0, d
    cdef int i
    for i in range(7):
        d = a[i] - b[i]
        if d > PI: d -= TWO_PI
        elif d < -PI: d += TWO_PI
        s += d * d
    return math.sqrt(s)

cdef double _wrap_diff_norm7_2prev(np.ndarray q, np.ndarray q_prev, np.ndarray q_prev2):
    cdef double s = 0.0, d
    cdef int i
    for i in range(7):
        d = q[i] - 2.0 * q_prev[i] + q_prev2[i]
        if d > PI: d -= TWO_PI
        elif d < -PI: d += TWO_PI
        s += d * d
    return math.sqrt(s)

cdef double _pose_error_norm(np.ndarray T_cur, np.ndarray T_des):
    cdef double dpx = T_cur[0,3]-T_des[0,3]
    cdef double dpy = T_cur[1,3]-T_des[1,3]
    cdef double dpz = T_cur[2,3]-T_des[2,3]
    cdef double rex = 0.5*((T_cur[1,0]*T_des[2,0]-T_cur[2,0]*T_des[1,0])
        +(T_cur[1,1]*T_des[2,1]-T_cur[2,1]*T_des[1,1])
        +(T_cur[1,2]*T_des[2,2]-T_cur[2,2]*T_des[1,2]))
    cdef double rey = 0.5*((T_cur[2,0]*T_des[0,0]-T_cur[0,0]*T_des[2,0])
        +(T_cur[2,1]*T_des[0,1]-T_cur[0,1]*T_des[2,1])
        +(T_cur[2,2]*T_des[0,2]-T_cur[0,2]*T_des[2,2]))
    cdef double rez = 0.5*((T_cur[0,0]*T_des[1,0]-T_cur[1,0]*T_des[0,0])
        +(T_cur[0,1]*T_des[1,1]-T_cur[1,1]*T_des[0,1])
        +(T_cur[0,2]*T_des[1,2]-T_cur[1,2]*T_des[0,2]))
    return math.sqrt(dpx*dpx+dpy*dpy+dpz*dpz+rex*rex+rey*rey+rez*rez)

cdef double _dist7(np.ndarray a, np.ndarray b):
    cdef double s = 0.0, d
    cdef int i
    for i in range(7):
        d = a[i] - b[i]
        if d > PI: d -= TWO_PI
        elif d < -PI: d += TWO_PI
        s += d * d
    return math.sqrt(s)


cpdef wrap_to_pi(x):
    return (x + np.pi) % (2.0 * np.pi) - np.pi


cpdef np.ndarray pose_error(np.ndarray T_cur, np.ndarray T_des):
    cdef double dpx = T_cur[0, 3] - T_des[0, 3]
    cdef double dpy = T_cur[1, 3] - T_des[1, 3]
    cdef double dpz = T_cur[2, 3] - T_des[2, 3]
    # Rotation error: 0.5 * sum of cross products of corresponding columns
    cdef double rex = 0.5 * (
        (T_cur[1, 0] * T_des[2, 0] - T_cur[2, 0] * T_des[1, 0])
        + (T_cur[1, 1] * T_des[2, 1] - T_cur[2, 1] * T_des[1, 1])
        + (T_cur[1, 2] * T_des[2, 2] - T_cur[2, 2] * T_des[1, 2])
    )
    cdef double rey = 0.5 * (
        (T_cur[2, 0] * T_des[0, 0] - T_cur[0, 0] * T_des[2, 0])
        + (T_cur[2, 1] * T_des[0, 1] - T_cur[0, 1] * T_des[2, 1])
        + (T_cur[2, 2] * T_des[0, 2] - T_cur[0, 2] * T_des[2, 2])
    )
    cdef double rez = 0.5 * (
        (T_cur[0, 0] * T_des[1, 0] - T_cur[1, 0] * T_des[0, 0])
        + (T_cur[0, 1] * T_des[1, 1] - T_cur[1, 1] * T_des[0, 1])
        + (T_cur[0, 2] * T_des[1, 2] - T_cur[1, 2] * T_des[0, 2])
    )
    return np.array([dpx, dpy, dpz, rex, rey, rez], dtype=float)


cdef inline np.ndarray _rotz(double t):
    cdef double c = math.cos(t)
    cdef double s = math.sin(t)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)


cdef inline np.ndarray _rotx(double t):
    cdef double c = math.cos(t)
    cdef double s = math.sin(t)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=float)


@dataclass
class NeroParams:
    # Keep this exactly as paper-style DH used for NERO experiments here.
    a_prev: np.ndarray
    alpha_prev: np.ndarray
    d_i: np.ndarray
    theta_offset: np.ndarray
    joint_limits: np.ndarray
    post_transform_d8: float
    theta0_coarse_divisor: int
    theta0_fine_count: int

    @staticmethod
    def default() -> "NeroParams":
        return NeroParams(
            a_prev=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=float),
            alpha_prev=np.deg2rad(
                np.array([0.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0], dtype=float)
            ),
            d_i=np.array([0.138, 0.0, 0.31, 0.0, 0.27, 0.0, 0.0235], dtype=float),
            theta_offset=np.deg2rad(
                np.array([0.0, -180.0, -180.0, -180.0, 90.0, 90.0, 0.0], dtype=float)
            ),
            joint_limits=np.array(
                [
                    [-2.705261, 2.705261],
                    [-1.745330, 1.745330],
                    [-2.757621, 2.757621],
                    [-1.012291, 2.146755],
                    [-2.757621, 2.757621],
                    [-0.733039, 0.959932],
                    [-1.570797, 1.570797],
                ],
                dtype=float,
            ),
            # Expose the solver end-effector as the URDF `link7` origin.
            # The last MDH link offset `d_i[6]` already models the link6->link7 distance.
            post_transform_d8=0.0,
            theta0_coarse_divisor=3,
            theta0_fine_count=41,
        )


@dataclass
class ContinuityParams:
    # Local theta0 tracking window around previous selected theta0.
    local_theta0_window: float = 0.35
    local_theta0_count: int = 41
    # Candidate score: w_vel*|dq| + w_acc*|ddq| + w_pose*pose_err + w_theta0*|dtheta0|
    w_vel: float = 1.0
    w_acc: float = 0.25
    w_pose: float = 0.1
    w_theta0: float = 0.15
    # Hysteresis: keep locked branch unless significantly better candidate appears.
    hysteresis_margin: float = 0.03
    # Initial global scan acquires the first branch; fallback controls later reacquire.
    enable_initial_global_search: bool = True
    enable_global_fallback: bool = True


@dataclass
class ContinuityRuntimeState:
    q_prev: np.ndarray
    q_prev2: Optional[np.ndarray] = None
    theta0_prev: Optional[float] = None
    q_lock: Optional[np.ndarray] = None


cpdef list fk_all(np.ndarray q, object p):
    cdef list Ts = [np.eye(4, dtype=float)]
    cdef double Tc[16], T_dh[16]
    cdef np.ndarray d_i = p.d_i
    cdef np.ndarray a_prev = p.a_prev
    cdef np.ndarray alpha_prev = p.alpha_prev
    cdef np.ndarray theta_offset = p.theta_offset
    cdef int i
    cdef double theta
    cdef np.ndarray T_np
    _mat4_eye(Tc)
    for i in range(7):
        theta = q[i] + theta_offset[i]
        _dh_A_into(theta, d_i[i], a_prev[i], alpha_prev[i], T_dh)
        _mat4_mul(Tc, T_dh, Tc)
        T_np = np.empty((4, 4), dtype=float)
        _c_to_np4(Tc, T_np)
        Ts.append(T_np)
    return Ts


cpdef np.ndarray fk(np.ndarray q, object p=None):
    if p is None:
        p = NeroParams.default()
    cdef double Tc[16], T_dh[16], T_post[16]
    cdef np.ndarray d_i = p.d_i
    cdef np.ndarray a_prev = p.a_prev
    cdef np.ndarray alpha_prev = p.alpha_prev
    cdef np.ndarray theta_offset = p.theta_offset
    cdef int i
    cdef double theta
    _mat4_eye(Tc)
    for i in range(7):
        theta = q[i] + theta_offset[i]
        _dh_A_into(theta, d_i[i], a_prev[i], alpha_prev[i], T_dh)
        _mat4_mul(Tc, T_dh, Tc)
    if p.post_transform_d8 != 0.0:
        _mat4_eye(T_post)
        T_post[11] = p.post_transform_d8
        _mat4_mul(Tc, T_post, Tc)
    cdef np.ndarray result = np.empty((4, 4), dtype=float)
    _c_to_np4(Tc, result)
    return result


cdef list _solve_q123_c(double Ex, double Ey, double Ez,
                         double Wx, double Wy, double Wz,
                         double q4, double d0, double d2, double d4):
    cdef list sols = []
    cdef double rho = math.hypot(Ex, Ey)
    cdef double c2 = (Ez - d0) / d2
    if c2 < -1.0 - 1e-8 or c2 > 1.0 + 1e-8:
        return []
    if c2 < -1.0: c2 = -1.0
    elif c2 > 1.0: c2 = 1.0
    cdef double s2_abs = math.sqrt(max(0.0, 1.0 - c2 * c2))
    if rho > abs(d2) + 1e-7:
        return []

    cdef double vx = Wx - Ex, vy = Wy - Ey, vz = Wz - Ez
    cdef double n_v = math.sqrt(vx*vx + vy*vy + vz*vz)
    if n_v < 1e-10:
        return []
    cdef double u1 = -vx / d4, u2 = -vy / d4, u3 = -vz / d4
    cdef double n_col = math.sqrt(u1*u1 + u2*u2 + u3*u3)
    if n_col < 1e-12: n_col = 1e-12
    u1 /= n_col; u2 /= n_col; u3 /= n_col

    cdef double s4 = math.sin(q4), c4 = math.cos(q4)
    if abs(s4) < 1e-8:
        return []

    cdef double s2, c1, s1, n1, q1, q2, b1, b2, s3, c2c3, c3, n3
    for s2 in (s2_abs, -s2_abs):
        if abs(s2) < 1e-10:
            continue
        c1 = -Ex / (d2 * s2)
        s1 = -Ey / (d2 * s2)
        n1 = math.hypot(c1, s1)
        if n1 < 1e-12:
            continue
        c1 /= n1; s1 /= n1
        q1 = math.atan2(s1, c1)
        q2 = math.atan2(s2, c2)
        b1 = (s2 * c1 * c4 - u1) / s4
        b2 = (u2 - s1 * s2 * c4) / s4
        s3 = s1 * b1 + c1 * b2
        c2c3 = -c1 * b1 + s1 * b2
        if abs(c2) > 1e-8:
            c3 = c2c3 / c2
        else:
            c3 = (u3 + c2 * c4) / (s2 * s4)
        n3 = math.hypot(s3, c3)
        if n3 < 1e-12:
            continue
        s3 /= n3; c3 /= n3
        sols.append((q1, q2, math.atan2(s3, c3)))
    return sols


cdef list _extract_567_c(double* T47):
    cdef list sols = []
    cdef double c6 = T47[6]
    cdef double s6, th5, th6, th7
    if c6 < -1.0: c6 = -1.0
    elif c6 > 1.0: c6 = 1.0
    cdef double sgn
    for sgn in (1.0, -1.0):
        s6 = sgn * math.sqrt(max(0.0, 1.0 - c6 * c6))
        # 腕部奇异保护: q6≈0 时 s6≈0 (球形腕两轴重合, q5/q7 不可分辨)。
        # 不跳过该分支，而是钳制 s6 到小值，保留近似解。
        # 打分阶段的 pose_cost 会自然惩罚不精确的解。
        if abs(s6) < 1e-4:
            s6 = sgn * 1e-4
        th6 = math.atan2(s6, c6)
        th5 = math.atan2(T47[10] / s6, T47[2] / s6)
        th7 = math.atan2(T47[5] / s6, -T47[4] / s6)
        sols.append((th5, th6, th7))
    return sols


cdef _solve_theta4_from_triangle(np.ndarray S, np.ndarray W, object p):
    # Paper geometric triangle SEW
    cdef double dx = W[0] - S[0], dy = W[1] - S[1], dz = W[2] - S[2]
    cdef double l_sw = math.sqrt(dx * dx + dy * dy + dz * dz)
    cdef double l_se = abs(p.d_i[2])
    cdef double l_ew = abs(p.d_i[4])
    if l_sw < 1e-10:
        return None
    cdef double c4 = (l_sw * l_sw - l_se * l_se - l_ew * l_ew) / (2.0 * l_se * l_ew)
    if c4 < -1.0 or c4 > 1.0:
        return None
    if c4 < -1.0:
        c4 = -1.0
    elif c4 > 1.0:
        c4 = 1.0
    return math.acos(c4)


cdef tuple _compute_swe_from_target(np.ndarray T07, object p):
    """
    Build S-W-E triangle primitives from target pose:
      - S: shoulder center
      - W: wrist center
      - q4_abs from triangle law of cosines
      - u_sw: unit vector from S to W
    """
    cdef double d6 = p.d_i[6]
    cdef double d1 = p.d_i[0]
    cdef double d8 = p.post_transform_d8

    # z7 = R[:, 2] = third column of rotation
    cdef double z7x = T07[0, 2], z7y = T07[1, 2], z7z = T07[2, 2]
    # O7 = p_target - d8 * z7
    cdef double O7x = T07[0, 3] - d8 * z7x
    cdef double O7y = T07[1, 3] - d8 * z7y
    cdef double O7z = T07[2, 3] - d8 * z7z
    # W = O7 - d6 * z7
    cdef np.ndarray W = np.array([O7x - d6 * z7x, O7y - d6 * z7y, O7z - d6 * z7z], dtype=float)
    cdef np.ndarray S = np.array([0.0, 0.0, d1], dtype=float)

    q4_abs = _solve_theta4_from_triangle(S, W, p)
    cdef double vswx = W[0] - S[0], vswy = W[1] - S[1], vswz = W[2] - S[2]
    cdef double n_sw = math.sqrt(vswx * vswx + vswy * vswy + vswz * vswz)
    cdef np.ndarray u_sw
    if n_sw < 1e-12:
        u_sw = np.array([0.0, 0.0, 1.0], dtype=float)
    else:
        u_sw = np.array([vswx / n_sw, vswy / n_sw, vswz / n_sw], dtype=float)
    return S, W, q4_abs, u_sw


def _weight_limits(q: float, q_min: float, q_max: float) -> float:
    """清华论文式(20)：越靠近限位，权重越大。"""
    span = q_max - q_min
    if span < 1e-12:
        return 1.0
    x = 2.0 * (q - (q_min + q_max) * 0.5) / span
    a = 2.38
    b = 2.28
    if x >= 0:
        if x >= 1.0:
            return 1e6
        den = math.exp(a * (1.0 - x)) - 1.0
        if abs(den) < 1e-12:
            return 1e6
        return b * x / den
    if x <= -1.0:
        return 1e6
    den = math.exp(a * (1.0 + x)) - 1.0
    if abs(den) < 1e-12:
        return 1e6
    return -b * x / den


def _scan_theta0_solutions(
    T07: np.ndarray, p: NeroParams, step: float
) -> Tuple[List[float], List[Tuple[float, List[np.ndarray]]]]:
    """
    One-pass theta0 sweep:
      - returns feasible theta0 list
      - and cached per-theta solutions to avoid recomputing _ik_one_arm_angle
    """
    feasible_theta0: List[float] = []
    cached: List[Tuple[float, List[np.ndarray]]] = []
    for theta0 in np.arange(-math.pi, math.pi, step):
        t = float(theta0)
        sols = _ik_one_arm_angle(T07, t, p)
        if sols:
            feasible_theta0.append(t)
            cached.append((t, sols))
    return feasible_theta0, cached


def _best_weighted_from_cached(
    cached: List[Tuple[float, List[np.ndarray]]], p: NeroParams, q_prev: np.ndarray
) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[float]]:
    """
    Select global best candidate over cached theta0 branches with original weighted rule.
    Returns (q_best_7d, q_best_full, theta0_selected).
    """
    best_cost = float("inf")
    q_best: Optional[np.ndarray] = None
    q_best_full: Optional[np.ndarray] = None
    theta0_best: Optional[float] = None
    for t, sols in cached:
        for q_full in sols:
            q = q_full[:7]
            cost = 0.0
            for i in range(7):
                lo, hi = float(p.joint_limits[i, 0]), float(p.joint_limits[i, 1])
                w = _weight_limits(q[i], lo, hi)
                dq = abs(q[i] - q_prev[i])
                cost += w * dq * dq
            if cost < best_cost:
                best_cost = cost
                q_best = q.copy()
                q_best_full = q_full
                theta0_best = t
    return q_best, q_best_full, theta0_best


cpdef list _ik_one_arm_angle(np.ndarray T07_np, double theta0, object p):
    cdef double T07[16], T_chain[16], T04[16], T_dh[16], T_inv[16], T47[16]
    _np4_to_c(T07_np, T07)

    # Extract params once
    cdef double d0 = p.d_i[0], d2 = p.d_i[2], d4 = p.d_i[4]
    cdef double d6 = p.d_i[6], d8 = p.post_transform_d8
    cdef np.ndarray theta_offset_np = p.theta_offset
    cdef np.ndarray d_i = p.d_i
    cdef np.ndarray a_prev = p.a_prev
    cdef np.ndarray alpha_prev = p.alpha_prev
    cdef np.ndarray joint_limits = p.joint_limits
    cdef double to0=theta_offset_np[0], to1=theta_offset_np[1], to2=theta_offset_np[2]
    cdef double to3=theta_offset_np[3], to4=theta_offset_np[4], to5=theta_offset_np[5], to6=theta_offset_np[6]

    # S, W, q4_abs from C-array T07 (no numpy allocation)
    cdef double z7x=T07[2], z7y=T07[6], z7z=T07[10]
    cdef double O7x=T07[3]-d8*z7x, O7y=T07[7]-d8*z7y, O7z=T07[11]-d8*z7z
    cdef double Wx=O7x-d6*z7x, Wy=O7y-d6*z7y, Wz=O7z-d6*z7z
    cdef double Sx=0.0, Sy=0.0, Sz=d0

    cdef double dx=Wx-Sx, dy=Wy-Sy, dz=Wz-Sz
    cdef double l_sw=math.sqrt(dx*dx+dy*dy+dz*dz)
    cdef double l_se=abs(d2), l_ew=abs(d4)
    if l_sw < 1e-10:
        return []
    cdef double c4v=(l_sw*l_sw-l_se*l_se-l_ew*l_ew)/(2.0*l_se*l_ew)
    if c4v < -1.0 or c4v > 1.0:
        return []
    if c4v < -1.0: c4v = -1.0
    elif c4v > 1.0: c4v = 1.0
    cdef double q4_abs=math.acos(c4v)

    # Elbow E from theta0 (scalar, no numpy)
    cdef double usx=dx/l_sw, usy=dy/l_sw, usz=dz/l_sw
    cdef double xv=(l_se*l_se-l_ew*l_ew+l_sw*l_sw)/(2.0*l_sw)
    cdef double r2v=l_se*l_se-xv*xv
    if r2v < -1e-10:
        return []
    cdef double rv=math.sqrt(max(0.0, r2v))
    cdef double Cx=Sx+xv*usx, Cy=Sy+xv*usy, Cz=Sz+xv*usz
    cdef double tx=Sy*usz-Sz*usy, ty=Sz*usx-Sx*usz, tz=Sx*usy-Sy*usx
    cdef double tn=math.sqrt(tx*tx+ty*ty+tz*tz)
    if tn < 1e-10:
        tx=usz; ty=0.0; tz=-usx; tn=math.sqrt(tx*tx+tz*tz)
    if tn < 1e-10:
        tx=-usz; ty=0.0; tz=usx; tn=math.sqrt(tx*tx+tz*tz)
    if tn < 1e-12: tn=1e-12
    cdef double e1x=tx/tn, e1y=ty/tn, e1z=tz/tn
    cdef double e2x=usy*e1z-usz*e1y, e2y=usz*e1x-usx*e1z, e2z=usx*e1y-usy*e1x
    cdef double e2n=math.sqrt(e2x*e2x+e2y*e2y+e2z*e2z)
    if e2n < 1e-12: e2n=1e-12
    e2x/=e2n; e2y/=e2n; e2z/=e2n
    cdef double ct=math.cos(theta0), st=math.sin(theta0)
    cdef double Ex=Cx+rv*(ct*e1x+st*e2x)
    cdef double Ey=Cy+rv*(ct*e1y+st*e2y)
    cdef double Ez=Cz+rv*(ct*e1z+st*e2z)

    # T_chain = remove_post_transform(T07) in C
    cdef int i
    for i in range(16):
        T_chain[i] = T07[i]
    T_chain[3]  -= d8 * T_chain[2]
    T_chain[7]  -= d8 * T_chain[6]
    T_chain[11] -= d8 * T_chain[10]

    cdef list q_solutions = []
    cdef double q4, th1, th2, th3, th4, th5, th6, th7
    cdef list q123_sols, th567_sols
    cdef tuple q123, th567
    cdef bint ok
    cdef int j
    cdef np.ndarray q_np, extra_np

    for q4 in (q4_abs, -q4_abs):
        q123_sols = _solve_q123_c(Ex, Ey, Ez, Wx, Wy, Wz, q4, d0, d2, d4)
        for q123 in q123_sols:
            th1 = q123[0] + to0
            th2 = q123[1] + to1
            th3 = q123[2] + to2
            th4 = q4 + to3

            _dh_A_into(th1, d_i[0], a_prev[0], alpha_prev[0], T04)
            _dh_A_into(th2, d_i[1], a_prev[1], alpha_prev[1], T_dh)
            _mat4_mul(T04, T_dh, T04)
            _dh_A_into(th3, d_i[2], a_prev[2], alpha_prev[2], T_dh)
            _mat4_mul(T04, T_dh, T04)
            _dh_A_into(th4, d_i[3], a_prev[3], alpha_prev[3], T_dh)
            _mat4_mul(T04, T_dh, T04)

            _invert_rigid_into(T04, T_inv)
            _mat4_mul(T_inv, T_chain, T47)

            for th567 in _extract_567_c(T47):
                th5 = th567[0]; th6 = th567[1]; th7 = th567[2]
                q_np = np.array([
                    _wrap_to_pi_scalar(th1 - to0),
                    _wrap_to_pi_scalar(th2 - to1),
                    _wrap_to_pi_scalar(th3 - to2),
                    _wrap_to_pi_scalar(th4 - to3),
                    _wrap_to_pi_scalar(th5 - to4),
                    _wrap_to_pi_scalar(th6 - to5),
                    _wrap_to_pi_scalar(th7 - to6),
                ], dtype=float)
                ok = True
                for j in range(7):
                    if q_np[j] < joint_limits[j, 0] - 1e-8 or q_np[j] > joint_limits[j, 1] + 1e-8:
                        ok = False
                        break
                if not ok:
                    continue
                extra_np = np.array([theta0, Sx, Sy, Sz, Wx, Wy, Wz, Ex, Ey, Ez], dtype=float)
                q_solutions.append(np.concatenate([q_np, extra_np]))
    return q_solutions


def _collect_unique_solutions_for_theta0_grid(
    T: np.ndarray, p: NeroParams, theta0_grid: np.ndarray
) -> List[np.ndarray]:
    all_solutions: List[np.ndarray] = []
    cdef double t
    for theta0 in theta0_grid:
        t = _wrap_to_pi_scalar(float(theta0))
        all_solutions.extend(_ik_one_arm_angle(T, t, p))
    unique_solutions: List[np.ndarray] = []
    for q in all_solutions:
        if not _is_duplicate(q, unique_solutions):
            unique_solutions.append(q)
    return unique_solutions


def ik_arm_angle_with_report(
    T_target: np.ndarray,
    p: Optional[NeroParams] = None,
    q_prev: Optional[np.ndarray] = None,
    n_psi: int = 181,
) -> Tuple[Optional[np.ndarray], List[np.ndarray], Dict[str, object]]:
    if p is None:
        p = NeroParams.default()
    if q_prev is None:
        q_prev = np.zeros(7, dtype=float)

    T = np.array(T_target, dtype=float)
    S_dbg, W_dbg, q4_dbg, _ = _compute_swe_from_target(T, p)
    step = min(0.01, 2.0 * math.pi / max(1, n_psi))
    feasible_theta0, cached = _scan_theta0_solutions(T, p, step=step)

    if len(feasible_theta0) == 0:
        return (
            None,
            [],
            {
                "method": "failed",
                "candidate_count": 0,
                "pose_err_best": None,
                "swe_debug": {
                    "S": S_dbg.tolist(),
                    "W": W_dbg.tolist(),
                    "E": [float("nan"), float("nan"), float("nan")],
                    "q4_abs": None if q4_dbg is None else float(q4_dbg),
                },
            },
        )

    q_best, q_best_full, best_t0 = _best_weighted_from_cached(cached, p, q_prev)

    if q_best is None:
        return (
            None,
            [],
            {
                "method": "failed",
                "candidate_count": 0,
                "pose_err_best": None,
                "swe_debug": {
                    "S": S_dbg.tolist(),
                    "W": W_dbg.tolist(),
                    "E": [float("nan"), float("nan"), float("nan")],
                    "q4_abs": None if q4_dbg is None else float(q4_dbg),
                },
            },
        )

    # 仅保留最优解一条分支，便于与原文接口一致；候选数用可行臂角个数表征
    if q_best_full is None or best_t0 is None:
        q_best_full = np.concatenate(
            [
                q_best,
                np.array(
                    [
                        0.0,
                        S_dbg[0],
                        S_dbg[1],
                        S_dbg[2],
                        W_dbg[0],
                        W_dbg[1],
                        W_dbg[2],
                        float("nan"),
                        float("nan"),
                        float("nan"),
                    ],
                    dtype=float,
                ),
            ]
        )
    theta0_best = float(q_best_full[7] if best_t0 is None else best_t0)
    swe_debug = {
        "S": q_best_full[8:11].tolist(),
        "W": q_best_full[11:14].tolist(),
        "E": q_best_full[14:17].tolist(),
        "q4_abs": None if q4_dbg is None else float(q4_dbg),
    }
    e_best = _pose_error_norm(fk(q_best, p), T)
    all_solutions = [q_best_full]
    return (
        q_best,
        all_solutions,
        {
            "method": "feasible_region",
            "candidate_count": len(feasible_theta0),
            "pose_err_best": e_best,
            "theta0_selected": theta0_best,
            "feasible_theta0_count": len(feasible_theta0),
            "swe_debug": swe_debug,
        },
    )


def solve_pose_continuous_with_state(
    T_target: np.ndarray,
    state: ContinuityRuntimeState,
    p: Optional[NeroParams] = None,
    n_psi: int = 181,
    continuity: Optional[ContinuityParams] = None,
) -> Tuple[Optional[np.ndarray], Dict[str, object], ContinuityRuntimeState]:
    """
    Single-step continuous IK with external runtime state.
    This is equivalent to one iteration of solve_trajectory_continuous.
    """
    if p is None:
        p = NeroParams.default()
    if continuity is None:
        continuity = ContinuityParams()

    T = np.array(T_target, dtype=float)
    q_prev = state.q_prev
    q_prev2 = state.q_prev2
    theta0_prev = state.theta0_prev
    q_lock = state.q_lock

    all_solutions: List[np.ndarray] = []
    method = "continuous_local_theta0"
    if theta0_prev is not None:
        theta0_grid = np.linspace(
            theta0_prev - continuity.local_theta0_window,
            theta0_prev + continuity.local_theta0_window,
            max(11, continuity.local_theta0_count),
            endpoint=True,
        )
        all_solutions = _collect_unique_solutions_for_theta0_grid(T, p, theta0_grid)

    allow_global = (theta0_prev is None and continuity.enable_initial_global_search) or (theta0_prev is not None and continuity.enable_global_fallback)

    if not all_solutions and allow_global:
        q_global, all_full, _ = ik_arm_angle_with_report(
            T, p=p, q_prev=q_prev, n_psi=n_psi
        )
        all_solutions = all_full
        method = "continuous_global_fallback"
        if q_global is None:
            return (
                None,
                {
                    "method": method,
                    "candidate_count": 0,
                    "selected_by": "failed",
                    "pose_err_best": None,
                },
                state,
            )
    elif not all_solutions:
        return (
            None,
            {
                "method": method,
                "candidate_count": 0,
                "selected_by": "failed",
                "pose_err_best": None,
            },
            state,
        )

    scored = []
    for cand in all_solutions:
        q = cand[:7]
        theta0 = float(cand[7]) if cand.shape[0] > 7 else 0.0
        vel_cost = _wrap_diff_norm7(q, q_prev)
        if q_prev2 is not None:
            acc_cost = _wrap_diff_norm7_2prev(q, q_prev, q_prev2)
        else:
            acc_cost = 0.0
        pose_cost = _pose_error_norm(fk(q, p), T)
        if theta0_prev is None:
            theta0_cost = 0.0
        else:
            theta0_cost = abs(_wrap_to_pi_scalar(theta0 - theta0_prev))
        score = (
            continuity.w_vel * vel_cost
            + continuity.w_acc * acc_cost
            + continuity.w_pose * pose_cost
            + continuity.w_theta0 * theta0_cost
        )
        scored.append((score, vel_cost, acc_cost, pose_cost, theta0_cost, cand))

    scored.sort(key=lambda x: x[0])
    best = scored[0]

    selected = best
    selected_by = "best_score"
    if q_lock is not None:
        lock_item = min(
            scored, key=lambda x: _dist7(x[5][:7], q_lock)
        )
        if lock_item[0] <= best[0] + continuity.hysteresis_margin:
            selected = lock_item
            selected_by = "hysteresis_locked"

    q_best_full = selected[5]
    q_best = q_best_full[:7]
    theta0_best = float(q_best_full[7]) if q_best_full.shape[0] > 7 else None

    pose_best = _pose_error_norm(fk(q_best, p), T)

    next_state = ContinuityRuntimeState(
        q_prev=q_best,
        q_prev2=q_prev.copy(),
        theta0_prev=theta0_best if theta0_best is not None else theta0_prev,
        q_lock=q_best,
    )
    report = {
        "method": method,
        "candidate_count": len(all_solutions),
        "selected_by": selected_by,
        "score_best": float(selected[0]),
        "vel_cost_best": float(selected[1]),
        "acc_cost_best": float(selected[2]),
        "pose_cost_best": float(selected[3]),
        "theta0_cost_best": float(selected[4]),
        "pose_err_best": pose_best,
        "theta0_selected": theta0_best,
    }
    return q_best, report, next_state
