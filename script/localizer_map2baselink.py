#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
import numpy as np
import open3d as o3d

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

from collections import deque


# ===========================
# SE(3) helpers
# ===========================
def pose_to_T(p):
    """geometry_msgs/Pose -> 4x4"""
    qx, qy, qz, qw = p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w
    tx, ty, tz = p.position.x, p.position.y, p.position.z

    x2, y2, z2 = qx + qx, qy + qy, qz + qz
    xx, yy, zz = qx * x2, qy * y2, qz * z2
    xy, xz, yz = qx * y2, qx * z2, qy * z2
    wx, wy, wz = qw * x2, qw * y2, qw * z2

    R = np.array([
        [1.0 - (yy + zz),       (xy - wz),       (xz + wy)],
        [      (xy + wz), 1.0 - (xx + zz),       (yz - wx)],
        [      (xz - wy),       (yz + wx), 1.0 - (xx + yy)],
    ], dtype=np.float64)

    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = [tx, ty, tz]
    return T


def cloud2_to_xyz_np(msg: PointCloud2):
    pts = []
    for p in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        pts.append([p[0], p[1], p[2]])
    if len(pts) == 0:
        return np.zeros((0, 3), dtype=np.float64)
    return np.asarray(pts, dtype=np.float64)


def o3d_to_ros_cloud(pcd: o3d.geometry.PointCloud, frame_id: str, stamp):
    pts = np.asarray(pcd.points, dtype=np.float32)
    header = Header()
    header.frame_id = frame_id
    header.stamp = stamp
    return point_cloud2.create_cloud_xyz32(header, pts)


def transform_pcd(pcd: o3d.geometry.PointCloud, T: np.ndarray):
    out = o3d.geometry.PointCloud(pcd)
    out.transform(T)
    return out


# ===========================
# Rear LiDAR + Roll/Pitch clamp
# ===========================
def T_base_lidar_rear():
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = np.array([
        [-1.0,  0.0,  0.0],
        [ 0.0, -1.0,  0.0],
        [ 0.0,  0.0,  1.0],
    ], dtype=np.float64)
    return T


def _rot_to_rpy_zyx(R):
    v = -float(R[2, 0])
    v = max(min(v, 1.0), -1.0)
    pitch = math.asin(v)
    cp = math.cos(pitch)

    if abs(cp) < 1e-9:
        roll = 0.0
        yaw = math.atan2(-float(R[0, 1]), float(R[1, 1]))
    else:
        roll = math.atan2(float(R[2, 1]), float(R[2, 2]))
        yaw = math.atan2(float(R[1, 0]), float(R[0, 0]))
    return roll, pitch, yaw


def _rpy_to_rot_zyx(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ], dtype=np.float64)


def clamp_roll_pitch_in_T(T, max_deg=5.0):
    R = T[:3, :3]
    roll, pitch, yaw = _rot_to_rpy_zyx(R)

    lim = max_deg * math.pi / 180.0
    roll = max(min(roll, lim), -lim)
    pitch = max(min(pitch, lim), -lim)

    Tout = T.copy()
    Tout[:3, :3] = _rpy_to_rot_zyx(roll, pitch, yaw)
    return Tout


# ===========================
# Rotation matrix to quaternion (xyzw)
# ===========================
def rot_to_quat_xyzw(R):
    """rotation matrix -> quat (x,y,z,w)"""
    tr = float(np.trace(R))
    if tr > 0.0:
        S = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S

    q = np.array([qx, qy, qz, qw], dtype=np.float64)
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    return q / n


# ===========================
# ✅ Scheme-A: SE(3) consensus + center estimate
# ===========================
def _rot_angle_deg(R):
    c = (float(np.trace(R)) - 1.0) * 0.5
    c = max(min(c, 1.0), -1.0)
    return math.degrees(math.acos(c))


def _rel_rot_angle_deg(Ra, Rb):
    return _rot_angle_deg(Ra.T @ Rb)


def _rot_to_quat_xyzw(R):
    """rotation matrix -> quat (x,y,z,w), stable enough for averaging"""
    tr = float(np.trace(R))
    if tr > 0.0:
        S = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S

    q = np.array([qx, qy, qz, qw], dtype=np.float64)
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    return q / n


def _quat_xyzw_to_rot(q):
    x, y, z, w = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z

    return np.array([
        [1.0 - 2.0 * (yy + zz),       2.0 * (xy - wz),       2.0 * (xz + wy)],
        [      2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz),       2.0 * (yz - wx)],
        [      2.0 * (xz - wy),       2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ], dtype=np.float64)


def _se3_mean(Ts):
    """engineering mean: t-avg + quat-avg (hemisphere fix)"""
    if len(Ts) == 0:
        return None

    t_mean = np.mean([T[:3, 3] for T in Ts], axis=0)

    Q = []
    for T in Ts:
        q = _rot_to_quat_xyzw(T[:3, :3])
        # hemisphere fix
        if q[3] < 0.0:
            q = -q
        Q.append(q)
    q_mean = np.mean(Q, axis=0)
    n = np.linalg.norm(q_mean)
    if n < 1e-12:
        q_mean = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    else:
        q_mean = q_mean / n

    Tm = np.eye(4, dtype=np.float64)
    Tm[:3, 3] = t_mean
    Tm[:3, :3] = _quat_xyzw_to_rot(q_mean)
    return Tm


def _se3_close(Ta, Tb, t_thresh_m, r_thresh_deg):
    dt = float(np.linalg.norm(Ta[:3, 3] - Tb[:3, 3]))
    dr = float(_rel_rot_angle_deg(Ta[:3, :3], Tb[:3, :3]))
    return (dt <= t_thresh_m) and (dr <= r_thresh_deg), dt, dr


class FPFHRelocalizer(Node):
    def __init__(self):
        super().__init__("fpfh_map_corrector_node")

        # ============================================================
        # ✅ 新增：ICP 最后一层 rmse 过大则作废（不改参数接口）
        # ============================================================
        self.ICP_ABORT_RMSE = 0.60

        # ============================================================
        # ✅ 新增：Scheme-A 一致性（不改参数接口，全常量）
        # ============================================================
        self.CONS_WIN_N = 30
        self.CONS_K = 3
        self.CONS_T_THRESH_M = 0.30
        self.CONS_R_THRESH_DEG = 10.0

        self.REF_MAX = 10
        self.OUTLIER_T_THRESH_M = 1.00
        self.OUTLIER_R_THRESH_DEG = 30.0

        self._T_window = deque(maxlen=self.CONS_WIN_N)  # recent T_final
        self._ref_Ts = deque(maxlen=self.REF_MAX)       # recent consensus centers

        # ============================================================
        # ✅ 新增：static tf 广播器（初次定位成功后发布 map->lidar）
        # ============================================================
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self._localization_done = False  # 只发一次

        # ========== 参数 ==========
        self.declare_parameter("target_pcd", "accumulated_map_ds.pcd")
        self.declare_parameter("source_frame", "lidar")

        self.declare_parameter("source_cloud_topic", "/fastlio2/world_cloud")
        self.declare_parameter("odom_topic", "/fastlio2/lio_odom")

        self.declare_parameter("pub_corrected_map_topic", "/corrected_map_cloud")
        self.declare_parameter("publish_corrected_map", True)

        self.declare_parameter("max_range_m", 20.0)

        # FPFH / RANSAC
        self.declare_parameter("voxel_global", 0.25)
        self.declare_parameter("normal_radius_factor", 2.5)
        self.declare_parameter("fpfh_radius_factor", 5.0)

        self.declare_parameter("ransac_corr_dist", 1.5)
        self.declare_parameter("ransac_max_iter", 200000)
        self.declare_parameter("ransac_confidence", 0.999)

        # ICP refine
        self.declare_parameter("enable_icp_refine", True)
        self.declare_parameter("icp_voxels", [0.30, 0.15, 0.08, 0.04])
        self.declare_parameter("icp_max_corr", [1.00, 0.50, 0.25, 0.10])
        self.declare_parameter("icp_max_iter", [200, 400, 800, 1200])

        self.declare_parameter("corrected_map_voxel", 0.10)
        self.declare_parameter("min_interval_sec", 0.3)

        self._max_rp_deg = 5.0

        # ========== 读参数 ==========
        self.target_pcd_path = self.get_parameter("target_pcd").value
        self.source_frame = self.get_parameter("source_frame").value

        self.max_range_m = float(self.get_parameter("max_range_m").value)
        self.max_range_sq = self.max_range_m ** 2

        self.voxel_global = float(self.get_parameter("voxel_global").value)
        self.normal_factor = float(self.get_parameter("normal_radius_factor").value)
        self.fpfh_factor = float(self.get_parameter("fpfh_radius_factor").value)

        self.ransac_corr = float(self.get_parameter("ransac_corr_dist").value)
        self.ransac_max_iter = int(self.get_parameter("ransac_max_iter").value)
        self.ransac_conf = float(self.get_parameter("ransac_confidence").value)

        self.enable_icp = bool(self.get_parameter("enable_icp_refine").value)
        self.icp_voxels = list(self.get_parameter("icp_voxels").value)
        self.icp_maxcorr = list(self.get_parameter("icp_max_corr").value)
        self.icp_maxiter = list(self.get_parameter("icp_max_iter").value)

        self.corrected_map_voxel = float(self.get_parameter("corrected_map_voxel").value)
        self.min_interval = float(self.get_parameter("min_interval_sec").value)
        self.publish_corrected_map = bool(self.get_parameter("publish_corrected_map").value)

        # ========== QoS ==========
        qos_sub = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        qos_pub = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.sub_cloud = self.create_subscription(
            PointCloud2, self.get_parameter("source_cloud_topic").value, self.on_cloud, qos_sub
        )
        self.sub_odom = self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self.on_odom, qos_sub
        )

        self.pub_map = self.create_publisher(
            PointCloud2, self.get_parameter("pub_corrected_map_topic").value, qos_pub
        )

        self.last_odom_pose = None
        self.last_run_time = 0.0

        self._accum_frames = 0
        self._accum_xyz_chunks = []
        self._exec_every_n_frames = 30
        self._accum_downsample_stride = 20

        self.get_logger().info(f"[INIT] load target map: {self.target_pcd_path}")
        self.tgt_full = o3d.io.read_point_cloud(self.target_pcd_path)
        self.tgt_down = self._down_and_normals(self.tgt_full, self.voxel_global)
        self.tgt_fpfh = self._fpfh(self.tgt_down, self.voxel_global)
        self.tgt_voxel = self.tgt_full.voxel_down_sample(self.corrected_map_voxel)

    def _down_and_normals(self, pcd, voxel):
        p = pcd.voxel_down_sample(voxel)
        r = max(self.normal_factor * voxel, 0.10)
        p.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=r, max_nn=80))
        return p

    def _fpfh(self, pcd_down, voxel):
        r = max(self.fpfh_factor * voxel, 0.25)
        return o3d.pipelines.registration.compute_fpfh_feature(
            pcd_down, o3d.geometry.KDTreeSearchParamHybrid(radius=r, max_nn=120)
        )

    def on_odom(self, msg: Odometry):
        self.last_odom_pose = msg.pose.pose

    def _publish_static_tf(self, T_final, stamp):
        """发布 map -> lidar 的 static tf，只执行一次"""
        if self._localization_done:
            return

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "map"
        t.child_frame_id = self.source_frame  # "lidar"

        # T_final 是 lidar -> map 的变换，所以 lidar 原点在 map 下的位置就是 T_final 的平移部分
        t.transform.translation.x = float(T_final[0, 3])
        t.transform.translation.y = float(T_final[1, 3])
        t.transform.translation.z = float(T_final[2, 3])

        # 旋转部分转四元数
        q = rot_to_quat_xyzw(T_final[:3, :3])
        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])

        self.tf_static_broadcaster.sendTransform(t)
        self._localization_done = True

        self.get_logger().info(
            f"[STATIC_TF] Published map->lidar: t=({t.transform.translation.x:.2f}, "
            f"{t.transform.translation.y:.2f}, {t.transform.translation.z:.2f})"
        )

    def _schemeA_update_and_get_ref(self, T_final):
        """
        1) 把 T_final 推入窗口
        2) 找是否存在近邻数>=K 的"中心"
        3) 若触发：返回 consensus_T（均值），并写入 ref 池
        否则返回 None
        """
        self._T_window.append(T_final)

        # 窗口太小也允许（你要的：不用等满30次）
        Ts = list(self._T_window)
        best_neighbors = None

        for Ti in Ts:
            neigh = []
            for Tj in Ts:
                ok, _, _ = _se3_close(Ti, Tj, self.CONS_T_THRESH_M, self.CONS_R_THRESH_DEG)
                if ok:
                    neigh.append(Tj)

            if len(neigh) >= self.CONS_K:
                best_neighbors = neigh
                break

        if best_neighbors is None:
            return None

        T_ref = _se3_mean(best_neighbors)
        if T_ref is None:
            return None

        # 保存参考真值（最多10个，自动丢最旧）
        self._ref_Ts.append(T_ref)
        self.get_logger().info(
            f"[CONS] consensus hit: size={len(best_neighbors)} | ref_pool={len(self._ref_Ts)}"
        )
        return T_ref

    def on_cloud(self, msg: PointCloud2):
        if self.last_odom_pose is None:
            return

        xyz = cloud2_to_xyz_np(msg)
        xyz = xyz[np.sum(xyz * xyz, axis=1) <= self.max_range_sq]
        if xyz.shape[0] < 200:
            return

        self._accum_xyz_chunks.append(xyz)
        self._accum_frames += 1
        if self._accum_frames < self._exec_every_n_frames:
            return

        xyz = np.vstack(self._accum_xyz_chunks)[::self._accum_downsample_stride]
        self._accum_xyz_chunks.clear()
        self._accum_frames = 0

        if (self.get_clock().now().nanoseconds * 1e-9 - self.last_run_time) < self.min_interval:
            return
        self.last_run_time = self.get_clock().now().nanoseconds * 1e-9

        src = o3d.geometry.PointCloud()
        src.points = o3d.utility.Vector3dVector(xyz)

        T_guess = pose_to_T(self.last_odom_pose) @ T_base_lidar_rear()
        src_guess = o3d.geometry.PointCloud(src)
        src_guess.transform(T_guess)

        src_down = self._down_and_normals(src_guess, self.voxel_global)
        src_fpfh = self._fpfh(src_down, self.voxel_global)

        ransac = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            src_down, self.tgt_down,
            src_fpfh, self.tgt_fpfh,
            True, self.ransac_corr,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
            4,
            [
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(self.ransac_corr),
            ],
            o3d.pipelines.registration.RANSACConvergenceCriteria(
                self.ransac_max_iter, self.ransac_conf
            )
        )

        T_corr = clamp_roll_pitch_in_T(ransac.transformation, self._max_rp_deg)
        T_final = T_corr @ T_guess

        if self.enable_icp:
            T_icp, last_rmse, last_fit = self._icp_refine(src_guess, self.tgt_full, T_corr)

            # ============================================================
            # ✅ ICP 最后一层误差过大则作废（保持你原逻辑）
            # ============================================================
            if (last_rmse is not None) and (last_rmse > self.ICP_ABORT_RMSE):
                self.get_logger().warn(
                    f"[DROP] ICP rmse too large: {last_rmse:.3f} > {self.ICP_ABORT_RMSE:.3f} "
                    f"(fitness={last_fit:.3f})"
                )
                return

            T_icp = clamp_roll_pitch_in_T(T_icp, self._max_rp_deg)
            T_final = T_icp @ T_guess

        # ============================================================
        # ✅ Scheme-A：若已有参考真值，偏离太大直接作废（不发布）
        # ============================================================
        if len(self._ref_Ts) > 0:
            T_ref_latest = self._ref_Ts[-1]
            ok, dt, dr = _se3_close(T_final, T_ref_latest, self.OUTLIER_T_THRESH_M, self.OUTLIER_R_THRESH_DEG)
            if not ok:
                self.get_logger().warn(
                    f"[DROP] outlier vs ref: dt={dt:.3f}m dr={dr:.2f}deg "
                    f"(th={self.OUTLIER_T_THRESH_M:.2f}m/{self.OUTLIER_R_THRESH_DEG:.1f}deg)"
                )
                return

        # ============================================================
        # ✅ Scheme-A：窗口一致性 + 中心估计（触发就用中心作为参考真值）
        # ============================================================
        T_cons = self._schemeA_update_and_get_ref(T_final)
        if T_cons is not None:
            T_final = T_cons  # 触发就用"中心估计"替代当前解（你要的：参考真值）

        # ============================================================
        # ✅ 新增：发布 static tf map->lidar（初次成功后只发一次）
        # ============================================================
        self._publish_static_tf(T_final, msg.header.stamp)

        if not self.publish_corrected_map:
            return

        T_inv = np.linalg.inv(T_final)
        corrected = transform_pcd(self.tgt_voxel, T_inv)
        corrected = corrected.voxel_down_sample(self.corrected_map_voxel)
        self.pub_map.publish(o3d_to_ros_cloud(corrected, self.source_frame, msg.header.stamp))

    def _icp_refine(self, src, tgt, T):
        last_rmse = None
        last_fit = None

        for vx, mc, itn in zip(self.icp_voxels, self.icp_maxcorr, self.icp_maxiter):
            s = src.voxel_down_sample(vx)
            t = tgt.voxel_down_sample(vx)
            if vx <= 0.15:
                r = max(6.0 * vx, 0.12)
                s.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=r, max_nn=120))
                t.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=r, max_nn=120))
                method = o3d.pipelines.registration.TransformationEstimationPointToPlane(
                    o3d.pipelines.registration.TukeyLoss(1.5)
                )
            else:
                method = o3d.pipelines.registration.TransformationEstimationPointToPoint()

            reg = o3d.pipelines.registration.registration_icp(
                s, t, mc, T, method,
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=itn)
            )
            T = reg.transformation
            last_rmse = float(reg.inlier_rmse)
            last_fit = float(reg.fitness)

        return T, last_rmse, last_fit


def main():
    rclpy.init()
    node = FPFHRelocalizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()