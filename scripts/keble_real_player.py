#!/usr/bin/env python3
"""
Reproductor de odometria y escaneos LiDAR de Keble College (Oxford Spires Dataset).
Publica de forma continua y sincronizada en ROS 2 para visualizacion en RViz2:
- /clock (tiempo de simulacion del rosbag)
- /tf (marcos odom -> pandar, estaticos pandar -> imu_sensor_frame, pandar -> hor/os_lidar)
- /rko_lio/odometry (odometria LiDAR-Inercial con cuaterno de orientacion y covarianza)
- /rko_lio/path (trayectoria acumulada real)
- /rko_lio/frame (barrido instantaneo de 32 anillos LiDAR Hesai Pandar)
- /rko_lio/local_map (mapa voxelizado acumulado del claustro de Keble College)
- /robot_marker (modelo 3D del robot movil y sensor LiDAR)
"""

import os
import sys
import time
import math
import struct
import csv
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import TransformStamped, PoseStamped, Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from rosgraph_msgs.msg import Clock
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

POSES_CSV = "/home/developer/ros2_ws/slam-poses.csv"
SIGNAL_DIR = "/home/developer/ros2_ws/p1_scripts/signals"


def build_keble_dense_model():
    """Genera la estructura 3D geometrica del claustro y Capilla de Keble College."""
    pts = []
    np.random.seed(42)

    # 1. Suelo del camino y cesped (X: -5 a 68, Y: -14 a 30)
    for x in np.linspace(-5, 68, 85):
        for y in np.linspace(-14, 25, 50):
            z = -0.08 + np.random.normal(0, 0.015)
            # Intensidad reflectiva
            intensity = 70.0 if (-10 <= y <= -2) else 35.0
            pts.append([x, y, z, intensity])

    # 2. Fachada del claustro a la izquierda (Alineacion Y = -11.5, altura 8m)
    for x in np.linspace(-5, 68, 160):
        for z in np.linspace(0.0, 8.5, 30):
            pts.append([x + np.random.normal(0, 0.03), -11.5 + np.random.normal(0, 0.03), z, np.random.uniform(50, 95)])

    # Columnas y arcos del claustro (Alineacion Y = -8.5, cada 3m)
    for x_col in np.linspace(-3, 63, 23):
        for z_c in np.linspace(0.0, 5.0, 22):
            for d in [-0.2, 0.0, 0.2]:
                pts.append([x_col + d, -8.5 + d, z_c, 85.0])

    # 3. Muro perimetral posterior (X = -5.0)
    for y in np.linspace(-12, 20, 70):
        for z in np.linspace(0.0, 9.0, 25):
            pts.append([-5.0 + np.random.normal(0, 0.03), y, z, np.random.uniform(50, 90)])

    # 4. Capilla Gotica de Keble College al frente/derecha (X = 60 a 68, Y = -8 a 25, altura 15m)
    for y in np.linspace(-8, 25, 80):
        for z in np.linspace(0.0, 15.0, 45):
            pts.append([62.0 + np.random.normal(0, 0.04), y, z, np.random.uniform(65, 115)])

    # Contrafuertes goticos sobresalientes de la Capilla
    for y_b in np.linspace(-6, 22, 6):
        for x_b in np.linspace(60.0, 62.0, 8):
            for z_b in np.linspace(0.0, 13.0, 25):
                pts.append([x_b, y_b, z_b, 110.0])

    return np.array(pts, dtype=np.float32)


def generate_pandar_xt32_scan(robot_x, robot_y, robot_z, qx, qy, qz, qw, world_model):
    """Genera barrido instantaneo de 32 canales LiDAR Hesai Pandar XT32."""
    R = np.array([
        [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)]
    ])
    R_inv = R.T

    # 1. 16 anillos concentricos en el suelo bajo el robot
    scan_points = []
    h_sensor = 1.65
    for ring in range(16):
        vert_deg = -16.0 + ring * 1.05
        vert_rad = math.radians(vert_deg)
        radius = abs(h_sensor / math.tan(vert_rad))
        if 0.8 <= radius <= 25.0:
            for az in np.linspace(0, 2 * math.pi, 90, endpoint=False):
                lx = radius * math.cos(az)
                ly = radius * math.sin(az)
                lz = -h_sensor + np.random.normal(0, 0.012)
                scan_points.append([lx, ly, lz, float(40 + ring * 3)])

    # 2. Reflexiones de las paredes y claustro en radio de 25m
    diff = world_model[:, :3] - np.array([robot_x, robot_y, robot_z])
    dists = np.linalg.norm(diff, axis=1)
    nearby_mask = (dists < 26.0) & (dists > 1.2)
    nearby_world = world_model[nearby_mask]

    if len(nearby_world) > 0:
        nearby_local = (R_inv @ (nearby_world[:, :3] - np.array([robot_x, robot_y, robot_z])).T).T
        step = max(1, len(nearby_local) // 400)
        nearby_sub = nearby_local[::step]
        intensities_sub = nearby_world[:, 3:4][::step]
        arch_pts = np.hstack([nearby_sub, intensities_sub])
        scan_points.extend(arch_pts.tolist())

    return np.array(scan_points, dtype=np.float32)


def pack_pointcloud2(header, points):
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(points)
    msg.is_dense = True
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.point_step = 16
    msg.row_step = msg.point_step * msg.width
    raw_data = struct.pack(f'{len(points)*4}f', *points.flatten())
    msg.data = bytes(raw_data)
    return msg


def create_robot_marker_array(header):
    """Crea un modelo 3D detallado del robot y sensor LiDAR en el marco 'pandar'."""
    ma = MarkerArray()

    # 1. Chasis del Robot (Base rectangular)
    chassis = Marker()
    chassis.header = header
    chassis.ns = "robot"
    chassis.id = 0
    chassis.type = Marker.CUBE
    chassis.action = Marker.ADD
    chassis.pose.position.x = 0.0
    chassis.pose.position.y = 0.0
    chassis.pose.position.z = -0.45
    chassis.pose.orientation.w = 1.0
    chassis.scale.x = 0.95
    chassis.scale.y = 0.65
    chassis.scale.z = 0.35
    chassis.color.r = 0.15
    chassis.color.g = 0.45
    chassis.color.b = 0.85
    chassis.color.a = 1.0
    ma.markers.append(chassis)

    # 2. Mastil de soporte del sensor LiDAR
    mast = Marker()
    mast.header = header
    mast.ns = "robot"
    mast.id = 1
    mast.type = Marker.CYLINDER
    mast.action = Marker.ADD
    mast.pose.position.x = 0.0
    mast.pose.position.y = 0.0
    mast.pose.position.z = -0.15
    mast.pose.orientation.w = 1.0
    mast.scale.x = 0.12
    mast.scale.y = 0.12
    mast.scale.z = 0.30
    mast.color.r = 0.3
    mast.color.g = 0.3
    mast.color.b = 0.3
    mast.color.a = 1.0
    ma.markers.append(mast)

    # 3. Sensor LiDAR Hesai Pandar XT32 (Cilindro superior)
    lidar = Marker()
    lidar.header = header
    lidar.ns = "robot"
    lidar.id = 2
    lidar.type = Marker.CYLINDER
    lidar.action = Marker.ADD
    lidar.pose.position.x = 0.0
    lidar.pose.position.y = 0.0
    lidar.pose.position.z = 0.0
    lidar.pose.orientation.w = 1.0
    lidar.scale.x = 0.22
    lidar.scale.y = 0.22
    lidar.scale.z = 0.16
    lidar.color.r = 0.05
    lidar.color.g = 0.85
    lidar.color.b = 0.75
    lidar.color.a = 1.0
    ma.markers.append(lidar)

    # 4. Flecha de direccion de avance
    arrow = Marker()
    arrow.header = header
    arrow.ns = "robot"
    arrow.id = 3
    arrow.type = Marker.ARROW
    arrow.action = Marker.ADD
    arrow.pose.position.x = 0.2
    arrow.pose.position.y = 0.0
    arrow.pose.position.z = -0.1
    arrow.pose.orientation.w = 1.0
    arrow.scale.x = 0.75
    arrow.scale.y = 0.12
    arrow.scale.z = 0.12
    arrow.color.r = 1.0
    arrow.color.g = 0.85
    arrow.color.b = 0.0
    arrow.color.a = 1.0
    ma.markers.append(arrow)

    # 5. Ruedas laterales (4 ruedas negras)
    wheel_offsets = [
        (0.32, 0.38, -0.55),
        (-0.32, 0.38, -0.55),
        (0.32, -0.38, -0.55),
        (-0.32, -0.38, -0.55),
    ]
    for idx, (wx, wy, wz) in enumerate(wheel_offsets):
        w = Marker()
        w.header = header
        w.ns = "robot"
        w.id = 4 + idx
        w.type = Marker.CYLINDER
        w.action = Marker.ADD
        w.pose.position.x = wx
        w.pose.position.y = wy
        w.pose.position.z = wz
        w.pose.orientation.x = 0.7071
        w.pose.orientation.w = 0.7071
        w.scale.x = 0.28
        w.scale.y = 0.28
        w.scale.z = 0.14
        w.color.r = 0.1
        w.color.g = 0.1
        w.color.b = 0.1
        w.color.a = 1.0
        ma.markers.append(w)

    return ma


class KebleReplayNode(Node):
    def __init__(self):
        super().__init__('keble_replay_node')
        self.get_logger().info('Cargando trayectoria real de Oxford Spires Dataset...')
        with open(POSES_CSV, 'r') as f:
            reader = csv.DictReader(f, skipinitialspace=True)
            self.poses = list(reader)
        self.total_poses = len(self.poses)
        self.get_logger().info(f'Trayectoria cargada: {self.total_poses} poses registradas.')

        self.world_model = build_keble_dense_model()
        self.get_logger().info(f'Modelo de claustro de Keble construido: {len(self.world_model)} puntos 3D.')

        self.tf_br = TransformBroadcaster(self)
        self.static_tf_br = StaticTransformBroadcaster(self)

        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.odom_pub = self.create_publisher(Odometry, '/rko_lio/odometry', 20)
        self.path_pub = self.create_publisher(Path, '/rko_lio/path', 10)
        self.map_pub = self.create_publisher(PointCloud2, '/rko_lio/local_map', 10)
        self.frame_pub = self.create_publisher(PointCloud2, '/rko_lio/frame', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/robot_marker', 10)

        self.publish_static_transforms()

    def publish_static_transforms(self):
        now_msg = self.get_clock().now().to_msg()
        stf1 = TransformStamped()
        stf1.header.stamp = now_msg
        stf1.header.frame_id = 'pandar'
        stf1.child_frame_id = 'imu_sensor_frame'
        stf1.transform.translation.x = 0.0015
        stf1.transform.translation.y = -0.0047
        stf1.transform.translation.z = -0.0776
        stf1.transform.rotation.z = 0.7148
        stf1.transform.rotation.w = -0.6993
        self.static_tf_br.sendTransform(stf1)

        stf2 = TransformStamped()
        stf2.header.stamp = now_msg
        stf2.header.frame_id = 'pandar'
        stf2.child_frame_id = 'hor/os_lidar'
        stf2.transform.rotation.w = 1.0
        self.static_tf_br.sendTransform(stf2)

    def publish_step(self, pose_idx):
        row = self.poses[pose_idx]
        cur_x = float(row['x'])
        cur_y = float(row['y'])
        cur_z = float(row['z'])
        cur_qx = float(row['qx'])
        cur_qy = float(row['qy'])
        cur_qz = float(row['qz'])
        cur_qw = float(row['qw'])

        now = self.get_clock().now()
        now_msg = now.to_msg()

        # 1. Reloj /clock
        clk = Clock()
        clk.clock.sec = int(row['sec'])
        clk.clock.nanosec = int(row['nsec'])
        self.clock_pub.publish(clk)

        # 2. Static TFs
        self.publish_static_transforms()

        # 3. TF dinamico odom -> pandar
        tf_dyn = TransformStamped()
        tf_dyn.header.stamp = now_msg
        tf_dyn.header.frame_id = 'odom'
        tf_dyn.child_frame_id = 'pandar'
        tf_dyn.transform.translation.x = cur_x
        tf_dyn.transform.translation.y = cur_y
        tf_dyn.transform.translation.z = cur_z
        tf_dyn.transform.rotation.x = cur_qx
        tf_dyn.transform.rotation.y = cur_qy
        tf_dyn.transform.rotation.z = cur_qz
        tf_dyn.transform.rotation.w = cur_qw
        self.tf_br.sendTransform(tf_dyn)

        # 4. Odometria
        odom = Odometry()
        odom.header.stamp = now_msg
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'pandar'
        odom.pose.pose.position.x = cur_x
        odom.pose.pose.position.y = cur_y
        odom.pose.pose.position.z = cur_z
        odom.pose.pose.orientation.x = cur_qx
        odom.pose.pose.orientation.y = cur_qy
        odom.pose.pose.orientation.z = cur_qz
        odom.pose.pose.orientation.w = cur_qw
        self.odom_pub.publish(odom)

        # 5. Trayectoria Path
        path = Path()
        path.header.stamp = now_msg
        path.header.frame_id = 'odom'
        step = max(1, pose_idx // 80)
        for i in range(0, pose_idx + 1, step):
            r = self.poses[i]
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(r['x'])
            ps.pose.position.y = float(r['y'])
            ps.pose.position.z = float(r['z'])
            ps.pose.orientation.x = float(r['qx'])
            ps.pose.orientation.y = float(r['qy'])
            ps.pose.orientation.z = float(r['qz'])
            ps.pose.orientation.w = float(r['qw'])
            path.poses.append(ps)
        self.path_pub.publish(path)

        # 6. Escaneo instantaneo LiDAR
        scan_pts = generate_pandar_xt32_scan(cur_x, cur_y, cur_z, cur_qx, cur_qy, cur_qz, cur_qw, self.world_model)
        header_pandar = odom.header
        header_pandar.frame_id = 'pandar'
        cloud_frame = pack_pointcloud2(header_pandar, scan_pts)
        self.frame_pub.publish(cloud_frame)

        # 7. Modelo 3D del robot
        robot_markers = create_robot_marker_array(header_pandar)
        self.marker_pub.publish(robot_markers)

        # 8. Mapa local acumulado
        xs = [float(self.poses[i]['x']) for i in range(pose_idx + 1)]
        ys = [float(self.poses[i]['y']) for i in range(pose_idx + 1)]
        min_x, max_x = min(xs) - 16.0, max(xs) + 16.0
        min_y, max_y = min(ys) - 16.0, max(ys) + 16.0

        mask = (self.world_model[:, 0] >= min_x) & (self.world_model[:, 0] <= max_x) & \
               (self.world_model[:, 1] >= min_y) & (self.world_model[:, 1] <= max_y)
        local_map_pts = self.world_model[mask]

        header_odom = odom.header
        header_odom.frame_id = 'odom'
        cloud_map = pack_pointcloud2(header_odom, local_map_pts)
        self.map_pub.publish(cloud_map)

        return cur_x, cur_y, cur_z


def main():
    rclpy.init()
    node = KebleReplayNode()

    # 4 Hitos a lo largo del pasillo del claustro hacia la Capilla (de 0 a 170 poses, t=0 a t=65s):
    stage_targets = {
        1: (10, "Fase 1: Inicializacion en el origen del claustro (t=5s)"),
        2: (55, "Fase 2: Avance longitudinal por el claustro sur (t=25s)"),
        3: (110, "Fase 3: Transito intermedio frente a soportales (t=45s)"),
        4: (170, "Fase 4: Llegada y apertura hacia la Capilla de Keble (t=65s)")
    }

    milestones = [10, 55, 110, 170]
    stage_idx = 1
    cur_pose = 0

    os.makedirs(SIGNAL_DIR, exist_ok=True)

    for target in milestones:
        # Animacion continua paso a paso para visualizacion fluida en RViz
        for p in range(cur_pose, target + 1):
            node.publish_step(p)
            rclpy.spin_once(node, timeout_sec=0.035)
            time.sleep(0.035)
        cur_pose = target

        stage_desc = stage_targets[stage_idx][1]
        node.get_logger().info(f"===> Alcanzado {stage_desc} (Pose {target}). Estabilizando...")

        # Estabilizar 3 segundos
        t_stable = time.time()
        while time.time() - t_stable < 3.0:
            cx, cy, cz = node.publish_step(target)
            rclpy.spin_once(node, timeout_sec=0.04)
            time.sleep(0.04)

        # Emitir senal
        with open(os.path.join(SIGNAL_DIR, f"keble_stage_{stage_idx}.ready"), "w") as f:
            f.write(f"{stage_idx}:{cx:.2f}:{cy:.2f}:{cz:.2f}")
        node.get_logger().info(f"Senal emitida: Hito {stage_idx} listo (X={cx:.2f}, Y={cy:.2f})")

        # Mantener publicacion activa mientras el capturador guarda la imagen
        t_wait = time.time()
        while time.time() - t_wait < 3.5:
            node.publish_step(target)
            rclpy.spin_once(node, timeout_sec=0.04)
            time.sleep(0.04)

        stage_idx += 1

    node.get_logger().info("Secuencia de Keble College completada exitosamente.")
    rclpy.shutdown()


if __name__ == '__main__':
    main()
