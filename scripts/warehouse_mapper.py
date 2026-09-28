#!/usr/bin/env python3
"""
=============================================================================
Práctica 1 - Robótica Inteligente (Grado en IA - EPS Universidad de Alicante)
Script de Mapeo y Recorrido Autónomo del Almacén para SLAM
=============================================================================

Descripción:
Guía al TurtleBot 4 a través de todos los pasillos y cuadrantes del entorno
Warehouse en Gazebo, ejecutando giros de 360 grados en puntos clave para
obtener una cobertura LiDAR completa del mapa en SLAM Toolbox.
Al finalizar el recorrido, invoca automáticamente el servidor de mapas para
guardar mapa_laboratorio.yaml y mapa_laboratorio.pgm.
"""

import math
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


WAYPOINTS = [
    # --- ZONA SUR (Pasillos entre estanterías grandes) ---
    # 1. Pasillo Centro-Derecha (hacia el sur)
    {"pos": (3.0, 0.0), "spin": False, "timeout": 25},
    {"pos": (3.0, -10.0), "spin": False, "timeout": 30},
    {"pos": (3.0, -21.0), "spin": True, "timeout": 35},

    # 2. Paso sur -> Pasillo Centro-Izquierda (hacia el norte)
    {"pos": (-1.0, -23.0), "spin": False, "timeout": 25},
    {"pos": (-5.0, -21.0), "spin": False, "timeout": 25},
    {"pos": (-5.0, -10.0), "spin": False, "timeout": 30},
    {"pos": (-5.0, 1.0), "spin": True, "timeout": 35},

    # 3. Cruce central -> Pasillo Oeste (hacia el sur)
    {"pos": (-8.5, 1.0), "spin": False, "timeout": 25},
    {"pos": (-11.5, -2.0), "spin": False, "timeout": 25},
    {"pos": (-11.5, -12.0), "spin": False, "timeout": 30},
    {"pos": (-11.5, -21.0), "spin": True, "timeout": 35},

    # 4. Paso sur -> Pasillo Este (hacia el norte)
    {"pos": (-5.0, -23.0), "spin": False, "timeout": 30},
    {"pos": (2.0, -23.0), "spin": False, "timeout": 30},
    {"pos": (9.5, -21.0), "spin": False, "timeout": 30},
    {"pos": (9.5, -10.0), "spin": False, "timeout": 30},
    {"pos": (9.5, 2.0), "spin": True, "timeout": 35},

    # --- ZONA NORTE (Estanterías horizontales y obstáculos) ---
    # 5. Pasillo intermedio entre estanterías norte
    {"pos": (9.5, 8.0), "spin": False, "timeout": 25},
    {"pos": (9.5, 14.0), "spin": False, "timeout": 25},
    {"pos": (2.0, 14.0), "spin": False, "timeout": 30},
    {"pos": (-6.5, 14.0), "spin": True, "timeout": 35},

    # 6. Pasillo superior norte
    {"pos": (0.0, 14.0), "spin": False, "timeout": 25},
    {"pos": (8.5, 14.0), "spin": False, "timeout": 30},
    {"pos": (8.5, 21.0), "spin": False, "timeout": 25},
    {"pos": (1.0, 21.0), "spin": False, "timeout": 30},
    {"pos": (-4.0, 21.0), "spin": True, "timeout": 35},

    # 7. Regreso por pasillo noroeste al centro
    {"pos": (-7.0, 15.0), "spin": False, "timeout": 25},
    {"pos": (-7.0, 8.0), "spin": False, "timeout": 25},
    {"pos": (-7.0, 2.0), "spin": False, "timeout": 25},
    {"pos": (0.0, 1.0), "spin": True, "timeout": 30},
]


class WarehouseTourMapper(Node):
    def __init__(self):
        super().__init__('warehouse_tour_mapper')
        self.get_logger().info('Inicializando explorador autónomo de almacén...')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_unstamped', 10)

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

        self.current_x = None
        self.current_y = None
        self.current_yaw = None

        self.min_front = 10.0
        self.min_front_left = 10.0
        self.min_front_right = 10.0
        self.min_left = 10.0
        self.min_right = 10.0

        self.map_coverage = 0.0
        self.known_cells = 0

        self.wp_idx = 0
        self.wp_start_time = time.time()
        self.state = 'NAVIGATING'  # 'NAVIGATING', 'SPINNING', 'RECOVERING'
        self.spin_start_time = None
        self.recover_start_time = None

        self.timer = self.create_timer(0.1, self.control_loop)
        self.last_log_time = time.time()
        self.finished = False

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = quaternion_to_yaw(msg.pose.pose.orientation)

    def scan_callback(self, msg: LaserScan):
        num_readings = len(msg.ranges)
        if num_readings == 0:
            return

        def get_min_in_sector(angle_start_deg, angle_end_deg):
            rad_start = math.radians(angle_start_deg)
            rad_end = math.radians(angle_end_deg)
            min_dist = 12.0
            for i, r in enumerate(msg.ranges):
                if math.isinf(r) or math.isnan(r) or r < msg.range_min:
                    continue
                angle = msg.angle_min + i * msg.angle_increment
                angle = normalize_angle(angle)
                if rad_start <= angle <= rad_end:
                    if r < min_dist:
                        min_dist = r
            return min_dist

        self.min_front = get_min_in_sector(-25, 25)
        self.min_front_left = get_min_in_sector(25, 60)
        self.min_front_right = get_min_in_sector(-60, -25)
        self.min_left = get_min_in_sector(60, 110)
        self.min_right = get_min_in_sector(-110, -60)

    def map_callback(self, msg: OccupancyGrid):
        total = len(msg.data)
        if total > 0:
            known = sum(1 for v in msg.data if v != -1)
            self.known_cells = known
            self.map_coverage = (known / total) * 100.0

    def control_loop(self):
        if self.finished or self.current_x is None:
            return

        now = time.time()
        twist = Twist()

        # Log periódico cada 3 segundos
        if now - self.last_log_time > 3.0:
            self.last_log_time = now
            wp_total = len(WAYPOINTS)
            wp_curr = min(self.wp_idx + 1, wp_total)
            target = WAYPOINTS[self.wp_idx]["pos"] if self.wp_idx < wp_total else (0, 0)
            dist_target = math.hypot(target[0] - self.current_x, target[1] - self.current_y) if self.wp_idx < wp_total else 0.0
            print(
                f"[Progreso {wp_curr}/{wp_total}] Estado: {self.state:<11} | "
                f"Robot: ({self.current_x:5.2f}, {self.current_y:5.2f}) | "
                f"Meta: ({target[0]:5.2f}, {target[1]:5.2f}) d={dist_target:4.2f}m | "
                f"Frente: {self.min_front:4.2f}m | Mapa: {self.map_coverage:4.1f}% ({self.known_cells} celdas)"
            )
            sys.stdout.flush()

        # -------------------------------------------------------------
        # ESTADO 1: GIRANDO (SPINNING) PARA REGISTRO LIDAR 360
        # -------------------------------------------------------------
        if self.state == 'SPINNING':
            if self.spin_start_time is None:
                self.spin_start_time = now

            if now - self.spin_start_time < 12.0:
                twist.linear.x = 0.0
                twist.angular.z = 0.55  # Giro suave para SLAM
                self.cmd_pub.publish(twist)
                return
            else:
                self.spin_start_time = None
                self.wp_idx += 1
                self.wp_start_time = now
                if self.wp_idx >= len(WAYPOINTS):
                    self.finish_tour()
                    return
                self.state = 'NAVIGATING'

        # -------------------------------------------------------------
        # ESTADO 2: RECUPERACIÓN (RECOVERING) SI HAY BLOQUEO
        # -------------------------------------------------------------
        elif self.state == 'RECOVERING':
            if self.recover_start_time is None:
                self.recover_start_time = now

            elapsed = now - self.recover_start_time
            if elapsed < 2.5:
                twist.linear.x = -0.15
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                return
            elif elapsed < 5.0:
                twist.linear.x = 0.0
                twist.angular.z = 0.6 if self.min_left > self.min_right else -0.6
                self.cmd_pub.publish(twist)
                return
            else:
                self.recover_start_time = None
                self.state = 'NAVIGATING'
                self.wp_start_time = now

        # -------------------------------------------------------------
        # ESTADO 3: NAVEGANDO HACIA WAYPOINT (NAVIGATING)
        # -------------------------------------------------------------
        elif self.state == 'NAVIGATING':
            target = WAYPOINTS[self.wp_idx]["pos"]
            dx = target[0] - self.current_x
            dy = target[1] - self.current_y
            dist = math.hypot(dx, dy)
            target_angle = math.atan2(dy, dx)
            angle_diff = normalize_angle(target_angle - self.current_yaw)

            # Comprobar timeout del waypoint
            wp_timeout = WAYPOINTS[self.wp_idx]["timeout"]
            if now - self.wp_start_time > wp_timeout:
                self.get_logger().warn(f"Timeout en Waypoint {self.wp_idx+1}. Avanzando...")
                if WAYPOINTS[self.wp_idx]["spin"]:
                    self.state = 'SPINNING'
                else:
                    self.wp_idx += 1
                    self.wp_start_time = now
                    if self.wp_idx >= len(WAYPOINTS):
                        self.finish_tour()
                        return
                return

            # Waypoint alcanzado
            if dist < 0.9:
                self.get_logger().info(f"Waypoint {self.wp_idx+1}/{len(WAYPOINTS)} alcanzado!")
                if WAYPOINTS[self.wp_idx]["spin"]:
                    self.state = 'SPINNING'
                    self.spin_start_time = now
                else:
                    self.wp_idx += 1
                    self.wp_start_time = now
                    if self.wp_idx >= len(WAYPOINTS):
                        self.finish_tour()
                        return
                return

            # Evasión reactiva de obstáculos
            safe_dist = 0.65
            crit_dist = 0.40

            if self.min_front < crit_dist or min(self.min_front_left, self.min_front_right) < crit_dist:
                # Obstáculo muy cerca: girar hacia el lado más despejado
                twist.linear.x = 0.0
                turn_dir = 1.0 if self.min_left > self.min_right else -1.0
                twist.angular.z = turn_dir * 0.7
            elif self.min_front < safe_dist:
                # Obstáculo moderado: reducir velocidad y esquivar
                twist.linear.x = 0.10
                turn_dir = 1.0 if self.min_left > self.min_right else -1.0
                twist.angular.z = turn_dir * 0.6
            else:
                # Camino despejado: orientarse al waypoint
                if abs(angle_diff) > 0.8:
                    twist.linear.x = 0.05
                    twist.angular.z = math.copysign(0.7, angle_diff)
                else:
                    speed = min(0.38, max(0.15, dist * 0.3))
                    twist.linear.x = speed * math.cos(angle_diff)
                    twist.angular.z = max(-0.8, min(0.8, angle_diff * 1.2))

                    # Pequeña repulsión lateral preventiva
                    if self.min_front_left < safe_dist:
                        twist.angular.z -= 0.3
                    elif self.min_front_right < safe_dist:
                        twist.angular.z += 0.3

            self.cmd_pub.publish(twist)

    def finish_tour(self):
        self.finished = True
        # Parar robot
        self.cmd_pub.publish(Twist())
        print("\n" + "=" * 60)
        print("🎉 ¡RECORRIDO DEL ALMACÉN COMPLETADO CON ÉXITO!")
        print(f"📊 Cobertura final del mapa: {self.map_coverage:.1f}% ({self.known_cells} celdas exploradas)")
        print("=" * 60)
        self.save_map()

    def save_map(self):
        print("\n[+] Guardando mapa permanente con nav2_map_server...")
        save_cmd = [
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
            '-f', '/home/developer/ros2_ws/mapa_laboratorio'
        ]
        try:
            res = subprocess.run(save_cmd, capture_output=True, text=True, timeout=30)
            print(res.stdout)
            if res.returncode == 0:
                print("[✓] ¡Mapa guardado correctamente en /home/developer/ros2_ws/mapa_laboratorio!")
            else:
                print(f"[!] Error al guardar mapa: {res.stderr}")
        except Exception as e:
            print(f"[!] Excepción al invocar map_saver_cli: {e}")


def main():
    rclpy.init()
    tour = WarehouseTourMapper()
    try:
        while rclpy.ok() and not tour.finished:
            rclpy.spin_once(tour, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        tour.finish_tour()
        tour.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
