#!/usr/bin/env python3
"""
=============================================================================
Práctica 1 - Robótica Inteligente (Grado en IA - EPS Universidad de Alicante)
Tarea de Programación Opcional 1: Envío de Pose Deseada en Nav2 por Código Python
=============================================================================

Descripción:
Este script permite enviar objetivos de navegación autónoma (Goal Pose) al
stack Nav2 (utilizado con TurtleBot 4 en simulación Gazebo) a través de la API
de acciones de ROS 2 (NavigateToPose) y mediante nav2_simple_commander,
sin requerir la interacción manual mediante clics en RViz2.

Uso:
  python3 nav2_send_goal.py --x 2.0 --y 1.5 --yaw 1.57
  python3 nav2_send_goal.py --x -1.0 --y 0.5 --yaw 0.0 --frame map
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Convierte ángulos de Euler (rad) a un cuaternión unitario de ROS 2."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class Nav2GoalSender(Node):
    """Nodo cliente de acción de ROS 2 para la navegación autónoma con Nav2."""

    def __init__(self):
        super().__init__('nav2_goal_sender_node')
        self.get_logger().info('Inicializando cliente de navegación autónoma Nav2...')

        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._goal_handle = None
        self._is_done = False
        self._start_time = None

    def send_goal(self, x: float, y: float, yaw_deg: float, frame_id: str = 'map'):
        yaw_rad = math.radians(yaw_deg)
        self.get_logger().info(
            f'Conectando con el servidor de acción "navigate_to_pose"...'
        )
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                'Servidor de acción "navigate_to_pose" no disponible. '
                'Verifique que Nav2 esté ejecutándose.'
            )
            return False

        # Construir el mensaje PoseStamped objetivo
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = frame_id
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position = Point(x=x, y=y, z=0.0)
        goal_msg.pose.pose.orientation = euler_to_quaternion(0.0, 0.0, yaw_rad)

        self.get_logger().info(
            f'Enviando objetivo -> X: {x:.2f} m, Y: {y:.2f} m, Yaw: {yaw_deg:.1f}° ({yaw_rad:.2f} rad) en frame "{frame_id}"'
        )

        self._start_time = time.time()
        send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
        return True

    def goal_response_callback(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().error('El planificador de Nav2 ha RECHAZADO el objetivo (meta inaccesible o en colisión).')
            self._is_done = True
            return

        self.get_logger().info('Objetivo ACEPTADO por Nav2. Iniciando ejecución de trayectoria...')
        get_result_future = self._goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        dist_remaining = feedback.distance_remaining
        time_elapsed = feedback.navigation_time.sec + feedback.navigation_time.nanosec * 1e-9
        recovers = feedback.number_of_recoveries

        sys.stdout.write(
            f'\r[Nav2 Feedback] Distancia restante: {dist_remaining:5.2f} m | '
            f'Tiempo: {time_elapsed:5.1f} s | Recuperaciones: {recovers}'
        )
        sys.stdout.flush()

    def get_result_callback(self, future):
        status = future.result().status
        sys.stdout.write('\n')
        total_time = time.time() - self._start_time if self._start_time else 0.0

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f'¡ÉXITO! El TurtleBot 4 ha alcanzado la pose objetivo en {total_time:.2f} s.'
            )
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('La navegación fue CANCELADA.')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error('La navegación fue ABORTADA por Nav2 (obstáculo dinámico insuperable o timeout).')
        else:
            self.get_logger().info(f'Navegación finalizada con código de estado: {status}')

        self._is_done = True

    def is_done(self) -> bool:
        return self._is_done


def main():
    parser = argparse.ArgumentParser(
        description='Envío programático de pose objetivo a Nav2 para TurtleBot 4 (ROS 2 Jazzy).'
    )
    parser.add_argument('--x', type=float, required=True, help='Coordenada X en metros')
    parser.add_argument('--y', type=float, required=True, help='Coordenada Y en metros')
    parser.add_argument('--yaw', type=float, default=0.0, help='Orientación deseada (grados). Por defecto 0.0')
    parser.add_argument('--frame', type=str, default='map', help='Sistema de referencia (defecto: map)')
    args = parser.parse_args()

    rclpy.init()
    node = Nav2GoalSender()

    if not node.send_goal(x=args.x, y=args.y, yaw_deg=args.yaw, frame_id=args.frame):
        rclpy.shutdown()
        sys.exit(1)

    while rclpy.ok() and not node.is_done():
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
