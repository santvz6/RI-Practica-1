#!/usr/bin/env python3
"""
=============================================================================
Práctica 1 - Robótica Inteligente (Grado en IA - EPS Universidad de Alicante)
Tarea de Programación Opcional 2: Envío de Pose Deseada al Brazo UR3e (MoveIt 2)
=============================================================================

Descripción:
Este script permite enviar objetivos cartesianos y articulares al manipulador
robótico Universal Robots UR3e mediante la interfaz de acciones de MoveIt 2
(MoveGroup / FollowJointTrajectory) sin requerir interacción manual con RViz2.

Modos de uso:
  1. Modo Articular (Joint Goal):
     python3 ur3e_send_pose.py --mode joint --joints 0.0 -1.57 1.57 -1.57 -1.57 0.0

  2. Modo Cartesiano (Pose Goal):
     python3 ur3e_send_pose.py --mode cartesian --x 0.25 --y 0.15 --z 0.35 --roll 0.0 --pitch 3.1415 --yaw 0.0
"""

import argparse
import math
import sys
import time
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# Intentar importar mensajes de MoveIt si están presentes en el entorno ROS2
try:
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint
    from shape_msgs.msg import SolidPrimitive
    MOVEIT_AVAILABLE = True
except ImportError:
    MOVEIT_AVAILABLE = False


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Convierte ángulos de Euler (rad) a cuaternión de orientación."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * cp * cy + sr * sp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class UR3eGoalSender(Node):
    """Nodo ROS 2 para gobernar el brazo UR3e mediante MoveIt 2 / ros2_control."""

    UR3E_JOINTS = [
        'shoulder_pan_joint',
        'shoulder_lift_joint',
        'elbow_joint',
        'wrist_1_joint',
        'wrist_2_joint',
        'wrist_3_joint',
    ]

    def __init__(self):
        super().__init__('ur3e_goal_sender_node')
        self.get_logger().info('Inicializando cliente de control para brazo Universal Robots UR3e...')

        # Cliente directo del controlador articular de ros2_control (máxima compatibilidad)
        self._traj_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )

        # Cliente para MoveGroup (MoveIt 2)
        if MOVEIT_AVAILABLE:
            self._moveit_client = ActionClient(self, MoveGroup, 'move_action')
        else:
            self._moveit_client = None

        self._is_done = False

    def send_joint_trajectory(self, joint_angles: List[float], duration_sec: float = 4.0) -> bool:
        """Envía una meta articular al Joint Trajectory Controller."""
        self.get_logger().info('Conectando con el controlador de trayectoria articular...')
        if not self._traj_client.wait_for_server(timeout_sec=5.0):
            # Intentar nombre alternativo con escalado de velocidad
            self._traj_client = ActionClient(
                self,
                FollowJointTrajectory,
                '/scaled_joint_trajectory_controller/follow_joint_trajectory'
            )
            if not self._traj_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('No se pudo contactar con ningún controlador de trayectoria articular de ros2_control.')
                return False

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.UR3E_JOINTS

        point = JointTrajectoryPoint()
        point.positions = [float(j) for j in joint_angles]
        point.velocities = [0.0] * len(joint_angles)
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))

        goal_msg.trajectory.points.append(point)

        self.get_logger().info(f'Enviando configuración articular objetivo: {[round(a, 3) for a in joint_angles]} rad')
        send_future = self._traj_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._trajectory_response_cb)
        return True

    def send_moveit_cartesian_goal(self, x: float, y: float, z: float, roll: float, pitch: float, yaw: float, frame_id: str = 'base_link') -> bool:
        """Envía un objetivo cartesiano al pipeline de cinemática y planificación de MoveIt 2."""
        if not MOVEIT_AVAILABLE:
            self.get_logger().error('Módulos de moveit_msgs no encontrados. Utilice el modo articular.')
            return False

        self.get_logger().info('Conectando con el servidor MoveGroup de MoveIt 2...')
        if not self._moveit_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Servidor MoveGroup ("move_action") no disponible. Asegúrese de que MoveIt 2 esté activo.')
            return False

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'ur3e_arm'
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.5
        goal_msg.request.max_acceleration_scaling_factor = 0.5

        # Restricción de posición
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = frame_id
        pos_constraint.link_name = 'tool0'
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.01]  # Tolerancia 1 cm
        pos_constraint.constraint_region.primitives.append(primitive)
        pos_constraint.constraint_region.primitive_poses.append(
            PoseStamped(
                pose=Point(x=x, y=y, z=z)
            ).pose
        )
        pos_constraint.weight = 1.0

        # Restricción de orientación
        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = frame_id
        ori_constraint.link_name = 'tool0'
        ori_constraint.orientation = euler_to_quaternion(roll, pitch, yaw)
        ori_constraint.absolute_x_axis_tolerance = 0.1
        ori_constraint.absolute_y_axis_tolerance = 0.1
        ori_constraint.absolute_z_axis_tolerance = 0.1
        ori_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(ori_constraint)
        goal_msg.request.goal_constraints.append(constraints)

        self.get_logger().info(f'Enviando objetivo cartesiano a MoveIt 2: Pos=[{x:.3f}, {y:.3f}, {z:.3f}] m')
        send_future = self._moveit_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._moveit_response_cb)
        return True

    def _trajectory_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Meta articular rechazada por el controlador.')
            self._is_done = True
            return
        self.get_logger().info('Meta articular aceptada. Moviendo servomotores del UR3e...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _moveit_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Objetivo de planificación rechazado por MoveGroup.')
            self._is_done = True
            return
        self.get_logger().info('MoveGroup calculando cinemática inversa y trayectoria OMPL...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('¡Movimiento completado con éxito!')
        else:
            self.get_logger().warn(f'Movimiento finalizado con estado {status}.')
        self._is_done = True

    def is_done(self) -> bool:
        return self._is_done


def main():
    parser = argparse.ArgumentParser(description='Controlador por código Python para brazo robótico UR3e en ROS 2.')
    parser.add_argument('--mode', type=str, choices=['joint', 'cartesian'], default='joint', help='Modo de control (joint o cartesian)')
    parser.add_argument('--joints', nargs=6, type=float, default=[0.0, -1.57, 1.57, -1.57, -1.57, 0.0], help='6 ángulos articulares en radianes')
    parser.add_argument('--x', type=float, default=0.25, help='X cartesiano (m)')
    parser.add_argument('--y', type=float, default=0.15, help='Y cartesiano (m)')
    parser.add_argument('--z', type=float, default=0.35, help='Z cartesiano (m)')
    parser.add_argument('--roll', type=float, default=0.0, help='Roll (rad)')
    parser.add_argument('--pitch', type=float, default=3.1415, help='Pitch (rad)')
    parser.add_argument('--yaw', type=float, default=0.0, help='Yaw (rad)')
    parser.add_argument('--duration', type=float, default=4.0, help='Duración de la trayectoria en segundos')
    args = parser.parse_args()

    rclpy.init()
    node = UR3eGoalSender()

    if args.mode == 'joint':
        node.send_joint_trajectory(args.joints, duration_sec=args.duration)
    else:
        node.send_moveit_cartesian_goal(args.x, args.y, args.z, args.roll, args.pitch, args.yaw)

    while rclpy.ok() and not node.is_done():
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
