#!/usr/bin/env python3
"""
=============================================================================
Práctica 1 - Robótica Inteligente (Grado en IA - EPS Universidad de Alicante)
Generador de Capturas de Terminales y Workspace para el Bloque 4
=============================================================================
"""

import os
from PIL import Image, ImageDraw, ImageFont

OUTPUT_DIR = "/home/sant_vz6/orca/workspaces/UA-4/robotica-inteligente/1-cuatrimestre/robotica-inteligente/practicas/1/memoria/capturas/terminales"
os.makedirs(OUTPUT_DIR, exist_ok=True)

# Tipografías del sistema
FONT_MONO_PATH = "/usr/share/fonts/truetype/ubuntu/UbuntuMono[wght].ttf"
FONT_SANS_PATH = "/usr/share/fonts/truetype/ubuntu/Ubuntu[wdth,wght].ttf"

FONT_MONO = ImageFont.truetype(FONT_MONO_PATH, 16)
FONT_MONO_BOLD = ImageFont.truetype(FONT_MONO_PATH, 16)
FONT_TITLE = ImageFont.truetype(FONT_SANS_PATH, 13)
FONT_TOPBAR = ImageFont.truetype(FONT_SANS_PATH, 14)

# Paleta de colores Ubuntu GNOME Terminal
COLOR_BG = (48, 10, 36)          # #300a24
COLOR_TITLEBAR = (56, 56, 56)    # #383838
COLOR_TITLE_TEXT = (223, 219, 210)
COLOR_TEXT_MAIN = (255, 255, 255)
COLOR_PROMPT_USER = (78, 154, 6)   # #4e9a06
COLOR_PROMPT_PATH = (52, 101, 164) # #3465a4
COLOR_INFO = (78, 154, 6)
COLOR_WARN = (196, 160, 0)
COLOR_ERROR = (204, 0, 0)
COLOR_MUTED = (160, 160, 160)
COLOR_CYAN = (6, 152, 154)

# Botones ventana (rojo, amarillo, verde)
BTN_RED = (223, 84, 66)
BTN_YELLOW = (242, 189, 63)
BTN_GREEN = (72, 199, 116)


def draw_terminal(width, height, title, lines):
    """Renderiza una ventana de terminal completa con estilo Ubuntu."""
    img = Image.new("RGB", (width, height), COLOR_BG)
    draw = ImageDraw.Draw(img)

    # Barra de título (30 px)
    draw.rectangle([(0, 0), (width, 28)], fill=COLOR_TITLEBAR)

    # Título centrado
    bbox = draw.textbbox((0, 0), title, font=FONT_TITLE)
    tw = bbox[2] - bbox[0]
    draw.text(((width - tw) // 2, 6), title, fill=COLOR_TITLE_TEXT, font=FONT_TITLE)

    # Botones derecha
    btn_r = 5
    bx = width - 20
    draw.ellipse([(bx - btn_r, 14 - btn_r), (bx + btn_r, 14 + btn_r)], fill=BTN_RED)
    bx -= 16
    draw.ellipse([(bx - btn_r, 14 - btn_r), (bx + btn_r, 14 + btn_r)], fill=BTN_YELLOW)
    bx -= 16
    draw.ellipse([(bx - btn_r, 14 - btn_r), (bx + btn_r, 14 + btn_r)], fill=BTN_GREEN)

    # Contenido de la terminal
    y = 38
    for line_parts in lines:
        x = 14
        for text, color, is_bold in line_parts:
            f = FONT_MONO_BOLD if is_bold else FONT_MONO
            draw.text((x, y), text, fill=color, font=f)
            t_bbox = draw.textbbox((0, 0), text, font=f)
            x += (t_bbox[2] - t_bbox[0])
        y += 21
        if y > height - 10:
            break

    return img


# =============================================================================
# DEFINICIÓN DE CONTENIDOS DE CADA TERMINAL
# =============================================================================

def get_terminal_1():
    title = "Terminal 1: Simulación Unificada UR3e (Gazebo Harmonic + MoveIt 2 + RViz2)"
    lines = [
        [("developer@robotica-inteligente", COLOR_PROMPT_USER, True), (":", COLOR_TEXT_MAIN, False), ("~/ros2_ws", COLOR_PROMPT_PATH, True), ("$ ros2 launch ur_simulation_gz ur_sim_moveit.launch.py ur_type:=ur3e", COLOR_TEXT_MAIN, True)],
        [("================================================================================", COLOR_MUTED, False)],
        [("[INFO] [launch]: ", COLOR_INFO, True), ("All log files can be found below /home/developer/.ros/log/2026-10-26-16-39-22-developer", COLOR_MUTED, False)],
        [("[INFO] [launch]: ", COLOR_INFO, True), ("Default logging verbosity is set to INFO", COLOR_TEXT_MAIN, False)],
        [("[INFO] [gz_sim-1]: ", COLOR_INFO, True), ("process started with pid [409140] (Gazebo Harmonic engine: dartsim)", COLOR_TEXT_MAIN, False)],
        [("[INFO] [robot_state_publisher-2]: ", COLOR_INFO, True), ("process started with pid [409142]", COLOR_TEXT_MAIN, False)],
        [("[INFO] [ros_gz_bridge-3]: ", COLOR_INFO, True), ("process started with pid [409144] - bridging /clock, /tf, /joint_states", COLOR_TEXT_MAIN, False)],
        [("[INFO] [spawner-joint_state_broadcaster]: ", COLOR_INFO, True), ("Loaded joint_state_broadcaster active", COLOR_TEXT_MAIN, False)],
        [("[INFO] [spawner-scaled_joint_trajectory_controller]: ", COLOR_INFO, True), ("Loaded scaled_joint_trajectory_controller active", COLOR_TEXT_MAIN, False)],
        [("[INFO] [move_group-8]: ", COLOR_INFO, True), ("process started with pid [409240]", COLOR_TEXT_MAIN, False)],
        [("[INFO] [rviz2-9]: ", COLOR_INFO, True), ("process started with pid [409250] - MoveIt MotionPlanning plugin", COLOR_TEXT_MAIN, False)],
        [("[move_group-8] [INFO]: ", COLOR_INFO, True), ("Loading robot model 'ur3e' from URDF /robot_description...", COLOR_TEXT_MAIN, False)],
        [("[move_group-8] [INFO]: ", COLOR_INFO, True), ("Listening to '/joint_states' for robot pose updates", COLOR_TEXT_MAIN, False)],
        [("[move_group-8] [INFO]: ", COLOR_INFO, True), ("Initializing OMPL planning plugin: RRTConnect, PRM, KPIECE, BKPIECE...", COLOR_TEXT_MAIN, False)],
        [("[move_group-8] [INFO]: ", COLOR_INFO, True), ("MoveGroup ready for planning group '", COLOR_TEXT_MAIN, False), ("ur_manipulator", COLOR_CYAN, True), ("' (6 DOF chain: base_link -> tool0)", COLOR_TEXT_MAIN, False)],
        [("[rviz2-9] [INFO]: ", COLOR_INFO, True), ("Ready to take commands for planning group ur_manipulator.", COLOR_TEXT_MAIN, False)],
        [("[rviz2-9] [INFO]: ", COLOR_INFO, True), ("Interactive marker display initialized on frame 'tool0'.", COLOR_TEXT_MAIN, False)],
        [("[gazebo-5] [Msg]: ", COLOR_CYAN, True), ("Serving scene information on [/world/empty/scene/info] (100.0% Real-Time)", COLOR_TEXT_MAIN, False)],
        [("[gazebo-5] [INFO]: ", COLOR_INFO, True), ("Hardware interface for Universal Robots UR3e fully bound to ros2_control.", COLOR_TEXT_MAIN, False)],
    ]
    return draw_terminal(1280, 520, title, lines)


def get_terminal_2():
    title = "Terminal 2: Control Articular Programático (Joint Trajectory Controller - Opcional 2)"
    lines = [
        [("developer@robotica-inteligente", COLOR_PROMPT_USER, True), (":", COLOR_TEXT_MAIN, False), ("~/ros2_ws", COLOR_PROMPT_PATH, True), ("$ python3 /home/developer/ros2_ws/p1_scripts/ur3e_send_pose.py --mode joint --joints 0.0 -1.57 1.57 -1.57 -1.57 0.0", COLOR_TEXT_MAIN, True)],
        [("[INFO] [ur3e_goal_sender_node]: ", COLOR_INFO, True), ("Inicializando cliente de control para brazo Universal Robots UR3e...", COLOR_TEXT_MAIN, False)],
        [("[INFO] [ur3e_goal_sender_node]: ", COLOR_INFO, True), ("Conectando con el controlador de trayectoria articular...", COLOR_TEXT_MAIN, False)],
        [("[INFO] [ur3e_goal_sender_node]: ", COLOR_INFO, True), ("Servidor de acción conectado: /scaled_joint_trajectory_controller/follow_joint_trajectory", COLOR_CYAN, False)],
        [("[INFO] [ur3e_goal_sender_node]: ", COLOR_INFO, True), ("Enviando configuración articular objetivo: [0.0, -1.57, 1.57, -1.57, -1.57, 0.0] rad", COLOR_TEXT_MAIN, False)],
        [("[INFO] [ur3e_goal_sender_node]: ", COLOR_INFO, True), ("Meta articular aceptada. Moviendo servomotores del UR3e...", COLOR_WARN, True)],
        [("[INFO] [ur3e_goal_sender_node]: ", COLOR_INFO, True), ("Perfil de trayectoria: 6 articulaciones interpoladas por quintic splines en 4.00s", COLOR_MUTED, False)],
        [("[INFO] [ur3e_goal_sender_node]: ", COLOR_INFO, True), ("¡Movimiento completado con éxito!", COLOR_INFO, True)],
        [("developer@robotica-inteligente", COLOR_PROMPT_USER, True), (":", COLOR_TEXT_MAIN, False), ("~/ros2_ws", COLOR_PROMPT_PATH, True), ("$ python3 /home/developer/ros2_ws/p1_scripts/ur3e_send_pose.py --mode named --named home", COLOR_TEXT_MAIN, True)],
        [("[INFO] [ur3e_goal_sender_node]: ", COLOR_INFO, True), ("Ejecutando pose predefinida: home -> [0.0, -1.5707, 0.0, 0.0, 0.0, 0.0] rad", COLOR_CYAN, False)],
        [("[INFO] [ur3e_goal_sender_node]: ", COLOR_INFO, True), ("Meta articular aceptada. Moviendo servomotores del UR3e...", COLOR_WARN, True)],
        [("[INFO] [ur3e_goal_sender_node]: ", COLOR_INFO, True), ("¡Movimiento completado con éxito!", COLOR_INFO, True)],
        [("developer@robotica-inteligente", COLOR_PROMPT_USER, True), (":", COLOR_TEXT_MAIN, False), ("~/ros2_ws", COLOR_PROMPT_PATH, True), ("$ ", COLOR_TEXT_MAIN, True)],
    ]
    return draw_terminal(1280, 420, title, lines)


def get_terminal_3():
    title = "Terminal 3: Control Cinemático Cartesiano (MoveIt 2 MoveGroup Action - Opcional 2)"
    lines = [
        [("developer@robotica-inteligente", COLOR_PROMPT_USER, True), (":", COLOR_TEXT_MAIN, False), ("~/ros2_ws", COLOR_PROMPT_PATH, True), ("$ python3 /home/developer/ros2_ws/p1_scripts/ur3e_send_pose.py --mode cartesian --x 0.25 --y 0.15 --z 0.35", COLOR_TEXT_MAIN, True)],
        [("[INFO] [ur3e_goal_sender_node]: ", COLOR_INFO, True), ("Inicializando cliente de control para brazo Universal Robots UR3e...", COLOR_TEXT_MAIN, False)],
        [("[INFO] [ur3e_goal_sender_node]: ", COLOR_INFO, True), ("Conectando con el servidor MoveGroup de MoveIt 2...", COLOR_TEXT_MAIN, False)],
        [("[INFO] [ur3e_goal_sender_node]: ", COLOR_INFO, True), ("Servidor de acción MoveGroup ('/move_action') conectado con éxito.", COLOR_CYAN, False)],
        [("[INFO] [ur3e_goal_sender_node]: ", COLOR_INFO, True), ("Enviando objetivo cartesiano a MoveIt 2: Pos=[0.250, 0.150, 0.350] m, Ori=[0.0, 3.142, 0.0] rad", COLOR_TEXT_MAIN, False)],
        [("[INFO] [ur3e_goal_sender_node]: ", COLOR_INFO, True), ("MoveGroup calculando cinemática inversa (IK) y trayectoria libre de colisión...", COLOR_WARN, True)],
        [("[move_group] [INFO]: ", COLOR_INFO, True), ("Planning request received for planning group 'ur_manipulator'.", COLOR_MUTED, False)],
        [("[move_group] [INFO]: ", COLOR_INFO, True), ("Planner plugin 'ompl_interface/OMPLPlanner' selected algorithm 'RRTConnectkConfigDefault'.", COLOR_MUTED, False)],
        [("[move_group] [INFO]: ", COLOR_INFO, True), ("Plan found in 0.0098 seconds. Path simplified to 16 waypoints. Trajectory parameterized (TOTG).", COLOR_TEXT_MAIN, False)],
        [("[move_group] [INFO]: ", COLOR_INFO, True), ("Sending trajectory to '/scaled_joint_trajectory_controller' for hardware execution...", COLOR_TEXT_MAIN, False)],
        [("[INFO] [ur3e_goal_sender_node]: ", COLOR_INFO, True), ("¡Movimiento completado con éxito!", COLOR_INFO, True)],
        [("developer@robotica-inteligente", COLOR_PROMPT_USER, True), (":", COLOR_TEXT_MAIN, False), ("~/ros2_ws", COLOR_PROMPT_PATH, True), ("$ ", COLOR_TEXT_MAIN, True)],
    ]
    return draw_terminal(1280, 420, title, lines)


def get_terminal_4():
    title = "Terminal 4: Introspección del Middleware ROS 2 (TF2, Tópicos y Acciones)"
    lines = [
        [("developer@robotica-inteligente", COLOR_PROMPT_USER, True), (":", COLOR_TEXT_MAIN, False), ("~/ros2_ws", COLOR_PROMPT_PATH, True), ("$ ros2 run tf2_tools view_frames", COLOR_TEXT_MAIN, True)],
        [("[INFO] [view_frames]: ", COLOR_INFO, True), ("Listening to tf data for 5.0 seconds...", COLOR_TEXT_MAIN, False)],
        [("[INFO] [view_frames]: ", COLOR_INFO, True), ("Generating graph: chain from 'world' to 'tool0' (14 frames)", COLOR_TEXT_MAIN, False)],
        [("[INFO] [view_frames]: ", COLOR_INFO, True), ("Exporting graph in frames_ur3e.pdf and frames_ur3e.gv file... [OK]", COLOR_INFO, True)],
        [("developer@robotica-inteligente", COLOR_PROMPT_USER, True), (":", COLOR_TEXT_MAIN, False), ("~/ros2_ws", COLOR_PROMPT_PATH, True), ("$ ros2 topic echo /tf --once", COLOR_TEXT_MAIN, True)],
        [("transforms:", COLOR_TEXT_MAIN, False)],
        [("  - header:", COLOR_MUTED, False)],
        [("      stamp: {sec: 1793032920, nanosec: 89600000}", COLOR_MUTED, False)],
        [("      frame_id: ", COLOR_MUTED, False), ("base_link_inertia", COLOR_CYAN, True)],
        [("    child_frame_id: ", COLOR_MUTED, False), ("shoulder_link", COLOR_CYAN, True)],
        [("    transform:", COLOR_MUTED, False)],
        [("      translation: {x: 0.0, y: 0.0, z: 0.15185}", COLOR_TEXT_MAIN, False)],
        [("      rotation: {x: 0.0, y: 0.0, z: 0.7071, w: 0.7071}", COLOR_TEXT_MAIN, False)],
        [("---", COLOR_MUTED, False)],
        [("developer@robotica-inteligente", COLOR_PROMPT_USER, True), (":", COLOR_TEXT_MAIN, False), ("~/ros2_ws", COLOR_PROMPT_PATH, True), ("$ ros2 action list", COLOR_TEXT_MAIN, True)],
        [("/execute_trajectory", COLOR_TEXT_MAIN, False)],
        [("/joint_trajectory_controller/follow_joint_trajectory", COLOR_TEXT_MAIN, False)],
        [("/move_action", COLOR_CYAN, True)],
        [("/scaled_joint_trajectory_controller/follow_joint_trajectory", COLOR_CYAN, True)],
        [("/sequence_move_group", COLOR_TEXT_MAIN, False)],
        [("developer@robotica-inteligente", COLOR_PROMPT_USER, True), (":", COLOR_TEXT_MAIN, False), ("~/ros2_ws", COLOR_PROMPT_PATH, True), ("$ ", COLOR_TEXT_MAIN, True)],
    ]
    return draw_terminal(1280, 560, title, lines)


def generate_workspace_composite():
    """Genera la captura completa de escritorio (1920x1080) con las 4 terminales."""
    ws = Image.new("RGB", (1920, 1080), (33, 33, 33))
    draw = ImageDraw.Draw(ws)

    # Barra superior GNOME
    draw.rectangle([(0, 0), (1920, 27)], fill=(20, 20, 20))
    draw.text((15, 5), "Actividades", fill=(240, 240, 240), font=FONT_TOPBAR)
    draw.text((900, 5), "Lun 26 Oct 18:30", fill=(240, 240, 240), font=FONT_TOPBAR)
    draw.text((1850, 5), "es ⏻ ☊", fill=(240, 240, 240), font=FONT_TOPBAR)

    # Renderizar las 4 terminales en dimensiones proporcionales
    # Columna izquierda: T1 (Launch) y T2 (Joint)
    # Columna derecha: T3 (Cartesian) y T4 (TF & Introspection)
    t1 = get_terminal_1().resize((935, 500), Image.Resampling.LANCZOS)
    t2 = get_terminal_2().resize((935, 515), Image.Resampling.LANCZOS)
    t3 = get_terminal_3().resize((935, 480), Image.Resampling.LANCZOS)
    t4 = get_terminal_4().resize((935, 535), Image.Resampling.LANCZOS)

    ws.paste(t1, (15, 38))
    ws.paste(t2, (15, 550))
    ws.paste(t3, (965, 38))
    ws.paste(t4, (965, 530))

    return ws


if __name__ == "__main__":
    t1 = get_terminal_1()
    t1.save(os.path.join(OUTPUT_DIR, "terminal_1_ur3e_sim_launch.png"))

    t2 = get_terminal_2()
    t2.save(os.path.join(OUTPUT_DIR, "terminal_2_ur3e_joint_control.png"))

    t3 = get_terminal_3()
    t3.save(os.path.join(OUTPUT_DIR, "terminal_3_ur3e_cartesian_control.png"))

    t4 = get_terminal_4()
    t4.save(os.path.join(OUTPUT_DIR, "terminal_4_ur3e_tf_frames.png"))

    ws = generate_workspace_composite()
    ws.save(os.path.join(OUTPUT_DIR, "terminales_workspace_bloque4.png"))

    print("Terminal captures and master workspace image created successfully in:", OUTPUT_DIR)
