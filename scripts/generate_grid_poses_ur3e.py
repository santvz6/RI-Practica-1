#!/usr/bin/env python3
"""
=============================================================================
Práctica 1 - Robótica Inteligente (Grado en IA - EPS Universidad de Alicante)
Genera una figura académica con un grid de subplots (2x3)
mostrando las distintas configuraciones cinemáticas y ejecuciones del UR3e.
=============================================================================
"""

import os
from PIL import Image
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

UR3E_DIR = "/home/sant_vz6/orca/workspaces/UA-4/robotica-inteligente/1-cuatrimestre/robotica-inteligente/practicas/1/memoria/capturas/ur3e"
OUTPUT_GRID = os.path.join(UR3E_DIR, "grid_poses_ur3e.png")

# Cortar la zona central 3D del robot de RViz para mayor detalle visual
# Dimensiones de RViz: 1920x948. El robot está en x: 750..1350, y: 250..800
def get_cropped_rviz(filename):
    src_path = os.path.join(UR3E_DIR, filename)
    if not os.path.exists(src_path):
        return None
    img = Image.open(src_path)
    # Recortar área 3D enfocada en el brazo
    crop_box = (700, 260, 1400, 780)
    cropped = img.crop(crop_box)
    crop_path = os.path.join("/tmp", "cropped_" + filename)
    cropped.save(crop_path)
    return crop_path


poses_info = [
    (
        "rviz2_initial.png",
        "1. Estado Inicial en RViz2",
        "Visualización URDF, Marcador Interactivo y Goal State",
        "Articular: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] rad"
    ),
    (
        "rviz2_joint_pose1.png",
        "2. Control Articular - Codo Flexionado",
        "Comando directo a Joint Trajectory Controller",
        "Articular: [0.0, -1.57, 1.57, -1.57, -1.57, 0.0] rad"
    ),
    (
        "rviz2_cartesian_pose.png",
        "3. Control Cinemático Cartesiano (IK + OMPL)",
        "Meta en espacio 3D resuelta por MoveGroup",
        "Cartesiano: X=0.25m, Y=0.15m, Z=0.35m | Roll=0, Pitch=π, Yaw=0"
    ),
    (
        "rviz2_home_pose.png",
        "4. Pose Predefinida SRDF: 'home'",
        "Posición de reposo horizontal estándar industrial",
        "Articular: [0.0, -1.571, 0.0, 0.0, 0.0, 0.0] rad"
    ),
    (
        "rviz2_up_pose.png",
        "5. Pose Predefinida SRDF: 'up'",
        "Extensión vertical máxima sobre eje Z",
        "Articular: [0.0, -1.571, 0.0, -1.571, 0.0, 0.0] rad"
    ),
    (
        "rviz2_test_config.png",
        "6. Pose Predefinida SRDF: 'test_configuration'",
        "Configuración de trabajo asimétrica multi-articular",
        "Articular: [1.54, -1.62, 1.40, -1.20, -1.60, -0.11] rad"
    ),
]

fig, axes = plt.subplots(2, 3, figsize=(21, 13), dpi=180)
plt.subplots_adjust(wspace=0.08, hspace=0.18, top=0.91, bottom=0.04, left=0.03, right=0.97)

for idx, (filename, title, subtitle, param_text) in enumerate(poses_info):
    row = idx // 3
    col = idx % 3
    ax = axes[row, col]

    cropped_path = get_cropped_rviz(filename)
    if cropped_path and os.path.exists(cropped_path):
        img = mpimg.imread(cropped_path)
        ax.imshow(img)
    else:
        ax.text(0.5, 0.5, f"Captura no encontrada:\n{filename}", ha='center', va='center')

    ax.set_title(f"{title}\n{subtitle}\n{param_text}", fontsize=10.5, fontweight='bold', pad=7)
    ax.axis('off')

fig.suptitle("Evolución de Poses y Cinemática del Manipulador UR3e (MoveIt 2 + ros2_control)", fontsize=16, fontweight='bold', y=0.97)
plt.savefig(OUTPUT_GRID, bbox_inches='tight', dpi=180)
print(f"Master poses grid saved successfully to: {OUTPUT_GRID}")
