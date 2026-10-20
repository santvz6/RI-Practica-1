#!/usr/bin/env python3
"""
Genera una figura académica con un grid completo de subplots (2x4)
mostrando la evolución temporal y espacial del recorrido completo de LIO en Keble College.
"""

import os
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

RECORRIDO_DIR = "/home/sant_vz6/orca/workspaces/UA-4/robotica-inteligente/1-cuatrimestre/robotica-inteligente/practicas/1/memoria/capturas/recorrido"
OUTPUT_GRID = os.path.join(RECORRIDO_DIR, "grid_recorrido_completo.png")

stages_info = [
    ("stage_1_origen_t0s.png", "1. Origen y Calibración inicial (t=0s)", "Pose 0 | X=0.0m, Y=0.0m"),
    ("stage_2_ala_sur_t47s.png", "2. Avance por el Ala Sur (t=47s)", "Pose 120 | X=42.6m, Y=-8.1m"),
    ("stage_3_giro_este_capilla_t78s.png", "3. Giro hacia la Capilla (t=78s)", "Pose 220 | X=59.8m, Y=18.2m"),
    ("stage_4_esquina_noreste_t112s.png", "4. Esquina Noreste (t=112s)", "Pose 340 | X=50.8m, Y=48.9m"),
    ("stage_5_ala_norte_t148s.png", "5. Tránsito por el Ala Norte (t=148s)", "Pose 460 | X=7.1m, Y=49.3m"),
    ("stage_6_esquina_noroeste_t185s.png", "6. Esquina Noroeste (t=185s)", "Pose 580 | X=0.5m, Y=43.1m"),
    ("stage_7_ala_oeste_retorno_t222s.png", "7. Retorno Sur por Ala Oeste (t=222s)", "Pose 690 | X=1.3m, Y=23.5m"),
    ("stage_8_cierre_bucle_t265s.png", "8. Cierre de Bucle Completo (t=265s)", "Pose 804 | X=20.0m, Y=-2.2m")
]

fig, axes = plt.subplots(2, 4, figsize=(24, 12), dpi=180)
plt.subplots_adjust(wspace=0.08, hspace=0.15, top=0.93, bottom=0.04, left=0.02, right=0.98)

for idx, (filename, title, subtitle) in enumerate(stages_info):
    row = idx // 4
    col = idx % 4
    ax = axes[row, col]
    
    img_path = os.path.join(RECORRIDO_DIR, filename)
    if os.path.exists(img_path):
        img = mpimg.imread(img_path)
        ax.imshow(img)
    else:
        ax.text(0.5, 0.5, f"Captura no encontrada:\n{filename}", ha='center', va='center')
    
    ax.set_title(f"{title}\n{subtitle}", fontsize=11, fontweight='bold', pad=6)
    ax.axis('off')

fig.suptitle("Evolución Temporal del Recorrido Perimetral Completo (RKO-LIO - Oxford Spires Dataset)", fontsize=16, fontweight='bold', y=0.98)
plt.savefig(OUTPUT_GRID, bbox_inches='tight', dpi=180)
print(f"Master grid saved successfully to: {OUTPUT_GRID}")
