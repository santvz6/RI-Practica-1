#!/bin/bash
set -e

DEST_DIR="/home/sant_vz6/orca/workspaces/UA-4/robotica-inteligente/1-cuatrimestre/robotica-inteligente/practicas/1/memoria/capturas/recorrido"
SIGNAL_DIR="/home/sant_vz6/ros2_ws/p1_scripts/signals"

mkdir -p "$DEST_DIR"
mkdir -p "$SIGNAL_DIR"
rm -f "$SIGNAL_DIR"/keble_stage_*.ready

# Find RViz window ID
WIN_ID=$(xwininfo -root -tree | grep -i '("rviz2" "rviz2")' | grep -o "0x[0-9a-fA-F]*" | head -n 1)
if [ -z "$WIN_ID" ]; then
    echo "ERROR: RViz window not found!"
    exit 1
fi
echo "Target RViz window ID: $WIN_ID"

# Launch player inside container in background
docker exec -d robotica_inteligente_grado bash -c "source /opt/ros/jazzy/setup.bash && python3 /home/developer/ros2_ws/p1_scripts/keble_real_player.py"

STAGES=("1_inicializacion_t5s.png" "2_avance_claustro_t65s.png" "3_giro_patio_t140s.png" "4_reconstruccion_completa_t280s.png")

for i in {1..4}; do
    READY_FILE="$SIGNAL_DIR/keble_stage_${i}.ready"
    echo "Waiting for stage $i ready signal..."
    while [ ! -f "$READY_FILE" ]; do
        sleep 0.2
    done
    
    STAGE_INFO=$(cat "$READY_FILE")
    echo "Stage $i stabilized ($STAGE_INFO). Taking live window capture..."
    sleep 1.0
    
    TARGET_IMG="${DEST_DIR}/${STAGES[$((i-1))]}"
    xwd -id "$WIN_ID" -out "/tmp/stage_${i}.xwd"
    convert "xwd:/tmp/stage_${i}.xwd" "$TARGET_IMG"
    echo "Saved: $TARGET_IMG ($(ls -lh "$TARGET_IMG" | awk '{print $5}'))"
done

echo "All 4 recorrido screenshots successfully captured from live RViz window!"
