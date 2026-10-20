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

STAGE_FILES=(
    "stage_1_origen_t0s.png"
    "stage_2_ala_sur_t47s.png"
    "stage_3_giro_este_capilla_t78s.png"
    "stage_4_esquina_noreste_t112s.png"
    "stage_5_ala_norte_t148s.png"
    "stage_6_esquina_noroeste_t185s.png"
    "stage_7_ala_oeste_retorno_t222s.png"
    "stage_8_cierre_bucle_t265s.png"
)

for i in {1..8}; do
    READY_FILE="$SIGNAL_DIR/keble_stage_${i}.ready"
    echo "Waiting for stage $i ready signal..."
    while [ ! -f "$READY_FILE" ]; do
        sleep 0.15
    done
    
    STAGE_INFO=$(cat "$READY_FILE")
    echo "Stage $i stabilized ($STAGE_INFO). Taking live window capture..."
    sleep 0.6
    
    TARGET_IMG="${DEST_DIR}/${STAGE_FILES[$((i-1))]}"
    xwd -id "$WIN_ID" -out "/tmp/stage_${i}.xwd"
    convert "xwd:/tmp/stage_${i}.xwd" "$TARGET_IMG"
    echo "Saved: $TARGET_IMG ($(ls -lh "$TARGET_IMG" | awk '{print $5}'))"
done

echo "Generating master subplot grid..."
python3 /home/sant_vz6/orca/workspaces/UA-4/robotica-inteligente/1-cuatrimestre/robotica-inteligente/practicas/1/scripts/generate_grid_subplots.py

echo "All 8 recorrido screenshots and master subplot grid successfully created!"
