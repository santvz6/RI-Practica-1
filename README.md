# Robótica Inteligente - Práctica 1

Repositorio para la **Práctica 1 de Robótica Inteligente (Curso 2026/2027)** - Grado en Ingeniería en Inteligencia Artificial, Escuela Politécnica Superior (Universidad de Alicante).

---

## Objetivo

Despliegue y análisis de un entorno de robótica sobre **ROS 2 (Jazzy Jalisco)** y **Gazebo Harmonic**:
1. Simulación y control de plataforma móvil (**TurtleBot 4**) con aceleración por GPU.
2. Generación de mapas 2D del entorno mediante **SLAM Toolbox** y teleoperación.
3. Configuración y ejecución de la pila de navegación autónoma **Nav2** (localización AMCL, mapas de coste y controladores).
4. Odometría LiDAR-Inercial con datos reales (**RKO-LIO** sobre el Oxford Spires Dataset).
5. Manipulación robótica industrial con brazo articulado **Universal Robots UR3e** y **MoveIt 2**.
6. Introspección y análisis del middleware ROS 2 (árboles de TF, grafos de nodos, tópicos, servicios y acciones).

---

## Estructura del Repositorio

```text
.
├── Dockerfile                  # Entorno de ejecución (ROS 2 Jazzy, Gazebo Harmonic, Nav2, MoveIt 2, RKO-LIO)
├── GUIA_EJECUCION.md           # Guía de ejecución paso a paso de los bloques de la práctica
├── README.md                   # Descripción general del proyecto
├── commands.txt                # Lista de comandos rápidos para terminales
├── oxford_spires.yaml          # Configuración de calibración extrínseca para Oxford Spires Dataset
├── diagramas/                  # Diagramas del sistema y árboles de transformaciones TF
│   ├── frames_nav2.gv
│   ├── frames_nav2.pdf
│   ├── frames_rko_lio.gv
│   ├── frames_rko_lio.pdf
│   ├── frames_rko_lio.png
│   ├── rqt_graph_rko_lio.gv
│   ├── rqt_graph_rko_lio.pdf
│   └── rqt_graph_rko_lio.png
├── enunciado/                  # Guiones oficiales de la práctica
│   ├── P1-guion.pdf
│   └── P1-introducción-prácticas.pdf
├── mapas/                      # Mapas generados mediante SLAM
│   ├── mapa_laboratorio.pgm
│   └── mapa_laboratorio.yaml
├── memoria/                    # Documentación técnica, análisis e informes
│   ├── reporte_nav2_pipeline.md
│   └── reporte_rko_lio_pipeline.md
└── scripts/                    # Scripts Python de soporte e interacción ROS 2
    ├── inspect_pipeline.py     # Inspección de nodos, tópicos y tipos del middleware
    ├── nav2_send_goal.py       # Envío programático de objetivos a Nav2 (Opcional 1)
    ├── ur3e_send_pose.py       # Control cartesiano y articular del brazo UR3e (Opcional 2)
    └── warehouse_mapper.py     # Script de apoyo para mapeo del almacén
```

---

## Guía Rápida de Despliegue

Consulta la [Guía de Ejecución](GUIA_EJECUCION.md) para ver el flujo detallado de cada bloque.

### 1. Construir la imagen Docker
```bash
docker build -t robotica_inteligente_grado:latest .
```

### 2. Arrancar el contenedor con aceleración gráfica y GUI X11
```bash
xhost +local:docker

docker run -d \
  --user developer \
  --network host \
  --ipc host \
  --pid host \
  --env DISPLAY=$DISPLAY \
  --env QT_X11_NO_MITSHM=1 \
  --env NVIDIA_VISIBLE_DEVICES=all \
  --env NVIDIA_DRIVER_CAPABILITIES=all \
  --env __NV_PRIME_RENDER_OFFLOAD=1 \
  --env __GLX_VENDOR_LIBRARY_NAME=nvidia \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --volume ~/ros2_ws:/home/developer/ros2_ws:rw \
  --device /dev/dri \
  --gpus all \
  --runtime nvidia \
  --name robotica_inteligente_grado \
  robotica_inteligente_grado:latest \
  sleep infinity
```

### 3. Entrar al contenedor
```bash
docker exec -it robotica_inteligente_grado bash
```

---

## Autores
- Grado en Ingeniería en Inteligencia Artificial (EPS - UA)
- Curso 4º (2026/2027) - Asignatura: *Robótica Inteligente*
