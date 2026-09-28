# 🤖 Robótica Inteligente - Práctica 1

Repositorio grupal para la **Práctica 1 de Robótica Inteligente (Curso 2026/2027)** - Grado en Ingeniería en Inteligencia Artificial, Escuela Politécnica Superior (Universidad de Alicante).

---

## 🎯 Objetivo de la Práctica
Despliegue de un entorno completo de robótica autónoma móvil sobre **ROS 2 (Jazzy Jalisco)** y **Gazebo Harmonic**:
1. Simulación y control de plataforma móvil (**iRobot Create 3 / TurtleBot 4**) con soporte de aceleración GPU NVIDIA.
2. Generación de mapas 2D del entorno mediante **SLAM (SLAM Toolbox)** y teleoperación.
3. Configuración y ejecución de la pila de navegación autónoma **Nav2** (localización AMCL, mapas de coste y planificadores/controladores de trayectoria).
4. Introspección y análisis del middleware ROS 2 (árboles de TF, nodos, tópicos y servicios).

---

## 📁 Estructura del Repositorio

```tree
.
├── Dockerfile                  # Entorno reproducible (ROS 2 Jazzy + Gazebo Harmonic + Nav2 + NVIDIA)
├── GUIA_EJECUCION.md           # Guía detallada paso a paso para desplegar simulación y navegación
├── README.md                   # Descripción general del proyecto
├── commands.txt                # Lista de comandos rápidos para terminales
├── diagramas/                  # Diagramas del sistema y árboles de transformaciones TF
│   ├── frames_nav2.gv
│   └── frames_nav2.pdf
├── enunciado/                  # Guiones oficiales y documentación de la práctica
│   ├── P1-guion.pdf
│   └── P1-introducción-prácticas.pdf
├── mapas/                      # Mapas generados mediante SLAM
│   ├── mapa_laboratorio.pgm
│   └── mapa_laboratorio.yaml
├── memoria/                    # Documentación técnica, análisis e informes
│   └── reporte_nav2_pipeline.md
└── scripts/                    # Scripts Python de soporte e interacción ROS 2
    ├── inspect_pipeline.py     # Inspección de nodos, tópicos y tipos del middleware
    ├── nav2_send_goal.py       # Envío programático de objetivos (goals) a Nav2
    ├── ur3e_send_pose.py       # Interacción con cinemática y poses del brazo UR3e
    └── warehouse_mapper.py     # Script de apoyo para mapeo
```

---

## ⚡ Inicio Rápido

Consulta la [Guía de Ejecución](GUIA_EJECUCION.md) para ver todos los detalles. Resumen básico:

### 1. Construir la imagen Docker
```bash
docker build -t robotica_inteligente_grado:latest .
```

### 2. Arrancar el contenedor con aceleración NVIDIA y GUI X11
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

## 👥 Integrantes del Grupo
- Grado en Ingeniería en Inteligencia Artificial (EPS - UA)
- Curso 4º (2026/2027) - Asignatura: *Robótica Inteligente*
