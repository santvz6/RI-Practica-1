# ============================================================================
# Base Image: Ubuntu 24.04
# ============================================================================
FROM ubuntu:24.04

# Evitar interacciones durante la instalación
ENV DEBIAN_FRONTEND=noninteractive

# ============================================================================
# 1. Configuración del Sistema y Locales
# ============================================================================
RUN apt-get update && apt-get install -y \
    locales \
    sudo \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# ============================================================================
# 2. Herramientas de Desarrollo Comunes y Dependencias Básicas
# ============================================================================
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    curl \
    gnupg2 \
    dirmngr \
    ca-certificates \
    lsb-release \
    wget \
    pkg-config \
    python3-pip \
    python3-flake8 \
    python3-pytest \
    python3-setuptools \
    terminator \
    tmux \
    vim \
    && rm -rf /var/lib/apt/lists/*

# ============================================================================
# 3. Repositorios Oficiales de ROS2 Jazzy y Gazebo Harmonic
# ============================================================================
RUN mkdir -p /root/.gnupg && chmod 700 /root/.gnupg

RUN gpg --no-default-keyring --keyring /usr/share/keyrings/ros-archive-keyring.gpg --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys F42ED6FBAB17C654 \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN gpg --no-default-keyring --keyring /usr/share/keyrings/pkgs.osrf.gpg --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 67170598AF249743 \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs.osrf.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# ============================================================================
# 4. Instalación de ROS2, Gazebo y Herramientas de Compilación
# ============================================================================
RUN apt-get update && apt-get install -y \
    ros-jazzy-desktop \
    ros-jazzy-ros-gz \
    gz-harmonic \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# ============================================================================
# 5. Dependencias de Navegación (Nav2) y Ecosistema TurtleBot4
# ============================================================================
RUN apt-get update && apt-get install -y \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-robot-localization \
    ros-jazzy-twist-mux \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-turtlebot4-simulator \
    ros-jazzy-turtlebot4-navigation \
    ros-jazzy-turtlebot4-description \
    ros-jazzy-turtlebot4-viz \
    && rm -rf /var/lib/apt/lists/*

# ============================================================================
# 6. Instalación de RKO-LIO
# ============================================================================
RUN apt-get update && apt-get install -y \
    ros-jazzy-rko-lio \
    && rm -rf /var/lib/apt/lists/*

# ============================================================================
# 7. Instalación completa de la pila de Manipulación Industrial (UR3e + MoveIt2)
# ============================================================================
RUN apt-get update && apt-get install -y \
    ros-jazzy-ur \
    ros-jazzy-ur-simulation-gz \
    ros-jazzy-moveit \
    && rm -rf /var/lib/apt/lists/*

# Inicializar rosdep
RUN rosdep init && rosdep update

# ============================================================================
# 8. Creación de Usuario No-Root (CORREGIDO PARA UBUNTU 24.04)
# ============================================================================
ARG USERNAME=developer
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Eliminamos el usuario 'ubuntu' por defecto de la imagen para liberar el UID/GID 1000
RUN if getent passwd ubuntu; then userdel -r ubuntu; fi \
    && groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# ============================================================================
# 9. Configuración del Entorno y Variables del TurtleBot4
# ============================================================================
USER $USERNAME
WORKDIR /home/$USERNAME

# Crear el workspace de ROS2 para el alumnado
RUN mkdir -p /home/$USERNAME/ros2_ws/src

# Inyectar configuraciones críticas en el .bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "export TURTLEBOT4_MODEL=standard" >> /home/$USERNAME/.bashrc \
    && echo "export GZ_VERSION=harmonic" >> /home/$USERNAME/.bashrc \
    && echo "if [ -f /home/$USERNAME/ros2_ws/install/setup.bash ]; then source /home/$USERNAME/ros2_ws/install/setup.bash; fi" >> /home/$USERNAME/.bashrc \
    && echo "export QT_X11_NO_MITSHM=1" >> /home/$USERNAME/.bashrc

# Establecer el directorio de trabajo por defecto en el workspace
WORKDIR /home/$USERNAME/ros2_ws

CMD ["bash"]
