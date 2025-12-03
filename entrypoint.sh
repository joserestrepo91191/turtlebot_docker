#!/usr/bin/env bash
# =====================================================================
# entrypoint.sh - Script de inicio para el contenedor Docker del TurtleBot2
# =====================================================================
# Este script asegura que siempre que el contenedor se ejecute:
#   1. Se cargue el entorno de ROS2 Humble.
#   2. Se compile el workspace del usuario (/ros2_ws).
#   3. Se configure el entorno del workspace.
#   4. Finalmente ejecute el comando pasado al contenedor.
# =====================================================================

set -e  # Hace que el script termine si ocurre algún error

# --------------------------------------------------
# 1. Cargar entorno base de ROS2 (instalado en la imagen)
# --------------------------------------------------
source /opt/ros/humble/setup.bash

# --------------------------------------------------
# 2. Ir al workspace del contenedor
# --------------------------------------------------
cd /ros2_ws
exec bash --rcfile <(echo "source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash;")


# --------------------------------------------------
# 3. Compilar el workspace (si ya está compilado, se salta rápidamente)
#    --symlink-install permite editar los archivos en host sin reinstalar
# --------------------------------------------------
colcon build --symlink-install

# --------------------------------------------------
# 4. Cargar entorno del workspace compilado
# --------------------------------------------------
if [ -f install/setup.bash ]; then
  source install/setup.bash
fi




exec bash
