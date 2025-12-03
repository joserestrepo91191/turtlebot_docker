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


# --------------------------------------------------
# 5. (Opcional) Lanzar directamente un nodo o un launch
#    ⚠️ Te recomiendo comentar/ajustar esta línea si quieres flexibilidad
# --------------------------------------------------
#ros2 launch turtlebot2_nav nav2.launch.py namespace:=tb2_5

# --------------------------------------------------
# 6. Ejecutar el comando que se pase al contenedor
#    Ejemplo: docker exec -it turtlebot2 bash
#    Esto asegura que se pueda entrar al bash o correr ros2 run
# --------------------------------------------------
#exec "$@"

exec bash
