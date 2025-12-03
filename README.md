# TurtleBot Docker – ROS2 Humble (Jetson Nano + Kobuki)

Este repositorio contiene el entorno completo de desarrollo y ejecución para el **TurtleBot2** con base **Kobuki**, ejecutando **ROS2 Humble dentro de Docker**, con nodos personalizados para control, identificación de planta, seguimiento de trayectoria y experimentación avanzada.

---

##  Objetivos del proyecto

- Ejecutar ROS2 Humble en un entorno aislado mediante Docker.
- Desarrollar y probar nodos de control (PID, Servo–Integrador, Lyapunov).
- Realizar identificación lineal y angular del robot mediante datos reales.
- Capturar y almacenar registros experimentales en formato CSV.
- Diseñar trayectorias (recta, cuadrada, compuesta) y analizarlas.
- Mantener un entorno reproducible para investigación.

---

##  Arquitectura del Sistema

```
Jetson Nano
│
├── Docker Engine
│   └── Container ROS2 Humble
│        ├── turtlebot_scripts (nodos ROS2)
│        ├── python_nodos
│        ├── Rviz2 / Teleop / Control
│        └── Comunicación con hardware Kobuki
│
└── Kobuki Base
     ├── Motores (velocidad lineal + angular)
     ├── Sensores (bumper, corriente, batería)
     └── Odometría
```

---

##  Cómo levantar el entorno Docker

```bash
cd ~/turtlebot_docker
docker-compose up --build
docker-compose up -d
docker exec -it turtlebot_container bash
docker-compose down
```

---

##  Recomendación al ingresar al contenedor

```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
colcon build --symlink-install
```

---

##  Cómo lanzar nodos ROS2

Control SERVO–integrador:

```bash
ros2 run turtlebot_scripts control_SERVO_v2 --ros-args -p traj:=cuadrada -p csv_name:=ServoTest
```

Control PID v2:

```bash
ros2 run turtlebot_scripts control_PID_v2 --ros-args -p traj:=cuadrada -p csv_name:=PID_ZN_1
```

---

##  Organización de carpetas

```
turtlebot_docker/
│
├── docker-compose.yml
├── dockerfile
├── entrypoint.sh
│
├── turtlebot_scripts/
│   ├── control_PID.py
│   ├── control_PID_v2.py
│   ├── control_SERVO_v2.py
│   ├── control_lyapunov.py
│   ├── pid_cuadrado_v2.py
│   ├── pid_autotune_node.py
│   ├── open_loop_id_node.py
│   ├── sintonizar_PID.py
│   └── ...
│
├── ros2host_ws/
│   └── src/turtlebot_scripts/
│
├── pruebas/
│   ├── data_CC_PID_1.csv
│   ├── data_CC_PID_2.csv
│   └── ...
│
└── README.md
```

---

##  Scripts relevantes

### control_PID_v2.py
- Anti wind-up  
- Derivada filtrada  
- Saturaciones dinámicas  
- Exportación CSV  

### control_SERVO_v2.py
- Ley discreta basada en estados previos  
- Eliminación del error estacionario  

### Trayectorias
- `pid_cuadrado_v2.py`
- `pid_sin_waypoints.py`
- `go_square.py`

### Identificación
- `open_loop_id_node.py`  
- Modelos FOPDT / ARX  

### Utilidades
- `grafica.py`, `sintonizar_PID.py`, `ticks.py`

---

##  Ejemplo de prueba

```bash
ros2 run turtlebot_scripts control_SERVO_v2 --ros-args -p traj:=cuadrada -p csv_name:=Test_SERVO
```

---

## 👤 Autor

José Restrepo  
Politécnico Colombiano Jaime Isaza Cadavid

---


