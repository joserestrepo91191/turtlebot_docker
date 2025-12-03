# TurtleBot Docker – ROS2 Humble (Jetson Nano + Kobuki)

Este repositorio contiene el entorno completo de desarrollo y ejecución para el **TurtleBot2** con base **Kobuki**, ejecutando **ROS2 Humble dentro de Docker**, con nodos personalizados para control, identificación de planta, seguimiento de trayectoria y experimentación avanzada.

El proyecto fue desarrollado sobre un **Jetson Nano**, usando **Docker Compose**, y contiene scripts, nodos y utilidades para pruebas reales del robot móvil.

---

## 🧭 Objetivos del proyecto

- Ejecutar ROS2 Humble en un entorno aislado mediante Docker.
- Desarrollar y probar nodos de control (PID, Servo–Integrador, Lyapunov).
- Realizar identificación lineal y angular del robot mediante datos reales.
- Capturar y almacenar registros experimentales en formato CSV.
- Diseñar trayectorias (recta, cuadrada, compuesta) y analizarlas.
- Mantener un entorno reproducible para investigación, pruebas y documentación.

---

# 🧱 Arquitectura del Sistema

Jetson Nano  
│  
├── Docker Engine  
│   └── Container ROS2 Humble  
│        ├── turtlebot_scripts (nodos ROS2)  
│        ├── python_nodos  
│        ├── Rviz2 / Teleop  
│        └── Comunicación con hardware Kobuki  
│  
└── Kobuki Base  
     ├── Motores  
     ├── Sensores  
     └── Odometría  

---

# 🐳 Cómo levantar el entorno Docker

```bash
cd ~/turtlebot_docker
docker-compose up --build
En segundo plano:

bash
Copiar código
docker-compose up -d
Ingresar al contenedor:

bash
Copiar código
docker exec -it turtlebot_container bash
Apagar contenedor:

bash
Copiar código
docker-compose down
🟦 Recomendación al ingresar al contenedor
Ejecutar siempre:

bash
Copiar código
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
colcon build --symlink-install
🤖 Cómo lanzar nodos ROS2
Los nodos se pueden ejecutar con parámetros personalizados usando --ros-args.

Control SERVO–integrador
bash
Copiar código
ros2 run turtlebot_scripts control_SERVO_v2 --ros-args -p traj:=cuadrada -p csv_name:=ServoTest
Control PID v2
bash
Copiar código
ros2 run turtlebot_scripts control_PID_v2 --ros-args -p traj:=cuadrada -p csv_name:=PID_ZN_1
Parámetros disponibles:

traj: recta | cuadrada | compuesta

csv_name: nombre del archivo CSV donde se guardará la prueba

📁 Organización de carpetas
bash
Copiar código
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
├── pruebas/   # CSV de pruebas reales
│
└── README.md
📜 Scripts relevantes
control_PID_v2.py
Controlador PID con:

Anti wind-up

Derivada filtrada

Saturaciones dinámicas

Exportación CSV

control_SERVO_v2.py
Implementación del regulador Servo–Integrador:

Ley discreta basada en estados anteriores

Eliminación de error estacionario

Ideal para trayectorias cuadradas

Trayectorias
pid_cuadrado_v2.py

pid_sin_waypoints.py

go_square.py

Incluyen:

Persecución pura

Lookahead ajustable

Rampa de frenado en esquinas

Registro CSV

Identificación
open_loop_id_node.py

Genera datos para modelos:

Planta lineal

Angular

FOPDT

ARX

Herramientas
grafica.py → gráficas tiempo real

sintonizar_PID.py → barrido de ganancias

ticks.py → manejo de encoders

📊 Datos experimentales
Los archivos CSV en /pruebas/ contienen logs de:

PID ZN

PID Cohen–Coon

Servo–integrador

Trayectorias rectas, cuadradas, compuestas

Energía

Curvatura y suavidad

RMSE / MAE / IAE / ITAE

🧪 Cómo generar una nueva prueba
bash
Copiar código
ros2 run turtlebot_scripts control_SERVO_v2 --ros-args -p traj:=cuadrada -p csv_name:=Test_SERVO
Los datos se guardarán automáticamente en /pruebas/.

👤 Autor
José Restrepo
Robótica Móvil · Control Automático · ROS2 · Docker
Politécnico Colombiano Jaime Isaza Cadavid

📝 Licencia
MIT — Uso académico y de investigación permitido.

yaml
Copiar código

---

Si quieres, te genero también:

✅ Una versión en inglés  
✅ Un logo minimalista para tu repositorio  
✅ Un diagrama Mermaid de arquitectura  
✅ Documentación extendida en `/docs/`

Solo dime cuál quieres.
