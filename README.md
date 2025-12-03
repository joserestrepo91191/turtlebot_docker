# TurtleBot Docker – ROS2 Humble (Jetson Nano + Kobuki)

Este repositorio contiene el entorno completo de desarrollo y ejecución para el **TurtleBot2** con base **Kobuki**, ejecutando **ROS2 Humble dentro de Docker**, con nodos personalizados para control, identificación de planta, seguimiento de trayectoria y experimentación avanzada.

---

##  Objetivos del proyecto

- Ejecutar ROS2 Humble en un entorno aislado mediante Docker.
- Desarrollar y probar nodos de control (PID, Servo–Integrador).
- Realizar identificación lineal y angular del robot mediante datos reales.
- Capturar y almacenar registros experimentales en formato CSV.
- Diseñar trayectorias (recta, cuadrada, compuesta) y analizarlas.
- Mantener un entorno reproducible para investigación.

---

##  Cómo clonar este repositorio

Asegúrate de tener **git** instalado:

```bash
sudo apt update
sudo apt install git -y
```

Clona el repositorio:

```bash
git clone https://github.com/joserestrepo91191/turtlebot_docker.git
```

Ingresa a la carpeta:

```bash
cd turtlebot_docker
```

---

##  Instalación inicial de Docker (solo primera vez)

### 1️ Instalar dependencias
```bash
sudo apt update
sudo apt install ca-certificates curl gnupg lsb-release -y
```

### 2️ Agregar clave oficial de Docker
```bash
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
```

### 3️ Agregar repositorio de Docker
```bash
echo   "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg]   https://download.docker.com/linux/ubuntu   $(lsb_release -cs) stable" |   sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
```

### 4️ Instalar Docker Engine
```bash
sudo apt update
sudo apt install docker-ce docker-ce-cli containerd.io -y
```

### 5️ Permitir Docker sin sudo
```bash
sudo usermod -aG docker $USER
```

Reinicia tu Jetson:

```bash
sudo reboot
```

---

##  Instalación de Docker Compose (solo primera vez)

```bash
sudo apt install docker-compose -y
```

Verifica:

```bash
docker-compose --version
```

---

##  Arquitectura del Sistema

```
Jetson Nano
│
├── Docker Engine
│   └── Container ROS2 Humble
│        ├── turtlebot_scripts (nodos ROS2)
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

Parámetros útiles:

```
traj: recta | cuadrada | compuesta  
csv_name: nombre del archivo CSV generado  
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
│   ├── control_PID.py #Nodo antiguo PID
│   ├── control_PID_v2.py # Nodo funcional para este proyecto
│   ├── control_SERVO_v2.py # Nodo funcional para este proyecto
│   ├── open_loop_id_node.py
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

### Identificación
- open_loop_id_node.py  

---

##  Ejemplo de prueba

```bash
ros2 run turtlebot_scripts control_SERVO_v2 --ros-args -p traj:=cuadrada -p csv_name:=Test_SERVO
```

---

##  Autor

José Restrepo    
Politécnico Colombiano Jaime Isaza Cadavid

---

