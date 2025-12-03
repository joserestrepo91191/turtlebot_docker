from setuptools import find_packages, setup

package_name = 'turtlebot_scripts'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'PID_cuadrado = turtlebot_scripts.PID_cuadrado:main', #profesor github
            'pid_controller = turtlebot_scripts.pid_controller:main',# controlador pid github
            'pid_cuadrado_v2 = turtlebot_scripts.pid_cuadrado_v2:main', #profesor modificado
            'pid_sin_waypoints = turtlebot_scripts.pid_sin_waypoints:main',
            'sintonizar_PID = turtlebot_scripts.sintonizar_PID:main',
            'control_PID = turtlebot_scripts.control_PID:main',#funcional 
            'control_PID_v2 = turtlebot_scripts.control_PID_v2:main',#funcional actual con el que se toman las muestras
            'pid_autotune_node = turtlebot_scripts.pid_autotune_node:main',
            'ticks = turtlebot_scripts.ticks:main',
            'open_loop_id_node = turtlebot_scripts.open_loop_id_node:main', #nodo para identificar FT del robot
            'control_lyapunov = turtlebot_scripts.control_lyapunov:main', # nodo para el control lyapunov
            'lyapunov_controller = turtlebot_scripts.lyapunov_controller:main',# controlador lyapunov, github
            'control_SERVO_v2 = turtlebot_scripts.control_SERVO_v2:main', # nodo para el control servo
            'servo_controller = turtlebot_scripts.servo_controller:main'# controlador servo
        ],
    },
)
