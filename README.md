# dl-project

# Comandos
* Correrlo en la Raspery Pi (1-4)
* Opcional (simulación): Correrlo en Local (1-2) y luego el simulador

## 0. Clonar repositorio en Local o Raspery Pi
```
git clone https://github.com/andrewkc/dl-project.git
```

## 1. Activar variables de entorno de ROS2
```
source /opt/ros/humble/setup.bash
```
## 2. Compilar paquete
```
cd ~/dl-project/r2_ws
colcon build
```
## 3. Bring up basic packages
```
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_bringup robot.launch.py
```
## 4. Correr el nodo
```
cd ~/dl-project/r2_ws/
colcon build
source ~/dl-project/r2_ws/install/setup.bash
ros2 run obstacle_avoidance obstacle_avoidance
```
## 5. Kill node
```
# Ctrl + C
ps aux | grep turtlebot3_node
kill -9 <PID>
```
ps aux | grep turtlebot3_node
kill -9 PID