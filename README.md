# Multi-Robot Frontier-based Exploration Simulation

In first terminal:
```
sudo apt install python3-vcstool
git clone https://github.com/lajoiepy/multi_robot_simulation.git
cd multi_robot_simulation
vcs import src < frontiers_simulation.repos
cd docker/exploration
make build
make run
make attach
colcon build
ros2 launch multi_robot_simulation multi_robot_simulation_warehouse.launch.py
```

In second terminal:
```
cd multi_robot_simulation/docker/exploration
make attach
ros2 launch multi_robot_simulation multi_robot_frontiers_exploration.launch.py
```