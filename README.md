# Multi-Robot Frontier-based Exploration Simulation

In first terminal:
```
git clone https://github.com/lajoiepy/multi_robot_simulation.git
cd multi_robot_simulation/docker/devel
make build
make run
make attach
ros2 launch multi_robot_simulation multi_robot_simulation_warehouse.launch.py
```

In second terminal:
```
cd multi_robot_simulation/docker/devel
make attach
ros2 launch multi_robot_simulation multi_robot_frontiers_exploration.launch.py
```