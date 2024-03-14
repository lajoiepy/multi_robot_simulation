```
sudo apt install python3-vcstool
vcs import src < frontiers_simulation.repos
```

```
source /opt/ros/humble/setup.zsh
sudo rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src --rosdistro $ROS_DISTRO
```