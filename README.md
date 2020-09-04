<img src="/image/GUI.png" width="600" height="400">



# Dependency

Bebop Autonomy should be build in workspace

https://bebop-autonomy.readthedocs.io/en/latest/

# bebop_gui

$ sudo apt-get install build-essential python-rosdep python-catkin-tools

# Create and initialize the workspace
$ mkdir -p ~/bebop_ws/src && cd ~/bebop_ws

$ catkin init

$ git clone https://github.com/AutonomyLab/bebop_autonomy.git src/bebop_autonomy

Update rosdep database and install dependencies (including parrot_arsdk)

$ rosdep update

$ rosdep install --from-paths src -i

# Build the workspace
$ catkin build

$ source ~/bebop_ws/devel/setup.[bash|zsh]

$ roslaunch bebop_driver bebop_node.launch

# Open new terminal
$ roscd bebop_ws

$ chmod +x *.py

$ python bebop_gui.py

# How to make QT GUI

[Step by step guide to make QT GUI](https://github.com/LTU-CEG/bebop_gui/blob/master/presentation/step-by-step-guide.pdf)

# Video


![](/video/GUI.gif)
