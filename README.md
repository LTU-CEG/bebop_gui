**This package is the GUI for bebop parrot platforms, which can be extended to any robotic platforms.**

<img src="/image/GUI.png" width="600" height="400">

# How to make QT GUI

[Step by step guide to make QT GUI](https://github.com/LTU-CEG/bebop_gui/blob/master/presentation/step-by-step-guide.pdf)

# Dependency

Bebop Autonomy should be build in workspace

https://bebop-autonomy.readthedocs.io/en/latest/

# bebop_gui
Everything below has been tested and developed for Parrot Bebop 2.

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

# Features of GUI
* Go forward by pressing button or "w" on keyboard

* Go backward by pressing button or "s" on keyboard

* Turn right by pressing button or "d" on keyboard

* Turn left by pressing button or "a" on keyboard

* Hover by pressing button

* Takeoff by pressing button or "k" on keyboard

* Land by pressing button or "l" on keyboard

* Choose topis to record, in table under "Topics" tab, and press record to start recording (if no topics has been choosen all will be recorded).

* In table under "Topic frequencies" tab, enter prefered minimum publishing frequency of topic. If it is lower then prefered it will turn red.

# Video


![](/video/GUI.gif)
