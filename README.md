# priority_scheduler
This project is for my Masters thesis topic: Cooperative Docking Control and Priority Scheduling for Multi-Robot Autonomy

The objective of this research is to demonstrate the feasibility of an autonomous docking controller for multiple robots with a singular charging station that allows each robot to charge without running out of power before reaching the dock by utilizing a priority scheduler that uses a fuzzy-logic based ranking and queuing system.

This repository contains the priority_scheduler portion package of the project. The autonomous docking controller can be found in the docking_control repository located at https://github.com/HadrienRoy/docking_control.git.

## Quick Start
```sh
mkdir priority_scheduler_ws/src
cd ~/priority_scheduler_ws/src
git clone https://github.com/HadrienRoy/priority_scheduler.git
cd ..
colcon build --symlink-install
```

## Usage
Run the priority scheduler 
```sh
ros2 launch docking_scheduler schduler
```
