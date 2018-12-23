# Lawn Mowing Robot

This simple lawn mowing robot simulation uses spawns a turtlebot in a Gazebo world and traverses a set of predetermined
locations.

## Getting Started

### Prerequisites

This simulation has been developed and tested using ROS Kinetic Kame and Ubuntu 16.04 LTS. Instructions on how to
install and set up ROS Kinetic can be found in ROS Wiki: http://wiki.ros.org/kinetic/Installation

### Cloning and Building

Navigate to /src directory your Catkin workspace:

```
$ cd /path/to/your/catkin/workspace/src
```

For example:

```
$ cd ~/catkin_ws/src
```

Clone repository:

```
$ git clone https://github.com/umutyazgan/lawn-mowing-robot.git
```

Navigate to root of your Catkin workspace and make a build:

```
$ cd ~/catkin_ws
$ catkin_make
```

Replace `~/catkin_ws` with path to your Catkin workspace.

## Running Simulation

Start `roscore` :

```
$ roscore
```

Use launch file provided in lawn_world package to launch Gazebo world with turtlebot, rviz, acml and move_base:

```
$ roslaunch lawn_world lawn.launch
```

Run `go_to_goals.py` script from lawn_explorer to start traversing through points given in script:

```
$ rosrun lawn_explorer go_to_goals.py
```

## Authors

* Umut Yazgan (https://github.com/umutyazgan)
* Orhan Kurto (https://github.com/kurtorhan)
* İbrahim Karahan (https://github.com/ibrahimkarahan47)

## License

This project is licensed under the GLPv3 - see the [LICENSE.md](LICENSE.md) file for details.
The goal seeking script [go_to_goals.py](lawn_explorer/src/go_to_goals.py) is lisenced under BSD lisence.

## Acknowledgments

This scrpit written by [markwsilliman](https://github.com/markwsilliman) provided us great help in this project:
https://github.com/markwsilliman/turtlebot/blob/master/go_to_specific_point_on_map.py