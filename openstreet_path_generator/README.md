# Path planning on OpenStreetMap

These repository allows you to plan path using OpenStreetMap. It can also work with the GY-GPS6MV2 GPS module and use its own location as the starting point of the planned path. It is also possible to filter the readings from the GPS module using the Kalman filter. The path planner mainly uses already defined roads, adding more only when the start or destination point is outside of them.

## Installation
First, install ROS. This repository was tested on Ubuntu 18.04 with ROS MELODIC (http://wiki.ros.org/melodic/Installation/Ubuntu).

Next step is create workspace, download and build repository:
```
mkdir -p catkin_gps/src
cd catkin_gps/src/
git clone https://github.com/Michal-Bidzinski/OpenStreetMap_path_generator.git
cd ..
catkin_make
```

All requirements can be install by command:
```
pip install -r src/OpenStreetMap_path_generator/requirements.txt
```

Python's library:
 - rospy
 - serial
 - re
 - numpy
 - pykalman
 - matplotlib
 - copy

Now package is prepare for use.

In each terminal you need write:
```
source devel/setup.bash
```

## Run

# GPS module

To start work with GPS module:

```bash
source devel/setup.bash
./src/OpenStreetMap_path_generator/scripts/prepare_gps.sh 
```
To run simple GPS program to get coordinates:
```bash
roslaunch OpenStreetMap_path_generator gps_coordinates.launch
```
Published topics:
 - /gps_point_lat_lng, type: NavSatFix

To run GPS program with kalman filter:
```bash
roslaunch OpenStreetMap_path_generator gps_coordinates_kalman_filter.launch
```
Published topics:
 - /gps_point_lat_lng, type: NavSatFix

# OpenStreetMap servis

In each roslaunch users can change area of map by devine diffrent bbox.

Run osm_path_example - program show example path from start point to end point(points can be changed in roslaunch file):
```bash
roslaunch osm_server osm_path_example.launch
```
Published topics:
 - /waypoints, type: PoseArray (message definition in msg folder, this is array of coordinates in UTM)

Run osm_path_gps - program get gps coordinates and coordinates of end point and find shoortest path:
```bash
roslaunch osm_server osm_path_gps.launch
```
Subscribed topics:
 - /gps_point_lat_lng, type: NavSatFix
 - /end_point_lat_lng, type: NavSatFix

Published topics:
 - /waypoints, type: PoseArray (message definition in msg folder, this is array of coordinates in UTM)


