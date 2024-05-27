# localization_manager

## Installation
```
cd [YOUR_WORKSPACE]/src
git clone https://github.com/MinSungjae/localization_manager
cd [YOUR_WORKSPACE]
catkin_make
```

## Usage 
```
roslaunch localization_manager localization_manager.launch
```

## Input arguments
```
world_frame_name:=[Frame name where managed localization(odometry) will represent]

relative_frame_name:=[Name of relative odometry(ORB odometry) frame w.r.t. world frame]
```