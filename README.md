# MRS Precise Landing

Autonomous precise landing on an AprilTag.

## Installation

For deployment:
`sudo apt install ros-noetic-mrs-precise-landing`

For simulations:
`sudo apt install ros-noetic-mrs-precise-landing-gazebo`

## Dependencies

* `apriltag_ros`
* `uav_core`

## Components

* AprilTag detector
* [Landing Pad Estimator](./ros_packages/mrs_landing_pad_estimation)
* [Precise Landing Conroller](./ros_packages/mrs_precise_landing)
* [Gazebo Resources for testing](./ros_packages/mrs_precise_landing_gazebo)

## Example simulation

```bash
roscd mrs_precise_landing_gazebo
./tmux/recursive/start.sh
```

## How to use it

You will need:
* [config file](./ros_packages/mrs_precise_landing_gazebo/tmux/recursive/config/apriltag.yaml) for the april tag detector
* camera node name, camera topic name (such that `/$UAV_NAME/camera_node/camera_topic`)
* the ids of the AprilTags need to be filled in the custom config for the estimator, [estimator_config](./ros_packages/mrs_precise_landing_gazebo/tmux/recursive/config/landing_estimator.yaml)
* how should the heading be controlled, prepare the controller config, [controller_config](./ros_packages/mrs_precise_landing_gazebo/tmux/recursive/config/landing_controller.yaml)

```bash
roslaunch mrs_precise_landing precise_landing.launch apriltag_config:=`rospack find mrs_landing_pad_estimation`/config/apriltag_recursive.yaml camera_node:=bluefox_optflow image_topic:=image_raw estimator_config:=<estimator_config> controller_config:=<controller_config>
```

When the AprilTag is in the view, call:
```bash
rosservice call /$UAV_NAME/precise_landing/land
```

To abort, call:
```bash
rosservice call /$UAV_NAME/precise_landing/abort
```
