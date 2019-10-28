# Canny Edge
Simple canny edge detector package with ROS.

## Goal
Have a canny edge algorithm working with ROS reading images directly from camera.


## Dependencies

This application depends heavily on ROS Melodic (running on Ubuntu 18.04), rviz and the package `cv_camera`, a system wide install of ROS and rviz can be done following the [official instructions](http://wiki.ros.org/melodic/Installation/Ubuntu). If not installed by default, `cv_camera` can be installed with:

```
$ sudo apt install ros-melodic-cv-camera
```

## Building

On catkin root, run:

```
$ catkin_make
```

## Running

To run the application simply run:

```
$ roslaunch canny_edge_my_face canny_edge.launch
```

Additionally to settip up the nodes, the launch script initializes the param `canny_threshold` (default: 40).

## Design

The package was designed with simplicity at mind, provided mainly by the python script `canny_edge_detector.py`, which listens for messages coming from the topic `/cv_camera/image_raw` and outputs the filtered image to the topic `canny_edge`.

The filter works by reading the published image from camera, applying a Gaussian Filter 3x3 on the image with the goal to produce a mask image with smooth components, from which the canny edge filter is applied. Having the canny mask, it is applied over the original image to create the canny one, which is published for further visualization.

The quality and sensivity of the canny edge can be adjusted by setting the parameter `canny_threshold` as:
```
$ rosparam set /canny_edge <value>
```

## Results

Results for a some test runs with varying degrees of threshold can be found a the folder `Results`.

## License

MIT.
