# Gazebo Plugins

Gazebo plugins give your URDF models greater functionality and can tie in ROS messages and service calls for sensor output and motor input. You can use both preexisting plugins and create your own custom plugins that can work with ROS.

## Plugin Types

Gazebo supports several plugin types, and all of them can be connected to ROS, but only a few types can be referenced through a URDF file:

  1. [ModelPlugins](http://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1ModelPlugin.html), to provide access to the [physics::Model](http://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1physics_1_1Model.html) API
  2. [SensorPlugins](http://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1SensorPlugin.html), to provide access to the [sensors::Sensor](http://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1sensors_1_1Sensor.html) API
  3. [VisualPlugins](http://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1VisualPlugin.html), to provide access to the [rendering::Visual](http://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1rendering_1_1Visual.html) API

  You can read more about it [here](http://gazebosim.org/tutorials?tut=ros_gzplugins)

<hr>