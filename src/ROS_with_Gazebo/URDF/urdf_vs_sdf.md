# Difference between URDF and SDF

## What is URDF

The URDF (Universal Robot Description Format) model is a collection of files that describe a robot’s physical description to ROS. These files are used by a program called ROS (Robot Operating System) to tell the computer what the robot actually looks like in real life.

## What is SDF

SDFormat (Simulation Description Format), sometimes abbreviated as SDF, is an XML format that describes objects and environments for robot simulators, visualization, and control. Originally developed as part of the Gazebo robot simulator, SDFormat was designed with scientific robot applications in mind.

## what is the difference

URDF specifies a robot, but SDF also specifies a world for the robot to live in, which is a much larger set of things. Based on this premise, SDF is designed to represent a superset of everything that can be represented in URDF. URDF is the established format for describing robot structure in ROS. URDF could not specify information necessary for other robotics domains. SDF was devised by Gazebo to meet simulation needs, but Gazebo can consume URDF when it is augmented by information within `<gazebo>` tags.

You can learn more about why sdf was developed for gazebo [here](http://gazebosim.org/tutorials?tut=ros_urdf).

### Reference

  - [URDF vs. Gazebo SDF](https://newscrewdriver.com/2018/07/31/ros-notes-urdf-vs-gazebo-sdf/)