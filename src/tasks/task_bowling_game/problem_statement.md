# Problem Statement

- **Objective**: The objective of this task is to create a spherical bowling ball bot in urdf and use it to do an automatic strike in bowling game.

- **Bowling ball Specifications**

    Property | Value
    --- | --
    Mass | 8.0
    Radius | 0.2
    ixx | 0.03872
    ixy | 0.004
    ixz | 0.005
    iyy | 0.03872
    iyz | 0.003
    izz | 0.03872

- The ball needs to spawned at **(x=0, y=0, z=0.5)**.

    > **Note**: You're allowed to choose any color/texture for bowling ball.

- **Launching the game environment**

    `bowling_game.launch`

    ```xml
    <launch>

        <!-- these are the arguments you can pass this launch file, for example paused:=true -->
        <arg name="paused" default="false"/>
        <arg name="use_sim_time" default="true"/>
        <arg name="gui" default="true"/>
        <arg name="headless" default="false"/>
        <arg name="debug" default="false"/>
        <arg name="extra_gazebo_args" default="--verbose"/>

        <include file="$(find gazebo_ros)/launch/empty_world.launch">
            <arg name="world_name" value="$(find pkg_gazebo)/worlds/bowling_empty.world"/>
            <arg name="debug" value="$(arg debug)" />
            <arg name="gui" value="$(arg gui)" />
            <arg name="paused" value="$(arg paused)"/>
            <arg name="use_sim_time" value="$(arg use_sim_time)"/>
            <arg name="headless" value="$(arg headless)"/>
            <arg name="extra_gazebo_args" value="$(arg extra_gazebo_args)"/>
        </include>

    </launch>
    ```


    To load the bowling ball game environment. Launch this file `bowling_game.launch`

    ```bash
    roslaunch pkg_gazebo bowling_game.launch
    ```

## Procedure

1. You have to use the package `pkg_gazebo` we created in the [learn section](../../ROS_with_Gazebo/installation.html).

1. You need to create a spherical bowling ball using urdf file in `urdf directory` in `pkg_gazebo` package.

1. You would need to add **effort controllers** to control the bowling ball from the python script. 

1. Create a python script which would automatically control the bowling ball to do a strike.

1. Finally, use a launch file `bowling_game_solution.launch` to load the bowling game environment, load the bowling ball, and the required nodes to do an automatic strike.

    Your launch file `bowling_game_solution.launch` would look something like this.

    `bowling_game_solution.launch`

    ```xml
    <launch>

    <!-- these are the arguments you can pass this launch file, for example paused:=true -->
    <arg name="paused" default="false"/>
    <arg name="use_sim_time" default="true"/>
    <arg name="gui" default="true"/>
    <arg name="headless" default="false"/>
    <arg name="debug" default="false"/>
    <arg name="model" default="$(find pkg_gazebo)/urdf/bowling_ball_bot.urdf.xacro"/>
    <arg name="extra_gazebo_args" default="--verbose"/>

    <include file="$(find gazebo_ros)/launch/empty_world.launch">
        <arg name="world_name" value="$(find pkg_gazebo)/worlds/bowling_empty.world"/>
        <arg name="debug" value="$(arg debug)" />
        <arg name="gui" value="$(arg gui)" />
        <arg name="paused" value="$(arg paused)"/>
        <arg name="use_sim_time" value="$(arg use_sim_time)"/>
        <arg name="headless" value="$(arg headless)"/>
        <arg name="extra_gazebo_args" value="$(arg extra_gazebo_args)"/>
    </include>

    <param name="robot_description" command="$(find xacro)/xacro $(arg model)"/>
    <rosparam file="$(find pkg_gazebo)/config/effort_control.yaml" command="load"/>

    <node name="controller_spawner" pkg ="controller_manager" type="spawner" ns="/bowling_game" args="
    x_effort_joint_controller y_effort_joint_controller z_effort_joint_controller joint_state_controller"/>
    
    <!-- push robot_description to factory and spawn bowling ball in gazebo -->
    <node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model"
            args="-x 0 -y 0 -z 0.5 -unpause -urdf -model robot -param robot_description" respawn="false" output="screen" />

    <!-- node to control the bowling ball -->
    <node name="node_control_bowling_game" pkg="pkg_gazebo" type="node_control_bowling_ball.py"/>

    </launch>
    ```

    > **Note**: Your launch file could be different than this. But, keep the naming same as `bowling_game_solution.launch` as we would use this launch file to check your solution.

## Hint

- Try to make a **shoulder joint** in urdf so that rotation is possible in all directions. You can create some dummy links to do this.

- You could reset the world in gazebo for testing purposes.

> **Note**: Strike here means knocking down all the 10 pins using the bowling ball.

---
