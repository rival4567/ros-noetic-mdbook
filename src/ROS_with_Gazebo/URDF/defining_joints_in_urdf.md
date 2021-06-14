# Building a Movable Robot Model with URDF

In this tutorial, we’re going to revise the model we made in the previous tutorial so that it has movable joints.

Below is the new urdf with flexible joints. You can compare it to the previous version to see everything that has changed.

```xml
<?xml version="1.0"?>

<robot name="myrobot">
    
    <material name="blue">
        <color rgba="0 0 0.8 1"/>
    </material>
    <material name="white">
        <color rgba="1 1 1 1"/>
    </material>
    <material name="black">
        <color rgba="0 0 0 1"/>
    </material>


    <link name="base_link">
        <visual>
            <origin rpy="0 0 0" xyz="0 0 0.5"/>
            <geometry>
                <box size="0.8 1.8 0.6"/>
            </geometry>
            <material name="blue"/>
        </visual>
    </link>

    <link name="right_front_wheel">
        <visual>
            <origin rpy="0 1.57075 0" xyz="0 0 0"/>
            <geometry>
                <cylinder length="0.2" radius="0.3"/>
            </geometry>
            <material name="black"/>
        </visual>
    </link>
    <joint name="right_front_wheel_joint" type="continuous">
        <axis rpy="0 0 0" xyz="1 0 0"/>
        <parent link="base_link"/>
        <child link="right_front_wheel"/>
        <origin rpy="0 0 0" xyz="0.4 0.5 0.3"/>
    </joint>

    <link name="left_front_wheel">
        <visual>
            <origin rpy="0 1.57075 0" xyz="0 0 0"/>
            <geometry>
                <cylinder length="0.2" radius="0.3"/>
            </geometry>
            <material name="black"/>
        </visual>
    </link>
    <joint name="left_front_wheel_joint" type="continuous">
        <axis rpy="0 0 0" xyz="1 0 0"/>
        <parent link="base_link"/>
        <child link="left_front_wheel"/>
        <origin rpy="0 0 0" xyz="-0.4 0.5 0.3"/>
    </joint>

    <link name="right_back_wheel">
        <visual>
            <origin rpy="0 1.57075 0" xyz="0 0 0"/>
            <geometry>
                <cylinder length="0.2" radius="0.3"/>
            </geometry>
            <material name="black"/>
        </visual>
    </link>
    <joint name="right_back_wheel_joint" type="continuous">
        <axis rpy="0 0 0" xyz="1 0 0"/>
        <parent link="base_link"/>
        <child link="right_back_wheel"/>
        <origin rpy="0 0 0" xyz="0.4 -0.5 0.3"/>
    </joint>

    <link name="left_back_wheel">
        <visual>
            <origin rpy="0 1.57075 0" xyz="0 0 0"/>
            <geometry>
                <cylinder length="0.2" radius="0.3"/>
            </geometry>
            <material name="black"/>
        </visual>
    </link>
    <joint name="left_back_wheel_joint" type="continuous">
        <axis rpy="0 0 0" xyz="1 0 0"/>
        <parent link="base_link"/>
        <child link="left_back_wheel"/>
        <origin rpy="0 0 0" xyz="-0.4 -0.5 0.3"/>
    </joint>
</robot>

```

Lets take look at the `right_front_wheel_joint`:

```xml
<joint name="right_front_wheel_joint" type="continuous">
    <axis rpy="0 0 0" xyz="1 0 0"/>
    <parent link="base_link"/>
    <child link="right_front_wheel"/>
    <origin rpy="0 0 0" xyz="0.4 0.5 0.3"/>
</joint>
```

the connection between the wheel and the body is a continuous joint, meaning that it can take on any angle from negative infinity to positive infinity, so that they can roll in both directions forever.

The only additional information we have to add is the axis of rotation, here specified by an xyz triplet, which specifies a vector around which the head will rotate. Since we want it to go around the X axis we specify the vector "1 0 0".

To visualize and control this model, run the same command as the last tutorial:

```bash
roslaunch urdf_tutorial display.launch
```

However now this will also pop up a GUI that allows you to control the values of all the non-fixed joints. Play with the model some and see how it moves.

## Other Types of Joints

There are other kinds of joints that move around in space. Whereas the prismatic joint can only move along one dimension, a planar joint can move around in a plane, or two dimensions, a revolute joint rotate in the same way that the continuous joints do, but they have strict limits. Hence, we must include the limit tag specifying the upper and lower limits of the joint (in radians). We also must specify a maximum velocity and effort for this joint but the actual values don't matter for our purposes here. Furthermore, a floating joint is unconstrained, and can move around in any of the three dimensions.

<hr>