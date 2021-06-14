# Adding Physical and Collision Properties to a URDF Model

In this tutorial, we’ll look at how to add some basic physical properties to your URDF model and how to specify its collision properties.

## 1. Collision

So far, we’ve only specified our links with a single sub-element, visual, which defines what the robot looks like. However, in order to get collision detection to work or to simulate the robot in something like Gazebo, we need to define a collision element as well.

Here is the code for our new base link. 

```xml
<link name="base_link">
    <visual>
        <origin rpy="0 0 0" xyz="0 0 0.5"/>
        <geometry>
            <box size="0.8 1.8 0.6"/>
        </geometry>
        <material name="blue"/>
    </visual>
    <collision>
        <geometry>
            <box size="0.8 1.8 0.6"/>
        </geometry>
    </collision>
</link>
```

  - The collision element is a direct subelement of the link object, at the same level as the visual tag 
  - The collision element defines its shape the same way the visual element does, with a geometry tag. The format for the geometry tag is exactly the same here as with the visual.
  - You can also specify an origin in the same way as a subelement of the collision tag (as with the visual) 

In many cases, you’ll want the collision geometry and origin to be exactly the same as the visual geometry and origin. However, there are two main cases where you wouldn’t. 

  - Quicker Processing - Doing collision detection for two meshes is a lot more computational complex than for two simple geometries. Hence, you may want to replace the meshes with simpler geometries in the collision element. 
  - Safe Zones - You may want to restrict movement close to sensitive equipment. For instance, if we didn’t want anything to collide with R2D2’s head, we might define the collision geometry to be a cylinder encasing his head to prevent anything from getting to near his head.

## 2. Physical Properties

In order to get your model to simulate properly, you need to define several physical properties of your robot, i.e. the properties that a physics engine like Gazebo would need.

### 2.1 Inertia

Every link element being simulated needs an inertial tag. Here is a simple one.

```xml
<link name="base_link">
    <visual>
        <origin rpy="0 0 0" xyz="0 0 0.5"/>
        <geometry>
            <box size="0.8 1.8 0.6"/>
        </geometry>
        <material name="blue"/>
    </visual>
    <collision>
        <geometry>
            <box size="0.8 1.8 0.6"/>
        </geometry>
    </collision>
    <inertial>
        <mass value="10"/>
        <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
</link>
```

  - This element is also a subelement of the link object. 
  - The mass is defined in kilograms. 
  - The 3x3 rotational inertia matrix is specified with the inertia element. Since this is symmetrical, it can be represented by only 6 elements, as such.

**ixx** **ixy** **ixz**

ixy **iyy** **iyz**

ixz iyz **izz**

  - This information can be provided to you by modeling programs such as MeshLab. The inertia of geometric primitives (cylinder, box, sphere) can be computed using Wikipedia's [list of moment of inertia tensors](https://en.wikipedia.org/wiki/List_of_moments_of_inertia#List_of_3D_inertia_tensors).
  - The inertia tensor depends on both the mass and the distribution of mass of the object. A good first approximation is to assume equal distribution of mass in the volume of the object and compute the inertia tensor based on the object's shape.
  - If unsure what to put, a matrix with ixx/iyy/izz=1e-3 or smaller is often a reasonable default for a mid-sized link (it corresponds to a box of 0.1 m side length with a mass of 0.6 kg). Although often chosen, the identity matrix is a particularly bad default, since it is often much too high (it corresponds to a box of 0.1 m side length with a mass of 600 kg!).
  - You can also specify an origin tag to specify the center of gravity and the inertial reference frame (relative to the link's reference frame).
  - When using realtime controllers, inertia elements of zero (or almost zero) can cause the robot model to collapse without warning, and all links will appear with their origins coinciding with the world origin.

### 2.2 Contact Coefficients

You can also define how the links behave when they are in contact with one another. This is done with a subelement of the collision tag called contact_coefficients. There are three attributes to specify:

  - mu - [Friction coefficient](https://simple.wikipedia.org/wiki/Coefficient_of_friction)
  - kp - [Stiffness coefficient](https://en.wikipedia.org/wiki/Stiffness)
  - kd - [Dampening coefficient](https://en.wikipedia.org/wiki/Damping_ratio#Definition)

### 2.3 Joint Dynamics

How the joint moves is defined by the dynamics tag for the joint. There are two attributes here:

  - friction - The physical static friction. For prismatic joints, the units are Newtons. For revolving joints, the units are Newton meters.
  - damping - The physical damping value. For prismatic joints, the units are Newton seconds per meter. For revolving joints, Newton meter secons per radian.

If not specified, these coefficients default to zero. 

## 3. Other Tags

In the realm of pure URDF (i.e. excluding Gazebo-specific tags), there are two remaining tags to help define the joints: calibration and safety controller. Check out the [spec](http://wiki.ros.org/urdf/XML/joint), as they are not included in this tutorial.

## Final urdf file

This is the final URDF file with collision and physical properties.

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
        <collision>
            <geometry>
                <box size="0.8 1.8 0.6"/>
            </geometry>
        </collision>
        <inertial>
            <mass value="10"/>
            <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
        </inertial>
    </link>

    <link name="right_front_wheel">
        <visual>
            <origin rpy="0 1.57075 0" xyz="0 0 0"/>
            <geometry>
                <cylinder length="0.2" radius="0.3"/>
            </geometry>
            <material name="black"/>
        </visual>
        <collision>
            <geometry>
                <cylinder length="0.2" radius="0.3"/>
            </geometry>
        </collision>
        <inertial>
            <mass value="1"/>
            <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
        </inertial>
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
        <collision>
            <geometry>
                <cylinder length="0.2" radius="0.3"/>
            </geometry>
        </collision>
        <inertial>
            <mass value="1"/>
            <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
        </inertial>
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
        <collision>
            <geometry>
                <cylinder length="0.2" radius="0.3"/>
            </geometry>
        </collision>
        <inertial>
            <mass value="1"/>
            <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
        </inertial>
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
        <collision>
            <geometry>
                <cylinder length="0.2" radius="0.3"/>
            </geometry>
        </collision>
        <inertial>
            <mass value="1"/>
            <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
        </inertial>
    </link>
    <joint name="left_back_wheel_joint" type="continuous">
        <axis rpy="0 0 0" xyz="1 0 0"/>
        <parent link="base_link"/>
        <child link="left_back_wheel"/>
        <origin rpy="0 0 0" xyz="-0.4 -0.5 0.3"/>
    </joint>
</robot>

```
<hr>