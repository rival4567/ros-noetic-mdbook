# Using Xacro to Clean Up a URDF File

By now, if you’re following all these steps at home with your own robot design, you might be sick of doing all sorts of math to get very simple robot descriptions to parse correctly. Fortunately, you can use the [xacro](http://wiki.ros.org/xacro) package to make your life simpler. It does three things that are very helpful.

  - Constants
  - Simple Math
  - Macros 

In this tutorial, we take a look at all these shortcuts to help reduce the overall size of the URDF file and make it easier to read and maintain.

## 1. Using Xacro

At the top of the URDF file, you must specify a namespace in order for the file to parse properly. For example, these are the first two lines of a valid xacro file:

```xml
<?xml version="1.0"?>

<robot name="myrobot" xmlns:xacro="http://ros.org/wiki/xacro">
```

## 2. Constants

Let’s take a quick look at our base_link

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

The information here is a little redundant. We specify the length and breadth and height of the box twice. Worse, if we want to change that, we need to do so in two different places.

Fortunately, xacro allows you to specify properties which act as constants. Instead, of the above code, we can write this.

```xml
<xacro:property name="width" value="0.8" />
<xacro:property name="bodylen" value="1.8" />
<xacro:property name="height" value="0.6" />

<link name="base_link">
    <visual>
        <origin rpy="0 0 0" xyz="0 0 0.5"/>
        <geometry>
            <box size="${width} ${bodylen} ${height}"/>
        </geometry>
        <material name="blue"/>
    </visual>
    <collision>
        <geometry>
            <box size="${width} ${bodylen} ${height}"/>
        </geometry>
    </collision>
</link>
```

  - The values are specified in the first three lines. They can be defined just about anywhere (assuming valid XML), at any level, before or after they are used. Usually they go at the top.
  - Instead of specifying the actual width in the geometry element, we use a dollar sign and curly brackets to signify the value.

The value of the contents of the ${} construct are then used to replace the ${}. This means you can combine it with other text in the attribute.

For example: 

```xml
<xacro:property name="robotname" value="marvin" />
<link name="${robotname}s_leg" />
```

This will generate

```xml
<link name="marvins_leg" />
```

However, the contents in the ${} don’t have to only be a property, which brings us to our next point... 

## 3. Math

You can build up arbitrarily complex expressions in the ${} construct using the four basic operations, the unary minus, and parenthesis. Examples:

```xml
<cylinder radius="${wheeldiam/2}" length="0.1"/>
<origin xyz="${reflect*(width+.02)} 0 0.25" />
```

All of the math is done using floats, hence

```xml
<link name="${5/6}"/>
```

evaluates to

```bash
<link name="0.833333333333"/>
```

In [Jade](http://wiki.ros.org/xacro#Math_expressions) and later distros, you can use more than the basic operations listed above, notably sin and cos.

## 4. Macros

Here’s the biggest and most useful component to the xacro package.

### 4.1 Simple Macro

Let’s take a look at a simple useless macro.

```xml
<xacro:macro name="default_origin">
	<origin xyz="0 0 0" rpy="0 0 0"/>
</xacro:macro>
<xacro:default_origin />
```

(This is useless, since if the origin is not specified, it has the same value as this.) This code will generate the following. 

```xml
<origin rpy="0 0 0" xyz="0 0 0"/>
```

  - The name is not technically a required element, but you need to specify it to be able to use it.
  - Every instance of the `<xacro:$NAME />` is replaced with the contents of the xacro:macro tag.
  - If the xacro with a specified name is not found, it will not be expanded and will NOT generate an error.

### 4.2 Parameterized Macro

You can also parameterize macros so that they don’t generate the same exact text every time. When combined with the math functionality, this is even more powerful.

First, let’s take an example of a simple macro we will use in our model.

```xml
<xacro:macro name="default_inertial" params="mass">
	<inertial>
		<mass value="${mass}" />
		<inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0" />
	</inertial>
</xacro:macro>
```

This can be used with the code 

```xml
<xacro:default_inertial mass="10"/>
```

The parameters act just like properties, and you can use them in expressions

You can also use entire blocks as parameters too.

```xml
<xacro:macro name="blue_shape" params="name *shape">
	<link name="${name}">
 		<visual>
			<geometry>
				<xacro:insert_block name="shape" />
			</geometry>
			<material name="blue"/>
		</visual>
		<collision>
			<geometry>
				<xacro:insert_block name="shape" />
			</geometry>
		</collision>
	</link>
</xacro:macro>

<xacro:blue_shape name="base_link">
	<cylinder radius=".42" length=".01" />
</xacro:blue_shape>
```

  - To specify a block parameter, include an asterisk before its parameter name.
  - A block can be inserted using the insert_block command 
  - Insert the block as many times as you wish. 

## 5. Practical Usage

The xacro language is rather flexible in what it allows you to do. This is the macroed version of our urdf model made in previous tutorials simply called `macroed.urdf.xacro`.

```xml
<?xml version="1.0"?>

<robot name="myrobot" xmlns:xacro="http://ros.org/wiki/xacro">
    
    <xacro:property name="width" value="0.8" />
    <xacro:property name="bodylen" value="1.8" />
    <xacro:property name="height" value="0.6" />
    <xacro:property name="wheellen" value="0.2" />
    <xacro:property name="wheeldiam" value="0.6" />
    <xacro:property name="pi" value="3.1415" />

    <material name="blue">
        <color rgba="0 0 0.8 1"/>
    </material>
    <material name="white">
        <color rgba="1 1 1 1"/>
    </material>
    <material name="black">
        <color rgba="0 0 0 1"/>
    </material>

    <xacro:macro name="default_inertial" params="mass">
        <inertial>
            <mass value="${mass}" />
            <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0" />
        </inertial>
    </xacro:macro>

    <link name="base_link">
        <visual>
            <origin rpy="0 0 0" xyz="0 0 0.5"/>
            <geometry>
                <box size="${width} ${bodylen} ${height}"/>
            </geometry>
            <material name="blue"/>
        </visual>
        <collision>
            <geometry>
                <box size="${width} ${bodylen} ${height}"/>
            </geometry>
        </collision>
        <xacro:default_inertial mass="10"/>
    </link>

     <xacro:macro name="wheel" params="prefix suffix reflect_x reflect_y">
        <link name="${prefix}_${suffix}_wheel">
            <visual>
                <origin xyz="0 0 0" rpy="0 ${pi/2} 0" />
                <geometry>
                    <cylinder radius="${wheeldiam/2}" length="${wheellen}"/>
                </geometry>
                <material name="black"/>
            </visual>
            <collision>
                <origin xyz="0 0 0" rpy="0 ${pi/2} 0" />
                <geometry>
                    <cylinder radius="${wheeldiam/2}" length="${wheellen}"/>
                </geometry>
            </collision>
            <xacro:default_inertial mass="1"/>
        </link>
        <joint name="${prefix}_${suffix}_wheel_joint" type="continuous">
            <axis xyz="1 0 0" rpy="0 0 0" />
            <parent link="base_link"/>
            <child link="${prefix}_${suffix}_wheel"/>
            <origin xyz="${reflect_x*width/2} ${reflect_y*bodylen/4} ${wheeldiam/2}" rpy="0 0 0"/>
        </joint>
    </xacro:macro>

    <xacro:wheel prefix="right" suffix="front" reflect_x="1" reflect_y="1" />
    <xacro:wheel prefix="left" suffix="front" reflect_x="-1" reflect_y="1" />
    <xacro:wheel prefix="right" suffix="back" reflect_x="1" reflect_y="-1" />
    <xacro:wheel prefix="left" suffix="back" reflect_x="-1" reflect_y="-1" />
</robot>

```

To see the model generated by a xacro file, run the following command:

first make sure you are in your package directory.

```bash
roscd myrobot_description
```

and then run:

```bash
roslaunch myrobot_description display.launch model:=urdf/macroed.urdf.xacro
```

This should give the same output as our previous tutorial.

<hr>