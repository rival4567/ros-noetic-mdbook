# Create a Simple mobile Robot in Gazebo

The objective is to create a simple diff-drive robot with a camera plugin installed.

For example:

![model_image](./model/png)

As you can see in the above image, the robot contains 4 wheels and 1 camera at the head location.

	Note: You dont have to make the exact model shown in the image, any mobile robot with a camera installed and ability to move in a 2-d plane will be sufficient.

## Adding the camera and camera plugin

To add a camera simply create `camera` link with the shape of a box and attach it to the `base_link` of your model with a fixed joint.

To add the camera plugin you can simple add the following line in your urdf file.

```xml
<gazebo reference="camera">
    <material>Gazebo/Green</material>
    <sensor type="camera" name="camera1">
        <update_rate>30.0</update_rate>
        <camera name="head">
            <horizontal_fov>1.3962634</horizontal_fov>
            <image>
                <width>800</width>
                <height>800</height>
                <format>R8G8B8</format>
            </image>
            <clip>
                <near>0.02</near>
                <far>300</far>
            </clip>
        </camera>
        <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
            <alwaysOn>true</alwaysOn>
            <updateRate>0.0</updateRate>
            <cameraName>follower/camera1</cameraName>
            <imageTopicName>image_raw</imageTopicName>
            <cameraInfoTopicName>camera_info</cameraInfoTopicName>
            <frameName>camera</frameName>
            <hackBaseline>0.07</hackBaseline>
            <distortionK1>0.0</distortionK1>
            <distortionK2>0.0</distortionK2>
            <distortionK3>0.0</distortionK3>
            <distortionT1>0.0</distortionT1>
            <distortionT2>0.0</distortionT2>
        </plugin>
    </sensor>
</gazebo>
```

you can read more about this [here](http://gazebosim.org/tutorials?tut=ros_gzplugins)

<hr>