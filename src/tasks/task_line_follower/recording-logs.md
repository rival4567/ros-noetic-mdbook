# Recording Logs

- Create a folder called bagfiles in your package as a save destination for the bag files.

- Add the following lines to your launch file to have the rosbag record run in parallel with your task.

```xml
<arg name="record" default="false"/>
<arg name="duration" default="180"/>
<arg name="rec_name" default="record.bag"/>

<group if="$(arg record)">
    <node name="rosbag_record_drone" pkg="rosbag" type="record" args="record -O $(find line_follower)/bagfiles/$(arg rec_name) --duration=$(arg duration) --chunksize=10 /gazebo/link_states" output="screen" />
</group>
```

- Launch the launch file with the following command:

```bash
roslaunch line_follower gazebo.launch record:=true rec_name:=follower_record.bag
```

<hr>