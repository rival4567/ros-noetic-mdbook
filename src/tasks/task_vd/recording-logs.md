# Recording Logs

- Create a folder called bagfiles in your package as a save destination for the bag files.

- You can run the rosbag record command separately on the command line. But to not loose any data you will have to start recording precisely at the same moment your drone starts moving. Hence it is a much more preferable option to include the rosbag recording in your launch file itself.

- Add the following lines to your launch file to have the rosbag record run in parallel with your task.

```xml
<arg name="record" default="false"/>
<arg name="duration" default="120"/>
<arg name="rec_name" default="record.bag"/>

<group if="$(arg record)">
    <node name="rosbag_record_drone" pkg="rosbag" type="record" args="record -O $(find vitarana_drone)/bagfiles/$(arg rec_name) --duration=$(arg duration) --chunksize=10 /edrone/gps" output="screen" />
</group>
```

- launch the launch file with the following command:

```bash
roslaunch vitarana_drone vitarana_drone.launch record:=true rec_name:=drone_record.bag
```

<hr>