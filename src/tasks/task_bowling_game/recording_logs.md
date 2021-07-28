# Recording Logs

- For recording the bags in this Task, you can refer to this [section](../task0/recording-logs.md).

- This launch file will specifically make use of `/gazebo/link_states` ROS topic so make sure to specify these as shown below.

- We recommend you to add the following in your `bowling_game_solution.launch` in order to record everything properly as soon as you start other nodes.

- Make sure to create a directory called `bag_files` within your `pkg_gazebo` package where your bag files will get stored.

```xml
<!-- Recording Bag File for Submission -->
    <arg name="record" default="false"/>
    <arg name="rec_name" default="bowling.bag"/>

    <group if="$(arg record)">
        <node name="rosbag_record_pick" pkg="rosbag" type="record"
       args="record -O $(find pkg_gazebo)/bag_files/$(arg rec_name) --chunksize=10 /gazebo/link_states" output="screen"/>
    </group>
 ```



- Verify that your bag file is properly recorded by using the `rosbag info` command followed by the absolute or relative path of the file.

- You can use the `rostopic echo -b bowling.bag /gazebo/link_states` command to display the messages from the topic onto your bag file.

- If you want to record the bag file needed for submission you would have to run the following command.

```bash
roslaunch pkg_gazebo bowling_game_solution.launch record:=true rec_name:=bowling.bag
```

- If you just want to run your implementation without recording a bag file. Do the following.

```bash
roslaunch pkg_gazebo bowling_game_solution.launch
```

---
