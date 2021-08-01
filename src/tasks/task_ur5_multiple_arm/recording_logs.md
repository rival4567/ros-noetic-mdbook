# Recording Logs

- For recording the bags in this Task, you can refer to this [section](../task0/recording-logs.md).

- This launch file will specifically make use of `/eyrc/vb/ur5_1/vacuum_gripper/logical_camera/ur5_1` and `/eyrc/vb/ur5_2/vacuum_gripper/logical_camera/ur5_2` ROS topics so make sure to specify these as shown below.

- We recommend you to add the following in your `task_sort.launch` in order to record everything properly as soon as you start other nodes.

- Make sure to create a directory called `bag_files` within your `pkg_vb_solution` package where your bag files will get stored.

    ```xml
    <!-- Recording Bag File for Submission -->
        <arg name="record" default="false"/>
        <arg name="rec_name" default="vb.bag"/>

        <group if="$(arg record)">
            <node name="rosbag_record_pick" pkg="rosbag" type="record" args="record -O $(find pkg_vb_solution)/bag_files/$(arg rec_name) --chunksize=10 /eyrc/vb/ur5_1/vacuum_gripper/logical_camera/ur5_1 /eyrc/vb/ur5_2/vacuum_gripper/logical_camera/ur5_2" output="screen"/>
        </group>

    ```


- Verify that your bag file is properly recorded by using the `rosbag info` command followed by the absolute or relative path of the file.

- You can use the `rostopic echo -b vb.bag /eyrc/vb/ur5_1/vacuum_gripper/logical_camera/ur5_1` command to display the messages from the topic onto your bag file.

- If you want to record the bag file needed for submission you would have to run the following command.

    ```bash
    roslaunch pkg_vb_solution task_sort.launch record:=true rec_name:=vb.bag
    ```

- If you just want to run your implementation without recording a bag file. Do the following.

    ```bash
    roslaunch pkg_vb_solution task_sort.launch
    ```

---
