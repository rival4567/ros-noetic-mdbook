# Autoeval script

> **Note**: Save this file in a ROS package scripts folder. Then to run this script, run in terminal rosrun autoeval.py. This script prints the result in the terminal window.

`autoeval.py`

```python
#!/usr/bin/env python3
import glob
import rosbag
import math
import csv

# USAGE: rosrun pkg_task2 autoeval.py
# This file will print results on the terminal window

def main():
	bag_files = []
	is_passed = False
	# Change this path to where all bag files are located.
	path = "/home/ubuntu/workspace/src/eysip/line_follower/bagfiles"
	for file in glob.glob(path + "/*.bag"):
		bag_files.append(file)

	# iterate through each file
	for file in bag_files:
		bag = rosbag.Bag(file)
		# look throuhg topics
		topics = bag.get_type_and_topic_info()[1].keys()
		if '/gazebo/link_states' in topics:
			is_topic_present = True
		else:
			is_topic_present = False
			print('topic not present')
			is_passed = False

		for topic, msg, t in bag.read_messages(topics=['/gazebo/link_states']):
			if (len(msg.pose) < 9):
				continue
			if (abs(msg.pose[9].position.x-(-3)) <= 0.2 and abs(msg.pose[9].position.y-(-6.387296170000643)) <= 0.2):
				is_passed = True

		if is_topic_present == True and is_passed == True:
			print(file + ' Result: passed!')
		else:
			print(file + ' Result: failed!')

if __name__ == '__main__':
	main()
```

<center><a href="tasks/task_line_follower/autoeval.py" download><button>Download</button></a></center>

<hr>