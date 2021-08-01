# Solution

- Complete solution is in this zip file => <a href="pkg_gazebo.zip" download><button>Download</button></a>

- To run the solution, launch this file.

    ```bash
    roslaunch pkg_gazebo bowling_game_solution.launch
    ```

<br/>

This file defines the effort controller to move the bowling ball in 3-axis.

`effort_control.yaml`

```yaml
bowling_game:
  # Publish all joint states -----------------------------------
  joint_state_controller:
    type: "joint_state_controller/JointStateController"
    publish_rate: 50

  # Position Controllers ---------------------------------------
  x_effort_joint_controller:
    type: "effort_controllers/JointEffortController"
    joint: rotate_x
    pid: {p: 100.0, i: 0.01, d: 10.0}

  y_effort_joint_controller:
    type: "effort_controllers/JointEffortController"
    joint: rotate_y
    pid: {p: 100.0, i: 0.01, d: 10.0}

  z_effort_joint_controller:
    type: "effort_controllers/JointEffortController"
    joint: rotate_z
    pid: {p: 100.0, i: 0.01, d: 10.0}

```

<center><a href="effort_control.yaml" download><button>Download</button></a></center>

<br/>

This node controls the bowling ball to do a Strike!

`node_control_bowling_ball.py`


```python
#! /usr/bin/env python3

import rospy

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

import math


class BowlingBall:
    def __init__(self):
        rospy.init_node('control_bowling_ball', anonymous=True)

        self.sub_ = rospy.Subscriber(
            '/bowling_game/joint_states', JointState, self.clbk_pos)

        self.pub_x = rospy.Publisher(
            '/bowling_game/x_effort_joint_controller/command', Float64, queue_size=1)

        self.pub_y = rospy.Publisher(
            '/bowling_game/y_effort_joint_controller/command', Float64, queue_size=1)

        self.pub_z = rospy.Publisher(
            '/bowling_game/z_effort_joint_controller/command', Float64, queue_size=1)

        self.ball_state = JointState()
        self.pos_y = 0
        self.rate = rospy.Rate(20)  # Rate in Hz

    def clbk_pos(self, msg):
        self.ball_state.position = msg.position
        self.pos_y = msg.position[1]

    def knock_pins(self):
        # Infinite Loop
        i = 0
        while not rospy.is_shutdown():
            if i < 5:
                self.pub_x.publish(3.5)
            print(self.pos_y)
            if self.pos_y < -0.1:
                self.pub_y.publish(-0.5)
            elif self.pos_y > -0.1:
                self.pub_y.publish(0.5)
            self.rate.sleep()

    def __del__(self):
        self.sub_.unregister()
        rospy.loginfo("Object of class BowlingBall deleted.")


def main():
    b = BowlingBall()
    b.knock_pins()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

```

<center><a href="node_control_bowling_ball.py" download><button>Download</button></a></center>

<br/>

This node shows the video of the bowling ball coming towards the bowling pins. Looks cool!

`node_bowling_pin_cam.py`

```python
#! /usr/bin/env python3

import rospy

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError
import math
import cv2


class DepthCamera(object):
    def __init__(self):
        rospy.init_node('node_live_cam', anonymous=True)

        self.bridge = CvBridge()

        self.sub_cam = rospy.Subscriber(
            "/depth_camera/color/image_raw", Image, self.clbk_cam)

        self.image = None
        self.rate = rospy.Rate(20)  # Rate in Hz

    def clbk_cam(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as err:
            rospy.logerr(err)

        self.image = cv_image
        cv2.imshow("bowling pin cam", self.image)
        cv2.waitKey(1)

    def work(self):
        # Infinite Loop
        while not rospy.is_shutdown():
            self.rate.sleep()

    def __del__(self):
        self.sub_cam.unregister()
        rospy.loginfo("Object of class BowlingBall deleted.")


def main():
    dc = DepthCamera()
    dc.work()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

```

<center><a href="node_bowling_pin_cam.py" download><button>Download</button></a></center>

---

