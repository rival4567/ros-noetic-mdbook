# Path planning and navigation

**Note**: While you can finish this task by just hard-coding the setpoints, while it may work for a static scenario with not a long path, it will be unsustainable to do for larger paths and dynamic scenarios and hence you should NOT do this. If hard-coding is found, it may lead to disqualification.

Hence it is recommended that you implement proper path planning algorithms.

  - On the controls side, looking into waypoint navigation (with the help of position controller and attitude controller) or trajectory tracking.

  - On the planning algorithm side look into simple algorithms like the Bug algorithms. You can also utilize algorithms like A* and Djikstra's, but pay special attention to your data structures as you're operating on a larger scale.

[This PDF](https://spacecraft.ssl.umd.edu/academics/788XF14/788XF14L14/788XF14L14.pathbugsmapsx.pdf) gives a nice overview of the points discussed above.

You can use **any** sorts of algorithms or packages that involve no prior knowledge of the arena/scene. i.e. you cannot use algorithms which take the sector mesh or its derivatives as inputs and plan around the obstacles using that approach.

If you choose to do all of this, this task will be much more programming heavy than earlier ones, it will be beneficial in the future tasks and impart lots of learning.

**Pro-Tip**: Try to bring the GPS coordinates into x, y pose (in metres) you can do this by using simple algebra. This will enable you to take into account things like distance etc. more easily while planning trajectories, to what degree you include position is metres into your script is up to you, i.e. just for planning, or maybe for both planning and the control loop (same or similar gains will likely work).

```python
def lat_to_x(self, input_latitude):
    return 110692.0702932625 * (input_latitude - 19)

def long_to_y(self, input_longitude):
    return -105292.0089353767 * (input_longitude - 72)
    # if you use this for control, you may have to change the relevant pitch   direction because of the sign

```

<hr>