# Problem Statement
So in this tutorial we are going to learn about micromouse setup in gazebo.

We will be using a URDF robot model of the micrmouse.

We have to make the micromouse navigate from its starting positon to the center of the maze .The navigation of the maze should be achieved by the use of the laser scan and odom data.

There are many path planning algorithms like **`BFS`**, **`DFS`**, **`Djikstra`**, **`A-star`**. You will be able to find plenty of online resources for the same.

## **Hints**

- You can try synchronizing the `odom` and `laserscan` with the help of 
  `ApproximateTimeSynchronizer` from the package `message_filters` that comes inbuilt with is ROS.
- You can use `euler_from_quaternion` which converts quaternion to euler     angles `roll`, `pitch` and `yaw`
- You can subdivied the laser scan data so that much more resposnsive descision can be taken.