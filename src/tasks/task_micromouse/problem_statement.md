# Problem Statement

- **Objective**: The objective of this task is to code an autonomous self-contained bot on ROS to be simulated on Gazebo, Micromouse, which can find its way and get to the centre of a maze in the shortest possible time.


- **Collision Avoidance**

  - Collision avoidance can be done by using laser scan data.

  - The run will be considered invalid if there is any collision with the walls of the maze.

- The micromouse should be spawned at **x=-1.35** and **y=1.35**.

- **Simulation Time**

  - In this task simulation time will be considered for grading. So, the teams must make sure to keep the simulation time as low as possible by quickly navigating the maze.

- **Odom**

  - Odometry is to be used by the micromouse to estimate its position and orientation relative to a starting location.

  - Use `/micromouse/odom` rostopic to publish odom data.

- **Laser Scan**

  - The laser scan can be used to detect walls or obstacles around the micromouse robot.

  - Use `/micromouse/laser/scan` rostopic to publish laser scan data.

> **Note**: Only laser scan and odom data should be used to navigate the maze.

## Procedure

1. **Optional**: Create your own micromouse robot model in urdf.

1. Create a launch file `micromouse_solution.launch` which should launch the gazebo environment of maze, spawn the micromouse robot and all the nodes necessary to solve the problem statement. We'll use this launch file to check your solution.

1. For reference you can check the `micromouse.launch` file.

> **Note**:You can make any number of python nodes to solve the problem statement.

1. The team would also have to create a `bag_files` folder in `micromouse` package for storing the bag file.

1. Instructions to record bag file is given in Submission Instruction section.

<br/>

## Hint 
There are many path planning algorithms like **`BFS`**, **`DFS`**, **`Djikstra`**, **`A-star`**. You will be able to find plenty of online resources for the same.

---
