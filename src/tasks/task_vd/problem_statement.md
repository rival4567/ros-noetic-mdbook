# Problem Statement

- The eDrone starts from the start position i.e. latitude: 19.0009248718 longitude: 71.9998318945 altitude: 22.16, this position is already entered in the relevant world file and the eDrone will spawn at this location itself and it will be in view.

- It is mandatory that the eDrone start from here during your submission recording.

- It has to go to latitude: 18.999981931836018 longitude: 71.99983191056211 altitude: 32.15999670352447. There is a building in between the starting point and ending point.

- Reach the destination with obstacle avoidance and path planning. Use the onboard rangefinder sensors on the eDrone to detect obstacles and navigate around them. While there is no limitation on script structure, it is recommended that you merge the navigation logic in your position control script as we think that will be the easiest way forward.

## Hints

- You will use the following boiler scripts:
  
  - attitude_controller.py: <center><a href="./attitude_controller.py" download><button>Download</button></a></center>
  
  - position_controller.py: <center><a href="./position_controller.py" download><button>Download</button></a></center>

- attitude controller maked the drone change its roll, pitch, yaw and throttle speed, position controller achieves a postion i.e. height, latitude and longitude with the help of attitude controller.

- You dont need to do any changes in attitude controller, follow the position controller boiler script and after completing it run you scripts to check the result.

- first run position controller:

```bash
rosrun vitarana_drone position_controller.py
```

- then attitude controller:

```bash
rosrun vitarana_drone attitude_controller.py

```

<hr>