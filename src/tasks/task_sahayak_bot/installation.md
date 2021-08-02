# Installation

## Setup


- First, we need a workspace. You can use an existing one or create a new one.

  ```bash
  cd ~/workspace/
  catkin_make
  ```
  ```bash
  source devel/setup.bash:
  ```
- To source your new `setup.*sh` file. This command can also be added to you `~/.bashrc` file

## Cloning the Sahayak Bot repository

- Navigate inside your `catkin_ws/src` directory.

  ```bash
  cd ~/workspace/src
  ```

- Clone the repository...

  ```bash
  git clone https://github.com/vishalgpt579/sahayak_bot.git

  ```
> For user who don't have `git` installed. Simply enter `sudo apt install git`

- This package will take time to download, due to its large size file such as the stl, meshes, images and executables files.

- Finally build your catkin workspace using `catkin_make` command.

- U will also need another package `ebot_nav` to navigate the bot in gazebo. Download the package from the button below (which is a zip file) and extract its content in your `workspace/src` directory.

<center><a href="tasks/task_sahayak_bot/ebot_nav.zip" download><button>Download</button></a></center>

- After extracting dont forget to build your packages.

  ```bash
  cd ~/workspace/
  caktkin_make
  ```

## Addidtion Installation

- Additional installations are also required to run the sahayak bot, follow the below instructions to install them.

  ```bash
  sudo apt-get install ros-noetic-object-recognition-msgs
  ```

  ```bash
  sudo apt-get install ros-noetic-navigation
  ```

now you should be able to run the package

## Running Sahayak Bot

to run Sahayak Bot u will need to do run the following commands:

- in one terminal:
  ```bash
  roslaunch ebot_description task2.launch
  ```
- in second terminal:
  ```bash
  roslaunch ebot_nav autonomous_nav.launch
  ```
you should be able to see the sahayak bot in gazebo window moving.

<hr>