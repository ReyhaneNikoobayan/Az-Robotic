# Az-Robotic

## Creating an environment in Gazebo

### Step 1: open a terminal and run the following command to start Gazebo

```bash
gazebo

```

### 🏗️ Step 2: Open the Building Editor

From the top menu, go to **Edit → Building Editor**  .
This opens the design interface where you can create your own **custom environment** in Gazebo.

<img width="2727" height="1515" alt="Screenshot from 2025-11-26 16-39-11" src="https://github.com/user-attachments/assets/055dedfa-0a3b-4161-b865-3f70eb551e9d" />


### Step 3: Design Your Environment

You can create your environment by selecting and placing walls in the **2D view**.  
You can also add features such as **doors, walls, and stairs**.  

In the **3D view**, you can apply **textures and colors** to your environment to make it more realistic.

### Step 4: Import Images

You can import images from the **Import** menu.  
This allows you to easily build your desired **environment** from a map.

<img width="2734" height="1121" alt="Screenshot from 2025-11-08 18-00-43" src="https://github.com/user-attachments/assets/9d94af8f-a9c9-497a-9a8a-e79692b631d1" />

### Step 5: Save and View Your Environment

After creating your environment, go to the **File** menu and choose **Save**.  
Specify a **Model Name** and **Location** where your model will be saved.  

Finally, click **Editor Building Exit** to close the editor and view your environment in Gazebo.

<img width="2761" height="1527" alt="Screenshot from 2025-11-26 16-19-14" src="https://github.com/user-attachments/assets/f607b842-b09e-4f56-8d1d-a2bde022ba93" />


### Step 6: Add Features

You can add more details to your environment by selecting **Model Editor** from the **Edit** menu.  
This allows you to insert and customize additional models, objects, and features in your simulation world.

<img width="2714" height="1569" alt="Screenshot from 2025-11-09 16-59-21" src="https://github.com/user-attachments/assets/6bade62e-37b9-47d9-ab6b-26edbf78fcd8" />


### Step 7: Add Simple Features and Models

You can add simple features from **Simple Shapes**, or insert objects and models from the **Model Database** — such as a **café table**, **house model**, or other available assets.  
These additions help you create a more realistic and detailed simulation environment.

> 💡 **Hint:** You can change the **density**, **size**, **color** and other features of simple shapes by clicking on **Open Link Inspector**.

<img width="2761" height="1527" alt="Screenshot from 2025-11-26 16-27-27" src="https://github.com/user-attachments/assets/dd451ebb-dd8c-4ccc-a2ab-47524b90b51d" />


### Step 8: Save and View Your Model

Now you can save your model and click **Exit Model Editor**.  
After that, save your world using **Save World As**, and make sure to use the **.world** extension.

Your entire model will then be visible in the Gazebo simulation environment.

<img width="2727" height="1515" alt="Screenshot from 2025-11-26 16-29-46" src="https://github.com/user-attachments/assets/e5918a01-f83c-4671-b39b-14e753bf931e" />


## How to Run the Simulation on Your Laptop

### Step 1: Install Dependent ROS 2 Packages

Run the following commands to install Cartographer and Navigation2:

```bash
sudo apt install ros-humble-cartographer
sudo apt install ros-humble-cartographer-ros

sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
```

### Step 2 : Install TurtleBot3 Packages

Set up your **TurtleBot3 workspace** and clone the required packages by running the following commands:

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src/

git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git

sudo apt install python3-colcon-common-extensions

cd ~/turtlebot3_ws
colcon build --symlink-install

echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc

```

### Step 3 : Configure Environment Variables

Add the following lines to your `.bashrc` file to configure your environment for **TurtleBot3** and **Gazebo**:

```bash
echo 'export ROS_DOMAIN_ID=30 # TURTLEBOT3' >> ~/.bashrc
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

### Step 4 : Install Simulation Package

Install the **TurtleBot3 Simulation** package by running the following commands:

```bash
cd ~/turtlebot3_ws/src/
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/turtlebot3_ws && colcon build --symlink-install
```

### Step 5: Launch the TurtleBot3 Simulation

After completing the setup, you can launch different TurtleBot3 worlds in Gazebo.  

**1. Launch an empty world:**

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py

```

**2. Launch a TurtleBot3 World:**

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**2. Launch a TurtleBot3 House:**

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

### Step 6: Control Your Robot with the Keyboard

You can move your TurtleBot3 robot in any of the Gazebo environments using your keyboard.  
Run the following command in a terminal:

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

## 🏞️ How to Add Your Environment to the TurtleBot3 Package

### Step 1: Tranfer World file

Copy the world you built in Gazebo into the **worlds** folder in Turtlebot3 package.

### Step 2: Change the File to XML Format

Make sure your world file begins with the correct XML declaration.  
Open your `.world` file and add the following line at the very top:

```xml
<?xml version="1.0"?>
```
<img width="2727" height="1515" alt="Screenshot from 2025-11-26 16-53-21" src="https://github.com/user-attachments/assets/bfda2363-33ba-41ac-b425-c4cc54478935" />


### Step 3: Create a Launch File

Navigate to the **launch** folder inside the TurtleBot3 Gazebo package and make a copy of the existing launch file (‫‪turtlebot3_world.launch.py‬‬). Then, change the world name to your own custom world file.  
If necessary, adjust the **x_pose** and **y_pose** values to modify the robot’s starting position, and add a **yaw** parameter to define the robot’s orientation.

<img width="2727" height="1515" alt="Screenshot from 2025-11-26 16-51-44" src="https://github.com/user-attachments/assets/80d538c7-5e5a-432d-b46b-17b1890ca291" />


<img width="2671" height="1502" alt="Screenshot from 2025-11-12 17-23-36" src="https://github.com/user-attachments/assets/3369834c-b75f-4872-b363-356d11668724" />

### Step 4: Edit the Spawn File

Open the `spawn_turtlebot3.launch.py` file located in the **launch** folder of the TurtleBot3 Gazebo package.

Add the following lines in the appropriate place within the file to include the **yaw** parameter (for controlling the robot’s orientation):

```python

x_pose = LaunchConfiguration('x_pose', default='0.0')
y_pose = LaunchConfiguration('y_pose', default='0.0')
yaw = LaunchConfiguration('yaw', default='3.14')

declare_yaw_cmd = DeclareLaunchArgument(
    'yaw', default_value='3.14',
    description='Yaw orientation (in radians) of the robot'
)

start_gazebo_ros_spawner_cmd = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=[
        '-entity', TURTLEBOT3_MODEL,
        '-file', urdf_path,
        '-x', x_pose,
        '-y', y_pose,
        '-z', '0.01',
        '-Y', yaw
    ],
    output='screen',
)

ld.add_action(declare_yaw_cmd)

```

<img width="2702" height="1516" alt="Screenshot from 2025-11-12 17-29-19" src="https://github.com/user-attachments/assets/e93f46f2-6d45-413d-a6e1-5dd97e72f126" />


### Step 5: Compile Your Package

Navigate to your workspace, open a terminal, and run the following commands:

```bash
colcon build
source install/setup.bash
```

### Step 6: Open TurtleBot3 in Your Custom Environment

Run the following commands in a separate terminal to launch your TurtleBot3 in your custom world:

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_myworld.launch.py
```
<img width="2727" height="1515" alt="Screenshot from 2025-11-26 16-55-43" src="https://github.com/user-attachments/assets/a8512918-0c7f-40ef-aa9b-5ae25c2eb905" />



## Localization and Mapping of the Environment

### Step 1: Launch TurtleBot3 in Your Custom Environment

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_myworld.launch.py
```
### Step 2: Run the Cartographer Package

In a **separate terminal**, run the following command:

```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=true
```
### Step 3: Create the Map

Move your robot throughout the entire environment.  
Using LiDAR data, the robot will measure distances and gradually build the map.

To move the robot with your keyboard, run:

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```
<img width="2727" height="1515" alt="Screenshot from 2025-11-26 17-10-32" src="https://github.com/user-attachments/assets/e3f630eb-fa96-4a23-adf7-4c0bc61f01dc" />


### Step 4: Save the Map

Open a terminal in the folder where you want to save your map, then run:

```bash
ros2 run nav2_map_server map_saver_cli -f my_map
```

## Automatic Robot Navigation in the Environment

### Step 1: Edit `burger.yaml`

Open the `burger.yaml` file to change the robot motion model.  
Run the following command:

```bash
gedit /opt/ros/humble/share/turtlebot3_navigation2/param/burger.yaml
```

Then, replace:

```yaml
robot_model_type: "differential"
```
with
```yaml
robot_model_type: "nav2_amcl::DifferentialMotionModel"
```

### Step 2: Run TurtleBot3

Run TurtleBot3 in your custom environment by executing:

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_myworld.launch.py
```

### Step 3: Run Nav2

Open a terminal **in the folder where your map is saved**, then run:

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
    use_sim_time:=true \
    map:=my_map.yaml

```
<img width="2727" height="1515" alt="Screenshot from 2025-11-26 17-34-55" src="https://github.com/user-attachments/assets/4944306a-cea7-4cce-afb8-fe1898bb1b05" />

### Step 4: Initialize TurtleBot3 Pose

Set the initial position of the TurtleBot3 inside the `Initial_Pose.py` script.  
After updating the coordinates, run the script in the same folder using:

```bash
python3 Initial_Pose.py
```

<img width="2727" height="1515" alt="Screenshot from 2025-11-26 17-39-26" src="https://github.com/user-attachments/assets/fd9a079d-cdb5-4535-b11b-903db3339a24" />

You can also determine the robot’s initial position using **2D Pose Estimate** in **RViz2**.


<img width="2727" height="1515" alt="Screenshot from 2025-11-26 17-41-26" src="https://github.com/user-attachments/assets/f84bc4ec-df79-4ead-845b-00e5a5ec18f5" />


### Step 5: Set the Goal Position

You can define the robot’s destination in **RViz2** using the **Nav2 Goal** tool.  
Alternatively, you can run the following script to publish a goal position:

```bash
python3 Goal_Pose.py
```
‬‬‬‬‬‬‬‬‬‬<img width="2727" height="1515" alt="Screenshot from 2025-11-26 18-29-09" src="https://github.com/user-attachments/assets/4fc3adad-891b-4556-be2c-2ab4d6f3b08e" />

### Step 6: Final Result

Now the robot will navigate toward the specified goal position automatically.

<img width="2727" height="1515" alt="Screenshot from 2025-11-26 18-31-13" src="https://github.com/user-attachments/assets/8fb26993-2d7a-4df8-8837-0f00acebd1f1" />

## How to Launch a Real TurtleBot3

### Step 1: Connect Your TurtleBot3 System to Your Laptop

First, make sure that **both your laptop and the TurtleBot3** are connected to the **same network**.

Then, open a terminal on your laptop and connect to the TurtleBot3 via SSH:

```bash
ssh TurtleBot3@10.18.154.195
```

### Step 2: Launch TurtleBot3 on the Robot

After connecting via SSH, run the following commands on the TurtleBot3 to set up the environment and start the robot:

```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash

export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export LDS_MODEL=LDS-02

ros2 launch turtlebot3_bringup robot.launch.py

```

### Step 3: Configure Your Laptop

On your laptop, open a terminal and run the following commands to set up the ROS 2 environment and configure TurtleBot3 parameters:

```bash
source /opt/ros/humble/setup.bash

export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export LDS_MODEL=LDS-02
```

### Step 4: Run the Cartographer Package

On your laptop, launch the Cartographer package to perform mapping and localization:

```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=true
```

### Step 5: Move Your Robot

You can manually control your TurtleBot3 in the real environment using the keyboard:

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

### Step 6: Run the Robot with Your Code

You can also run a code that subscribes to the `/scan` topic to detect obstacles and publishes commands to `cmd_vel` to move the robot autonomously:

```bash
ros2 run robot_controller Lidar_Data_alone_robot
```

