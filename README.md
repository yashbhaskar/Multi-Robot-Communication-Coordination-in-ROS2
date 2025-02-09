# Multi-Robot-Communication-Coordination-in-ROS2
This project demonstrates multi-robot communication and coordination in ROS 2 using Gazebo. It implements a leader-follower system, where one robot (leader) shares its position data, and the second robot (follower) follows it using ROS 2 topics, services, or actions.
# Multi-Robot Communication & Coordination in ROS 2

This project demonstrates multi-robot communication and coordination in ROS 2 using Gazebo. It implements a **leader-follower system**, where one robot (leader) shares its position data, and the second robot (follower) follows it using ROS 2 topics, services, or actions.

## 📌 Features
- ✅ **Spawn Multiple Robots** – Spawns `ros2_bot` and `tortoisebot` in Gazebo using Xacro and URDF.
- ✅ **Teleoperation** – Control the leader robot using `teleop_twist_keyboard`.
- ✅ **Leader-Follower System** – The follower robot subscribes to the leader's pose and replicates its movement.
- ✅ **ROS 2 Topics & Services** – Implements inter-robot communication using `geometry_msgs/Twist` and `nav_msgs/Odometry`.
- ✅ **Gazebo & Rviz Integration** – Simulates multi-robot coordination in a realistic environment.

## 🚀 Installation

1. Clone the repository and navigate to the workspace:
   ```bash
   cd ~/ros_ws/src
   git clone <repo_url>
   cd ~/ros_ws
   ```
2. Build the workspace:
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```

## 🎮 Usage

### 1️⃣ Launch Gazebo with both robots:
```bash
ros2 launch multirobot_sim spawn_ros2_bot.launch.py
ros2 launch multirobot_sim spawn_tortoisebot.launch.py
```

### 2️⃣ Control the Leader Robot (ros2_bot):
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/ros2_bot/cmd_vel
```

### 3️⃣ Start the Follower Node:
```bash
ros2 run multirobot_sim follower.py
```

The follower robot (`tortoisebot`) will track and follow the leader (`ros2_bot`) based on real-time pose updates.

## 📹 Demo
Below is a short demonstration of the **leader-follower system** in action:

https://github.com/user-attachments/assets/6552688e-b4d0-4ea0-9ac1-2bccced4314a

*(Upload a `.gif` file inside the `docs/` folder in your repo to display it here.)*

## 📜 ROS 2 Topics & Services Used
- **Leader Robot publishes:**
  - `/ros2_bot/cmd_vel` – Leader's velocity commands
  - `/ros2_bot/odom` – Leader's odometry data
- **Follower Robot subscribes:**
  - `/ros2_bot/odom` – Reads leader's position
  - `/tortoisebot/cmd_vel` – Moves the follower accordingly

## 📂 Project Structure
```
multirobot_sim/
│── launch/
│   ├── gazebo.launch.py
│   ├── master.launch.py
│   ├── slave.launch.py
│── master_models/         # URDF of master
│   ├── meshes
│   ├── urdf
│── slave_models/         # URDF of slave
│   ├── meshes
│   ├── urdf
│── scripts/
│   ├── follower.py       # Follower robot logic
│   ├── leader.py
│── CMakeLists.txt
│── package.xml
│── README.md
```

## 🤝 Contributing
Feel free to fork this repository, submit pull requests, or report issues.

## 📜 License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---
🚀 **Happy Coding & Robotics!** 🤖

