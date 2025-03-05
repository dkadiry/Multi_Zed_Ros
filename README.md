# ZED Dual Camera ROS Package

## 📌 Project Overview
This ROS package, **multi_zed_ros**, is designed to interface with **two Stereolabs ZED cameras** and a **Clearpath Husky Rover** to collect:
- **Pose data** (absolute and relative motion tracking)
- **Image data** (left-camera RGB images)
- **Depth maps** (captured in `.npy` format)
- **Wireless controller inputs** (Logitech F710 for manual toggling)
- **Velocity commands** (captured from `/joy_teleop/cmd_vel`)

This package allows for real-time data acquisition and storage, making it suitable for **robotic perception and navigation research**. The cameras are managed using multi-threading, and data is saved in structured directories. 

It is designed to be run on an NVIDIA Jetson TX2 with a Ubuntu 18.04 Image.

## 🛠️ Key Features
- **Interfaces with multiple ZED cameras** and allows simultaneous recording.
- **Captures images, depth maps, and pose data** at adjustable frequencies.
- **Uses Wireless Logitech F710 joystick teleoperation** to control data collection (start, stop, capture, etc.).
- **Stores pose data in CSV format**, including timestamps and orientation.
- **Multi-threaded implementation** for handling multiple cameras efficiently.
- **Automatic directory creation** for organized data storage.

---

## 🚀 Getting Started
### **1. Clone the Repository**
```bash
git clone https://github.com/your-username/ros-zed-interface.git
cd ros-zed-interface
```

### **2. Install Dependencies**
Ensure you have **Clearpath Husky and the NVIDIA Jetson TX2** configured and set up:
[Installing a Jetson TX2 on Husky Official Guide](https://www.clearpathrobotics.com/assets/guides/melodic/husky/jetson_tx2.html)
[Interfacing with Husky Official Guide](https://www.clearpathrobotics.com/assets/guides/melodic/husky/InterfacingWithHusky.html)

```bash
sudo apt-get install ros-melodic-husky-desktop
```
Additionally, install the **Stereolabs ZED SDK and Python API** by following the [official guide](https://github.com/stereolabs/zed-python-api).

### **3. Add Package to a Catkin Workspace**
Navigate to your Catkin workspace and clone the package inside the `src/` directory:
```bash
cd ~/catkin_ws/src
git clone https://github.com/your-username/ros-zed-interface.git
cd ~/catkin_ws
catkin_make  # Or catkin build if using catkin_tools
source devel/setup.bash
```

### **4. Run the ROS Node**
Once everything is set up, you can launch the node:
```bash
rosrun zed_ros multi_zed_control.py
```

## 📊 Example Data Outputs
Data is saved inside `/media/jetsond/9F4F-28D0/data_camera_SERIAL/` with structured directories:
```plaintext
/data_camera_123456/
├── twist_data.csv      # Velocity commands (linear & angular)
├── pose_data.csv       # Absolute pose measurements
├── 1709876543210/      # Timestamped directory for image & depth map
│   ├── zed_image_left_1709876543210.jpg
│   ├── depth_map_1709876543210.npy
│   ├── pose_data_1709876543210.txt
```

---

## 🎮 Controller Mappings
This package supports manual triggering using the **Logitech F710 wireless controller**:
| Button  | Function  |
|---------|------------|
| **X (2)** | Capture single depth map  |
| **B (1)** | Toggle continuous depth map recording |
| **Start (7)** | Toggle camera connection  |
| **Back (6)** | Shutdown program  |

---

