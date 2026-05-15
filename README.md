# 🤖 ARKit to ROS 2 Bridge V2 (Receiver Node)

[![ROS 2](https://img.shields.io/badge/ROS_2-Foxy%20%7C%20Humble-22314E?logo=ros)](https://docs.ros.org/)
[![Python](https://img.shields.io/badge/Python-3.8+-3776AB?logo=python)](#)

A lightweight ROS 2 Python package that acts as the receiving end for the **ARKit to ROS 2 Bridge V2**. It listens for custom chunked UDP packets sent from an iOS device, decodes the payloads, and dynamically publishes standard ROS 2 messages (`nav_msgs/Odometry`, `sensor_msgs/Image`, `sensor_msgs/CameraInfo`, and `tf2`).

This repository is the **receiver** half of the bridge. For the native iOS application that generates this data, see the [ARKit-to-ROS-Bridge-V2](https://github.com/ion206/ARKit-to-ROS-Bridge-V2) repository.

**V1 (Legacy TCP version):** [iOS-ARKit-to-ROS2](https://github.com/ion206/iOS-ARKit-to-ROS2)  
**Real-World Application:** This low-latency V2 bridge was engineered specifically for indoor autonomous vehicle navigation. Check out the full hardware deployment here: [DAES AutonomousRacer](https://github.com/daengineeringsociety/AutonomousRacer)

---

## 🚀 What's New in V2?
* **Zero-Latency UDP Sockets:** Completely rebuilt from the ground up to ingest high-speed UDP streams. By removing TCP handshake overhead, odometry packets arrive with microsecond precision.
* **Tailored Decoding Scripts:** Features dedicated executables to handle either pure, blazing-fast VIO or dense, multi-topic mapping data.
* **Euler Angle Corrections:** Automatically applies coordinate transformations to map the iOS Right-Hand rule to the ROS 2 standard base links, correcting orientation before it hits the `/tf` tree.

## 📊 Topic Availability Matrix
Depending on which script you execute (and which branch is running on the iOS app), the node will publish the following topics:

| ROS 2 Topic | `vio` Script | `mapping` Script | Data Type | Description |
| :--- | :---: | :---: | :--- | :--- |
| `/synced/odom` | ✅ | ✅ | `nav_msgs/Odometry` | Real-time 6-DOF odometry tracking |
| `/tf` | ✅ | ✅ | `tf2_msgs/TFMessage` | Transform tree (`odom` ➔ `camera_link` ➔ `base_link`) |
| `/synced/rgb/image_raw` | ❌ | ✅ | `sensor_msgs/Image` | Decompressed RGB Camera feed |
| `/synced/depth/image_raw` | ❌ | ✅ | `sensor_msgs/Image` | Decompressed LiDAR depth matrix |
| `/synced/camera_info` | ❌ | ✅ | `sensor_msgs/CameraInfo`| Formatted Camera intrinsics |

---

## ⚙️ Prerequisites & Docker Notes

This package is designed to be as lightweight as possible to run on edge-compute modules (like an NVIDIA Jetson or AWS DeepRacer). 

**Dependencies:**
The only external Python library required is `scipy` (used for quaternion/Euler math):
```bash
pip3 install scipy
```

**🐳 Docker Deployment**
If you are running this ROS 2 package inside a Docker container, you **must** run the container with the host network configuration. If you do not use `--network host`, Docker will isolate the container on a virtual subnet, and the UDP packets sent from your iPhone will crash against the container wall and drop.
```bash
docker run -it --network host <your_image_name> bash
```

---

## 🛠️ Installation

1. Clone this repository into your ROS 2 workspace `src` directory:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/ion206/arkit_ros2_bridge.git
   ```
2. Build the package:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select arkit_ros2_bridge
   source install/setup.bash
   ```

---

## 🎮 Usage Guide

This package contains different execution scripts tailored to the incoming data stream. **You must run the script that corresponds to the branch currently running on your iPhone.** Ensure your iOS app is configured to stream to the IP address of the machine running these scripts on **Port 8765**.

### Scenario A: High-Speed Telemetry (VIO)
If your iPhone is on the `vio` branch, it is broadcasting pure odometry. Launch the VIO receiver node. This script consumes virtually 0% CPU and broadcasts a flawless, zero-latency `/tf` tree at ~180Hz.
```bash
ros2 launch arkit_ros2_bridge vio.launch.py
```

### Scenario B: Dense Spatial Payload (Mapping)
If your iPhone is on the `mapping` branch, it is broadcasting the full RGB-D payload. Launch the mapping receiver node. This script features a custom UDP synchronization buffer to perfectly match incoming depth chunks with RGB frames and odometry microseconds before publishing them to the ROS network.
```bash
ros2 launch arkit_ros2_bridge mapping.launch.py
```

## 🛜 Network Troubleshooting
* **No Data Arriving?** Use `udptest.py` to verify packets are actually reaching the machine:
  `python3 arkit_ros2_bridge/arkit_ros2_bridge/udptest.py`
* **Local Network Privacy:** Ensure the iOS app has "Local Network" permissions enabled in the iPhone's Settings app, or iOS will silently drop the packets.
* **USB Tethering:** For the absolute best performance, tether the iPhone to your ROS 2 machine via a direct USB cable and use the wired tethering IP (usually a `172.20.10.x` or `192.168.x.x` address).
