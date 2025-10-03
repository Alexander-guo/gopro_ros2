# gopro_ros2

This repository contains code for parsing GoPro telemetry metadata to obtain GoPro images with synchronized IMU measurements. The GoPro visual-inertial data can then be saved in [ros2 bag](http://wiki.ros.org/rosbag) or [EuRoC](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) format. Thus, effectively paving the way for visual-inertial odometry/SLAM for GoPro cameras.

This repository use [gpmf-parser](https://github.com/gopro/gpmf-parser)  from [GoPro](https://gopro.com) to extract metadata and timing information from GoPro cameras.

## Related Paper
If you find the code useful in your research, please cite our paper:
```bash
@inproceedings{joshi_gopro_icra_2022,
  author      = {Bharat Joshi and Marios Xanthidis and Sharmin Rahman and Ioannis Rekleitis},
  title       = {High Definition, Inexpensive, Underwater Mapping},
  booktitle   = {IEEE International Conference on Robotics and Automation (ICRA)},
  year        = {2022},
  pages       = {1113-1121},
  doi         = {10.1109/ICRA46639.2022.9811695},
  abbr        = {ICRA},
  bibtex_show = {true},
  code        = {https://github.com/AutonomousFieldRoboticsLab/gopro_ros},
}
```

# Installation

Tested on Ubuntu 24.04 (ROS2-Jazzy).

## Prerequisites

- ros-jazzy-desktop-full
- [OpenCV](https://github.com/opencv/opencv) >= 4.6
- [FFmpeg](http://ffmpeg.org/) >= 6.1.1
- [Eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page)

## Install Dependencies

- First install ROS2 using [this guide](https://docs.ros.org/en/jazzy/Installation.html).

```bash
# First update package list
sudo apt-get update

# ROS 2 dependencies
sudo apt-get install -y \
    ros-$ROS_DISTRO-rosbag2-cpp \
    ros-$ROS_DISTRO-rosbag2-storage-default-plugins \

# System libraries
sudo apt-get install -y \
    libeigen3-dev \
    libopencv-dev \
    ffmpeg
```

## Install gopro_ros

Before proceeding, ensure all dependencies are installed. To install gopro_ros2 (currently save to bag file only):

```bash
mkdir -p ~/gopro_ros2_ws/src
cd gopro_ros2_ws/src
git clone https://github.com/Alexander-guo/gopro_ros2.git
cd ~/gopro_ros2_ws
colcon build --packages-select gopro_ros2 --symlink-install --cmake-args -DBUILD_GOPRO_TO_ASL OFF
source ~/gopro_ros2_ws/install/setup.bash # Or add this to ~/.bashrc to make it permanent
```

# Usage

GoPro splits video into smaller chunks. By splitting up the video it reduces the chance of you losing all your footage if the file gets corrupted somehow. It’s called chaptering, and the idea is that if one chapter gets corrupted the others should still be okay because they’re separate files.

## Save to ROS2 bag

Both storage backends, MCAP(.mcap) and SQLite3(.db3), are supported and automatically identified by the suffix of `<bag_file>` for `rosbag` argument. To save GoPro video with IMU measurements to ros2 bag:

```bash
ros2 launch gopro_ros2 gopro_to_rosbag.xml gopro_video:=<gopro_video_file> rosbag:=<bag_file>
```

If you have multiple files from a single session, put all videos in same folder you can use the following command to concatenate into a single rosbag:

```bash
ros2 launch gopro_ros2 gopro_to_rosbag.xml gopro_folder:=<folder_with_gopro_video_files> multiple_files:=true rosbag:=<bag_file>
```

## Save to EuRoC format

To save GoPro video with IMU measurements in Euroc format:

```bash
ros2 launch gopro_ros2 gopro_to_asl.xml gopro_video:=<gopro_video_file> asl_dir:=<asl_format_dir>
```

If you have multiple files from a single session, put all videos in same folder you can use the following command extract all videos in a single folder:

```bash
ros2 launch gopro_ros2 gopro_to_asl.xml gopro_folder:=<folder_with_gopro_video_files> multiple_files:=true asl_dir:=<asl_format_dir>
```

# TODO:
Enable save into EuRoC format.