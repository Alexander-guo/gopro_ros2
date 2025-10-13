# Dockerized gopro_ros2

This is the dockerized gopro_ros2 workspace, which makes the ROS2 package compatible with any host system.

## Usage

Before proceeding, ensure docker and docker compose are installed.

```bash
git clone -b docker https://github.com/Alexander-guo/gopro_ros2.git
cd gopro_ros2
docker compose build
docker compose up -d  # detached mode
```

To enter the docker container:

```bash
docker exec -it gopro_ros2 bash
```

Put any GoPro videos you want to convert under `./dataset`, then you are able to refer the data via `/media/data` inside the container.

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

## TODO

Enable save into EuRoC format.
