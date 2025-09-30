//
// Created by bjoshi on 10/29/20.
//


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/header.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rmw/rmw.h>

#include <experimental/filesystem>
#include <iostream>

#include "ImuExtractor.h"
#include "VideoExtractor.h"
#include "color_codes.h"
#include "Utils.h"

using namespace std;
namespace fs = std::experimental::filesystem;


int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("gopro_to_rosbag");

  node->declare_parameter<std::string>("gopro_video", "");
  node->declare_parameter<std::string>("gopro_folder", "");
  node->declare_parameter<std::string>("rosbag", "");
  node->declare_parameter<double>("scale", 1.0);
  node->declare_parameter<bool>("compressed_image_format", false);
  node->declare_parameter<bool>("grayscale", false);
  node->declare_parameter<bool>("display_images", false);
  node->declare_parameter<bool>("multiple_files", false);

  std::string gopro_video;
  std::string gopro_folder;
  std::string rosbag;
  double scaling;
  bool compress_images;
  bool grayscale;
  bool display_images;
  bool multiple_files;

  node->get_parameter("gopro_video", gopro_video);
  node->get_parameter("gopro_folder", gopro_folder);
  node->get_parameter("rosbag", rosbag);
  node->get_parameter("scale", scaling);
  node->get_parameter("compressed_image_format", compress_images);
  node->get_parameter("grayscale", grayscale);
  node->get_parameter("display_images", display_images);
  node->get_parameter("multiple_files", multiple_files);

  bool is_gopro_video = !gopro_video.empty();
  bool is_gopro_folder = !gopro_folder.empty();

  if (!is_gopro_video && !is_gopro_folder) {
    RCLCPP_FATAL(node->get_logger(), "Please specify the gopro video or folder");
    rclcpp::shutdown();
    return 1;
  }

  if (rosbag.empty()) {
    RCLCPP_FATAL(node->get_logger(), "No rosbag file specified");
    rclcpp::shutdown();
    return 1;
  }

  // infer bag config from rosbag file name
  BagConfig cfg = infer_bag_config(rosbag);

  rosbag2_cpp::Writer bagnfer;
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = cfg.uri;
  storage_options.storage_id = cfg.storage_id;  // e.g., "sqlite3" or "mcap"

  rosbag2_cpp::ConverterOptions converter_options{
      rmw_get_serialization_format(),
      rmw_get_serialization_format()
  };

  bag.open(storage_options, converter_options);

  vector<fs::path> video_files;

  if (is_gopro_folder && multiple_files) {
    std::copy(fs::directory_iterator(gopro_folder),
              fs::directory_iterator(),
              std::back_inserter(video_files));
    std::sort(video_files.begin(), video_files.end());

  } else {
    video_files.push_back(fs::path(gopro_video));
  }

  auto end = std::remove_if(video_files.begin(), video_files.end(), [](const fs::path& p) {
    return p.extension() != ".MP4" || fs::is_directory(p);
  });
  video_files.erase(end, video_files.end());


  vector<uint64_t> start_stamps;
  vector<uint32_t> samples;
  std::deque<AcclMeasurement> accl_queue;
  std::deque<GyroMeasurement> gyro_queue;
  std::deque<MagMeasurement> magnetometer_queue;

  vector<uint64_t> image_stamps;

  bool has_magnetic_field_readings = false;

  // Read from each video chunks and write video to rosbag
  for (uint32_t i = 0; i < video_files.size(); i++) {
    image_stamps.clear();

    RCLCPP_WARN(node->get_logger(), "Opening Video File: %s", video_files[i].filename().string().c_str());

    fs::path file = video_files[i];
    GoProImuExtractor imu_extractor(file.string());
    GoProVideoExtractor video_extractor(file.string(), scaling, true);

    if (i == 0 && imu_extractor.getNumofSamples(STR2FOURCC("MAGN"))) {
      has_magnetic_field_readings = true;
    }

    imu_extractor.getPayloadStamps(STR2FOURCC("ACCL"), start_stamps, samples);
    RCLCPP_INFO(node->get_logger(), 
                "[ACCL] Payloads: %zu Start stamp: %lu End stamp: %lu Total Samples: %u", 
                start_stamps.size(), start_stamps[0], start_stamps[samples.size() - 1], samples.at(samples.size() - 1));
    imu_extractor.getPayloadStamps(STR2FOURCC("GYRO"), start_stamps, samples);
    RCLCPP_INFO(node->get_logger(), 
                "[GYRO] Payloads: %zu Start stamp: %lu End stamp: %lu Total Samples: %u", 
                start_stamps.size(), start_stamps[0], start_stamps[samples.size() - 1], samples.at(samples.size() - 1));
    imu_extractor.getPayloadStamps(STR2FOURCC("CORI"), start_stamps, samples);
    RCLCPP_INFO(node->get_logger(), 
                "[Image] Payloads: %zu Start stamp: %lu End stamp: %lu Total Samples: %u", 
                start_stamps.size(), start_stamps[0], start_stamps[samples.size() - 1], samples.at(samples.size() - 1));
    if (has_magnetic_field_readings) {
      imu_extractor.getPayloadStamps(STR2FOURCC("MAGN"), start_stamps, samples);
      RCLCPP_INFO(node->get_logger(), 
                  "[MAGN] Payloads: %zu Start stamp: %lu End stamp: %lu Total Samples: %u", 
                  start_stamps.size(), start_stamps[0], start_stamps[samples.size() - 1], samples.at(samples.size() - 1));
    }

    uint64_t accl_end_stamp = 0, gyro_end_stamp = 0;
    uint64_t video_end_stamp = 0;
    uint64_t magnetometer_end_stamp = 0;

    if (i < video_files.size() - 1) {
      GoProImuExtractor imu_extractor_next(video_files[i + 1].string());
      accl_end_stamp = imu_extractor_next.getPayloadStartStamp(STR2FOURCC("ACCL"), 0);
      gyro_end_stamp = imu_extractor_next.getPayloadStartStamp(STR2FOURCC("GYRO"), 0);
      video_end_stamp = imu_extractor_next.getPayloadStartStamp(STR2FOURCC("CORI"), 0);
      if (has_magnetic_field_readings) {
        magnetometer_end_stamp = imu_extractor_next.getPayloadStartStamp(STR2FOURCC("MAGN"), 0);
      }
    }

    imu_extractor.readImuData(accl_queue, gyro_queue, accl_end_stamp, gyro_end_stamp);
    imu_extractor.readMagnetometerData(magnetometer_queue, magnetometer_end_stamp);

    uint32_t gpmf_frame_count = imu_extractor.getImageCount();
    uint32_t ffmpeg_frame_count = video_extractor.getFrameCount();
    if (gpmf_frame_count != ffmpeg_frame_count) {
      RCLCPP_FATAL(rclcpp::get_logger("gopro_to_rosbag"),"Video and metadata frame count do not match");
      rclcpp::shutdown();
    }

    uint64_t gpmf_video_time = imu_extractor.getVideoCreationTime();
    uint64_t ffmpeg_video_time = video_extractor.getVideoCreationTime();

    if (ffmpeg_video_time != gpmf_video_time) {
      RCLCPP_FATAL(rclcpp::get_logger("gopro_to_rosbag"), "Video creation time does not match");
      rclcpp::shutdown();
    }

    imu_extractor.getImageStamps(image_stamps, video_end_stamp);
    if (i != video_files.size() - 1 && image_stamps.size() != ffmpeg_frame_count) {
      RCLCPP_FATAL(rclcpp::get_logger("gopro_to_rosbag"), "ffmpeg and gpmf frame count does not match. %zu vs %u", 
        image_stamps.size(), ffmpeg_frame_count);
      rclcpp::shutdown();
    }

    video_extractor.writeVideo(
        bag, "/gopro/image_raw", image_stamps, grayscale, compress_images, display_images);
  }

  // write imu data
  RCLCPP_INFO(node->get_logger(), "[ACCL] Payloads: %zu", accl_queue.size());
  RCLCPP_INFO(node->get_logger(), "[GYRO] Payloads: %zu", gyro_queue.size());

  assert(accl_queue.size() == gyro_queue.size());

  double previous = accl_queue.front().timestamp_ * 1e-9;
  rclcpp::Serialization<sensor_msgs::msg::Imu> imu_serializer;

  while (!accl_queue.empty() && !gyro_queue.empty()) {
    AcclMeasurement accl = accl_queue.front();
    GyroMeasurement gyro = gyro_queue.front();
    int64_t diff = accl.timestamp_ - gyro.timestamp_;
    Timestamp stamp;

    if (abs(diff) > 100000) {
      // I will need to handle this case more carefully
      RCLCPP_WARN(node->get_logger(), "%ld ns difference between gyro and accl", diff);
      stamp = (Timestamp)(((double)accl.timestamp_ + (double)gyro.timestamp_) / 2.0);
    } else {
      stamp = accl.timestamp_;
    }

    double current = stamp * 1e-9;
    // assert(abs(current - previous) < 0.001);
    previous = current;

    builtin_interfaces::msg::Time ros_time;
    ros_time.sec = stamp / 1000000000;
    ros_time.nanosec = stamp % 1000000000;
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = ros_time;
    imu_msg.header.frame_id = "body";
    imu_msg.linear_acceleration.x = accl.data_.x();
    imu_msg.linear_acceleration.y = accl.data_.y();
    imu_msg.linear_acceleration.z = accl.data_.z();
    imu_msg.angular_velocity.x = gyro.data_.x();
    imu_msg.angular_velocity.y = gyro.data_.y();
    imu_msg.angular_velocity.z = gyro.data_.z();

    // Serialize
    rclcpp::SerializedMessage imu_serialized_msg;
    imu_serializer.serialize_message(&imu_msg, &imu_serialized_msg);

    // convert builtin_interfaces::msg::Time to rclcpp::Time
    rclcpp::Time ros2_time(ros_time.sec, ros_time.nanosec);
    auto imu_msg_ptr = std::make_shared<rclcpp::SerializedMessage>(imu_serialized_msg);
    bag.write(imu_msg_ptr, "/gopro/imu", "sensor_msgs/msg/Imu", ros2_time);
    // bag.write(imu_serialized_msg, "/gopro/imu", "sensor_msgs/msg/Imu", ros2_time);

    accl_queue.pop_front();
    gyro_queue.pop_front();
  }

  // write magnetometer data
  rclcpp::Serialization<sensor_msgs::msg::MagneticField> mag_serializer;

  while (!magnetometer_queue.empty()) {
    MagMeasurement mag = magnetometer_queue.front();
    Timestamp stamp = mag.timestamp_;

    builtin_interfaces::msg::Time ros_time;
    ros_time.sec = stamp / 1000000000;
    ros_time.nanosec = stamp % 1000000000;
    sensor_msgs::msg::MagneticField magnetic_field_msg;
    magnetic_field_msg.header.stamp = ros_time;
    magnetic_field_msg.header.frame_id = "body";
    magnetic_field_msg.magnetic_field.x = mag.magfield_.x();
    magnetic_field_msg.magnetic_field.y = mag.magfield_.y();
    magnetic_field_msg.magnetic_field.z = mag.magfield_.z();

    // Serialize
    rclcpp::SerializedMessage mag_serialized_msg;
    mag_serializer.serialize_message(&magnetic_field_msg, &mag_serialized_msg);

    // convert builtin_interfaces::msg::Time to rclcpp::Time
    rclcpp::Time ros2_time(ros_time.sec, ros_time.nanosec);
    auto mag_msg_ptr = std::make_shared<rclcpp::SerializedMessage>(mag_serialized_msg);
    bag.write(mag_msg_ptr, "/gopro/magnetic_field", "sensor_msgs/msg/MagneticField", ros2_time);
    // bag.write(mag_serialized_msg, "/gopro/magnetic_field", "sensor_msgs/msg/MagneticField", ros2_time);

    magnetometer_queue.pop_front();
  }
  rclcpp::shutdown();
  return 0;
}
