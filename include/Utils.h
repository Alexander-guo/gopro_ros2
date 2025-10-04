//
// Created by bjoshi on 8/26/20.
//

#pragma once

#include <Eigen/Core>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <ostream>
#include <string>

uint64_t parseISO(const std::string& iso_date);
std::string uint64_to_string(uint64_t value);
uint64_t get_offset_1904();

struct BagConfig {
  std::string uri;
  std::string storage_id;
};

/**
 * @brief Infer storage_id and normalized bag file path
 * 
 * Rules:
 *   - ".mcap"  → storage_id = "mcap"
 *   - ".db3"   → storage_id = "sqlite3"
 *   - no ext   → default storage_id = "sqlite3" and ".db3" is appended
 *   - unknown ext → storage_id = "sqlite3"
 * 
 * @param bag_path input path (may or may not have extension)
 * @return BagConfig containing uri and storage_id
 */
BagConfig infer_bag_config(const std::string &bag_path);

class ProgressBar {
private:
  static const auto overhead = sizeof " [100%]";
  std::ostream& os;
  const std::size_t bar_width;
  std::string message;
  const std::string full_bar;

public:
  ProgressBar(std::ostream& os,
              std::size_t line_width,
              std::string message_,
              const char symbol = '.');

  // not copyable
  ProgressBar(const ProgressBar&) = delete;
  ProgressBar& operator=(const ProgressBar&) = delete;

  ~ProgressBar();

  void write(double fraction);
};

// Inertial containers.
using Timestamp = uint64_t;
using ImuStamp = Timestamp;
// First 3 elements correspond to acceleration data [m/s^2]
// while the 3 last correspond to angular velocities [rad/s].
using ImuAccGyr = Eigen::Matrix<double, 6, 1>;
using ImuAccl = Eigen::Matrix<double, 3, 1>;
using ImuGyro = Eigen::Matrix<double, 3, 1>;
using MagneticField = Eigen::Matrix<double, 3, 1>;

struct MagMeasurement {
  MagMeasurement() = default;
  MagMeasurement(const Timestamp& timestamp, const MagneticField& magnetic_field)
      : timestamp_(timestamp), magfield_(magnetic_field) {}
  MagMeasurement(Timestamp&& timestamp, MagneticField&& magnetic_field)
      : timestamp_(std::move(timestamp)), magfield_(std::move(magnetic_field)) {}

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Timestamp timestamp_;
  MagneticField magfield_;
};

struct ImuMeasurement {
  ImuMeasurement() = default;
  ImuMeasurement(const ImuStamp& timestamp, const ImuAccGyr& imu_data)
      : timestamp_(timestamp), acc_gyr_(imu_data) {}
  ImuMeasurement(ImuStamp&& timestamp, ImuAccGyr&& imu_data)
      : timestamp_(std::move(timestamp)), acc_gyr_(std::move(imu_data)) {}

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ImuStamp timestamp_;
  ImuAccGyr acc_gyr_;
};

struct AcclMeasurement {
  AcclMeasurement() = default;
  AcclMeasurement(const ImuStamp& timestamp, const ImuAccl& accl_data)
      : timestamp_(timestamp), data_(accl_data) {}
  AcclMeasurement(ImuStamp&& timestamp, ImuAccl& accl_data)
      : timestamp_(std::move(timestamp)), data_(std::move(accl_data)) {}

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ImuStamp timestamp_;
  ImuAccl data_;
};

struct GyroMeasurement {
  GyroMeasurement() = default;
  GyroMeasurement(const ImuStamp& timestamp, const ImuAccl& accl_data)
      : timestamp_(timestamp), data_(accl_data) {}
  GyroMeasurement(ImuStamp&& timestamp, ImuAccl& accl_data)
      : timestamp_(std::move(timestamp)), data_(std::move(accl_data)) {}

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ImuStamp timestamp_;
  ImuGyro data_;
};