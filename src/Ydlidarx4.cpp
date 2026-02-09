/*
 * YDLidar X4 Driver for Arduino
 * State machine exactly matching ROS driver
 * 
 * Features:
 * - Angular correction for mechanical offset
 * - 720-bin accumulation (0.5° resolution)
 * - Raw data output (no filtering)
 */

#include "Ydlidarx4.h"

YDLidarX4::YDLidarX4() 
  : serial_(nullptr),
    state_(YDLidarState::kIddle),  // Match ROS typo
    pkg_zero_(false),
    pkg_num_samples_(0),
    sample_count_(0),
    scan_360_ready_(false),
    //total_points_(0),
    data_count(0),
    last_scan_time_(0),
    scan_rate_(0.0f),
    num_scans_(0),
    last_point_count_(0)
{
  // for (int i = 0; i < 2000; i++) {
  //   dist_data[i] = 0.0f;
  //   ang_data[i] = 0.0f;
  // }
  dist_data.resize(kLaserScanMaxNumSamplesYDLIDARX4);
  ang_data.resize(kLaserScanMaxNumSamplesYDLIDARX4);
}

YDLidarX4::~YDLidarX4() {
  if (isOpen()) {
    stop();
    end();
  }
}

bool YDLidarX4::begin(HardwareSerial &serialobj, uint32_t baudrate) {
  if (isOpen()) {
    end();
  }
  serial_ = &serialobj;
  serial_->end();
  serial_->begin(baudrate);
  delay(100);
  return true;
}

void YDLidarX4::end() {
  if (isOpen()) {
    serial_->end();
    serial_ = nullptr;
  }
}

bool YDLidarX4::isOpen() {
  return serial_ != nullptr;
}

void YDLidarX4::start() {
  if (!isOpen()) return;
  
  uint8_t cmd[2] = {0xA5, 0x60};
  serial_->write(cmd, 2);
  
  state_ = YDLidarState::kIddle;
  scan_360_ready_ = false;
  //total_points_ = 0;
  data_count = 0;
  last_scan_time_ = millis();
  
  // for (int i = 0; i < 2000; i++) {
  //   dist_data[i] = 0.0f;
  //   ang_data[i] = 0.0f;
  // }
}

void YDLidarX4::stop() {
  if (!isOpen()) return;
  
  uint8_t cmd[2] = {0xA5, 0x65};
  serial_->write(cmd, 2);
}

bool YDLidarX4::processData() {
  if (!isOpen()) return false;

  bool scan_completed = false;
  while (serial_->available() > 0) {
    uint8_t ch = serial_->read();
    processSerialByte(ch);
    
    if (scan_360_ready_ && !scan_completed) {
      scan_completed = true;
    }
  }
  
  return scan_completed;
}

void YDLidarX4::processSerialByte(uint8_t ch) {

  switch (state_) {
    case YDLidarState::kPH1:
      if (ch == 0x55) {
        state_ = YDLidarState::kPH2;
      } else {
        state_ = YDLidarState::kIddle;
      }
      break;
      
    case YDLidarState::kPH2:
      state_ = YDLidarState::kCT;
      break;
      
    case YDLidarState::kCT:
      state_ = YDLidarState::kLSN;
      break;
      
    case YDLidarState::kLSN:
      state_ = YDLidarState::kFSA1;
      break;
      
    case YDLidarState::kFSA1:
      state_ = YDLidarState::kFSA2;
      break;
      
    case YDLidarState::kFSA2:
      state_ = YDLidarState::kLSA1;
      break;
      
    case YDLidarState::kLSA1:
      state_ = YDLidarState::kLSA2;
      break;
      
    case YDLidarState::kLSA2:
      state_ = YDLidarState::kCS1;
      break;
      
    case YDLidarState::kCS1:
      state_ = YDLidarState::kCS2;
      break;
      
    case YDLidarState::kCS2:
      state_ = YDLidarState::kSi1;
      sample_count_ = 0;
      break;
      
    case YDLidarState::kSi1:
      state_ = YDLidarState::kSi2;
      break;
      
    case YDLidarState::kSi2:
      if (sample_count_ >= pkg_num_samples_) {
        if (ch == 0xAA) {
          state_ = YDLidarState::kPH1;
          processLaserPackage();
        } else {
          state_ = YDLidarState::kIddle;
        }
      } else {
        state_ = YDLidarState::kSi1;
      }
      break;
      
    case YDLidarState::kIddle:
      if (ch == 0xAA) {
        state_ = YDLidarState::kPH1;
      }
      break;
  }


  switch (state_) {
    case YDLidarState::kPH1:
    case YDLidarState::kPH2:
      break;
    case YDLidarState::kCT:
      if ((ch & 0x01) == 0x01) {
        pkg_zero_ = true;
        
        if (data_count > 0) {  // ROS checks pubLaserData existence
          scan_360_ready_ = true;
          last_point_count_ = data_count;
          
          // Calculate scan rate
          unsigned long now = millis();
          if (last_scan_time_ > 0) {
            unsigned long interval = now - last_scan_time_;
            if (interval > 0) {
              scan_rate_ = 1000.0f / interval;
            }
          }
          last_scan_time_ = now;
          num_scans_++;
        }
        data_count = 0;  // ROS resets here
        
      } else {
        pkg_zero_ = false;
      }
      break;

    case YDLidarState::kLSN:
      pkg_num_samples_ = (ch & 0x00FF);
      break;
      
    case YDLidarState::kFSA1:
      raw_start_ang_ = (ch & 0x00FF);
      break;
      
    case YDLidarState::kFSA2:
      raw_start_ang_ = ((raw_start_ang_ | (ch << 8)) >> 1);
      start_ang_ = (float)raw_start_ang_ / 64.0f;
      break;
      
    case YDLidarState::kLSA1:
      raw_end_ang_ = ch & 0x00FF;
      break;
      
    case YDLidarState::kLSA2:
      raw_end_ang_ = ((raw_end_ang_ | (ch << 8)) >> 1);
       end_ang_ = (float)raw_end_ang_ / 64.0f;
      break;
      
    case YDLidarState::kCS1:
      pkg_check_code_ = ch & 0x00FF;
      break;
      
    case YDLidarState::kCS2:
      pkg_check_code_ = (pkg_check_code_ | (ch << 8));
      break;
      
    case YDLidarState::kSi1:
       raw_dist_data_[sample_count_] = ch & 0x00FF;
      break;
      
    case YDLidarState::kSi2:
       raw_dist_data_[sample_count_] = ( raw_dist_data_[sample_count_] | (ch << 8));
      sample_count_++;
      break;
      
    case YDLidarState::kIddle:
      sample_count_ = 0;
      break;
  }
}

void YDLidarX4::processLaserPackage() {

  if ((!pkg_zero_) && (sample_count_ > 0)) {
    //Serial.println("Processing laser package with " + String(sample_count_) + " samples");
    bool is_sample_ok;
    float angle_correction, delta_ang;
    
    for (size_t i = 0; i < sample_count_; i++) {
      
      dist_data[data_count] = rawDist2Double(raw_dist_data_[i]);

      delta_ang = end_ang_ - start_ang_;
      if (start_ang_ > end_ang_) {
        delta_ang += 360.0;
      }

      if (raw_dist_data_[i] == 0) {
        angle_correction = 0;
      } else {
        angle_correction =
            atanf(21.8f * (155.3f - dist_data[data_count]) /
                (155.3f * dist_data[data_count])) * 360.0f / M_PI;
      }

      ang_data[data_count] = normAngRad(-(start_ang_ + delta_ang * static_cast<float>(i) 
      / (static_cast<float>(sample_count_) - 1.0f) + angle_correction) * M_PI / 180.0f);
      
      // distance data (m)
      dist_data[data_count] /= 1000.0f;

      

      is_sample_ok = true;
      // if ((dist_data[data_count] < 0.12f) ||
      //     (dist_data[data_count] > 10.0f)) {
      //   is_sample_ok = false;
      // }

      // if ((ang_data[data_count] < ang_min_) ||
      //     (ang_data[data_count] > ang_max_)) {
      //   is_sample_ok = false;
      // }

      if (is_sample_ok) {
        data_count++;
      }
    }
    //Serial.println("Processed laser package with " + String(sample_count_) + " samples, " + String(data_count) + " valid points");
  }
}

float YDLidarX4::rawAngleToFloat(uint16_t raw_angle) {
  return (float)raw_angle / 64.0f;
}

float YDLidarX4::normalizeAngle(float angle) {
  while (angle < 0.0f) angle += 360.0f;
  while (angle >= 360.0f) angle -= 360.0f;
  return angle;
}