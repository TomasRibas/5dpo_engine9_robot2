/*
 * YDLidar X4 Driver for Arduino - FINAL PRODUCTION VERSION
 * 
 * Features:
 * - Correct packet parsing per YDLIDAR X4 protocol
 * - Temporal gap filling (keeps previous valid reading for intermittent angles)
 * - Proper distance filtering (120mm - 10000mm per X4 spec)
 * - Configurable angle direction (CW or CCW)
 */

#include "Ydlidarx4.h"

YDLidarX4::YDLidarX4() 
  : serial_(nullptr),
    state_(YDLidarState::kIdle),
    pkg_is_start_(false),
    pkg_num_samples_(0),
    sample_count_(0),
    scan_360_ready_(false),
    total_points_(0),
    last_scan_time_(0),
    scan_rate_(0.0f),
    num_scans_(0),
    last_point_count_(0)
{
  for (int i = 0; i < 360; i++) {
    full_scan_mm_[i] = 0.0f;
    full_scan_m_[i] = 0.0f;
  }
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
  
  state_ = YDLidarState::kIdle;
  scan_360_ready_ = false;
  total_points_ = 0;
  last_scan_time_ = millis();
  
  for (int i = 0; i < 360; i++) {
    full_scan_mm_[i] = 0.0f;
    full_scan_m_[i] = 0.0f;
  }
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
    case YDLidarState::kIdle:
      sample_count_ = 0;
      if (ch == 0xAA) {
        state_ = YDLidarState::kPH1;
      }
      break;
      
    case YDLidarState::kPH1:
      if (ch == 0x55) {
        state_ = YDLidarState::kPH2;
      } else if (ch == 0xAA) {
        state_ = YDLidarState::kPH1;
      } else {
        state_ = YDLidarState::kIdle;
      }
      break;
      
    case YDLidarState::kPH2:
      if ((ch & 0x01) == 0x01) {
        pkg_is_start_ = true;
        if (total_points_ > 0) {
          finalizeScan();
        }
      } else {
        pkg_is_start_ = false;
      }
      state_ = YDLidarState::kCT;
      break;
      
    case YDLidarState::kCT:
      pkg_num_samples_ = ch;
      state_ = YDLidarState::kLSN;
      break;
      
    case YDLidarState::kLSN:
      raw_start_angle_ = ch;
      state_ = YDLidarState::kFSA1;
      break;
      
    case YDLidarState::kFSA1:
      raw_start_angle_ = (raw_start_angle_ | ((uint16_t)ch << 8)) >> 1;
      start_angle_ = (float)raw_start_angle_ / 64.0f;
      state_ = YDLidarState::kFSA2;
      break;
      
    case YDLidarState::kFSA2:
      raw_end_angle_ = ch;
      state_ = YDLidarState::kLSA1;
      break;
      
    case YDLidarState::kLSA1:
      raw_end_angle_ = (raw_end_angle_ | ((uint16_t)ch << 8)) >> 1;
      end_angle_ = (float)raw_end_angle_ / 64.0f;
      state_ = YDLidarState::kLSA2;
      break;
      
    case YDLidarState::kLSA2:
      pkg_checksum_ = ch;
      state_ = YDLidarState::kCS1;
      break;
      
    case YDLidarState::kCS1:
      pkg_checksum_ = pkg_checksum_ | ((uint16_t)ch << 8);
      sample_count_ = 0;
      
      if (pkg_num_samples_ == 0) {
        state_ = YDLidarState::kIdle;
      } else {
        state_ = YDLidarState::kCS2;
      }
      break;
      
    case YDLidarState::kCS2:
      if (sample_count_ < 40) {
        raw_distances_[sample_count_] = ch;
      }
      state_ = YDLidarState::kSi1;
      break;
      
    case YDLidarState::kSi1:
      if (sample_count_ < 40) {
        raw_distances_[sample_count_] |= ((uint16_t)ch << 8);
      }
      sample_count_++;
      
      if (sample_count_ >= pkg_num_samples_) {
        processLaserPackage();
        state_ = YDLidarState::kIdle;
      } else {
        state_ = YDLidarState::kCS2;
      }
      break;
      
    default:
      state_ = YDLidarState::kIdle;
      break;
  }
}

void YDLidarX4::processLaserPackage() {
  if (pkg_is_start_) {
    return;
  }
  
  if (sample_count_ == 0) {
    return;
  }
  
  float delta_angle = end_angle_ - start_angle_;
  if (delta_angle < 0.0f) {
    delta_angle += 360.0f;
  }
  
  for (uint8_t i = 0; i < sample_count_; i++) {
    uint16_t raw_dist = raw_distances_[i];
    
    // Skip hardware zeros
    if (raw_dist == 0) {
      continue;
    }
    
    // Distance in mm (raw / 4 per YDLIDAR protocol)
    float distance_mm = (float)raw_dist / 4.0f;
    
    // Filter by YDLIDAR X4 specifications: 120mm - 10000mm
    if (distance_mm < 120.0f || distance_mm > 10000.0f) {
      continue;
    }
    
    // Calculate angle for this sample
    float angle;
    if (sample_count_ > 1) {
      angle = start_angle_ + delta_angle * (float)i / ((float)sample_count_ - 1.0f);
    } else {
      angle = start_angle_;
    }
    
    // Normalize to 0-360
    while (angle < 0.0f) angle += 360.0f;
    while (angle >= 360.0f) angle -= 360.0f;
    
    // ============================================================
    // ANGLE DIRECTION CONFIGURATION
    // ============================================================
    // YDLIDAR X4 spins clockwise (CW) when viewed from above
    // Uncomment the following to convert to CCW (standard math convention):
    
    // angle = 360.0f - angle;
    // if (angle >= 360.0f) angle = 0.0f;
    
    // ============================================================
    
    // Bin to integer degree
    int idx = (int)(angle + 0.5f);
    if (idx >= 360) idx = 0;
    if (idx < 0) idx = 359;
    
    // Store minimum distance per bin (closest obstacle wins)
    if (full_scan_mm_[idx] == 0.0f || distance_mm < full_scan_mm_[idx]) {
      full_scan_mm_[idx] = distance_mm;
    }
    
    total_points_++;
  }
}

void YDLidarX4::finalizeScan() {
  // Convert mm to meters with TEMPORAL GAP FILLING
  // If current scan has no data for an angle, keep the previous valid reading
  for (int i = 0; i < 360; i++) {
    if (full_scan_mm_[i] > 0.0f) {
      // New valid reading - update output
      full_scan_m_[i] = full_scan_mm_[i] * 0.001f;
    }
    // If full_scan_mm_[i] == 0, keep previous value in full_scan_m_[i]
    // This provides temporal gap filling for intermittent readings
  }
  
  scan_360_ready_ = true;
  num_scans_++;
  last_point_count_ = total_points_;
  
  // Calculate scan rate
  unsigned long now = millis();
  if (last_scan_time_ > 0) {
    unsigned long interval = now - last_scan_time_;
    if (interval > 0) {
      scan_rate_ = 1000.0f / interval;
    }
  }
  last_scan_time_ = now;
  
  // Reset accumulation buffer for next scan
  for (int i = 0; i < 360; i++) {
    full_scan_mm_[i] = 0.0f;
  }
  total_points_ = 0;
}

float YDLidarX4::rawAngleToFloat(uint16_t raw_angle) {
  return (float)raw_angle / 64.0f;
}

float YDLidarX4::normalizeAngle(float angle) {
  while (angle < 0.0f) angle += 360.0f;
  while (angle >= 360.0f) angle -= 360.0f;
  return angle;
}