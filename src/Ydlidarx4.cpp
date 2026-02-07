/*
 * YDLidar X4 Driver for Arduino - WITH TIMEOUT-BASED TEMPORAL FILTERING
 * 
 * Features:
 * - Angular correction for mechanical offset
 * - Outlier filtering (removes isolated points)
 * - Timeout-based temporal persistence (keeps data for N scans, then clears)
 * - Prevents ghost beacons while reducing flickering
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
    last_point_count_(0),
    enable_outlier_filter_(false),
    outlier_neighbor_threshold_(2),
    outlier_angle_window_(5),
    temporal_persistence_limit_(0)  // Keep data for 3 scans (~150ms)
{
  for (int i = 0; i < 720; i++) {
    full_scan_mm_[i] = 0.0f;
    full_scan_m_[i] = 0.0f;
    scan_age_[i] = 255;  // 255 = no data
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
  
  for (int i = 0; i < 720; i++) {
    full_scan_mm_[i] = 0.0f;
    full_scan_m_[i] = 0.0f;
    scan_age_[i] = 255;
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
  
  // Initialize bin_counts
  int bin_counts[720];
  for (int i = 0; i < 720; i++) {
    bin_counts[i] = 0;
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
    if (distance_mm < 10.0f/*120.0f*/  || distance_mm > 4000.0f) { ///CHANGED DISTANCE LIMITS from 120mm-10m to 10mm-4m to allow  only closer points (for better beacon detection)
      continue;
    }
    
    // Calculate angle for this sample
    float angle;
    if (sample_count_ > 1) {
      angle = start_angle_ + delta_angle * (float)i / ((float)sample_count_ - 1.0f);
    } else {
      angle = start_angle_;
    }
    
    // Angular correction for mechanical offset
    float angle_correction;
    if (distance_mm == 0.0f) {
      angle_correction = 0.0f;
    } else {
      angle_correction = atan2f(21.8f * (155.3f - distance_mm), 
                                 155.3f * distance_mm) * 180.0f / PI;
    }
    
    // Apply correction to angle
    angle = angle + angle_correction;
    
    // Normalize to 0-360
    while (angle < 0.0f) angle += 360.0f;
    while (angle >= 360.0f) angle -= 360.0f;
    
    // Bin to 0.5° resolution (720 bins)
    int idx = (int)(angle * 2.0f + 0.5f);
    if (idx >= 720) idx = 0;
    if (idx < 0) idx = 719;
    
    // Accumulate instead of minimum
    if (bin_counts[idx] == 0) {
      full_scan_mm_[idx] = distance_mm;
    } else {
      full_scan_mm_[idx] += distance_mm;
    }
    bin_counts[idx]++;
    
    total_points_++;
  }
  
  // Compute averages
  for (int i = 0; i < 720; i++) {
    if (bin_counts[i] > 1) {
      full_scan_mm_[i] = full_scan_mm_[i] / bin_counts[i];
    }
  }
}

void YDLidarX4::finalizeScan() {
  // Apply outlier filtering BEFORE converting to meters
  if (enable_outlier_filter_) {
    filterOutliers();
  }
  
  // ========== TIMEOUT-BASED TEMPORAL PERSISTENCE ==========
  updateTemporalPersistence();
  // ========================================================
  
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
  for (int i = 0; i < 720; i++) {
    full_scan_mm_[i] = 0.0f;
  }
  total_points_ = 0;
}

void YDLidarX4::updateTemporalPersistence() {
  // For each bin, update age and decide whether to keep or clear data
  for (int i = 0; i < 720; i++) {
    if (full_scan_mm_[i] > 0.0f) {
      // New data received - reset age and update value
      scan_age_[i] = 0;
      full_scan_m_[i] = full_scan_mm_[i] * 0.001f;
    } else {
      // No new data - age the existing data
      if (scan_age_[i] < 255) {  // 255 = already cleared
        scan_age_[i]++;
        
        // Check if data is too old
        if (scan_age_[i] >= temporal_persistence_limit_) {
          // Data expired - clear it
          full_scan_m_[i] = 0.0f;
          scan_age_[i] = 255;  // Mark as cleared
        }
        // else: keep previous value (temporal persistence)
      }
    }
  }
}

void YDLidarX4::filterOutliers() {
  // Remove isolated points that don't have enough neighbors
  // This helps eliminate noise and phantom reflections
  
  for (int i = 0; i < 720; i++) {
    if (full_scan_mm_[i] == 0.0f) {
      continue; // Skip empty bins
    }
    
    // Count valid neighbors within angular window
    int neighbor_count = 0;
    float current_dist = full_scan_mm_[i];
    
    // Check neighbors within +/- outlier_angle_window_ bins (default ±5 = ±2.5°)
    for (int offset = -outlier_angle_window_; offset <= outlier_angle_window_; offset++) {
      if (offset == 0) continue; // Don't count self
      
      int neighbor_idx = i + offset;
      
      // Handle wraparound at 0/360°
      if (neighbor_idx < 0) neighbor_idx += 720;
      if (neighbor_idx >= 720) neighbor_idx -= 720;
      
      if (full_scan_mm_[neighbor_idx] > 0.0f) {
        // Check if neighbor is at similar distance (within 30% or 200mm)
        float neighbor_dist = full_scan_mm_[neighbor_idx];
        float dist_diff = abs(neighbor_dist - current_dist);
        float max_allowed_diff = max(current_dist * 0.3f, 200.0f);
        
        if (dist_diff < max_allowed_diff) {
          neighbor_count++;
        }
      }
    }
    
    // If not enough neighbors, mark as outlier (set to 0)
    if (neighbor_count < outlier_neighbor_threshold_) {
      full_scan_mm_[i] = 0.0f;
    }
  }
}

void YDLidarX4::setOutlierFilterEnabled(bool enabled) {
  enable_outlier_filter_ = enabled;
}

void YDLidarX4::setOutlierFilterParams(int min_neighbors, int angle_window) {
  outlier_neighbor_threshold_ = min_neighbors;
  outlier_angle_window_ = angle_window;
}

void YDLidarX4::setTemporalPersistence(int num_scans) {
  temporal_persistence_limit_ = num_scans;
}

float YDLidarX4::rawAngleToFloat(uint16_t raw_angle) {
  return (float)raw_angle / 64.0f;
}

float YDLidarX4::normalizeAngle(float angle) {
  while (angle < 0.0f) angle += 360.0f;
  while (angle >= 360.0f) angle -= 360.0f;
  return angle;
}