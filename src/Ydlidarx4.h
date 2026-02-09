/*
 * YDLidar X4 Driver for Arduino
 * State machine exactly matching ROS driver
 */

#pragma once

#include "Arduino.h"
#include <iostream>
#include <functional>
#include <vector>

// Maximum samples per scan (~720 for X4 at 0.5° resolution)
#define YDLIDAR_MAX_SAMPLES 800
const size_t kLaserScanMaxNumSamplesYDLIDARX4 = 1024;//4096;

// Result codes
#define RESULT_OK       0
#define RESULT_FAIL     1
#define RESULT_TIMEOUT  2

typedef uint8_t result_t;

// State machine states - EXACTLY MATCHING ROS DRIVER
enum class YDLidarState {
  kIddle,  // Note: Matches ROS typo "kIddle" instead of "kIdle"
  kPH1,    // Package header byte 1 (0xAA)
  kPH2,    // Package header byte 2 (0x55)
  kCT,     // Package type
  kLSN,    // Sample quantity
  kFSA1,   // First sample angle byte 1
  kFSA2,   // First sample angle byte 2
  kLSA1,   // Last sample angle byte 1
  kLSA2,   // Last sample angle byte 2
  kCS1,    // Checksum byte 1
  kCS2,    // Checksum byte 2
  kSi1,    // Sample distance byte 1
  kSi2     // Sample distance byte 2
};

// Scan point structure
struct ScanPoint {
  float angle;      // Angle in degrees (0-360)
  float distance;   // Distance in mm
};

class YDLidarX4 {
public:
  YDLidarX4();
  ~YDLidarX4();

  // Initialize serial connection
  bool begin(HardwareSerial &serialobj, uint32_t baudrate = 128000);
  
  // Close serial connection
  void end();
  
  // Check if serial is open
  bool isOpen();
  
  // Start scanning
  void start();
  
  // Stop scanning
  void stop();
  
  // Process incoming serial data (call this frequently in loop())
  // Returns true when a complete 360° scan is ready
  bool processData();
  
  // Get 720-point scan (0.5° resolution) in meters
  // Returns pointer to array of 720 values (one per 0.5 degree)
  // Check if a new 360° scan is ready
  bool isScanReady() { return scan_360_ready_; }
  
  // Clear the scan ready flag after processing
  void clearScanReady() { scan_360_ready_ = false; }
  
  // Get scan rate in Hz (calculated from last scan interval)
  float getScanRate() { return scan_rate_; }
  
  // Get number of complete 360° scans processed
  unsigned long getNumScans() { return num_scans_; }
  
  // Get number of points in last complete scan
  uint16_t getLastScanPointCount() { return last_point_count_; }

  // NEW: Get raw point arrays (ROS style)
  // float* getDistances() { return dist_data; }
  // float* getAngles() { return ang_data; }
  uint16_t getPointCount() { return data_count; }

//private:
  void processSerialByte(uint8_t ch);
  void processLaserPackage();
  void finalizeScan();
  float rawAngleToFloat(uint16_t raw_angle);
  float normalizeAngle(float angle);
  
  HardwareSerial* serial_;
  YDLidarState state_;
  
  // Package parsing variables
  uint16_t sample_count_;        // Current sample being read
  bool pkg_zero_;          
  uint16_t pkg_num_samples_;     // Number of samples in current package
  uint16_t pkg_check_code_;       // Package checksum
  uint32_t raw_start_ang_;    // Raw start angle from package
  uint32_t raw_end_ang_;      // Raw end angle from package
  float start_ang_;           // Parsed start angle (degrees)
  float  end_ang_;             // Parsed end angle (degrees)
  uint32_t raw_dist_data_[kLaserScanMaxNumSamplesYDLIDARX4];

  std::vector<float> dist_data;
  std::vector<float> ang_data;
  size_t data_count = 0;

  inline static float rawStartEndAng2Double(const uint32_t& raw_angle) {
    return static_cast<float>(raw_angle) / 64.0f;  // >> 1 already in raw data
  }
  inline static float rawDist2Double(const uint32_t& raw_dist) {
    return static_cast<float>(raw_dist) / 4.0f;
  }

  inline float normAngDeg(float angle) {
    // Source: https://stackoverflow.com/a/11498248
    angle = fmodf(angle + 180.0f, 360.0f);
    if (angle < 0) {
      angle += 360.0f;
    }
    return angle - 180.0f;
  }

  inline double normAngDeg(double angle) {
    // Source: https://stackoverflow.com/a/11498248
    angle = fmod(angle + 180.0, 360.0);
    if (angle < 0) {
      angle += 360.0;
    }
    return angle - 180.0;
  }

  inline float normAngRad(float angle) {
    // Source: https://stackoverflow.com/a/11498248
    angle = fmodf(angle + M_PI, M_PI * 2.0f);
    if (angle < 0) {
      angle += M_PI * 2.0f;
    }
    return angle - M_PI;
  }

  inline double normAngRad(double angle) {
    // Source: https://stackoverflow.com/a/11498248
    angle = fmod(angle + M_PI, M_PI * 2.0);
    if (angle < 0) {
      angle += M_PI * 2.0;
    }
    return angle - M_PI;
  }
   
 
  // 720-point binned scan (0.5° resolution)
  // float full_scan_mm_[720];     // Accumulation buffer in mm
  // float full_scan_m_[720];      // Final output in meters
  bool scan_360_ready_;         // Flag for 360° scan completion
  // float dist_data[2000];  // Distances in meters
  // float ang_data[2000];   // Angles in radians
  //uint16_t data_count;    // Current number of points
  //uint16_t total_points_;       // Total points accumulated in current scan
  
  // Timing and statistics
  unsigned long last_scan_time_;
  float scan_rate_;
  unsigned long num_scans_;
  uint16_t last_point_count_;
};