/*
 * YDLidar X4 Driver for Arduino
 * Based on 5DPO ROS driver
 * WITH TIMEOUT-BASED TEMPORAL FILTERING
 */

#pragma once

#include "Arduino.h"

// Maximum samples per scan (~720 for X4 at 0.5° resolution)
#define YDLIDAR_MAX_SAMPLES 800

// Result codes
#define RESULT_OK       0
#define RESULT_FAIL     1
#define RESULT_TIMEOUT  2

typedef uint8_t result_t;

// State machine states for parsing
enum class YDLidarState {
  kIdle,
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
  float* get360Scan() { return full_scan_m_; }
  
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
  
  // ========== OUTLIER FILTERING FUNCTIONS ==========
  
  // Enable/disable outlier filtering (enabled by default)
  void setOutlierFilterEnabled(bool enabled);
  
  // Set outlier filter parameters
  // min_neighbors: minimum number of nearby points required (default: 2)
  // angle_window: angular window in bins to check for neighbors (default: 5 = ±2.5°)
  void setOutlierFilterParams(int min_neighbors, int angle_window);
  
  // ========== TEMPORAL PERSISTENCE CONTROL ==========
  
  // Set how many scans to keep old data before clearing it
  // Default: 3 scans (~150ms at 5Hz)
  // Higher = more stable but slower to remove ghost beacons
  // Lower = faster removal but more flickering
  void setTemporalPersistence(int num_scans);
  
  // =================================================

private:
  void processSerialByte(uint8_t ch);
  void processLaserPackage();
  void finalizeScan();
  void filterOutliers();
  void updateTemporalPersistence();
  float rawAngleToFloat(uint16_t raw_angle);
  float normalizeAngle(float angle);
  
  HardwareSerial* serial_;
  YDLidarState state_;
  
  // Package parsing variables
  bool pkg_is_start_;           // Is this a scan start package?
  uint8_t pkg_num_samples_;     // Number of samples in current package
  uint16_t raw_start_angle_;    // Raw start angle from package
  uint16_t raw_end_angle_;      // Raw end angle from package
  float start_angle_;           // Parsed start angle (degrees)
  float end_angle_;             // Parsed end angle (degrees)
  uint16_t pkg_checksum_;       // Package checksum
  uint16_t raw_distances_[40];  // Raw distance data for current package (max 40 samples per pkg)
  uint8_t sample_count_;        // Current sample being read
  
  // 720-point binned scan (0.5° resolution)
  float full_scan_mm_[720];     // Accumulation buffer in mm
  float full_scan_m_[720];      // Final output in meters
  bool scan_360_ready_;         // Flag for 360° scan completion
  uint16_t total_points_;       // Total points accumulated in current scan
  
  // Timing and statistics
  unsigned long last_scan_time_;
  float scan_rate_;
  unsigned long num_scans_;
  uint16_t last_point_count_;
  
  // Outlier filtering parameters
  bool enable_outlier_filter_;
  int outlier_neighbor_threshold_;  // Minimum neighbors required
  int outlier_angle_window_;        // Angular window to check (in bins)
  
  // Temporal persistence tracking
  uint8_t scan_age_[720];           // How many scans since last update for each bin
  int temporal_persistence_limit_;   // Max scans to keep old data
};