#ifndef APPLICATIONS_LDS_DRIVERLDS_DRIVER_HPP
#define APPLICATIONS_LDS_DRIVERLDS_DRIVER_HPP

#include <string>
//#include <async>
#include <array>
#include <Arduino.h>
namespace lds
{
class LFCDLaser
{
public:
  uint16_t rpms;  ///< @brief RPMS derived from the rpm bytes in an LFCD packet
  /**
    * @brief Constructs a new LFCDLaser attached to the given serial port
  * @param port The string for the serial port device to attempt to connect to, e.g. "/dev/ttyUSB0"
  * @param baud_rate The baud rate to open the serial port at.
  * @param io Boost ASIO IO Service to use when creating the serial port object
  */

  LFCDLaser(uint32_t baud_rate);
  /**
  * @brief Default destructor
  */
  ~LFCDLaser();

  /**
  * @brief Poll the laser to get a new scan. Blocks until a complete new scan is received or close is called.
  * @param scan LaserScan message pointer to fill in with the scan. The caller is responsible for filling in the ROS timestamp and frame_id
  */

  void poll();

  /**
  * @brief Close the driver down and prevent the polling loop from advancing
  */

  void close() { shutting_down_ = true; };


private:
  // @brief The serial port the driver is attached to
  std::string port_;
  // @brief The baud rate for the serial connection
  uint32_t baud_rate_;
  // @brief Flag for whether the driver is supposed to be shutting down or not
  bool shutting_down_=false;
  // @brief Actual serial port object for reading/writing to the LFCD Laser Scanner
  //boost::asio::serial_port serial_;
  // @brief current motor speed as reported by the LFCD.
  uint16_t motor_speed_;
    // latest scan in meters; NaN when invalid/missed
  float ranges[360];
  volatile bool new_scan = false;

};
}  // namespace lds
#endif  // APPLICATIONS_LDS_DRIVERLDS_DRIVER_HPP