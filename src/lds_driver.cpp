#include "lds_driver.hpp"


namespace lds
{
LFCDLaser::LFCDLaser(uint32_t baud_rate)
{
  Serial1.begin(baud_rate); //set baud rate
  // Below command is not required after firmware upgrade (2017.10)
  Serial1.write("b");  // start motor
}

LFCDLaser::~LFCDLaser()
{
  Serial1.write("e");  // stop motor
}

void LFCDLaser::poll()
{
  uint8_t temp_char;
  uint8_t start_count = 0;
  bool got_scan = false;
  std::array<uint8_t, 2520> raw_bytes;
  uint8_t good_sets = 0;
  uint32_t motor_speed = 0;
  rpms = 0;
  int index;

  //Serial.print("AAAAAAAAAAAAA");
  while (/*!shutting_down_ && !got_scan*/got_scan==false && shutting_down_==false) {
    // Wait until first data sync of frame: 0xFA, 0xA0
    raw_bytes[start_count] = Serial1.read();  

    if (start_count == 0) {
      if (raw_bytes[start_count] == 0xFA) {
        start_count = 1;
      }
    } else if (start_count == 1) {
      if (raw_bytes[start_count] >= 0xA0) {
        start_count = 0;
        

        // Now that entire start sequence has been found, read in the rest of the message
        got_scan = true;
        uint16_t bytes_read = 0;
        while (bytes_read < 2518 && !shutting_down_) {
          if (Serial1.available()) {
            raw_bytes[2 + bytes_read] = Serial1.read();
            bytes_read++;
            //Serial.println(bytes_read);
          }
        }


        // scan->angle_min = 0.0;
        // scan->angle_max = 2.0*M_PI;
        // scan->angle_increment = (2.0*M_PI/360.0);
        // scan->range_min = 0.12;
        // scan->range_max = 3.5;
        // scan->ranges.resize(360);
        // scan->intensities.resize(360);

        // read data in sets of 6
        
        for (uint16_t i = 0; i < raw_bytes.size(); i = i + 42) {
          if (raw_bytes[i] == 0xFA /*&& raw_bytes[i + 1] == (0xA0 + i/42)*/) {
            good_sets++;
            motor_speed += (raw_bytes[i + 3] << 8) + raw_bytes[i + 2];
            rpms = (raw_bytes[i + 3] << 8 | raw_bytes[i + 2]) / 10;

            for (uint16_t j = i + 4; j < i + 40; j = j + 6) {
              index = 6 * (i / 42) + (j - 4 - i) / 6;

              // uint8_t b0 = raw_bytes[j];     // distance LSB
              // uint8_t b1 = raw_bytes[j + 1]; // distance MSB
              // uint8_t b2 = raw_bytes[j + 2]; // intensity LSB
              // uint8_t b3 = raw_bytes[j + 3]; // intensity MSB
              // uint8_t b4 = raw_bytes[j + 4]; // reserved / flags
              // uint8_t b5 = raw_bytes[j + 5]; // reserved / flags

              // uint16_t range = (b1 << 8) | b0;
              // uint16_t intensity = (b3 << 8) | b2;
              // b4/b5 → you can use for flags or ignore

              //Four bytes per reading
              uint8_t byte0 = raw_bytes[j];
              uint8_t byte1 = raw_bytes[j + 1];
              uint8_t byte2 = raw_bytes[j + 2];
              uint8_t byte3 = raw_bytes[j + 3];

              // Remaining bits are the range in mm
              uint16_t intensity = (byte1 << 8) + byte0;

              // Last two bytes represent the uncertanty or intensity,
              // might also be pixel area of target...
              // uint16_t intensity = (byte3 << 8) + byte2;
              uint16_t range = (byte3 << 8) + byte2;

              // scan->ranges[359-index] = range / 1000.0;
              // scan->intensities[359-index] = intensity;
              //printf("r[%d]=%f,", 359 - index, range / 1000.0);
                if (range > 0) {
                    // Assuming the scan is in reverse order, we can use the index directly
                    // scan->ranges[index] = range / 1000.0;
                    // scan->intensities[index] = intensity;
                    Serial.print("r[");
                    Serial.print(index);
                    Serial.print("]=");
                    Serial.print(range / 1000.0);
                    Serial.println(",");
                } else {
                    // If range is zero, we can set it to NaN or some other value
                    // scan->ranges[index] = std::numeric_limits<float>::quiet_NaN();
                    Serial.print("r[");
                    Serial.print(index);
                    Serial.println("]=NaN,");
                }
            }
          }
        }

        // scan->time_increment = motor_speed/good_sets/1e8;
      } else {
        start_count = 0;
      }
    }
  }
} // namespace lds
}
// int main(int argc, char ** argv)
// {
//   std::string port;
//   int baud_rate;
//   uint16_t rpms;
//   port = "/dev/ttyUSB0";
//   baud_rate = 230400;



//     lds::LFCDLaser laser(baud_rate);

//     while (1) {
//       laser.poll();
//     }
//     laser.close();

//     return 0;
// }