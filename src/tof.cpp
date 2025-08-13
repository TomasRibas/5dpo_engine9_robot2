#include "tof.h"

// Constructor: use Wire on default I2C and no XSHUT pin (always-on)
TOF::TOF(): sensor(&Wire, -1)  // -1 = no shutdown pin
{
}

void TOF::initializeToFSensor() {
  // Probe and start sensor
  while (sensor.begin() == 0) {
    Serial.println(F("Failed to initialize VL53L4CX!"));
    //while (1);
  }
  // Optionally set custom address here, default is 0x29
  // sensor.InitSensor(0x29);

  // Start continuous ranging
  sensor.VL53L4CX_StartMeasurement();

}

void TOF::calculateTOF() {
  uint8_t ready = 0;
  // Poll data-ready flag
  if ((sensor.VL53L4CX_GetMeasurementDataReady(&ready) == 0) && ready) {
    // Fetch multi-ranging data
    if (sensor.VL53L4CX_GetMultiRangingData(&ranging_data) == 0) {
    prev_distance_tof = distance_tof;
    if (ranging_data.NumberOfObjectsFound > 0) {
    // Convert mm to meters
    distance_tof = ranging_data.RangeData[0].RangeMilliMeter * 1e-3f;
    }
    }
    // Clear interrupt and start next measurement
    sensor.VL53L4CX_ClearInterruptAndStartMeasurement();

        // Toggle LED every few loops
        if (++loop_count > 5) {
            loop_count = 0;
            led_state = !led_state;
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_state);
        }
  }

}

