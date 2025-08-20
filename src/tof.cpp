#include "tof.h"

// Constructor: use Wire on default I2C and no XSHUT pin (always-on)
TOF::TOF(): sensor(&Wire, -1) {}

void TOF::initializeToFSensor() {
  delay(2);  // let sensor boot after power

  // In this driver, 0 == OK. Anything else is an error.
  if (sensor.begin()!= 0) {
    Serial.println(F("Failed to initialize VL53L4CX (begin() != 0). Check wiring/power."));
    return;
  }

  // Start continuous ranging
  sensor.VL53L4CX_StartMeasurement();
}


void TOF::calculateTOF() {
  // Non-blocking poll: return immediately if not ready
  uint8_t ready = 0;
  //if (sensor.VL53L4CX_GetMeasurementDataReady(&ready) != 0 || !ready) {
    //return; // nothing new yet
  //}

  if (sensor.VL53L4CX_GetMultiRangingData(&ranging_data) == 0) {
    prev_distance_tof = distance_tof;
    if (ranging_data.NumberOfObjectsFound > 0) {
      distance_tof = ranging_data.RangeData[0].RangeMilliMeter / 1000.0f; // m
    }
  }
  // Ack and trigger next measurement, still non-blocking
  sensor.VL53L4CX_ClearInterruptAndStartMeasurement();
}

