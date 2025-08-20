#ifndef TOF_H
#define TOF_H

#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cx_class.h>
#include <pico/cyw43_arch.h>


#define CYW43_WL_GPIO_LED_PIN 0

class TOF {
public:
    TOF();  // initialize I2C and sensor instance

    float distance_tof = 0.0f;
    float prev_distance_tof = 0.0f;

    void initializeToFSensor();
    void calculateTOF();

private:
    VL53L4CX sensor;
    VL53L4CX_MultiRangingData_t ranging_data;
};

#endif