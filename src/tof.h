#ifndef TOF_H
#define TOF_H

#include <Arduino.h>
#include <VL53L0X.h>
#include "pico/cyw43_arch.h"

#define CYW43_WL_GPIO_LED_PIN 0

class TOF{

public:
    float distance_tof, prev_distance_tof;
    void initializeToFSensor();
    void calculateTOF();
    
};

#endif