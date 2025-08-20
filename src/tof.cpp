#include "tof.h"

#define CYW43_WL_GPIO_LED_PIN 0
VL53L0X tof;
int LED_state;
int loop_count;

void TOF::initializeToFSensor(){
  tof.setTimeout(500);
  while (!tof.init()) {
    Serial.println(F("Failed to detect and initialize VL53L0X!"));
    delay(100);
  }
  // Reduce timing budget to 20 ms (default is about 33 ms)
  //tof.setMeasurementTimingBudget(20000);
  
  tof.startReadRangeMillimeters();
}

void TOF::calculateTOF(){
  if(tof.readRangeAvailable()){
    prev_distance_tof = distance_tof;
    distance_tof = tof.readRangeMillimeters() * 1e-3;
  }
  tof.startReadRangeMillimeters(); // Start new distance_tof measure
  // Toggle builtin LED    
  loop_count++;
  if (loop_count > 5) {
    LED_state = !LED_state;
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, LED_state);
    loop_count = 0;
  }
}