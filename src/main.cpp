#include <Arduino.h>
#include <RPi_Pico_TimerInterrupt.h>
#include <Wire.h>
#include "TOF.h"
#include "robot.h"
#include "trajectories.h"

#include "lds_driver.hpp"


#include "gchannels.h"

TOF stof;

// Create after optional Serial1 pin mapping, so use a pointer:
lds::LFCDLaser* lidar = nullptr;

//void init_control(robot_t& robot);
//void control(robot_t& robot);

PID_pars_t wheel_PID_pars;


/////////////////////////GCHANNELS///////////////////////////

gchannels_t serial_commands;
gchannels_t comRobot_commands;
commands_list_t pars_list;

int analogWriteBits = 10; 
int analogWriteMax = (1 << analogWriteBits) - 1; 

SerialPIO SerialTiny(SerialPIO::NOPIN, 21);


/////////////////////////////MOTORS/////////////////////////////
const int motor_left_in1 = 10;
const int motor_left_in2 = 11;
const int motor_right_in1 = 12;
const int motor_right_in2 = 13;


//////////////////////////////ENCODERS///////////////////////////////
#include "PicoEncoder.h"

#define ENCL_A 2 //LEFT
#define ENCL_B 3 

#define ENCR_A 6 //RIGHT 
#define ENCR_B 7 


#define NUM_ENCODERS 2
PicoEncoder encoders[NUM_ENCODERS];
pin_size_t encoder_pins[NUM_ENCODERS] = {ENCL_A, ENCR_A};

bool calibration_requested;

void read_PIO_encoders(void)
{
  encoders[0].update();
  encoders[1].update();  
  robot.enc1 = encoders[0].speed;
  robot.enc2 = encoders[1].speed;

}

void initializeEncoders()
{
  pinMode(ENCL_A, INPUT_PULLUP);
  pinMode(ENCL_B, INPUT_PULLUP);
  pinMode(ENCR_A, INPUT_PULLUP);
  pinMode(ENCR_B, INPUT_PULLUP);

}

void initializeMotors()
{
  pinMode(motor_left_in1, OUTPUT);
  pinMode(motor_left_in2, OUTPUT);
  pinMode(motor_right_in1, OUTPUT);
  pinMode(motor_right_in2, OUTPUT);

}

float voltageToPWM(float voltage, float maxVoltage, float maxPWM)
{
  if (maxVoltage < 1e-3f) return 0;   // avoid NaN/Inf at boot

  // Map voltage to PWM
  int pwm = (voltage / maxVoltage) * maxPWM;
  
  // Constrain the PWM value to the valid range
  pwm = constrain(pwm, -maxPWM, maxPWM);
  return pwm;
}

void setMotorsPWM(float u1, float u2)
{

  float maxVoltage = robot.battery_voltage; // Maximum voltage
  float maxPWM = 230;      // Maximum PWM value for 8-bit resolution
  
  int PWM1 = voltageToPWM(u1, maxVoltage, maxPWM);
  int PWM2 = voltageToPWM(u2, maxVoltage, maxPWM);

  if (PWM1 > 0){
      analogWrite(motor_left_in1, PWM1);
      digitalWrite(motor_left_in2, LOW);
  }else{
    PWM1 = -PWM1;
    analogWrite(motor_left_in2, PWM1);
    digitalWrite(motor_left_in1, LOW);
  }

  if (PWM2 > 0){
    analogWrite(motor_right_in1, PWM2);
    digitalWrite(motor_right_in2, LOW);
  }else{
    PWM2 = -PWM2;
    analogWrite(motor_right_in2, PWM2);
    digitalWrite(motor_right_in1, LOW);
  }

}

unsigned long interval;
unsigned long currentMicros, previousMicros;
char received;

void set_interval(float new_interval)
{
  interval = new_interval * 1000000L;   // In microseconds
  robot.dt = new_interval;   // In seconds
  wheel_PID_pars.dt = robot.dt;  
}

void printEncoders()
{
  Serial.print("Encoder Left:");
  Serial.print(robot.enc1);
  Serial.print(" Encoder Right:");
  Serial.print(robot.enc2);
}

void printOdometry()
{
  Serial.print(" X:");
  Serial.print(robot.xe);
  Serial.print(" Y:");
  Serial.print(robot.ye);
  Serial.print(" Theta:");
  Serial.print((robot.thetae*180)/PI);
}

void printTof()
{  
  Serial.print(" TOF: ");
  Serial.print(stof.distance_tof);
}


void setup() {

  interval = 40 * 1000;

  analogReadResolution(10);

  Serial.begin(115200);
  initializeMotors();

  lidar = new lds::LFCDLaser(230400);  // ctor signature per new header :contentReference[oaicite:3]{index=3}

  // Setup motor pin
  //analogWriteFreq(10000); // 10 kHz PWM
  //analogWriteRange(255);
   // 50% duty cycle


  Wire.setSDA(4);
  Wire.setSCL(5);

  Wire.begin();


  initializeEncoders();
  encoders[0].begin(encoder_pins[0]);
  encoders[1].begin(encoder_pins[1]);
  
  stof.initializeToFSensor();

  SerialTiny.begin(); //leitura da tensão da bateria

  robot.control_mode = cm_pid;
}

uint8_t b;

void loop() {
  
  if (SerialTiny.available()) { 
    b = SerialTiny.read();  
    robot.battery_voltage = 1e-3 * ((b >> 1) * 50 + 4800);
  }

  /*if (Serial1.available()) {
    int b = Serial1.read();
    if (b >= 0) {
      Serial.print("0x"); Serial.print(b, HEX);
    }
  }*/
  lidar->poll();  // prints inside driver (no distance array exposed) :contentReference[oaicite:4]{index=4}


  currentMicros = micros();
  if(currentMicros - previousMicros >= interval){
    previousMicros = currentMicros;

    read_PIO_encoders();
    //stof.calculateTOF();

    robot.odometry();

  /*if (robot.xe < 0.29){
      robot.v = 0.05;
      robot.w = 0;
    }else{
      robot.v = 0;
      robot.w = 0;     
    }
   if (robot.thetae<6.28){
      robot.w1_req=2;
      robot.w2_req=0;
    } else {
      robot.w1_req=0;
      robot.w2_req=0;
    }*/
   
    //Serial1.write("e");

    //printEncoders();
    //printOdometry();
    //printTof();
    //Serial.printf(" rel_s: %f", robot.rel_s);
    //Serial.printf(" U1: %f", robot.u1);
    //Serial.printf(" U2: %f", robot.u2);

    robot.calcMotorsVoltage();
    setMotorsPWM(robot.u1, robot.u2);
    //lidar->poll();  // prints inside driver (no distance array exposed) :contentReference[oaicite:4]{index=4}

    //Serial.println();
  }
}

