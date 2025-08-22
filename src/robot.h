#ifndef ROBOT_H
  #define ROBOT_H
#endif

#include <Arduino.h>
#include <math.h>
#include "PID.h"


#ifndef NUM_WHEELS
#define NUM_WHEELS 2

typedef enum { 
  cm_voltage,
  cm_pid,
  cm_kinematics
} control_mode_t;

enum RobotMode {
  followMode,
  turningLeft,
  turningRight,
  pickingBox,
  droppingBoxFront,
  droppingBoxBack
};

class robot_t {
  public:
  int enc1, enc2,pos;
  int Senc1, Senc2;
  float w1e, w2e;
  float v1e, v2e;
  float ve, we;
  float ds, dtheta;
  float rel_s, rel_theta;
  float xe, ye, thetae;
  
  float dt;
  float v, w;
  float v_req, w_req;
  float dv_max, dw_max;

  byte state;
  uint32_t tis, tes;
  
  float wheel_radius, wheel_dist;
  
  float v1ref, v2ref;
  float w1ref, w2ref;
  float u1, u2;
  float u1_req, u2_req;
  float i_sense, u_sense;
  float i_lambda;
  int PWM_1, PWM_2;
  //int PWM_1_req, PWM_2_req;
  float w1_req, w2_req;
  control_mode_t control_mode;
  float follow_v, follow_k;

  float debug1, debug2;
  
  PID_t PID[NUM_WHEELS];
  float battery_voltage;
  int button_state;

  int solenoid_PWM;
  int led;

  float tof_dist, prev_tof_dist;
  float thetaeLine ;

  //state_machine_t* pfsm;
  //gchannels_t* pchannels;

  RobotMode mode;

  robot_t();

  void odometry(void);
  void setRobotVW(float Vnom, float Wnom);

  void accelerationLimit(void);
  void calcMotorsVoltage(void);


};

extern robot_t robot;

#endif 