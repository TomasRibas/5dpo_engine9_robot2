#ifndef ROBOT_H
  #define ROBOT_H
#endif

#include <Arduino.h>
#include <math.h>
#include "PID.h"
#include "utils.h"


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

//GotoXY Declarations
// #define ROTATE_RIGHT 1
// #define ROTATE_LEFT -1

// #define MAX_ETF 0.05f
// #define TOL_FINDIST 0.01f
// #define DIST_DA 0.2f
// #define HIST_ETF (5*M_PI/180) //5 degrees in radians
// #define DIST_NEWPOSE 0.5f
// #define THETA_DA (15*M_PI/180) //15 degrees in radians
// #define TOL_FINTHETA (M_PI/180) //1 degrees in radians
// #define THETA_NEWPOSE (15*M_PI/180) //15 degrees in radians
// #define GAIN_DA 0.005f
// #define GAIN_FWD 0.02f
// #define VEL_ANG_NOM 0.2
// #define VEL_LIN_NOM 5
// #define W_DA 0.01

enum class GotoXYState {
  ROTATION=0,
  GO_FORWARD,
  DEACCEL,
  FINAL_ROT,
  DEACCEL_FINAL_ROT,
  STOP
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
  //GotoXYState gotoXYState;
  GotoXYState gotoXYState;

  robot_t();

  void odometry(void);
  void setRobotVW(float Vnom, float Wnom);

  void accelerationLimit(void);
  void calcMotorsVoltage(void);

  // void gotoXY(float xf, float yf, float tf);
  // void navigateSquare();


};

extern robot_t robot;

#endif 