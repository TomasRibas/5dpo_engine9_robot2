#include <Arduino.h>
#include "robot.h"
#include <math.h>
#include <trajectories.h>


robot_t robot;


template <typename T> int sign(T val) 
{
  return (T(0) < val) - (val < T(0));
}

robot_t::robot_t()
{
  pos=0;
  wheel_dist = 0.125;
  wheel_radius = 0.0689 / 2;
  dv_max = 5;
  dw_max = 10;
  dt = 0.04;

  follow_k = -0.08;
  follow_v = 0.10;

  /*control_event = false;
  led = 0;

  pchannels = NULL;
  pfsm = NULL;*/
}

void robot_t::odometry(void)
{
  
  // Estimate wheels speed using the encoders
  w1e = enc1 * TWO_PI / (2.0 * 1920.0 * dt);
  w2e = enc2 * TWO_PI / (2.0 * 1920.0 * dt);

  v1e = w1e * wheel_radius;
  v2e = w2e * wheel_radius;

  // Estimate robot speed
  ve = (v1e + v2e) / 2.0;
  we = (v1e - v2e) / wheel_dist;
  
  // Estimate the distance and the turn angle
  ds = ve * dt;
  dtheta = we * dt;

  // Estimate pose
  xe += ds * cos(thetae + dtheta/2);
  ye += ds * sin(thetae + dtheta/2);
  thetae = thetae + dtheta;

  // Relative displacement
  rel_s += ds;
  rel_theta += dtheta;
  
}


void robot_t::setRobotVW(float Vnom, float Wnom)
{
  v_req = Vnom;
  w_req = Wnom;
}


void robot_t::accelerationLimit(void)
{
  float dv = v_req - v;
  dv = constrain(dv, -dv_max, dv_max);
  v += dv;

  float dw = w_req - w;
  dw = constrain(dw, -dw_max, dw_max);
  w += dw;
}


void robot_t::calcMotorsVoltage(void)
{
  if (control_mode == cm_voltage) {
    u1 = u1_req;  
    u2 = u2_req;  
    return;

  } else if (control_mode == cm_pid) {
    w1ref = w1_req;
    w2ref = w2_req;

  } else if (control_mode == cm_kinematics) {
    v1ref = v + w * wheel_dist / 2;
    v2ref = v - w * wheel_dist / 2; 
    
    w1ref = v1ref / wheel_radius;
    w2ref = v2ref / wheel_radius;    
  }

  
  if (w1ref != 0) u1 = PID[0].calc(w1ref, w1e) + sign(w1ref) *  PID[0].ppars->dead_zone;
  else {
    u1 = 0;
    PID[0].Se = 0;
    PID[0].y_ref = 0;
  }

  if (w2ref != 0) u2 = PID[1].calc(w2ref, w2e) + sign(w2ref) *  PID[1].ppars->dead_zone;
  else {
    u2 = 0;      
    PID[1].Se = 0;
    PID[1].y_ref = 0;
  }

  //PWM_2 = u2 / battery_voltage * 255;
  //PWM_1 = u1 / battery_voltage * 255;

}



