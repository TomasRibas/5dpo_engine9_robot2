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
  wheel_dist = 0.115;//foi diminuido 1 cm da realidade para fazer "360º" mais accurate
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

  gotoXYState = GotoXYState::STOP;
}

void robot_t::odometry(void)
{
  
  // Estimate wheels speed using the encoders
  w1e = enc1 * (TWO_PI / (64 * 2 * 1920));//(2.0 * 1920.0 * dt); DÚVIDA!!!
  w2e = enc2 * (TWO_PI / (64* 2 * 1920));//(2.0 * 1920.0 * dt); DÚVIDA!!!

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

// void robot_t::gotoXY(float xf, float yf, float tf)
// {
//   float rotateTo, rotateToFinal;
//   float angTarget;
//   float errDist, errAng, errFinalRot;

//   angTarget = atan2f(yf - ye, xf - xe);
//   errAng = normalizeAngle(angTarget - thetae);
//   errDist = dist(xf-xe, yf-ye);
//   errFinalRot = normalizeAngle(tf - thetae);


//   //Find fastest rotation direction
//   if (errAng > 0) rotateTo = ROTATE_RIGHT;
//   else rotateTo = ROTATE_LEFT;

//   if (errFinalRot > 0) rotateToFinal = ROTATE_RIGHT;
//   else rotateToFinal = ROTATE_LEFT;

//   // Transitions
//   switch(gotoXYState) {
//     case GotoXYState::ROTATION:
//       if (fabs(errAng) < MAX_ETF) {
//         gotoXYState = GotoXYState::GO_FORWARD;
//       } else if (errDist < TOL_FINDIST) {
//         gotoXYState = GotoXYState::FINAL_ROT;
//       }
//       break;

//     case GotoXYState::GO_FORWARD:
//       if (errDist < TOL_FINDIST) {
//         gotoXYState = GotoXYState::FINAL_ROT;
//       } else if (errDist < DIST_DA) {
//         gotoXYState = GotoXYState::DEACCEL;
//       } else if (fabs(errAng) > MAX_ETF + HIST_ETF) {
//         gotoXYState = GotoXYState::ROTATION;
//       }
//       break;

//     case GotoXYState::DEACCEL:
//       if (errDist < TOL_FINDIST) {
//         gotoXYState = GotoXYState::FINAL_ROT;
//       } else if (errDist > DIST_NEWPOSE) {
//         gotoXYState = GotoXYState::ROTATION;
//       }
//       break;

//     case GotoXYState::FINAL_ROT:
//       if (fabs(errFinalRot) < THETA_DA) {
//         gotoXYState = GotoXYState::DEACCEL_FINAL_ROT;
//       } else if (errDist > DIST_NEWPOSE) {
//         gotoXYState = GotoXYState::ROTATION;
//       }
//       break;

//     case GotoXYState::DEACCEL_FINAL_ROT:
//       if (fabs(errFinalRot) < TOL_FINTHETA) {
//         gotoXYState = GotoXYState::STOP;
//       } else if ((errDist > DIST_NEWPOSE) || (fabs(errFinalRot) > THETA_NEWPOSE)) {
//         gotoXYState = GotoXYState::ROTATION;
//       }
//       break;

//     case GotoXYState::STOP:
//       if ((errDist > DIST_NEWPOSE) || (fabs(errFinalRot) > THETA_NEWPOSE)) {
//         gotoXYState = GotoXYState::ROTATION;
//       }
//       break;

//     default:
//       gotoXYState = GotoXYState::STOP;
//       break;
//   }

//   // Outputs
//   switch(gotoXYState) {
//     case GotoXYState::ROTATION:
//       v_req = 0;
//       w_req = rotateTo * 0.1f * VEL_ANG_NOM;
//       break;

//     case GotoXYState::GO_FORWARD:
//       v_req = VEL_LIN_NOM;
//       w_req = GAIN_FWD * errAng;
//       break;

//     case GotoXYState::DEACCEL:
//       // Linear deceleration based on distance
//       v_req = VEL_LIN_NOM * (errDist / DIST_DA);
//       w_req = GAIN_DA * errAng;
//       break;

//     case GotoXYState::FINAL_ROT:
//       v_req = 0;
//       w_req = rotateToFinal * 0.1f * VEL_ANG_NOM;
//       break;

//     case GotoXYState::DEACCEL_FINAL_ROT:
//       v_req = 0;
//       w_req = sign(errFinalRot) * W_DA;
//       break;

//     case GotoXYState::STOP:
//       v_req = 0;
//       w_req = 0;
//       break;

//     default:
//       v_req = 0;
//       w_req = 0;
//       break;
//   }
//   setRobotVW(v_req, w_req);
//   calcMotorsVoltage();
// }

// void robot_t::navigateSquare() {
//   static int waypointIndex = 0;
//   static float squareWaypoints[][3] = {
//     {0.0, 0.0, M_PI/2},    // Waypoint 0: (0.5, 0, 90°)
//     {0.0, 1.27, M_PI},      // Waypoint 1: (0.5, 0.5, 180°)
//     {1.78, 1.27, 3*M_PI/2},  // Waypoint 2: (0, 0.5, 270°)
//     {1.78, 1.27, 0.0}        // Waypoint 3: (0, 0, 0°)
//   };
  
//   // Execute current waypoint
//   gotoXY(squareWaypoints[waypointIndex][0], 
//                squareWaypoints[waypointIndex][1], 
//                squareWaypoints[waypointIndex][2]);
  
//   // Check if reached waypoint and move to next
//   if(gotoXYState == GotoXYState::STOP) {
//     waypointIndex = (waypointIndex + 1) % 4; // Loop through waypoints
//     gotoXYState = GotoXYState::ROTATION; // Reset for next waypoint
//     // Optional pause at each corner
//   }
// }



