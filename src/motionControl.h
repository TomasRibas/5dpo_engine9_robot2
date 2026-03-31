#ifndef MOTIONCONTROL_H
#define MOTIONCONTROL_H

#include <Arduino.h>
#include <cmath>
#include "robot.h"

// ----- Configurable Constants (extern - defined in followLine.cpp) -----
// These can be modified via gchannels commands
// Using float for gchannels compatibility

// Velocity constants
extern float VEL_ANG_NOM;    // Nominal angular velocity (rad/s) - cmd: "van"
extern float VEL_LIN_NOM;    // Nominal linear velocity (m/s) - cmd: "vln"
extern float W_DA;           // Deceleration angular velocity - cmd: "wda"
extern float LinDeAccel;     // Deceleration linear velocity - cmd: "lda"

// Angle thresholds (radians)
extern float MAX_ETF;        // Max error to finish rotation - cmd: "metf"
extern float HIST_ETF;       // Hysteresis for angle error - cmd: "hetf"
extern float GAIN_FWD;       // Forward gain for angle correction - cmd: "gfwd"
extern float DIST_DA;        // Distance to start deceleration - cmd: "dda"
extern float GAIN_DA;        // Deceleration gain - cmd: "gda"

// Distance thresholds (meters)
extern float TOL_FINDIST;    // Final distance tolerance - cmd: "tfd"
extern float DIST_NEWPOSE;   // Distance to trigger new pose - cmd: "dnp"
extern float THETA_NEWPOSE;  // Theta to trigger new pose - cmd: "tnp"
extern float THETA_DA;       // Theta deceleration threshold - cmd: "tda"
extern float TOL_FINTHETA;   // Final theta tolerance - cmd: "tft"

// Line following thresholds
extern float DIST_NEWLINE;   // Distance to trigger goto nearXY - cmd: "dnl"
extern float DIST_NEARLINE;  // Distance to return to follow line - cmd: "dnel"

// Line following omega gains (NEW)
extern float K_DIST;         // Gain for distance-to-line correction - cmd: "kdst"
extern float K_ANG;          // Gain for angle error correction - cmd: "kang"
extern float KV_RAMP;        // Velocity ramp rate m/s² (0=disabled) - cmd: "kvramp"

// ----- State Enums -----
enum FollowLineState {
    Follow_Line = 0,
    Approaching = 1,
    Final_Rot_FL = 2,
    Stop_FL = 3,
    Goto_NearXY = 4
};

enum GoToXYState {
    Rotation = 0,
    Go_Forward = 1,
    De_Accel = 2,
    Final_Rot = 3,
    DeAccel_Final_Rot = 4,
    StopState = 5
};


// ----- Global State Variables (extern) -----
extern FollowLineState followLineState;
extern GoToXYState state;
extern double distLine;
extern double kl;

// ----- Functions -----
void setPose(double xe, double ye, double thetae);
void gotoXY(double xf, double yf, double tf);
void followLine(double xi, double yi, double xf, double yf, double tf);
void resetFollowLine();  // Reset all internal state for a new segment


double NormalizeAngle(double a);
double Deg(double rad);

// ============================================================================
// FollowCircle — Independent Tuning Constants
// All separate from followLine, tunable via ComRobot
// ============================================================================
 
extern float FC_VEL_LIN_NOM;   // Nominal linear velocity (m/s)        - cmd: "fcvln"
extern float FC_VEL_ANG_NOM;   // Nominal angular velocity (rad/s)      - cmd: "fcvan"
extern float FC_LinDeAccel;    // Deceleration linear velocity (m/s)    - cmd: "fclda"
extern float FC_W_DA;          // Deceleration angular velocity (rad/s) - cmd: "fcwda"
 
extern float FC_TOL_FINDIST;   // Final distance tolerance (m)          - cmd: "fctfd"
extern float FC_DIST_DA;       // Arc length to start deceleration (m)  - cmd: "fcdda"
extern float FC_DIST_NEWPOSE;  // Distance to re-trigger from stop (m)  - cmd: "fcdnp"
extern float FC_THETA_DA;      // Theta deceleration threshold (rad)    - cmd: "fctda"
extern float FC_THETA_NEWPOSE; // Theta to re-trigger from stop (rad)   - cmd: "fctnp"
extern float FC_TOL_FINTHETA;  // Final theta tolerance (rad)           - cmd: "fctft"
 
extern float FC_K_ANG;         // Heading correction gain               - cmd: "fckang"
extern float FC_K_RAD;         // Radial correction gain                - cmd: "fckrad"
 
extern float FC_KV_RAMP;       // Velocity ramp rate m/s² (0=disabled)  - cmd: "fckvramp"
 
// ============================================================================
// State Enum
// ============================================================================
enum FollowCircleState {
    FC_Follow_Arc  = 0,
    FC_Approaching = 1,
    FC_Final_Rot   = 2,
    FC_Stop        = 3
};
 
// ============================================================================
// Global State (extern)
// ============================================================================
extern FollowCircleState followCircleState;
 
// ============================================================================
// Functions
// ============================================================================
void Dist2Arc(double xc, double yc, double R,
              double xr, double yr,
              double &pix, double &piy, double &dist);
 
void resetFollowCircle();
 
void followCircle(double xc, double yc, double R,
                  double angf, double tf, int dir);

#endif // MOTIONCONTROL_H