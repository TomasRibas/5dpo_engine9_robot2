#ifndef FOLLOWLINE_H
#define FOLLOWLINE_H

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

// ----- Waypoint Structure -----
struct Waypoint {
    double xi, yi;   // Start point
    double xf, yf;   // End point
    double tf;       // Final theta
};

// ----- Trajectory State -----
struct TrajectoryState {
    bool active;           // Is trajectory running?
    int current_segment;   // Current segment index (0-7 for figure-8)
    int total_segments;    // Total number of segments
    bool loop;             // Loop trajectory when finished?
    Waypoint* waypoints;   // Pointer to waypoint array
};

// ----- Global State Variables (extern) -----
extern FollowLineState followLineState;
extern GoToXYState state;
extern double distLine;
extern double kl;
extern TrajectoryState trajectory;

// ----- Functions -----
void setPose(double xe, double ye, double thetae);
void gotoXY(double xf, double yf, double tf);
void followLine(double xi, double yi, double xf, double yf, double tf);
void resetFollowLine();  // Reset all internal state for a new segment

// Trajectory functions
void startFigure8();              // Start figure-8 trajectory
void startCustomTrajectory(Waypoint* waypoints, int count, bool loop);
void stopTrajectory();            // Stop trajectory execution
void executeTrajectory();         // Call this in main loop if trajectory.active
bool isTrajectoryComplete();      // Check if trajectory finished

double NormalizeAngle(double a);
double Deg(double rad);

#endif // FOLLOWLINE_H