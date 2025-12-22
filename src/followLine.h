#ifndef FOLLOWLINE_H
#define FOLLOWLINE_H

#include <cmath>
#include "robot.h"


// ----- Helper functions -----
double NormalizeAngle(double a);
double Deg(double rad);
int sign(double v);

// Update the internal pose used by gotoXY()/followLine().
void setPose(double xe, double ye, double thetae);

// ----- Constants -----
extern const double VEL_ANG_NOM;
extern const double VEL_LIN_NOM;
extern const double W_DA;
extern const double LinDeAccel;

extern const double MAX_ETF;
extern const double HIST_ETF;
extern const double GAIN_FWD;
extern const double DIST_DA;
extern const double GAIN_DA;

extern const double TOL_FINDIST;
extern const double DIST_NEWPOSE;
extern const double THETA_NEWPOSE;
extern const double THETA_DA;
extern const double TOL_FINTHETA;

extern const double DIST_NEWLINE;
extern const double DIST_NEARLINE;

// ----- Enums for state machines -----
enum GoToXYState {
    Rotation = 0,
    Go_Forward,
    De_Accel,
    Final_Rot,
    DeAccel_Final_Rot,
    StopState
};

enum FollowLineState {
    Follow_Line = 0,
    Approaching,
    Final_Rot_FL,
    Stop_FL,
    Goto_NearXY
};

// FollowLineState is already defined in robot.h as:
//   enum class FollowLineState { Follow_Line, Approaching, ... }
// So we do NOT redefine it here.

// ----- External robot pose/state (globals, like your example) -----
extern double x, y, theta;     // estimated pose
extern double vlin, omega;     // commanded v,w

extern GoToXYState state;      // gotoXY state machine
// We reuse the global state machine variable declared in robot.h:
extern FollowLineState followLineState;


// ----- FollowLine globals -----
extern double distLine;        // unsigned distance to line (abs)
extern double testSideLine;    // sign of distance to line (+1/-1/0)
extern double nearX, nearY;    // closest point on line
extern double error_dist_prev; // previous distance-to-goal

// ----- Function declarations -----
void MotorVel(float v_req, float w_req); // sends vlin, omega to motors

void Dist2Line(double xi, double yi, double xf, double yf,
               double xr, double yr,
               double &kl, double &pix, double &piy);

void gotoXY(double xf, double yf, double tf);
void followLine(double xi, double yi, double xf, double yf, double tf);

#endif // FOLLOWLINE_H
