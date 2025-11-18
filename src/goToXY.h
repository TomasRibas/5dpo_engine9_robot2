#ifndef GOTOXY_H
#define GOTOXY_H

#include <cmath>
#include "robot.h"



// ----- Helper functions -----
double NormalizeAngle(double a);
double Deg(double rad);
int sign(double v);

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

// ----- Enum for state machine -----
enum GoToXYState {
    Rotation = 0,
    Go_Forward,
    De_Accel,
    Final_Rot,
    DeAccel_Final_Rot,
    StopState
};

// ----- External robot pose/state -----
extern double x, y, theta;
extern double vlin, omega;
extern GoToXYState state;

// ----- Square navigation -----
extern int waypointIndex;
extern unsigned long waypointReachedTime;
extern bool waitingAtWaypoint;

// ----- Function declarations -----
void gotoXY(double xf, double yf, double tf);
void MotorVel(float v_req, float w_req);  // You can implement this to send vlin, omega to motors
void navigateSquare(double sideLength, unsigned long pauseMs);
void resetSquareNavigation();

#endif // GOTOXY_H
