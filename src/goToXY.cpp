#include "goToXY.h"

// ----- Constants -----
const double VEL_ANG_NOM = 1;//0.4;
const double VEL_LIN_NOM = 0.2;//0.2;
const double W_DA        = 0.4;//0.5;
const double LinDeAccel  = 0.075;//0.1;

const double MAX_ETF     = 10 * M_PI/180.0;
const double HIST_ETF    =  5 * M_PI/180.0;
const double GAIN_FWD    = 0.02;
const double DIST_DA     = 0.2;
const double GAIN_DA     = 0.0005;

const double TOL_FINDIST = 0.02;
const double DIST_NEWPOSE= 0.5;
const double THETA_NEWPOSE = 15 * M_PI/180.0;
const double THETA_DA      = 15 * M_PI/180.0;
const double TOL_FINTHETA  =  1 * M_PI/180.0;

// ----- Global variables -----
double x = 0.2, y = 0, theta = 1.57; //Initial State
double vlin = 0.0, omega = 0.0;

GoToXYState state = Rotation;

// Square navigation variables
int waypointIndex = 0;
unsigned long waypointReachedTime = 0;
bool waitingAtWaypoint = false;

// ----- Helper implementations -----
double NormalizeAngle(double a) {
    while (a <= -M_PI) a += 2.0 * M_PI;
    while (a >   M_PI) a -= 2.0 * M_PI;
    return a;
}

double Deg(double rad) {
    return rad * 180.0 / M_PI;
}

int sign(double v) {
    return (v > 0) - (v < 0);
}

// ----- Stub (replace with your motor control code) -----
void MotorVel(float v_req, float w_req) {
    robot.setRobotVW(v_req, w_req);

}

// ----- Main function -----
void gotoXY(double xf, double yf, double tf)
{
    x = robot.xe;
    y = robot.ye;
    theta = robot.thetae;

    double ang_target      = std::atan2(yf - y, xf - x);
    double error_ang       = NormalizeAngle(ang_target - theta);
    double error_dist      = std::sqrt((xf - x)*(xf - x) + (yf - y)*(yf - y));
    double error_final_rot = NormalizeAngle(tf - theta);
    int    sign_dir        = sign(error_ang);
    int    sign_dir_f      = sign(error_final_rot);

    // ---- State transitions ----
    switch (state) {
        case Rotation:
            if (std::abs(error_ang) < MAX_ETF)
                state = Go_Forward;
            else if (error_dist < TOL_FINDIST)
                state = Final_Rot;
            break;

        case Go_Forward:
            if (error_dist < TOL_FINDIST)
                state = Final_Rot;
            else if (error_dist < DIST_DA)
                state = De_Accel;
            else if (std::abs(error_ang) > MAX_ETF + HIST_ETF)
                state = Rotation;
            break;

        case De_Accel:
            if (error_dist < TOL_FINDIST)
                state = Final_Rot;
            else if (error_dist > DIST_NEWPOSE)
                state = Rotation;
            break;

        case Final_Rot:
            if (std::abs(error_final_rot) < THETA_DA)
                state = DeAccel_Final_Rot;
            else if (error_dist > DIST_NEWPOSE)
                state = Rotation;
            break;

        case DeAccel_Final_Rot:
            if (std::abs(error_final_rot) < TOL_FINTHETA)
                state = StopState;
            else if ((error_dist > DIST_NEWPOSE) ||
                     (std::abs(error_final_rot) > THETA_NEWPOSE))
                state = Rotation;
            break;

        case StopState:
            if ((error_dist > DIST_NEWPOSE) ||
                (std::abs(error_final_rot) > THETA_NEWPOSE))
                state = Rotation;
            break;
    }

    // ---- Output commands ----
    switch (state) {
        case Rotation:
            vlin  = 0.0;
            omega = sign_dir * VEL_ANG_NOM;
            break;

        case Go_Forward:
            vlin  = VEL_LIN_NOM;
            omega = GAIN_FWD * error_ang;
            break;

        case De_Accel:
            vlin  = LinDeAccel;
            omega = GAIN_DA * error_ang;
            break;

        case Final_Rot:
            vlin  = 0.0;
            omega = sign_dir_f * VEL_ANG_NOM;
            break;

        case DeAccel_Final_Rot:
            vlin  = 0.0;
            omega = sign_dir_f * W_DA;
            break;

        case StopState:
            vlin  = 0.0;
            omega = 0.0;
            break;
    }

    MotorVel(vlin, omega);
}

