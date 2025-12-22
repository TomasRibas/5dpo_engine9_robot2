#include "followLine.h"

// ----- Constants -----
const double VEL_ANG_NOM = 1.57;
const double VEL_LIN_NOM = 0.2;
const double W_DA        = 0.5;
const double LinDeAccel  = 0.1;

const double MAX_ETF      = 10 * M_PI/180.0;
const double HIST_ETF     =  5 * M_PI/180.0;
const double GAIN_FWD     = 0.02;
const double DIST_DA      = 0.05;
const double GAIN_DA      = 0.1;//0.0005;

const double TOL_FINDIST  = 0.02;
const double DIST_NEWPOSE = 0.5;
const double THETA_NEWPOSE = 15 * M_PI/180.0;
const double THETA_DA      = 15 * M_PI/180.0;
const double TOL_FINTHETA  =  1 * M_PI/180.0;

const double DIST_NEWLINE  = 0.1;
const double DIST_NEARLINE = 0.05;

// ----- Global variables (like your goToXY example) -----
double x , y , theta; // Initial estimated pose
double vlin = 0.0, omega = 0.0;

FollowLineState followLineState = Follow_Line;

GoToXYState state = Rotation;


double distLine = 0.0;
double testSideLine = 0.0;
double nearX = 0.0, nearY = 0.0;
double error_dist_prev = 99999.0;
double kl = 0.0;

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

void setPose(double xe, double ye, double thetae) {
    x = xe;
    y = ye;
    theta = NormalizeAngle(thetae);
}

void MotorVel(float v_req, float w_req) {
    robot.setRobotVW(v_req, w_req);
}

// void Dist2Line(double xi, double yi, double xf, double yf, double xr, double yr, double &kl, double &pix, double &piy)
// {
//     double ux = (xf - xi)/sqrt((xf - xi)*(xf - xi) + (yf - yi)*(yf - yi));
//     double uy = (yf - yi)/sqrt((xf - xi)*(xf - xi) + (yf - yi)*(yf - yi));   
//     kl = (xr*uy - yr*ux - xi*uy + yi*ux) / (ux*ux + uy*uy);
//     pix = -kl*uy + xr;
//     piy =  kl*ux + yr;
// }

void Dist2Line(double xi, double yi, double xf, double yf,
               double xr, double yr,
               double &signedDist, double &pix, double &piy)
{
    const double vx = xf - xi;
    const double vy = yf - yi;
    const double wx = xr - xi;
    const double wy = yr - yi;

    const double vv = vx*vx + vy*vy;
    if (vv < 1e-12) { // degenerate segment
        pix = xi; piy = yi;
        signedDist = 0.0;
        return;
    }

    // projection along the segment
    double t = (wx*vx + wy*vy) / vv;

    // clamp to [0,1] so closest point stays on the segment
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;

    pix = xi + t*vx;
    piy = yi + t*vy;

    const double dx = xr - pix;
    const double dy = yr - piy;

    // sign using cross product (segment direction x vector from start to robot)
    const double cross = vx*wy - vy*wx;
    const double side = (cross > 0) ? 1.0 : (cross < 0 ? -1.0 : 0.0);

    signedDist = side * std::sqrt(dx*dx + dy*dy);
}

void gotoXY(double xf, double yf, double tf)
{
    const double ang_target      = std::atan2(yf - y, xf - x);
    const double error_ang       = NormalizeAngle(ang_target - theta);
    const double error_dist      = std::sqrt((xf - x)*(xf - x) + (yf - y)*(yf - y));
    const double error_final_rot = NormalizeAngle(tf - theta);

    const int rotateTo     = (error_ang > 0.0) ?  1 : -1;
    const int rotateToFinal= (error_final_rot > 0.0) ?  1 : -1;

    switch (state) {
        case Rotation:
            if (std::abs(error_ang) < MAX_ETF) state = Go_Forward;
            else if (error_dist < TOL_FINDIST) state = Final_Rot;
            break;

        case Go_Forward:
            if (error_dist < TOL_FINDIST) state = Final_Rot;
            else if (error_dist < DIST_DA) state = De_Accel;
            else if (std::abs(error_ang) > MAX_ETF + HIST_ETF) state = Rotation;
            break;

        case De_Accel:
            if (error_dist < TOL_FINDIST) state = Final_Rot;
            else if (error_dist > DIST_NEWPOSE) state = Rotation;
            break;

        case Final_Rot:
            if (std::abs(error_final_rot) < THETA_DA) state = DeAccel_Final_Rot;
            else if (error_dist > DIST_NEWPOSE) state = Rotation;
            break;

        case DeAccel_Final_Rot:
            if (std::abs(error_final_rot) < TOL_FINTHETA) state = StopState;
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

    switch (state) {
        case Rotation:
            vlin  = 0.0;
            omega = rotateTo * 0.1 * VEL_ANG_NOM;
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
            omega = rotateToFinal * 0.1 * VEL_ANG_NOM;
            break;

        case DeAccel_Final_Rot:
            vlin  = 0.0;
            omega = sign(error_final_rot) * W_DA;
            break;

        case StopState:
            vlin  = 0.0;
            omega = 0.0;
            break;
    }

    MotorVel((float)vlin, (float)omega);
}


void followLine(double xi, double yi, double xf, double yf, double tf)
{
    double tr = std::atan2(yf - yi, xf - xi);
    double error_ang  = NormalizeAngle(tr - theta);
    double error_dist = std::sqrt((xf - x)*(xf - x) + (yf - y)*(yf - y));

    Dist2Line(xi, yi, xf, yf, x, y, kl, nearX, nearY);
    testSideLine = sign(kl);
    distLine     = std::abs(kl);

    switch (followLineState) {
        case Follow_Line:
            if (error_dist < 10.0 * TOL_FINDIST) {
                followLineState = Approaching;
            } else if (distLine > DIST_NEWLINE) {
                followLineState = Goto_NearXY;
            }
            break;

        case Approaching:
            if (error_dist < TOL_FINDIST)
                followLineState = Final_Rot_FL;
            break;

        case Final_Rot_FL:
            if (state == StopState)
                followLineState = Stop_FL;
            break;

        case Stop_FL:
            if (distLine > DIST_NEWLINE)
                followLineState =Goto_NearXY;
            break;

        case Goto_NearXY:
            if (distLine < DIST_NEARLINE)
                followLineState = Follow_Line;
            break;
    }

    switch (followLineState) {
        case Follow_Line:
            vlin  = VEL_LIN_NOM;
            omega = 0.3 * testSideLine * distLine + 0.15 * error_ang * VEL_ANG_NOM;
            MotorVel((float)vlin, (float)omega);
            break;

        case Approaching:
            vlin  = LinDeAccel;
            omega = 0.3 * testSideLine * distLine + 0.15 * error_ang * VEL_ANG_NOM;
            MotorVel((float)vlin, (float)omega);
            break;

        case Final_Rot_FL:
            gotoXY(xf, yf, tf);
            break;

        case Stop_FL:
            vlin = 0.0;
            omega = 0.0;
            MotorVel((float)vlin, (float)omega);
            break;

        case Goto_NearXY:
            gotoXY(nearX, nearY, tr);
            break;
    }

    error_dist_prev = error_dist;
}
