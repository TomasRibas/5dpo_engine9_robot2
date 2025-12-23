#include "followLine.h"

// ----- Constants -----
const double VEL_ANG_NOM = 1;
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

// Modified Dist2Line: also computes the projection parameter 't' along the line segment
// t < 0 means robot is behind the start point
// t > 1 means robot is past the end point
// 0 <= t <= 1 means robot is alongside the segment
void Dist2Line(double xi, double yi, double xf, double yf, double xr, double yr,
               double &kl, double &pix, double &piy, double &t_param)
{
    double dx = xf - xi;
    double dy = yf - yi;
    double lineLen = std::sqrt(dx*dx + dy*dy);
    
    if (lineLen < 1e-6) {
        kl = 0;
        pix = xi;
        piy = yi;
        t_param = 0;
        return;
    }
    
    double ux = dx / lineLen;
    double uy = dy / lineLen;
    
    // kl is the signed perpendicular distance (original Pascal formula)
    kl = (xr*uy - yr*ux - xi*uy + yi*ux);
    
    // Nearest point on the INFINITE line
    pix = -kl*uy + xr;
    piy =  kl*ux + yr;
    
    // t_param: projection parameter along line segment
    t_param = ((pix - xi)*ux + (piy - yi)*uy) / lineLen;
    
    Serial.print(" kl: "); Serial.print(kl);
    Serial.print(" pix: "); Serial.print(pix);
    Serial.print(" piy: "); Serial.print(piy);
    Serial.print(" t: "); Serial.print(t_param);
}

void gotoXY(double xf, double yf, double tf)
{
    const double ang_target      = std::atan2(yf - y, xf - x);
    const double error_ang       = NormalizeAngle(ang_target - theta);
    const double error_dist      = std::sqrt((xf - x)*(xf - x) + (yf - y)*(yf - y));
    const double error_final_rot = NormalizeAngle(tf - theta);

    const int rotateTo      = (error_ang > 0.0) ?  1 : -1;
    const int rotateToFinal = (error_final_rot > 0.0) ?  1 : -1;

    // State transitions
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

    // Outputs - ORIGINAL signs
    switch (state) {
        case Rotation:
            vlin  = 0.0;
            omega = rotateTo * VEL_ANG_NOM;
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
            omega = rotateToFinal * VEL_ANG_NOM;
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
    // tr = angle of the line
    double tr = std::atan2(yf - yi, xf - xi);
    double error_ang  = NormalizeAngle(tr - theta);
    double error_dist = std::sqrt((xf - x)*(xf - x) + (yf - y)*(yf - y));

    // Get distance to line, nearest point, AND projection parameter
    double t_param;
    Dist2Line(xi, yi, xf, yf, x, y, kl, nearX, nearY, t_param);

    testSideLine = sign(kl);
    distLine     = std::abs(kl);
    
    // Check if robot is behind start point (t < 0)
    bool behindStart = (t_param < 0.0);
    
    // If behind the start point, set nearX/nearY to be the start point
    if (behindStart) {
        nearX = xi;
        nearY = yi;
        distLine = std::sqrt((x - xi)*(x - xi) + (y - yi)*(y - yi));
    }

    Serial.print(" distLine: "); Serial.print(distLine);
    Serial.print(" error_dist: "); Serial.print(error_dist);
    Serial.print(" error_ang: "); Serial.print(Deg(error_ang));
    Serial.print(" t_param: "); Serial.print(t_param);
    Serial.print(" State: "); Serial.println(followLineState);

    // State transitions
    switch (followLineState) {
        case Follow_Line:
            if (behindStart || distLine > DIST_NEWLINE) {
                followLineState = Goto_NearXY;
                state = Rotation;
            } else if (error_dist < 10.0 * TOL_FINDIST) {
                followLineState = Approaching;
            }
            break;

        case Approaching:
            if (error_dist < TOL_FINDIST) {
                followLineState = Final_Rot_FL;
                state = Rotation;
            }
            break;

        case Final_Rot_FL:
            if (state == StopState)
                followLineState = Stop_FL;
            break;

        case Stop_FL:
            if (distLine > DIST_NEWLINE) {
                followLineState = Goto_NearXY;
                state = Rotation;
            }
            break;

        case Goto_NearXY:
            if (distLine < DIST_NEARLINE && !behindStart) {
                followLineState = Follow_Line;
            }
            if (behindStart && state == StopState) {
                followLineState = Follow_Line;
                state = Rotation;
            }
            break;
    }

    // Outputs - ORIGINAL signs for omega
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
            vlin  = 0.0;
            omega = 0.0;
            MotorVel((float)vlin, (float)omega);
            break;

        case Goto_NearXY:
            gotoXY(nearX, nearY, tr);
            break;
    }

    error_dist_prev = error_dist;
}
