#include "followLine.h"

// ----- Configurable Constants (can be set via gchannels) -----
// Using float instead of double for gchannels compatibility
float VEL_ANG_NOM = 1.0f;
float VEL_LIN_NOM = 0.2f;
float W_DA        = 0.5f;
float LinDeAccel  = 0.1f;

float MAX_ETF      = 10.0f * M_PI/180.0f;//0.1745
float HIST_ETF     =  5.0f * M_PI/180.0f;//0.0873
float GAIN_FWD     = 0.1f;
float DIST_DA      = 0.05f;
float GAIN_DA      = 0.1f;

float TOL_FINDIST  = 0.01f;
float DIST_NEWPOSE = 0.5f;
float THETA_NEWPOSE = 15.0f * M_PI/180.0f;//0.2618
float THETA_DA      = 5.0f * M_PI/180.0f;//0.0873
float TOL_FINTHETA  = 1.0f * M_PI/180.0f;//0.01745

float DIST_NEWLINE  = 0.1f;
float DIST_NEARLINE = 0.05f;

// Line following omega gains (NEW)
// omega = K_DIST * testSideLine * distLine + K_ANG * error_ang * VEL_ANG_NOM
float K_DIST = 4.0f;      // Gain for distance-to-line correction - cmd: "kdst"
float K_ANG  = 0.15f;     // Gain for angle error correction - cmd: "kang"

// ----- Global variables -----
double x, y, theta;
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

void resetFollowLine() {
    followLineState = Follow_Line;
    state = Rotation;
    distLine = 0.0;
    testSideLine = 0.0;
    nearX = 0.0;
    nearY = 0.0;
    error_dist_prev = 99999.0;
    kl = 0.0;
    vlin = 0.0;
    omega = 0.0;
}

void MotorVel(float v_req, float w_req) {
   robot.setRobotVW(v_req, w_req);
}

// Modified Dist2Line: also computes the projection parameter 't' along the line segment
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
    
    kl = (xr*uy - yr*ux - xi*uy + yi*ux);
    
    pix = -kl*uy + xr;
    piy =  kl*ux + yr;
    
    t_param = ((pix - xi)*ux + (piy - yi)*uy) / lineLen;
}

void gotoXY(double xf, double yf, double tf)
{
    const double ang_target      = std::atan2(yf - y, xf - x);
    const double error_ang       = NormalizeAngle(ang_target - theta);
    const double error_dist      = std::sqrt((xf - x)*(xf - x) + (yf - y)*(yf - y));
    const double error_final_rot = NormalizeAngle(tf - theta);

    const int rotateTo      = (error_ang > 0.0) ?  1 : -1;
    const int rotateToFinal = (error_final_rot > 0.0) ?  1 : -1;

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

    // Outputs
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
    double tr = std::atan2(yf - yi, xf - xi);
    double error_ang  = NormalizeAngle(tr - theta);
    double error_dist = std::sqrt((xf - x)*(xf - x) + (yf - y)*(yf - y));

    double t_param;
    Dist2Line(xi, yi, xf, yf, x, y, kl, nearX, nearY, t_param);

    testSideLine = sign(kl);
    distLine     = std::abs(kl);
    
    bool behindStart = (t_param < 0.0);
    bool pastEnd     = (t_param > 1.0);
    
    if (behindStart) {
        nearX = xi;
        nearY = yi;
        distLine = std::sqrt((x - xi)*(x - xi) + (y - yi)*(y - yi));
    }

    // State transitions
    switch (followLineState) {
        case Follow_Line:
            // if (behindStart || distLine > DIST_NEWLINE) {
            //     followLineState = Goto_NearXY;
            //     state = Rotation;
            // } else if (pastEnd || error_dist < 10.0 * TOL_FINDIST) {
            //     followLineState = Approaching;
            // }
            if ( error_dist < DIST_DA) 
                 followLineState = Approaching;
            break;

        case Approaching:
            if (error_dist < TOL_FINDIST) {
                followLineState = Final_Rot_FL;
                state = Rotation;
            }
            // Past end of segment - hand off to gotoXY to reach final point
            else if (pastEnd && error_dist < DIST_DA) {
                followLineState = Final_Rot_FL;
                state = De_Accel;  // Start in De_Accel since we're close
            }
            // Past end and far from target - still hand off but start from Rotation
            else if (pastEnd && error_dist >= DIST_DA) {
                followLineState = Final_Rot_FL;
                state = Rotation;
            }
            // ESCAPE 1: Saiu muito da linha
            else if (distLine > DIST_NEWLINE) {
                followLineState = Goto_NearXY;
                state = Rotation;
            }
            // ESCAPE 2: Destino ficou muito longe (nova instrução)
            else if (error_dist > 0.5f) {
                followLineState = Follow_Line;
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
            // Return to Follow_Line when close enough to the line
            if (distLine < DIST_NEARLINE && !behindStart && !pastEnd) {
                followLineState = Follow_Line;
            }
            // gotoXY finished (reached near point) - return to Follow_Line
            // even if not perfectly on the line
            else if (state == StopState) {
                followLineState = Follow_Line;
                state = Rotation;
            }
            break;
    }

    // Outputs - using configurable K_DIST and K_ANG gains
    switch (followLineState) {
        case Follow_Line:
            vlin  = VEL_LIN_NOM;
            omega = K_DIST * testSideLine * distLine + K_ANG * error_ang * VEL_ANG_NOM;
            MotorVel((float)vlin, (float)omega);
            break;

        case Approaching:
            vlin  = LinDeAccel;
            omega = K_DIST * testSideLine * distLine + K_ANG * error_ang * VEL_ANG_NOM;
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