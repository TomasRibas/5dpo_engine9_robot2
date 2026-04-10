#include "motionControl.h"

// ----- Configurable Constants (can be set via gchannels) -----
// Using float instead of double for gchannels compatibility
float VEL_ANG_NOM = 1.0f;
float VEL_LIN_NOM = 0.3f;
float W_DA        = 0.5f;
float LinDeAccel  = 0.1f;

float MAX_ETF      = 10.0f * M_PI/180.0f;//0.1745
float HIST_ETF     =  5.0f * M_PI/180.0f;//0.0873
float GAIN_FWD     = 0.1f;
float DIST_DA      = 0.1f;
float GAIN_DA      = 0.1f;

float TOL_FINDIST  = 0.02f;
float DIST_NEWPOSE = 0.3f;
float THETA_NEWPOSE = 15.0f * M_PI/180.0f;//0.2618
float THETA_DA      = 5.0f * M_PI/180.0f;//0.0873
float TOL_FINTHETA  = 1.0f * M_PI/180.0f;//0.01745

float DIST_NEWLINE  = 0.3f;  // Increased for smoother merging
float DIST_NEARLINE = 0.05f;

// Line following omega gains
float K_DIST = 9.0f;
float K_ANG  = 4.0f;

// float K_DIST_I = 0.0f;  // start at 0, tune via command
// double distLine_integral = 0.0;
float K_DIST_D = 0.0f;
double kl_prev = 0.0;


// Velocity ramp rate (m/s²) — 0 = disabled (instant step)
float KV_RAMP = 0.4f;   // cmd: "kvramp"

// ----- Global variables -----
double x, y, theta;
double vlin = 0.0, omega = 0.0;

FollowLineState followLineState = Follow_Line;
GoToXYState state = Rotation;

double distLine = 0.0;
double testSideLine = 0.0;
double vlin_ramp = 0.0;   // current ramped velocity
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
    vlin_ramp = 0.0;
    kl_prev = 0.0;
}

void MotorVel(float v_req, float w_req) {
    const double dt = 0.04;  // 40ms control period
    if (KV_RAMP > 0.0f && v_req != 0.0f) {
        double step = (double)KV_RAMP * dt;
        if (vlin_ramp < (double)v_req) {
            vlin_ramp += step;
            if (vlin_ramp > (double)v_req)
                vlin_ramp = (double)v_req;
        } else if (vlin_ramp > (double)v_req) {
            vlin_ramp -= step;
            if (vlin_ramp < (double)v_req)
                vlin_ramp = (double)v_req;
        }
    } else {
        // v_req == 0 (stop): always instant
        // KV_RAMP == 0: instant in all cases
        vlin_ramp = 0.0;
    }
    robot.setRobotVW((float)vlin_ramp, w_req);
}

void Dist2Line(double xi, double yi, double xf, double yf, double xr, double yr,
               double &kl, double &pix, double &piy)
{
    double dx = xf - xi;
    double dy = yf - yi;
    double lineLen = std::sqrt(dx*dx + dy*dy);
    
    if (lineLen < 1e-6) {
        kl = 0;
        pix = xi;
        piy = yi;
        return;
    }
    
    double ux = dx / lineLen;
    double uy = dy / lineLen;
    
    kl = (xr*uy - yr*ux - xi*uy + yi*ux);
    
    pix = -kl*uy + xr;
    piy =  kl*ux + yr;
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

        // case Final_Rot:
        //     vlin  = 0.0;
        //     omega = rotateToFinal * VEL_ANG_NOM;
        //     break;

        // case DeAccel_Final_Rot:
        //     vlin  = 0.0;
        //     omega = sign(error_final_rot) * W_DA;
        //     break;
        case Final_Rot:
            vlin  = 0.0;
            omega = error_final_rot * VEL_ANG_NOM / MAX_ETF;
            omega = constrain(omega, -(double)VEL_ANG_NOM, (double)VEL_ANG_NOM);
            break;

        case DeAccel_Final_Rot:
            vlin  = 0.0;
            omega = error_final_rot * VEL_ANG_NOM / MAX_ETF;
            omega = constrain(omega, -(double)W_DA, (double)W_DA);
            break;
            
        case StopState:
            vlin  = 0.0;
            omega = 0.0;
            break;
    }

    MotorVel((float)vlin, (float)omega);
}


void followLine(double xi, double yi, double xf, double yf, double tf, int dir)
{
    // Line direction angle — flipped by PI for reverse
    double tr = std::atan2(yf - yi, xf - xi);
    if (dir < 0) tr = NormalizeAngle(tr + M_PI);  // reverse: face backwards

    double error_ang  = NormalizeAngle(tr - theta);
    double error_dist = std::sqrt((xf - x)*(xf - x) + (yf - y)*(yf - y));

    Dist2Line(xi, yi, xf, yf, x, y, kl, nearX, nearY);

    testSideLine = sign(kl);
    distLine     = std::abs(kl);

    // State transitions - MATCHING PASCAL LOGIC
    switch (followLineState) {
        case Follow_Line:
            if ((error_dist < 10.0 * TOL_FINDIST) && (std::abs(error_ang) < MAX_ETF)) {
                followLineState = Approaching;
            } else if (distLine > DIST_NEWLINE) {
                followLineState = Goto_NearXY;
                state = Rotation;
            }
            break;

        case Approaching:
            if (error_dist < TOL_FINDIST) {
                followLineState = Final_Rot_FL;
                state = Rotation;  // Reset gotoXY state cleanly — prevents omega spike
                                   // from stale Go_Forward/De_Accel being active on first gotoXY call
                vlin  = 0.0;       // Kill linear velocity immediately
                omega = 0.0;
                MotorVel(0.0f, 0.0f);
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
            if (distLine < DIST_NEARLINE) {
                followLineState = Follow_Line;
            }
            break;
    }

    // Outputs - with pure rotation for large angles
    switch (followLineState) {
        case Follow_Line: {
            static bool in_pure_rotation = false;
            
            // Hysteresis for pure rotation
            if (std::abs(error_ang) > MAX_ETF + HIST_ETF) {
                in_pure_rotation = true;
            } else if (std::abs(error_ang) < MAX_ETF) {
                in_pure_rotation = false;
            }
            
            if (in_pure_rotation) {
                // Pure rotation - no forward motion
                vlin = 0.0;
                omega = sign(error_ang) * VEL_ANG_NOM;
            } else {
                // Normal line following — dir flips vlin sign
                vlin  = dir * VEL_LIN_NOM;
                // omega = K_DIST * testSideLine * distLine + K_ANG * error_ang * VEL_ANG_NOM;
                // distLine_integral += testSideLine * distLine * 0.04;  // integrate with dt=40ms
                // distLine_integral = constrain(distLine_integral, -0.5, 0.5);  // anti-windup
                // omega = K_DIST * testSideLine * distLine + K_DIST_I * distLine_integral + K_ANG * error_ang * VEL_ANG_NOM;
                double kl_dot = (kl - kl_prev) / 0.04;
                kl_prev = kl;

                omega = K_DIST * kl
                    + K_DIST_D * kl_dot
                    + K_ANG * error_ang * VEL_ANG_NOM;
            }
            MotorVel((float)vlin, (float)omega);
            break;
        }

        case Approaching:
            vlin = dir * VEL_LIN_NOM * (error_dist / (10.0 * TOL_FINDIST));
            if (dir > 0)
                vlin = constrain(vlin, (double)LinDeAccel, (double)VEL_LIN_NOM);
            else
                vlin = constrain(vlin, -(double)VEL_LIN_NOM, -(double)LinDeAccel);
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

// ============================================================================
// followCircle - new globals (add near the other globals at the top)
// ============================================================================

/// ============================================================================
// Independent Tuning Constants — all separate from followLine
// ============================================================================
float FC_VEL_LIN_NOM  = 0.1f;                     // cmd: "fcvln"
float FC_VEL_ANG_NOM  = 3.0f;                      // cmd: "fcvan"
float FC_LinDeAccel   = 0.1f;                     // cmd: "fclda"
float FC_W_DA         = 0.3f;                      // cmd: "fcwda"
 
float FC_TOL_FINDIST  = 0.02f;                     // cmd: "fctfd"
float FC_DIST_DA      = 0.05f;                     // cmd: "fcdda"
float FC_DIST_NEWPOSE = 0.5f;                      // cmd: "fcdnp"
float FC_THETA_DA     = 5.0f  * M_PI / 180.0f;    // cmd: "fctda"
float FC_THETA_NEWPOSE= 15.0f * M_PI / 180.0f;    // cmd: "fctnp"
float FC_TOL_FINTHETA = 1.0f  * M_PI / 180.0f;    // cmd: "fctft"
 
float FC_K_ANG        = 2.0f;                    // cmd: "fckang"
float FC_K_RAD        = -60.0f;                     // cmd: "fckrad"
 
float FC_KV_RAMP      = 0.4f;                     // cmd: "fckvramp"  (disabled by default for arcs)
 
// ============================================================================
// Global State
// ============================================================================
FollowCircleState followCircleState = FC_Follow_Arc;
 
static double fc_nearX     = 0.0;
static double fc_nearY     = 0.0;
static double fc_distToArc = 0.0;   // signed: >0 outside circle, <0 inside
static double fc_vlin_ramp = 0.0;   // independent ramp — does not touch followLine's vlin_ramp
 
// ============================================================================
// FC_MotorVel — independent ramp using FC constants, does not touch followLine
// ============================================================================
static void FC_MotorVel(float v_req, float w_req)
{
    const double dt = 0.04;
    if (FC_KV_RAMP > 0.0f && v_req > 0.0f) {
        double step = (double)FC_KV_RAMP * dt;
        if (fc_vlin_ramp < (double)v_req) {
            fc_vlin_ramp += step;
            if (fc_vlin_ramp > (double)v_req) fc_vlin_ramp = (double)v_req;
        } else if (fc_vlin_ramp > (double)v_req) {
            fc_vlin_ramp -= step;
            if (fc_vlin_ramp < (double)v_req) fc_vlin_ramp = (double)v_req;
        }
    } else {
        fc_vlin_ramp = (v_req == 0.0f) ? 0.0 : (double)v_req;
    }
    robot.setRobotVW((float)fc_vlin_ramp, w_req);
}
 
// ============================================================================
// Dist2Arc
//   pix, piy : nearest point ON the circle
//   dist     : signed radial error (robot_dist_to_center - R)
//              >0 → robot is outside circle
//              <0 → robot is inside  circle
// ============================================================================
void Dist2Arc(double xc, double yc, double R,
              double xr, double yr,
              double &pix, double &piy, double &dist)
{
    double dx = xr - xc;
    double dy = yr - yc;
    double d  = std::sqrt(dx*dx + dy*dy);
 
    if (d < 1e-6) {
        pix  = xc + R;
        piy  = yc;
        dist = -R;
        return;
    }
 
    double ux = dx / d;
    double uy = dy / d;
 
    pix  = xc + R * ux;
    piy  = yc + R * uy;
    dist = d - R;
}
 
// ============================================================================
// resetFollowCircle
// ============================================================================
void resetFollowCircle()
{
    followCircleState = FC_Follow_Arc;
    state             = Rotation;   // primes gotoXY cleanly for FC_Final_Rot
    fc_nearX          = 0.0;
    fc_nearY          = 0.0;
    fc_distToArc      = 0.0;
    fc_vlin_ramp      = 0.0;
    vlin              = 0.0;
    omega             = 0.0;
}
 
// ============================================================================
// followCircle
//
//   xc, yc : circle centre (m)
//   R      : radius (m, must be > 0)
//   angf   : final angle on circle (rad, global frame)
//            robot stops at (xc + R*cos(angf), yc + R*sin(angf))
//   tf     : desired final robot heading (rad)
//   dir    : +1 = CCW,  -1 = CW
// ============================================================================
void followCircle(double xc, double yc, double R, double angf, double tf, int dir)
{
    // ------------------------------------------------------------------
    // Geometry
    // ------------------------------------------------------------------
    Dist2Arc(xc, yc, R, x, y, fc_nearX, fc_nearY, fc_distToArc);
 
    // Current angle of robot projected onto circle
    double alfa = std::atan2(y - yc, x - xc);
 
    // Final point on circle (used by gotoXY in FC_Final_Rot)
    double xf = xc + R * std::cos(angf);
    double yf = yc + R * std::sin(angf);
 
    // Remaining arc length in travel direction
    double arc_diff = NormalizeAngle(angf - alfa);
    if (dir > 0 && arc_diff < 0) arc_diff += 2.0 * M_PI;
    if (dir < 0 && arc_diff > 0) arc_diff -= 2.0 * M_PI;
    double error_dist = std::abs(arc_diff) * R;
 
    // Tangent direction: CCW → alfa + pi/2,  CW → alfa - pi/2
    double tangentAngle = alfa + dir * M_PI / 2.0;
    double error_ang    = NormalizeAngle(tangentAngle - theta);
 
    // ------------------------------------------------------------------
    // State transitions
    // ------------------------------------------------------------------
    switch (followCircleState) {
 
        case FC_Follow_Arc:
            if (error_dist < 10.0 * FC_TOL_FINDIST)
                followCircleState = FC_Approaching;
            break;
 
        case FC_Approaching:
            if (error_dist < FC_TOL_FINDIST) {
                followCircleState = FC_Final_Rot;
                state         = Rotation;
                vlin          = 0.0;
                omega         = 0.0;
                fc_vlin_ramp  = 0.0;
                FC_MotorVel(0.0f, 0.0f);
            }
            break;
 
        case FC_Final_Rot:
            if (state == StopState)
                followCircleState = FC_Stop;
            break;
 
        case FC_Stop:
            break;
    }
 
    // ------------------------------------------------------------------
    // Outputs
    // ------------------------------------------------------------------
    switch (followCircleState) {
 
        case FC_Follow_Arc:
            vlin  = FC_VEL_LIN_NOM;
            omega = FC_K_ANG * error_ang * FC_VEL_ANG_NOM
                  - FC_K_RAD * fc_distToArc * dir;
            FC_MotorVel((float)vlin, (float)omega);
            break;
 
        case FC_Approaching:
            vlin  = FC_LinDeAccel;
            omega = FC_K_ANG * error_ang * FC_VEL_ANG_NOM
                  - FC_K_RAD * fc_distToArc * dir;
            FC_MotorVel((float)vlin, (float)omega);
            break;
 
        case FC_Final_Rot:
            // gotoXY still uses followLine constants (MAX_ETF, TOL_FINTHETA, etc.)
            // which is fine — it's just a short positional snap at the end
            gotoXY(xf, yf, tf);
            break;
 
        case FC_Stop:
            vlin  = 0.0;
            omega = 0.0;
            FC_MotorVel(0.0f, 0.0f);
            break;
    }
}