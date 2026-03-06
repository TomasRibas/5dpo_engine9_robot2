#include "followLine.h"

// ----- Configurable Constants (can be set via gchannels) -----
// Using float instead of double for gchannels compatibility
float VEL_ANG_NOM = 1.0f;
float VEL_LIN_NOM = 0.2f;
float W_DA        = 0.5f;
float LinDeAccel  = 0.1f;

float MAX_ETF      = 15.0f * M_PI/180.0f;//0.2618
float HIST_ETF     =  5.0f * M_PI/180.0f;//0.0873
float GAIN_FWD     = 0.1f;
float DIST_DA      = 0.05f;
float GAIN_DA      = 0.1f;

float TOL_FINDIST  = 0.01f;
float DIST_NEWPOSE = 0.5f;
float THETA_NEWPOSE = 15.0f * M_PI/180.0f;//0.2618
float THETA_DA      = 5.0f * M_PI/180.0f;//0.0873
float TOL_FINTHETA  = 1.0f * M_PI/180.0f;//0.01745

float DIST_NEWLINE  = 0.3f;  // Increased for smoother merging
float DIST_NEARLINE = 0.05f;

// Line following omega gains
float K_DIST = 0.3f;
float K_ANG  = 0.15f;

// Velocity ramp rate (m/s²) — 0 = disabled (instant step)
float KV_RAMP = 0.5f;   // cmd: "kvramp"

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

// Trajectory state
TrajectoryState trajectory = {false, 0, 0, false, nullptr};

// Figure-8 waypoints (static storage)
static Waypoint figure8_waypoints[8] = {
    {-0.695, -0.355,  0.000, -0.355,  1.57},  // Segment 1
    { 0.000, -0.355,  0.000,  0.355,  0.00},  // Segment 2
    { 0.000,  0.355,  0.695,  0.355, -1.57},  // Segment 3
    { 0.695,  0.355,  0.695, -0.355,  3.14},  // Segment 4
    { 0.695, -0.355,  0.000, -0.355,  1.57},  // Segment 5
    { 0.000, -0.355,  0.000,  0.355,  3.14},  // Segment 6
    { 0.000,  0.355, -0.695,  0.355, -1.57},  // Segment 7
    {-0.695,  0.355, -0.695, -0.355,  1.57}   // Segment 8
};

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
}

void MotorVel(float v_req, float w_req) {
    const double dt = 0.04;  // 40ms control period
    if (KV_RAMP > 0.0f && v_req > 0.0f) {
        // Ramp up OR ramp down between two non-zero speeds
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
        // v_req == 0 (stop): always instant — robot must stop immediately for rotations
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

    Dist2Line(xi, yi, xf, yf, x, y, kl, nearX, nearY);

    testSideLine = sign(kl);
    distLine     = std::abs(kl);

    // State transitions - MATCHING PASCAL LOGIC
    switch (followLineState) {
        case Follow_Line:
            if (error_dist < 10.0 * TOL_FINDIST) {
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
                // Normal line following
                vlin  = VEL_LIN_NOM;
                omega = K_DIST * testSideLine * distLine + K_ANG * error_ang * VEL_ANG_NOM;
            }
            MotorVel((float)vlin, (float)omega);
            break;
        }

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

// ============================================================================
// TRAJECTORY FUNCTIONS
// ============================================================================

void startFigure8() {
    trajectory.waypoints = figure8_waypoints;
    trajectory.total_segments = 8;
    trajectory.current_segment = 0;
    trajectory.loop = true;  // Loop by default
    trajectory.active = true;
    
    resetFollowLine();
    
    // Serial.println("Figure-8 trajectory started");
}

void startCustomTrajectory(Waypoint* waypoints, int count, bool loop) {
    trajectory.waypoints = waypoints;
    trajectory.total_segments = count;
    trajectory.current_segment = 0;
    trajectory.loop = loop;
    trajectory.active = true;
    
    resetFollowLine();
    
    // Serial.print("Custom trajectory started: ");
    // Serial.print(count);
    // Serial.println(" segments");
}

void stopTrajectory() {
    trajectory.active = false;
    vlin = 0.0;
    omega = 0.0;
    MotorVel(0.0, 0.0);
    
    // Serial.println("Trajectory stopped");
}

bool isTrajectoryComplete() {
    if (!trajectory.active) return true;
    return (trajectory.current_segment >= trajectory.total_segments && !trajectory.loop);
}

void executeTrajectory() {
    if (!trajectory.active) return;
    if (trajectory.waypoints == nullptr) return;
    
    // Check if current segment is complete
    if (followLineState == Stop_FL && state == StopState) {
        // Segment complete - move to next
        trajectory.current_segment++;
        
        // Serial.print("Segment ");
        // Serial.print(trajectory.current_segment);
        // Serial.print("/");
        // Serial.print(trajectory.total_segments);
        // Serial.println(" complete");
        
        // Check if trajectory finished
        if (trajectory.current_segment >= trajectory.total_segments) {
            if (trajectory.loop) {
                // Loop back to start
                trajectory.current_segment = 0;
                // Serial.println("Looping trajectory...");
            } else {
                // Stop trajectory
                stopTrajectory();
                // Serial.println("Trajectory complete!");
                return;
            }
        }
        
        resetFollowLine();
        
    }
    
    // Execute current segment
    if (trajectory.current_segment < trajectory.total_segments) {
        Waypoint &wp = trajectory.waypoints[trajectory.current_segment];
        followLine(wp.xi, wp.yi, wp.xf, wp.yf, wp.tf);
    }
}