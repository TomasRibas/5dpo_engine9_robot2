#ifndef FOLLOWLINE_H
#define FOLLOWLINE_H

#include <cmath>

class FollowLineController {
public:
    // ---- Follow-line state machine ----
    enum LineState {
        Follow_Line = 0,
        Approaching,
        Final_Rot_FL,
        Stop_FL,
        GoTo_NearXY
    };

    // ---- Go-to-point internal state machine ----
    enum GotoState {
        Goto_Go = 0,
        Goto_FinalRot,
        Goto_Stop
    };

    FollowLineController();

    // Pose input (meters, radians)
    void setPose(double x, double y, double theta);

    // Run controller
    // Line defined by (xi,yi) -> (xf,yf), final desired heading tf
    void followLine(double xi, double yi, double xf, double yf, double tf);

    // Outputs (linear velocity, angular velocity)
    void getOutputs(double &v, double &w) const;

    // Optional direct getters
    double v() const { return vlin_; }
    double w() const { return omega_; }

    // State getter
    LineState lineState() const { return stateLine_; }

    // Stop command (forces Stop_FL)
    void stop();

    // ---- Parameters (tune as needed) ----
    // Distances in meters, angles in radians, velocities in m/s and rad/s.
    double TOL_FINDIST;     // final distance tolerance
    double DIST_NEWLINE;    // if too far from line -> go to near point
    double DIST_NEARLINE;   // when close enough -> resume follow
    double VEL_LIN_NOM;     // nominal linear speed on line
    double VEL_ANG_NOM;     // nominal scaling used by controller (kept for similarity)
    double LinDeAccel;      // linear speed during approach

    // Gains (you had fixed 0.3 and 0.15; make them tunable)
    double K_line_dist;     // multiplies signed dist to line
    double K_line_ang;      // multiplies heading error

    // Goto controller parameters
    double GOTO_V_MAX;      // max linear speed
    double GOTO_W_MAX;      // max angular speed
    double GOTO_K_RHO;      // distance gain
    double GOTO_K_ALPHA;    // heading-to-target gain
    double GOTO_K_BETA;     // final heading gain
    double GOTO_TOL_POS;    // position tolerance for goto
    double GOTO_TOL_ANG;    // angle tolerance for goto final rotation

    // Debug/diagnostics (optional)
    double distLine;        // abs distance to line
    double nearX, nearY;    // closest point on line
    int    testSideLine;    // sign of distance (+1/-1)

private:
    // Pose
    double x_, y_, theta_;

    // Outputs
    double vlin_, omega_;

    // States
    LineState stateLine_;
    GotoState stateGoto_;

    // Internal helpers
    static double NormalizeAngle(double a);
    static int sign(double v);

    static void Dist2Line(double xi, double yi,
                          double xf, double yf,
                          double xr, double yr,
                          double &kl, double &pix, double &piy);

    // Internal goto (standalone)
    void gotoXY(double gx, double gy, double gtheta);

    // Clamp
    static double clamp(double v, double lo, double hi);
};

#endif // FOLLOWLINE_H
