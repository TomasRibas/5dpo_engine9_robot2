#include "followLine.h"

FollowLineController::FollowLineController()
: x_(0.0), y_(0.0), theta_(0.0),
  vlin_(0.0), omega_(0.0),
  stateLine_(Follow_Line),
  stateGoto_(Goto_Go),
  distLine(0.0), nearX(0.0), nearY(0.0), testSideLine(0)
{
    // ---- Defaults (tune to your robot) ----
    TOL_FINDIST   = 0.05;   // 5 cm
    DIST_NEWLINE  = 0.20;   // 20 cm
    DIST_NEARLINE = 0.05;   // 5 cm
    VEL_LIN_NOM   = 0.20;   // m/s
    VEL_ANG_NOM   = 1.00;   // used as scaling term
    LinDeAccel    = 0.10;   // m/s

    K_line_dist = 0.3;
    K_line_ang  = 0.15;

    // Simple goto defaults
    GOTO_V_MAX   = 0.25;
    GOTO_W_MAX   = 1.50;
    GOTO_K_RHO   = 0.8;
    GOTO_K_ALPHA = 2.0;
    GOTO_K_BETA  = -0.5;
    GOTO_TOL_POS = 0.05;     // 5 cm
    GOTO_TOL_ANG = 0.05;     // ~3 degrees
}

void FollowLineController::setPose(double x, double y, double theta) {
    x_ = x;
    y_ = y;
    theta_ = NormalizeAngle(theta);
}

void FollowLineController::getOutputs(double &v, double &w) const {
    v = vlin_;
    w = omega_;
}

void FollowLineController::stop() {
    stateLine_ = Stop_FL;
    vlin_ = 0.0;
    omega_ = 0.0;
}

// -------- Helpers --------

double FollowLineController::NormalizeAngle(double a) {
    // Normalize to (-pi, pi]
    while (a <= -M_PI) a += 2.0 * M_PI;
    while (a >   M_PI) a -= 2.0 * M_PI;
    return a;
}

int FollowLineController::sign(double v) {
    if (v > 0.0) return  1;
    if (v < 0.0) return -1;
    return 0;
}

double FollowLineController::clamp(double v, double lo, double hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void FollowLineController::Dist2Line(double xi, double yi,
                                    double xf, double yf,
                                    double xr, double yr,
                                    double &kl, double &pix, double &piy)
{
    const double dx = (xf - xi);
    const double dy = (yf - yi);
    const double n  = std::sqrt(dx*dx + dy*dy);

    // Degenerate line (two equal points): nearest point is (xi,yi)
    if (n < 1e-12) {
        pix = xi;
        piy = yi;
        // signed distance doesn't really make sense; return euclidean distance
        kl = std::sqrt((xr - xi)*(xr - xi) + (yr - yi)*(yr - yi));
        return;
    }

    // Unit direction
    const double ux = dx / n;
    const double uy = dy / n;

    // Same formula as your Pascal (signed perpendicular distance)
    kl = (xr*uy - yr*ux - xi*uy + yi*ux) / (ux*ux + uy*uy); // denom is ~1, kept for fidelity

    // Nearest point on the infinite line
    pix = -kl*uy + xr;
    piy =  kl*ux + yr;
}

// -------- Internal gotoXY (standalone) --------
// This produces vlin_/omega_ directly (no external dependency)
void FollowLineController::gotoXY(double gx, double gy, double gtheta)
{
    const double dx = gx - x_;
    const double dy = gy - y_;
    const double rho = std::sqrt(dx*dx + dy*dy);

    // Angle to goal
    const double goalAng = std::atan2(dy, dx);
    const double alpha = NormalizeAngle(goalAng - theta_);
    const double beta  = NormalizeAngle(gtheta - goalAng);

    // State transitions
    switch (stateGoto_) {
        case Goto_Go:
            if (rho < GOTO_TOL_POS) stateGoto_ = Goto_FinalRot;
            break;
        case Goto_FinalRot:
            if (std::fabs(NormalizeAngle(gtheta - theta_)) < GOTO_TOL_ANG) stateGoto_ = Goto_Stop;
            break;
        case Goto_Stop:
            // stay
            break;
    }

    // Outputs
    switch (stateGoto_) {
        case Goto_Go: {
            double v = GOTO_K_RHO * rho;
            double w = GOTO_K_ALPHA * alpha + GOTO_K_BETA * beta;

            v = clamp(v, -GOTO_V_MAX, GOTO_V_MAX);
            w = clamp(w, -GOTO_W_MAX, GOTO_W_MAX);

            vlin_  = v;
            omega_ = w;
        } break;

        case Goto_FinalRot: {
            const double e = NormalizeAngle(gtheta - theta_);
            double w = 2.5 * e; // simple P controller for final heading
            w = clamp(w, -GOTO_W_MAX, GOTO_W_MAX);

            vlin_  = 0.0;
            omega_ = w;
        } break;

        case Goto_Stop:
            vlin_  = 0.0;
            omega_ = 0.0;
            break;
    }
}

// -------- Follow line main --------

void FollowLineController::followLine(double xi, double yi, double xf, double yf, double tf)
{
    // Heading of the line (xi,yi)->(xf,yf)
    const double tr = std::atan2(yf - yi, xf - xi);

    // Heading error relative to the line direction
    const double error_ang = NormalizeAngle(tr - theta_);

    // Distance to final point
    const double ex = (xf - x_);
    const double ey = (yf - y_);
    const double error_dist = std::sqrt(ex*ex + ey*ey);

    // Perpendicular signed distance to line + nearest point
    double signedDist = 0.0;
    Dist2Line(xi, yi, xf, yf, x_, y_, signedDist, nearX, nearY);

    testSideLine = sign(signedDist);
    distLine     = std::fabs(signedDist);

    // ----- State transitions (same logic as your Pascal) -----
    switch (stateLine_) {
        case Follow_Line:
            if (error_dist < 10.0 * TOL_FINDIST) {
                stateLine_ = Approaching;
            } else if (distLine > DIST_NEWLINE) {
                stateLine_ = GoTo_NearXY;
                stateGoto_ = Goto_Go; // reset goto
            }
            break;

        case Approaching:
            if (error_dist < TOL_FINDIST) {
                stateLine_ = Final_Rot_FL;
                stateGoto_ = Goto_Go; // reset goto for final
            }
            break;

        case Final_Rot_FL:
            // when goto reaches stop, we stop follow-line too
            if (stateGoto_ == Goto_Stop) {
                stateLine_ = Stop_FL;
            }
            break;

        case Stop_FL:
            if (distLine > DIST_NEWLINE) {
                stateLine_ = GoTo_NearXY;
                stateGoto_ = Goto_Go;
            }
            break;

        case GoTo_NearXY:
            if (distLine < DIST_NEARLINE) {
                stateLine_ = Follow_Line;
            }
            break;
    }

    // ----- Outputs (same intent as Pascal) -----
    switch (stateLine_) {
        case Follow_Line:
            vlin_  = VEL_LIN_NOM;
            omega_ = K_line_dist * (double)testSideLine * distLine
                   + K_line_ang  * error_ang * VEL_ANG_NOM;
            break;

        case Approaching:
            vlin_  = LinDeAccel;
            omega_ = K_line_dist * (double)testSideLine * distLine
                   + K_line_ang  * error_ang * VEL_ANG_NOM;
            break;

        case Final_Rot_FL:
            // go to final point with final heading tf
            gotoXY(xf, yf, tf);
            break;

        case Stop_FL:
            vlin_  = 0.0;
            omega_ = 0.0;
            break;

        case GoTo_NearXY:
            // go to nearest point on the line, aligned with the line direction
            gotoXY(nearX, nearY, tr);
            break;
    }
}
