#ifndef EKF_H
#define EKF_H

// =============================================================================
// EKF State and Matrix Dimensions:
// =============================================================================
// XR:        [3 x 1]   - State vector [x, y, theta]
// P:         [3 x 3]   - State covariance
// Q:         [2 x 2]   - Process noise covariance (v_lin, omega)
// R:         [2 x 2]   - Measurement noise covariance (distance, angle)
// grad_f_X:  [3 x 3]   - State transition Jacobian (df/dX)
// grad_f_q:  [3 x 2]   - Input Jacobian (df/dU)
// grad_h_X:  [2 x 3]   - Measurement Jacobian (dh/dX)
// K:         [3 x 2]   - Kalman gain
// Z:         [2 x 1]   - Measurement vector [distance, angle]
// innovation:[2 x 1]   - Innovation (Z - h(X))
// =============================================================================

#include <BasicLinearAlgebra.h>
#include <math.h>
#include <Arduino.h>

// Standard deviations for tuning (kept for reference)
#define lin_stddev 1E-2
#define omega_stddev 1E-2
#define sensD_stddev 0.05
#define sensA_stddev 0.009 // radians

// Search window for beacon association (rays on each side)
#define deltaRay 15

// EKF dimensions
#define NStates 3   // x, y, theta
#define NObs 2      // range, angle

// Number of beacons in the environment
#define NBEACONS 6

// Minimum number of LIDAR points to consider a valid beacon detection
#define MIN_BEACON_POINTS 1

// Maximum innovation threshold for outlier rejection
#define MAX_DIST_INNOVATION 0.3    // meters
#define MAX_ANGLE_INNOVATION 0.5   // radians (~28 degrees)

// Position structure
typedef struct {
    double x;
    double y;
} TPos;

// Beacon cluster structure (for association)
typedef struct {
    double x;
    double y;
    double dist;
    double angle;
    int n;          // number of points in cluster
    int firstRay;   // first ray index for searching
} TClusterPos;

// External variables for debugging
extern int idx_beacon, idx;
extern double MeasureDist;
extern TPos MeasurePos;

using namespace BLA;

class EKF {
public:
    // Robot/control state variables
    int irobot;
    int iLaser;
    int firstRay, lastRay;
    int imp1, imp2;
    int state, stateLine, stateSquare;
    double vlin, omega;
    double vlin_prev, omega_prev;
    double t, dt, w, theta;
    double error_dist_prev;

    // Identity matrix [3x3]
    BLA::Matrix<3, 3, double> I;

    // State vector and predicted state [3x1]
    BLA::Matrix<NStates, 1, double> XR, XRe;
    
    // State covariance [3x3]
    BLA::Matrix<NStates, NStates, double> P;
    
    // Process noise covariance [2x2]
    BLA::Matrix<NObs, NObs, double> Q;
    
    // Measurement noise covariance [2x2]
    BLA::Matrix<NObs, NObs, double> R;
    
    // State transition Jacobian [3x3]
    BLA::Matrix<NStates, NStates, double> grad_f_X;
    
    // Input Jacobian [3x2]
    BLA::Matrix<NStates, 2, double> grad_f_q;
    
    // Measurement Jacobian [2x3]
    BLA::Matrix<NObs, NStates, double> grad_h_X;
    
    // Kalman gain [3x2]
    BLA::Matrix<NStates, NObs, double> K;
    
    // Measurement and innovation vectors [2x1]
    BLA::Matrix<NObs, 1, double> Z;           // Actual measurement
    BLA::Matrix<NObs, 1, double> innovation;  // Z - h(X) (renamed from Z_E for clarity)
    
    // Innovation covariance [2x2]
    BLA::Matrix<NObs, NObs, double> S;

    // LIDAR measurements array [1x360] - range values in meters
    BLA::Matrix<1, 360, double> LaserValues;
    
    // Known beacon positions in world frame
    TPos BeaconPos[NBEACONS];
    
    // Detected beacon clusters from LIDAR
    TClusterPos BeaconCluster[NBEACONS];

    // Constructor
    EKF();

    // Prediction step: propagate state and covariance
    void predict(double vlin, double omega, double dt);
    
    // Update state using motion model (without covariance update)
    void updateXR(double vlin, double omega, double dt);
    
    // Update state transition Jacobian
    void updateJacState(double vlin, double omega, double dt);
    
    // Update input Jacobian
    void updateJacInput(double vlin, double omega, double dt);
    
    // Propagate covariance matrix
    void covariancePropagation();

    // EKF update step with beacon measurement
    void updateEKF(int nBeacon);
    
    // Beacon association and validation from LIDAR scan
    void phaseAV();
    
    // Process all detected beacons for EKF update
    void motionmodelEKF();

private:
    // Helper: compute expected measurement for a beacon
    void computeExpectedMeasurement(int nBeacon, double& expectedDist, double& expectedAngle);
    
    // Helper: update measurement Jacobian for a beacon
    void updateMeasurementJacobian(int nBeacon, double dBeacon);
    
    // Helper: check if innovation is within acceptable bounds
    bool isInnovationValid(double distInnovation, double angleInnovation);
};

extern EKF ekf;

#endif // EKF_H