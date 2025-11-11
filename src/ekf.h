// XR:        [3 x 1]   - State vector
// P:         [3 x 3]   - State covariance
// Q:         [2 x 2]   - Process noise (v_lin, omega)
// R:         [2 x 2]   - Measurement noise (distance, angle)
// grad_f_X:  [3 x 3]   - State Jacobian
// grad_f_q:  [3 x 2]   - Input Jacobian
// grad_h_X:  [2 x 3]   - Measurement Jacobian
// K:         [3 x 2]   - Kalman gain
// Z:         [2 x 1]   - Measurement [distance, angle]
// Z_E:       [2 x 1]   - Expected measurement

#include <BasicLinearAlgebra.h>
#include <math.h>
#include <Arduino.h>

#define lin_stddev 1E-2
#define omega_stddev 1E-2
#define sensD_stddev 0.005
#define sensA_stddev 0.009 //radians

#define deltaRay 8

#define NStates 3 // x, y, theta
#define NObs 2 // range, angle

#define NBEACONS 4



// double dist(double x, double y) {
//     return sqrt(pow(x, 2) + pow(y, 2));
// };
// double normalizeAngle(double angle) {
//     angle = fmod(angle, 2*M_PI);   // Wrap within (-360, 360)
//     if (angle < 0)
//         angle += 2*M_PI;           // Shift negative angles into [0, 360)
//     return angle;
// };



//should probably be inside publixc of EKF
// int irobot;
// int iLaser;
// int firstRay, lastRay;
// int imp1, imp2;
// int state, stateLine, stateSquare;
// double vlin, omega;
// double vlin_prev, omega_prev;
// double t, dt, w, theta;
// double error_dist_prev;

typedef struct {
    double x;
    double y;
} TPos;

typedef struct{
    double x;
    double y;
    double dist;
    double angle;
    int n;
    int firstRay;
} TClusterPos;

extern int idx_beacon, idx;
extern double MeasureDist;
extern TPos MeasurePos;

using namespace BLA;

class EKF {
    public:
        int irobot;
        int iLaser;
        int firstRay, lastRay;
        int imp1, imp2;
        int state, stateLine, stateSquare;
        double vlin, omega;
        double vlin_prev, omega_prev;
        double t, dt, w, theta;
        double error_dist_prev;

        BLA::Matrix<3, 3, double> I; //identity matrix

        BLA::Matrix<NStates, 1, double> XR, XRe; //state
        BLA::Matrix<NStates, NStates, double> P; //covariance
        BLA::Matrix<NObs, NObs, double> Q; //process noise
        BLA::Matrix<NObs, NObs, double> R; //observation noise
        BLA::Matrix<NStates, NStates, double> grad_f_X; //grad f(X)
        BLA::Matrix<NStates, 2, double> grad_f_q; //grad f(U)
        BLA::Matrix<NObs, NStates, double> grad_h_X; //grad h(X)
        BLA::Matrix<NStates, NObs, double> K; //Kalman gain
        BLA::Matrix<NObs, 1, double> Z, Z_E;
        BLA::Matrix<NObs, NObs, double> S;

        //need access to full LIDAR scan
        BLA::Matrix<1, 360, double> LaserValues; //LIDAR measurements [range, angle]'
        TPos BeaconPos[NBEACONS]; //beacon positions
        TClusterPos BeaconCluster[NBEACONS]; //beacon positions from clustering

        //EKF init
        EKF();

        void predict(double vlin, double omega, double dt);
        void updateXR(double vlin, double omega, double dt);
        void updateJacState(double vlin, double omega, double dt);
        void updateJacInput(double vlin, double omega, double dt);
        void covariancePropagation();

        void updateEKF(int nBeacon);
        //dh/dX, Kalman gain, P covariance update, update state with measurement Z
        //expected measurement Z_E
        //XR=XR+K(Z-Z_E)
        //update x,y,theta

        void phaseAV(); //Association and validation
        void motionmodelEKF();

};