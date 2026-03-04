#include <ekf.h>
#include "utils.h"

EKF ekf;

int idx_beacon, idx;
double MeasureDist;
TPos MeasurePos;

EKF::EKF() {

    dt = 0.04;
    
    // LaserValues.Fill(0.0);

    I.Fill(0.0);
    I(0, 0) = 1.0;
    I(1, 1) = 1.0;
    I(2, 2) = 1.0;

    P.Fill(0.0);
    P(0, 0) = 0.01;  // Initial uncertainty in x (m²)
    P(1, 1) = 0.01;  // Initial uncertainty in y (m²)
    P(2, 2) = 0.01;  // Initial uncertainty in theta (rad²)
    
    Q.Fill(0.0);
    Q(0, 0) = pow(0.1, 2);   // Process noise for v_lin (m/s)²  pow(0.01, 2);
    Q(1, 1) = pow(0.02, 2);   // Process noise for omega (rad/s)²

    R.Fill(0.0);
    R(0, 0) = pow(0.05, 2);   // Initial/fallback — overwritten per-beacon by calRrr(r)
    R(1, 1) = pow(0.02, 2);   // Initial/fallback — overwritten per-beacon by calRaa(r)

    BeaconPos[0].x = -0.8985;  BeaconPos[0].y = -0.6485;
    BeaconPos[1].x = -0.8985;  BeaconPos[1].y = 0.6485;
    BeaconPos[2].x = 0;        BeaconPos[2].y = -0.6485;     
    BeaconPos[3].x = 0;        BeaconPos[3].y = 0.6485;
    BeaconPos[4].x = 0.8985;   BeaconPos[4].y = -0.6485;
    BeaconPos[5].x = 0.8985;   BeaconPos[5].y = 0.6485;

    for (int j = 0; j < NBEACONS; j++) {
        BeaconCluster[j].x = 0.0;
        BeaconCluster[j].y = 0.0;
        BeaconCluster[j].n = 0;
        BeaconCluster[j].dist = 0.0;
        BeaconCluster[j].angle = 0.0;
        BeaconCluster[j].firstRay = 0;   
    }

    // Set initial robot state
    // XR(0) = -0.785;
    // XR(1) = -0.57;
    // XR(2) = PI/2;   
    XR(0) = -0.695;
    XR(1) = -0.355;
    XR(2) = PI/2;   

    grad_f_X.Fill(0.0);
    grad_f_X(0, 0) = 1.0;
    grad_f_X(1, 1) = 1.0;
    grad_f_X(2, 2) = 1.0;

    grad_f_q.Fill(0.0);

    grad_h_X.Fill(0.0);
}

// =============================================================================
// PREDICTION STEP
// =============================================================================
void EKF::predict(double vlin, double omega, double dt) {
    updateXR(vlin, omega, dt);
    updateJacState(vlin, omega, dt);
    updateJacInput(vlin, omega, dt);
    covariancePropagation();
}

void EKF::updateXR(double vlin, double omega, double dt) {
    double theta_mid = XR(2) + 0.5 * omega * dt;
    
    XR(0) = XR(0) + vlin * dt * cos(theta_mid);
    XR(1) = XR(1) + vlin * dt * sin(theta_mid);
    XR(2) = normalizeAngle(XR(2) + omega * dt);
}

void EKF::updateJacState(double vlin, double omega, double dt) {
    double theta_mid = XR(2) + 0.5 * omega * dt;
    
    grad_f_X(0, 2) = -vlin * dt * sin(theta_mid);
    grad_f_X(1, 2) =  vlin * dt * cos(theta_mid);
}

void EKF::updateJacInput(double vlin, double omega, double dt) {
    double theta_mid = XR(2) + 0.5 * omega * dt;
    
    grad_f_q(0, 0) = dt * cos(theta_mid);
    grad_f_q(1, 0) = dt * sin(theta_mid);
    grad_f_q(2, 0) = 0.0;
    
    grad_f_q(0, 1) = -0.5 * vlin * dt * dt * sin(theta_mid);
    grad_f_q(1, 1) =  0.5 * vlin * dt * dt * cos(theta_mid);
    grad_f_q(2, 1) = dt;
}

void EKF::covariancePropagation() {
    P = grad_f_X * P * (~grad_f_X) + grad_f_q * Q * (~grad_f_q);
}

// =============================================================================
// CALIBRATION MODELS (Gaussian-weighted fits from calibration data)
// Offset, Rrr, Raa as functions of measured distance r
// =============================================================================

// Offset (bias): r_true = r_raw + offset(r)
// Gauss cubic fit: offset(r) = a·r³ + b·r² + c·r + d
static double calOffset(double r) {
    return -0.00827633*r*r*r + 0.009161*r*r + (-0.020808)*r + 0.060586;
}

// Rrr: variance of distance measurement (m²)
// Gauss cubic fit
static double calRrr(double r) {
    double v = -0.00000127*r*r*r + 0.000007*r*r + (-0.000005)*r + 0.000001;
    return (v > 1e-8) ? v : 1e-8;  // floor to avoid zero variance
}

// Raa: variance of angle measurement (rad²)
// Gauss cubic fit (coefficients in deg², converted to rad²)
static double calRaa(double r) {
    double deg2 = -0.01870396*r*r*r + 0.086466*r*r + (-0.118351)*r + 0.072489;
    if (deg2 < 0.005) deg2 = 0.005;  // floor at 0.005 deg²
    return deg2 * (M_PI / 180.0) * (M_PI / 180.0);  // convert to rad²
}

// =============================================================================
// UPDATE STEP — with distance-dependent R from calibration
// =============================================================================
void EKF::updateEKF(int nBeacon) {
    // Raw measurement from phaseAV (already includes BEACON_RADIUS_OFFSET)
    double rawDist  = BeaconCluster[nBeacon].dist;
    double rawAngle = BeaconCluster[nBeacon].angle;

    // Apply calibration offset correction:
    // phaseAV adds BEACON_RADIUS_OFFSET (fixed 0.04m), but calibration shows
    // the true offset varies with distance. Correct by replacing fixed with model.
    double dist_no_offset = rawDist - BEACON_RADIUS_OFFSET;  // undo fixed offset
    double correctedDist = dist_no_offset + calOffset(dist_no_offset);  // apply calibrated offset

    Z(0) = correctedDist;
    Z(1) = rawAngle;

    // Update R matrix based on measured distance
    R(0, 0) = calRrr(dist_no_offset);
    R(1, 1) = calRaa(dist_no_offset);

    double dx = BeaconPos[nBeacon].x - XR(0);
    double dy = BeaconPos[nBeacon].y - XR(1);
    double dBeacon = sqrt(dx * dx + dy * dy);
    
    if (dBeacon < 0.001) {
        return;
    }
    
    double expectedDist = dBeacon;
    double expectedAngle = normalizeAngle(atan2(dy, dx) - XR(2));
    
    innovation(0) = Z(0) - expectedDist;
    innovation(1) = normalizeAngle(Z(1) - expectedAngle);
    
    if (!isInnovationValid(innovation(0), innovation(1))) {
        return;
    }

    double d2 = dBeacon * dBeacon;
    
    grad_h_X(0, 0) = -dx / dBeacon;
    grad_h_X(0, 1) = -dy / dBeacon;
    grad_h_X(0, 2) = 0.0;
    
    grad_h_X(1, 0) = dy / d2;
    grad_h_X(1, 1) = -dx / d2;
    grad_h_X(1, 2) = -1.0;

    S = grad_h_X * P * (~grad_h_X) + R;
    
    BLA::Matrix<NObs, NObs, double> S_inv = Inverse(S);
    K = P * (~grad_h_X) * S_inv;

    P = (I - K * grad_h_X) * P;

    XR = XR + K * innovation;
    XR(2) = normalizeAngle(XR(2));
}

bool EKF::isInnovationValid(double distInnovation, double angleInnovation) {
    if (fabs(distInnovation) > MAX_DIST_INNOVATION) {
        return false;
    }
    if (fabs(angleInnovation) > MAX_ANGLE_INNOVATION) {
        return false;
    }
    return true;
}


void EKF::phaseAV() {

  // Reset all beacon clusters
  for (int j = 0; j < NBEACONS; j++) {
    BeaconCluster[j].x = 0.0;
    BeaconCluster[j].y = 0.0;
    BeaconCluster[j].n = 0;
    BeaconCluster[j].dist = 0.0;
    BeaconCluster[j].angle = 0.0;

  }

  // For each beacon, try to associate scan points near its expected angle
  for (int j = 0; j < NBEACONS; j++) {

    // int passDist = 0;
    // int passAng  = 0;
    // int passAssoc = 0;
    // double bestAbsAngErr = 1e9;
    // double bestAssocDist = 1e9;

    // Beacon expected angle in robot frame
    double dx = BeaconPos[j].x - XR(0) - LIDAR_TO_ROBOT_OFFSET * cos(XR(2));
    double dy = BeaconPos[j].y - XR(1) - LIDAR_TO_ROBOT_OFFSET * sin(XR(2));
    double expectedAngle = normalizeAngle(atan2(dy, dx) - XR(2)); // [-pi, +pi]

    double windowRad = (double)deltaRay * 0.5 * M_PI / 180.0;


    for (uint16_t k = 0; k < scanN; k++) {

        double MeasureDist = (double)scanPts[k].dist;

        const double LIDAR_YAW_OFFSET = -5; 
        double rayAngleRad = (double)scanPts[k].ang + LIDAR_YAW_OFFSET * M_PI / 180.0; 
        rayAngleRad = normalizeAngle(rayAngleRad); 

        if (MeasureDist < 0.03 || MeasureDist > 2.5) continue;
        //passDist++;

        double angErr = fabs(normalizeAngle(rayAngleRad - expectedAngle));
        // if (angErr < bestAbsAngErr) bestAbsAngErr = angErr;

        if (angErr > windowRad) continue;
        //passAng++;


        double dAng = normalizeAngle(rayAngleRad - expectedAngle);
        if (fabs(dAng) > windowRad) continue;

        // Convert measurement to world coordinates
        MeasurePos.x = MeasureDist * cos(rayAngleRad + XR(2)) + XR(0) + LIDAR_TO_ROBOT_OFFSET * cos(XR(2));
        MeasurePos.y = MeasureDist * sin(rayAngleRad + XR(2)) + XR(1) + LIDAR_TO_ROBOT_OFFSET * sin(XR(2));

        double distToBeacon = dist(BeaconPos[j].x - MeasurePos.x, BeaconPos[j].y - MeasurePos.y);
        // if (distToBeacon < bestAssocDist) bestAssocDist = distToBeacon;

        if (distToBeacon < ASSOCIATION_THRESHOLD) {
            //passAssoc++;

            BeaconCluster[j].n++;
            double n = BeaconCluster[j].n;

            BeaconCluster[j].x = BeaconCluster[j].x * (n - 1) / n + MeasurePos.x / n;
            BeaconCluster[j].y = BeaconCluster[j].y * (n - 1) / n + MeasurePos.y / n;
        }
    }

    // Final distance+angle for this beacon if we saw anything
    if (BeaconCluster[j].n >= 1) {
      double cluster_dx = BeaconCluster[j].x - XR(0);
      double cluster_dy = BeaconCluster[j].y - XR(1);

      BeaconCluster[j].angle = normalizeAngle(atan2(cluster_dy, cluster_dx) - XR(2));
      BeaconCluster[j].dist  = sqrt(cluster_dx * cluster_dx + cluster_dy * cluster_dy) + BEACON_RADIUS_OFFSET;

      BeaconCluster[j].x += BEACON_RADIUS_OFFSET * cos(atan2(cluster_dy, cluster_dx));
      BeaconCluster[j].y += BEACON_RADIUS_OFFSET * sin(atan2(cluster_dy, cluster_dx));
    }

    // Serial.print("[Beacon "); Serial.print(j);
    // // Serial.print("] passDist="); Serial.print(passDist);
    // // Serial.print(" passAng="); Serial.print(passAng);
    // // Serial.print(" passAssoc="); Serial.print(passAssoc);
    // // Serial.print(" bestAngErrDeg="); Serial.print(bestAbsAngErr * 180.0 / M_PI, 2);
    // // Serial.print(" bestAssocDist="); Serial.println(bestAssocDist, 3);

    // Serial.print("Beacon "); Serial.print(j);
    // Serial.print(": nPts="); Serial.print(BeaconCluster[j].n);
    // Serial.print(" x="); Serial.print(BeaconCluster[j].x, 3);
    // Serial.print(" y="); Serial.println(BeaconCluster[j].y, 3);
  }
}



void EKF::motionmodelEKF() {
    for (int j = 0; j < NBEACONS; j++) {
        if (BeaconCluster[j].n >= MIN_BEACON_POINTS) {
            updateEKF(j);
        }
    }
}

void EKF::setScan(const float* ang, const float* dist, uint16_t n) {

    for (uint16_t i = 0; i < n; i++) {
        scanPts[i].ang  = ang[i];
        scanPts[i].dist = dist[i]; 
    }
    scanN = n;
}