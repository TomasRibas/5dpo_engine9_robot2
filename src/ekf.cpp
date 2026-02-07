#include <ekf.h>
#include "utils.h"

EKF ekf;

int idx_beacon, idx;
double MeasureDist;
TPos MeasurePos;

EKF::EKF() {
    // Time step (40 ms)
    dt = 0.04;
    
    // LIDAR data storage
    LaserValues.Fill(0.0);

    // Identity matrix [3x3]
    I.Fill(0.0);
    I(0, 0) = 1.0;
    I(1, 1) = 1.0;
    I(2, 2) = 1.0;

    P.Fill(0.0);
    P(0, 0) = 0.01;  // Initial uncertainty in x (m²)
    P(1, 1) = 0.01;  // Initial uncertainty in y (m²)
    P(2, 2) = 0.01;  // Initial uncertainty in theta (rad²)
    
    Q.Fill(0.0);
    Q(0, 0) = pow(0.01, 2);   // Process noise for v_lin (m/s)²
    Q(1, 1) = pow(0.02, 2);   // Process noise for omega (rad/s)²

    R.Fill(0.0);
    R(0, 0) = pow(0.05, 2);   // Measurement noise for distance (m²)
    R(1, 1) = pow(0.02, 2);   // Measurement noise for angle (rad²)

    // Set known beacon positions in world frame
    BeaconPos[0].x = -0.8985;  BeaconPos[0].y = -0.6485;
    BeaconPos[1].x = -0.8985;  BeaconPos[1].y = 0.6485;
    BeaconPos[2].x = 0;        BeaconPos[2].y = -0.6485;     
    BeaconPos[3].x = 0;        BeaconPos[3].y = 0.6485;
    BeaconPos[4].x = 0.8985;   BeaconPos[4].y = -0.6485;
    BeaconPos[5].x = 0.8985;   BeaconPos[5].y = 0.6485;

    // Initialize beacon clusters
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
    XR(0) = -0.785;
    XR(1) = -0.355;
    XR(2) = PI/2;   

    // Initialize state transition Jacobian [3x3]
    grad_f_X.Fill(0.0);
    grad_f_X(0, 0) = 1.0;
    grad_f_X(1, 1) = 1.0;
    grad_f_X(2, 2) = 1.0;

    // Initialize input Jacobian [3x2]
    grad_f_q.Fill(0.0);

    // Initialize measurement Jacobian [2x3]
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
// UPDATE STEP
// =============================================================================
void EKF::updateEKF(int nBeacon) {
    Z(0) = BeaconCluster[nBeacon].dist;
    Z(1) = BeaconCluster[nBeacon].angle;

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

// =============================================================================
// BEACON ASSOCIATION AND VALIDATION - FOR YDLIDAR X4
// =============================================================================
// Configuration constants - adjust these for your setup
#define LIDAR_TO_ROBOT_OFFSET 0.005f   // Distance from LIDAR center to robot center (m)
#define LIDAR_CCW false                  // true if angles increase counter-clockwise
#define LIDAR_ANGLE_OFFSET 0            // Add offset if 0° is not forward (e.g., 180 if mounted backwards)
#define ASSOCIATION_THRESHOLD 0.20     // Max distance to associate point with beacon (m)
#define BEACON_RADIUS_OFFSET 0.042       // Offset for beacon center (m)

void EKF::phaseAV() {
    // Reset all beacon clusters
    for (int j = 0; j < NBEACONS; j++) {
        BeaconCluster[j].x = 0.0;
        BeaconCluster[j].y = 0.0;
        BeaconCluster[j].n = 0;
        BeaconCluster[j].dist = 0.0;
        BeaconCluster[j].angle = 0.0;
    }
    
    // For each known beacon, search for corresponding LIDAR points
    for (int j = 0; j < NBEACONS; j++) {
        // Compute expected angle to beacon in robot frame
        double dx = BeaconPos[j].x - XR(0) - LIDAR_TO_ROBOT_OFFSET * cos(XR(2));
        double dy = BeaconPos[j].y - XR(1) - LIDAR_TO_ROBOT_OFFSET * sin(XR(2));
        
        // Expected angle in robot frame (radians, -PI to PI)
        double expectedAngle = normalizeAngle(atan2(dy, dx) - XR(2));
        
        // Convert to LIDAR index (0-719, at 0.5 deg per bin)
        int idx_beacon_half = (int)round(expectedAngle * 180.0 / M_PI * 2.0);
        
        // Apply LIDAR angle convention
        if (LIDAR_CCW) {
            idx_beacon = idx_beacon_half;
        } else {
            idx_beacon = -idx_beacon_half;
        }
        
        // Add any mounting offset (converted to 0.5 deg units)
        idx_beacon += LIDAR_ANGLE_OFFSET * 2;
        
        // Normalize to 0-719 range
        while (idx_beacon < 0) idx_beacon += 720;
        idx_beacon = idx_beacon % 720;
        
        // Search window around expected angle
        BeaconCluster[j].firstRay = idx_beacon - deltaRay;
        if (BeaconCluster[j].firstRay < 0) {
            BeaconCluster[j].firstRay += 720;
        }

        // Search through LIDAR rays in window
        for (int i = 0; i < 2 * deltaRay; i++) {
            idx = (BeaconCluster[j].firstRay + i) % 720;
            
            MeasureDist = LaserValues(0, idx);  // Distance in meters
            
            // Skip invalid measurements
            // YDLidar X4 range: 0.12m to 10m
            if (MeasureDist < 0.03 || MeasureDist > 2.5) { 
                continue;
            }
            
            // Convert LIDAR index back to angle in radians (0.5 deg per bin)
            double rayAngleDeg = (double)idx * 0.5;
            double rayAngleRad;
            
            if (LIDAR_CCW) {
                rayAngleRad = (rayAngleDeg - LIDAR_ANGLE_OFFSET) * M_PI / 180.0;
            } else {
                rayAngleRad = -(rayAngleDeg - LIDAR_ANGLE_OFFSET) * M_PI / 180.0;
            }
            
            // Convert LIDAR point to world coordinates
            MeasurePos.x = MeasureDist * cos(rayAngleRad + XR(2)) + XR(0) + LIDAR_TO_ROBOT_OFFSET * cos(XR(2));
            MeasurePos.y = MeasureDist * sin(rayAngleRad + XR(2)) + XR(1) + LIDAR_TO_ROBOT_OFFSET * sin(XR(2));
            
            // Check if this point is close to the expected beacon position
            double distToBeacon = dist(BeaconPos[j].x - MeasurePos.x, BeaconPos[j].y - MeasurePos.y);
            
            if (distToBeacon < ASSOCIATION_THRESHOLD) {
                // Update cluster with running mean
                BeaconCluster[j].n++;
                double n = BeaconCluster[j].n;
                BeaconCluster[j].x = BeaconCluster[j].x * (n - 1) / n + MeasurePos.x / n;
                BeaconCluster[j].y = BeaconCluster[j].y * (n - 1) / n + MeasurePos.y / n;
            }
        }

        // Compute final distance and angle for detected beacons
        if (BeaconCluster[j].n >= 1) {
            double cluster_dx = BeaconCluster[j].x - XR(0);
            double cluster_dy = BeaconCluster[j].y - XR(1);
            
            BeaconCluster[j].angle = normalizeAngle(atan2(cluster_dy, cluster_dx) - XR(2));
            BeaconCluster[j].dist = sqrt(cluster_dx * cluster_dx + cluster_dy * cluster_dy) + BEACON_RADIUS_OFFSET;
            BeaconCluster[j].x += BEACON_RADIUS_OFFSET * cos(atan2(cluster_dy, cluster_dx));
            BeaconCluster[j].y += BEACON_RADIUS_OFFSET * sin(atan2(cluster_dy, cluster_dx));
        }
    }
}

void EKF::motionmodelEKF() {
    for (int j = 0; j < NBEACONS; j++) {
        if (BeaconCluster[j].n >= MIN_BEACON_POINTS) {
            updateEKF(j);
        }
    }
}