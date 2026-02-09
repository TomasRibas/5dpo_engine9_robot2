#include <ekf.h>
#include "utils.h"

EKF ekf;

int idx_beacon, idx;
double MeasureDist;
TPos MeasurePos;

EKF::EKF() {

    dt = 0.04;
    
    // LaserValues.Fill(0.0);
    for (int i = 0; i < 720; i++) {
        LaserValues[i] = -1.0;
    }

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
    XR(0) = -0.785;
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
#define ASSOCIATION_THRESHOLD 0.10     // Max distance to associate point with beacon (m)
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

    
    for (int j = 0; j < NBEACONS; j++) {
        double dx = BeaconPos[j].x - XR(0) - LIDAR_TO_ROBOT_OFFSET * cos(XR(2));
        double dy = BeaconPos[j].y - XR(1) - LIDAR_TO_ROBOT_OFFSET * sin(XR(2));
        double expectedAngle = normalizeAngle(atan2(dy, dx) - XR(2));
        
        int idx_beacon_half = (int)round(expectedAngle * 180.0 / M_PI * 2.0);
        
        if (LIDAR_CCW) {
            idx_beacon = idx_beacon_half;
        } else {
            idx_beacon = -idx_beacon_half;
        }
        
        BeaconCluster[j].firstRay = idx_beacon - deltaRay;
        if (BeaconCluster[j].firstRay < 0) {
            BeaconCluster[j].firstRay += 720;
        }

        for (int i = 0; i < 2 * deltaRay; i++) {
            idx = (BeaconCluster[j].firstRay + i) % 720;
            
            // MeasureDist = LaserValues(0, idx); 
            MeasureDist = LaserValues[idx];
            
            
            if (MeasureDist < 0.03 || MeasureDist > 2.5) { 
                continue;
            }
            
            double rayAngleDeg = (double)idx * 0.5;
            double rayAngleRad;
            
            if (LIDAR_CCW) {
                rayAngleRad = (rayAngleDeg - LIDAR_ANGLE_OFFSET) * M_PI / 180.0;
            } else {
                rayAngleRad = -(rayAngleDeg - LIDAR_ANGLE_OFFSET) * M_PI / 180.0;
            }
            
            MeasurePos.x = MeasureDist * cos(rayAngleRad + XR(2)) + XR(0) + LIDAR_TO_ROBOT_OFFSET * cos(XR(2));
            MeasurePos.y = MeasureDist * sin(rayAngleRad + XR(2)) + XR(1) + LIDAR_TO_ROBOT_OFFSET * sin(XR(2));
            
            double distToBeacon = dist(BeaconPos[j].x - MeasurePos.x, BeaconPos[j].y - MeasurePos.y);
            
            if (distToBeacon < ASSOCIATION_THRESHOLD) {
                
                BeaconCluster[j].n++;
                double n = BeaconCluster[j].n;
                BeaconCluster[j].x = BeaconCluster[j].x * (n - 1) / n + MeasurePos.x / n;
                BeaconCluster[j].y = BeaconCluster[j].y * (n - 1) / n + MeasurePos.y / n;

                // Serial.print("Cluster_x: ");
                // Serial.print(BeaconCluster[j].x );
                // Serial.print(" Cluster_y: ");
                // Serial.println(BeaconCluster[j].y);
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

void EKF::setScan(const float* ang, const float* dist, uint16_t n) {
    // Reset all bins
    for (int i = 0; i < 720; i++) {
        LaserValues[i] = -1.0;
    }

    for (uint16_t i = 0; i < n; i++) {
        float distance = dist[i];
        float angleRad = ang[i];  // Already normalized to [-π, +π]
        float angleDeg = angleRad * 180.0f / M_PI;

        //Serial.print("Scan point "); Serial.print(i); Serial.print(": angleDeg="); Serial.print(angleDeg, 2); Serial.print(" dist="); Serial.println(distance, 3);
            
        

        int bin = (int)lroundf(-angleDeg * 2.0f);     
        bin = bin % 720;     

        LaserValues[bin] = distance;

        //Serial.print("Mapped to bin "); Serial.print(bin); Serial.print(": angleDeg="); Serial.print(-bin * 0.5f, 2); Serial.print(" dist="); Serial.println(distance, 3);
    }

    // Serial.println("---- LaserValues dump: idx, angleDeg, dist ----");
    // for (int idx = 0; idx < 720; idx++) {
    //     Serial.print("idx=");
    //     Serial.print(idx);
    //     Serial.print(" dist=");
    //     Serial.println(LaserValues[idx], 3);          // -1.000 means empty
    // }

}  

