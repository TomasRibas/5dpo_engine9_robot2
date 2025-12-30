#include <ekf.h>
#include "utils.h"

EKF ekf;

int idx_beacon, idx;
double MeasureDist;
TPos MeasurePos;

EKF::EKF() {
    // Time step (40 ms)
    dt = 0.04;
    
    // LIDAR rate ~1.8kHz -> 0.555 ms per sample
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
    R(1, 1) = pow(0.05, 2);   // Measurement noise for angle (rad²)

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
    XR(0) = -0.785;
    XR(1) = -0.58;
    XR(2) = 1.57;   // theta (rad) - facing +Y direction

    // Initialize state transition Jacobian [3x3]
    // df/dX - partial derivatives of motion model w.r.t. state
    grad_f_X.Fill(0.0);
    grad_f_X(0, 0) = 1.0;
    grad_f_X(1, 1) = 1.0;
    grad_f_X(2, 2) = 1.0;
    // grad_f_X(0,2) and grad_f_X(1,2) updated in updateJacState()

    // Initialize input Jacobian [3x2]
    // df/dU - partial derivatives of motion model w.r.t. inputs
    grad_f_q.Fill(0.0);
    // Updated in updateJacInput()

    // Initialize measurement Jacobian [2x3]
    // dh/dX - partial derivatives of measurement model w.r.t. state
    grad_h_X.Fill(0.0);
    // Updated in updateEKF()
}

// =============================================================================
// PREDICTION STEP
// =============================================================================
void EKF::predict(double vlin, double omega, double dt) {
    // 1. Propagate state using motion model
    updateXR(vlin, omega, dt);

    // 2. Update state transition Jacobian
    updateJacState(vlin, omega, dt);

    // 3. Update input Jacobian
    updateJacInput(vlin, omega, dt);

    // 4. Propagate covariance
    covariancePropagation();
}

void EKF::updateXR(double vlin, double omega, double dt) {
    // Motion model: constant velocity model with mid-point integration
    // This uses the angle at the midpoint of the time step for better accuracy
    double theta_mid = XR(2) + 0.5 * omega * dt;
    
    XR(0) = XR(0) + vlin * dt * cos(theta_mid);
    XR(1) = XR(1) + vlin * dt * sin(theta_mid);
    XR(2) = normalizeAngle(XR(2) + omega * dt);
}

void EKF::updateJacState(double vlin, double omega, double dt) {
    // Jacobian of motion model w.r.t. state
    // f(x,y,θ) = [x + v*dt*cos(θ + ω*dt/2)]
    //            [y + v*dt*sin(θ + ω*dt/2)]
    //            [θ + ω*dt                ]
    //
    // df/dθ for x: -v*dt*sin(θ + ω*dt/2)
    // df/dθ for y:  v*dt*cos(θ + ω*dt/2)
    
    double theta_mid = XR(2) + 0.5 * omega * dt;
    
    grad_f_X(0, 2) = -vlin * dt * sin(theta_mid);
    grad_f_X(1, 2) =  vlin * dt * cos(theta_mid);
    // Note: grad_f_X(2,2) = 1 is already set in constructor
}

void EKF::updateJacInput(double vlin, double omega, double dt) {
    // Jacobian of motion model w.r.t. inputs [v, ω]
    // df/dv and df/dω
    
    double theta_mid = XR(2) + 0.5 * omega * dt;
    
    // df/dv
    grad_f_q(0, 0) = dt * cos(theta_mid);
    grad_f_q(1, 0) = dt * sin(theta_mid);
    grad_f_q(2, 0) = 0.0;
    
    // df/dω
    grad_f_q(0, 1) = -0.5 * vlin * dt * dt * sin(theta_mid);
    grad_f_q(1, 1) =  0.5 * vlin * dt * dt * cos(theta_mid);
    grad_f_q(2, 1) = dt;
}

void EKF::covariancePropagation() {
    // P = F * P * F^T + G * Q * G^T
    // where F = grad_f_X, G = grad_f_q
    
    P = grad_f_X * P * (~grad_f_X) + grad_f_q * Q * (~grad_f_q);
}

// =============================================================================
// UPDATE STEP
// =============================================================================
void EKF::updateEKF(int nBeacon) {
    // Get measured distance and angle from beacon cluster
    Z(0) = BeaconCluster[nBeacon].dist;   // Measured distance to beacon
    Z(1) = BeaconCluster[nBeacon].angle;  // Measured angle to beacon (robot frame)

    // Compute expected measurement h(X)
    double dx = BeaconPos[nBeacon].x - XR(0);
    double dy = BeaconPos[nBeacon].y - XR(1);
    double dBeacon = sqrt(dx * dx + dy * dy);
    
    // Avoid division by zero
    if (dBeacon < 0.001) {
        Serial.println("Warning: Beacon too close, skipping update");
        return;
    }
    
    // Expected distance
    double expectedDist = dBeacon;
    
    // Expected angle in robot frame
    // angle_world = atan2(dy, dx) is the angle to beacon in world frame
    // angle_robot = angle_world - robot_theta
    double expectedAngle = normalizeAngle(atan2(dy, dx) - XR(2));
    
    // Compute innovation (measurement residual)
    // innovation = Z - h(X)
    innovation(0) = Z(0) - expectedDist;
    innovation(1) = normalizeAngle(Z(1) - expectedAngle);
    
    // Debug output
    Serial.print("Beacon "); Serial.print(nBeacon);
    Serial.print(" | Z_dist: "); Serial.print(Z(0), 4);
    Serial.print(" exp: "); Serial.print(expectedDist, 4);
    Serial.print(" innov: "); Serial.print(innovation(0), 4);
    Serial.print(" | Z_ang: "); Serial.print(Z(1) * 180.0 / PI, 1);
    Serial.print(" exp: "); Serial.print(expectedAngle * 180.0 / PI, 1);
    Serial.print(" innov: "); Serial.println(innovation(1) * 180.0 / PI, 1);
    
    // Outlier rejection - skip update if innovation is too large
    if (!isInnovationValid(innovation(0), innovation(1))) {
        Serial.println("  -> Innovation too large, skipping update");
        return;
    }

    // Update measurement Jacobian H = dh/dX
    // h(X) = [sqrt((bx-x)² + (by-y)²)           ]  (distance)
    //        [atan2(by-y, bx-x) - theta         ]  (angle in robot frame)
    //
    // dh/dx for distance: -(bx-x)/d = -dx/d
    // dh/dy for distance: -(by-y)/d = -dy/d
    // dh/dθ for distance: 0
    //
    // For angle: let φ = atan2(dy, dx)
    // dφ/dx = -dy / (dx² + dy²) = -dy / d²
    // dφ/dy =  dx / (dx² + dy²) =  dx / d²
    // dh/dθ for angle: -1
    
    double d2 = dBeacon * dBeacon;
    
    // Distance measurement Jacobian (row 0)
    grad_h_X(0, 0) = -dx / dBeacon;
    grad_h_X(0, 1) = -dy / dBeacon;
    grad_h_X(0, 2) = 0.0;
    
    // Angle measurement Jacobian (row 1)
    // FIXED: Correct signs for atan2 derivative
    grad_h_X(1, 0) = dy / d2;   // dφ/dx = -dy/d², but we have (bx-x), so sign flips
    grad_h_X(1, 1) = -dx / d2;  // dφ/dy = dx/d², but we have (by-y), so sign flips
    grad_h_X(1, 2) = -1.0;

    // Compute innovation covariance: S = H * P * H^T + R
    S = grad_h_X * P * (~grad_h_X) + R;
    
    // Compute Kalman gain: K = P * H^T * S^(-1)
    BLA::Matrix<NObs, NObs, double> S_inv = Inverse(S);
    K = P * (~grad_h_X) * S_inv;

    // Update state covariance: P = (I - K * H) * P
    // Using Joseph form for numerical stability would be better, but this is simpler
    P = (I - K * grad_h_X) * P;

    // Update state: X = X + K * innovation
    XR = XR + K * innovation;
    
    // CRITICAL: Normalize angle after update
    XR(2) = normalizeAngle(XR(2));
    
    Serial.print("  -> Updated state: x="); Serial.print(XR(0), 4);
    Serial.print(" y="); Serial.print(XR(1), 4);
    Serial.print(" th="); Serial.println(XR(2) * 180.0 / PI, 1);
}

bool EKF::isInnovationValid(double distInnovation, double angleInnovation) {
    // Reject measurements with innovation too large (likely outliers or misassociations)
    if (fabs(distInnovation) > MAX_DIST_INNOVATION) {
        return false;
    }
    if (fabs(angleInnovation) > MAX_ANGLE_INNOVATION) {
        return false;
    }
    return true;
}

// =============================================================================
// BEACON ASSOCIATION AND VALIDATION
// =============================================================================
void EKF::phaseAV() {
    const float dist_ldr2rbt = 0.005;
    
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
        double dx = BeaconPos[j].x - XR(0) - dist_ldr2rbt * cos(XR(2));
        double dy = BeaconPos[j].y - XR(1) - dist_ldr2rbt * sin(XR(2));
        double expectedAngle = normalizeAngle(atan2(dy, dx) - XR(2));
        
        // Convert angle to LIDAR index (0-359 degrees)
        // LIDAR index 0 = 0 degrees in robot frame
        idx_beacon = (int)round(expectedAngle * 180.0 / M_PI);
        
        // Handle negative angles
        if (idx_beacon < 0) {
            idx_beacon += 360;
        }
        idx_beacon = idx_beacon % 360;
        
        // Search window around expected angle
        BeaconCluster[j].firstRay = idx_beacon - deltaRay;
        if (BeaconCluster[j].firstRay < 0) {
            BeaconCluster[j].firstRay += 360;
        }

        // Search through LIDAR rays in window
        for (int i = 0; i < 2 * deltaRay; i++) {
            idx = (BeaconCluster[j].firstRay + i) % 360;
            
            MeasureDist = LaserValues(0, idx);  // Distance in meters
            
            // Skip invalid measurements
            if (MeasureDist <= 0.01 || MeasureDist > 3) { 
                continue;
            }
            
            // Convert LIDAR point to world coordinates
            double rayAngle = idx * M_PI / 180.0;  // Ray angle in robot frame
            MeasurePos.x = MeasureDist * cos(rayAngle + XR(2)) + XR(0) + dist_ldr2rbt * cos(XR(2));
            MeasurePos.y = MeasureDist * sin(rayAngle + XR(2)) + XR(1) + dist_ldr2rbt * sin(XR(2));
            
            // Check if this point is close to the expected beacon position
            double distToBeacon = dist(BeaconPos[j].x - MeasurePos.x, BeaconPos[j].y - MeasurePos.y);
            
            // Association threshold (adjust based on beacon size and measurement noise)
            const double ASSOCIATION_THRESHOLD = 0.1;//0.2;  // 10 cm
            
            if (distToBeacon < ASSOCIATION_THRESHOLD) {
                // Update cluster with running mean
                BeaconCluster[j].n++;
                double n = BeaconCluster[j].n;
                BeaconCluster[j].x = BeaconCluster[j].x * (n - 1) / n + MeasurePos.x / n;
                BeaconCluster[j].y = BeaconCluster[j].y * (n - 1) / n + MeasurePos.y / n;
            }
        }

        if(BeaconCluster[j].n >=  1) {
            double cluster_dx = BeaconCluster[j].x - XR(0);
            double cluster_dy = BeaconCluster[j].y - XR(1);
            BeaconCluster[j].dist = sqrt(cluster_dx * cluster_dx + cluster_dy * cluster_dy)+ 0.0375; //+ 0.0375; // offset pra centro do poste
            BeaconCluster[j].angle = normalizeAngle(atan2(cluster_dy, cluster_dx) - XR(2));
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