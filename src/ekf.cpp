#include <ekf.h>
#include "utils.h"


int idx_beacon, idx;
double MeasureDist;
TPos MeasurePos;
double Dist_Treshold = 0.15; //meters

#define LIDAR_ANGLE_OFFSET 0

EKF::EKF(){
    //dt
    dt = 0.04; //40 ms
    //Lidar rate 1.8kHz -> 0.555 ms
    LaserValues.Fill(0.0);

    //identity matrix
    I.Fill(0.0);
    I(0, 0) = 1.0;
    I(1, 1) = 1.0;
    I(2, 2) = 1.0;

    //init P,Q,R
    P.Fill(0.0);
    P(0, 0) = 1E-2; //initial uncertainty x
    P(1, 1) = 1E-2; //initial uncertainty y
    P(2, 2) = 1E-2; //initial uncertainty theta

    Q.Fill(0.0);
    // Q(0, 0) = pow(0.0005, 2); //process noise vlin
    // Q(1, 1) = pow(0.0005, 2); //process noise omega
    Q(0,0) = pow(0.0005, 2);
    Q(1,1) = pow(0.0005, 2);


    R.Fill(0.0);
<<<<<<< HEAD
    R(0, 0) = pow(0.035, 2); //measurement noise distance
    R(1, 1) = pow(0.01745, 2); //measurement noise angle
=======
    // R(0, 0) = pow(0.1, 2); //measurement noise distance
    // R(1, 1) = pow(0.1, 2); //measurement noise angle
    R(0,0) = pow(0.05, 2);   // Increase from 0.005 to 0.05 (5cm instead of 5mm)
    R(1,1) = pow(0.05, 2);   // Increase from 0.009 to 0.05 (about 3 degrees)
>>>>>>> 894523e1cdd62271f82a7fb7df0718b8148749ff

    //pista de  1.7 x 1.2 
    //set Beacon positions
<<<<<<< HEAD
    BeaconPos[0].x = 0; BeaconPos[0].y = 0;
    BeaconPos[1].x = 0; BeaconPos[1].y = 1.19;
    BeaconPos[2].x = 1.56; BeaconPos[2].y = 1.19;
    BeaconPos[3].x = 1.56; BeaconPos[3].y = 0;
=======
    // BeaconPos[0].x = 0; BeaconPos[0].y = 0;
    // BeaconPos[1].x = 0; BeaconPos[1].y = 1.27;
    // BeaconPos[2].x = 1.78; BeaconPos[2].y = 1.27;
    // BeaconPos[3].x = 1.78; BeaconPos[3].y = 0;
    //NOVAS COORDENADAS BEACONS
    BeaconPos[0].x = -0.8985; BeaconPos[0].y = -0.6485;
    BeaconPos[1].x = -0.8985; BeaconPos[1].y = 0.6485;
    BeaconPos[2].x = 0.8985; BeaconPos[2].y = 0.6485;
    BeaconPos[3].x = 0.8985; BeaconPos[3].y = -0.6485;

>>>>>>> 894523e1cdd62271f82a7fb7df0718b8148749ff

    //init BeaconCluster
    for(int j=0; j<NBEACONS; j++){
        BeaconCluster[j].x = 0;
        BeaconCluster[j].y = 0;
        BeaconCluster[j].n = 0;
        BeaconCluster[j].dist = 0;
        BeaconCluster[j].angle = 0;
        BeaconCluster[j].firstRay = 0;
    }

    //set initial state
<<<<<<< HEAD
    XR(0) = 0.125;
    XR(1) = 0;
=======
    XR(0) = -0.785;
    XR(1) = -0.57;
>>>>>>> 894523e1cdd62271f82a7fb7df0718b8148749ff
    XR(2) = 1.57;

    //set df//dX
    grad_f_X.Fill(0.0);
    grad_f_X(0, 0) = 1.0;
    grad_f_X(1, 1) = 1.0;
    grad_f_X(2, 2) = 1.0;

    //set df/dU
    grad_f_q.Fill(0.0);
    grad_f_q(2, 1) = 1.0;

    //set dh/dX
    grad_h_X.Fill(0.0);
    grad_h_X(1, 2) = 1.0;
}

void EKF::predict(double vlin, double omega, double dt){
    //Update XR, motion model
    XR(0) = XR(0) + vlin * dt * cos(XR(2) + 0.5*omega*dt);
    XR(1) = XR(1) + vlin * dt * sin(XR(2) + 0.5*omega*dt);
    XR(2) = normalizeAngle(XR(2) + omega * dt);

    //df/dX, updateJacState, XR vlin omega dt
    updateJacState(vlin, omega, dt);

    //df/dU, updateJacInput, XR vlin omega dt
    updateJacInput(vlin, omega, dt);

    //Update P, covariancePropagation, grad_f_X, grad_f_q, P, Q
    covariancePropagation();
    
}


void EKF::updateXR(double vlin, double omega, double dt){
    //Update XR, motion model
    XR(0) = XR(0) + vlin * dt * cos(XR(2) + 0.5*omega*dt); //inverted
    XR(1) = XR(1) + vlin * dt * sin(XR(2) + 0.5*omega*dt);
    XR(2) = normalizeAngle(XR(2) + omega * dt);
};
void EKF::updateJacState(double vlin, double omega, double dt){
    grad_f_X(0, 2) = -vlin * dt * sin(XR(2) + ((omega * dt )/ 2));
    grad_f_X(1, 2) =  vlin * dt * cos(XR(2) + ((omega * dt )/ 2));
};
void EKF::updateJacInput(double vlin, double omega, double dt){
    grad_f_q(0, 0) = cos(XR(2) + 0.5*omega*dt);
    grad_f_q(0, 1) = -0.5*vlin*dt*sin(XR(2) + 0.5*omega*dt);
    grad_f_q(1, 0) = sin(XR(2) + 0.5*omega*dt);
    grad_f_q(1, 1) = 0.5*vlin*dt*cos(XR(2) + 0.5*omega*dt);
    grad_f_q(2, 0) = 0;
    grad_f_q(2, 1) = 1;
};
void EKF::covariancePropagation(){
    P = grad_f_X * P;
    P = P * (~grad_f_X);
    P = P + grad_f_q * Q * (~grad_f_q);
};

void EKF::updateEKF(int nBeacon){
    //dh/dX, Kalman gain, P covariance update, update state with measurement Z
    //update state with measurement Z
    Z(0) = BeaconCluster[nBeacon].dist; //measured distance to beacon
    Z(1) = BeaconCluster[nBeacon].angle; //measured angle to beacon

    //expected measurement Z_E - in this case it's already the difference (Z-Z_E)
    double dBeacon;
    dBeacon = dist(BeaconPos[nBeacon].x - XR(0), 
                    BeaconPos[nBeacon].y - XR(1));
    //BeaconCluster[nBeacon].dist
    Z_E(0) = Z(0) - dBeacon; //expected distance to beacon Z(0) - dbeacon
    Z_E(1) = normalizeAngle(Z(1) - 
        normalizeAngle(atan2(BeaconPos[nBeacon].y - XR(1), BeaconPos[nBeacon].x - XR(0)) - XR(2))); 
    Serial.print(" Ze_Dist: "); Serial.print(Z_E(0));
    Serial.print("  Ze_Angle: "); Serial.println((Z_E(1)*180)/PI);
      
    //normalizeAngle(BeaconCluster[nBeacon].angle - 
        //normalizeAngle(atan2(BeaconPos[nBeacon].y - XR(1), BeaconPos[nBeacon].x - XR(0)) - XR(2))); //expected angle to beacon
        //Z(1) - (atan2(...) - XR(2))
    
    //expected measurement Z_E
    //XR=XR+K(Z-Z_E)
    //update x,y,theta
    grad_h_X(0, 0) = -(BeaconPos[nBeacon].x - XR(0)) / dBeacon;
    grad_h_X(0, 1) = -(BeaconPos[nBeacon].y - XR(1)) / dBeacon;
    grad_h_X(0, 2) = 0;
    grad_h_X(1, 0) = (BeaconPos[nBeacon].y - XR(1)) / pow(dBeacon, 2);
    grad_h_X(1, 1) = -(BeaconPos[nBeacon].x - XR(0)) / pow(dBeacon, 2);
    grad_h_X(1, 2) = -1;

    //Kalman gain
    // K = grad_h_X * P;
    // K = K * (~grad_h_X);
    // K = K + R;
    // K = Inverse(K);
    // K = P * (~grad_h_X) * K;

    // //P covariance update
    // P = (I - K * grad_h_X) * P;

    // XR = XR + K * (Z_E);
    //Update x,y,theta with odom
    S = grad_h_X * P * (~grad_h_X) + R;
    
    // Invert innovation covariance
    S = Inverse(S);
    
    // Compute Kalman gain K = P * H^T * S^-1
    K = P * (~grad_h_X) * S;

    //P covariance update
    P = (I - K * grad_h_X) * P;


    XR = XR + K * (Z_E);
    normalizeAngle(XR(2));
};


void EKF::phaseAV(){
    //Association and validation
    float dist_ldr2rbt = 0.005;
    for(int j=0; j < NBEACONS; j++){
        //BeaconCluster[j].x = 0;
        //BeaconCluster[j].y = 0;
        BeaconCluster[j].n = 0;
<<<<<<< HEAD
        idx_beacon = round((normalizeAngle(atan2(BeaconPos[j].y - XR(1) - dist_ldr2rbt*sin(XR(2)), 
                BeaconPos[j].x - XR(0) - dist_ldr2rbt*cos(XR(2))) - XR(2)))
=======
        idx_beacon = round((normalizeAngle(atan2(BeaconPos[j].y - XR(1) - 0.01*sin(XR(2)), 
                BeaconPos[j].x - XR(0) - 0.01*cos(XR(2))) - XR(2)))
>>>>>>> 894523e1cdd62271f82a7fb7df0718b8148749ff
                    /(M_PI/180)); //angle to beacon in robot frame
                    //might not need + M_PI
        BeaconCluster[j].firstRay = idx_beacon - deltaRay;
        if(BeaconCluster[j].firstRay < 0) BeaconCluster[j].firstRay += 360;
        //if(BeaconCluster[j].firstRay > 360) BeaconCluster[j].firstRay -= 360;

        for(int i = 0; i < 2*deltaRay; i++){
            idx = BeaconCluster[j].firstRay + i; //doesn't need -1??
            if(idx >= 360) idx -= 360;
            //has to have access to full LIDAR scan
            MeasureDist = LaserValues(0, idx); //get LIDAR measurement
            Serial.print("First Ray: "); Serial.print(BeaconCluster[j].firstRay);
            Serial.print(" LIDAR idx: "); Serial.print(idx);
            Serial.print(" Dist: "); Serial.println(MeasureDist);
            if(MeasureDist > 0){
<<<<<<< HEAD
                MeasurePos.x = MeasureDist*cos((idx)*M_PI/180 +  XR(2)) + XR(0) + dist_ldr2rbt*cos(XR(2) );//was -0.085
                MeasurePos.y = MeasureDist*sin((idx)*M_PI/180 +  XR(2)) + XR(1) + dist_ldr2rbt*sin(XR(2) );//was -0.085
=======
                MeasurePos.x = MeasureDist*cos((idx + LIDAR_ANGLE_OFFSET)*M_PI/180 + XR(2)) + XR(0) + 0.01*cos(XR(2));
                MeasurePos.y = MeasureDist*sin((idx + LIDAR_ANGLE_OFFSET)*M_PI/180 + XR(2)) + XR(1) + 0.01*sin(XR(2));
>>>>>>> 894523e1cdd62271f82a7fb7df0718b8148749ff
                float d = dist(BeaconPos[j].x - MeasurePos.x, BeaconPos[j].y - MeasurePos.y);
                Serial.print("Threshold Distance: "); Serial.println(d);
                if(dist(BeaconPos[j].x - MeasurePos.x, BeaconPos[j].y - MeasurePos.y) < Dist_Treshold){ //Adjust threshold
                    Serial.print(" Dist_X: "); Serial.println(MeasurePos.x);
                    Serial.print(" Dist_Y: "); Serial.println(MeasurePos.y);
                    BeaconCluster[j].n++;
                    BeaconCluster[j].x = ((BeaconCluster[j].x * (BeaconCluster[j].n - 1)) + MeasurePos.x) / BeaconCluster[j].n; //update mean
                    BeaconCluster[j].y = ((BeaconCluster[j].y * (BeaconCluster[j].n - 1)) + MeasurePos.y) / BeaconCluster[j].n; //update mean
                    Serial.print("BeaconCluster_N: "); Serial.println(BeaconCluster[j].n);
                    Serial.print("BeaconCluster_X: "); Serial.println(BeaconCluster[j].x);
                    Serial.print("BeaconCluster_Y: "); Serial.println(BeaconCluster[j].y);
                    
                }
            }
        }
        //Calculate measured distance and angle for the beacon
        BeaconCluster[j].dist = dist(BeaconCluster[j].x - XR(0),
                                            BeaconCluster[j].y - XR(1));
        BeaconCluster[j].angle = normalizeAngle(atan2(BeaconCluster[j].y - XR(1),
                                            BeaconCluster[j].x - XR(0)) - XR(2));
        //BeaconCluster[j].angle = (BeaconCluster[j].angle * 180 / M_PI); //in degrees
    }
};

void EKF::motionmodelEKF(){
    for(int j=0; j<NBEACONS; j++){
        if(BeaconCluster[j].n > 0){
            //predict(vlin, omega, dt);
<<<<<<< HEAD
            //updateEKF(j);
=======
            if(BeaconCluster[j].n > 2){ // Require at least 3 measurements NOVO
                updateEKF(j);
            }
>>>>>>> 894523e1cdd62271f82a7fb7df0718b8148749ff
        }
    }
    
};