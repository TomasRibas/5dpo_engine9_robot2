#include "utils.h"
#include <math.h>

double dist(double x, double y) {
    return sqrt(pow(x, 2) + pow(y, 2));
};
double normalizeAngle(double angle) {
       // Wrap within (-180, 180)
    if (angle < -M_PI)
        angle += 2*M_PI;  
    if(angle > M_PI)
        angle -=2*M_PI;     
    return angle;
};