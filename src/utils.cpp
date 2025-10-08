#include "utils.h"
#include <math.h>

double dist(double x, double y) {
    return sqrt(pow(x, 2) + pow(y, 2));
};
double normalizeAngle(double angle) {
    angle = fmod(angle, 2*M_PI);   // Wrap within (-360, 360)
    if (angle < 0)
        angle += 2*M_PI;           // Shift negative angles into [0, 360)
    return angle;
};