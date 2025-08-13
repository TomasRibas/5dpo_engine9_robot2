#ifndef PID_H
#define PID_H

#include "Arduino.h"

typedef struct {
  float Ki, Kc, Kd, Kf, Kfd;
  float dead_zone;
  float dt;
} PID_pars_t;


class PID_t
{
  public:
    PID_pars_t pars;   // Internal pars
    PID_pars_t* ppars; // External pars
    float y, y_ref;
    float e, last_e, Se;
    float m, m_max, m_min;
    uint8_t active;

    PID_t();
    void init_pars(PID_pars_t* appars);
    
    float calc(float new_y_ref, float new_y);
};

#endif // PID_H