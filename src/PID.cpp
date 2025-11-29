#include "Arduino.h"
#include "PID.h"

PID_t::PID_t()
{
  ppars = &pars;  // Use the internal pars
  
  // Some typical values
  pars.Kfd = 0;
  pars.Kf = 0.4;
  pars.Kc = 0.7;
  pars.Ki = 1.5;
  pars.Kd = 0;
  pars.dt = 0.06;
  pars.dead_zone = 0.2;

  m_max = 12;
  m_min = -12;

  Se = 0;
  e = 0;
  last_e = 0;
  y = 0;
  y_ref = 0;
}

void PID_t::init_pars(PID_pars_t* appars)
{
  ppars = appars;
}

float PID_t::calc(float new_y_ref, float new_y)
{
  float de, dy_ref;
  y = new_y;

  // Apply reference filter to cancel PI zero
  // Filter: y_ref_filt(k) = a0 * y_ref_filt(k-1) + b0 * y_ref(k)
  // where a0 = exp(-dt/Ti) and b0 = 1 - a0
  float a0 = exp(-ppars->dt / (ppars->Kc/ppars->Ki));
  float b0 = 1.0f - a0;
  y_ref_filt = a0 * y_ref_filt + b0 * new_y_ref;
  
  // dy_ref = (y_ref_filt - y_ref) / ppars->dt;
  // y_ref = y_ref_filt;

  dy_ref = (new_y_ref - y_ref) / ppars->dt;
  y_ref = new_y_ref;

  last_e = e;
  e = y_ref - y;
  
  // Integral and derivative of the error
  Se += e * ppars->dt;
  de = (e - last_e) / ppars->dt;

  // Limit the Integral to the case where it would saturate the output only with this component
  float KiSe = ppars->Ki * Se;
  if (KiSe > m_max) Se = m_max / ppars->Ki;
  if (KiSe < m_min) Se = m_min / ppars->Ki;
  
  // Calc PID output
  m = ppars->Kc * e + ppars->Ki * Se + ppars->Kd * de + ppars->Kf * y_ref + ppars->Kfd * dy_ref;
 
  // Anti windup
  //if (m > m_max || m < m_min) {
  if ((m > m_max && e > 0) || (m < m_min && e < 0)) {
    // undo integration
    Se -= e * ppars->dt;
  }
  
  // Saturate the output
  if (m > m_max) {
    m = m_max;
  } else if (m < m_min) {
    m = m_min;
  }

  return m;
}
