#include "state_machines.h"
#include <Arduino.h>

state_machine_t::state_machine_t()
{
  
}

void state_machine_t::set_new_state(int astate)
{
  new_state = astate;
}


void state_machine_t::update_state(void)
{
  if (state != new_state) {  // if the state changed 'tis' is reset
    prev_state = state;
    state = new_state;
    tes_ms = millis();
    tis_ms = 0;
    tis = 0;
    actions_count = 0;
  }
}

void state_machine_t::calc_next_state(void)
{
  tis_ms = millis() - tes_ms;
  tis = 1e-3 * tis_ms;
  next_state_rules();
}


float state_machine_t::time_since(uint32_t when)
{
  int delta = millis() - when;
  return 1e-3 * delta;
}


void state_machine_t::do_enter_state_actions(void)
{
  enter_state_actions_rules();
}

void state_machine_t::do_state_actions(void)
{
  state_actions_rules();
  actions_count++;
}


// state_machines_t: TODO: test

int state_machines_t::register_state_machine(pstate_machine_t new_psm)
{
  if (count >= MAX_STATE_MACHINES) return -1;
  psm[count] = new_psm;
  count++;
  return count - 1;
}  
  
void state_machines_t::calc_next_states(void)
{
  int i;
  uint32_t cur_time = millis();
  for (i = 0; i < count; i++) {
    psm[i]->tis_ms = cur_time - psm[i]->tes_ms;
    psm[i]->tis = 1e-3 * psm[i]->tis_ms;
    psm[i]->next_state_rules();
  }   
}

void state_machines_t::update_states(void)
{
  int i;
  for (i = 0; i < count; i++) {
    psm[i]->update_state();
  }   
}

void state_machines_t::do_enter_states_actions(void)
{
  int i;
  for (i = 0; i < count; i++) {
    psm[i]->do_enter_state_actions();
  }
}

void state_machines_t::do_states_actions(void)
{
  int i;
  for (i = 0; i < count; i++) {
    psm[i]->state_actions_rules();
    psm[i]->actions_count++;
  }
}
  


void state_machines_t::step(void)
{
  calc_next_states();
  update_states();

  do_enter_states_actions();
  do_states_actions();
}


state_machines_t state_machines;
