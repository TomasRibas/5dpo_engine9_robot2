#ifndef STATE_MACHINES_H
#define STATE_MACHINES_H


#include <stdint.h>

class state_machine_t
{
  public:

  int state, new_state, prev_state;

  // tes - time entering state
  // tis - time in state
  uint32_t tes_ms, tis_ms;
  float tis; 
  uint32_t actions_count;

  state_machine_t();
  void set_new_state(int astate);
  void update_state(void);
  
  float time_since(uint32_t when);
  
  void calc_next_state(void);
  virtual void next_state_rules(void) {};

  void do_enter_state_actions(void);
  virtual void enter_state_actions_rules(void) {};

  void do_state_actions(void);
  virtual void state_actions_rules(void) {};

  void step(void);
};


#ifndef MAX_STATE_MACHINES
#define MAX_STATE_MACHINES 32
#endif 

typedef state_machine_t* pstate_machine_t;

class state_machines_t
{
  public:
  pstate_machine_t psm[MAX_STATE_MACHINES];
  int count;

  int register_state_machine(pstate_machine_t new_psm);

  void calc_next_states(void);
  void update_states(void);

  void do_enter_states_actions(void);
  void do_states_actions(void);

  void step(void);
};


extern state_machines_t state_machines;

#endif // STATE_MACHINES_H
