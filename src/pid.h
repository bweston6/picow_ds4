#include "pico/types.h"

#ifndef PID_H
#define PID_H

struct pid {
  float k_p;
  float k_i;
  float k_d;
  absolute_time_t last_update;
  float error_sum;
  float last_error;
  float last_input;
};

#endif
