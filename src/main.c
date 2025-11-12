// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2023 Brian Starkey <stark3y@gmail.com>

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"

#include "bt_hid.h"
#include "hid_state_queue_config.h"
#include "quadrature_encoder.pio.h"

// aim for frequency of 100Hz
#define WRAP_VALUE 2549

#define STEPS_PER_REV 200
#define MAX_VELOCITY 30 // steps/20ms
#define KP 0.1f

#define FRONT_LEFT_SM 0
#define FRONT_RIGHT_SM 1
#define REAR_LEFT_SM 2
#define REAR_RIGHT_SM 3

queue_t hid_state_queue;

struct pwm {
  unsigned int slice_num;
  unsigned int channel;
};

struct motor {
  int encoder_steps;
  float encoder_rpm;
  struct pwm forward_pwm;
  struct pwm reverse_pwm;
  unsigned int encoder_base_gpio;
  unsigned int forward_gpio;
  unsigned int reverse_gpio;
};

struct chassis {
  struct motor front_left;
  struct motor front_right;
  struct motor rear_left;
  struct motor rear_right;
  PIO encoder_pio;
};

void init_pwm(struct pwm *pwm, uint8_t gpio_pin) {
  gpio_set_function(gpio_pin, GPIO_FUNC_PWM);

  pwm->slice_num = pwm_gpio_to_slice_num(gpio_pin);
  pwm->channel = pwm_gpio_to_channel(gpio_pin);

  pwm_config config = pwm_get_default_config();

  pwm_config_set_wrap(&config, WRAP_VALUE);

  pwm_init(pwm->slice_num, &config, true);
}

void chassis_init(struct chassis *chassis) {
  pio_add_program(chassis->encoder_pio, &quadrature_encoder_program);
  quadrature_encoder_program_init(chassis->encoder_pio, FRONT_LEFT_SM,
                                  chassis->front_left.encoder_base_gpio, 0);
  quadrature_encoder_program_init(chassis->encoder_pio, FRONT_RIGHT_SM,
                                  chassis->front_right.encoder_base_gpio, 0);
  quadrature_encoder_program_init(chassis->encoder_pio, REAR_LEFT_SM,
                                  chassis->rear_left.encoder_base_gpio, 0);
  quadrature_encoder_program_init(chassis->encoder_pio, REAR_RIGHT_SM,
                                  chassis->rear_right.encoder_base_gpio, 0);

  init_pwm(&chassis->front_left.forward_pwm, chassis->front_left.forward_gpio);
  init_pwm(&chassis->front_left.reverse_pwm, chassis->front_left.reverse_gpio);

  init_pwm(&chassis->front_right.forward_pwm,
           chassis->front_right.forward_gpio);
  init_pwm(&chassis->front_right.reverse_pwm,
           chassis->front_right.reverse_gpio);

  init_pwm(&chassis->rear_left.forward_pwm, chassis->rear_left.forward_gpio);
  init_pwm(&chassis->rear_left.reverse_pwm, chassis->rear_left.reverse_gpio);

  init_pwm(&chassis->rear_right.forward_pwm, chassis->rear_right.forward_gpio);
  init_pwm(&chassis->rear_right.reverse_pwm, chassis->rear_right.reverse_gpio);
}

uint16_t motor_level(float x) { return (uint16_t)(x * 2550.0f); }

float clamp1(float x) {
  if (x > 1.0f) {
    return 1.0f;
  }
  if (x < -1.0f) {
    return -1.0f;
  }
  return x;
}

void motor_set(struct motor *motor, float percent) {
  percent = clamp1(percent);
  uint16_t level = 0;

  if (percent != 0.0f) {
    level = motor_level(fabs(percent));
  }

  if (!signbit(percent)) {
    pwm_set_chan_level(motor->forward_pwm.slice_num, motor->forward_pwm.channel,
                       level);
    pwm_set_chan_level(motor->reverse_pwm.slice_num, motor->reverse_pwm.channel,
                       0);
  } else {
    pwm_set_chan_level(motor->forward_pwm.slice_num, motor->forward_pwm.channel,
                       0);
    pwm_set_chan_level(motor->reverse_pwm.slice_num, motor->reverse_pwm.channel,
                       level);
  }
}

void chassis_set(struct chassis *chassis, float front_left, float front_right,
                 float rear_left, float rear_right) {

  motor_set(&chassis->front_left, front_left);
  motor_set(&chassis->front_right, front_right);
  motor_set(&chassis->rear_left, rear_left);
  motor_set(&chassis->rear_right, rear_right);
}

absolute_time_t last_update = 0;

void updateEncoderValues(struct chassis *chassis) {
  int front_left_displacement =
      quadrature_encoder_get_count(chassis->encoder_pio, FRONT_LEFT_SM) *
      -1; // front encoders are backwards
  int front_right_displacement =
      quadrature_encoder_get_count(chassis->encoder_pio, FRONT_RIGHT_SM) * -1;
  int rear_left_displacement =
      quadrature_encoder_get_count(chassis->encoder_pio, REAR_LEFT_SM);
  int rear_right_displacement =
      quadrature_encoder_get_count(chassis->encoder_pio, REAR_RIGHT_SM);

  absolute_time_t now = get_absolute_time();
  int64_t us_since_last_update = absolute_time_diff_us(last_update, now);
  last_update = now;

  float mins_since_last_update = (float)us_since_last_update / 6e+7;

  chassis->front_left.encoder_rpm =
      ((float)(front_left_displacement - chassis->front_left.encoder_steps) /
       STEPS_PER_REV) /
      mins_since_last_update;
  chassis->front_right.encoder_rpm =
      ((float)(front_right_displacement - chassis->front_right.encoder_steps) /
       STEPS_PER_REV) /
      mins_since_last_update;
  chassis->rear_left.encoder_rpm =
      ((float)(rear_left_displacement - chassis->rear_left.encoder_steps) /
       STEPS_PER_REV) /
      mins_since_last_update;
  chassis->rear_right.encoder_rpm =
      ((float)(rear_right_displacement - chassis->rear_right.encoder_steps) /
       STEPS_PER_REV) /
      mins_since_last_update;

  chassis->front_left.encoder_steps = front_left_displacement;
  chassis->front_right.encoder_steps = front_right_displacement;
  chassis->rear_left.encoder_steps = rear_left_displacement;
  chassis->rear_right.encoder_steps = rear_right_displacement;
}

int main(void) {
  stdio_init_all();

  queue_init(&hid_state_queue, QUEUE_ELEMENT_SIZE, QUEUE_DEPTH);
  multicore_launch_core1(bt_main);

  struct motor front_left_motor = {
      .forward_gpio = 26, .reverse_gpio = 27, .encoder_base_gpio = 20};
  struct motor front_right_motor = {
      .forward_gpio = 17, .reverse_gpio = 16, .encoder_base_gpio = 18};
  struct motor rear_left_motor = {
      .forward_gpio = 2, .reverse_gpio = 3, .encoder_base_gpio = 0};
  struct motor rear_right_motor = {
      .forward_gpio = 15, .reverse_gpio = 14, .encoder_base_gpio = 12};

  struct chassis chassis = {.front_left = front_left_motor,
                            .front_right = front_right_motor,
                            .rear_left = rear_left_motor,
                            .rear_right = rear_right_motor,
                            .encoder_pio = pio0};

  chassis_init(&chassis);

  struct bt_hid_state hid_state;

  float max_rpm = 1000.0f;
  float front_left_last = 0.0f;
  float front_right_last = 0.0f;
  float rear_left_last = 0.0f;
  float rear_right_last = 0.0f;
  float k_p = 0.0001f;
  float deadzone = 0.05f;

  float throttle, steering, left_target, right_target, front_left, front_right,
      rear_left, rear_right;

  while (1) {
    // get controller input
    queue_try_remove(&hid_state_queue, &hid_state);
    // get encoder input
    updateEncoderValues(&chassis);

    throttle = (float)hid_state.r2 / 255.0f - (float)hid_state.l2 / 255.0f;
    steering = ((float)hid_state.lx - 127.5f) / 127.5f;

    if (fabs(steering) < deadzone) {
      steering = 0.0f;
    }

    left_target = clamp1(throttle + steering) * max_rpm;
    right_target = clamp1(throttle - steering) * max_rpm;

    front_left = clamp1(front_left_last +
                        (left_target - chassis.front_left.encoder_rpm) * k_p);
    front_right =
        clamp1(front_right_last +
               (right_target - chassis.front_right.encoder_rpm) * k_p);
    rear_left = clamp1(rear_left_last +
                       (left_target - chassis.rear_left.encoder_rpm) * k_p);
    rear_right = clamp1(rear_right_last +
                        (right_target - chassis.rear_right.encoder_rpm) * k_p);

    chassis_set(&chassis, front_left, front_right, rear_left, rear_right);

    front_left_last = left_target == 0.0f ? 0.0f : front_left;
    front_right_last = right_target == 0.0f ? 0.0f : front_right;
    rear_left_last = left_target == 0.0f ? 0.0f : rear_left;
    rear_right_last = right_target == 0.0f ? 0.0f : rear_right;
  }

  return 1;
}
