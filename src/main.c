// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2023 Brian Starkey <stark3y@gmail.com>

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "bt_hid.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "hardware/timer.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "quadrature_encoder.pio.h"

#define MIN_MOTOR_VALUE 140
// aim for frequency of 100Hz
#define WRAP_VALUE 2549
#define PERIOD 20 // ms

#define MAX_VELOCITY 30 // steps/20ms
#define KP 0.1f

#define FRONT_LEFT_SM 0
#define FRONT_RIGHT_SM 1
#define REAR_LEFT_SM 2
#define REAR_RIGHT_SM 3

struct pwm {
  unsigned int slice_num;
  unsigned int channel;
};

struct motor {
  int encoder_displacement;
  int encoder_velocity;
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

uint16_t motor_level(float x) { return (uint16_t)(x * 1150.0f + 1400.0f); }

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
  printf("fr: %f, fr: %f, rl: %f, rr: %f\n", front_left, front_right, rear_left,
         rear_right);

  motor_set(&chassis->front_left, front_left);
  motor_set(&chassis->front_right, front_right);
  motor_set(&chassis->rear_left, rear_left);
  motor_set(&chassis->rear_right, rear_right);
}

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

  chassis->front_left.encoder_velocity =
      front_left_displacement - chassis->front_left.encoder_displacement;
  chassis->front_right.encoder_velocity =
      front_right_displacement - chassis->front_right.encoder_displacement;
  chassis->rear_left.encoder_velocity =
      rear_left_displacement - chassis->rear_left.encoder_displacement;
  chassis->rear_right.encoder_velocity =
      rear_right_displacement - chassis->rear_right.encoder_displacement;

  printf("fl: %d, fr: %d, rl: %d, rr: %d", chassis->front_left.encoder_velocity,
         chassis->front_right.encoder_velocity,
         chassis->rear_left.encoder_velocity,
         chassis->rear_right.encoder_velocity);

  chassis->front_left.encoder_displacement = front_left_displacement;
  chassis->front_right.encoder_displacement = front_right_displacement;
  chassis->rear_left.encoder_displacement = rear_left_displacement;
  chassis->rear_right.encoder_displacement = rear_right_displacement;
}

int main(void) {
  stdio_init_all();

  multicore_launch_core1(bt_main);

  struct motor front_left = {
      .forward_gpio = 26, .reverse_gpio = 27, .encoder_base_gpio = 20};
  struct motor front_right = {
      .forward_gpio = 17, .reverse_gpio = 16, .encoder_base_gpio = 18};
  struct motor rear_left = {
      .forward_gpio = 2, .reverse_gpio = 3, .encoder_base_gpio = 0};
  struct motor rear_right = {
      .forward_gpio = 15, .reverse_gpio = 14, .encoder_base_gpio = 12};

  struct chassis chassis = {.front_left = front_left,
                            .front_right = front_right,
                            .rear_left = rear_left,
                            .rear_right = rear_right,
                            .encoder_pio = pio0};

  chassis_init(&chassis);

  struct bt_hid_state state;

  while (1) {
    sleep_ms(PERIOD);
    bt_hid_get_latest(&state);
    updateEncoderValues(&chassis);

    float throttle = (float)state.r2 / 255.0f - (float)state.l2 / 255.0f;
    float steering = ((float)state.lx - 127.5f) / 127.5f;

    float deadzone = 0.05f;
    if (fabs(steering) < deadzone) {
      steering = 0.0f;
    }

    float left = clamp1(throttle + steering);
    float right = clamp1(throttle - steering);
    chassis_set(&chassis, left, right, left, right);
  }

  return 1;
}
