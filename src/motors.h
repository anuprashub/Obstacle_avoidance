#pragma once
#include "driver/gpio.h"
#include "driver/ledc.h"

void motors_init(void);
void motor_set_speed(int speed);
void motor_forward(void);
void motor_backward(void);
void motor_left(void);
void motor_right(void);
void motor_stop(void);
