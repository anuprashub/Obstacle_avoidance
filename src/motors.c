#include "motors.h"
#include "esp_log.h"

static const char *TAGM = "MOTORS";

/* -------- PIN MAPPING -------- */

// Left motor (Motor A)
#define ENA  38
#define IN1  39
#define IN2  40

// Right motor (Motor B)
#define IN3  41
#define IN4  42
#define ENB  2

/* -------- PWM CONFIG -------- */

#define PWM_FREQ       15000
#define PWM_RES        LEDC_TIMER_8_BIT
#define PWM_SPEED_MAX  255

#define PWM_CH_A       LEDC_CHANNEL_0
#define PWM_CH_B       LEDC_CHANNEL_1

static int motor_speed = 180;

/* -------- INIT -------- */

void motors_init(void)
{
    // Direction pins as outputs
    gpio_set_direction(IN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(IN2, GPIO_MODE_OUTPUT);
    gpio_set_direction(IN3, GPIO_MODE_OUTPUT);
    gpio_set_direction(IN4, GPIO_MODE_OUTPUT);

    // Configure LEDC timer
    ledc_timer_config_t timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = PWM_FREQ,
        .duty_resolution  = PWM_RES,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    // Channel A (ENA)
    ledc_channel_config_t chA = {
        .gpio_num   = ENA,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = PWM_CH_A,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&chA);

    // Channel B (ENB)
    ledc_channel_config_t chB = {
        .gpio_num   = ENB,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = PWM_CH_B,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&chB);

    // ESP_LOGI(TAGM, "Motors initialized");
}

void motor_set_speed(int speed)
{
    if (speed < 0)  speed = 0;
    if (speed > PWM_SPEED_MAX) speed = PWM_SPEED_MAX;
    motor_speed = speed;
}

static void set_pwm(int ch, int duty)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}

/* -------- MOVEMENTS -------- */

void motor_forward(void)
{
    ESP_LOGI(TAGM, "Forward");
    gpio_set_level(IN1, 1);
    gpio_set_level(IN2, 0);
    gpio_set_level(IN3, 1);
    gpio_set_level(IN4, 0);

    set_pwm(PWM_CH_A, motor_speed);
    set_pwm(PWM_CH_B, motor_speed);
}

void motor_backward(void)
{
    ESP_LOGI(TAGM, "Backward");
    gpio_set_level(IN1, 0);
    gpio_set_level(IN2, 1);
    gpio_set_level(IN3, 0);
    gpio_set_level(IN4, 1);

    set_pwm(PWM_CH_A, motor_speed);
    set_pwm(PWM_CH_B, motor_speed);
}

void motor_left(void)
{
    ESP_LOGI(TAGM, "Left");
    // Left motor backward, right motor forward
    gpio_set_level(IN1, 0);
    gpio_set_level(IN2, 1);
    gpio_set_level(IN3, 1);
    gpio_set_level(IN4, 0);

    set_pwm(PWM_CH_A, motor_speed);
    set_pwm(PWM_CH_B, motor_speed);
}

void motor_right(void)
{
    ESP_LOGI(TAGM, "Right");
    // Left motor forward, right motor backward
    gpio_set_level(IN1, 1);
    gpio_set_level(IN2, 0);
    gpio_set_level(IN3, 0);
    gpio_set_level(IN4, 1);

    set_pwm(PWM_CH_A, motor_speed);
    set_pwm(PWM_CH_B, motor_speed);
}

void motor_stop(void)
{
    ESP_LOGI(TAGM, "Stop");
    set_pwm(PWM_CH_A, 0);
    set_pwm(PWM_CH_B, 0);

    gpio_set_level(IN1, 0);
    gpio_set_level(IN2, 0);
    gpio_set_level(IN3, 0);
    gpio_set_level(IN4, 0);
}
