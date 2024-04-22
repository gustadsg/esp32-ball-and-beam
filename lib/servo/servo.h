#if !defined(SERVO_H)
#define SERVO_H

#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "inttypes.h"

#define SERVO_FREQUENCY_HZ 50
#define SERVO_RESOLUTION LEDC_TIMER_10_BIT
#define SERVO_SPEED_MODE LEDC_HIGH_SPEED_MODE
#define SERVO_MIN_DUTY 76.8     // equivalent to 1ms (-90deg), using 10 bit resolution
#define SERVO_MAX_DUTY 102.4    // equivalent to 2ms (90deg), using 10 bit resolution
#define SERVO_MAX_PERCENTAGE 100
#define SERVO_MIN_PERCENTAGE 0


typedef struct {
    gpio_num_t signal_pin;
    ledc_timer_t timer;
    ledc_channel_t timer_channel;
    int max_delta_angle;

    ledc_timer_config_t _timer_config;
    ledc_channel_config_t _channel_config;
} servo_t;

typedef struct {
    float max;
    float min;
} scale_t;

esp_err_t servo_init(servo_t config);
esp_err_t set_angle(servo_t servo, float angle);
esp_err_t set_percentage(servo_t servo, float percentage);
esp_err_t set_duty(servo_t servo, uint32_t duty);
esp_err_t calculate_duty_by_angle(servo_t servo, float desired_angle, int32_t *duty_out);
esp_err_t calculate_duty_by_percentage(servo_t servo, float desired_percentage, int32_t *duty_out);
esp_err_t convert_scale(scale_t from, scale_t to, float value_in, float *value_out);

#endif // SERVO_H
