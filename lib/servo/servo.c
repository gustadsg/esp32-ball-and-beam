#include "servo.h"

esp_err_t servo_init(servo_t config) {
    config._timer_config = (ledc_timer_config_t) {
        .duty_resolution = SERVO_RESOLUTION,
        .freq_hz = SERVO_FREQUENCY_HZ,
        .speed_mode = SERVO_SPEED_MODE,
        .timer_num = config.timer
    };
    esp_err_t status = ledc_timer_config(&config._timer_config);
    if(ESP_OK != status) return status;

    config._channel_config = (ledc_channel_config_t) {
        .channel    = config.timer_channel,
        .duty       = 0,
        .gpio_num   = config.signal_pin,
        .speed_mode = SERVO_SPEED_MODE,
        .timer_sel  = config.timer
    };
    status = ledc_channel_config(&config._channel_config);
    if(ESP_OK != status) return status;

    return ESP_OK;
}

esp_err_t set_angle(servo_t servo, float angle) {
    int32_t duty = ledc_get_duty(servo._timer_config.speed_mode, servo.timer_channel);
    esp_err_t status = calculate_duty_by_angle(servo, angle, &duty);
    if(ESP_OK != status) return status;

    status = set_duty(servo, duty);
    if(ESP_OK!=status) return status;

    return ESP_OK;
}

esp_err_t set_percentage(servo_t servo, float percentage) {
    int32_t duty = ledc_get_duty(servo._timer_config.speed_mode, servo.timer_channel);
    esp_err_t status = calculate_duty_by_percentage(servo, percentage, &duty);
    if(ESP_OK != status) return status;

    status = set_duty(servo, duty);
    if(ESP_OK!=status) return status;

    return ESP_OK;
}

esp_err_t set_duty(servo_t servo, uint32_t duty) {
    esp_err_t status = ledc_set_duty(servo._timer_config.speed_mode, servo.timer_channel, duty);
    if(ESP_OK != status) return status;

    status = ledc_update_duty(servo._timer_config.speed_mode, servo.timer_channel);
    if(ESP_OK != status) return status;

    return ESP_OK;
}

esp_err_t calculate_duty_by_angle(servo_t servo, float desired_angle, int32_t *duty_out) {
    if(desired_angle > servo.max_delta_angle) desired_angle = servo.max_delta_angle;
    if(desired_angle < -servo.max_delta_angle) desired_angle = -servo.max_delta_angle;

    scale_t scale_duty = {
        .max = SERVO_MAX_DUTY,
        .min = SERVO_MIN_DUTY
    };

    scale_t scale_angle = {
        .max = 90,
        .min = -90
    };

    float conversion_result = 0;
    esp_err_t status = convert_scale(scale_angle, scale_duty, desired_angle, &conversion_result);
    *duty_out = (int32_t) conversion_result;
    if(ESP_OK != status) return status;

    return ESP_OK;
}

esp_err_t calculate_duty_by_percentage(servo_t servo, float desired_percentage, int32_t *duty_out) {
    if(desired_percentage > SERVO_MAX_PERCENTAGE) desired_percentage = SERVO_MAX_PERCENTAGE;
    if(desired_percentage < SERVO_MIN_PERCENTAGE) desired_percentage = SERVO_MIN_PERCENTAGE;

    scale_t scale_percentage = {
        .max = SERVO_MAX_PERCENTAGE,
        .min = SERVO_MIN_PERCENTAGE
    };

    scale_t scale_angle = {
        .max = servo.max_delta_angle,
        .min = -servo.max_delta_angle
    };

    float percentage_to_angle = 0;
    esp_err_t status = convert_scale(scale_percentage, scale_angle, desired_percentage, &percentage_to_angle);
    if(ESP_OK != status) return status;
    
    status = calculate_duty_by_angle(servo, percentage_to_angle, duty_out);
    if(ESP_OK != status) return status;

    return ESP_OK;
}

esp_err_t convert_scale(scale_t from, scale_t to, float value_in, float *value_out) {
    float delta_from = from.max - from.min;
    float delta_to = to.max - to.min;

    if(delta_from == 0.0) return ESP_ERR_INVALID_ARG;

    *value_out = to.min + ((value_in-from.min) * (delta_to/delta_from));
    return ESP_OK;
}