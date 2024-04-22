#include <stdio.h>
#include <stdint.h>

// esp32 includes
#include <driver/gpio.h>
#include <esp_log.h>

// freeRTOS includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// lib includes
#include <ultrasonic.h>
#include <servo.h>
#include <pid.h>

#define ULTRASONIC_TRIGGER_PIN GPIO_NUM_2
#define ULTRASONIC_ECHO_PIN GPIO_NUM_4
#define BEAM_LENGTH_CM 60

#define MG996R_MAX_ANGLE 60

#define CORE0 0 // this is the most powerful  (240MHz)
#define CORE1 1 // this is the least powerful (160MHz)

// LOG TAGS
#define CONTROL_LOOP_TAG "ControlLoop"

// global variables
uint32_t setpoint = 0;
const int period_ms = 200;

ultrasonic_sensor_t ultrasonic_sensor = {
    .echo_pin = ULTRASONIC_ECHO_PIN,
    .trigger_pin = ULTRASONIC_TRIGGER_PIN
};

servo_t servo = {
    .max_delta_angle = MG996R_MAX_ANGLE,
    .signal_pin = GPIO_NUM_17,
    .timer = LEDC_TIMER_0,
    .timer_channel = LEDC_CHANNEL_0
};

pid_controller_t pid_controller;

// PID variables
float Kp = 1;
float Ki = 0;
float Kd = 0;


// function signatures
// tasks
void app_main();
void control_loop();

// init
void init();
void init_ultrasonic_sensor();
void init_servo_motor();
void init_pid();

// process
void read_pv(ultrasonic_sensor_t *sensor, uint32_t *distance_out);
void calculate_control_action(uint32_t setpoint, uint32_t process_variable, float *control_action_out);
void write_mv(float control_action);

// helpers
void print_read_error(esp_err_t error);

// code begin
void app_main()
{
    init();
    // Control loop task requires least power, this is why its associated with least powerful core
    xTaskCreatePinnedToCore(control_loop, "ControlLoopTask", 10 * configMINIMAL_STACK_SIZE, NULL, 1, NULL, CORE1);
}

void init()
{
    init_ultrasonic_sensor();
    init_servo_motor();
    init_pid();
}

void init_ultrasonic_sensor()
{
    esp_err_t status = ultrasonic_init(&ultrasonic_sensor);
    if (ESP_OK != status)
        esp_restart();
}

void init_servo_motor()
{
    esp_err_t status = servo_init(servo);
    if (ESP_OK != status)
        esp_restart();
}

void init_pid() {
    pid_config_t config = {
        .Kp = Kp,
        .Ki = 0,
        .Kd = 0,
        .out_max = MG996R_MAX_ANGLE,
        .out_min = -MG996R_MAX_ANGLE,
        .period_ms = period_ms
    };

    pid_controller = pid_init(config);
}

void control_loop()
{
    uint32_t distance = 0;
    float control_action = 0;

    while (1)
    {
        read_pv(&ultrasonic_sensor, &distance);
        ESP_LOGI(CONTROL_LOOP_TAG, "PV lido: %lu", distance);

        calculate_control_action(setpoint, distance, &control_action);
        ESP_LOGI(CONTROL_LOOP_TAG, "AC calculada: %f", control_action);

        write_mv(control_action);

        vTaskDelay(pdMS_TO_TICKS(period_ms));
    }
}

void read_pv(ultrasonic_sensor_t *sensor, uint32_t *distance_out)
{
    int32_t previous_val = *distance_out;
    esp_err_t state = ultrasonic_measure_cm(sensor, BEAM_LENGTH_CM, distance_out);

    if (ESP_OK == state)
        return;

    print_read_error(state);

    // Echo took longer than expected to return. Probably object is too far. Return max logical distance
    if (ESP_ERR_ULTRASONIC_ECHO_TIMEOUT == state)
    {
        *distance_out = BEAM_LENGTH_CM;
        return;
    }

    // Other errors occourred, just take the last value.
    *distance_out = previous_val;
}

void calculate_control_action(uint32_t setpoint, uint32_t process_variable, float *control_action_out)
{
    pid_set_setpoint(&pid_controller, setpoint);
    
    *control_action_out = pid_calculate(&pid_controller, process_variable);;
}

void write_mv(float control_action) {
    set_angle(servo, control_action);
}

void print_read_error(esp_err_t error)
{
    switch (error)
    {
    case ESP_ERR_ULTRASONIC_PING:
        ESP_LOGE(CONTROL_LOOP_TAG, "Cannot ping (device is in invalid state)");
        break;
    case ESP_ERR_ULTRASONIC_PING_TIMEOUT:
        ESP_LOGE(CONTROL_LOOP_TAG, "Ping timeout (no device found)");
        break;
    case ESP_ERR_ULTRASONIC_ECHO_TIMEOUT:
        ESP_LOGE(CONTROL_LOOP_TAG, "Echo timeout (i.e. distance too big)");
        break;
    default:
        ESP_LOGE(CONTROL_LOOP_TAG, "%s\n", esp_err_to_name(error));
    }
}