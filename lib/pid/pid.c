#include "pid.h"

pid_controller_t pid_init(pid_config_t pid_config) {
    pid_controller_t pid = {
        .__config = pid_config,
        .__setpoint = 0,
        .__last_error = 0,
        .__integral = 0
    };

    return pid;
}

void pid_set_setpoint(pid_controller_t *pid, float setpoint) {
    pid->__setpoint = setpoint;
}

float pid_calculate(pid_controller_t *pid, float reading) {
    float error = pid->__setpoint - reading;
    float derivative = (error - pid->__last_error)/pid->__config.period_ms;
    float integral = pid->__integral + error*pid->__config.period_ms;

    pid->__integral = limit(pid->__config.out_min, pid->__config.out_max, integral);
    pid->__last_error = error;

    float P = error * pid->__config.Kp;
    float I = pid->__integral * pid->__config.Ki;
    float D = derivative * pid->__config.Kd;

    return limit(pid->__config.out_min, pid->__config.out_max, P+I+D);
}

float limit(float min, float max, float val_in) {
    if(val_in>max) return max;
    if(val_in<min) return min;
    return val_in;
}