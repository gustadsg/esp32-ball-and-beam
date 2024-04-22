#if !defined(PID_H)
#define PID_H

typedef struct  {
    float Kp;
    float Ki;
    float Kd;
    float out_max;
    float out_min;
    float period_ms;
} pid_config_t;

typedef struct {
    pid_config_t __config;
    float __setpoint;
    float __last_error;
    float __integral;
} pid_controller_t;


pid_controller_t pid_init(pid_config_t pid_config);
void pid_set_setpoint(pid_controller_t *pid, float setpoint);
float pid_calculate(pid_controller_t *pid, float reading);
float limit(float min, float max, float val_in);

#endif // PID_H