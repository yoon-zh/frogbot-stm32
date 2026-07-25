#include "Motor_Speed_pid.h"
#include "drive_control.h"

#define MAX_SPEED_RPM 20000
#define BRAKE_STOP_RPM 120
#define BRAKE_LOW_SPEED_RPM 300
#define BRAKE_CURRENT_LIMIT 8000
#define BRAKE_LOW_SPEED_CURRENT_LIMIT 4500
#define BRAKE_DAMPING_KP 12.0f
#define BRAKE_CURRENT_STEP 1200

PID_TypeDef motor_pid[4];
float set_spdL;
float set_spdR;
float set_spdL1;
float set_spdR1;
float C = 0.46f;
float r = 0.1f;
int flaggg = 0;

static int16_t brake_last_current[4] = {0, 0, 0, 0};
static DriveState previous_drive_state = DRIVE_STATE_DISARMED;

static void Clear_Motor_PID_State(void)
{
    for (int i = 0; i < 4; i++)
    {
        motor_pid[i].target = 0;
        motor_pid[i].err = 0;
        motor_pid[i].last_err = 0;
        motor_pid[i].pout = 0;
        motor_pid[i].iout = 0;
        motor_pid[i].dout = 0;
        motor_pid[i].output = 0;
        motor_pid[i].last_output = 0;
        motor_pid[i].calculate_output = 0;
    }
}

void Clear_Brake_State(void)
{
    for (int i = 0; i < 4; i++)
    {
        brake_last_current[i] = 0;
    }
    Clear_Motor_PID_State();
}

static int16_t Damping_Brake_Current(int16_t speed_rpm)
{
    int32_t output = (int32_t)(-BRAKE_DAMPING_KP * speed_rpm);
    int16_t limit = BRAKE_CURRENT_LIMIT;

    if (speed_rpm < BRAKE_LOW_SPEED_RPM && speed_rpm > -BRAKE_LOW_SPEED_RPM)
    {
        limit = BRAKE_LOW_SPEED_CURRENT_LIMIT;
    }

    if (output > limit)
    {
        return limit;
    }
    if (output < -limit)
    {
        return -limit;
    }
    return (int16_t)output;
}

static int16_t Ramp_Brake_Current(uint8_t index, int16_t target_current)
{
    int32_t delta = target_current - brake_last_current[index];

    if (delta > BRAKE_CURRENT_STEP)
    {
        target_current = (int16_t)(brake_last_current[index] + BRAKE_CURRENT_STEP);
    }
    else if (delta < -BRAKE_CURRENT_STEP)
    {
        target_current = (int16_t)(brake_last_current[index] - BRAKE_CURRENT_STEP);
    }

    brake_last_current[index] = target_current;
    return target_current;
}

static bool Brake_Wheels_Stopped(void)
{
    for (int i = 0; i < 4; i++)
    {
        if (motor_chassis[i].speed_rpm > BRAKE_STOP_RPM ||
            motor_chassis[i].speed_rpm < -BRAKE_STOP_RPM)
        {
            return false;
        }
    }
    return true;
}

float low_pass_filter(float value, float fc, float Ts)
{
    float b = fc * Ts;
    float alpha = b / (b + 1.0f);
    static float out_last = 0.0f;
    static bool first_sample = true;
    float out;

    if (first_sample)
    {
        first_sample = false;
        out_last = value;
    }

    out = out_last + alpha * (value - out_last);
    out_last = out;
    return out;
}

static void Motor_Speed_Calc(float linear_mps, float angular_radps)
{
    set_spdL = (2.0f * linear_mps - angular_radps * C) / (2.0f * r) * 9.55f * 19.2f;
    set_spdR = (2.0f * linear_mps + angular_radps * C) / (2.0f * r) * 9.55f * 19.2f;

    if (set_spdL > MAX_SPEED_RPM)
    {
        set_spdL = MAX_SPEED_RPM;
    }
    if (set_spdL < -MAX_SPEED_RPM)
    {
        set_spdL = -MAX_SPEED_RPM;
    }
    if (set_spdR > MAX_SPEED_RPM)
    {
        set_spdR = MAX_SPEED_RPM;
    }
    if (set_spdR < -MAX_SPEED_RPM)
    {
        set_spdR = -MAX_SPEED_RPM;
    }
}

void Motor_Speed_pid_init(void)
{
    for (int i = 0; i < 4; i++)
    {
        pid_init(&motor_pid[i]);
        motor_pid[i].f_param_init(&motor_pid[i], PID_Speed,
                                  15000,
                                  1000,
                                  0,
                                  0,
                                  1000,
                                  0,
                                  10,
                                  100,
                                  0);
    }
}

void Set_free(void)
{
    Emergency_Stop_Output();
}

void Emergency_Stop_Output(void)
{
    set_spdL = 0.0f;
    set_spdR = 0.0f;
    Clear_Motor_PID_State();
    CAN_cmd_chassis(0, 0, 0, 0);
}

void Active_Brake_Output(void)
{
    set_spdL = 0.0f;
    set_spdR = 0.0f;

    if (Brake_Wheels_Stopped())
    {
        CAN_cmd_chassis(0, 0, 0, 0);
        return;
    }

    CAN_cmd_chassis(Ramp_Brake_Current(0, Damping_Brake_Current(motor_chassis[0].speed_rpm)),
                    Ramp_Brake_Current(1, Damping_Brake_Current(motor_chassis[1].speed_rpm)),
                    Ramp_Brake_Current(2, Damping_Brake_Current(motor_chassis[2].speed_rpm)),
                    Ramp_Brake_Current(3, Damping_Brake_Current(motor_chassis[3].speed_rpm)));
}

void Speed_set(void)
{
    DriveControlOutput output;

    DriveControl_Tick(HAL_GetTick(), Brake_Wheels_Stopped());
    output = DriveControl_GetOutput();

    if (output.state == DRIVE_STATE_BRAKING)
    {
        if (previous_drive_state != DRIVE_STATE_BRAKING)
        {
            Clear_Brake_State();
        }
        Active_Brake_Output();
        previous_drive_state = output.state;
        return;
    }

    previous_drive_state = output.state;
    if (!output.authorised)
    {
        if (output.state == DRIVE_STATE_ESTOP)
        {
            led_red_start();
        }
        Set_free();
        return;
    }

    Motor_Speed_Calc(output.linear_mps, output.angular_radps);
    motor_pid[0].target = set_spdL;
    motor_pid[1].target = set_spdL;
    motor_pid[2].target = -set_spdR;
    motor_pid[3].target = -set_spdR;
    for (int i = 0; i < 4; i++)
    {
        motor_pid[i].f_cal_pid(&motor_pid[i], motor_chassis[i].speed_rpm);
    }

    CAN_cmd_chassis(motor_pid[0].output,
                    motor_pid[1].output,
                    motor_pid[2].output,
                    motor_pid[3].output);
}

// Using motor speed (rpm) to calculate vehicle linear/angular velocity.
void MOTORrpm2vw(float left_motor_speed, float right_motor_speed, float *vcx, float *w)
{
    const float c = 0.5f;
    const float wheel_radius = 0.1f;
    float left_wheel_w = left_motor_speed / 19.0f / 9.55f;
    float right_wheel_w = right_motor_speed / 19.0f / 9.55f;
    float vL = left_wheel_w * wheel_radius;
    float vR = right_wheel_w * wheel_radius;

    *vcx = 0.5f * vL + 0.5f * vR;
    *w = -vL / c + vR / c;
}

void speed_print(void)
{
}
