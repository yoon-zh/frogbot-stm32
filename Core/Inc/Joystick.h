#ifndef JOYSTICK_H
#define JOYSTICK_H

#include "drive_control.h"

extern int speed_count;
extern float Max_speed[];
extern float MAX_Speed;

void Joystick_motor_control(void);
void Joystick_v_set(const GamepadFrame *frame);

#endif
