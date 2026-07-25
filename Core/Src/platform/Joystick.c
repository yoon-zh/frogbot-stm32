#include "Joystick.h"
#include "led.h"
#include "ps2.h"

#define SPEED_LEVEL_COUNT 6
#define GAMEPAD_LY_CENTRE 128u
#define GAMEPAD_RX_CENTRE 127u

float MAX_rotate_Speed = 1.0f;
float Max_speed[SPEED_LEVEL_COUNT] = {0.5f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f};
float MAX_Speed = 0.5f;
int speed_count = 0;

static bool ButtonPressedEdge(const GamepadFrame *frame, uint16_t button_mask)
{
    return frame != 0 && (frame->buttons_pressed & button_mask) != 0u;
}

void Joystick_v_set(const GamepadFrame *frame)
{
    if (ButtonPressedEdge(frame, GAMEPAD_BUTTON_DPAD_UP))
    {
        if (speed_count < (SPEED_LEVEL_COUNT - 1))
        {
            speed_count++;
        }
    }

    if (ButtonPressedEdge(frame, GAMEPAD_BUTTON_DPAD_DOWN))
    {
        if (speed_count > 0)
        {
            speed_count--;
        }
    }

    MAX_Speed = Max_speed[speed_count];
}

static void SubmitGamepadCommand(const GamepadFrame *frame)
{
    DriveCommand command = {
        .valid = true,
        .received_ms = frame->received_ms,
        .linear_mps = DriveControl_MapAxis(frame->ly,
                                           GAMEPAD_LY_CENTRE,
                                           MAX_Speed),
        .angular_radps = DriveControl_MapAxis(frame->rx,
                                             GAMEPAD_RX_CENTRE,
                                             MAX_rotate_Speed),
    };

    DriveControl_SubmitGamepadCommand(&command);
}

static void SubmitGamepadEvents(const GamepadFrame *frame)
{
    if (ButtonPressedEdge(frame, GAMEPAD_BUTTON_L2))
    {
        DriveControl_RequestSource(DRIVE_SOURCE_SERIAL, frame->received_ms);
    }

    if (ButtonPressedEdge(frame, GAMEPAD_BUTTON_R2))
    {
        DriveControl_RequestSource(DRIVE_SOURCE_GAMEPAD, frame->received_ms);
    }

    if (ButtonPressedEdge(frame, GAMEPAD_BUTTON_Y))
    {
        DriveControl_RequestArm(frame);
    }

    if (ButtonPressedEdge(frame, GAMEPAD_BUTTON_B))
    {
        DriveControl_RequestBrake(frame->received_ms);
    }
}

static void UpdateJoystickLed(void)
{
    switch (DriveControl_GetState())
    {
    case DRIVE_STATE_ESTOP:
    case DRIVE_STATE_DISARMED:
        led_red_start();
        break;
    case DRIVE_STATE_BRAKING:
        led_pink_start();
        break;
    case DRIVE_STATE_ARMED:
        led_green_start();
        break;
    default:
        break;
    }
}

void Joystick_motor_control(void)
{
    GamepadFrame frame;

    if (!PS2_ReadFrame(&frame))
    {
        return;
    }

    DriveControl_SubmitGamepadFrame(&frame);
    Joystick_v_set(&frame);
    SubmitGamepadCommand(&frame);
    SubmitGamepadEvents(&frame);
    UpdateJoystickLed();
}
