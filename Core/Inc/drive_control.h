#ifndef DRIVE_CONTROL_H
#define DRIVE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#define DRIVE_GAMEPAD_TIMEOUT_MS 100u
#define DRIVE_SERIAL_TIMEOUT_MS 250u
#define DRIVE_AXIS_NEUTRAL_BAND 15u

#define GAMEPAD_BUTTON(index) ((uint16_t)1u << ((index) - 1u))
#define GAMEPAD_BUTTON_L2 GAMEPAD_BUTTON(9u)
#define GAMEPAD_BUTTON_R2 GAMEPAD_BUTTON(10u)
#define GAMEPAD_BUTTON_Y GAMEPAD_BUTTON(13u)
#define GAMEPAD_BUTTON_B GAMEPAD_BUTTON(14u)
#define GAMEPAD_BUTTON_DPAD_UP GAMEPAD_BUTTON(5u)
#define GAMEPAD_BUTTON_DPAD_DOWN GAMEPAD_BUTTON(7u)

typedef enum {
    DRIVE_SOURCE_GAMEPAD,
    DRIVE_SOURCE_SERIAL,
} DriveSource;

typedef enum {
    DRIVE_STATE_DISARMED,
    DRIVE_STATE_ARMED,
    DRIVE_STATE_BRAKING,
    DRIVE_STATE_ESTOP,
} DriveState;

typedef struct {
    bool valid;
    uint32_t received_ms;
    uint16_t buttons;          // Complete active-low PS2 bitmap.
    uint16_t buttons_pressed;  // Active-high pressed-edge bitmap.
    uint16_t buttons_released; // Active-high released-edge bitmap.
    uint8_t lx;
    uint8_t ly;
    uint8_t rx;
    uint8_t ry;
} GamepadFrame;

typedef struct {
    bool valid;
    uint32_t received_ms;
    float linear_mps;
    float angular_radps;
} DriveCommand;

typedef struct {
    DriveSource source;
    DriveState state;
    bool authorised;
    float linear_mps;
    float angular_radps;
} DriveControlOutput;

void DriveControl_Init(uint32_t now_ms);
void DriveControl_SubmitGamepadFrame(const GamepadFrame *frame);
void DriveControl_SubmitGamepadCommand(const DriveCommand *command);
void DriveControl_SubmitSerialCommand(const DriveCommand *command);

void DriveControl_RequestSource(DriveSource source, uint32_t requested_ms);
void DriveControl_RequestArm(const GamepadFrame *frame);
void DriveControl_RequestBrake(uint32_t requested_ms);
void DriveControl_LatchEmergencyStopFromISR(void);
void DriveControl_Tick(uint32_t now_ms, bool brake_wheels_stopped);

DriveControlOutput DriveControl_GetOutput(void);
DriveSource DriveControl_GetSource(void);
DriveState DriveControl_GetState(void);
bool DriveControl_IsGamepadNeutral(const GamepadFrame *frame);
float DriveControl_MapAxis(uint8_t value, uint8_t centre, float maximum);

#endif
