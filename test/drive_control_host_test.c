#include <assert.h>
#include <math.h>
#include <stdbool.h>

#include "drive_control.h"
#include "ps2_frame.h"

static GamepadFrame GamepadFrameAt(uint32_t timestamp, uint8_t ly, uint8_t rx)
{
    GamepadFrame frame = {
        .valid = true,
        .received_ms = timestamp,
        .buttons = UINT16_MAX,
        .ly = ly,
        .rx = rx,
    };
    return frame;
}

static DriveCommand CommandAt(uint32_t timestamp, float linear, float angular)
{
    DriveCommand command = {
        .valid = true,
        .received_ms = timestamp,
        .linear_mps = linear,
        .angular_radps = angular,
    };
    return command;
}

static void SubmitGamepad(uint32_t timestamp, float linear, float angular)
{
    GamepadFrame frame = GamepadFrameAt(timestamp, 128u, 127u);
    DriveCommand command = CommandAt(timestamp, linear, angular);

    DriveControl_SubmitGamepadFrame(&frame);
    DriveControl_SubmitGamepadCommand(&command);
}

static void ArmSerial(uint32_t timestamp)
{
    GamepadFrame frame = GamepadFrameAt(timestamp, 128u, 127u);
    DriveCommand command = CommandAt(timestamp, 0.4f, -0.2f);

    DriveControl_SubmitGamepadFrame(&frame);
    DriveControl_SubmitSerialCommand(&command);
    DriveControl_RequestArm(&frame);
    DriveControl_Tick(timestamp, false);
    assert(DriveControl_GetState() == DRIVE_STATE_ARMED);
    assert(DriveControl_GetOutput().authorised);
}

static void TestInvalidGamepadCannotChangeSerial(void)
{
    DriveControl_Init(0u);
    ArmSerial(1u);

    /* Invalid gamepad data is deliberately never submitted to the arbiter. */
    DriveControl_Tick(50u, false);
    assert(DriveControl_GetSource() == DRIVE_SOURCE_SERIAL);
    assert(DriveControl_GetState() == DRIVE_STATE_ARMED);
    assert(DriveControl_GetOutput().authorised);
}

static void TestSwitchNeedsPostSwitchCommand(void)
{
    DriveControlOutput output;

    DriveControl_Init(0u);
    ArmSerial(1u);
    SubmitGamepad(2u, 0.7f, 0.3f);

    DriveControl_RequestSource(DRIVE_SOURCE_GAMEPAD, 10u);
    DriveControl_Tick(10u, false);
    output = DriveControl_GetOutput();
    assert(DriveControl_GetSource() == DRIVE_SOURCE_GAMEPAD);
    assert(DriveControl_GetState() == DRIVE_STATE_ARMED);
    assert(!output.authorised);
    assert(output.linear_mps == 0.0f);

    SubmitGamepad(11u, 0.7f, 0.3f);
    DriveControl_Tick(11u, false);
    output = DriveControl_GetOutput();
    assert(output.authorised);
    assert(fabsf(output.linear_mps - 0.7f) < 0.0001f);
    assert(fabsf(output.angular_radps - 0.3f) < 0.0001f);
}

static void TestArmAndSourceSwitchOrdering(void)
{
    GamepadFrame neutral = GamepadFrameAt(1u, 128u, 127u);

    DriveControl_Init(0u);
    SubmitGamepad(1u, 0.3f, 0.0f);
    DriveControl_RequestArm(&neutral);
    DriveControl_RequestSource(DRIVE_SOURCE_GAMEPAD, 1u);
    DriveControl_Tick(1u, false);
    assert(DriveControl_GetState() == DRIVE_STATE_ARMED);
    assert(DriveControl_GetSource() == DRIVE_SOURCE_GAMEPAD);
    assert(!DriveControl_GetOutput().authorised);

    SubmitGamepad(2u, 0.3f, 0.0f);
    DriveControl_Tick(2u, false);
    assert(DriveControl_GetOutput().authorised);
}

static void TestTimeoutsApplyOnlyToActiveSource(void)
{
    DriveControl_Init(0u);
    ArmSerial(1u);
    DriveControl_Tick(251u, false);
    assert(DriveControl_GetState() == DRIVE_STATE_DISARMED);
    assert(!DriveControl_GetOutput().authorised);

    DriveControl_Init(0u);
    ArmSerial(1u);
    DriveControl_RequestSource(DRIVE_SOURCE_GAMEPAD, 2u);
    DriveControl_Tick(2u, false);
    SubmitGamepad(3u, 0.5f, 0.0f);
    DriveControl_Tick(3u, false);
    SubmitGamepad(299u, 0.5f, 0.0f);
    DriveControl_Tick(300u, false);
    assert(DriveControl_GetSource() == DRIVE_SOURCE_GAMEPAD);
    assert(DriveControl_GetState() == DRIVE_STATE_ARMED);
    assert(DriveControl_GetOutput().authorised);

    DriveControl_Tick(399u, false);
    assert(DriveControl_GetState() == DRIVE_STATE_DISARMED);
}

static void TestBrakingAndEstopRejectOtherRequests(void)
{
    GamepadFrame neutral = GamepadFrameAt(4u, 128u, 127u);

    DriveControl_Init(0u);
    ArmSerial(1u);
    DriveControl_RequestBrake(2u);
    DriveControl_Tick(2u, false);
    assert(DriveControl_GetState() == DRIVE_STATE_ARMED);

    DriveControl_Init(0u);
    SubmitGamepad(1u, 0.2f, 0.0f);
    DriveControl_RequestArm(&(GamepadFrame){
        .valid = true,
        .received_ms = 1u,
        .ly = 144u,
        .rx = 127u,
    });
    DriveControl_Tick(1u, false);
    assert(DriveControl_GetState() == DRIVE_STATE_DISARMED);

    DriveControl_Init(0u);
    SubmitGamepad(1u, 0.2f, 0.0f);
    DriveControl_RequestSource(DRIVE_SOURCE_GAMEPAD, 2u);
    DriveControl_Tick(2u, false);
    SubmitGamepad(3u, 0.2f, 0.0f);
    DriveControl_RequestArm(&neutral);
    DriveControl_Tick(4u, false);
    assert(DriveControl_GetState() == DRIVE_STATE_ARMED);

    DriveControl_RequestBrake(5u);
    DriveControl_Tick(5u, false);
    assert(DriveControl_GetState() == DRIVE_STATE_BRAKING);
    assert(!DriveControl_GetOutput().authorised);

    DriveControl_RequestSource(DRIVE_SOURCE_SERIAL, 6u);
    DriveControl_RequestArm(&neutral);
    DriveControl_RequestBrake(6u);
    DriveControl_Tick(6u, false);
    assert(DriveControl_GetSource() == DRIVE_SOURCE_GAMEPAD);
    assert(DriveControl_GetState() == DRIVE_STATE_BRAKING);

    DriveControl_Tick(7u, true);
    assert(DriveControl_GetState() == DRIVE_STATE_DISARMED);

    DriveControl_LatchEmergencyStopFromISR();
    DriveControl_RequestSource(DRIVE_SOURCE_SERIAL, 8u);
    DriveControl_RequestArm(&neutral);
    DriveControl_Tick(8u, false);
    assert(DriveControl_GetState() == DRIVE_STATE_ESTOP);
    assert(!DriveControl_GetOutput().authorised);
}

static void TestNeutralBandAndAxisMapping(void)
{
    GamepadFrame neutral = GamepadFrameAt(1u, 143u, 112u);
    GamepadFrame outside = GamepadFrameAt(1u, 144u, 127u);

    assert(DriveControl_IsGamepadNeutral(&neutral));
    assert(!DriveControl_IsGamepadNeutral(&outside));
    assert(DriveControl_MapAxis(128u, 128u, 2.0f) == 0.0f);
    assert(DriveControl_MapAxis(143u, 128u, 2.0f) == 0.0f);
    assert(fabsf(DriveControl_MapAxis(0u, 128u, 2.0f) - 2.0f) < 0.0001f);
    assert(fabsf(DriveControl_MapAxis(255u, 128u, 2.0f) + 2.0f) < 0.0001f);
}

static void TestPs2FrameValidationAndEdges(void)
{
    uint8_t response[PS2_FRAME_LENGTH] = {0};
    GamepadFrame frame;
    uint16_t pressed_buttons;

    response[1] = 0x73u;
    response[2] = 0x5Au;
    response[3] = 0xFFu;
    response[4] = 0xFEu; /* L2 is pressed in the active-low bitmap. */
    response[5] = 0x00u;
    response[6] = 0x80u;
    response[7] = 0xFFu;
    response[8] = 0x00u;

    assert(PS2_DecodeGamepadFrame(response,
                                  sizeof(response),
                                  10u,
                                  0u,
                                  &frame,
                                  &pressed_buttons));
    assert(frame.buttons == 0xFEFFu);
    assert((frame.buttons_pressed & GAMEPAD_BUTTON_L2) != 0u);
    assert(frame.lx == 0xFFu);
    assert(frame.ly == 0x00u);
    assert(frame.rx == 0x00u);
    assert(frame.ry == 0x80u);

    response[4] = 0xFFu;
    assert(PS2_DecodeGamepadFrame(response,
                                  sizeof(response),
                                  11u,
                                  pressed_buttons,
                                  &frame,
                                  &pressed_buttons));
    assert((frame.buttons_released & GAMEPAD_BUTTON_L2) != 0u);

    assert(!PS2_DecodeGamepadFrame(response,
                                   PS2_FRAME_LENGTH - 1u,
                                   12u,
                                   pressed_buttons,
                                   &frame,
                                   &pressed_buttons));
    response[1] = 0x41u;
    assert(!PS2_DecodeGamepadFrame(response,
                                   sizeof(response),
                                   12u,
                                   pressed_buttons,
                                   &frame,
                                   &pressed_buttons));
    response[1] = 0x73u;
    response[2] = 0x00u;
    assert(!PS2_DecodeGamepadFrame(response,
                                   sizeof(response),
                                   12u,
                                   pressed_buttons,
                                   &frame,
                                   &pressed_buttons));
}

int main(void)
{
    TestInvalidGamepadCannotChangeSerial();
    TestSwitchNeedsPostSwitchCommand();
    TestArmAndSourceSwitchOrdering();
    TestTimeoutsApplyOnlyToActiveSource();
    TestBrakingAndEstopRejectOtherRequests();
    TestNeutralBandAndAxisMapping();
    TestPs2FrameValidationAndEdges();
    return 0;
}
