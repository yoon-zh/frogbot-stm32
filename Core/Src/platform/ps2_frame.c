#include "ps2_frame.h"

#define PS2_RESPONSE_ANALOG 0x73u
#define PS2_RESPONSE_MAGIC 0x5Au
#define PS2_RESPONSE_BUTTONS_LOW 3u
#define PS2_RESPONSE_BUTTONS_HIGH 4u
#define PS2_RESPONSE_RX 5u
#define PS2_RESPONSE_RY 6u
#define PS2_RESPONSE_LX 7u
#define PS2_RESPONSE_LY 8u

bool PS2_DecodeGamepadFrame(const uint8_t *response,
                            size_t response_length,
                            uint32_t received_ms,
                            uint16_t previous_pressed_buttons,
                            GamepadFrame *frame,
                            uint16_t *current_pressed_buttons)
{
    uint16_t active_low_buttons;
    uint16_t pressed_buttons;

    if (frame != 0)
    {
        frame->valid = false;
    }

    if (response == 0 || frame == 0 || current_pressed_buttons == 0 ||
        response_length != PS2_FRAME_LENGTH ||
        response[1] != PS2_RESPONSE_ANALOG ||
        response[2] != PS2_RESPONSE_MAGIC)
    {
        return false;
    }

    active_low_buttons = (uint16_t)(((uint16_t)response[PS2_RESPONSE_BUTTONS_HIGH] << 8) |
                                    response[PS2_RESPONSE_BUTTONS_LOW]);
    pressed_buttons = (uint16_t)(~active_low_buttons);

    frame->valid = true;
    frame->received_ms = received_ms;
    frame->buttons = active_low_buttons;
    frame->buttons_pressed = (uint16_t)(pressed_buttons & (uint16_t)(~previous_pressed_buttons));
    frame->buttons_released = (uint16_t)(((uint16_t)(~pressed_buttons)) & previous_pressed_buttons);
    frame->lx = response[PS2_RESPONSE_LX];
    frame->ly = response[PS2_RESPONSE_LY];
    frame->rx = response[PS2_RESPONSE_RX];
    frame->ry = response[PS2_RESPONSE_RY];
    *current_pressed_buttons = pressed_buttons;
    return true;
}
