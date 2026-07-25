#ifndef PS2_FRAME_H
#define PS2_FRAME_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "drive_control.h"

#define PS2_FRAME_LENGTH 9u

bool PS2_DecodeGamepadFrame(const uint8_t *response,
                            size_t response_length,
                            uint32_t received_ms,
                            uint16_t previous_pressed_buttons,
                            GamepadFrame *frame,
                            uint16_t *current_pressed_buttons);

#endif
