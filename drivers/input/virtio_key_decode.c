/****************************************************************************
 * drivers/input/virtio_key_decode.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>

#include <nuttx/config.h>

#include <nuttx/input/kbd_codec.h>
#include <nuttx/input/virtio-input-event-codes.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function
 ****************************************************************************/

/****************************************************************************
 * keyboard_translate_virtio_code
 ****************************************************************************/

uint32_t keyboard_translate_virtio_code(uint16_t keycode, bool *special)
{
  *special = true;
  switch (keycode)
    {
      case KEY_DELETE:
        return KEYCODE_FWDDEL;
      case KEY_BACKSPACE:
        return KEYCODE_BACKDEL;
      case KEY_HOME:
        return KEYCODE_HOME;
      case KEY_END:
        return KEYCODE_END;
      case KEY_LEFT:
        return KEYCODE_LEFT;
      case KEY_RIGHT:
        return KEYCODE_RIGHT;
      case KEY_UP:
        return KEYCODE_UP;
      case KEY_DOWN:
        return KEYCODE_DOWN;
      case KEY_PAGEUP:
        return KEYCODE_PAGEUP;
      case KEY_PAGEDOWN:
        return KEYCODE_PAGEDOWN;
      case KEY_ENTER:
        return KEYCODE_ENTER;
      case KEY_CAPSLOCK:
        return KEYCODE_CAPSLOCK;
      case KEY_SCROLLLOCK:
        return KEYCODE_SCROLLLOCK;
      case KEY_NUMLOCK:
        return KEYCODE_NUMLOCK;
      case KEY_SYSRQ:
        return KEYCODE_PRTSCRN;
      case KEY_F1:
        return KEYCODE_F1;
      case KEY_F2:
        return KEYCODE_F2;
      case KEY_F3:
        return KEYCODE_F3;
      case KEY_F4:
        return KEYCODE_F4;
      case KEY_F5:
        return KEYCODE_F5;
      case KEY_F6:
        return KEYCODE_F6;
      case KEY_F7:
        return KEYCODE_F7;
      case KEY_F8:
        return KEYCODE_F8;
      case KEY_F9:
        return KEYCODE_F9;
      case KEY_F10:
        return KEYCODE_F10;
      case KEY_F11:
        return KEYCODE_F11;
      case KEY_F12:
        return KEYCODE_F12;
      case KEY_F13:
        return KEYCODE_F13;
      case KEY_F14:
        return KEYCODE_F14;
      case KEY_F15:
        return KEYCODE_F15;
      case KEY_F16:
        return KEYCODE_F16;
      case KEY_F17:
        return KEYCODE_F17;
      case KEY_F18:
        return KEYCODE_F18;
      case KEY_F19:
        return KEYCODE_F19;
      case KEY_F20:
        return KEYCODE_F20;
      case KEY_F21:
        return KEYCODE_F21;
      case KEY_F22:
        return KEYCODE_F22;
      case KEY_F23:
        return KEYCODE_F23;
      case KEY_F24:
        return KEYCODE_F24;
      default:
    }

  *special = false;
  switch (keycode)
    {
       case KEY_ESC:
        return '\e';
      case KEY_1:
        return '1';
      case KEY_2:
        return '2';
      case KEY_3:
        return '3';
      case KEY_4:
        return '4';
      case KEY_5:
        return '5';
      case KEY_6:
        return '6';
      case KEY_7:
        return '7';
      case KEY_8:
        return '8';
      case KEY_9:
        return '9';
      case KEY_0:
        return '0';
      case KEY_MINUS:
        return '-';
      case KEY_EQUAL:
        return '=';
      case KEY_TAB:
        return '\t';
      case KEY_Q:
        return 'q';
      case KEY_W:
        return 'w';
      case KEY_E:
        return 'e';
      case KEY_R:
        return 'r';
      case KEY_T:
        return 't';
      case KEY_Y:
        return 'y';
      case KEY_U:
        return 'u';
      case KEY_I:
        return 'i';
      case KEY_O:
        return 'o';
      case KEY_P:
        return 'p';
      case KEY_LEFTBRACE:
        return '(';
      case KEY_RIGHTBRACE:
        return ')';
      case KEY_A:
        return 'a';
      case KEY_S:
        return 's';
      case KEY_D:
        return 'd';
      case KEY_F:
        return 'f';
      case KEY_G:
        return 'g';
      case KEY_H:
        return 'h';
      case KEY_J:
        return 'j';
      case KEY_K:
        return 'k';
      case KEY_L:
        return 'l';
      case KEY_SEMICOLON:
        return ';';
      case KEY_APOSTROPHE:
        return '\'';
      case KEY_BACKSLASH:
        return '\\';
      case KEY_Z:
        return 'z';
      case KEY_X:
        return 'x';
      case KEY_C:
        return 'c';
      case KEY_V:
        return 'v';
      case KEY_B:
        return 'b';
      case KEY_N:
        return 'n';
      case KEY_M:
        return 'm';
      case KEY_COMMA:
        return ',';
      case KEY_DOT:
        return '.';
      case KEY_SLASH:
        return '/';
      case KEY_KPASTERISK:
        return '*';
      case KEY_SPACE:
        return ' ';
      case KEY_KP7:
        return '7';
      case KEY_KP8:
        return '8';
      case KEY_KP9:
        return '9';
      case KEY_KPMINUS:
        return '-';
      case KEY_KP4:
        return '4';
      case KEY_KP5:
        return '5';
      case KEY_KP6:
        return '6';
      case KEY_KPPLUS:
        return '+';
      case KEY_KP1:
        return '1';
      case KEY_KP2:
        return '2';
      case KEY_KP3:
        return '3';
      case KEY_KP0:
        return '0';
      case KEY_KPDOT:
        return '.';
      default:
    }

  return 0;
}
