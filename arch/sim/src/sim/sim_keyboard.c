/****************************************************************************
 * arch/sim/src/sim/sim_keyboard.c
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

#include <nuttx/config.h>
#include <sys/types.h>

#include <assert.h>
#include <nuttx/debug.h>
#include <string.h>

#include <nuttx/input/keyboard.h>
#include <nuttx/input/kbd_codec.h>

#include "sim_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DEVNAME "/dev/kbd"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sim_dev_s
{
  int eventloop;
  struct keyboard_lowerhalf_s lower;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Only one simulated keyboard is supported so the driver state
 * structure may as well be pre-allocated.
 */

static struct sim_dev_s g_simkeyboard;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: translate_x11_key
 *
 * Description:
 *   Translate X11 key codes into the NuttX keyboard codec so that
 *   applications can actually rely on the output of the keyboard driver.
 *   Some X11 key codes do not need translation.
 *
 * Returns: The translated key code.
 ****************************************************************************/

static uint32_t translate_x11_key(uint32_t code)
{
  switch (code)
    {
    case XK_Delete:
      return KEYCODE_FWDDEL;
    case XK_BackSpace:
      return KEYCODE_BACKDEL;

      /* Cursor movement */

    case XK_Home:
      return KEYCODE_HOME;
    case XK_End:
      return KEYCODE_END;
    case XK_Left:
      return KEYCODE_LEFT;
    case XK_Right:
      return KEYCODE_RIGHT;
    case XK_Up:
      return KEYCODE_UP;
    case XK_Down:
      return KEYCODE_DOWN;
    case XK_Page_Up:
      return KEYCODE_PAGEUP;
    case XK_Page_Down:
      return KEYCODE_PAGEDOWN;

      /* Edit commands */

    case XK_Insert:
      return KEYCODE_INSERT;
    case XK_Undo:
      return KEYCODE_UNDO;
    case XK_Redo:
      return KEYCODE_REDO;
    case XK_Find:
      return KEYCODE_FIND;

      /* Selection codes */

    case XK_Return:
    case XK_KP_Enter:
      return KEYCODE_ENTER;
    case XK_Select:
      return KEYCODE_SELECT;
    case XK_Execute:
      return KEYCODE_EXECUTE;

      /* Keyboard modes */

    case XK_Caps_Lock:
      return KEYCODE_CAPSLOCK;
    case XK_Scroll_Lock:
      return KEYCODE_SCROLLLOCK;
    case XK_Num_Lock:
      return KEYCODE_NUMLOCK;

      /* Misc control codes */

    case XK_Help:
      return KEYCODE_HELP;
    case XK_Menu:
      return KEYCODE_MENU;
    case XK_Pause:
      return KEYCODE_PAUSE;
    case XK_Break:
      return KEYCODE_BREAK;
    case XK_Cancel:
      return KEYCODE_CANCEL;
    case XK_Print:
      return KEYCODE_PRTSCRN;
    case XK_Sys_Req:
      return KEYCODE_SYSREQ;

      /* Context-specific function keys */

    case XK_F1:
      return KEYCODE_F1;
    case XK_F2:
      return KEYCODE_F2;
    case XK_F3:
      return KEYCODE_F3;
    case XK_F4:
      return KEYCODE_F4;
    case XK_F5:
      return KEYCODE_F5;
    case XK_F6:
      return KEYCODE_F6;
    case XK_F7:
      return KEYCODE_F7;
    case XK_F8:
      return KEYCODE_F8;
    case XK_F9:
      return KEYCODE_F9;
    case XK_F10:
      return KEYCODE_F10;
    case XK_F11:
      return KEYCODE_F11;
    case XK_F12:
      return KEYCODE_F12;
    case XK_F13:
      return KEYCODE_F13;
    case XK_F14:
      return KEYCODE_F14;
    case XK_F15:
      return KEYCODE_F15;
    case XK_F16:
      return KEYCODE_F16;
    case XK_F17:
      return KEYCODE_F17;
    case XK_F18:
      return KEYCODE_F18;
    case XK_F19:
      return KEYCODE_F19;
    case XK_F20:
      return KEYCODE_F20;
    case XK_F21:
      return KEYCODE_F21;
    case XK_F22:
      return KEYCODE_F22;
    case XK_F23:
      return KEYCODE_F23;
    case XK_F24:
      return KEYCODE_F24;

    default:
      return code;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_kbd_initialize
 ****************************************************************************/

int sim_kbd_initialize(void)
{
  int              ret;
  struct sim_dev_s *priv = &g_simkeyboard;

  memset(priv, 0, sizeof(*priv));

  /* Register the device as an input device */

  ret = keyboard_register(&priv->lower, DEVNAME,
                          CONFIG_SIM_KEYBOARD_BUFFSIZE);
  if (ret < 0)
    {
      ierr("ERROR: keyboard_register() failed: %d\n", ret);
      return ret;
    }

  /* Enable X11 event processing from the IDLE loop */

  priv->eventloop = 1;
  return OK;
}

/****************************************************************************
 * Name: sim_kbdevent
 ****************************************************************************/

void sim_kbdevent(uint32_t key, bool is_press)
{
  uint32_t trans_key;
  struct sim_dev_s *priv = (struct sim_dev_s *) &g_simkeyboard;
  uint32_t types[2] =
    {
      KEYBOARD_RELEASE, KEYBOARD_PRESS
    };

  if (priv->eventloop == 0)
    {
      return;
    }

  trans_key = translate_x11_key(key);
  iinfo("key=%04x\n", key);

  /* Report data changes */

  keyboard_event(&priv->lower, trans_key, types[is_press]);
}
