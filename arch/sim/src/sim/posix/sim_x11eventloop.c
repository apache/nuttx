/****************************************************************************
 * arch/sim/src/sim/posix/sim_x11eventloop.c
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

#include <stdio.h>
#include <X11/Xlib.h>
#include <X11/keysym.h>

#include "sim_internal.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Defined in sim_x11framebuffer.c */

extern Display *g_display;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_buttonmap
 ****************************************************************************/

static int sim_buttonmap(int state, int button)
{
  int buttons = 0;

  /* "state" is a bitmask representing the state prior to the event, so
   * translate that first to our button bitmap
   */

  if ((state & Button1Mask) != 0)
    {
      buttons |= 1;
    }

  if ((state & Button2Mask) != 0)
    {
      buttons |= 2;
    }

  if ((state & Button3Mask) != 0)
    {
      buttons |= 4;
    }

  /* button represents the button which changed state, so change the
   * corresponding bit now
   */

  switch (button)
    {
      case Button1:
        buttons ^= 1;
        break;
      case Button2:
        buttons ^= 2;
        break;
      case Button3:
        buttons ^= 4;
        break;
    }

  return buttons;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_x11events
 *
 * Description:
 *   Called periodically from the IDLE loop to check for queued X11 events.
 *
 ****************************************************************************/

void sim_x11events(void)
{
  XEvent event;

  /* Check if there are any pending, queue X11 events. */

  if (g_display && XPending(g_display) > 0)
    {
      /* Yes, get the event (this should not block since we know there are
       * pending events)
       */

      XNextEvent(g_display, &event);

      /* Then process the event */

      switch (event.type)
        {
          #ifdef CONFIG_SIM_KEYBOARD
          case KeyPress:
            sim_kbdevent(XLookupKeysym(&event.xkey, 0), true);
            break;
          case KeyRelease:
            sim_kbdevent(XLookupKeysym(&event.xkey, 0), false);
            break;
          #endif

          case MotionNotify : /* Enabled by ButtonMotionMask */
            {
              sim_buttonevent(event.xmotion.x, event.xmotion.y,
                              sim_buttonmap(event.xmotion.state, 0));
            }
            break;

          case ButtonPress  : /* Enabled by ButtonPressMask */
          case ButtonRelease: /* Enabled by ButtonReleaseMask */
            {
              sim_buttonevent(event.xbutton.x, event.xbutton.y,
                              sim_buttonmap(event.xbutton.state,
                                          event.xbutton.button));
            }
            break;

          default :
            break;
        }
    }
}
