/****************************************************************************
 * arch/sim/src/sim/posix/sim_x11eventloop.c
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

#ifdef CONFIG_SIM_TOUCHSCREEN
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
#endif

/****************************************************************************
 * Name: sim_mousebuttonmap
 *
 * Description:
 *   Same idea as sim_buttonmap() above, but producing a bitmap in
 *   struct mouse_report_s's MOUSE_BUTTON_1/2/3 numbering (see
 *   include/nuttx/input/mouse.h) instead of touch_sample_s's single
 *   undifferentiated "pen down" bit -- the whole point of
 *   CONFIG_SIM_MOUSE over CONFIG_SIM_TOUCHSCREEN is to keep which
 *   physical button was pressed. Not including that header here since
 *   it's the only thing that would pull NuttX input headers into this
 *   host-compiled (HOSTSRCS) file; the three bit values are hand-rolled
 *   instead, matching MOUSE_BUTTON_1=1, MOUSE_BUTTON_2=2, MOUSE_BUTTON_3=4.
 *
 *   Note X11's own button numbering has Button2 as the *middle* button
 *   and Button3 as the *right* button (Button1 is left, same as
 *   everywhere else) -- MOUSE_BUTTON_2/3 need the opposite assignment
 *   (right, then middle) to match, so this cannot just reuse
 *   sim_buttonmap()'s bit positions.
 ****************************************************************************/

#ifdef CONFIG_SIM_MOUSE
static int sim_mousebuttonmap(int state, int button)
{
  int buttons = 0;

  if ((state & Button1Mask) != 0)
    {
      buttons |= 1;   /* MOUSE_BUTTON_1: left */
    }

  if ((state & Button2Mask) != 0)
    {
      buttons |= 4;   /* MOUSE_BUTTON_3: middle */
    }

  if ((state & Button3Mask) != 0)
    {
      buttons |= 2;   /* MOUSE_BUTTON_2: right */
    }

  switch (button)
    {
      case Button1:
        buttons ^= 1;
        break;
      case Button2:
        buttons ^= 4;
        break;
      case Button3:
        buttons ^= 2;
        break;
    }

  return buttons;
}
#endif

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

  /* Dequeue any pending X11 events. */

  while (g_display && XPending(g_display) > 0)
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

#ifdef CONFIG_SIM_TOUCHSCREEN
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
#endif

#ifdef CONFIG_SIM_MOUSE
          case MotionNotify : /* Enabled by PointerMotionMask */
            {
              sim_mouseevent(event.xmotion.x, event.xmotion.y,
                             sim_mousebuttonmap(event.xmotion.state, 0), 0);
            }
            break;

          case ButtonPress  : /* Enabled by ButtonPressMask */
          case ButtonRelease: /* Enabled by ButtonReleaseMask */
            {
              /* Buttons 4/5 are the traditional X11 scroll-wheel
               * encoding: a Press/Release pair fires on every "click"
               * of the wheel, with no held state of its own -- report
               * the wheel delta on Press only (matching how a real
               * mouse_report_s sample is consumed once, not polled as
               * persistent state), and pass through the *other*
               * buttons' current state unchanged either way.
               */

              int wheel = 0;

              if (event.type == ButtonPress && event.xbutton.button == 4)
                {
                  wheel = 1;
                }
              else if (event.type == ButtonPress &&
                       event.xbutton.button == 5)
                {
                  wheel = -1;
                }

              sim_mouseevent(event.xbutton.x, event.xbutton.y,
                             sim_mousebuttonmap(event.xbutton.state,
                                                event.xbutton.button),
                             wheel);
            }
            break;
#endif

          default:
            break;
        }
    }
}
