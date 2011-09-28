/****************************************************************************
 * arch/sim/src/up_x11eventloop.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdio.h>
#include <X11/Xlib.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ***************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

extern int up_tcenter(int x, int y, int buttons);
extern int up_tcleave(int x, int y, int buttons);

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/* Defined in up_x11framebuffer.c */

extern Display *g_display;
extern Window g_window;

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ***************************************************************************/

/****************************************************************************
 * Name: up_x11eventloop
 ***************************************************************************/

static int up_buttonmap(int state)
{
  int ret = 0;

  /* Remove any X11 dependencies.  Possible bit settings include:  Button1Mask,
   * Button2Mask, Button3Mask, Button4Mask, Button5Mask, ShiftMask, LockMask,
   * ControlMask, Mod1Mask, Mod2Mask, Mod3Mask, Mod4Mask, Mod5Mask.  I assume
   * that for a mouse device Button1Mask, Button2Mask, and Button3Mask are
   * sufficient.
   */

  if ((state & Button1Mask) != 0)
    {
      ret |= 1;
    }

  if ((state & Button2Mask) != 0)
    {
      ret |= 2;
    }

  if ((state & Button3Mask) != 0)
    {
      ret |= 4;
    }
  return ret;
}

/****************************************************************************
 * Public Functions
 ***************************************************************************/

/****************************************************************************
 * Name: up_x11eventloop
 ***************************************************************************/

int up_x11eventloop(int argc, char *argv[])
{
  XEvent event;
  int ret;

  /* Grab the pointer (mouse) */

  ret = XGrabPointer(g_display, g_window, 0,
                     EnterWindowMask|LeaveWindowMask,
                     GrabModeAsync, GrabModeAsync, None, None, CurrentTime);
  if (ret != GrabSuccess)
    {
      fprintf(stderr, "Failed grap pointer\n");
      return -1;
    }

  /* Then loop forever, waiting for events and processing events as they are
   * received.
   */
 
  for (;;)
    {
      XNextEvent(g_display, &event);
      switch (event.type)
        {
          case EnterNotify:
            {
              fprintf(stderr, "EnterNotify event: (%d,%d) %08x\n",
                      event.xcrossing.x, event.xcrossing.y, event.xcrossing.state);
              up_tcenter(event.xcrossing.x, event.xcrossing.y,
                         up_buttonmap(event.xcrossing.state));
            }
            break;

          case LeaveNotify :
            {
              fprintf(stderr, "LeaveNotify event: (%d,%d) %08x\n",
                      event.xcrossing.x, event.xcrossing.y, event.xcrossing.state);
              up_tcleave(event.xcrossing.x, event.xcrossing.y,
                         up_buttonmap(event.xcrossing.state));
            }
            break;

          default :
            fprintf(stderr, "Unrecognized event: %d\n", event.type);
            break;
        }
    }
  return 0;
}
