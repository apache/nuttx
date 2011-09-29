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
#include <pthread.h>

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

extern int up_buttonevent(int x, int y, int buttons);
extern int up_tcleave(int x, int y, int buttons);

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/* Defined in up_x11framebuffer.c */

extern Display *g_display;
extern Window g_window;

pthread_t g_eventloop;
volatile int g_evloopactive;

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ***************************************************************************/

/****************************************************************************
 * Name: up_buttonmap
 ***************************************************************************/

static int up_buttonmap(int state)
{
  /* Remove any X11 dependencies.  Just maps Button1Mask to bit 0. */

  return ((state & Button1Mask) != 0) ? 1 : 0;
}

/****************************************************************************
 * Name: up_x11eventthread
 ***************************************************************************/

static void *up_x11eventthread(void *arg)
{
  Window window;
  XEvent event;

  /* Release queued events on the display */

  (void)XAllowEvents(g_display, AsyncBoth, CurrentTime);

  /* Grab mouse button 1, enabling mouse-related events */

  window = DefaultRootWindow(g_display);
  (void)XGrabButton(g_display, Button1, AnyModifier, window, 1,
                    ButtonPressMask|ButtonReleaseMask|ButtonMotionMask,
                    GrabModeAsync, GrabModeAsync, None, None);

  /* Then loop until we are commanded to stop (when g_evloopactive becomes zero),
   * waiting for events and processing events as they are received.
   */
 
  while ( g_evloopactive)
    {
      XNextEvent(g_display, &event);
      switch (event.type)
        {
          case MotionNotify : /* Enabled by ButtonMotionMask */
            {
              up_buttonevent(event.xmotion.x, event.xmotion.y,
                             up_buttonmap(event.xmotion.state));
            }
            break;

          case ButtonPress  : /* Enabled by ButtonPressMask */
          case ButtonRelease : /* Enabled by ButtonReleaseMask */
            {
              up_buttonevent(event.xbutton.x, event.xbutton.y,
                             up_buttonmap(event.xbutton.state));
            }
            break;

          default :
            break;
        }
    }

  XUngrabButton(g_display, Button1, AnyModifier, window);
  return NULL;
}

/****************************************************************************
 * Name: up_x11eventloop
 ***************************************************************************/

int up_x11eventloop(void)
{
  /* Start the X11 event loop */

  g_evloopactive = 1;
  return pthread_create(&g_eventloop, 0, up_x11eventthread, 0);
}


