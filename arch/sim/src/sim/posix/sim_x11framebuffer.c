/****************************************************************************
 * arch/sim/src/sim/posix/sim_x11framebuffer.c
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
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <errno.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <X11/extensions/XShm.h>

#include "sim_internal.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

typedef union
{
  struct
    {
      uint16_t blue : 5;
      uint16_t green : 6;
      uint16_t red : 5;
    };
  uint16_t full;
} x11_color16_t;

typedef union
{
  struct
    {
      uint8_t blue;
      uint8_t green;
      uint8_t red;
      uint8_t alpha;
    };
  uint32_t full;
} x11_color32_t;

/* Also used in sim_x11eventloop */

Display *g_display;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int g_screen;
static Window g_window;
static GC g_gc;
#ifndef CONFIG_SIM_X11NOSHM
static XShmSegmentInfo g_xshminfo;
static int g_xerror;
#endif
static XImage *g_image;
static char *g_framebuffer;
static unsigned short g_fbpixelwidth;
static unsigned short g_fbpixelheight;
static int g_fbbpp;
static int g_fblen;
static int g_fbcount;
static int g_shmcheckpoint = 0;
static int b_useshm;

static unsigned char *g_trans_framebuffer;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_x11createframe
 ****************************************************************************/

static inline Display *sim_x11createframe(void)
{
  Display *display;
  XGCValues gcval;
  char *argv[2] =
    {
      "nuttx", NULL
    };

  char *winname = "NuttX";
  char *iconname = "NX";
  XTextProperty winprop;
  XTextProperty iconprop;
  XSizeHints hints;

  display = XOpenDisplay(NULL);
  if (display == NULL)
    {
      syslog(LOG_ERR, "Unable to open display.\n");
      return NULL;
    }

  g_screen = DefaultScreen(display);
  g_window = XCreateSimpleWindow(display, DefaultRootWindow(display),
                                 0, 0, g_fbpixelwidth, g_fbpixelheight, 2,
                                 BlackPixel(display, g_screen),
                                 BlackPixel(display, g_screen));

  XStringListToTextProperty(&winname, 1, &winprop);
  XStringListToTextProperty(&iconname, 1, &iconprop);

  hints.flags  = PSize | PMinSize | PMaxSize;
  hints.width  = hints.min_width  = hints.max_width  = g_fbpixelwidth;
  hints.height = hints.min_height = hints.max_height = g_fbpixelheight;

  XSetWMProperties(display, g_window, &winprop, &iconprop, argv, 1,
                   &hints, NULL, NULL);
  XFree(winprop.value);
  XFree(iconprop.value);

  /* Select window input events */

#if defined(CONFIG_SIM_AJOYSTICK)
  XSelectInput(display, g_window,
               ButtonPressMask | ButtonReleaseMask | PointerMotionMask);
#else
  XSelectInput(display, g_window,
               ButtonPressMask | ButtonReleaseMask | PointerMotionMask |
               KeyPressMask | KeyReleaseMask);
#endif

  /* Release queued events on the display */

#if defined(CONFIG_SIM_TOUCHSCREEN) || defined(CONFIG_SIM_AJOYSTICK) || \
    defined(CONFIG_SIM_BUTTONS)
  XAllowEvents(display, AsyncBoth, CurrentTime);

  /* Grab mouse button 1, enabling mouse-related events */

  XGrabButton(display, Button1, AnyModifier, g_window, 1,
              ButtonPressMask | ButtonReleaseMask | ButtonMotionMask,
              GrabModeAsync, GrabModeAsync, None, None);
#endif

  gcval.graphics_exposures = 0;
  g_gc = XCreateGC(display, g_window, GCGraphicsExposures, &gcval);

  return display;
}

/****************************************************************************
 * Name: sim_x11errorhandler
 ****************************************************************************/

#ifndef CONFIG_SIM_X11NOSHM
static int sim_x11errorhandler(Display *display, XErrorEvent *event)
{
  g_xerror = 1;
  return 0;
}
#endif

/****************************************************************************
 * Name: sim_x11traperrors
 ****************************************************************************/

#ifndef CONFIG_SIM_X11NOSHM
static void sim_x11traperrors(void)
{
  g_xerror = 0;
  XSetErrorHandler(sim_x11errorhandler);
}
#endif

/****************************************************************************
 * Name: sim_x11untraperrors
 ****************************************************************************/

#ifndef CONFIG_SIM_X11NOSHM
static int sim_x11untraperrors(Display *display)
{
  XSync(display, 0);
  XSetErrorHandler(NULL);
  return g_xerror;
}
#endif

/****************************************************************************
 * Name: sim_x11uninit
 ****************************************************************************/

static void sim_x11uninit(void)
{
  if (g_display == NULL)
    {
      return;
    }

#ifndef CONFIG_SIM_X11NOSHM
  if (g_shmcheckpoint > 4)
    {
      XShmDetach(g_display, &g_xshminfo);
    }

  if (g_shmcheckpoint > 3)
    {
      shmdt(g_xshminfo.shmaddr);
    }

  if (g_shmcheckpoint > 2)
    {
      shmctl(g_xshminfo.shmid, IPC_RMID, 0);
    }
#endif

  if (g_shmcheckpoint > 1)
    {
#ifdef CONFIG_SIM_X11NOSHM
      g_image->data = g_framebuffer;
#endif
      XDestroyImage(g_image);
    }

  /* Un-grab the mouse buttons */

#if defined(CONFIG_SIM_TOUCHSCREEN) || defined(CONFIG_SIM_AJOYSTICK) || \
    defined(CONFIG_SIM_BUTTONS)
  XUngrabButton(g_display, Button1, AnyModifier, g_window);
#endif

  XCloseDisplay(g_display);
}

/****************************************************************************
 * Name: sim_x11uninitialize
 ****************************************************************************/

#ifndef CONFIG_SIM_X11NOSHM
static void sim_x11uninitialize(void)
{
  if (g_shmcheckpoint > 1)
    {
      if (!b_useshm && g_framebuffer)
        {
          free(g_framebuffer);
          g_framebuffer = 0;
        }
    }

  if (g_shmcheckpoint > 0)
    {
      g_shmcheckpoint = 1;
    }
}
#endif

/****************************************************************************
 * Name: sim_x11mapsharedmem
 ****************************************************************************/

static inline int sim_x11mapsharedmem(Display *display,
                                      int depth, unsigned int fblen,
                                      int fbcount)
{
#ifndef CONFIG_SIM_X11NOSHM
  Status result;
#endif

  atexit(sim_x11uninit);
  g_shmcheckpoint = 1;
  b_useshm = 0;

#ifndef CONFIG_SIM_X11NOSHM
  if (XShmQueryExtension(display))
    {
      b_useshm = 1;

      sim_x11traperrors();
      g_image = XShmCreateImage(display,
                                DefaultVisual(display, g_screen),
                                depth, ZPixmap, NULL, &g_xshminfo,
                                g_fbpixelwidth, g_fbpixelheight);
      if (sim_x11untraperrors(display))
        {
          sim_x11uninitialize();
          goto shmerror;
        }

      if (!g_image)
        {
          syslog(LOG_ERR, "Unable to create g_image.\n");
          return -1;
        }

      g_shmcheckpoint++;

      g_xshminfo.shmid = shmget(IPC_PRIVATE,
                                g_image->bytes_per_line *
                                g_image->height * fbcount,
                                IPC_CREAT | 0777);
      if (g_xshminfo.shmid < 0)
        {
          sim_x11uninitialize();
          goto shmerror;
        }

      g_shmcheckpoint++;

      g_image->data = (char *) shmat(g_xshminfo.shmid, 0, 0);
      if (g_image->data == ((char *) -1))
        {
          sim_x11uninitialize();
          goto shmerror;
        }

      g_shmcheckpoint++;

      g_xshminfo.shmaddr = g_image->data;
      g_xshminfo.readOnly = 0;

      sim_x11traperrors();
      result = XShmAttach(display, &g_xshminfo);
      if (sim_x11untraperrors(display) || !result)
        {
          sim_x11uninitialize();
          goto shmerror;
        }

      g_framebuffer = g_image->data;
      g_shmcheckpoint++;
    }
  else
#endif
  if (!b_useshm)
    {
#ifndef CONFIG_SIM_X11NOSHM
shmerror:
#endif
      b_useshm = 0;

      g_framebuffer = malloc(fblen * fbcount);

      g_image = XCreateImage(display, DefaultVisual(display, g_screen),
                             depth, ZPixmap, 0, g_framebuffer,
                             g_fbpixelwidth, g_fbpixelheight,
                             8, 0);

      if (g_image == NULL)
        {
          syslog(LOG_ERR, "Unable to create g_image\n");
          return -1;
        }

      g_shmcheckpoint++;
    }

  return 0;
}

/****************************************************************************
 * Name: sim_x11depth16to32
 ****************************************************************************/

static inline void sim_x11depth16to32(void *d_mem, size_t size,
                                      const void *s_mem)
{
  x11_color32_t *dst = d_mem;
  const x11_color16_t *src = s_mem;

  for (size /= 4; size; size--)
    {
      dst->red = (src->red * 263 + 7) >> 5;
      dst->green = (src->green * 259 + 3) >> 6;
      dst->blue = (src->blue * 263 + 7) >> 5;
      dst->alpha = 0xff;
      dst++;
      src++;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_x11initialize
 *
 * Description:
 *   Make an X11 window look like a frame buffer.
 *
 ****************************************************************************/

int sim_x11initialize(unsigned short width, unsigned short height,
                     void **fbmem, size_t *fblen, unsigned char *bpp,
                     unsigned short *stride, int fbcount)
{
  XWindowAttributes windowattributes;
  Display *display;
  int depth;

  /* Save inputs */

  g_fbpixelwidth  = width;
  g_fbpixelheight = height;

  if (fbcount < 1)
    {
      return -EINVAL;
    }

  /* Create the X11 window */

  display = sim_x11createframe();
  if (display == NULL)
    {
      return -ENODEV;
    }

  /* Determine the supported pixel bpp of the current window */

  XGetWindowAttributes(display, DefaultRootWindow(display),
                       &windowattributes);

  /* Get the pixel depth.  If the depth is 24-bits, use 32 because X expects
   * 32-bit alignment anyway.
   */

  depth = windowattributes.depth;
  if (depth == 24)
    {
      depth = 32;
    }

  if (depth != CONFIG_SIM_FBBPP && depth != 32 && CONFIG_SIM_FBBPP != 16)
    {
      return -1;
    }

  *bpp    = depth;
  *stride = (depth * width / 8);
  *fblen  = (*stride * height);

  /* Map the window to shared memory */

  sim_x11mapsharedmem(display, windowattributes.depth, *fblen, fbcount);

  g_fbbpp = depth;
  g_fblen = *fblen;
  g_fbcount = fbcount;

  /* Create conversion framebuffer */

  if (depth == 32 && CONFIG_SIM_FBBPP == 16)
    {
      *bpp = CONFIG_SIM_FBBPP;
      *stride = (CONFIG_SIM_FBBPP * width / 8);
      *fblen = (*stride * height);

      g_trans_framebuffer = malloc((*fblen) * fbcount);
      if (g_trans_framebuffer == NULL)
        {
          syslog(LOG_ERR, "Failed to allocate g_trans_framebuffer\n");
          return -1;
        }

      *fbmem = g_trans_framebuffer;
    }
  else
    {
      *fbmem = g_framebuffer;
    }

  g_display = display;
  return 0;
}

/****************************************************************************
 * Name: sim_x11openwindow
 ****************************************************************************/

int sim_x11openwindow(void)
{
  if (g_display == NULL)
    {
      return -ENODEV;
    }

  XMapWindow(g_display, g_window);
  XSync(g_display, 0);

  return 0;
}

/****************************************************************************
 * Name: sim_x11closewindow
 ****************************************************************************/

int sim_x11closewindow(void)
{
  if (g_display == NULL)
    {
      return -ENODEV;
    }

  XUnmapWindow(g_display, g_window);
  XSync(g_display, 0);

  return 0;
}

/****************************************************************************
 * Name: sim_x11setoffset
 ****************************************************************************/

int sim_x11setoffset(unsigned int offset)
{
  if (g_display == NULL)
    {
      return -ENODEV;
    }

  if (g_fbbpp == 32 && CONFIG_SIM_FBBPP == 16)
    {
      g_image->data = g_framebuffer + (offset << 1);
    }
  else
    {
      g_image->data = g_framebuffer + offset;
    }

  return 0;
}

/****************************************************************************
 * Name: sim_x11cmap
 ****************************************************************************/

int sim_x11cmap(unsigned short first, unsigned short len,
               unsigned char *red, unsigned char *green,
               unsigned char *blue, unsigned char  *transp)
{
  Colormap cmap;
  int ndx;

  if (g_display == NULL)
    {
      return -ENODEV;
    }

  /* Convert each color to X11 scaling */

  cmap = DefaultColormap(g_display, g_screen);
  for (ndx = first; ndx < first + len; ndx++)
    {
      XColor color;

      /* Convert to RGB.  In the NuttX cmap, each component
       * ranges from 0-255; for X11 the range is 0-65536
       */

      color.red   = (short)(*red++) << 8;
      color.green = (short)(*green++) << 8;
      color.blue  = (short)(*blue++) << 8;
      color.flags = DoRed | DoGreen | DoBlue;

      /* Then allocate a color for this selection */

      if (!XAllocColor(g_display, cmap, &color))
        {
          syslog(LOG_ERR, "Failed to allocate color%d\n", ndx);
          return -1;
        }
    }

  return 0;
}

/****************************************************************************
 * Name: sim_x11update
 ****************************************************************************/

int sim_x11update(void)
{
  if (g_display == NULL)
    {
      return -ENODEV;
    }

#ifndef CONFIG_SIM_X11NOSHM
  if (b_useshm)
    {
      XShmPutImage(g_display, g_window, g_gc, g_image, 0, 0, 0, 0,
                   g_fbpixelwidth, g_fbpixelheight, 0);
    }
  else
#endif
    {
      XPutImage(g_display, g_window, g_gc, g_image, 0, 0, 0, 0,
                g_fbpixelwidth, g_fbpixelheight);
    }

  if (g_fbbpp == 32 && CONFIG_SIM_FBBPP == 16)
    {
      sim_x11depth16to32(g_framebuffer,
                         g_fblen * g_fbcount,
                         g_trans_framebuffer);
    }

  XSync(g_display, 0);

  return 0;
}
