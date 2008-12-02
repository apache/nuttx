/****************************************************************************
 * graphics/nxtk/nxtk_internal.h
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
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

#ifndef __GRAPHICS_NXTK_NXTK_INTERNAL_H
#define __GRAPHICS_NXTK_NXTK_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

#include <nuttx/nxtk.h>
#include "nxbe.h"
#include "nxfe.h"

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_NXTK_BORDERWIDTH
#  define CONFIG_NXTK_BORDERWIDTH 2
#endif

#ifndef CONFIG_NXTK_BORDERCOLOR1
#  define CONFIG_NXTK_BORDERCOLOR1 0x00a9a9a9
#endif

#ifndef CONFIG_NXTK_BORDERCOLOR2
#  define CONFIG_NXTK_BORDERCOLOR2 0x00696969
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This is the internal representation of the framed window object */

struct nxtk_framedwindow_s
{
  struct nxbe_window_s wnd;      /* The raw NX window */

  /* The toolbar region and callbacks */

  nxgl_coord_t tbheight;
  struct nxgl_rect_s tbrect;
  FAR const struct nx_callback_s *tbcb;
  FAR void *tbarg;

  /* Window data region and callbacks */

  struct nxgl_rect_s fwrect;
  FAR const struct nx_callback_s *fwcb;
  FAR void *fwarg;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
# define EXTERN extern "C"
extern "C" {
#else
# define EXTERN extern
#endif

/* That is the callback for the framed window */

extern FAR const struct nx_callback_s g_nxtkcb;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nxtk_setsubwindows
 *
 * Description:
 *   Give the window dimensions, border width, and toolbar height,
 *   calculate the new dimensions of the toolbar region and client window
 *   region
 *
 ****************************************************************************/

EXTERN void nxtk_setsubwindows(FAR struct nxtk_framedwindow_s *fwnd);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __GRAPHICS_NXTK_NXTK_INTERNAL_H */
