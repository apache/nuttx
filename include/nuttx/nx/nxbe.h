/****************************************************************************
 * include/nuttx/nx/nxbe.h
 *
 *   Copyright (C) 2008-2011, 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __INCLUDE_NUTTX_NX_NXBE_H
#define __INCLUDE_NUTTX_NX_NXBE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxglib.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifndef CONFIG_NX_NPLANES
#  define CONFIG_NX_NPLANES      1  /* Max number of color planes supported */
#endif

#ifndef CONFIG_NX_NCOLORS
#  define CONFIG_NX_NCOLORS 256
#endif

/* NXBE Definitions *********************************************************/
/* Window flags and helper macros */

#define NXBE_WINDOW_BLOCKED  (1 << 0) /* The window is blocked and will not
                                       * receive further input. */

#define NXBE_ISBLOCKED(wnd)  (((wnd)->flags & NXBE_WINDOW_BLOCKED) != 0)
#define NXBE_SETBLOCKED(wnd) do { (wnd)->flags |= NXBE_WINDOW_BLOCKED; } while (0)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Windows ******************************************************************/

/* This structure represents one window.  This is the "base" form of the
 * opaque types NXWINDOW and NXTKWINDOW.  Any Window implementatin must
 * be described with a structure that is at least cast-compatible with
 * struct nxbe_window_s.
 */

struct nxbe_state_s;
struct nxfe_conn_s;
struct nxbe_window_s
{
  /* State information */

  FAR struct nxbe_state_s *be;        /* The back-end state structure */
#ifdef CONFIG_NX_MULTIUSER
  FAR struct nxfe_conn_s *conn;       /* Connection to the window client */
#endif
  FAR const struct nx_callback_s *cb; /* Event handling callbacks */

  /* The following links provide the window's vertical position using a
   * singly linked list.
   */

  FAR struct nxbe_window_s *above;    /* The window "above" this window */
  FAR struct nxbe_window_s *below;    /* The window "below this one */

  /* Window geometry.  The window is described by a rectangle in the
   * absolute screen coordinate system (0,0)->(xres,yres)
   */

  struct nxgl_rect_s bounds;          /* The bounding rectangle of window */

  /* Window flags (see the NXBE_* bit definitions above) */

#ifdef CONFIG_NX_MULTIUSER            /* Currently used only in multi-user mode */
  uint8_t flags;
#endif

  /* Client state information this is provide in window callbacks */

  FAR void *arg;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_NX_NXBE_H */

