/****************************************************************************
 * include/nuttx/nx/nxterm.h
 *
 *   Copyright (C) 2012, 2019 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_NUTTX_NX_NXTERM_H
#define __INCLUDE_NUTTX_NX_NXTERM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/fs/ioctl.h>
#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxtk.h>

#ifdef CONFIG_NXTERM

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Nx Console prerequistes */

#ifndef CONFIG_NX
#  warning "NX is not enabled (CONFIG_NX)
#endif

/* Nx Console configuration options:
 *
 * CONFIG_NXTERM
 *   Enables building of the NxTerm driver.
 *
 * Output text/graphics options:
 *
 * CONFIG_NXTERM_BPP
 *   Currently, NxTerm supports only a single pixel depth. This
 *   configuration setting must be provided to support that single pixel depth.
 *   Default: The smallest enabled pixel depth. (see CONFIG_NX_DISABLE_*BPP)
 * CONFIG_NXTERM_CURSORCHAR
 *   The bitmap code to use as the cursor.  Default '_'
 * CONFIG_NXTERM_MXCHARS
 *   NxTerm needs to remember every character written to the console so
 *   that it can redraw the window. This setting determines the size of some
 *   internal memory allocations used to hold the character data. Default: 128.
 * CONFIG_NXTERM_CACHESIZE
 *   NxTerm supports caching of rendered fonts. This font caching is required
 *   for two reasons: (1) First, it improves text performance, but more
 *   importantly (2) it preserves the font memory. Since the NX server runs on
 *   a separate server thread, it requires that the rendered font memory persist
 *   until the server has a chance to render the font. (NOTE: There is still
 *   inherently a race condition in this!). Unfortunately, the font cache would
 *   be quite large if all fonts were saved. The CONFIG_NXTERM_CACHESIZE setting
 *   will control the size of the font cache (in number of glyphs). Only that
 *   number of the most recently used glyphs will be retained. Default: 16.
 * CONFIG_NXTERM_LINESEPARATION
 *   This the space (in rows) between each row of test.  Default: 0
 * CONFIG_NXTERM_NOWRAP
 *   By default, lines will wrap when the test reaches the right hand side
 *   of the window. This setting can be defining to change this behavior so
 *   that the text is simply truncated until a new line is  encountered.
 *
 * Input options:
 *
 * CONFIG_NXTERM_NXKBDIN
 *   Take input from the NX keyboard input callback.  By default, keyboard
 *   input is taken from stdin (/dev/console).  If this option is set, then
 *   the interface nxterm_kbdin() is enabled.  That interface may be driven
 *   by window callback functions so that keyboard input *only* goes to the
 *   top window.
 * CONFIG_NXTERM_KBDBUFSIZE
 *   If CONFIG_NXTERM_NXKBDIN is enabled, then this value may be used to
 *   define the size of the per-window keyboard input buffer.  Default: 16
 * CONFIG_NXTERM_NPOLLWAITERS
 *   The number of threads that can be waiting for read data available.
 *   Default: 4
 */

/* Cursor character */

#ifndef CONFIG_NXTERM_CURSORCHAR
#  define CONFIG_NXTERM_CURSORCHAR '_'
#endif

/* The maximum number of characters that can be remembered */

#ifndef CONFIG_NXTERM_MXCHARS
#  define CONFIG_NXTERM_MXCHARS 128
#endif

/* Font cache -- this is the number or pre-rendered font glyphs that can be
 * remembered.
 */

#ifndef CONFIG_NXTERM_CACHESIZE
#  define CONFIG_NXTERM_CACHESIZE 16
#endif

/* Pixel depth */

#if defined(CONFIG_NXTERM_BPP) && \
    CONFIG_NXTERM_BPP != 1 && \
    CONFIG_NXTERM_BPP != 2 && \
    CONFIG_NXTERM_BPP != 4 && \
    CONFIG_NXTERM_BPP != 8 && \
    CONFIG_NXTERM_BPP != 16 && \
    CONFIG_NXTERM_BPP != 32
#  error Invalid selection for CONFIG_NXTERM_BPP
#  undef CONFIG_NXTERM_BPP
#endif

#ifndef CONFIG_NXTERM_BPP
#  if !defined(CONFIG_NX_DISABLE_1BPP)
#    define CONFIG_NXTERM_BPP 1
#  elif !defined(CONFIG_NX_DISABLE_2BPP)
#    define CONFIG_NXTERM_BPP 2
#  elif !defined(CONFIG_NX_DISABLE_4BPP)
#    define CONFIG_NXTERM_BPP 4
#  elif !defined(CONFIG_NX_DISABLE_8BPP)
#    define CONFIG_NXTERM_BPP 8
#  elif !defined(CONFIG_NX_DISABLE_16BPP)
#    define CONFIG_NXTERM_BPP 16
//#elif !defined(CONFIG_NX_DISABLE_24BPP)
//#    define CONFIG_NXTERM_BPP 24
#  elif !defined(CONFIG_NX_DISABLE_32BPP)
#    define CONFIG_NXTERM_BPP 32
#  else
#    error "No pixel depth provided"
#  endif
#endif

/* Space (in rows) between lines */

#ifndef CONFIG_NXTERM_LINESEPARATION
#  define CONFIG_NXTERM_LINESEPARATION 0
#endif

/* Input options */

#ifndef CONFIG_NX_KBD
#  undef CONFIG_NXTERM_NXKBDIN
#endif

#ifdef CONFIG_NXTERM_NXKBDIN

#  ifndef CONFIG_NXTERM_KBDBUFSIZE
#    define CONFIG_NXTERM_KBDBUFSIZE 16
#  elif (CONFIG_NXTERM_KBDBUFSIZE < 1) || (CONFIG_NXTERM_KBDBUFSIZE > 255)
#    error "CONFIG_NXTERM_KBDBUFSIZE is out of range (1-255)"
#  endif

#  ifndef CONFIG_NXTERM_NPOLLWAITERS
#    define CONFIG_NXTERM_NPOLLWAITERS 4
#  endif

#else
#  undef CONFIG_NXTERM_KBDBUFSIZE
#  define CONFIG_NXTERM_KBDBUFSIZE 0
#  define CONFIG_NXTERM_NPOLLWAITERS 0
#endif

/* IOCTL commands ***********************************************************/

/* CMD:           NXTERMIOC_NXTERM_REDRAW
 * DESCRIPTION:   Re-draw a portion of the NX console.  This function
 *                should be called from the appropriate window callback
 *                logic.
 * ARG:           A reference readable instance of struct
 *                nxtermioc_redraw_s
 * CONFIGURATION: CONFIG_NXTERM
 *
 * CMD:           NXTERMIOC_NXTERM_KBDIN
 * DESCRIPTION:   Provide NxTerm keyboard input to NX.
 * ARG:           A reference readable instance of struct nxtermioc_kbdin_s
 * CONFIGURATION: CONFIG_NXTERM_NXKBDIN
 *
 * CMD:           NXTERMIOC_NXTERM_RESIZE
 * DESCRIPTION:   Inform NxTerm keyboard the the size of the window has
 *                changed
 * ARG:           A reference readable instance of struct nxtermioc_resize_s
 * CONFIGURATION: CONFIG_NXTERM
 */

#define _NXTERMIOC(nr)    _IOC(_NXTERMBASE,nr)
#define NXTERMIOC_NXTERM_REDRAW     _NXTERMIOC(0x0000)
#define NXTERMIOC_NXTERM_KBDIN      _NXTERMIOC(0x0001)
#define NXTERMIOC_NXTERM_RESIZE     _NXTERMIOC(0x0002)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This is the handle that can be used to access the consoles */

typedef FAR void *NXTERM;

/* This structure describes the window and font characteristics.
 * For raw windows, wsize if the full size of the window.  For
 * NxTK windows, wsize is the size of the sub-window.
 */

struct nxterm_window_s
{
  nxgl_mxpixel_t wcolor[CONFIG_NX_NPLANES]; /* Window background color */
  nxgl_mxpixel_t fcolor[CONFIG_NX_NPLANES]; /* Font color */
  struct nxgl_size_s wsize;                 /* Window/Sub-window size */
  int fontid;                               /* The ID of the font to use */
};

/* Arguments passed with the NXTERMIOC_NXTERM_REDRAW command */

struct nxtermioc_redraw_s
{
  NXTERM handle;                            /* NxTerm handle */
  struct nxgl_rect_s rect;                  /* Rectangle to be re-drawn */
  bool more;                                /* True: More redraw commands follow */
};

#ifdef CONFIG_NXTERM_NXKBDIN
/* Arguments passed with the NXTERMIOC_NXTERM_KBDIN command */

struct nxtermioc_kbdin_s
{
  NXTERM handle;                            /* NxTerm handle */
  FAR const uint8_t *buffer;                /* Buffered keyboard data */
  uint8_t buflen;                           /* Amount of data in buffer */
};
#endif

/* Arguments passed with the NXTERMIOC_NXTERM_RESIZE command */

struct nxtermioc_resize_s
{
  NXTERM handle;                            /* NxTerm handle */
  struct nxgl_size_s size;                  /* New Window Size */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
# define EXTERN extern "C"
extern "C"
{
#else
# define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nx_register
 *
 * Description:
 *   Register a console device on a raw NX window.  The device will be
 *   registered at /dev/nxtermN where N is the provided minor number.
 *
 *   This is an internal NuttX interface and should not be called directly
 *   from applications.  Application access is supported only indirectly via
 *   the boardctl(BOARDIOC_NXTERM) interface.
 *
 * Input Parameters:
 *   hwnd - A handle that will be used to access the window.  The window must
 *     persist and this handle must be valid for the life of the NX console.
 *   wndo - Describes the window and font to be used.  The information in
 *     this structure is copied and the original need not persist after
 *     nxtool_register() returns.
 *   minor - The device minor number
 *
 * Returned Value:
 *   A non-NULL handle is returned on success.
 *
 ****************************************************************************/

NXTERM nx_register(NXWINDOW hwnd, FAR struct nxterm_window_s *wndo,
                   int minor);

/****************************************************************************
 * Name: nxtk_register
 *
 * Description:
 *   Register a console device on a framed NX window.  The device will be
 *   registered at /dev/nxtermN where N is the provided minor number.
 *
 *   This is an internal NuttX interface and should not be called directly
 *   from applications.  Application access is supported only indirectly via
 *   the boardctl(BOARDIOC_NXTERM) interface.
 *
 * Input Parameters:
 *   hfwnd - A handle that will be used to access the window.  The window must
 *     persist and this handle must be valid for the life of the NX console.
 *   wndo - Describes the window and font to be used.  The information in
 *     this structure is copied and the original need not persist after
 *     nxtool_register() returns.
 *   minor - The device minor number
 *
 * Returned Value:
 *   A non-NULL handle is returned on success.
 *
 ****************************************************************************/

NXTERM nxtk_register(NXTKWINDOW hfwnd, FAR struct nxterm_window_s *wndo,
                     int minor);

/****************************************************************************
 * Name: nxtool_register
 *
 * Description:
 *   Register a console device on a toolbar of a framed NX window.  The
 *   device will be registered at /dev/nxtermN where N is the provided minor
 *   number.  Application access is supported only indirectly via
 *   the boardctl(BOARDIOC_NXTERM) interface.
 *
 *   This is an internal NuttX interface and should not be called directly
 *   from applications.  Application access is supported only indirectly via
 *   the boardctl(BOARDIOC_NXTERM) interface.
 *
 * Input Parameters:
 *   hfwnd - A handle that will be used to access the toolbar.  The toolbar
 *     must persist and this handle must be valid for the life of the NX
 *     console.
 *   wndo - Describes the window and font to be used.  The information in
 *     this structure is copied and the original need not persist after
 *     nxtool_register() returns.
 *   minor - The device minor number
 *
 * Returned Value:
 *   A non-NULL handle is returned on success.
 *
 ****************************************************************************/

NXTERM nxtool_register(NXTKWINDOW hfwnd, FAR struct nxterm_window_s *wndo,
                       int minor);

/****************************************************************************
 * Name: nxterm_ioctl_tap
 *
 * Description:
 *   Execute an NXTERM IOCTL command from an external caller.
 *
 * NOTE:  We don't need driver context here because the NXTERM handle
 * provided within each of the NXTERM IOCTL command data.  Mutual
 * exclusion is similar managed by the IOCTL cmmand hendler.
 *
 * This permits the IOCTL to be called in abnormal context (such as
 * from boardctl())
 *
 ****************************************************************************/

int nxterm_ioctl_tap(int cmd, uintptr_t arg);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_NXTERM */
#endif /* __INCLUDE_NUTTX_NX_NXTERM_H */
