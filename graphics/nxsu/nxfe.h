/****************************************************************************
 * graphics/nxsu/nxfe.h
 *
 *   Copyright (C) 2008-2010, 2013 Gregory Nutt. All rights reserved.
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

#ifndef __GRAPHICS_NXSU_NXFE_H
#define __GRAPHICS_NXSU_NXFE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <mqueue.h>
#include <semaphore.h>

#include <nuttx/nx/nx.h>

#include "nxbe.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Server state structure ***************************************************/

/* This the 'front-end' state structure.  It is really the same as the
 * the back-end state, but we wrap the back-end state so that we can add
 * things to the structure in the future
 */

struct nxfe_state_s
{
  /* The 'back-end' window status.  Must be first so that instances of
   * struct nxbe_state_s can be simply cast to an instance of struct
   * nxfe_state_s
   */

  struct nxbe_state_s be;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

EXTERN const struct nx_callback_s g_bkgdcb;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxfe_redrawreq
 *
 * Descripton:
 *   Request the client that has this window to redraw the rectangular region.
 *
 ****************************************************************************/

void nxfe_redrawreq(FAR struct nxbe_window_s *wnd,
                    FAR const struct nxgl_rect_s *rect);

/****************************************************************************
 * Name: nxfe_reportposition
 *
 * Descripton:
 *   Report the new size/position of the window.
 *
 ****************************************************************************/

void nxfe_reportposition(FAR struct nxbe_window_s *wnd);

/****************************************************************************
 * Name: nxmu_mouseinit
 *
 * Description:
 *   Initialize with the mouse in the center of the display
 *
 ****************************************************************************/

#ifdef CONFIG_NX_XYINPUT
void nxsu_mouseinit(int x, int y);
#endif

/****************************************************************************
 * Name: nxmu_mousereport
 *
 * Description:
 *   Report mouse position info to the specified window
 *
 * Input Parameters:
 *   wnd - The window to receive the mouse report
 *
 * Returned Value:
 *   0: Mouse report sent; >0: Mouse report not sent; <0: An error occurred
 *
 ****************************************************************************/

#ifdef CONFIG_NX_XYINPUT
int nxsu_mousereport(struct nxbe_window_s *wnd);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif  /* __GRAPHICS_NXSU_NXFE_H */
