/****************************************************************************
 * include/nuttx/nxtk.h
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

#ifndef __INCLUDE_NUTTX_NXTK_H
#define __INCLUDE_NUTTX_NXTK_H

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

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This is an objects, implemented in C, that provides access to the NX
 * window APIs.  Why would you use this object instead of direct calls to the
 * nx_ APIs?  So that you can support polymorphism.  (Doesn't this make you
 * really appreciate C++?).  All of the APIs provided in the NX toolkit
 * export this interface.
 *
 * The methods provided in this call table are the nx_* window APIs discussed
 * in include/nx.h accessed through a vtable.  See that header for description
 * of each method.
 */

typedef FAR struct nxtk_base_s *NXTWINDOW;
struct nxtk_base_s
{
  /* Destructor */

  void (*close)(NXTWINDOW hwnd);

  /* Window methods (documented in include/nuttx/nx.h) */

  int (*getposition)(NXTWINDOW hwnd);
  int (*setposition)(NXTWINDOW hwnd, FAR struct nxgl_point_s *pos);
  int (*setsize)(NXTWINDOW hwnd, FAR struct nxgl_rect_s *size);
  int (*raise)(NXTWINDOW hwnd);
  int (*lower)(NXTWINDOW hwnd);
  int (*fill)(NXTWINDOW hwnd, FAR const struct nxgl_rect_s *rect,
              nxgl_mxpixel_t color[CONFIG_NX_NPLANES]);
  int (*filltrapezoid)(NXTWINDOW hwnd, FAR struct nxgl_trapezoid_s *trap,
                       nxgl_mxpixel_t color[CONFIG_NX_NPLANES]);
  int (*move)(NXTWINDOW hwnd, FAR const struct nxgl_rect_s *rect,
              FAR const struct nxgl_point_s *offset);
  int (*bitmap)(NXTWINDOW hwnd, FAR const struct nxgl_rect_s *dest,
                FAR const void *src[CONFIG_NX_NPLANES],
                FAR const struct nxgl_point_s *origin,
                unsigned int stride);
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

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nxtk_rawwindow
 *
 * Description:
 *    This function is the constructor for a raw NXTWINDOW object. 
 *    This provides a one-to-one mapping with the window APIs in nx.h.
 *
 * Input Parameters:
 *   handle - The handle returned by nx_connect or nx_open
 *   cb     - Callbacks used to process window events
 *   arg    - User provided value that will be returned with NX callbacks.
 *
 * Return:
 *   Success: A non-NULL handle used with subsequent method calls
 *   Failure:  NULL is returned and errno is set appropriately
 *
 ****************************************************************************/

EXTERN NXTWINDOW nxtk_rawwindow(NXHANDLE handle,
                                FAR const struct nx_callback_s *cb,
                                FAR void *arg);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_NXTK_H */
