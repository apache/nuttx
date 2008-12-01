/****************************************************************************
//  * graphics/nxtk/nxtk_rawwindow.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdlib.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/nx.h>
#include <nuttx/nxtk.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nxtk_rawwindow_s
{
  struct nxtk_base_s base;
  NXHANDLE           handle;
  NXWINDOW           hwnd;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxtk_close
 ****************************************************************************/

static void nxtk_close(NXTWINDOW hwnd)
{
  FAR struct nxtk_rawwindow_s *this = (FAR struct nxtk_rawwindow_s *)hwnd;
  nx_close(this->hwnd);
  free(this);
}

/****************************************************************************
 * Name: nxtk_getposition
 ****************************************************************************/

static int nxtk_getposition(NXTWINDOW hwnd)
{
  FAR struct nxtk_rawwindow_s *this = (FAR struct nxtk_rawwindow_s *)hwnd;
  return nx_getposition(this->hwnd);
}

/****************************************************************************
 * Name: nxtk_setposition
 ****************************************************************************/

static int nxtk_setposition(NXTWINDOW hwnd, FAR struct nxgl_point_s *pos)
{
  FAR struct nxtk_rawwindow_s *this = (FAR struct nxtk_rawwindow_s *)hwnd;
  return nx_setposition(this->hwnd, pos);
}

/****************************************************************************
 * Name: nxtk_setsize
 ****************************************************************************/

static int nxtk_setsize(NXTWINDOW hwnd, FAR struct nxgl_rect_s *size)
{
  FAR struct nxtk_rawwindow_s *this = (FAR struct nxtk_rawwindow_s *)hwnd;
  return nx_setsize(this->hwnd, size);
}

/****************************************************************************
 * Name: nxtk_raise
 ****************************************************************************/

static int nxtk_raise(NXTWINDOW hwnd)
{
  FAR struct nxtk_rawwindow_s *this = (FAR struct nxtk_rawwindow_s *)hwnd;
  return nx_raise(this->hwnd);
}

/****************************************************************************
 * Name: nxtk_lower
 ****************************************************************************/

static int nxtk_lower(NXTWINDOW hwnd)
{
  FAR struct nxtk_rawwindow_s *this = (FAR struct nxtk_rawwindow_s *)hwnd;
  return nx_lower(this->hwnd);
}

/****************************************************************************
 * Name: nxtk_fill
 ****************************************************************************/

static int nxtk_fill(NXTWINDOW hwnd, FAR const struct nxgl_rect_s *rect,
                     nxgl_mxpixel_t color[CONFIG_NX_NPLANES])
{
  FAR struct nxtk_rawwindow_s *this = (FAR struct nxtk_rawwindow_s *)hwnd;
  return nx_fill(this->hwnd, rect, color);
}

/****************************************************************************
 * Name: nxtk_filltrapezoid
 ****************************************************************************/

static int nxtk_filltrapezoid(NXTWINDOW hwnd, FAR struct nxgl_trapezoid_s *trap,
                              nxgl_mxpixel_t color[CONFIG_NX_NPLANES])
{
  FAR struct nxtk_rawwindow_s *this = (FAR struct nxtk_rawwindow_s *)hwnd;
  return nx_filltrapezoid(this->hwnd, trap, color);
}

/****************************************************************************
 * Name: nxtk_move
 ****************************************************************************/

static int nxtk_move(NXTWINDOW hwnd, FAR const struct nxgl_rect_s *rect,
                     FAR const struct nxgl_point_s *offset)
{
  FAR struct nxtk_rawwindow_s *this = (FAR struct nxtk_rawwindow_s *)hwnd;
  return nx_move(this->hwnd, rect, offset);
}

/****************************************************************************
 * Name: nxtk_bitmap
 ****************************************************************************/

static int nxtk_bitmap(NXTWINDOW hwnd, FAR const struct nxgl_rect_s *dest,
                       FAR const void *src[CONFIG_NX_NPLANES],
                       FAR const struct nxgl_point_s *origin,
                       unsigned int stride)
{
  FAR struct nxtk_rawwindow_s *this = (FAR struct nxtk_rawwindow_s *)hwnd;
  return nx_bitmap(this->hwnd, dest, src, origin, stride);
}

/****************************************************************************
 * Public Functions
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

NXTWINDOW nxtk_rawwindow(NXHANDLE handle,
                         FAR const struct nx_callback_s *cb,
                         FAR void *arg)
{
  FAR struct nxtk_rawwindow_s *this = malloc(sizeof(struct nxtk_rawwindow_s));

  /* Pre-allocate this internal state structure so we won't don't have to
   * handle the failure to allocate later.
   */

  if (!this)
    {
      errno = ENOMEM;
      return NULL;
    }

  /* Create the window */

  this->hwnd = nx_openwindow(handle, cb, arg);
  if (!this->hwnd)
    {
      free(this);
      return NULL;
    }

  /* Then initialize the rest of the object */

  this->handle             = handle;
  this->base.close         = nxtk_close;
  this->base.getposition   = nxtk_getposition;
  this->base.setposition   = nxtk_setposition;
  this->base.setsize       = nxtk_setsize;
  this->base.raise         = nxtk_raise;
  this->base.lower         = nxtk_lower;
  this->base.fill          = nxtk_fill;
  this->base.filltrapezoid = nxtk_filltrapezoid;
  this->base.move          = nxtk_move;
  this->base.bitmap        = nxtk_bitmap;

  return &this->base;
}
