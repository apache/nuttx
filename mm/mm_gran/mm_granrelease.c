/****************************************************************************
 * mm/mm_gran/mm_graninit.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <errno.h>

#include <nuttx/mm/gran.h>
#include <nuttx/kmalloc.h>

#include "mm_gran/mm_gran.h"

#ifdef CONFIG_GRAN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* State of the single GRAN allocator */

#ifdef CONFIG_GRAN_SINGLE
FAR struct gran_s *g_graninfo;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gran_release_common
 *
 * Description:
 *   Perform common GRAN initialization.
 *
 * Input Parameters:
 *   priv - Reference to the granule heap structure to be released.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void gran_release_common(FAR struct gran_s *priv)
{
  DEBUGASSERT(priv);
#ifndef CONFIG_GRAN_INTR
  sem_destroy(&priv->exclsem);
#endif
  kmm_free(priv);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gran_release
 *
 * Description:
 *   Uninitialize a gram memory allocator and release resources held by the
 *   allocator.
 *
 * Input Parameters:
 *   handle - The handle previously returned by gran_initialize
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_GRAN_SINGLE
void gran_release(void)
{
  gran_release_common(g_graninfo);
  g_graninfo = NULL;
}
#else
void gran_release(GRAN_HANDLE handle)
{
  gran_release_common(handle);
}
#endif

#endif /* CONFIG_GRAN */
