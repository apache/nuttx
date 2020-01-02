/****************************************************************************
 * nuttx/graphics/nxterm/nxterm_unregister.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/nx/nxterm.h>

#include "nxterm.h"

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxterm_unregister
 *
 * Description:
 *   Un-register an NX console device.
 *
 * Input Parameters:
 *   priv - NxTerm private state structure instance.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxterm_unregister(FAR struct nxterm_state_s *priv)
{
  char devname[NX_DEVNAME_SIZE];

  DEBUGASSERT(priv != NULL);

  /* Destroy semaphores */

  nxsem_destroy(&priv->exclsem);
#ifdef CONFIG_NXTERM_NXKBDIN
  nxsem_destroy(&priv->waitsem);
#endif

  /* Free the font cache */

  nxf_cache_disconnect(priv->fcache);

  /* Unregister the driver */

  snprintf(devname, NX_DEVNAME_SIZE, NX_DEVNAME_FORMAT, priv->minor);
  unregister_driver(devname);

  /* Free the private data structure */

  kmm_free(priv);
}

#endif /* !CONFIG_DISABLE_PSEUDOFS_OPERATIONS */
