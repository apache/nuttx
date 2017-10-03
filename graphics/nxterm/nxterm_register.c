/****************************************************************************
 * nuttx/graphics/nxterm/nxterm_register.c
 *
 *   Copyright (C) 2012, 2016-2017 Gregory Nutt. All rights reserved.
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
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>
#include <nuttx/fs/fs.h>

#include "nxterm.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxterm_allocate
 ****************************************************************************/

FAR struct nxterm_state_s *
  nxterm_register(NXTERM handle, FAR struct nxterm_window_s *wndo,
                 FAR const struct nxterm_operations_s *ops, int minor)
{
  FAR struct nxterm_state_s *priv;
  FAR const struct nx_font_s *fontset;
  char devname[NX_DEVNAME_SIZE];
  NXHANDLE hfont;
  int ret;

  DEBUGASSERT(handle && wndo && ops && (unsigned)minor < 256);

  /* Allocate the driver structure */

  priv = (FAR struct nxterm_state_s *)kmm_zalloc(sizeof(struct nxterm_state_s));
  if (!priv)
    {
      gerr("ERROR: Failed to allocate the NX driver structure\n");
      return NULL;
    }

  /* Initialize the driver structure */

  priv->ops     = ops;
  priv->handle  = handle;
  priv->minor   = minor;
  memcpy(&priv->wndo, wndo, sizeof(struct nxterm_window_s));

  nxsem_init(&priv->exclsem, 0, 1);
#ifdef CONFIG_DEBUG_FEATURES
  priv->holder  = NO_HOLDER;
#endif

#ifdef CONFIG_NXTERM_NXKBDIN
  /* The waitsem semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  nxsem_init(&priv->waitsem, 0, 0);
  nxsem_setprotocol(&priv->waitsem, SEM_PRIO_NONE);
#endif

  /* Connect to the font cache for the configured font characteristics */

  priv->fcache = nxf_cache_connect(wndo->fontid, wndo->fcolor[0],
                                   wndo->wcolor[0], CONFIG_NXTERM_BPP,
                                   CONFIG_NXTERM_CACHESIZE);
  if (priv->fcache == NULL)
    {
      gerr("ERROR: Failed to connect to font cache for font ID %d: %d\n",
           wndo->fontid, errno);
      goto errout;
    }

  /* Get the handle of the font managed by the font cache */

  hfont = nxf_cache_getfonthandle(priv->fcache);
  if (hfont == NULL)
    {
      gerr("ERROR: Failed to get handlr for font ID %d: %d\n",
           wndo->fontid, errno);
      goto errout;
    }

  /* Get information about the font set being used and save this in the
   * state structure
   */

  fontset         = nxf_getfontset(hfont);
  priv->fheight   = fontset->mxheight;
  priv->fwidth    = fontset->mxwidth;
  priv->spwidth   = fontset->spwidth;

  /* Set up the text cache */

  priv->maxchars  = CONFIG_NXTERM_MXCHARS;

  /* Set the initial display position */

  nxterm_home(priv);

  /* Show the cursor */

  priv->cursor.code = CONFIG_NXTERM_CURSORCHAR;
  nxterm_showcursor(priv);

  /* Register the driver */

  snprintf(devname, NX_DEVNAME_SIZE, NX_DEVNAME_FORMAT, minor);
  ret = register_driver(devname, &g_nxterm_drvrops, 0666, priv);
  if (ret < 0)
    {
      gerr("ERROR: Failed to register %s\n", devname);
    }

  return (NXTERM)priv;

errout:
  kmm_free(priv);
  return NULL;
}
