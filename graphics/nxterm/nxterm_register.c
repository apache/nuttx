/****************************************************************************
 * nuttx/graphics/nxterm/nxterm_register.c
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

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
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

  priv = (FAR struct nxterm_state_s *)
    kmm_zalloc(sizeof(struct nxterm_state_s));
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
#ifdef CONFIG_DEBUG_GRAPHICS
  priv->holder  = NO_HOLDER;
#endif

#ifdef CONFIG_NXTERM_NXKBDIN
  /* The waitsem semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  nxsem_init(&priv->waitsem, 0, 0);
  nxsem_set_protocol(&priv->waitsem, SEM_PRIO_NONE);
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

  /* Clear the display */

  nxterm_clear(priv);

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
