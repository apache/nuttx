/****************************************************************************
 * graphics/nxterm/nxterm_unregister.c
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
