/****************************************************************************
 * drivers/ipcc/ipcc_unlink.c
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
#include <nuttx/ipcc.h>
#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>

#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <sys/types.h>

#include "ipcc_priv.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ipcc_unlink
 *
 * Description:
 *   Action to take upon file unlinking. Function will free resources if
 *   noone is using the driver when unlinking occured. If driver is still
 *   in use, it will be marked as unlinked and resource freeing will take
 *   place in ipcc_close() function instead, once last reference is closed.
 *
 * Input Parameters:
 *   inode - driver inode that is being unlinked
 *
 * Returned Value:
 *   OK on successfull close, or negated errno on failure.
 *
 * Assumptions/Limitations:
 *
 ****************************************************************************/

int ipcc_unlink(FAR struct inode *inode)
{
  FAR struct ipcc_driver_s *priv;
  int ret;

  /* Get our private data structure */

  DEBUGASSERT(inode);
  priv = inode->i_private;

  /* Get exclusive access to the IPCC driver state structure */

  if ((ret = nxsem_wait(&priv->exclsem)) < 0)
    {
      return ret;
    }

  /* Is anyone still using the driver? */

  if (priv->crefs <= 0)
    {
      /* No, we are free to free resources */

      ipcc_cleanup(priv);
      return OK;
    }

  /* Yes, someone is still using the driver, just mark file
   * as unlinked and free resources in ipcc_close() once last
   * reference is closed
   */

  priv->unlinked = true;
  nxsem_post(&priv->exclsem);
  return OK;
}
