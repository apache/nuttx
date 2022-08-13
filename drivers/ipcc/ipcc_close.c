/****************************************************************************
 * drivers/ipcc/ipcc_close.c
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
 * Name: ipcc_close
 *
 * Description:
 *   Closes the driver device. If this is last reference and file has been
 *   unlinked, we will also free resources allocated by ipcc_register()
 *
 * Input Parameters:
 *   filep - pointer to a file structure to close.
 *
 * Returned Value:
 *   OK on successfull close, or negated errno on failure.
 *
 * Assumptions/Limitations:
 *
 ****************************************************************************/

int ipcc_close(FAR struct file *filep)
{
  FAR struct ipcc_driver_s *priv;
  int ret;

  /* Get our private data structure */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  priv = filep->f_inode->i_private;

  /* Get exclusive access to the IPCC driver state structure */

  if ((ret = nxsem_wait(&priv->exclsem)) < 0)
    {
      return ret;
    }

  /* Decrement the count of open references on the driver */

  DEBUGASSERT(priv->crefs > 0);
  priv->crefs--;

  if (priv->crefs <= 0 && priv->unlinked)
    {
      /* If count ref is zero and file has been unlinked, it
       * means nobody uses the driver and seems like nobody
       * wants to use it anymore, so free up resources.
       */

      ipcc_cleanup(priv);
      return OK;
    }

  nxsem_post(&priv->exclsem);
  return OK;
}
