/****************************************************************************
 * drivers/ipcc/ipcc_poll.c
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
#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdbool.h>
#include <stdio.h>
#include <sys/types.h>

#include "ipcc_priv.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ipcc_poll
 *
 * Description:
 *   Sets up or tears down poll for the driver associated with filep pointer
 *
 * Input Parameters:
 *   filep - file associated with the driver instance
 *   fds - The structure describing the events to be monitored, OR NULL if
 *         this is a request to stop monitoring events.
 *   setup - true: Setup up the poll; false: Teardown the poll
 *
 * Returned Value:
 *   0: Success; Negated errno on failure
 *
 * Assumptions/Limitations:
 *
 ****************************************************************************/

int ipcc_poll(FAR struct file *filep, FAR struct pollfd *fds,
                     bool setup)
{
  FAR struct ipcc_driver_s *priv;
  FAR struct pollfd **slot;
  pollevent_t eventset;
  int ret;
  int i;

  /* Get our private data structure */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  priv = filep->f_inode->i_private;

  /* Get exclusive access to driver */

  if ((ret = nxmutex_lock(&priv->lock)) < 0)
    {
      /* nxmutex_lock() will return on signal, we did not start
       * any transfer yet, so we can safely return with error
       */

      return ret;
    }

  /* Are we setting up the poll? Or tearing it down? */

  if (!setup)
    {
      /* We are tearing down the poll */

      slot = (FAR struct pollfd **)fds->priv;

      /* remove all memory of the poll setup */

      *slot = NULL;
      fds->priv = NULL;
      nxmutex_unlock(&priv->lock);
      return OK;
    }

  /* This is a request to set up the poll. Find an available slot
   * for the poll structure reference
   */

  for (i = 0; i < CONFIG_IPCC_NPOLLWAITERS; i++)
    {
      /* Find an available slot */

      if (!priv->fds[i])
        {
          /* Bind the poll structure and this slot */

          priv->fds[i] = fds;
          fds->priv    = &priv->fds[i];
          break;
        }
    }

  if (i >= CONFIG_IPCC_NPOLLWAITERS)
    {
      /* No free poll slot found */

      fds->priv = NULL;
      nxmutex_unlock(&priv->lock);
      return -EBUSY;
    }

  /* Should immediately notify on any of the requested events?  */

  eventset = 0;

#ifdef CONFIG_IPCC_BUFFERED
  if (circbuf_used(&priv->ipcc->rxbuf) > 0)
    {
      eventset |= POLLIN;
    }

  if (circbuf_space(&priv->ipcc->txbuf) > 0)
    {
      eventset |= POLLOUT;
    }
#endif

  poll_notify(priv->fds, CONFIG_IPCC_NPOLLWAITERS, eventset);

  nxmutex_unlock(&priv->lock);
  return OK;
}
