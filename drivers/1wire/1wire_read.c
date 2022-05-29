/****************************************************************************
 * drivers/1wire/1wire_read.c
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

#include <assert.h>

#include <nuttx/1wire/1wire.h>
#include <nuttx/1wire/1wire_master.h>

#include "1wire_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: 1wire_read
 *
 * Description:
 *   Receive a block of data from 1WIRE. Each read operation will be an
 *   'atomic' operation in the sense that any other 1WIRE actions will be
 *   serialized and pend until this read completes.
 *
 * Input Parameters:
 *   master - Device-specific state data
 *   buffer - A pointer to a buffer of data to receive the data from the
 *            device
 *   buflen - The requested number of bytes to be read
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

int onewire_read(FAR struct onewire_master_s *master,
                 FAR const struct onewire_config_s *config,
                 FAR uint8_t *buffer, int buflen)
{
  int ret;

  /* Avoid calling this function from a search callback to prevent a
   * deadlock
   */

  if (master->insearch == true)
    {
      return -EAGAIN;
    }

  ret = nxrmutex_lock(&master->devlock);
  if (ret < 0)
    {
      return ret;
    }

  ret = onewire_reset_select(master, config->romcode);
  if (ret < 0)
    {
      goto err_unlock;
    }

  /* Then perform the transfer. */

  ret = ONEWIRE_READ(master->dev, buffer, buflen);

err_unlock:
  nxrmutex_unlock(&master->devlock);
  return ret;
}
