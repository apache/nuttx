/****************************************************************************
 * drivers/i3c/device.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include "internals.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: i3c_device_do_priv_xfers
 *
 * Description:
 *   Do I3C SDR private transfers directed to a specific device.
 *
 *   Initiate one or several private SDR transfers with @dev.
 *
 *   This function can sleep and thus cannot be called in atomic context.
 *
 * Input Parameters:
 *   dev    - Device with which the transfers should be done
 *   xfers  - Array of transfers
 *   nxfers - Number of transfers
 *
 * Returned Value:
 *   0 in case of success, a negative error core otherwise.
 *
 ****************************************************************************/

int i3c_device_do_priv_xfers(FAR const struct i3c_device *dev,
                             FAR struct i3c_priv_xfer *xfers,
                             int nxfers)
{
  int ret;
  int i;

  if (nxfers < 1)
    {
      return 0;
    }

  for (i = 0; i < nxfers; i++)
    {
      if (!xfers[i].len || !xfers[i].data.in)
        {
          return -EINVAL;
        }
    }

  i3c_bus_normaluse_lock(dev->bus);
  ret = i3c_dev_do_priv_xfers_locked(dev->desc, xfers, nxfers);
  i3c_bus_normaluse_unlock(dev->bus);

  return ret;
}

/****************************************************************************
 * Name: i3c_device_get_info
 *   Get I3C device information, Retrieve I3C dev info.
 *
 * Input Parameters:
 *   dev  - Device we want information on.
 *   info - The information object to fill in.
 *
 ****************************************************************************/

void i3c_device_get_info(FAR const struct i3c_device *dev,
                         FAR struct i3c_device_info *info)
{
  if (info == NULL)
    {
      return;
    }

  i3c_bus_normaluse_lock(dev->bus);
  if (dev->desc)
    {
      *info = dev->desc->info;
    }

  i3c_bus_normaluse_unlock(dev->bus);
}

/****************************************************************************
 * Name: i3c_device_disable_ibi
 *
 * Description:
 *   Disable IBIs coming from a specific device.
 *
 *   This function disable IBIs coming from a specific device and wait for
 *   all pending IBIs to be processed.
 *
 * Input Parameters:
 *   dev - Device on which IBIs should be disabled
 *
 * Returned Value:
 *   0 in case of success, a negative error core otherwise.
 *
 ****************************************************************************/

int i3c_device_disable_ibi(FAR const struct i3c_device *dev)
{
  int ret = -ENOENT;

  i3c_bus_normaluse_lock(dev->bus);
  if (dev->desc)
    {
      nxmutex_lock(&dev->desc->ibi_lock);
      ret = i3c_dev_disable_ibi_locked(dev->desc);
      nxmutex_unlock(&dev->desc->ibi_lock);
    }

  i3c_bus_normaluse_unlock(dev->bus);

  return ret;
}

/****************************************************************************
 * Name: i3c_device_enable_ibi
 *
 * Description:
 *   Enable IBIs coming from a specific device.
 *
 *   This function enable IBIs coming from a specific device and wait for
 *   all pending IBIs to be processed. This should be called on a device
 *   where i3c_device_request_ibi() has succeeded.
 *
 *   Note that IBIs from this device might be received before this function
 *   returns to its caller.
 *
 * Input Parameters:
 *   dev - Device on which IBIs should be enabled
 *
 * Returned Value:
 *   0 in case of success, a negative error core otherwise.
 *
 ****************************************************************************/

int i3c_device_enable_ibi(FAR const struct i3c_device *dev)
{
  int ret = -ENOENT;

  i3c_bus_normaluse_lock(dev->bus);
  if (dev->desc)
    {
      nxmutex_lock(&dev->desc->ibi_lock);
      ret = i3c_dev_enable_ibi_locked(dev->desc);
      nxmutex_unlock(&dev->desc->ibi_lock);
    }

  i3c_bus_normaluse_unlock(dev->bus);

  return ret;
}

/****************************************************************************
 * Name: i3c_device_request_ibi
 *
 * Description:
 *   Request an IBI.
 *
 *   This function is responsible for pre-allocating all resources needed to
 *   process IBIs coming from @dev. When this function returns, the IBI is
 *   not enabled until i3c_device_enable_ibi() is called.
 *
 * Input Parameters:
 *   dev - Device for which we should enable IBIs
 *   req - Setup requested for this IBI
 *
 * Returned Value:
 *   0 in case of success, a negative error core otherwise.
 *
 ****************************************************************************/

int i3c_device_request_ibi(FAR const struct i3c_device *dev,
                           FAR const struct i3c_ibi_setup *req)
{
  int ret = -ENOENT;

  if (req->handler == NULL || !req->num_slots)
    {
      return -EINVAL;
    }

  i3c_bus_normaluse_lock(dev->bus);
  if (dev->desc)
    {
      nxmutex_lock(&dev->desc->ibi_lock);
      ret = i3c_dev_request_ibi_locked(dev->desc, req);
      nxmutex_unlock(&dev->desc->ibi_lock);
    }

  i3c_bus_normaluse_unlock(dev->bus);

  return ret;
}

/****************************************************************************
 * Name: i3c_device_free_ibi
 *
 * Description:
 *   Free all resources needed for IBI handling.
 *
 *   This function is responsible for de-allocating resources previously
 *   allocated by i3c_device_request_ibi(). It should be called after
 *   disabling IBIs with i3c_device_disable_ibi().
 *
 * Input Parameters:
 *   dev - Device on which you want to release IBI resources
 *
 ****************************************************************************/

void i3c_device_free_ibi(FAR const struct i3c_device *dev)
{
  i3c_bus_normaluse_lock(dev->bus);
  if (dev->desc)
    {
      nxmutex_lock(&dev->desc->ibi_lock);
      i3c_dev_free_ibi_locked(dev->desc);
      nxmutex_unlock(&dev->desc->ibi_lock);
    }

  i3c_bus_normaluse_unlock(dev->bus);
}

/****************************************************************************
 * Name: i3c_device_send_ccc_cmd
 *
 * Description:
 *   This function is used to send a common ccc command.
 *
 * Input Parameters:
 *   dev  - An I3C device descriptor will be used for
 *   cmd  - The buf of ccc commands to transfer, only one frame at a time
 *
 * Returned Value:
 *   0 or positive if Success, nagative otherwise.
 ****************************************************************************/

int i3c_device_send_ccc_cmd(FAR const struct i3c_device *dev,
                            FAR struct i3c_ccc_cmd *cmd)
{
  int ret;
  FAR struct i3c_master_controller *master;

  if (dev == NULL || cmd == NULL)
    {
      return -EINVAL;
    }

  master = i3c_dev_get_master(dev->desc);

  i3c_bus_normaluse_lock(dev->bus);

  ret = i3c_master_send_ccc_cmd_locked(master, cmd);

  i3c_bus_normaluse_unlock(dev->bus);

  return ret;
}

/****************************************************************************
 * Name: i3c_master_find_i3c_dev
 *
 * Description:
 *   This function is used to be find a i3c_device address by master handle
 *   and provisional ID.
 *
 * Input Parameters:
 *   master - The master used to get i3c_device on the bus
 *   id     - An instance of i3c_device_id, include manufid,partid and so on.
 *
 * Returned Value:
 *   Struct i3c_device var in case of success, NULL otherwise.
 ****************************************************************************/

FAR const struct i3c_device *i3c_master_find_i3c_dev(
                              FAR struct i3c_master_controller *master,
                              FAR const struct i3c_device_id *id)
{
  FAR struct i3c_dev_desc *desc;
  uint16_t manuf;
  uint16_t part;
  uint16_t ext_info;
  bool rndpid;

  i3c_bus_normaluse_lock(&master->bus);
  i3c_bus_for_each_i3cdev(&master->bus, desc)
    {
      manuf = I3C_PID_MANUF_ID(desc->info.pid);
      part = I3C_PID_PART_ID(desc->info.pid);
      ext_info = I3C_PID_EXTRA_INFO(desc->info.pid);
      rndpid = I3C_PID_RND_LOWER_32BITS(desc->info.pid);

      if ((id->match_flags & I3C_MATCH_DCR) &&
           id->dcr != desc->info.dcr)
        {
          continue;
        }

      if ((id->match_flags & I3C_MATCH_MANUF) &&
           id->manuf_id != manuf)
        {
          continue;
        }

      if ((id->match_flags & I3C_MATCH_PART) &&
          (rndpid || id->part_id != part))
        {
          continue;
        }

      if ((id->match_flags & I3C_MATCH_EXTRA_INFO) &&
          (rndpid || id->extra_info != ext_info))
        {
          continue;
        }

      i3c_bus_normaluse_unlock(&master->bus);
      return desc->dev;
    }

  i3c_bus_normaluse_unlock(&master->bus);
  return NULL;
}
