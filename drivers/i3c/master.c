/****************************************************************************
 * drivers/i3c/master.c
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

#include <errno.h>
#include <stdio.h>
#include <debug.h>

#include <nuttx/list.h>
#include <nuttx/wqueue.h>
#include <nuttx/nuttx.h>
#include <nuttx/mutex.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mm/mm.h>
#include <nuttx/spinlock.h>
#include <netinet/in.h>

#include <nuttx/i3c/master.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/i3c/i3c_driver.h>
#include <nuttx/i3c/device.h>
#include "internals.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: i3c_bus_maintenance_lock
 *
 * Description:
 *   Lock the bus for a maintenance operation
 *
 *   This function takes the bus lock so that no other operations can
 *   occur on the bus. This is needed for all kind of bus maintenance
 *   operation, like
 *   - enabling/disabling slave events
 *   - re-triggering DAA
 *   - changing the dynamic address of a device
 *   - relinquishing mastership
 *
 *   The reason for this kind of locking is that we don't want drivers
 *   and core logic to rely on I3C device information that could be
 *   changed behind their back.
 *
 * Input Parameters:
 *   bus - I3C bus to take the lock on.
 *
 ****************************************************************************/

static void i3c_bus_maintenance_lock(FAR struct i3c_bus *bus)
{
  nxmutex_lock(&bus->lock);
}

/****************************************************************************
 * Name: i3c_bus_maintenance_unlock
 *
 * Description:
 *   Release the bus lock after a maintenance operation.
 *
 *   Should be called when the bus maintenance operation is done.
 *   See i3c_bus_maintenance_lock() for more details on what these
 *   maintenance operations are.
 *
 * Input Parameters:
 *   bus - I3C bus to release the lock on.
 *
 ****************************************************************************/

static void i3c_bus_maintenance_unlock(FAR struct i3c_bus *bus)
{
  nxmutex_unlock(&bus->lock);
}

/****************************************************************************
 * Name: i3c_bus_to_i3c_master
 ****************************************************************************/

static FAR struct i3c_master_controller *
i3c_bus_to_i3c_master(FAR struct i3c_bus *i3cbus)
{
  return container_of(i3cbus, struct i3c_master_controller, bus);
}

static FAR struct i3c_master_controller *
i2c_adapter_to_i3c_master(FAR struct i2c_master_s *i2c_master)
{
  return container_of(i2c_master, struct i3c_master_controller, i2c);
}

static int i3c_master_to_i2c_bus_number(
                        FAR struct i3c_master_controller *master)
{
  return master->i2c_bus_id;
}

/****************************************************************************
 * Name: i3c_bus_get_addr_slot_status
 ****************************************************************************/

static enum i3c_addr_slot_status
i3c_bus_get_addr_slot_status(FAR struct i3c_bus *bus, uint16_t addr)
{
  int32_t status;
  int  bitpos = addr * 2;

  if (addr > I3C_I2C_MAX_ADDR)
    {
      return I3C_ADDR_SLOT_RSVD;
    }

  status = bus->addrslots[bitpos / I3C_BITS_PER_LONG];
  status >>= bitpos % I3C_BITS_PER_LONG;

  return status & I3C_ADDR_SLOT_STATUS_MASK;
}

/****************************************************************************
 * Name: i3c_bus_set_addr_slot_status
 ****************************************************************************/

static void i3c_bus_set_addr_slot_status(FAR struct i3c_bus *bus,
               uint16_t addr, enum i3c_addr_slot_status status)
{
  int bitpos = addr * 2;
  FAR uint32_t *ptr;

  if (addr > I3C_I2C_MAX_ADDR)
    {
      return;
    }

  ptr = bus->addrslots + (bitpos / I3C_BITS_PER_LONG);
  *ptr &= ~((unsigned long)I3C_ADDR_SLOT_STATUS_MASK <<
          (bitpos % I3C_BITS_PER_LONG));
  *ptr |= (unsigned long)status << (bitpos % I3C_BITS_PER_LONG);
}

/****************************************************************************
 * Name: i3c_bus_dev_addr_is_avail
 ****************************************************************************/

static bool i3c_bus_dev_addr_is_avail(FAR struct i3c_bus *bus,
                                      unsigned char addr)
{
  enum i3c_addr_slot_status status;

  status = i3c_bus_get_addr_slot_status(bus, addr);
  return status == I3C_ADDR_SLOT_FREE;
}

/****************************************************************************
 * Name: i3c_bus_get_free_addr
 ****************************************************************************/

static int i3c_bus_get_free_addr(FAR struct i3c_bus *bus,
                                 unsigned char start_addr)
{
  enum i3c_addr_slot_status status;
  unsigned char addr;

  for (addr = start_addr; addr < I3C_MAX_ADDR; addr++)
    {
      status = i3c_bus_get_addr_slot_status(bus, addr);
      if (status == I3C_ADDR_SLOT_FREE)
        {
          return addr;
        }
    }

  return -ENOMEM;
}

/****************************************************************************
 * Name: i3c_bus_init_addrslots
 ****************************************************************************/

static void i3c_bus_init_addrslots(FAR struct i3c_bus *bus)
{
  int i;

  /* Addresses 0 to 7 are reserved. */

  for (i = 0; i < 8; i++)
    {
      i3c_bus_set_addr_slot_status(bus, i, I3C_ADDR_SLOT_RSVD);
    }

  /* Reserve broadcast address and all addresses that might collide
   * with the broadcast address when facing a single bit error.
   */

  i3c_bus_set_addr_slot_status(bus, I3C_BROADCAST_ADDR,
                               I3C_ADDR_SLOT_RSVD);
  for (i = 0; i < 7; i++)
    {
      i3c_bus_set_addr_slot_status(bus, I3C_BROADCAST_ADDR ^ I3C_BIT(i),
                                 I3C_ADDR_SLOT_RSVD);
    }
}

/****************************************************************************
 * Name: i3c_bus_init
 ****************************************************************************/

static void i3c_bus_init(FAR struct i3c_bus *i3cbus)
{
  FAR struct i3c_master_controller *master;
  master = i3c_bus_to_i3c_master(i3cbus);

  nxmutex_init(&i3cbus->lock);
  list_initialize(&i3cbus->devs.i3c);
  i3c_bus_init_addrslots(i3cbus);
  i3cbus->mode = I3C_BUS_MODE_PURE;
  i3cbus->id = master->i3c_bus_id;
}

/****************************************************************************
 * Name: i3c_bus_set_mode
 ****************************************************************************/

static int i3c_bus_set_mode(FAR struct i3c_bus *i3cbus,
             enum i3c_bus_mode mode, unsigned long max_i2c_scl_rate)
{
  i3cbus->mode = mode;

  switch (i3cbus->mode)
    {
      case I3C_BUS_MODE_PURE:
        {
          if (!i3cbus->scl_rate.i3c)
            {
              i3cbus->scl_rate.i3c = I3C_BUS_TYP_I3C_SCL_RATE;
            }
        }
        break;
      case I3C_BUS_MODE_MIXED_FAST:
      case I3C_BUS_MODE_MIXED_LIMITED:
        {
          if (!i3cbus->scl_rate.i3c)
            {
              i3cbus->scl_rate.i3c = I3C_BUS_TYP_I3C_SCL_RATE;
            }

          if (!i3cbus->scl_rate.i2c)
            {
              i3cbus->scl_rate.i2c = max_i2c_scl_rate;
            }
        }
        break;
      case I3C_BUS_MODE_MIXED_SLOW:
        {
          if (!i3cbus->scl_rate.i2c)
            {
              i3cbus->scl_rate.i2c = max_i2c_scl_rate;
            }

          if (!i3cbus->scl_rate.i3c ||
              i3cbus->scl_rate.i3c > i3cbus->scl_rate.i2c)
            {
              i3cbus->scl_rate.i3c = i3cbus->scl_rate.i2c;
            }
        }
        break;
      default:
        return -EINVAL;
    }

  /* I3C/I2C frequency may have been overridden, check that user-provided
   * values are not exceeding max possible frequency.
   */

  if (i3cbus->scl_rate.i3c > I3C_BUS_MAX_I3C_SCL_RATE ||
      i3cbus->scl_rate.i2c > I3C_BUS_I2C_FM_PLUS_SCL_RATE)
      {
        return -EINVAL;
      }

  return 0;
}

static int i3c_master_to_i3c_bus_number(
                            FAR struct i3c_master_controller *master)
{
  return master->i3c_bus_id;
}

/****************************************************************************
 * Name: i3c_ccc_cmd_dest_init
 ****************************************************************************/

static FAR void *i3c_ccc_cmd_dest_init(FAR struct i3c_ccc_cmd_dest *dest,
                    unsigned char addr, uint16_t payloadlen)
{
  dest->addr = addr;
  dest->payload.len = payloadlen;
  if (payloadlen)
    {
      dest->payload.data = kmm_zalloc(payloadlen);
    }
  else
    {
      dest->payload.data = NULL;
    }

  return dest->payload.data;
}

/****************************************************************************
 * Name: i3c_ccc_cmd_dest_cleanup
 ****************************************************************************/

static void i3c_ccc_cmd_dest_cleanup(FAR struct i3c_ccc_cmd_dest *dest)
{
  kmm_free(dest->payload.data);
}

static void i3c_ccc_cmd_init(FAR struct i3c_ccc_cmd *cmd, bool rnw,
               unsigned char id, FAR struct i3c_ccc_cmd_dest *dests,
               unsigned int ndests)
{
  cmd->rnw = rnw ? 1 : 0;
  cmd->id = id;
  cmd->dests = dests;
  cmd->ndests = ndests;
  cmd->err = I3C_ERROR_UNKNOWN;
}

static FAR struct i3c_dev_desc *
i3c_master_alloc_i3c_dev(FAR struct i3c_master_controller *master,
                         FAR const struct i3c_device_info *info)
{
  FAR struct i3c_dev_desc *dev;

  dev = kmm_zalloc(sizeof(*dev));
  if (dev == NULL)
    {
      return NULL;
    }

  dev->common.master = master;
  dev->info = *info;
  nxmutex_init(&dev->ibi_lock);

  return dev;
}

static int i3c_master_rstdaa_locked(FAR struct i3c_master_controller *master,
                                    unsigned char addr)
{
  enum i3c_addr_slot_status addrstat;
  struct i3c_ccc_cmd_dest dest;
  struct i3c_ccc_cmd cmd;
  int ret;

  if (master == NULL)
    {
      return -EINVAL;
    }

  addrstat = i3c_bus_get_addr_slot_status(&master->bus, addr);
  if (addr != I3C_BROADCAST_ADDR && addrstat != I3C_ADDR_SLOT_I3C_DEV)
    {
      return -EINVAL;
    }

  i3c_ccc_cmd_dest_init(&dest, addr, 0);
  i3c_ccc_cmd_init(&cmd, false,
                   I3C_CCC_RSTDAA(addr == I3C_BROADCAST_ADDR),
                   &dest, 1);
  ret = i3c_master_send_ccc_cmd_locked(master, &cmd);
  i3c_ccc_cmd_dest_cleanup(&dest);

  return ret;
}

static int i3c_master_enec_disec_locked(
              FAR struct i3c_master_controller *master,
              unsigned char addr, bool enable, unsigned char evts)
{
  FAR struct i3c_ccc_events *events;
  struct i3c_ccc_cmd_dest dest;
  struct i3c_ccc_cmd cmd;
  int ret;

  events = i3c_ccc_cmd_dest_init(&dest, addr, sizeof(*events));
  if (events == NULL)
    {
      return -ENOMEM;
    }

  events->events = evts;
  i3c_ccc_cmd_init(&cmd, false,
                   enable ?
                   I3C_CCC_ENEC(addr == I3C_BROADCAST_ADDR) :
                   I3C_CCC_DISEC(addr == I3C_BROADCAST_ADDR),
                   &dest, 1);
  ret = i3c_master_send_ccc_cmd_locked(master, &cmd);
  i3c_ccc_cmd_dest_cleanup(&dest);

  return ret;
}

static int i3c_master_setda_locked(FAR struct i3c_master_controller *master,
              unsigned char oldaddr, unsigned char newaddr, bool setdasa)
{
  struct i3c_ccc_cmd_dest dest;
  FAR struct i3c_ccc_setda *setda;
  struct i3c_ccc_cmd cmd;
  int ret;

  if (!oldaddr || !newaddr)
    {
      return -EINVAL;
    }

  setda = i3c_ccc_cmd_dest_init(&dest, oldaddr, sizeof(*setda));
  if (setda == NULL)
    {
      return -ENOMEM;
    }

  setda->addr = newaddr << 1;
  i3c_ccc_cmd_init(&cmd, false,
                    setdasa ? I3C_CCC_SETDASA : I3C_CCC_SETNEWDA,
                    &dest, 1);
  ret = i3c_master_send_ccc_cmd_locked(master, &cmd);
  i3c_ccc_cmd_dest_cleanup(&dest);

  return ret;
}

static int i3c_master_setnewda_locked(
              FAR struct i3c_master_controller *master,
              unsigned char oldaddr, unsigned char newaddr)
{
  return i3c_master_setda_locked(master, oldaddr, newaddr, false);
}

static int i3c_master_getmrl_locked(FAR struct i3c_master_controller *master,
                                    FAR struct i3c_device_info *info)
{
  struct i3c_ccc_cmd_dest dest;
  FAR struct i3c_ccc_mrl *mrl;
  struct i3c_ccc_cmd cmd;
  int ret;

  mrl = i3c_ccc_cmd_dest_init(&dest, info->dyn_addr, sizeof(*mrl));
  if (mrl == NULL)
    {
      return -ENOMEM;
    }

  /* When the device does not have IBI payload GETMRL only returns 2
   * bytes of data.
   */

  if (!(info->bcr & I3C_BCR_IBI_PAYLOAD))
    {
      dest.payload.len -= 1;
    }

  i3c_ccc_cmd_init(&cmd, true, I3C_CCC_GETMRL, &dest, 1);
  ret = i3c_master_send_ccc_cmd_locked(master, &cmd);
  if (ret)
    {
      goto out;
    }

  switch (dest.payload.len)
    {
      case 3:
        {
          info->max_ibi_len = mrl->ibi_len;
          info->max_read_len = ntohs(mrl->read_len);
        }
        break;
      case 2:
        {
          info->max_read_len = ntohs(mrl->read_len);
        }
        break;
      default:
        ret = -EIO;
        goto out;
    }

out:
  i3c_ccc_cmd_dest_cleanup(&dest);

  return ret;
}

static int i3c_master_getmwl_locked(FAR struct i3c_master_controller *master,
                                    FAR struct i3c_device_info *info)
{
  struct i3c_ccc_cmd_dest dest;
  FAR struct i3c_ccc_mwl *mwl;
  struct i3c_ccc_cmd cmd;
  int ret;

  mwl = i3c_ccc_cmd_dest_init(&dest, info->dyn_addr, sizeof(*mwl));
  if (mwl == NULL)
    {
      return -ENOMEM;
    }

  i3c_ccc_cmd_init(&cmd, true, I3C_CCC_GETMWL, &dest, 1);
  ret = i3c_master_send_ccc_cmd_locked(master, &cmd);
  if (ret)
    {
      goto out;
    }

  if (dest.payload.len != sizeof(*mwl))
    {
      ret = -EIO;
      goto out;
    }

  info->max_write_len = ntohs(mwl->len);

out:
  i3c_ccc_cmd_dest_cleanup(&dest);

  return ret;
}

static int i3c_master_getmxds_locked(
              FAR struct i3c_master_controller *master,
              FAR struct i3c_device_info *info)
{
  FAR struct i3c_ccc_getmxds *getmaxds;
  struct i3c_ccc_cmd_dest dest;
  struct i3c_ccc_cmd cmd;
  int ret;

  getmaxds = i3c_ccc_cmd_dest_init(&dest, info->dyn_addr,
                                    sizeof(*getmaxds));
  if (getmaxds == NULL)
    {
      return -ENOMEM;
    }

  i3c_ccc_cmd_init(&cmd, true, I3C_CCC_GETMXDS, &dest, 1);
  ret = i3c_master_send_ccc_cmd_locked(master, &cmd);
  if (ret)
    {
      goto out;
    }

  if (dest.payload.len != 2 && dest.payload.len != 5)
    {
      ret = -EIO;
      goto out;
    }

  info->max_read_ds = getmaxds->maxrd;
  info->max_write_ds = getmaxds->maxwr;
  if (dest.payload.len == 5)
    {
      info->max_read_turnaround = getmaxds->maxrdturn[0] |
                ((uint32_t)getmaxds->maxrdturn[1] << 8) |
                ((uint32_t)getmaxds->maxrdturn[2] << 16);
    }

out:
  i3c_ccc_cmd_dest_cleanup(&dest);

  return ret;
}

static int i3c_master_gethdrcap_locked(
              FAR struct i3c_master_controller *master,
              FAR struct i3c_device_info *info)
{
  FAR struct i3c_ccc_gethdrcap *gethdrcap;
  struct i3c_ccc_cmd_dest dest;
  struct i3c_ccc_cmd cmd;
  int ret;

  gethdrcap = i3c_ccc_cmd_dest_init(&dest, info->dyn_addr,
                                    sizeof(*gethdrcap));
  if (gethdrcap == NULL)
    {
      return -ENOMEM;
    }

  i3c_ccc_cmd_init(&cmd, true, I3C_CCC_GETHDRCAP, &dest, 1);
  ret = i3c_master_send_ccc_cmd_locked(master, &cmd);
  if (ret)
    {
      goto out;
    }

  if (dest.payload.len != 1)
    {
      ret = -EIO;
      goto out;
    }

  info->hdr_cap = gethdrcap->modes;

out:
  i3c_ccc_cmd_dest_cleanup(&dest);

  return ret;
}

static int i3c_master_getpid_locked(FAR struct i3c_master_controller *master,
                                    FAR struct i3c_device_info *info)
{
  FAR struct i3c_ccc_getpid *getpid;
  struct i3c_ccc_cmd_dest dest;
  struct i3c_ccc_cmd cmd;
  int ret;
  int i;

  getpid = i3c_ccc_cmd_dest_init(&dest, info->dyn_addr, sizeof(*getpid));
  if (getpid == NULL)
    {
      return -ENOMEM;
    }

  i3c_ccc_cmd_init(&cmd, true, I3C_CCC_GETPID, &dest, 1);
  ret = i3c_master_send_ccc_cmd_locked(master, &cmd);
  if (ret)
    {
      goto out;
    }

  info->pid = 0;
  for (i = 0; i < sizeof(getpid->pid); i++)
    {
      int sft = (sizeof(getpid->pid) - i - 1) * 8;
      info->pid |= (uint64_t)getpid->pid[i] << sft;
    }

out:
  i3c_ccc_cmd_dest_cleanup(&dest);

  return ret;
}

static int i3c_master_getbcr_locked(FAR struct i3c_master_controller *master,
                                    FAR struct i3c_device_info *info)
{
  FAR struct i3c_ccc_getbcr *getbcr;
  struct i3c_ccc_cmd_dest dest;
  struct i3c_ccc_cmd cmd;
  int ret;

  getbcr = i3c_ccc_cmd_dest_init(&dest, info->dyn_addr, sizeof(*getbcr));
  if (getbcr == NULL)
    {
      return -ENOMEM;
    }

  i3c_ccc_cmd_init(&cmd, true, I3C_CCC_GETBCR, &dest, 1);
  ret = i3c_master_send_ccc_cmd_locked(master, &cmd);
  if (ret)
    {
      goto out;
    }

  info->bcr = getbcr->bcr;

out:
  i3c_ccc_cmd_dest_cleanup(&dest);

  return ret;
}

static int i3c_master_getdcr_locked(FAR struct i3c_master_controller *master,
                                    FAR struct i3c_device_info *info)
{
  FAR struct i3c_ccc_getdcr *getdcr;
  struct i3c_ccc_cmd_dest dest;
  struct i3c_ccc_cmd cmd;
  int ret;

  getdcr = i3c_ccc_cmd_dest_init(&dest, info->dyn_addr, sizeof(*getdcr));
  if (getdcr == NULL)
    {
      return -ENOMEM;
    }

  i3c_ccc_cmd_init(&cmd, true, I3C_CCC_GETDCR, &dest, 1);
  ret = i3c_master_send_ccc_cmd_locked(master, &cmd);
  if (ret)
    {
      goto out;
    }

  info->dcr = getdcr->dcr;

out:
  i3c_ccc_cmd_dest_cleanup(&dest);

  return ret;
}

static int i3c_master_retrieve_dev_info(FAR struct i3c_dev_desc *dev)
{
  FAR struct i3c_master_controller *master = i3c_dev_get_master(dev);
  enum i3c_addr_slot_status slot_status;
  int ret;

  if (!dev->info.dyn_addr)
    {
      return -EINVAL;
    }

  slot_status = i3c_bus_get_addr_slot_status(&master->bus,
                dev->info.dyn_addr);
  if (slot_status == I3C_ADDR_SLOT_RSVD)
    {
      return -EINVAL;
    }

  ret = i3c_master_getpid_locked(master, &dev->info);
  if (ret)
    {
      return ret;
    }

  ret = i3c_master_getbcr_locked(master, &dev->info);
  if (ret)
    {
      return ret;
    }

  ret = i3c_master_getdcr_locked(master, &dev->info);
  if (ret)
    {
      return ret;
    }

  if (dev->info.bcr & I3C_BCR_MAX_DATA_SPEED_LIM)
    {
      ret = i3c_master_getmxds_locked(master, &dev->info);
      if (ret)
        {
          return ret;
        }
    }

  if (dev->info.bcr & I3C_BCR_IBI_PAYLOAD)
    {
      dev->info.max_ibi_len = 1;
    }

  i3c_master_getmrl_locked(master, &dev->info);
  i3c_master_getmwl_locked(master, &dev->info);

  if (dev->info.bcr & I3C_BCR_HDR_CAP)
    {
      ret = i3c_master_gethdrcap_locked(master, &dev->info);
      if (ret)
        {
          return ret;
        }
    }

  return 0;
}

static void i3c_master_put_i3c_addrs(FAR struct i3c_dev_desc *dev)
{
  FAR struct i3c_master_controller *master = i3c_dev_get_master(dev);

  if (dev->info.dyn_addr)
    {
      i3c_bus_set_addr_slot_status(&master->bus, dev->info.dyn_addr,
                                    I3C_ADDR_SLOT_FREE);
    }
}

static int i3c_master_get_i3c_addrs(FAR struct i3c_dev_desc *dev)
{
  FAR struct i3c_master_controller *master = i3c_dev_get_master(dev);
  enum i3c_addr_slot_status status;

  if (!dev->info.dyn_addr)
    {
      return 0;
    }

  /* ->init_dyn_addr should have been reserved before that, so, if we're
   * trying to apply a pre-reserved dynamic address, we should not try
   * to reserve the address slot a second time.
   */

  if (dev->info.dyn_addr)
    {
      status = i3c_bus_get_addr_slot_status(&master->bus,
                    dev->info.dyn_addr);
      if (status != I3C_ADDR_SLOT_FREE)
        {
          return -EBUSY;
        }

      i3c_bus_set_addr_slot_status(&master->bus, dev->info.dyn_addr,
               I3C_ADDR_SLOT_I3C_DEV);
    }

  return 0;
}

static int i3c_master_attach_i3c_dev(
              FAR struct i3c_master_controller *master,
              FAR struct i3c_dev_desc *dev)
{
  int ret;

  /* We don't attach devices to the controller until they are
   * addressable on the bus.
   */

  if (!dev->info.dyn_addr)
    {
      return 0;
    }

  ret = i3c_master_get_i3c_addrs(dev);
  if (ret)
    {
      return ret;
    }

  /* Do not attach the master device itself. */

  if (master->this != dev && master->ops->attach_i3c_dev)
    {
      ret = master->ops->attach_i3c_dev(dev);
      if (ret)
        {
          i3c_master_put_i3c_addrs(dev);
          return ret;
        }
    }

  list_add_tail(&master->bus.devs.i3c, &dev->common.node);

  return 0;
}

static int i3c_master_reattach_i3c_dev(FAR struct i3c_dev_desc *dev,
                                       unsigned char old_dyn_addr)
{
  FAR struct i3c_master_controller *master = i3c_dev_get_master(dev);
  enum i3c_addr_slot_status status;
  int ret;

  if (dev->info.dyn_addr != old_dyn_addr)
    {
      status = i3c_bus_get_addr_slot_status(&master->bus,
                    dev->info.dyn_addr);
      if (status != I3C_ADDR_SLOT_FREE)
        {
          return -EBUSY;
        }

      i3c_bus_set_addr_slot_status(&master->bus,
               dev->info.dyn_addr,
               I3C_ADDR_SLOT_I3C_DEV);
    }

  if (master->ops->reattach_i3c_dev)
    {
      ret = master->ops->reattach_i3c_dev(dev, old_dyn_addr);
      if (ret)
        {
          i3c_master_put_i3c_addrs(dev);
          return ret;
        }
    }

  return 0;
}

static void i3c_master_detach_i3c_dev(FAR struct i3c_dev_desc *dev)
{
  FAR struct i3c_master_controller *master = i3c_dev_get_master(dev);

  /* Do not detach the master device itself. */

  if (master->this != dev && master->ops->detach_i3c_dev)
    {
      master->ops->detach_i3c_dev(dev);
    }

  i3c_master_put_i3c_addrs(dev);
  list_delete(&dev->common.node);
}

static void
i3c_master_register_new_i3c_devs(FAR struct i3c_master_controller *master)
{
  FAR struct i3c_dev_desc *desc;

  if (!master->init_done)
    {
      return;
    }

  i3c_bus_for_each_i3cdev(&master->bus, desc)
    {
      if (desc->dev || !desc->info.dyn_addr || desc == master->this)
        {
          continue;
        }

      desc->dev = kmm_zalloc(sizeof(*desc->dev));
      if (desc->dev == NULL)
        {
          continue;
        }

      desc->dev->bus = &master->bus;
      desc->dev->desc = desc;
    }
}

static void i3c_master_detach_free_devs(
               FAR struct i3c_master_controller *master)
{
  FAR struct i3c_dev_desc *i3cdev;
  FAR struct i3c_dev_desc *i3ctmp;

  list_for_every_entry_safe(&master->bus.devs.i3c, i3cdev, i3ctmp,
        struct i3c_dev_desc, common.node)
    {
      i3c_master_detach_i3c_dev(i3cdev);
      kmm_free(i3cdev);
    }
}

/****************************************************************************
 * Name: i3c_master_bus_init
 *
 * Description:
 *   Initialize an I3C bus
 *
 *   This function is following all initialisation steps described in the I3C
 *   specification:
 *
 *   1. Attach I2C devs to the master so that the master can fill its
 *      internal device table appropriately
 *
 *   2. Call &i3c_master_controller_ops->bus_init() method to initialize
 *      the master controller. That's usually where the bus mode is selected
 *      (pure bus or mixed fast/slow bus)
 *
 *   3. Instruct all devices on the bus to drop their dynamic address. This
 *      is particularly important when the bus was previously configured by
 *      someone else (for example the bootloader)
 *
 *   4. Disable all slave events.
 *
 *   5. Reserve address slots for I3C devices with init_dyn_addr. And if
 *      devices also have static_addr, try to pre-assign dynamic addresses
 *      requested by the FW with SETDASA and attach corresponding statically
 *      defined I3C devices to the master.
 *
 *   6. Do a DAA (Dynamic Address Assignment) to assign dynamic addresses to
 *      all remaining I3C devices
 *
 *   Once this is done, all I3C and I2C devices should be usable.
 *
 * Input Parameters:
 *   master - main master initializing the bus
 *
 * Returned Value:
 *   A 0 in case of success, an negative error code otherwise.
 ****************************************************************************/

static int i3c_master_bus_init(FAR struct i3c_master_controller *master)
{
  int ret;

  /* Now execute the controller specific ->bus_init() routine, which
   * might configure its internal logic to match the bus limitations.
   */

  ret = master->ops->bus_init(master);
  if (ret)
    {
      goto err_detach_devs;
    }

  /* The master device should have been instantiated in ->bus_init(),
   * complain if this was not the case.
   */

  if (master->this == NULL)
    {
      i3cerr("master_set_info() was not called in ->bus_init()\n");
      ret = -EINVAL;
      goto err_bus_cleanup;
    }

  /* Reset all dynamic address that may have been assigned before
   * (assigned by the bootloader for example).
   */

  ret = i3c_master_rstdaa_locked(master, I3C_BROADCAST_ADDR);
  if (ret && ret != I3C_ERROR_M2)
    {
      i3cerr("master_rstdaa_locked() failed:%d (no device or timeout)\n",
             ret);
    }

  /* Disable all slave events before starting DAA. */

  ret = i3c_master_disec_locked(master, I3C_BROADCAST_ADDR,
              I3C_CCC_EVENT_SIR | I3C_CCC_EVENT_MR |
              I3C_CCC_EVENT_HJ);
  if (ret && ret != I3C_ERROR_M2)
    {
      i3cerr("master_disec_locked() failed:%d (no device or timeout)\n",
             ret);
    }

  /* Reserve init_dyn_addr first, and then try to pre-assign dynamic
   * address and retrieve device information if needed.
   * In case pre-assign dynamic address fails, setting dynamic address to
   * the requested init_dyn_addr is retried after DAA is done in
   * i3c_master_add_i3c_dev_locked().
   */

  ret = i3c_master_do_daa(master);
  if (ret)
    {
      i3c_master_rstdaa_locked(master, I3C_BROADCAST_ADDR);
      i3cerr("master_do_daa failed:%d (no device)\n", ret);
    }

  return 0;

err_bus_cleanup:
  if (master->ops->bus_cleanup)
    {
      master->ops->bus_cleanup(master);
    }

err_detach_devs:
  i3c_master_detach_free_devs(master);

  return ret;
}

static void i3c_master_bus_cleanup(FAR struct i3c_master_controller *master)
{
  if (master->ops->bus_cleanup)
    {
      master->ops->bus_cleanup(master);
    }

  i3c_master_detach_free_devs(master);
}

static FAR struct i3c_dev_desc *
i3c_master_search_i3c_dev_duplicate(FAR struct i3c_dev_desc *refdev)
{
  FAR struct i3c_master_controller *master = i3c_dev_get_master(refdev);
  FAR struct i3c_dev_desc *i3cdev;

  i3c_bus_for_each_i3cdev(&master->bus, i3cdev)
    {
      if (i3cdev != refdev && i3cdev->info.pid == refdev->info.pid)
        {
          return i3cdev;
        }
    }

  return NULL;
}

static int i3c_master_i2c_adapter_xfer(FAR struct i2c_master_s *i2c,
               FAR struct i2c_msg_s *xfers, int nxfers)
{
  FAR struct i3c_master_controller *master = i2c_adapter_to_i3c_master(i2c);
  int i;
  int ret;
  uint16_t addr;

  if (xfers == NULL || master == NULL || nxfers <= 0)
    {
      return -EINVAL;
    }

  if (master->ops->i2c_xfers == NULL)
    {
      return -ENOTSUP;
    }

  /* Doing transfers to different devices is not supported. */

  addr = xfers[0].addr;
  for (i = 1; i < nxfers; i++)
    {
      if (addr != xfers[i].addr)
        {
          return -ENOTSUP;
        }
    }

  i3c_bus_normaluse_lock(&master->bus);
  ret = master->ops->i2c_xfers(master, xfers, nxfers);
  i3c_bus_normaluse_unlock(&master->bus);

  return ret ? ret : nxfers;
}

static struct i2c_ops_s  i3c_master_i2c_algo =
{
  .transfer = i3c_master_i2c_adapter_xfer,
};

static void i3c_master_unregister_i3c_devs(
         FAR struct i3c_master_controller *master)
{
  FAR struct i3c_dev_desc *i3cdev;

  i3c_bus_for_each_i3cdev(&master->bus, i3cdev)
    {
      if (i3cdev->dev == NULL)
        {
          continue;
        }

      i3cdev->dev->desc = NULL;
      i3cdev->dev->bus = NULL;
      kmm_free(i3cdev->dev);
      i3cdev->dev = NULL;
    }
}

static void i3c_master_handle_ibi(FAR void *arg)
{
  FAR struct i3c_ibi_slot *slot = (struct i3c_ibi_slot *)arg;
  FAR struct i3c_dev_desc *dev = slot->dev;
  FAR struct i3c_master_controller *master = i3c_dev_get_master(dev);
  struct i3c_ibi_payload payload;

  payload.data = slot->data;
  payload.len = slot->len;

  if (dev->dev)
    {
      dev->ibi->handler(dev->dev, &payload);
    }

  master->ops->recycle_ibi_slot(dev, slot);
  atomic_fetch_sub(&dev->ibi->pending_ibis, 1);
  if (!atomic_load(&dev->ibi->pending_ibis))
    {
      sem_post(&dev->ibi->all_ibis_handled);
    }
}

static void i3c_master_init_ibi_slot(FAR struct i3c_dev_desc *dev,
             FAR struct i3c_ibi_slot *slot)
{
  slot->dev = dev;
}

static int i3c_master_check_ops(
            FAR const struct i3c_master_controller_ops *ops)
{
  if (ops == NULL || ops->bus_init == NULL || ops->priv_xfers == NULL ||
      ops->send_ccc_cmd == NULL || ops->do_daa == NULL ||
      ops->i2c_xfers == NULL)
    {
      return -EINVAL;
    }

  if (ops->request_ibi &&
      (ops->enable_ibi == NULL || ops->disable_ibi == NULL ||
       ops->free_ibi == NULL || ops->recycle_ibi_slot == NULL))
    {
      return -EINVAL;
    }

  return 0;
}

/****************************************************************************
 * Name: i2c_unregister_driver
 *
 * Description:
 *   Perform an I2C driver unregister operation.
 *
 * Input Parameters:
 *   master - I3C master object.
 *
 ****************************************************************************/

static void i2c_unregister_driver(FAR struct i3c_master_controller *master)
{
  char devname[12];
  int i2c_bus_id = i3c_master_to_i2c_bus_number(master);

  snprintf(devname, sizeof(devname), "/dev/i2c%d", i2c_bus_id);
  nx_unlink(devname);
}

/****************************************************************************
 * Name: i3c_unregister_driver
 *
 * Description:
 *   Perform an I3C driver unregister operation.
 *
 * Input Parameters:
 *   master - I3C master object.
 *
 ****************************************************************************/

static void i3c_unregister_driver(FAR struct i3c_master_controller *master)
{
  char devname[12];

  snprintf(devname, sizeof(devname), "/dev/i3c%d",
           i3c_master_to_i3c_bus_number(master));
  nx_unlink(devname);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: i3c_master_send_ccc_cmd_locked
 ****************************************************************************/

int i3c_master_send_ccc_cmd_locked(
              FAR struct i3c_master_controller *master,
              FAR struct i3c_ccc_cmd *cmd)
{
  int ret;

  if (cmd == NULL || master == NULL)
    {
      return -EINVAL;
    }

  if (master->init_done &&
      !nxmutex_is_locked(&master->bus.lock))
    {
      return -EINVAL;
    }

  if (master->ops->send_ccc_cmd == NULL)
    {
      return -ENOTSUP;
    }

  if ((cmd->id & I3C_CCC_DIRECT) && (cmd->dests == NULL || !cmd->ndests))
    {
      return -EINVAL;
    }

  if (master->ops->supports_ccc_cmd &&
      !master->ops->supports_ccc_cmd(master, cmd))
    {
      return -ENOTSUP;
    }

  ret = master->ops->send_ccc_cmd(master, cmd);
  if (ret)
    {
      if (cmd->err != I3C_ERROR_UNKNOWN)
        {
          return cmd->err;
        }

      return ret;
    }

  return 0;
}

/****************************************************************************
 * Name: i3c_bus_normaluse_lock
 *
 * Description:
 *   I3C bus to take the lock on.
 *
 *   This function takes the bus lock for any operation that is not a
 *   maintenance operation (see i3c_bus_maintenance_lock() for
 *   non-exhaustive list of maintenance operations). Basically all
 *   communications with I3C devices are normal operations (HDR, SDR
 *   transfers or CCC commands that do not change bus state or I3C dynamic
 *   address).
 *
 * Input Parameters:
 *   bus - I3C bus to release the lock on.
 *
 ****************************************************************************/

void i3c_bus_normaluse_lock(FAR struct i3c_bus *bus)
{
  nxmutex_lock(&bus->lock);
}

/****************************************************************************
 * Name: i3c_bus_normaluse_unlock
 *
 * Description:
 *   Release the bus lock after a normal operation.
 *
 *   Should be called when a normal operation is done. See
 *   i3c_bus_normaluse_lock() for more details on what these
 *   normal operations are.
 *
 * Input Parameters:
 *   bus - I3C bus to release the lock on.
 *
 *
 ****************************************************************************/

void i3c_bus_normaluse_unlock(FAR struct i3c_bus *bus)
{
  nxmutex_unlock(&bus->lock);
}

/****************************************************************************
 * Name: i3c_master_get_free_addr
 *
 * Description:
 *   Get a free address on the bus.
 *
 *   This function must be called with the bus lock held in write mode.
 *
 * Input Parameters:
 *   master     - I3C master object.
 *   start_addr - Where to start searching.
 *
 * Returned Value:
 *   The first free address starting at @start_addr (included) or -ENOMEM
 *   if there's no more address available.
 *
 ****************************************************************************/

int i3c_master_get_free_addr(FAR struct i3c_master_controller *master,
                             unsigned char start_addr)
{
  return i3c_bus_get_free_addr(&master->bus, start_addr);
}

/****************************************************************************
 * Name: i3c_master_do_daa
 *
 * Description:
 *   Do a DAA (Dynamic Address Assignment)
 *
 *   This function is instantiating an I3C device object and adding it to
 *   the I3C device list. All device information are automatically retrieved
 *   using standard CCC commands.
 *
 *   The I3C device object is returned in case the master wants to attach
 *   private data to it using i3c_dev_set_master_data().
 *
 *   This function must be called with the bus lock held in write mode.
 *
 * Input Parameters:
 *   master - Master doing the DAA
 *
 * Returned Value:
 *   A 0 in case of success, an negative error code otherwise.
 *
 ****************************************************************************/

int i3c_master_do_daa(FAR struct i3c_master_controller *master)
{
  int ret;

  i3c_bus_maintenance_lock(&master->bus);
  ret = master->ops->do_daa(master);
  i3c_bus_maintenance_unlock(&master->bus);

  if (ret)
    {
      return ret;
    }

  i3c_bus_normaluse_lock(&master->bus);
  i3c_master_register_new_i3c_devs(master);
  i3c_bus_normaluse_unlock(&master->bus);

  return 0;
}

/****************************************************************************
 * Name: i3c_master_enec_locked
 *
 * Description:
 *   Send an ENEC CCC command
 *
 *   Sends an ENEC CCC command to enable some or all events coming from a
 *   specific slave, or all devices if @addr is %I3C_BROADCAST_ADDR.
 *
 *   This function must be called with the bus lock held in write mode.
 *
 * Input Parameters:
 *   master - Master used to send frames on the bus.
 *   addr   - A valid I3C slave address or %I3C_BROADCAST_ADDR
 *   evts   - Events to enable
 *
 * Returned Value:
 *   0 in case of success, a positive I3C error code if the error is
 *   one of the official Mx error codes, and a negative error code otherwise.
 *
 ****************************************************************************/

int i3c_master_enec_locked(FAR struct i3c_master_controller *master,
                           unsigned char addr, unsigned char evts)
{
  return i3c_master_enec_disec_locked(master, addr, true, evts);
}

/****************************************************************************
 * Name: i3c_master_entdaa_locked
 *
 * Description:
 *   Start a DAA (Dynamic Address Assignment) procedure
 *
 *   This function must be called with the bus lock held in write mode.
 *
 *   Note that this function only sends the ENTDAA CCC command, all the logic
 *   behind dynamic address assignment has to be handled in the I3C master
 *   driver.
 *
 * Input Parameters:
 *   master - Master used to send frames on the bus.
 *
 * Returned Value:
 *   0 in case of success, a positive I3C error code if the error is one of
 *   the official Mx error codes, and a negative error code otherwise.
 *
 ****************************************************************************/

int i3c_master_entdaa_locked(FAR struct i3c_master_controller *master)
{
  struct i3c_ccc_cmd_dest dest;
  struct i3c_ccc_cmd cmd;
  int ret;

  i3c_ccc_cmd_dest_init(&dest, I3C_BROADCAST_ADDR, 0);
  i3c_ccc_cmd_init(&cmd, false, I3C_CCC_ENTDAA, &dest, 1);
  ret = i3c_master_send_ccc_cmd_locked(master, &cmd);
  i3c_ccc_cmd_dest_cleanup(&dest);

  return ret;
}

/****************************************************************************
 * Name: i3c_master_disec_locked
 *
 * Description:
 *   Send a DISEC CCC command
 *
 *   Send a DISEC CCC command to disable some or all events coming from a
 *   specific slave, or all devices if @addr is %I3C_BROADCAST_ADDR.
 *
 *   This function must be called with the bus lock held in write mode.
 *
 * Input Parameters:
 *   master - Master used to send frames on the bus.
 *   addr   - A valid I3C slave address or %I3C_BROADCAST_ADDR
 *   evts   - Events to disable
 *
 * Returned Value:
 *   0 in case of success, a positive I3C error code if the error is one of
 *   the official Mx error codes, and a negative error code otherwise.
 *
 ****************************************************************************/

int i3c_master_disec_locked(FAR struct i3c_master_controller *master,
                            unsigned char addr, unsigned char evts)
{
  return i3c_master_enec_disec_locked(master, addr, false, evts);
}

/****************************************************************************
 * Name: i3c_master_set_info
 *
 * Description:
 *   Set master device information
 *
 *   Set master device info. This should be called from
 *   &i3c_master_controller_ops->bus_init().
 *
 *   Not all &i3c_device_info fields are meaningful for a master device.
 *   Here is a list of fields that should be properly filled:
 *
 *   - &i3c_device_info->dyn_addr
 *   - &i3c_device_info->bcr
 *   - &i3c_device_info->dcr
 *   - &i3c_device_info->pid
 *   - &i3c_device_info->hdr_cap if %I3C_BCR_HDR_CAP bit is set in
 *    &i3c_device_info->bcr
 *
 *   This function must be called with the bus lock held in maintenance mode.
 *
 * Input Parameters:
 *   master - Master used to send frames on the bus
 *   info   - I3C device information
 *
 * Returned Value:
 *   0 if @info contains valid information (not every piece of
 *   information can be checked, but we can at least make sure info->dyn_addr
 *   and @info->bcr are correct), -EINVAL otherwise.
 *
 ****************************************************************************/

int i3c_master_set_info(FAR struct i3c_master_controller *master,
                        FAR const struct i3c_device_info *info)
{
  FAR struct i3c_dev_desc *i3cdev;
  int ret;

  if (!i3c_bus_dev_addr_is_avail(&master->bus, info->dyn_addr))
    {
      return -EINVAL;
    }

  if (I3C_BCR_DEVICE_ROLE(info->bcr) == I3C_BCR_I3C_MASTER &&
      master->secondary)
    {
      return -EINVAL;
    }

  if (master->this)
    {
      return -EINVAL;
    }

  i3cdev = i3c_master_alloc_i3c_dev(master, info);
  if (i3cdev == NULL)
    {
      return -ENOMEM;
    }

  master->this = i3cdev;
  master->bus.cur_master = master->this;

  ret = i3c_master_attach_i3c_dev(master, i3cdev);
  if (ret)
    {
      goto err_free_dev;
    }

  return 0;

err_free_dev:
  kmm_free(i3cdev);

  return ret;
}

/****************************************************************************
 * Name: i3c_master_add_i3c_dev_locked
 *
 * Description:
 *   Add an I3C slave to the bus
 *
 *   This function is instantiating an I3C device object and adding it to
 *   the I3C device list. All device information are automatically retrieved
 *   using standard CCC commands.
 *
 *   The I3C device object is returned in case the master wants to attach
 *   private data to it using i3c_dev_set_master_data().
 *
 *   This function must be called with the bus lock held in write mode.
 *
 * Input Parameters:
 *   master - Master used to send frames on the bus
 *   addr   - I3C slave dynamic address assigned to the device
 *
 * Returned Value:
 *   A 0 in case of success, an negative error code otherwise.
 *
 ****************************************************************************/

int i3c_master_add_i3c_dev_locked(FAR struct i3c_master_controller *master,
                                  unsigned char addr)
{
  struct i3c_device_info info =
    {
      .dyn_addr = addr
    };

  FAR struct i3c_dev_desc *newdev;
  FAR struct i3c_dev_desc *olddev;
  unsigned char old_dyn_addr = addr;
  unsigned char expected_dyn_addr;
  struct i3c_ibi_setup ibireq;
  bool enable_ibi = false;
  int ret;

  memset(&ibireq, 0, sizeof(struct i3c_ibi_setup));
  if (master == NULL)
    {
      return -EINVAL;
    }

  newdev = i3c_master_alloc_i3c_dev(master, &info);
  if (newdev == NULL)
    {
      return -ENOMEM;
    }

  ret = i3c_master_attach_i3c_dev(master, newdev);
  if (ret)
    {
      goto err_free_dev;
    }

  ret = i3c_master_retrieve_dev_info(newdev);
  if (ret)
    {
      goto err_detach_dev;
    }

  olddev = i3c_master_search_i3c_dev_duplicate(newdev);
  if (olddev)
    {
      newdev->dev = olddev->dev;
      if (newdev->dev)
        {
          newdev->dev->desc = newdev;
        }

      /* We need to restore the IBI state too, so let's save the
       * IBI information and try to restore them after olddev has
       * been detached+released and its IBI has been stopped and
       * the associated resources have been freed.
       */

      nxmutex_lock(&olddev->ibi_lock);
      if (olddev->ibi)
        {
          ibireq.handler = olddev->ibi->handler;
          ibireq.max_payload_len = olddev->ibi->max_payload_len;
          ibireq.num_slots = olddev->ibi->num_slots;

          if (olddev->ibi->enabled)
            {
              enable_ibi = true;
              i3c_dev_disable_ibi_locked(olddev);
            }

          i3c_dev_free_ibi_locked(olddev);
        }

      nxmutex_unlock(&olddev->ibi_lock);

      old_dyn_addr = olddev->info.dyn_addr;

      i3c_master_detach_i3c_dev(olddev);
      kmm_free(olddev);
    }

  ret = i3c_master_reattach_i3c_dev(newdev, old_dyn_addr);
  if (ret)
    {
      goto err_detach_dev;
    }

  /* Depending on our previous state, the expected dynamic address might
   * differ:
   * - if the device already had a dynamic address assigned, let's try to
   *   re-apply this one
   * - if the device did not have a dynamic address and the firmware
   *   requested a specific address, pick this one
   * - in any other case, keep the address automatically assigned by the
   *   master
   */

  if (old_dyn_addr && old_dyn_addr != newdev->info.dyn_addr)
    {
      expected_dyn_addr = old_dyn_addr;
    }
  else
    {
      expected_dyn_addr = newdev->info.dyn_addr;
    }

  if (newdev->info.dyn_addr != expected_dyn_addr)
    {
      /* Try to apply the expected dynamic address. If it fails, keep
       * the address assigned by the master.
       */

      ret = i3c_master_setnewda_locked(master,
               newdev->info.dyn_addr,
               expected_dyn_addr);
      if (!ret)
        {
          old_dyn_addr = newdev->info.dyn_addr;
          newdev->info.dyn_addr = expected_dyn_addr;
          i3c_master_reattach_i3c_dev(newdev, old_dyn_addr);
        }
      else
        {
          i3cerr("Failed to assign reserved/old address%d%"PRIx64"\n",
                  master->bus.id, newdev->info.pid);
        }
    }

  /* Now is time to try to restore the IBI setup. If we're lucky,
   * everything works as before, otherwise, all we can do is complain.
   * FIXME: maybe we should add callback to inform the driver that it
   * should request the IBI again instead of trying to hide that from
   * him.
   */

  if (ibireq.handler)
    {
      nxmutex_lock(&newdev->ibi_lock);
      ret = i3c_dev_request_ibi_locked(newdev, &ibireq);
      if (ret)
        {
          i3cerr("Failed to request IBI on device %d-%"PRIx64"\n",
                  master->bus.id, newdev->info.pid);
        }
      else if (enable_ibi)
        {
          ret = i3c_dev_enable_ibi_locked(newdev);
          if (ret)
            {
              i3cerr("Failed to re-enable IBI on device %d-%"PRIx64"\n",
                                     master->bus.id, newdev->info.pid);
            }
        }

      nxmutex_unlock(&newdev->ibi_lock);
    }

  return 0;

err_detach_dev:
  if (newdev->dev && newdev->dev->desc)
    {
      newdev->dev->desc = NULL;
    }

  i3c_master_detach_i3c_dev(newdev);

err_free_dev:
  kmm_free(newdev);

  return ret;
}

/****************************************************************************
 * Name: i3c_master_queue_ibi
 *
 * Description:
 *   Queue an IBI
 *
 *   Queue an IBI to the controller workqueue. The IBI handler attached
 *   to the dev will be called from a workqueue context.
 *
 * Input Parameters:
 *   dev  - The device this IBI is coming from
 *   slot - The IBI slot used to store the payload
 *
 ****************************************************************************/

void i3c_master_queue_ibi(FAR struct i3c_dev_desc *dev,
                          FAR struct i3c_ibi_slot *slot)
{
  atomic_fetch_add(&dev->ibi->pending_ibis, 1);
  work_queue(HPWORK, &slot->work, i3c_master_handle_ibi, slot, 0);
}

/****************************************************************************
 * Name: i3c_generic_ibi_free_pool
 *
 * Description:
 *   Free a generic IBI pool
 *
 *   Free all IBI slots allated by a generic IBI pool.
 *
 * Input Parameters:
 *   pool - The IBI pool to free
 *
 ****************************************************************************/

void i3c_generic_ibi_free_pool(FAR struct i3c_generic_ibi_pool *pool)
{
  FAR struct i3c_generic_ibi_slot *slot;
  unsigned int nslots = 0;

  while (!list_is_empty(&pool->free_slots))
    {
      slot = list_first_entry(&pool->free_slots,
               struct i3c_generic_ibi_slot, node);
      list_delete(&slot->node);
      nslots++;
    }

  if (nslots != pool->num_slots)
    {
      i3cwarn("freed slots num is not equal to allocated slots num\n");
    }

  /* If the number of freed slots is not equal to the number of allocated
   * slots we have a leak somewhere.
   */

  kmm_free(pool->payload_buf);
  kmm_free(pool->slots);
  kmm_free(pool);
}

/****************************************************************************
 * Name: i3c_generic_ibi_alloc_pool
 *
 * Description:
 *   Create a generic IBI pool
 *
 *   Create a generic IBI pool based on the information provided in @req.
 *
 * Input Parameters:
 *   dev - The device this pool will be used for
 *   req - IBI setup request describing what the device driver expects
 *
 * Returned Value:
 *   A valid IBI pool in case of success, an ERR_PTR() otherwise.
 ****************************************************************************/

FAR struct i3c_generic_ibi_pool *
i3c_generic_ibi_alloc_pool(FAR struct i3c_dev_desc *dev,
                           FAR const struct i3c_ibi_setup *req)
{
  FAR struct i3c_generic_ibi_pool *pool;
  FAR struct i3c_generic_ibi_slot *slot;
  unsigned int i;

  struct i3c_generic_ibi_pool *ret;

  pool = kmm_zalloc(sizeof(*pool));
  if (pool == NULL)
    {
      return NULL;
    }

  list_initialize(&pool->free_slots);
  list_initialize(&pool->pending);

  pool->slots = kmm_calloc(req->num_slots, sizeof(*slot));
  if (pool->slots == NULL)
    {
      ret = NULL;
      goto err_free_pool;
    }

  if (req->max_payload_len)
    {
      pool->payload_buf = kmm_calloc(req->num_slots,
              req->max_payload_len);
      if (pool->payload_buf == NULL)
        {
          ret = NULL;
          goto err_free_pool;
        }
    }

  for (i = 0; i < req->num_slots; i++)
    {
      slot = &pool->slots[i];
      i3c_master_init_ibi_slot(dev, &slot->base);

      if (req->max_payload_len)
        {
          slot->base.data = pool->payload_buf +
                (i * req->max_payload_len);
        }

      list_add_tail(&pool->free_slots, &slot->node);
      pool->num_slots++;
    }

  return pool;

err_free_pool:
  i3c_generic_ibi_free_pool(pool);
  return ret;
}

/****************************************************************************
 * Name: i3c_generic_ibi_get_free_slot
 *
 * Description:
 *   Get a free slot from a generic IBI pool
 *
 *   Search for a free slot in a generic IBI pool.
 *   The slot should be returned to the pool using
 *   i3c_generic_ibi_recycle_slot()
 *   when it's no longer needed.
 *
 * Input Parameters:
 *   pool - The pool to query an IBI slot on
 *
 * Returned Value:
 *   A pointer to a free slot, or NULL if there's no free slot available.
 *
 ****************************************************************************/

FAR struct i3c_ibi_slot *
i3c_generic_ibi_get_free_slot(FAR struct i3c_generic_ibi_pool *pool)
{
  FAR struct i3c_generic_ibi_slot *slot = NULL;
  unsigned long flags;

  flags = spin_lock_irqsave(&pool->lock);
  if (!list_is_empty(&pool->free_slots))
    {
      slot = list_first_entry(&pool->free_slots,
                               struct i3c_generic_ibi_slot, node);
      if (slot)
        {
          list_delete(&slot->node);
        }
    }

  spin_unlock_irqrestore(&pool->lock, flags);

  return slot ? &slot->base : NULL;
}

/****************************************************************************
 * Name: i3c_generic_ibi_recycle_slot
 *
 * Description:
 *   Return a slot to a generic IBI pool
 *
 *   Add an IBI slot back to its generic IBI pool. Should be called from the
 *   master driver struct_master_controller_ops->recycle_ibi() method.
 *
 * Input Parameters:
 *   pool - The pool to return the IBI slot to
 *   s    - IBI slot to recycle
 *
 ****************************************************************************/

void i3c_generic_ibi_recycle_slot(FAR struct i3c_generic_ibi_pool *pool,
                                  FAR struct i3c_ibi_slot *s)
{
  FAR struct i3c_generic_ibi_slot *slot;
  unsigned long flags;

  if (s == NULL)
    {
      return;
    }

  slot = container_of(s, struct i3c_generic_ibi_slot, base);
  flags = spin_lock_irqsave(&pool->lock);
  list_add_tail(&pool->free_slots, &slot->node);
  spin_unlock_irqrestore(&pool->lock, flags);
}

int i3c_dev_do_priv_xfers_locked(FAR struct i3c_dev_desc *dev,
         FAR struct i3c_priv_xfer *xfers,
         int nxfers)
{
  FAR struct i3c_master_controller *master;

  if (dev == NULL)
    {
      return -ENOENT;
    }

  master = i3c_dev_get_master(dev);
  if (master == NULL || xfers == NULL)
    {
      return -EINVAL;
    }

  if (master->ops->priv_xfers == NULL)
    {
      return -ENOTSUP;
    }

  return master->ops->priv_xfers(dev, xfers, nxfers);
}

int i3c_dev_disable_ibi_locked(FAR struct i3c_dev_desc *dev)
{
  FAR struct i3c_master_controller *master;
  int ret;

  if (dev->ibi == NULL)
    {
      return -EINVAL;
    }

  master = i3c_dev_get_master(dev);
  ret = master->ops->disable_ibi(dev);
  if (ret)
    {
      return ret;
    }

  if (atomic_load(&dev->ibi->pending_ibis))
    {
      sem_wait(&dev->ibi->all_ibis_handled);
    }

  dev->ibi->enabled = false;
  sem_destroy(&dev->ibi->all_ibis_handled);

  return 0;
}

int i3c_dev_enable_ibi_locked(FAR struct i3c_dev_desc *dev)
{
  FAR struct i3c_master_controller *master = i3c_dev_get_master(dev);
  int ret;

  if (dev->ibi == NULL)
    {
      return -EINVAL;
    }

  ret = master->ops->enable_ibi(dev);
  if (!ret)
    {
      dev->ibi->enabled = true;
    }

  return ret;
}

int i3c_dev_request_ibi_locked(FAR struct i3c_dev_desc *dev,
                               FAR const struct i3c_ibi_setup *req)
{
  FAR struct i3c_master_controller *master = i3c_dev_get_master(dev);
  FAR struct i3c_device_ibi_info *ibi;
  int ret;

  if (master->ops->request_ibi == NULL)
    {
      return -ENOTSUP;
    }

  if (dev->ibi)
    {
      return -EBUSY;
    }

  ibi = kmm_zalloc(sizeof(*ibi));
  if (ibi == NULL)
    {
      return -ENOMEM;
    }

  atomic_init(&ibi->pending_ibis, 0);
  sem_init(&ibi->all_ibis_handled, 0, 1);
  ibi->handler = req->handler;
  ibi->max_payload_len = req->max_payload_len;
  ibi->num_slots = req->num_slots;

  dev->ibi = ibi;
  ret = master->ops->request_ibi(dev, req);
  if (ret)
    {
      kmm_free(ibi);
      dev->ibi = NULL;
    }

  return ret;
}

void i3c_dev_free_ibi_locked(FAR struct i3c_dev_desc *dev)
{
  FAR struct i3c_master_controller *master = i3c_dev_get_master(dev);

  if (dev->ibi == NULL)
    {
      return;
    }

  master->ops->free_ibi(dev);
  kmm_free(dev->ibi);
  dev->ibi = NULL;
}

int i3c_master_i2c_attach(FAR struct i3c_master_controller *master,
                          FAR struct i2c_config_s *config)
{
  int ret;

  i3c_bus_normaluse_lock(&master->bus);

  if (master->ops->attach_i2c_dev)
    {
      ret = master->ops->attach_i2c_dev(master, config);
      if (ret < 0)
        {
          i3c_bus_normaluse_unlock(&master->bus);
          return ret;
        }
    }

  i3c_bus_normaluse_unlock(&master->bus);

  return 0;
}

void i3c_master_detach_i2c_dev(FAR struct i3c_master_controller *master,
                               FAR struct i2c_config_s *config)
{
  i3c_bus_normaluse_lock(&master->bus);

  if (master->ops->detach_i2c_dev)
    {
      master->ops->detach_i2c_dev(master, config);
    }

  i3c_bus_normaluse_unlock(&master->bus);
}

/****************************************************************************
 * Name: i3c_master_register
 *
 * Description:
 *   Register an I3C master.
 *
 *   This function takes care of everything for you:
 *    - creates and initializes the I3C bus.
 *    - registers all I3C charactor driver that supports I3C transfer.
 *    - registers the I2C charactor driver that supports I2C transfer.
 *
 * Input Parameters:
 *   master    - Master used to send frames on the bus.
 *   ops       - The master controller operations
 *   secondary - True if you are registering a secondary master. Will return
 *     -ENOTSUP if set to true since secondary masters are not yet supported
 *
 * return:
 *   OK if the driver was successfully register; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int i3c_master_register(FAR struct i3c_master_controller *master,
                        FAR const struct i3c_master_controller_ops *ops,
                        bool secondary)
{
  unsigned long i2c_scl_rate = I3C_BUS_I2C_FM_PLUS_SCL_RATE;
  FAR struct i3c_bus *i3cbus = i3c_master_get_bus(master);
  enum i3c_bus_mode mode = I3C_BUS_MODE_PURE;
  int ret;

  /* We do not support secondary masters yet. */

  if (secondary)
    {
      return -ENOTSUP;
    }

  ret = i3c_master_check_ops(ops);
  if (ret)
    {
      return ret;
    }

  master->ops = ops;
  master->secondary = secondary;

  i3c_bus_init(i3cbus);

  if (master->mode)
    {
      mode = master->mode;
    }

  if (master->max_i2c_scl_rate)
    {
      i2c_scl_rate = master->max_i2c_scl_rate;
    }

  ret = i3c_bus_set_mode(i3cbus, mode, i2c_scl_rate);
  if (ret != 0)
    {
      return ret;
    }

  ret = i3c_master_bus_init(master);
  if (ret != 0)
    {
      return ret;
    }

  /* Expose I2C driver node so by the i2c_driver on our I3C Bus */

  master->i2c.ops = &i3c_master_i2c_algo;
  ret = i2c_register(&master->i2c, master->i2c_bus_id);
  if (ret < 0)
    {
      i3cerr("ERROR: Failed to register I2C%d.\n", master->i2c_bus_id);
      goto err_cleanup_bus;
    }

  /* We're done initializing the bus and the controller, we can now
   * register I3C devices discovered during the initial DAA.
   */

  master->init_done = true;

  /* Expose I3C driver node by the i3c_driver on our I3C Bus, i3c driver id
   * equal to i3c bus id.
   */

  ret = i3c_register(master, master->i3c_bus_id);
  if (ret < 0)
    {
      i3cerr("ERROR: Failed to register I2C%d.\n", master->i3c_bus_id);
      goto err_del_dev;
    }

  return 0;

err_del_dev:
  i2c_unregister_driver(master);

err_cleanup_bus:
  i3c_master_bus_cleanup(master);

  return ret;
}

/****************************************************************************
 * Name: i3c_master_unregister
 *
 * Description:
 *  Unregister an I3C master.
 *
 *  Basically undo everything done in i3c_master_register().
 *
 * Input Parameters:
 *   master - Master used to send frames on the bus.
 *
 ****************************************************************************/

void i3c_master_unregister(FAR struct i3c_master_controller *master)
{
  i2c_unregister_driver(master);
  i3c_master_unregister_i3c_devs(master);
  i3c_master_bus_cleanup(master);
  i3c_unregister_driver(master);
}
