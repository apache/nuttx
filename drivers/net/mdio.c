/****************************************************************************
 * drivers/net/mdio.c
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
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/net/mdio.h>
#include <nuttx/net/mii.h>
#include <debug.h>

/****************************************************************************
 * Private Defines
 ****************************************************************************/

#define MDIO_READ(d,a,r,v) d->lower->ops->read(d->lower, a, r, v);

#define MDIO_WRITE(d,a,r,v) d->lower->ops->write(d->lower, a, r, v);

#define MDIO_RESET(d,a,v) d->lower->ops->reset(d->lower, a, v);

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This is the opaque handle used by application code to access the
 * MDIO bus.
 */

struct mdio_bus_s
{
  /* Pointer to the lower-half driver's state */

  FAR struct mdio_lowerhalf_s *lower;

  /* For exclusive access to the bus */

  mutex_t lock;

  /* Last requested PHY reset line state:
   * -1 = unknown, 0 = deasserted, 1 = asserted.
   */

  int8_t reset_state;
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mdio_register
 *
 * Description:
 *   Register a new MDIO bus instance.
 *
 * Input Parameters:
 *   lower - An instance of the lower-half MDIO driver, with the ops vtable
 *           as the first member.
 *
 * Returned Value:
 *   A non-NULL handle on success; NULL on failure.
 *
 ****************************************************************************/

FAR struct mdio_bus_s *mdio_register(FAR struct mdio_lowerhalf_s *lower)
{
  FAR struct mdio_bus_s *dev;

  /* Allocate the upper-half MDIO driver state structure */

  dev = (FAR struct mdio_bus_s *)kmm_zalloc(sizeof(struct mdio_bus_s));
  if (dev != NULL)
    {
      /* Initialize the upper-half driver state */

      nxmutex_init(&dev->lock);
      dev->lower = lower;
      dev->reset_state = -1;
    }
  else
    {
      nerr("ERROR: Failed to allocate MDIO device structure\n");
    }

  return dev;
}

/****************************************************************************
 * Name: mdio_unregister
 *
 * Description:
 *   Unregister an MDIO bus instance.
 *
 * Input Parameters:
 *   dev - The MDIO bus handle returned by mdio_register.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int mdio_unregister(FAR struct mdio_bus_s *dev)
{
  DEBUGASSERT(dev != NULL);

  nxmutex_destroy(&dev->lock);
  kmm_free(dev);
  return 0;
}

/****************************************************************************
 * Name: mdio_lock
 *
 * Description:
 *   Acquire the MDIO bus mutex explicitly.  Used by callers that need to
 *   perform a multi-transaction atomic sequence on the bus (e.g. paged
 *   register access: select page -> read/write -> restore page) that must
 *   not be interleaved with other bus users such as MII ioctl.
 *
 *   Between mdio_lock() and mdio_unlock() the caller must use the lockless
 *   primitives mdio_read_nolock() / mdio_write_nolock(); calling the locking
 *   mdio_read()/mdio_write() while holding the lock would deadlock.
 *
 * Input Parameters:
 *   dev - The MDIO bus handle.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int mdio_lock(FAR struct mdio_bus_s *dev)
{
  DEBUGASSERT(dev != NULL);
  return nxmutex_lock(&dev->lock);
}

/****************************************************************************
 * Name: mdio_unlock
 *
 * Description:
 *   Release the MDIO bus mutex previously acquired with mdio_lock().  Must
 *   balance a successful mdio_lock().
 *
 * Input Parameters:
 *   dev - The MDIO bus handle.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void mdio_unlock(FAR struct mdio_bus_s *dev)
{
  DEBUGASSERT(dev != NULL);
  nxmutex_unlock(&dev->lock);
}

/****************************************************************************
 * Name: mdio_read_nolock
 *
 * Description:
 *   Clause 22 read WITHOUT taking the bus mutex.  The caller must already
 *   hold the bus lock via mdio_lock().  Intended for composing atomic
 *   multi-transaction sequences (see mdio_lock()).
 *
 * Input Parameters:
 *   dev     - The MDIO bus handle.
 *   phyaddr - The PHY address (0-31).
 *   regaddr - The PHY register address (0-31).
 *   value   - Pointer to store the value read.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int mdio_read_nolock(FAR struct mdio_bus_s *dev, uint8_t phyaddr,
                     uint8_t regaddr, FAR uint16_t *value)
{
  DEBUGASSERT(dev != NULL && dev->lower != NULL);
  DEBUGASSERT(dev->lower->ops->read != NULL);

  return MDIO_READ(dev, phyaddr, regaddr, value);
}

/****************************************************************************
 * Name: mdio_write_nolock
 *
 * Description:
 *   Clause 22 write WITHOUT taking the bus mutex.  The caller must already
 *   hold the bus lock via mdio_lock().  Intended for composing atomic
 *   multi-transaction sequences (see mdio_lock()).
 *
 * Input Parameters:
 *   dev     - The MDIO bus handle.
 *   phyaddr - The PHY address (0-31).
 *   regaddr - The PHY register address (0-31).
 *   value   - The value to write.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int mdio_write_nolock(FAR struct mdio_bus_s *dev, uint8_t phyaddr,
                      uint8_t regaddr, uint16_t value)
{
  DEBUGASSERT(dev != NULL && dev->lower != NULL);
  DEBUGASSERT(dev->lower->ops->write != NULL);

  return MDIO_WRITE(dev, phyaddr, regaddr, value);
}

/****************************************************************************
 * Name: mdio_read
 *
 * Description:
 *   Read a 16-bit value from a PHY register on the MDIO bus.
 *
 * Input Parameters:
 *   dev     - The MDIO bus handle.
 *   phyaddr - The PHY address (0-31).
 *   regaddr - The PHY register address (0-31).
 *   value   - A pointer to the location to store the read value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int mdio_read(FAR struct mdio_bus_s *dev, uint8_t phyaddr, uint8_t regaddr,
              FAR uint16_t *value)
{
  int ret;

  DEBUGASSERT(dev != NULL && dev->lower != NULL);
  DEBUGASSERT(dev->lower->ops->read != NULL);

  /* Take the mutex */

  ret = nxmutex_lock(&dev->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Call the lowerhalf driver's read method */

  ret = MDIO_READ(dev, phyaddr, regaddr, value);

  /* Release the mutex */

  nxmutex_unlock(&dev->lock);
  return ret;
}

/****************************************************************************
 * Name: mdio_write
 *
 * Description:
 *   Write a 16-bit value to a PHY register on the MDIO bus.
 *
 * Input Parameters:
 *   dev     - The MDIO bus handle.
 *   phyaddr - The PHY address (0-31).
 *   regaddr - The PHY register address (0-31).
 *   value   - The value to write.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int mdio_write(FAR struct mdio_bus_s *dev, uint8_t phyaddr, uint8_t regaddr,
               uint16_t value)
{
  int ret;

  DEBUGASSERT(dev != NULL && dev->lower != NULL);
  DEBUGASSERT(dev->lower->ops->write != NULL);

  /* Take the mutex */

  ret = nxmutex_lock(&dev->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Call the lowerhalf driver's write method */

  ret = MDIO_WRITE(dev, phyaddr, regaddr, value);

  /* Release the mutex */

  nxmutex_unlock(&dev->lock);
  return ret;
}

/****************************************************************************
 * Name: mdio_reset
 *
 * Description:
 *   Control the hardware reset line of a PHY on the MDIO bus.
 *
 * Input Parameters:
 *   dev     - The MDIO bus handle.
 *   phyaddr - The PHY address (0-31) to reset.
 *   value   - Non-zero to assert reset, zero to deassert.
 *
 * Returned Value:
 *   Zero (OK) on success; -ENOSYS if no reset op is provided; a negated
 *   errno value on failure.
 *
 ****************************************************************************/

int mdio_reset(FAR struct mdio_bus_s *dev, uint8_t phyaddr, int value)
{
  int ret;

  DEBUGASSERT(dev != NULL && dev->lower != NULL);

  /* No reset line wired / no lower-half op: nothing to do. */

  if (dev->lower->ops->reset == NULL)
    {
      return -ENOSYS;
    }

  ret = nxmutex_lock(&dev->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Idempotency: skip if the requested state is already in effect. */

  if (dev->reset_state == value)
    {
      nxmutex_unlock(&dev->lock);
      return OK;
    }

  /* Call the lowerhalf driver's reset method (it performs the
   * datasheet-mandated assert/deassert settle delay internally).
   */

  ret = MDIO_RESET(dev, phyaddr, value);
  if (ret >= 0)
    {
      dev->reset_state = (int8_t)value;
    }

  nxmutex_unlock(&dev->lock);
  return ret;
}

#ifdef CONFIG_MDIO_CLAUSE_45

/****************************************************************************
 * Name: mdio_c45_read
 *
 * Description:
 *   Read a Clause 45 MMD register.  Uses the lower-half native C45 read
 *   when available; otherwise performs indirect access over Clause 22
 *   registers 13 (MMD control) and 14 (MMD address/data).
 *
 * Input Parameters:
 *   dev     - The MDIO bus handle.
 *   phyaddr - The PHY address (0-31).
 *   devad   - The MMD device address.
 *   regaddr - The 16-bit MMD register address.
 *   value   - A pointer to the location to store the read value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int mdio_c45_read(FAR struct mdio_bus_s *dev, uint8_t phyaddr,
                  uint8_t devad, uint16_t regaddr, FAR uint16_t *value)
{
  int ret;

  DEBUGASSERT(dev != NULL && dev->lower != NULL && value != NULL);

  ret = nxmutex_lock(&dev->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Native C45 path (single set of transactions in the lower half) */

  if (dev->lower->ops->read_c45 != NULL)
    {
      ret = dev->lower->ops->read_c45(dev->lower, phyaddr, devad,
                                      regaddr, value);
      nxmutex_unlock(&dev->lock);
      return ret;
    }

  /* Fallback: indirect access over Clause 22 (reg 13/14) */

  ret = dev->lower->ops->write(dev->lower, phyaddr, MII_MMDCONTROL, devad);
  if (ret < 0)
    {
      goto out;
    }

  ret = dev->lower->ops->write(dev->lower, phyaddr, MII_MMDADDRDATA,
                               regaddr);
  if (ret < 0)
    {
      goto out;
    }

  ret = dev->lower->ops->write(dev->lower, phyaddr, MII_MMDCONTROL,
                               devad | MII_MMDCONTROL_FUNC_NOINCR);
  if (ret < 0)
    {
      goto out;
    }

  ret = dev->lower->ops->read(dev->lower, phyaddr, MII_MMDADDRDATA, value);

out:
  nxmutex_unlock(&dev->lock);
  return ret;
}

/****************************************************************************
 * Name: mdio_c45_write
 *
 * Description:
 *   Write a Clause 45 MMD register.  Uses the lower-half native C45 write
 *   when available; otherwise performs indirect access over Clause 22
 *   registers 13 (MMD control) and 14 (MMD address/data).
 *
 * Input Parameters:
 *   dev     - The MDIO bus handle.
 *   phyaddr - The PHY address (0-31).
 *   devad   - The MMD device address.
 *   regaddr - The 16-bit MMD register address.
 *   value   - The value to write.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int mdio_c45_write(FAR struct mdio_bus_s *dev, uint8_t phyaddr,
                   uint8_t devad, uint16_t regaddr, uint16_t value)
{
  int ret;

  DEBUGASSERT(dev != NULL && dev->lower != NULL);

  ret = nxmutex_lock(&dev->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Native C45 path */

  if (dev->lower->ops->write_c45 != NULL)
    {
      ret = dev->lower->ops->write_c45(dev->lower, phyaddr, devad,
                                       regaddr, value);
      nxmutex_unlock(&dev->lock);
      return ret;
    }

  /* Fallback: indirect access over Clause 22 (reg 13/14) */

  ret = dev->lower->ops->write(dev->lower, phyaddr, MII_MMDCONTROL, devad);
  if (ret < 0)
    {
      goto out;
    }

  ret = dev->lower->ops->write(dev->lower, phyaddr, MII_MMDADDRDATA,
                               regaddr);
  if (ret < 0)
    {
      goto out;
    }

  ret = dev->lower->ops->write(dev->lower, phyaddr, MII_MMDCONTROL,
                               devad | MII_MMDCONTROL_FUNC_NOINCR);
  if (ret < 0)
    {
      goto out;
    }

  ret = dev->lower->ops->write(dev->lower, phyaddr, MII_MMDADDRDATA, value);

out:
  nxmutex_unlock(&dev->lock);
  return ret;
}

#endif /* CONFIG_MDIO_CLAUSE_45 */
