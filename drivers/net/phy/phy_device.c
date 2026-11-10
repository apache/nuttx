/****************************************************************************
 * drivers/net/phy/phy_device.c
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

#include <nuttx/config.h>

#include <errno.h>
#include <inttypes.h>
#include <debug.h>
#include <string.h>
#include <syslog.h>

#include <nuttx/mutex.h>
#include <nuttx/kmalloc.h>
#include <nuttx/net/phy.h>
#include <nuttx/net/mii.h>
#include <nuttx/net/netdev_lowerhalf.h>
#include <nuttx/ethtool.h>

#include "phy.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR const struct phy_driver_s *
g_phy_drivers[CONFIG_NET_PHYLIB_MAX_DRIVERS];
static FAR struct phy_device_s *g_phy_devices[CONFIG_NET_PHYLIB_MAX_PHYS];
static mutex_t g_phy_lock = NXMUTEX_INITIALIZER;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: phy_find_driver
 *
 * Description:
 *   Find a matching PHY driver for the given PHY ID.
 *
 * Input Parameters:
 *   phy_id - 32-bit PHY identifier read from MII registers
 *
 * Returned Value:
 *   Pointer to the matching PHY driver, or the generic PHY driver
 *   if no specific match is found.
 *
 ****************************************************************************/

static FAR const struct phy_driver_s *phy_find_driver(uint32_t phy_id,
                                                      bool c45)
{
  int i;

  nxmutex_lock(&g_phy_lock);
  for (i = 0; i < CONFIG_NET_PHYLIB_MAX_DRIVERS; i++)
    {
      FAR const struct phy_driver_s *drv = g_phy_drivers[i];

      if (drv != NULL && drv->phy_id_mask != 0 &&
          (drv->phy_id & drv->phy_id_mask) ==
          (phy_id & drv->phy_id_mask))
        {
          nxmutex_unlock(&g_phy_lock);
          return drv;
        }
    }

  nxmutex_unlock(&g_phy_lock);

  /* Fallback: use genphy_c45 for C45 PHYs, genphy for C22 PHYs */

#ifdef CONFIG_MDIO_CLAUSE_45
  if (c45)
    {
      return &g_genphy_c45_driver;
    }
#else
  UNUSED(c45);
#endif

  return &g_genphy_driver;
}

/****************************************************************************
 * Name: phy_alloc_device
 *
 * Description:
 *   Allocate a PHY device and register it in the device table.
 *
 * Input Parameters:
 *   bus - MDIO bus to stamp into the slot as the in-use marker
 *
 * Returned Value:
 *   Pointer to a newly allocated PHY device structure, or NULL if the
 *   table is full or the allocation fails.
 *
 ****************************************************************************/

static FAR struct phy_device_s *phy_alloc_device(FAR struct mdio_bus_s *bus)
{
  FAR struct phy_device_s *phydev;
  int i;

  nxmutex_lock(&g_phy_lock);
  for (i = 0; i < CONFIG_NET_PHYLIB_MAX_PHYS; i++)
    {
      if (g_phy_devices[i] == NULL)
        {
          phydev = kmm_zalloc(sizeof(struct phy_device_s));
          if (phydev == NULL)
            {
              nxmutex_unlock(&g_phy_lock);
              return NULL;
            }

          /* Claim the slot while still holding g_phy_lock by publishing the
           * pointer now.  Releasing the lock with the slot still NULL would
           * let a concurrent allocator on another core grab the same slot.
           */

          phydev->bus = bus;
          g_phy_devices[i] = phydev;
          nxmutex_unlock(&g_phy_lock);
          return phydev;
        }
    }

  nxmutex_unlock(&g_phy_lock);
  return NULL;
}

/****************************************************************************
 * Name: phy_release_device
 *
 * Description:
 *   Remove a PHY device from the device table, tear down its lock and free
 *   it.  Shared by phy_device_free() (create/scan path) and
 *   phy_disconnect() (connected path).  The caller must have already
 *   detached the device (lower == NULL) and stopped the state
 *   machine.  Any driver-private state referenced by phydev->priv is owned
 *   by the vendor driver, which releases it (e.g. in its teardown path)
 *   before the device reaches here.
 *
 * Input Parameters:
 *   phydev - Pointer to the PHY device structure to release
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void phy_release_device(FAR struct phy_device_s *phydev)
{
  int i;

  /* Remove the device from the table and tear down its lock under
   * g_phy_lock, the same lock phy_alloc_device() uses to scan for a free
   * slot.  Doing both here serializes slot reuse: no other core can observe
   * the freed slot, reuse it and nxmutex_init() a fresh lock while we are
   * still destroying this one.
   */

  nxmutex_lock(&g_phy_lock);
  for (i = 0; i < CONFIG_NET_PHYLIB_MAX_PHYS; i++)
    {
      if (g_phy_devices[i] == phydev)
        {
          g_phy_devices[i] = NULL;
          break;
        }
    }

  nxmutex_destroy(&phydev->lock);
  nxmutex_unlock(&g_phy_lock);

  kmm_free(phydev);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: phy_driver_register
 *
 * Description:
 *   Register a PHY driver in the global driver table. The driver will
 *   be matched against PHY devices by PHY ID during device creation.
 *
 * Input Parameters:
 *   drv - Pointer to the PHY driver structure to register
 *
 * Returned Value:
 *   OK on success; -EINVAL if drv is NULL;
 *   -ENOMEM if the driver table is full.
 *
 ****************************************************************************/

int phy_driver_register(FAR const struct phy_driver_s *drv)
{
  int ret;
  int i;

  if (drv == NULL)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&g_phy_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Store the driver in the first free slot of the fixed-size table. */

  for (i = 0; i < CONFIG_NET_PHYLIB_MAX_DRIVERS; i++)
    {
      if (g_phy_drivers[i] == NULL)
        {
          g_phy_drivers[i] = drv;
          nxmutex_unlock(&g_phy_lock);

          ninfo("PHY driver '%s' registered "
                "(id=0x%08" PRIx32 " mask=0x%08" PRIx32 ")\n",
                drv->name, drv->phy_id, drv->phy_id_mask);
          return OK;
        }
    }

  nxmutex_unlock(&g_phy_lock);
  return -ENOMEM;
}

/****************************************************************************
 * Name: phy_device_create
 *
 * Description:
 *   Create a PHY device by reading its PHY ID from the MDIO bus and
 *   assigning a matching driver. The device is dynamically allocated and
 *   registered in the device table.
 *
 * Input Parameters:
 *   bus  - Pointer to the MDIO bus structure
 *   addr - PHY address on the bus (0-31)
 *
 * Returned Value:
 *   Pointer to the created PHY device on success; NULL on failure.
 *
 ****************************************************************************/

FAR struct phy_device_s *phy_device_create(FAR struct mdio_bus_s *bus,
                                           uint8_t addr)
{
  FAR struct phy_device_s *phydev;
  uint16_t id1;
  uint16_t id2;
  uint32_t phy_id;
  bool c45 = false;
  int ret;

  if (bus == NULL)
    {
      return NULL;
    }

#ifdef CONFIG_MDIO_CLAUSE_45
  /* Try native/indirect Clause 45 first: read the PHY ID from MMD1
   * (PMA/PMD).  This handles MDIO controllers configured in C45-only mode
   * where plain C22 reads would fail.  Fall back to C22 on error.
   */

  ret = mdio_c45_read(bus, addr, 1, MII_PHYID1, &id1);
  if (ret >= 0)
    {
      ret = mdio_c45_read(bus, addr, 1, MII_PHYID2, &id2);
    }

  if (ret >= 0 && !((id1 == 0x0000 && id2 == 0x0000) ||
                    (id1 == 0xffff && id2 == 0xffff)))
    {
      c45 = true;
    }
  else
#endif
    {
      ret = mdio_read(bus, addr, MII_PHYID1, &id1);
      if (ret < 0)
        {
          return NULL;
        }

      ret = mdio_read(bus, addr, MII_PHYID2, &id2);
      if (ret < 0)
        {
          return NULL;
        }
    }

  /* Reject invalid PHY IDs */

  if ((id1 == 0x0000 && id2 == 0x0000) ||
      (id1 == 0xffff && id2 == 0xffff))
    {
      return NULL;
    }

  phy_id = ((uint32_t)id1 << 16) | (uint32_t)id2;

  phydev = phy_alloc_device(bus);
  if (phydev == NULL)
    {
      nerr("ERROR: Failed to allocate PHY device\n");
      return NULL;
    }

  phydev->c45       = c45;
  phydev->phy_id    = phy_id;
  phydev->mdio_addr = addr;
  phydev->drv       = phy_find_driver(phy_id, phydev->c45);
  phydev->state     = PHY_DOWN;
  phydev->speed     = 0;
  phydev->duplex    = DUPLEX_UNKNOWN;
  phydev->link      = false;
  phydev->autoneg   = true;
  phydev->irq       = PHY_POLL;  /* Default to polling; MAC sets a real IRQ
                                  * number to enable interrupt mode. */

  nxmutex_init(&phydev->lock);

  ninfo("PHY device created: id=0x%08" PRIx32 " addr=%d drv='%s' c45=%d\n",
        phy_id, addr, phydev->drv->name, phydev->c45);

  return phydev;
}

/****************************************************************************
 * Name: phy_device_scan
 *
 * Description:
 *   Probe all 32 addresses on an MDIO bus and create a PHY device for
 *   every address that returns a valid PHY ID. Discovered devices are
 *   stored into the caller-provided array. The caller takes ownership of
 *   each returned phydev: connect the ones it uses with phy_connect() (and
 *   release them with phy_disconnect() on teardown), and release any
 *   discovered-but-unused phydev with phy_device_free().  A phydev that is
 *   neither connected nor freed leaks both its device-table slot and the
 *   memory backing it.
 *
 *   Use this only when PHY addresses are not known in advance. When the
 *   address is known (the common NuttX case, from board defconfig or the
 *   datasheet), call phy_device_create(bus, addr) directly instead.
 *
 * Input Parameters:
 *   bus     - Pointer to the MDIO bus structure to scan
 *   devs    - Caller-provided array to receive discovered phydev pointers
 *   maxdevs - Capacity of the devs[] array
 *
 * Returned Value:
 *   Number of PHY devices discovered (>= 0); negated errno on failure.
 *
 ****************************************************************************/

ssize_t phy_device_scan(FAR struct mdio_bus_s *bus,
                        FAR struct phy_device_s **devs, size_t maxdevs)
{
  size_t count = 0;
  uint8_t addr;
  FAR struct phy_device_s *phydev;

  if (bus == NULL || devs == NULL || maxdevs == 0)
    {
      return -EINVAL;
    }

  for (addr = 0; addr < 32 && count < maxdevs; addr++)
    {
      phydev = phy_device_create(bus, addr);
      if (phydev != NULL)
        {
          devs[count++] = phydev;
        }
    }

  ninfo("MDIO bus scan: found %zu PHY(s)\n", count);
  return count;
}

/****************************************************************************
 * Name: phy_connect
 *
 * Description:
 *   Connect a PHY device to a network device. Calls the PHY driver's
 *   config_init callback and transitions the PHY to the READY state.
 *
 * Input Parameters:
 *   phydev - Pointer to the PHY device structure
 *   lower  - Pointer to the network device lower half
 *
 * Returned Value:
 *   OK on success; -EINVAL if arguments are NULL;
 *   -EBUSY if the PHY is already attached.
 *
 ****************************************************************************/

int phy_connect(FAR struct phy_device_s *phydev,
                FAR struct netdev_lowerhalf_s *lower)
{
  int ret;

  if (phydev == NULL || lower == NULL)
    {
      return -EINVAL;
    }

  nxmutex_lock(&phydev->lock);

  if (phydev->lower != NULL)
    {
      nxmutex_unlock(&phydev->lock);
      return -EBUSY;
    }

  phydev->lower = lower;
  lower->phydev = phydev;

  /* Deassert the PHY hardware reset (release the RESET_N line) before
   * running config_init.  The request is forwarded to the lower-half
   * mdio_reset(); the lower half owns any idempotency and the
   * datasheet-mandated settle delay.  A PHY with no controllable reset
   * line is a quiet no-op.
   */

  phy_reset(phydev, 0);

  /* Call config_init */

  if (phydev->drv->config_init)
    {
      ret = phydev->drv->config_init(phydev);
      if (ret < 0)
        {
          phydev->lower = NULL;
          lower->phydev = NULL;
          nxmutex_unlock(&phydev->lock);
          return ret;
        }
    }

  phydev->state = PHY_READY;
  ninfo("PHY connected: addr=%d -> %s\n",
        phydev->mdio_addr, lower->netdev.d_ifname);
  nxmutex_unlock(&phydev->lock);
  return OK;
}

/****************************************************************************
 * Name: phy_disconnect
 *
 * Description:
 *   Disconnect a PHY device from its attached network device. Stops the
 *   state machine if running and transitions the PHY to the DOWN state.
 *
 * Input Parameters:
 *   phydev - Pointer to the PHY device structure
 *
 * Returned Value:
 *   OK on success; negated errno on failure.
 *
 ****************************************************************************/

int phy_disconnect(FAR struct phy_device_s *phydev)
{
  uint8_t addr;

  if (phydev == NULL)
    {
      return -EINVAL;
    }

  /* Always stop the state machine before tearing the device down.
   *
   * phy_stop() is idempotent and safe to call from any state: it disables
   * PHY interrupts, detaches the phylib-owned ISR (phy_free_interrupt() is
   * a no-op when nothing is attached) and cancels any pending work.  This
   * must run unconditionally: a PHY that reached PHY_ERROR (or was never
   * started) still has its ISR attached in interrupt mode, and skipping
   * phy_stop() would leave that ISR pointing at a phydev we are about to
   * free.  phy_stop() takes phydev->lock and runs work_cancel_sync()
   * itself, so it must not be called while holding phydev->lock.
   */

  phy_stop(phydev);

  nxmutex_lock(&phydev->lock);

  phydev->state = PHY_DOWN;

  if (phydev->lower != NULL)
    {
      phydev->lower->phydev = NULL;
      phydev->lower = NULL;
    }

  /* Assert the PHY hardware reset (drive RESET_N active) as the device is
   * torn down.  No-op when the lower half provides no reset op.
   */

  phy_reset(phydev, 1);

  /* Snapshot the address for the trace below: once the device is freed its
   * memory may be reused (and mdio_addr overwritten) by a concurrent
   * phy_device_create() on another core.
   */

  addr = phydev->mdio_addr;

  nxmutex_unlock(&phydev->lock);

  /* Free the device and release its table slot. */

  phy_release_device(phydev);

  ninfo("PHY disconnected: addr=%d\n", addr);
  return OK;
}

/****************************************************************************
 * Name: phy_device_free
 *
 * Description:
 *   Release a PHY device that was obtained from phy_device_create() or
 *   phy_device_scan() but never connected to a network device.  This is
 *   the symmetric counterpart to phy_device_create(): the caller must use
 *   it to free any discovered phydev it does not go on to phy_connect(),
 *   otherwise the device and its table slot leak.
 *
 *   A connected PHY (lower != NULL) must be released with
 *   phy_disconnect() instead, which stops the state machine first.
 *
 * Input Parameters:
 *   phydev - Pointer to the PHY device structure to free
 *
 * Returned Value:
 *   OK on success; -EINVAL if phydev is NULL;
 *   -EBUSY if the PHY is still connected (use phy_disconnect() instead).
 *
 ****************************************************************************/

int phy_device_free(FAR struct phy_device_s *phydev)
{
  uint8_t addr;

  if (phydev == NULL)
    {
      return -EINVAL;
    }

  nxmutex_lock(&phydev->lock);

  /* A connected device must go through phy_disconnect(): it owns a running
   * state machine and a carrier notification that this path does not
   * handle.
   */

  if (phydev->lower != NULL)
    {
      nxmutex_unlock(&phydev->lock);
      return -EBUSY;
    }

  addr = phydev->mdio_addr;

  nxmutex_unlock(&phydev->lock);

  /* Free the device and release its table slot. */

  phy_release_device(phydev);

  ninfo("PHY device freed: addr=%d\n", addr);
  return OK;
}
