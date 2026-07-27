/****************************************************************************
 * include/nuttx/net/phy.h
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

#ifndef __INCLUDE_NUTTX_NET_PHY_H
#define __INCLUDE_NUTTX_NET_PHY_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <signal.h>
#include <sys/types.h>

#include <nuttx/mutex.h>

#ifdef CONFIG_NET_PHYLIB
#  include <nuttx/net/mdio.h>
#  include <nuttx/wqueue.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Maximum number of phy_notify clients */

#ifndef CONFIG_PHY_NOTIFICATION_NCLIENTS
#  define CONFIG_PHY_NOTIFICATION_NCLIENTS 1
#endif

#ifdef CONFIG_NET_PHYLIB

/* Sentinel values for phy_device_s.irq:
 *
 *   PHY_POLL          - No interrupt source available/desired.  Link state
 *                       is discovered purely by periodic MDIO polling.
 *   PHY_MAC_INTERRUPT - The PHY has no independent interrupt line; the
 *                       attached MAC driver detects the PHY event in its
 *                       own ISR and calls phy_interrupt().  The phylib
 *                       core does NOT attach an ISR in this case.
 *   >= 0              - A real interrupt number (e.g. an INTB pin routed to
 *                       a GPIO/IRQ).  The phylib core attaches its own ISR
 *                       to this line internally from phy_start().
 *
 * In interrupt mode the state machine runs only when a PHY interrupt kicks
 * it; no periodic poll is used.
 */

#define PHY_POLL                      (-1)
#define PHY_MAC_INTERRUPT             (-2)

#endif /* CONFIG_NET_PHYLIB */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef CONFIG_NET_PHYLIB

/* PHY states */

enum phy_state_e
{
  PHY_DOWN = 0,
  PHY_READY,
  PHY_UP,
  PHY_RUNNING,
  PHY_NOLINK,
  PHY_HALTED,
  PHY_ERROR
};

/* PHY interface modes */

enum phy_interface_e
{
  PHY_INTERFACE_MII = 0,
  PHY_INTERFACE_GMII,
  PHY_INTERFACE_RMII,
  PHY_INTERFACE_RGMII,
  PHY_INTERFACE_RGMII_ID,
  PHY_INTERFACE_RGMII_TXID,
  PHY_INTERFACE_RGMII_RXID,
  PHY_INTERFACE_SGMII,
  PHY_INTERFACE_MAX
};

/* PHY device.
 *
 * struct phy_driver_s (the vendor driver callback table) is an internal
 * interface used only by PHY drivers under drivers/net/phy; its full
 * definition lives in drivers/net/phy/phy.h.  Only a forward declaration is
 * needed here for the phydev->drv back-pointer.
 */

struct phy_driver_s;
struct netdev_lowerhalf_s;

struct phy_device_s
{
  uint32_t phy_id;
  uint8_t  mdio_addr;
  FAR struct mdio_bus_s *bus;
  FAR const struct phy_driver_s *drv;
  FAR struct netdev_lowerhalf_s *lower;

  /* Link state */

  bool     c45;
  bool     link;
  uint8_t  duplex;
  uint32_t speed;
  enum phy_interface_e interface;

  /* Capabilities */

  uint32_t supported;
  uint32_t advertising;
  uint32_t lp_advertising;
  bool     autoneg;

  /* State machine */

  enum phy_state_e state;
  struct work_s state_work;

  /* Per-device lock.
   *
   * Serializes access to the PHY state fields (state/link/speed/duplex/
   * autoneg/advertising) between the state-machine work, ethtool/ioctl
   * callers and the start/stop/connect/disconnect lifecycle operations.
   * Serializes state transitions against ethtool/ioctl access.
   */

  mutex_t  lock;

  /* Interrupt mode.
   *
   * irq - PHY interrupt source, and the single source of truth for the
   *       runtime interrupt mode:
   *         PHY_POLL (-1)          - Polling mode: no interrupt source.
   *         PHY_MAC_INTERRUPT (-2) - The MAC driver owns the line and calls
   *                                  phy_interrupt; phylib attaches no ISR.
   *         >= 0                   - A real PHY interrupt line: the MAC
   *                                  driver supplies the SoC-specific INTB
   *                                  vector here before phy_start; phylib
   *                                  attaches its own ISR (phy_isr) via
   *                                  irq_attach().
   *
   *       The three cases are distinguished with phy_interrupt_is_valid()
   *       (irq >= 0, a real PHY line) and a plain PHY_POLL comparison
   *       (polling vs interrupt-driven), so no separate runtime-state field
   *       is needed.
   *
   * In interrupt mode the PHY ISR kicks state_work directly (delay 0); the
   * same work handle is reused for the polling and the interrupt-triggered
   * path, so no separate IRQ work is needed.
   */

  int      irq;

  /* Driver-private data.
   *
   * An opaque, driver-owned pointer set by the vendor PHY driver when it
   * needs per-device state that must survive across state-machine
   * callbacks.  Left NULL by drivers that are stateless.
   *
   * The phylib core treats this pointer as opaque: it neither dereferences
   * nor frees it.  A driver that stores heap-allocated state here is
   * responsible for releasing it in its own teardown path; a driver may
   * also use it as a simple per-device flag without allocating.
   */

  FAR void *priv;
};

#endif /* CONFIG_NET_PHYLIB */

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: phy_notify_subscribe
 *
 * Description:
 *   Setup up to deliver signals to the task identified by 'pid' when
 *   there is any change indicated by an interrupt from the PHY associated
 *   with 'intf'
 *
 *   NOTE: This function is intended to be called only from an Ethernet
 *   driver in support of the SIOCMIISIG ioctl command.  It should never
 *   by called directly by application logic.
 *
 * Input Parameters:
 *   intf  - Provides the name of the network interface, for example, "eth0".
 *   pid   - Identifies the task to receive the signal.  The special value
 *           of zero means to use the pid of the current task.
 *   event - Describe the way a task is to be notified
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_PHY_INTERRUPT
int phy_notify_subscribe(FAR const char *intf, pid_t pid,
                         FAR struct sigevent *event);
#endif

/****************************************************************************
 * Name: phy_notify_unsubscribe
 *
 * Description:
 *   Stop the deliver of signals for events from the PHY associated with
 *   'intf' to the task identified by 'pid'
 *
 *   NOTE: This function is intended to be called only from an Ethernet
 *   driver in support of the SIOCMIISIG ioctl command.  It should never
 *   by called directly by application logic.
 *
 * Input Parameters:
 *   intf  - Provides the name of the network interface, for example, "eth0".
 *   pid   - Identifies the task that was receiving notifications.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_PHY_INTERRUPT
int phy_notify_unsubscribe(FAR const char *intf, pid_t pid);
#endif

/****************************************************************************
 * PHYLIB Subsystem Declarations
 ****************************************************************************/

#ifdef CONFIG_NET_PHYLIB

/* PHY device lifecycle */

FAR struct phy_device_s *phy_device_create(FAR struct mdio_bus_s *bus,
                                           uint8_t addr);
int phy_device_free(FAR struct phy_device_s *phydev);
ssize_t phy_device_scan(FAR struct mdio_bus_s *bus,
                        FAR struct phy_device_s **devs, size_t maxdevs);

/* PHY connect/disconnect */

int phy_connect(FAR struct phy_device_s *phydev,
                FAR struct netdev_lowerhalf_s *lower);
int phy_disconnect(FAR struct phy_device_s *phydev);

/* PHY state machine */

int phy_start(FAR struct phy_device_s *phydev);
int phy_stop(FAR struct phy_device_s *phydev);

/* PHY interrupt handling
 *
 * phy_interrupt() is the entry point a MAC/board driver calls from its
 * own ISR (the PHY_MAC_INTERRUPT case); it simply triggers the PHY state
 * machine.  It is safe to call from interrupt context.
 *
 * The phylib-owned ISR for a real PHY interrupt line (phy_device_s.irq >= 0)
 * is attached/detached internally by phy_start()/phy_stop(); on attach
 * failure the PHY silently falls back to PHY_POLL.  For PHY_POLL and
 * PHY_MAC_INTERRUPT the MAC ISR is expected to call phy_interrupt()
 * directly.
 */

void phy_interrupt(FAR struct phy_device_s *phydev);

/* Upper half integration (phy_core.c) */

int phy_ioctl(FAR struct phy_device_s *phydev, int cmd, unsigned long arg);

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: phy_reset
 *
 * Description:
 *   Control the PHY hardware reset line.  Thin wrapper over the MDIO bus
 *   reset op (forwards to mdio_reset()).
 *   value != 0 asserts reset, value == 0 deasserts.
 *   Idempotency and the per-PHY settle delay are handled by mdio_reset()
 *   and the lower half; a PHY without a reset line is a quiet no-op.
 *
 * Input Parameters:
 *   phydev - Pointer to the PHY device structure
 *   value  - Non-zero to assert reset, zero to deassert
 *
 ****************************************************************************/

static inline void phy_reset(FAR struct phy_device_s *phydev,
                             int value)
{
  if (phydev != NULL && phydev->bus != NULL)
    {
      mdio_reset(phydev->bus, phydev->mdio_addr, value);
    }
}

#endif /* CONFIG_NET_PHYLIB */

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_NET_PHY_H */
