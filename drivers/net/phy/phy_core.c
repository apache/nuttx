/****************************************************************************
 * drivers/net/phy/phy_core.c
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
#include <debug.h>
#include <inttypes.h>
#include <string.h>
#include <sys/types.h>
#include <syslog.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/net/phy.h>
#include <nuttx/net/mii.h>
#include <nuttx/net/ioctl.h>
#include <nuttx/net/netdev_lowerhalf.h>
#include <nuttx/ethtool.h>

#include <net/if.h>

#include "phy.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PHY_POLL_TICKS MSEC2TICK(CONFIG_NET_PHYLIB_POLL_INTERVAL)

/* The PHY core currently stores each link mode mask in one 32-bit word. */

#define PHY_LINK_MODE_MASK_NWORDS 1

/* phy_interrupt_is_valid - true only when phy_device_s.irq is a real,
 * attachable interrupt number.  Both sentinels (PHY_POLL, PHY_MAC_INTERRUPT)
 * are negative, so a non-negative irq is a real, attachable IRQ number.
 */

#define phy_interrupt_is_valid(p) ((p)->irq >= 0)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: phy_check_link
 *
 * Description:
 *   Read the current PHY status and run the link up/down state transitions,
 *   updating phydev->state.  Shared by both the polling work and the
 *   interrupt work so the two paths behave identically.
 *
 *   Carrier notification (netdev_lower_carrier_on/off) is NOT performed
 *   here: it takes the netdev d_lock and would invert the lock order used
 *   by the ioctl path (d_lock then phydev->lock), risking an AB-BA
 *   deadlock.  The caller therefore compares phydev->state before and after
 *   this call and performs the carrier notification after releasing
 *   phydev->lock.
 *
 * Input Parameters:
 *   phydev - Pointer to the PHY device structure
 *
 * Returned Value:
 *   None.  The new link state is reflected in phydev->state.
 *
 ****************************************************************************/

static void phy_check_link(FAR struct phy_device_s *phydev)
{
  int ret;

  /* In interrupt mode, service the PHY interrupt source first so INTB
   * de-asserts and a fresh edge can be latched.  Both the polling and the
   * interrupt paths share this single state machine entry point: the
   * interrupt path only kicks the state machine, and the driver's
   * handle_interrupt() does the actual decode/ack.
   *
   *   handle_interrupt - the driver reads its own vendor interrupt-status
   *     register, decodes ALL pending sources (link and non-link), services
   *     non-link diagnostic events internally, and returns:
   *     >0 : a link-relevant event is pending -> re-read link status below
   *      0 : no link-relevant event           -> skip the link re-read
   *     <0 : MDIO/HW failure (PHY unreachable) -> enter PHY_ERROR
   */

  if (phydev->irq != PHY_POLL && phydev->drv->handle_interrupt)
    {
      ret = phydev->drv->handle_interrupt(phydev);
      if (ret < 0)
        {
          /* The PHY is unreachable (could not even read its interrupt
           * status register).  Enter the error state: force the carrier
           * off and stop rescheduling.  Recovery needs an explicit
           * stop/start cycle.
           */

          nerr("ERROR: handle_interrupt failed: %d\n", ret);
          phydev->state = PHY_ERROR;
          return;
        }
      else if (ret == 0)
        {
          /* No link-relevant event (e.g. a pure diagnostic interrupt that
           * the driver already serviced internally).  Leave the link state
           * untouched and do not re-read status.
           */

          return;
        }

      /* ret > 0: fall through to re-read link status. */
    }

  /* Read current PHY status */

  if (phydev->drv->read_status)
    {
      ret = phydev->drv->read_status(phydev);
      if (ret < 0)
        {
          /* A failed status read means the PHY is unreachable.  Enter the
           * error state rather than silently retrying every poll period.
           */

          nerr("ERROR: read_status failed: %d\n", ret);
          phydev->state = PHY_ERROR;
          return;
        }
    }

  /* Handle state transitions based on link status */

  switch (phydev->state)
    {
      case PHY_UP:
      case PHY_NOLINK:
        if (phydev->link)
          {
            phydev->state = PHY_RUNNING;
            ninfo("PHY link up: %" PRIu32 " Mbps %s\n", phydev->speed,
                  phydev->duplex == DUPLEX_FULL ? "full" : "half");
          }
        else if (phydev->state == PHY_UP)
          {
            phydev->state = PHY_NOLINK;
          }
        break;

      case PHY_RUNNING:
        if (!phydev->link)
          {
            phydev->state = PHY_NOLINK;
            ninfo("PHY link down\n");
          }
        break;

      default:
        break;
    }
}

/****************************************************************************
 * Name: phy_work
 *
 * Description:
 *   Work queue callback for PHY state machine polling. Reads the current
 *   PHY link status and handles state transitions between UP, RUNNING,
 *   and NOLINK. Notifies the attached network device of carrier changes.
 *
 *   In polling mode this runs every PHY_POLL_TICKS.  In interrupt mode it
 *   runs only when kicked by a PHY interrupt and does not reschedule
 *   itself, trusting the PHY interrupt to report every link change.
 *
 * Input Parameters:
 *   arg - Pointer to the PHY device structure (cast to void *)
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void phy_work(FAR void *arg)
{
  FAR struct phy_device_s *phydev = (FAR struct phy_device_s *)arg;
  FAR struct netdev_lowerhalf_s *lower;
  enum phy_state_e oldstate;
  enum phy_state_e newstate;

  if (phydev == NULL)
    {
      return;
    }

  nxmutex_lock(&phydev->lock);

  /* Run the link state transitions.  phy_check_link() only updates
   * phydev->state; the carrier notification is derived from the state
   * change and performed after the unlock (see below).  Both the old and
   * new state are snapshotted under the lock so the comparison below does
   * not race with a concurrent phy_stop()/phy_start().
   */

  oldstate = phydev->state;
  phy_check_link(phydev);
  newstate = phydev->state;

  /* Snapshot the attached device while holding the lock; the carrier
   * notification itself must run after the unlock (see below).
   */

  lower = phydev->lower;

  /* If we own a real interrupt line, the source has now been decoded and
   * acknowledged inside phy_check_link()->handle_interrupt(), so it is safe
   * to unmask the line that phy_isr() disabled (one-shot
   * equivalent).  Skip this while halted/errored: phy_stop() detaches the
   * IRQ, and in PHY_ERROR the PHY is unreachable so leaving the line masked
   * avoids an interrupt storm until an explicit stop/start recovery.
   */

  if (phy_interrupt_is_valid(phydev) &&
      phydev->state != PHY_HALTED && phydev->state != PHY_ERROR)
    {
      up_enable_irq(phydev->irq);
    }

  /* In polling mode, reschedule the next poll while in an active state.
   * In interrupt mode we trust the PHY interrupt to kick the state machine
   * on every link change, so no periodic poll is queued.
   */

  if (phydev->irq == PHY_POLL &&
      (phydev->state == PHY_UP ||
       phydev->state == PHY_RUNNING ||
       phydev->state == PHY_NOLINK))
    {
      work_queue(LPWORK, &phydev->state_work,
                 phy_work, phydev, PHY_POLL_TICKS);
    }

  nxmutex_unlock(&phydev->lock);

  /* Perform the carrier notification outside the phydev->lock critical
   * section, driven by the state change observed above.  Entering
   * PHY_RUNNING means link up; leaving it means link down.
   *
   * netdev_lower_carrier_on/off() take the netdev d_lock, while the ioctl
   * path takes d_lock before phydev->lock.  The carrier notification is
   * synchronous, so calling it while holding phydev->lock would invert that
   * order and risk an AB-BA deadlock.  Performing it here, after the
   * unlock, avoids that.  This mirrors phy_stop().
   */

  if (newstate != oldstate)
    {
      if (newstate == PHY_RUNNING)
        {
          netdev_lower_carrier_on(lower);
        }
      else if (oldstate == PHY_RUNNING)
        {
          netdev_lower_carrier_off(lower);
        }
    }
}

/****************************************************************************
 * Name: phy_isr
 *
 * Description:
 *   Low-level interrupt handler attached to a real PHY interrupt line
 *   (phy_device_s.irq >= 0) by phy_request_interrupt().  Runs in hard
 *   interrupt context, so it must NOT touch the MDIO bus (reading the PHY
 *   would block).  It only kicks the shared state machine work, mirroring
 *   the "trigger only" half.  The actual PHY
 *   interrupt-status decode/ack happens later in phy_check_link() ->
 *   handle_interrupt(), which runs in the LPWORK thread context.
 *
 *   NuttX has no threaded-IRQ primitive, so the work-queue
 *   hand-off is what gives us the sleepable context needed to read MDIO.
 *
 * Input Parameters:
 *   irq     - The interrupt number (unused)
 *   context - Saved register context (unused)
 *   arg     - Pointer to the PHY device structure
 *
 * Returned Value:
 *   OK.
 *
 ****************************************************************************/

static int phy_isr(int irq, FAR void *context, FAR void *arg)
{
  FAR struct phy_device_s *phydev = (FAR struct phy_device_s *)arg;

  /* Mask the line until the worker has serviced (and de-asserted) the PHY
   * interrupt source.  This emulates a one-shot interrupt: without it a
   * level-triggered INTB that stays asserted until the status register is
   * read in the worker would re-enter this handler forever.  The worker
   * re-enables the line once handle_interrupt() has cleared the source.
   */

  up_disable_irq(irq);

  /* Kick the shared state machine work to run immediately.  If the work is
   * already pending/running, work_queue() simply re-queues it; the PHY
   * status is read freshly when it runs, so coalescing is harmless.
   */

  work_queue(LPWORK, &phydev->state_work, phy_work, phydev, 0);
  return OK;
}

/****************************************************************************
 * Name: phy_free_interrupt
 *
 * Description:
 *   Disable and detach the phylib-owned ISR previously attached by
 *   phy_request_interrupt().  A no-op
 *   if this PHY does not own an attached line.
 *
 * Input Parameters:
 *   phydev - Pointer to the PHY device structure
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void phy_free_interrupt(FAR struct phy_device_s *phydev)
{
  /* Only a real PHY line (irq >= 0) has a phylib-owned ISR to detach.  A
   * MAC-owned line (PHY_MAC_INTERRUPT) or polling (PHY_POLL) has none.
   * Mask the line, then detach the phylib handler from the interrupt
   * number supplied by the MAC driver.
   */

  if (phy_interrupt_is_valid(phydev))
    {
      up_disable_irq(phydev->irq);
      irq_detach(phydev->irq);

      ninfo("PHY ISR detached from irq %d\n", phydev->irq);
    }
}

/****************************************************************************
 * Name: phy_request_interrupt
 *
 * Description:
 *   Bring the PHY up in interrupt-driven mode: for a real PHY interrupt
 *   line (phy_device_s.irq >= 0) attach the phylib-owned ISR, then program
 *   the PHY's interrupt-enable register via config_intr().  A MAC-owned
 *   line (PHY_MAC_INTERRUPT) skips the ISR attach (the MAC ISR calls
 *   phy_interrupt() itself) but still enables the PHY interrupt output.
 *
 *   A failure here is not hidden by degrading to polling: attaching a bad
 *   irq is a board misconfiguration, and a config_intr failure means the
 *   PHY is unreachable over MDIO (polling would fail the same way).  The
 *   error is returned so phy_start() can drive the PHY into PHY_ERROR.
 *
 * Input Parameters:
 *   phydev - Pointer to the PHY device structure
 *
 * Assumptions:
 *   Called from phy_start() with phydev->irq != PHY_POLL and a non-NULL
 *   config_intr callback.
 *
 * Returned Value:
 *   OK on success; a negated errno if the interrupt could not be set up.
 *
 ****************************************************************************/

static int phy_request_interrupt(FAR struct phy_device_s *phydev)
{
  int ret;

  /* A real PHY interrupt line (irq >= 0) needs a phylib-owned ISR attached
   * to the interrupt number supplied by the MAC/board driver.  A MAC-owned
   * line (PHY_MAC_INTERRUPT) has no phylib ISR; the MAC driver calls
   * phy_interrupt() itself.
   */

  if (phy_interrupt_is_valid(phydev))
    {
      ret = irq_attach(phydev->irq, phy_isr, phydev);
      if (ret < 0)
        {
          /* A bad interrupt number is a board/integration error.  Report it
           * instead of silently falling back to polling: the caller
           * requested interrupt mode with a concrete irq, so honour that
           * contract and surface the misconfiguration.
           */

          nerr("ERROR: irq_attach(%d) failed: %d\n", phydev->irq, ret);
          return ret;
        }

      up_enable_irq(phydev->irq);
    }

  /* Program the PHY's interrupt-enable register.  Both a real PHY line and
   * a MAC-owned line need this.
   */

  ret = phydev->drv->config_intr(phydev, true);
  if (ret < 0)
    {
      /* config_intr programs the PHY's interrupt-enable register over MDIO.
       * A failure here is a genuine setup error (PHY unreachable over MDIO,
       * or the driver cannot enable the PHY's interrupt).  Do not mask it by
       * degrading to polling: the polling path (read_status) shares the same
       * MDIO bus, so the fault would only resurface as a link that never
       * comes up.  Detach the ISR and report the error so phy_start() drives
       * the PHY into PHY_ERROR, consistent with how phy_check_link() treats
       * a read_status/handle_interrupt failure.
       */

      nerr("ERROR: config_intr failed: %d\n", ret);
      phy_free_interrupt(phydev);
      return ret;
    }

  ninfo("PHY interrupt mode enabled (irq=%d)\n", phydev->irq);
  return OK;
}

/****************************************************************************
 * Name: phy_ethtool_get_link_settings
 *
 * Description:
 *   Get the current link settings from the PHY device, including speed,
 *   duplex, auto-negotiation state, and advertising capabilities.
 *
 * Input Parameters:
 *   phydev - Pointer to the PHY device structure
 *   cmd    - Pointer to ethtool link settings structure to fill
 *
 * Returned Value:
 *   OK on success; negated errno on failure.
 *
 ****************************************************************************/

static int phy_ethtool_get_link_settings(
      FAR struct phy_device_s *phydev,
      FAR struct ethtool_link_settings *cmd)
{
  /* Negotiate the number of 32-bit words in each link mode mask. */

  if (cmd->link_mode_masks_nwords != PHY_LINK_MODE_MASK_NWORDS)
    {
      memset(cmd, 0, sizeof(*cmd));
      cmd->cmd = ETHTOOL_GLINKSETTINGS;
      cmd->link_mode_masks_nwords = -PHY_LINK_MODE_MASK_NWORDS;
      return OK;
    }

  nxmutex_lock(&phydev->lock);

  memset(cmd, 0, sizeof(*cmd));
  cmd->cmd            = ETHTOOL_GLINKSETTINGS;
  cmd->speed          = phydev->speed;
  cmd->duplex         = phydev->duplex;
  cmd->autoneg        = phydev->autoneg ? AUTONEG_ENABLE : AUTONEG_DISABLE;
  cmd->phy_address    = phydev->mdio_addr;
  cmd->mdio_support   = phydev->c45 ? ETH_MDIO_SUPPORTS_C45 :
                                      ETH_MDIO_SUPPORTS_C22;
  cmd->link_mode_masks_nwords = PHY_LINK_MODE_MASK_NWORDS;
  cmd->link_mode_masks[0] = phydev->supported;
  cmd->link_mode_masks[1] = phydev->advertising;
  cmd->link_mode_masks[2] = phydev->lp_advertising;

  nxmutex_unlock(&phydev->lock);

  return OK;
}

/****************************************************************************
 * Name: phy_ethtool_set_link_settings
 *
 * Description:
 *   Set the link settings on the PHY device. Updates the advertising
 *   capabilities and restarts auto-negotiation if configured.
 *
 * Input Parameters:
 *   phydev - Pointer to the PHY device structure
 *   cmd    - Pointer to ethtool link settings with desired values
 *
 * Returned Value:
 *   OK on success; negated errno on failure.
 *
 ****************************************************************************/

static int phy_ethtool_set_link_settings(
      FAR struct phy_device_s *phydev,
      FAR const struct ethtool_link_settings *cmd)
{
  uint32_t advertising;
  int ret = OK;

  if (cmd->link_mode_masks_nwords != PHY_LINK_MODE_MASK_NWORDS)
    {
      return -EINVAL;
    }

  /* link_mode_masks[] layout matches the get path:
   *   [0] supported      (read-only)
   *   [1] advertising    (writable - the value being set here)
   *   [2] lp_advertising (read-only)
   */

  advertising = cmd->link_mode_masks[1];

  nxmutex_lock(&phydev->lock);

  /* Validate advertising is subset of supported */

  if (advertising & ~phydev->supported)
    {
      nxmutex_unlock(&phydev->lock);
      return -EINVAL;
    }

  phydev->advertising = advertising;
  phydev->autoneg = (cmd->autoneg == AUTONEG_ENABLE);

  /* Restart auto-negotiation */

  if (phydev->drv->config_aneg)
    {
      ret = phydev->drv->config_aneg(phydev);
    }

  nxmutex_unlock(&phydev->lock);
  return ret;
}

/****************************************************************************
 * Name: phy_ethtool_nway_reset
 *
 * Description:
 *   Restart auto-negotiation on the PHY device by calling the driver's
 *   config_aneg callback.
 *
 * Input Parameters:
 *   phydev - Pointer to the PHY device structure
 *
 * Returned Value:
 *   OK on success; -EOPNOTSUPP if no config_aneg callback is available;
 *   negated errno on failure.
 *
 ****************************************************************************/

static int phy_ethtool_nway_reset(FAR struct phy_device_s *phydev)
{
  int ret;

  nxmutex_lock(&phydev->lock);

  if (phydev->drv->config_aneg)
    {
      ret = phydev->drv->config_aneg(phydev);
    }
  else
    {
      ret = -EOPNOTSUPP;
    }

  nxmutex_unlock(&phydev->lock);
  return ret;
}

/****************************************************************************
 * Name: phy_mii_ioctl
 *
 * Description:
 *   Handle MII ioctls when a PHY device is connected. Supports
 *   SIOCGMIIPHY, SIOCGMIIREG, and SIOCSMIIREG commands.
 *
 * Input Parameters:
 *   phydev - Pointer to the PHY device structure
 *   cmd    - MII ioctl command
 *   arg    - Pointer to mii_ioctl_data_s structure
 *
 * Returned Value:
 *   OK on success; -ENOTTY if not handled; negated errno on error.
 *
 ****************************************************************************/

static int phy_mii_ioctl(FAR struct phy_device_s *phydev,
                         int cmd, unsigned long arg)
{
  FAR struct mii_ioctl_data_s *mii =
    (FAR struct mii_ioctl_data_s *)(uintptr_t)arg;
  int ret;

  if (mii == NULL)
    {
      return -ENOTTY;
    }

  switch (cmd)
    {
      case SIOCGMIIPHY:
        mii->phy_id = phydev->mdio_addr;
        return OK;

      case SIOCGMIIREG:
        ret = phy_read(phydev, mii->reg_num);
        if (ret < 0)
          {
            return ret;
          }

        mii->val_out = ret;
        return OK;

      case SIOCSMIIREG:
        return phy_write(phydev, mii->reg_num, mii->val_in);

      default:
        return -ENOTTY;
    }
}

/****************************************************************************
 * Name: phy_ethtool_ioctl
 *
 * Description:
 *   Handle ethtool ioctls when a PHY device is connected. Dispatches
 *   ETHTOOL_GLINKSETTINGS, ETHTOOL_SLINKSETTINGS, ETHTOOL_NWAY_RST,
 *   and ETHTOOL_GLINK commands.
 *
 * Input Parameters:
 *   phydev - Pointer to the PHY device structure
 *   arg    - Pointer to the ethtool command structure
 *
 * Returned Value:
 *   OK on success; -ENOTTY if the command is not handled;
 *   negated errno on error.
 *
 ****************************************************************************/

static int phy_ethtool_ioctl(FAR struct phy_device_s *phydev,
                             unsigned long arg)
{
  uint32_t cmd;

  if (arg == 0)
    {
      return -ENOTTY;
    }

  cmd = *(FAR const uint32_t *)(uintptr_t)arg;

  switch (cmd)
    {
      case ETHTOOL_GLINKSETTINGS:
        return phy_ethtool_get_link_settings(phydev,
                 (FAR struct ethtool_link_settings *)(uintptr_t)arg);

      case ETHTOOL_SLINKSETTINGS:
        return phy_ethtool_set_link_settings(phydev,
                 (FAR const struct ethtool_link_settings *)(uintptr_t)arg);

      case ETHTOOL_NWAY_RST:
        return phy_ethtool_nway_reset(phydev);

      case ETHTOOL_GLINK:
        {
          FAR struct ethtool_value *val =
            (FAR struct ethtool_value *)(uintptr_t)arg;

          nxmutex_lock(&phydev->lock);
          val->data = phydev->link ? 1 : 0;
          nxmutex_unlock(&phydev->lock);
          return OK;
        }

      default:
        return -ENOTTY;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: phy_start
 *
 * Description:
 *   Start the PHY state machine. Configures auto-negotiation and starts
 *   link status monitoring on the low-priority work queue: periodic polling
 *   in polling mode, or interrupt-driven updates in interrupt mode.
 *
 * Input Parameters:
 *   phydev - Pointer to the PHY device structure
 *
 * Returned Value:
 *   OK on success; -EINVAL if phydev is NULL;
 *   -EBUSY if the PHY is not in READY, HALTED or ERROR state.
 *
 ****************************************************************************/

int phy_start(FAR struct phy_device_s *phydev)
{
  int ret;

  if (phydev == NULL)
    {
      return -EINVAL;
    }

  nxmutex_lock(&phydev->lock);

  /* Allow starting from READY (fresh connect), HALTED (after phy_stop) or
   * ERROR (recovery from an unreachable-PHY condition).
   */

  if (phydev->state != PHY_READY && phydev->state != PHY_HALTED &&
      phydev->state != PHY_ERROR)
    {
      nxmutex_unlock(&phydev->lock);
      return -EBUSY;
    }

  /* Power the PHY hardware up before programming it.  phy_stop() powers it
   * down, so this is what brings it back for a restart.  A driver with no
   * power-management support leaves the callback NULL (a no-op).
   */

  if (phydev->drv->config_power)
    {
      ret = phydev->drv->config_power(phydev, true);
      if (ret < 0)
        {
          nerr("ERROR: power on failed: %d\n", ret);
          nxmutex_unlock(&phydev->lock);
          return ret;
        }
    }

  /* Configure auto-negotiation */

  if (phydev->drv->config_aneg)
    {
      ret = phydev->drv->config_aneg(phydev);
      if (ret < 0)
        {
          nerr("ERROR: config_aneg failed: %d\n", ret);
          nxmutex_unlock(&phydev->lock);
          return ret;
        }
    }

  phydev->state = PHY_UP;

  /* Enable interrupt-driven operation when an interrupt source is available
   * and the driver provides interrupt callbacks.  Two source kinds:
   *
   *   irq >= 0          - a real PHY interrupt line: phylib attaches its own
   *                       ISR (phy_request_interrupt) which kicks the state
   *                       machine.
   *   PHY_MAC_INTERRUPT - the MAC driver owns the interrupt and calls
   *                       phy_interrupt() itself; no ISR attached here.
   *
   * Either way we program the PHY's interrupt-enable register via
   * config_intr.  The state machine then runs only on interrupt, with no
   * periodic poll.  A genuine no-interrupt PHY takes the PHY_POLL path and
   * never reaches here.  If interrupt setup fails for a PHY that was
   * supposed to use interrupts, that is a real fault (bad irq, or an
   * unreachable PHY) and is NOT masked by degrading to polling: enter
   * PHY_ERROR and fail the start so the caller sees the problem.
   */

  if (phydev->irq != PHY_POLL && phydev->drv->config_intr)
    {
      ret = phy_request_interrupt(phydev);
      if (ret < 0)
        {
          phydev->state = PHY_ERROR;
          nxmutex_unlock(&phydev->lock);
          return ret;
        }
    }

  ninfo("PHY state machine started (%s mode)\n",
        phydev->irq != PHY_POLL ? "interrupt" : "polling");

  nxmutex_unlock(&phydev->lock);

  /* Run the state machine once, right now, to read the initial link state.
   * A PHY (or MAC) interrupt only fires on a link *change*, so if the cable
   * is already up when phy_start() runs no interrupt would ever arrive and
   * the carrier would never come up without this initial run.
   *
   * phy_work() acquires phydev->lock itself, so it is invoked here after
   * the unlock rather than while the lock is held.  In polling mode
   * phy_work() reschedules itself every PHY_POLL_TICKS; in interrupt mode
   * it runs just this once and afterwards only when an interrupt kicks it.
   */

  phy_work(phydev);
  return OK;
}

/****************************************************************************
 * Name: phy_stop
 *
 * Description:
 *   Stop the PHY state machine. Cancels pending work queue polling and
 *   transitions the PHY to the HALTED state. Notifies the attached
 *   network device that the carrier is off.
 *
 * Input Parameters:
 *   phydev - Pointer to the PHY device structure
 *
 * Returned Value:
 *   OK on success; negated errno on failure.
 *
 ****************************************************************************/

int phy_stop(FAR struct phy_device_s *phydev)
{
  if (phydev == NULL)
    {
      return -EINVAL;
    }

  nxmutex_lock(&phydev->lock);

  if (phydev->irq != PHY_POLL && phydev->drv->config_intr)
    {
      phydev->drv->config_intr(phydev, false);
    }

  phy_free_interrupt(phydev);

  /* Mark the state machine as halted while holding the lock so that any
   * concurrent state-machine work observes the new state and does not
   * reschedule itself.
   */

  phydev->state = PHY_HALTED;

  /* Cancel the state-machine work, but only now that every source that
   * could (re)queue it has been shut off above:
   *
   *   - config_intr(false) + phy_free_interrupt() stop any PHY/MAC
   *     interrupt from calling work_queue(phy_work) again;
   *   - PHY_HALTED stops phy_work() from rescheduling itself.
   *
   * This ordering is why the cancel cannot simply be done first: if it ran
   * before the interrupt was disabled, a PHY interrupt in the gap could
   * re-queue phy_work() after the cancel and defeat it.
   *
   * The lock must be released before the cancel: work_cancel_sync() blocks
   * until an already-running phy_work() finishes, and phy_work() takes
   * phydev->lock as its first action, so cancelling while holding the lock
   * would deadlock (AB-BA).
   *
   * NOTE: phy_stop() must not be called from interrupt or idle context,
   * because work_cancel_sync() may block waiting for a running callback.
   */

  nxmutex_unlock(&phydev->lock);

  work_cancel_sync(LPWORK, &phydev->state_work);

  /* Power the PHY hardware down.  This is done after work_cancel_sync() so
   * that no in-flight phy_work() can still touch the PHY (PHY_HALTED only
   * stops the work from re-queuing, not one already blocked on the lock),
   * but re-acquiring phydev->lock so the power-down MDIO access is still
   * serialised against any other access to this phydev (e.g. a concurrent
   * phy_start() or ioctl path).  This mirrors phy_start(), which powers the
   * PHY up under the lock.  A driver with no power-management support leaves
   * the callback NULL (a no-op).
   */

  nxmutex_lock(&phydev->lock);

  if (phydev->drv->config_power)
    {
      phydev->drv->config_power(phydev, false);
    }

  nxmutex_unlock(&phydev->lock);

  if (phydev->lower)
    {
      netdev_lower_carrier_off(phydev->lower);
    }

  ninfo("PHY state machine stopped\n");
  return OK;
}

/****************************************************************************
 * Name: phy_interrupt
 *
 * Description:
 *   Entry point for a MAC/board driver to call from its own ISR when it
 *   detects that the PHY link has changed (the PHY_MAC_INTERRUPT case where
 *   the PHY has no independent interrupt line owned by phylib).  It simply
 *   triggers the shared state machine.
 *
 * Input Parameters:
 *   phydev - Pointer to the PHY device structure
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void phy_interrupt(FAR struct phy_device_s *phydev)
{
  /* Only kick the state machine when interrupt-driven operation is active
   * (irq != PHY_POLL) and the state machine is still running.  The state
   * check matters for a MAC-owned line (PHY_MAC_INTERRUPT), whose interrupt
   * source phylib cannot mask: after phy_stop() sets PHY_HALTED (or on
   * PHY_ERROR) a late MAC ISR must not re-queue state_work behind the
   * work_cancel_sync() done in phy_stop().  For a real PHY line phy_stop()
   * already detached the ISR, so this is belt-and-suspenders there.
   *
   * Safe from interrupt context: work_queue() just schedules the shared
   * state machine work to run immediately, and coalesces if it is already
   * pending.
   */

  if (phydev->irq != PHY_POLL &&
      phydev->state != PHY_HALTED && phydev->state != PHY_ERROR)
    {
      work_queue(LPWORK, &phydev->state_work, phy_work, phydev, 0);
    }
}

/****************************************************************************
 * Name: phy_ioctl
 *
 * Description:
 *   Single entry point for PHY-related ioctls from the network upper half.
 *   Dispatches ethtool commands (SIOCETHTOOL) and MII register commands
 *   (SIOCGMIIPHY, SIOCGMIIREG, SIOCSMIIREG) to their respective handlers.
 *
 * Input Parameters:
 *   phydev - Pointer to the PHY device structure
 *   cmd    - The ioctl command
 *   arg    - Command-specific argument
 *
 * Returned Value:
 *   OK on success; -ENOTTY if the command is not handled by the PHY layer;
 *   negated errno on error.
 *
 ****************************************************************************/

int phy_ioctl(FAR struct phy_device_s *phydev, int cmd, unsigned long arg)
{
  if (phydev == NULL)
    {
      return -ENOTTY;
    }

  switch (cmd)
    {
      case SIOCETHTOOL:
        return phy_ethtool_ioctl(phydev, arg);

      case SIOCGMIIPHY:
      case SIOCGMIIREG:
      case SIOCSMIIREG:
        return phy_mii_ioctl(phydev, cmd, arg);

      default:
        return -ENOTTY;
    }
}
