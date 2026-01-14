/****************************************************************************
 * arch/arm/src/at32/at32_wwdg.c
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
#include <nuttx/arch.h>

#include <stdint.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/timers/watchdog.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/at32_dbgmcu.h"
#include "at32_wdg.h"

#if defined(CONFIG_WATCHDOG) && defined(CONFIG_AT32_WWDG)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The minimum frequency of the WWDG clock is:
 *
 *  Fmin = PCLK1 / 4096 / 8
 *
 * So the maximum delay (in milliseconds) is then:
 *
 *   1000 * (WWDG_CR_T_MAX+1) / Fmin
 *
 * For example, if PCLK1 = 42MHz, then the maximum delay is:
 *
 *   Fmin = 1281.74
 *   1000 * 64 / Fmin = 49.93 msec
 */

#define WWDG_FMIN       (AT32_PCLK1_FREQUENCY / 4096 / 8)
#define WWDG_MAXTIMEOUT (1000 * (WWDG_CR_T_MAX+1) / WWDG_FMIN)

/* Configuration ************************************************************/

#ifndef CONFIG_AT32_WWDG_DEFTIMOUT
#  define CONFIG_AT32_WWDG_DEFTIMOUT WWDG_MAXTIMEOUT
#endif

#ifndef CONFIG_DEBUG_WATCHDOG_INFO
#  undef CONFIG_AT32_WWDG_REGDEBUG
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure provides the private representation of the "lower-half"
 * driver state structure.  This structure must be cast-compatible with the
 * well-known watchdog_lowerhalf_s structure.
 */

struct at32_lowerhalf_s
{
  const struct watchdog_ops_s *ops; /* Lower half operations */
  xcpt_t   handler;                 /* Current EWI interrupt handler */
  uint32_t timeout;                 /* The actual timeout value */
  uint32_t fwwdg;                   /* WWDG clock frequency */
  bool     started;                 /* The timer has been started */
  uint8_t  reload;                  /* The 7-bit reload field reset value */
  uint8_t  window;                  /* The 7-bit window (W) field value */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register operations ******************************************************/

#ifdef CONFIG_AT32_WWDG_REGDEBUG
static uint16_t at32_getreg(uint32_t addr);
static void     at32_putreg(uint16_t val, uint32_t addr);
#else
# define        at32_getreg(addr)     getreg32(addr)
# define        at32_putreg(val,addr) putreg32(val,addr)
#endif
static void     at32_setwindow(struct at32_lowerhalf_s *priv,
                  uint8_t window);

/* Interrupt handling *******************************************************/

static int      at32_interrupt(int irq, void *context, void *arg);

/* "Lower half" driver methods **********************************************/

static int      at32_start(struct watchdog_lowerhalf_s *lower);
static int      at32_stop(struct watchdog_lowerhalf_s *lower);
static int      at32_keepalive(struct watchdog_lowerhalf_s *lower);
static int      at32_getstatus(struct watchdog_lowerhalf_s *lower,
                  struct watchdog_status_s *status);
static int      at32_settimeout(struct watchdog_lowerhalf_s *lower,
                  uint32_t timeout);
static xcpt_t   at32_capture(struct watchdog_lowerhalf_s *lower,
                  xcpt_t handler);
static int      at32_ioctl(struct watchdog_lowerhalf_s *lower, int cmd,
                  unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* "Lower half" driver methods */

static const struct watchdog_ops_s g_wdgops =
{
  .start      = at32_start,
  .stop       = at32_stop,
  .keepalive  = at32_keepalive,
  .getstatus  = at32_getstatus,
  .settimeout = at32_settimeout,
  .capture    = at32_capture,
  .ioctl      = at32_ioctl,
};

/* "Lower half" driver state */

static struct at32_lowerhalf_s g_wdgdev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: at32_getreg
 *
 * Description:
 *   Get the contents of an AT32 register
 *
 ****************************************************************************/

#ifdef CONFIG_AT32_WWDG_REGDEBUG
static uint16_t at32_getreg(uint32_t addr)
{
  static uint32_t prevaddr = 0;
  static uint32_t count = 0;
  static uint16_t preval = 0;

  /* Read the value from the register */

  uint16_t val = getreg16(addr);

  /* Is this the same value that we read from the same register last time?
   * Are we polling the register?  If so, suppress some of the output.
   */

  if (addr == prevaddr && val == preval)
    {
      if (count == 0xffffffff || ++count > 3)
        {
          if (count == 4)
            {
              wdinfo("...\n");
            }

          return val;
        }
    }

  /* No this is a new address or value */

  else
    {
      /* Did we print "..." for the previous value? */

      if (count > 3)
        {
          /* Yes.. then show how many times the value repeated */

          wdinfo("[repeats %d more times]\n", count - 3);
        }

      /* Save the new address, value, and count */

      prevaddr = addr;
      preval   = val;
      count    = 1;
    }

  /* Show the register value read */

  wdinfo("%08x->%04x\n", addr, val);
  return val;
}
#endif

/****************************************************************************
 * Name: at32_putreg
 *
 * Description:
 *   Set the contents of an AT32 register to a value
 *
 ****************************************************************************/

#ifdef CONFIG_AT32_WWDG_REGDEBUG
static void at32_putreg(uint16_t val, uint32_t addr)
{
  /* Show the register value being written */

  wdinfo("%08x<-%04x\n", addr, val);

  /* Write the value */

  putreg16(val, addr);
}
#endif

/****************************************************************************
 * Name: at32_setwindow
 *
 * Description:
 *   Set the CFR window value. The window value is compared to the down-
 *   counter when the counter is updated.  The WWDG counter should be updated
 *   only when the counter is below this window value (and greater than 64)
 *   otherwise a reset will be generated
 *
 ****************************************************************************/

static void at32_setwindow(struct at32_lowerhalf_s *priv,
                            uint8_t window)
{
  uint16_t regval;

  /* Set W[6:0] bits according to selected window value */

  regval = at32_getreg(AT32_WWDG_CFR);
  regval &= ~WWDG_CFR_W_MASK;
  regval |= window << WWDG_CFR_W_SHIFT;
  at32_putreg(regval, AT32_WWDG_CFR);

  /* Remember the window setting */

  priv->window = window;
}

/****************************************************************************
 * Name: at32_interrupt
 *
 * Description:
 *   WWDG early warning interrupt
 *
 * Input Parameters:
 *   Usual interrupt handler arguments.
 *
 * Returned Value:
 *   Always returns OK.
 *
 ****************************************************************************/

static int at32_interrupt(int irq, void *context, void *arg)
{
  struct at32_lowerhalf_s *priv = &g_wdgdev;
  uint16_t regval;

  /* Check if the EWI interrupt is really pending */

  regval = at32_getreg(AT32_WWDG_SR);
  if ((regval & WWDG_SR_EWIF) != 0)
    {
      /* Is there a registered handler? */

      if (priv->handler)
        {
          /* Yes... NOTE:  This interrupt service routine (ISR) must reload
           * the WWDG counter to prevent the reset.  Otherwise, we will reset
           * upon return.
           */

          priv->handler(irq, context, arg);
        }

      /* The EWI interrupt is cleared by writing '0' to the EWIF bit in the
       * WWDG_SR register.
       */

      regval &= ~WWDG_SR_EWIF;
      at32_putreg(regval, AT32_WWDG_SR);
    }

  return OK;
}

/****************************************************************************
 * Name: at32_start
 *
 * Description:
 *   Start the watchdog timer, resetting the time to the current timeout,
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-
 *           half" driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int at32_start(struct watchdog_lowerhalf_s *lower)
{
  struct at32_lowerhalf_s *priv = (struct at32_lowerhalf_s *)lower;

  wdinfo("Entry\n");
  DEBUGASSERT(priv);

  /* The watchdog is always disabled after a reset. It is enabled by setting
   * the WDGA bit in the WWDG_CR register, then it cannot be disabled again
   * except by a reset.
   */

  at32_putreg(WWDG_CR_WDGA | WWDG_CR_T_RESET | priv->reload, AT32_WWDG_CR);
  priv->started = true;
  return OK;
}

/****************************************************************************
 * Name: at32_stop
 *
 * Description:
 *   Stop the watchdog timer
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-
 *           half" driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int at32_stop(struct watchdog_lowerhalf_s *lower)
{
  /* The watchdog is always disabled after a reset. It is enabled by setting
   * the WDGA bit in the WWDG_CR register, then it cannot be disabled again
   * except by a reset.
   */

  wdinfo("Entry\n");
  return -ENOSYS;
}

/****************************************************************************
 * Name: at32_keepalive
 *
 * Description:
 *   Reset the watchdog timer to the current timeout value, prevent any
 *   imminent watchdog timeouts.  This is sometimes referred as "pinging"
 *   the watchdog timer or "petting the dog".
 *
 *   The application program must write in the WWDG_CR register at regular
 *   intervals during normal operation to prevent an MCU reset. This
 *   operation must occur only when the counter value is lower than the
 *   window register value. The value to be stored in the WWDG_CR register
 *   must be between 0xff and 0xC0:
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-
 *           half" driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int at32_keepalive(struct watchdog_lowerhalf_s *lower)
{
  struct at32_lowerhalf_s *priv = (struct at32_lowerhalf_s *)lower;

  wdinfo("Entry\n");
  DEBUGASSERT(priv);

  /* Write to T[6:0] bits to configure the counter value, no need to do
   * a read-modify-write; writing a 0 to WDGA bit does nothing.
   */

  at32_putreg((WWDG_CR_T_RESET | priv->reload), AT32_WWDG_CR);
  return OK;
}

/****************************************************************************
 * Name: at32_getstatus
 *
 * Description:
 *   Get the current watchdog timer status
 *
 * Input Parameters:
 *   lower  - A pointer the publicly visible representation of the "lower-
 *            half" driver state structure.
 *   status - The location to return the watchdog status information.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int at32_getstatus(struct watchdog_lowerhalf_s *lower,
                           struct watchdog_status_s *status)
{
  struct at32_lowerhalf_s *priv = (struct at32_lowerhalf_s *)lower;
  uint32_t elapsed;
  uint16_t reload;

  wdinfo("Entry\n");
  DEBUGASSERT(priv);

  /* Return the status bit */

  status->flags = WDFLAGS_RESET;
  if (priv->started)
    {
      status->flags |= WDFLAGS_ACTIVE;
    }

  if (priv->handler)
    {
      status->flags |= WDFLAGS_CAPTURE;
    }

  /* Return the actual timeout is milliseconds */

  status->timeout = priv->timeout;

  /* Get the time remaining until the watchdog expires (in milliseconds) */

  reload = (at32_getreg(AT32_WWDG_CR) >> WWDG_CR_T_SHIFT) & 0x7f;
  elapsed = priv->reload - reload;
  status->timeleft = (priv->timeout * elapsed) / (priv->reload + 1);

  wdinfo("Status     :\n");
  wdinfo("  flags    : %08x\n", (unsigned)status->flags);
  wdinfo("  timeout  : %u\n", (unsigned)status->timeout);
  wdinfo("  timeleft : %u\n", (unsigned)status->flags);
  return OK;
}

/****************************************************************************
 * Name: at32_settimeout
 *
 * Description:
 *   Set a new timeout value (and reset the watchdog timer)
 *
 * Input Parameters:
 *   lower   - A pointer the publicly visible representation of the
 *             "lower-half" driver state structure.
 *   timeout - The new timeout value in milliseconds.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int at32_settimeout(struct watchdog_lowerhalf_s *lower,
                            uint32_t timeout)
{
  struct at32_lowerhalf_s *priv = (struct at32_lowerhalf_s *)lower;
  uint32_t fwwdg;
  uint32_t reload;
  uint16_t regval;
  int wdgtb;

  DEBUGASSERT(priv);
  wdinfo("Entry: timeout=%u\n", (unsigned)timeout);

  /* Can this timeout be represented? */

  if (timeout < 1 || timeout > WWDG_MAXTIMEOUT)
    {
      wderr("ERROR: Cannot represent timeout=%u > %lu\n",
            (unsigned)timeout, WWDG_MAXTIMEOUT);
      return -ERANGE;
    }

  /* Determine prescaler value.
   *
   * Fwwdg = PCLK1/4096/prescaler.
   *
   * Where
   *  Fwwwdg is the frequency of the WWDG clock
   *  wdgtb is one of {1, 2, 4, or 8}
   */

  /* Select the smallest prescaler that will result in a reload field value
   * that is less than the maximum.
   */

  for (wdgtb = 0; ; wdgtb++)
    {
      /* WDGTB = 0 -> Divider = 1  = 1 << 0
       * WDGTB = 1 -> Divider = 2  = 1 << 1
       * WDGTB = 2 -> Divider = 4  = 1 << 2
       * WDGTB = 3 -> Divider = 8  = 1 << 3
       */

      /* Get the WWDG counter frequency in Hz. */

      fwwdg = (AT32_PCLK1_FREQUENCY / 4096) >> wdgtb;

      /* The formula to calculate the timeout value is given by:
       *
       * timeout =  1000 * (reload + 1) / Fwwdg, OR
       * reload = timeout * Fwwdg / 1000 - 1
       *
       * Where
       *  timeout is the desired timeout in milliseconds
       *  reload is the contents of T{5:0]
       *  Fwwdg is the frequency of the WWDG clock
       */

       reload = timeout * fwwdg / 1000 - 1;

      /* If this reload valid is less than the maximum or we are not ready
       * at the prescaler value, then break out of the loop to use these
       * settings.
       */

#if 0
      wdinfo("wdgtb=%d fwwdg=%d reload=%d timeout=%d\n",
             wdgtb, fwwdg, reload,  1000 * (reload + 1) / fwwdg);
#endif
      if (reload <= WWDG_CR_T_MAX || wdgtb == 3)
        {
          /* Note that we explicitly break out of the loop rather than using
           * the 'for' loop termination logic because we do not want the
           * value of wdgtb to be incremented.
           */

          break;
        }
    }

  /* Make sure that the final reload value is within range */

  if (reload > WWDG_CR_T_MAX)
    {
      reload = WWDG_CR_T_MAX;
    }

  /* Calculate and save the actual timeout value in milliseconds:
   *
   * timeout =  1000 * (reload + 1) / Fwwdg
   */

  priv->timeout = 1000 * (reload + 1) / fwwdg;

  /* Remember the selected values */

  priv->fwwdg  = fwwdg;
  priv->reload = reload;

  wdinfo("wdgtb=%d fwwdg=%u reload=%u timeout=%u\n",
         wdgtb, (unsigned)fwwdg, (unsigned)reload, (unsigned)priv->timeout);

  /* Set WDGTB[1:0] bits according to calculated value */

  regval = at32_getreg(AT32_WWDG_CFR);
  regval &= ~WWDG_CFR_WDGTB_MASK;
  regval |= (uint16_t)wdgtb << WWDG_CFR_WDGTB_SHIFT;
  at32_putreg(regval, AT32_WWDG_CFR);

  /* Reset the 7-bit window value to the maximum value.. essentially
   * disabling the lower limit of the watchdog reset time.
   */

  at32_setwindow(priv, 0x7f);
  return OK;
}

/****************************************************************************
 * Name: at32_capture
 *
 * Description:
 *   Don't reset on watchdog timer timeout; instead, call this user provider
 *   timeout handler.  NOTE:  Providing handler==NULL will restore the reset
 *   behavior.
 *
 * Input Parameters:
 *   lower      - A pointer the publicly visible representation of the
 *                "lower-half" driver state structure.
 *   newhandler - The new watchdog expiration function pointer.  If this
 *                function pointer is NULL, then the reset-on-expiration
 *                behavior is restored,
 *
 * Returned Value:
 *   The previous watchdog expiration function pointer or NULL is there was
 *   no previous function pointer, i.e., if the previous behavior was
 *   reset-on-expiration (NULL is also returned if an error occurs).
 *
 ****************************************************************************/

static xcpt_t at32_capture(struct watchdog_lowerhalf_s *lower,
                            xcpt_t handler)
{
  struct at32_lowerhalf_s *priv = (struct at32_lowerhalf_s *)lower;
  irqstate_t flags;
  xcpt_t oldhandler;
  uint16_t regval;

  DEBUGASSERT(priv);
  wdinfo("Entry: handler=%p\n", handler);

  /* Get the old handler return value */

  flags = enter_critical_section();
  oldhandler = priv->handler;

  /* Save the new handler */

  priv->handler = handler;

  /* Are we attaching or detaching the handler? */

  regval = at32_getreg(AT32_WWDG_CFR);
  if (handler)
    {
      /* Attaching... Enable the EWI interrupt */

      regval |= WWDG_CFR_EWI;
      at32_putreg(regval, AT32_WWDG_CFR);

      up_enable_irq(AT32_IRQ_WWDG);
    }
  else
    {
      /* Detaching... Disable the EWI interrupt */

      regval &= ~WWDG_CFR_EWI;
      at32_putreg(regval, AT32_WWDG_CFR);

      up_disable_irq(AT32_IRQ_WWDG);
    }

  leave_critical_section(flags);
  return oldhandler;
}

/****************************************************************************
 * Name: at32_ioctl
 *
 * Description:
 *   Any ioctl commands that are not recognized by the "upper-half" driver
 *   are forwarded to the lower half driver through this method.
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-
 *           half" driver state structure.
 *   cmd   - The ioctl command value
 *   arg   - The optional argument that accompanies the 'cmd'.  The
 *           interpretation of this argument depends on the particular
 *           command.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int at32_ioctl(struct watchdog_lowerhalf_s *lower, int cmd,
                    unsigned long arg)
{
  struct at32_lowerhalf_s *priv = (struct at32_lowerhalf_s *)lower;
  int ret = -ENOTTY;

  DEBUGASSERT(priv);
  wdinfo("Entry: cmd=%d arg=%ld\n", cmd, arg);

  /* WDIOC_MINTIME: Set the minimum ping time.  If two keepalive ioctls
   * are received within this time, a reset event will be generated.
   * Argument: A 32-bit time value in milliseconds.
   */

  if (cmd == WDIOC_MINTIME)
    {
      uint32_t mintime = (uint32_t)arg;

      /* The minimum time should be strictly less than the total delay
       * which, in turn, will be less than or equal to WWDG_CR_T_MAX
       */

      ret = -EINVAL;
      if (mintime < priv->timeout)
        {
          uint32_t window = (priv->timeout - mintime) * priv->fwwdg /
                            1000 - 1;
          DEBUGASSERT(window < priv->reload);
          at32_setwindow(priv, window | WWDG_CR_T_RESET);
          ret = OK;
        }
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: at32_wwdginitialize
 *
 * Description:
 *   Initialize the WWDG watchdog timer.  The watchdog timer is initialized
 *   and registers as 'devpath'.  The initial state of the watchdog timer is
 *   disabled.
 *
 * Input Parameters:
 *   devpath - The full path to the watchdog.  This should be of the form
 *     /dev/watchdog0
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void at32_wwdginitialize(const char *devpath)
{
  struct at32_lowerhalf_s *priv = &g_wdgdev;

  wdinfo("Entry: devpath=%s\n", devpath);

  /* NOTE we assume that clocking to the WWDG has already been provided by
   * the RCC initialization logic.
   */

  /* Initialize the driver state structure.  Here we assume: (1) the state
   * structure lies in .bss and was zeroed at reset time.  (2) This function
   * is only called once so it is never necessary to re-zero the structure.
   */

  priv->ops = &g_wdgops;

  /* Attach our EWI interrupt handler (But don't enable it yet) */

  irq_attach(AT32_IRQ_WWDG, at32_interrupt, NULL);

  /* Select an arbitrary initial timeout value.  But don't start the watchdog
   * yet. NOTE: If the "Hardware watchdog" feature is enabled through the
   * device option bits, the watchdog is automatically enabled at power-on.
   */

  at32_settimeout((struct watchdog_lowerhalf_s *)priv,
                   CONFIG_AT32_WWDG_DEFTIMOUT);

  /* Register the watchdog driver as /dev/watchdog0 */

  watchdog_register(devpath, (struct watchdog_lowerhalf_s *)priv);

  /* When the microcontroller enters debug mode (Cortex-M core halted),
   * the WWDG counter either continues to work normally or stops, depending
   * on DBG_WWDG_STOP configuration bit in DBG module.
   */

#if defined(CONFIG_AT32_JTAG_FULL_ENABLE) || \
    defined(CONFIG_AT32_JTAG_NOJNTRST_ENABLE) || \
    defined(CONFIG_AT32_JTAG_SW_ENABLE)
    {
#if defined(CONFIG_AT32_AT32F43XX)
      uint32_t cr = getreg32(AT32_DEBUG_APB1_PAUSE);
      cr |= DEBUG_APB1_APUSE_WWDT_PAUSE;
      putreg32(cr, AT32_DEBUG_APB1_PAUSE);
#endif
    }
#endif
}

#endif /* CONFIG_WATCHDOG && CONFIG_AT32_WWDG */
