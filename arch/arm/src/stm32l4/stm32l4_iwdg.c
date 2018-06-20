/****************************************************************************
 * arch/arm/src/stm32l4/stm32l4_iwdg.c
 *
 *   Copyright (C) 2012, 2016, 2017 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Juha Niskanen <juha.niskanen@haltian.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>

#include <stdint.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/timers/watchdog.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "stm32l4_rcc.h"
#include "chip/stm32l4_dbgmcu.h"
#include "stm32l4_wdg.h"

#if defined(CONFIG_WATCHDOG) && defined(CONFIG_STM32L4_IWDG)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Clocking *****************************************************************/
/* The minimum frequency of the IWDG clock is:
 *
 *  Fmin = Flsi / 256
 *
 * So the maximum delay (in milliseconds) is then:
 *
 *   1000 * IWDG_RLR_MAX / Fmin
 *
 * For example, if Flsi = 30Khz (the nominal, uncalibrated value), then the
 * maximum delay is:
 *
 *   Fmin = 117.1875
 *   1000 * 4095 / Fmin = 34,944 MSec
 */

#define IWDG_FMIN       (STM32L4_LSI_FREQUENCY / 256)
#define IWDG_MAXTIMEOUT (1000 * IWDG_RLR_MAX / IWDG_FMIN)

/* Configuration ************************************************************/

#ifndef CONFIG_STM32L4_IWDG_DEFTIMOUT
#  define CONFIG_STM32L4_IWDG_DEFTIMOUT IWDG_MAXTIMEOUT
#endif

#ifndef CONFIG_DEBUG_WATCHDOG_INFO
#  undef CONFIG_STM32L4_IWDG_REGDEBUG
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* This structure provides the private representation of the "lower-half"
 * driver state structure.  This structure must be cast-compatible with the
 * well-known watchdog_lowerhalf_s structure.
 */

struct stm32l4_lowerhalf_s
{
  FAR const struct watchdog_ops_s  *ops;  /* Lower half operations */
  uint32_t lsifreq;   /* The calibrated frequency of the LSI oscillator */
  uint32_t timeout;   /* The (actual) selected timeout */
  uint32_t lastreset; /* The last reset time */
  bool     started;   /* true: The watchdog timer has been started */
  uint8_t  prescaler; /* Clock prescaler value */
  uint16_t reload;    /* Timer reload value */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* Register operations ******************************************************/

#ifdef CONFIG_STM32L4_IWDG_REGDEBUG
static uint16_t stm32l4_getreg(uint32_t addr);
static void     stm32l4_putreg(uint16_t val, uint32_t addr);
#else
# define        stm32l4_getreg(addr)     getreg16(addr)
# define        stm32l4_putreg(val,addr) putreg16(val,addr)
#endif

static inline void stm32l4_setprescaler(FAR struct stm32l4_lowerhalf_s *priv);

/* "Lower half" driver methods **********************************************/

static int      stm32l4_start(FAR struct watchdog_lowerhalf_s *lower);
static int      stm32l4_stop(FAR struct watchdog_lowerhalf_s *lower);
static int      stm32l4_keepalive(FAR struct watchdog_lowerhalf_s *lower);
static int      stm32l4_getstatus(FAR struct watchdog_lowerhalf_s *lower,
                  FAR struct watchdog_status_s *status);
static int      stm32l4_settimeout(FAR struct watchdog_lowerhalf_s *lower,
                  uint32_t timeout);

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* "Lower half" driver methods */

static const struct watchdog_ops_s g_wdgops =
{
  .start      = stm32l4_start,
  .stop       = stm32l4_stop,
  .keepalive  = stm32l4_keepalive,
  .getstatus  = stm32l4_getstatus,
  .settimeout = stm32l4_settimeout,
  .capture    = NULL,
  .ioctl      = NULL,
};

/* "Lower half" driver state */

static struct stm32l4_lowerhalf_s g_wdgdev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32l4_getreg
 *
 * Description:
 *   Get the contents of an STM32 IWDG register
 *
 ****************************************************************************/

#ifdef CONFIG_STM32L4_IWDG_REGDEBUG
static uint16_t stm32l4_getreg(uint32_t addr)
{
  static uint32_t prevaddr = 0;
  static uint32_t count = 0;
  static uint16_t preval = 0;

  /* Read the value from the register */

  uint16_t val = getreg16(addr);

  /* Is this the same value that we read from the same register last time?  Are
   * we polling the register?  If so, suppress some of the output.
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

          wdinfo("[repeats %d more times]\n", count-3);
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
 * Name: stm32l4_putreg
 *
 * Description:
 *   Set the contents of an STM32 register to a value
 *
 ****************************************************************************/

#ifdef CONFIG_STM32L4_IWDG_REGDEBUG
static void stm32l4_putreg(uint16_t val, uint32_t addr)
{
  /* Show the register value being written */

  wdinfo("%08x<-%04x\n", addr, val);

  /* Write the value */

  putreg16(val, addr);
}
#endif

/****************************************************************************
 * Name: stm32l4_setprescaler
 *
 * Description:
 *   Set up the prescaler and reload values.
 *
 * Input Parameters:
 *   priv   - A pointer the internal representation of the "lower-half"
 *             driver state structure.
 *
 ****************************************************************************/

static inline void stm32l4_setprescaler(FAR struct stm32l4_lowerhalf_s *priv)
{
  irqstate_t flags;

  flags = enter_critical_section();

  /* Enable write access to IWDG_PR and IWDG_RLR registers */

  stm32l4_putreg(IWDG_KR_KEY_ENABLE, STM32L4_IWDG_KR);

  /* Wait for the PVU and RVU bits to be reset by hardware.  These bits
   * were set the last time that the PR register was written and may not
   * yet be cleared.
   */

  while ((stm32l4_getreg(STM32L4_IWDG_SR) & (IWDG_SR_PVU | IWDG_SR_RVU)) != 0);

  /* Set the prescaler */

  stm32l4_putreg((uint16_t)priv->prescaler << IWDG_PR_SHIFT, STM32L4_IWDG_PR);

  /* Set the reload value */

  stm32l4_putreg((uint16_t)priv->reload, STM32L4_IWDG_RLR);

  /* Reload the counter (and disable write access) */

  stm32l4_putreg(IWDG_KR_KEY_RELOAD, STM32L4_IWDG_KR);

  /* Wait for the PVU and RVU bits to be reset by hardware.  This is
   * to wait for the change to take effect before exiting critical section,
   * as we are not allowed to enter any low-power modes while this update is
   * in progress.
   *
   * REVISIT: PVU and RVU don't get cleared as promised, until the IWDG is
   * started by writing IWDG_KR_KEY_START into IWDG_KR, regardless of whether
   * LSI has been started explicitly previously, or not. RM does not document
   * this behavior. Lets hope no low-power mode entry happens in this case
   * during the next up to five RC 40 kHz cycles.
   */

  if (priv->started)
    {
      while ((stm32l4_getreg(STM32L4_IWDG_SR) & (IWDG_SR_PVU | IWDG_SR_RVU)) != 0);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: stm32l4_start
 *
 * Description:
 *   Start the watchdog timer, resetting the time to the current timeout,
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int stm32l4_start(FAR struct watchdog_lowerhalf_s *lower)
{
  FAR struct stm32l4_lowerhalf_s *priv = (FAR struct stm32l4_lowerhalf_s *)lower;
  irqstate_t flags;

  wdinfo("Entry: started=%d\n");
  DEBUGASSERT(priv);

  /* Have we already been started? */

  if (!priv->started)
    {
      /* Set up prescaler and reload value for the selected timeout before
       * starting the watchdog timer.
       */

      stm32l4_setprescaler(priv);

      /* Enable IWDG (the LSI oscillator will be enabled by hardware).  NOTE:
       * If the "Hardware watchdog" feature is enabled through the device option
       * bits, the watchdog is automatically enabled at power-on.
       */

      flags           = enter_critical_section();
      stm32l4_putreg(IWDG_KR_KEY_START, STM32L4_IWDG_KR);
      priv->lastreset = clock_systimer();
      priv->started   = true;
      leave_critical_section(flags);
    }

  return OK;
}

/****************************************************************************
 * Name: stm32l4_stop
 *
 * Description:
 *   Stop the watchdog timer
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int stm32l4_stop(FAR struct watchdog_lowerhalf_s *lower)
{
  /* There is no way to disable the IDWG timer once it has been started */

  wdinfo("Entry\n");
  return -ENOSYS;
}

/****************************************************************************
 * Name: stm32l4_keepalive
 *
 * Description:
 *   Reset the watchdog timer to the current timeout value, prevent any
 *   imminent watchdog timeouts.  This is sometimes referred as "pinging"
 *   the watchdog timer or "petting the dog".
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int stm32l4_keepalive(FAR struct watchdog_lowerhalf_s *lower)
{
  FAR struct stm32l4_lowerhalf_s *priv = (FAR struct stm32l4_lowerhalf_s *)lower;
  irqstate_t flags;

  wdinfo("Entry\n");

  /* Reload the IWDG timer */

  flags = enter_critical_section();
  stm32l4_putreg(IWDG_KR_KEY_RELOAD, STM32L4_IWDG_KR);
  priv->lastreset = clock_systimer();
  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: stm32l4_getstatus
 *
 * Description:
 *   Get the current watchdog timer status
 *
 * Input Parameters:
 *   lower  - A pointer the publicly visible representation of the "lower-half"
 *            driver state structure.
 *   status - The location to return the watchdog status information.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int stm32l4_getstatus(FAR struct watchdog_lowerhalf_s *lower,
                             FAR struct watchdog_status_s *status)
{
  FAR struct stm32l4_lowerhalf_s *priv = (FAR struct stm32l4_lowerhalf_s *)lower;
  uint32_t ticks;
  uint32_t elapsed;

  wdinfo("Entry\n");
  DEBUGASSERT(priv);

  /* Return the status bit */

  status->flags = WDFLAGS_RESET;
  if (priv->started)
    {
      status->flags |= WDFLAGS_ACTIVE;
    }

  /* Return the actual timeout in milliseconds */

  status->timeout = priv->timeout;

  /* Get the elapsed time since the last ping */

  ticks   = clock_systimer() - priv->lastreset;
  elapsed = (int32_t)TICK2MSEC(ticks);

  if (elapsed > priv->timeout)
    {
      elapsed = priv->timeout;
    }

  /* Return the approximate time until the watchdog timer expiration */

  status->timeleft = priv->timeout - elapsed;

  wdinfo("Status     :\n");
  wdinfo("  flags    : %08x\n", status->flags);
  wdinfo("  timeout  : %d\n", status->timeout);
  wdinfo("  timeleft : %d\n", status->timeleft);
  return OK;
}

/****************************************************************************
 * Name: stm32l4_settimeout
 *
 * Description:
 *   Set a new timeout value (and reset the watchdog timer)
 *
 * Input Parameters:
 *   lower   - A pointer the publicly visible representation of the "lower-half"
 *             driver state structure.
 *   timeout - The new timeout value in milliseconds.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int stm32l4_settimeout(FAR struct watchdog_lowerhalf_s *lower,
                              uint32_t timeout)
{
  FAR struct stm32l4_lowerhalf_s *priv = (FAR struct stm32l4_lowerhalf_s *)lower;
  uint32_t fiwdg;
  uint64_t reload;
  int prescaler;
  int shift;

  wdinfo("Entry: timeout=%d\n", timeout);
  DEBUGASSERT(priv);

  /* Can this timeout be represented? */

  if (timeout < 1 || timeout > IWDG_MAXTIMEOUT)
    {
      wderr("ERROR: Cannot represent timeout=%d > %d\n",
            timeout, IWDG_MAXTIMEOUT);
      return -ERANGE;
    }

  /* Select the smallest prescaler that will result in a reload value that is
   * less than the maximum.
   */

  for (prescaler = 0; ; prescaler++)
    {
      /* PR = 0 -> Divider = 4   = 1 << 2
       * PR = 1 -> Divider = 8   = 1 << 3
       * PR = 2 -> Divider = 16  = 1 << 4
       * PR = 3 -> Divider = 32  = 1 << 5
       * PR = 4 -> Divider = 64  = 1 << 6
       * PR = 5 -> Divider = 128 = 1 << 7
       * PR = 6 -> Divider = 256 = 1 << 8
       * PR = n -> Divider       = 1 << (n+2)
       */

      shift = prescaler + 2;

      /* Get the IWDG counter frequency in Hz. For a nominal 32Khz LSI clock,
       * this is value in the range of 7500 and 125.
       */

      fiwdg = priv->lsifreq >> shift;

      /* We want:
       *  1000 * reload / Fiwdg = timeout
       * Or:
       *  reload = Fiwdg * timeout / 1000
       */

      reload = (uint64_t)fiwdg * (uint64_t)timeout / 1000;

      /* If this reload valid is less than the maximum or we are not ready
       * at the prescaler value, then break out of the loop to use these
       * settings.
       */

      if (reload <= IWDG_RLR_MAX || prescaler == 6)
        {
          /* Note that we explicitly break out of the loop rather than using
           * the 'for' loop termination logic because we do not want the
           * value of prescaler to be incremented.
           */

          break;
        }
    }

  /* Make sure that the final reload value is within range */

  if (reload > IWDG_RLR_MAX)
    {
      reload = IWDG_RLR_MAX;
    }

  /* Get the actual timeout value in milliseconds.
   *
   * We have:
   *  reload = Fiwdg * timeout / 1000
   * So we want:
   *  timeout = 1000 * reload / Fiwdg
   */

  priv->timeout = (1000 * (uint32_t)reload) / fiwdg;

  /* Save setup values for later use */

  priv->prescaler = prescaler;
  priv->reload    = reload;

  /* Write the prescaler and reload values to the IWDG registers. */

  if (priv->started)
    {
      stm32l4_setprescaler(priv);
    }

  wdinfo("prescaler=%d fiwdg=%d reload=%d\n", prescaler, fiwdg, reload);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32l4_iwdginitialize
 *
 * Description:
 *   Initialize the IWDG watchdog timer.  The watchdog timer is initialized and
 *   registers as 'devpath'.  The initial state of the watchdog timer is
 *   disabled.
 *
 * Input Parameters:
 *   devpath - The full path to the watchdog.  This should be of the form
 *     /dev/watchdog0
 *   lsifreq - The calibrated LSI clock frequency
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stm32l4_iwdginitialize(FAR const char *devpath, uint32_t lsifreq)
{
  FAR struct stm32l4_lowerhalf_s *priv = &g_wdgdev;
  uint32_t cr;

  wdinfo("Entry: devpath=%s lsifreq=%d\n", devpath, lsifreq);

  /* NOTE we assume that clocking to the IWDG has already been provided by
   * the RCC initialization logic.
   */

  /* Initialize the driver state structure. */

  priv->ops     = &g_wdgops;
  priv->lsifreq = lsifreq;
  priv->started = false;

  /* Make sure that the LSI oscillator is enabled.  NOTE:  The LSI oscillator
   * is enabled here but is not disabled by this file, because this file does
   * not know the global usage of the oscillator.  Any clock management
   * logic (say, as part of a power management scheme) needs handle other
   * LSI controls outside of this file.
   */

  stm32l4_rcc_enablelsi();
  wdinfo("RCC CSR: %08x\n", getreg32(STM32L4_RCC_CSR));

  /* Select an arbitrary initial timeout value.  But don't start the watchdog
   * yet. NOTE: If the "Hardware watchdog" feature is enabled through the
   * device option bits, the watchdog is automatically enabled at power-on.
   */

  stm32l4_settimeout((FAR struct watchdog_lowerhalf_s *)priv, CONFIG_STM32L4_IWDG_DEFTIMOUT);

  /* Register the watchdog driver as /dev/watchdog0 */

  (void)watchdog_register(devpath, (FAR struct watchdog_lowerhalf_s *)priv);

  /* When the microcontroller enters debug mode (Cortex-M4F core halted),
   * the IWDG counter either continues to work normally or stops, depending
   * on DBG_IWDG_STOP configuration bit in DBG module.
   */

  cr = getreg32(STM32_DBGMCU_APB1_FZ);
  cr |= DBGMCU_APB1_IWDGSTOP;
  putreg32(cr, STM32_DBGMCU_APB1_FZ);
}

#endif /* CONFIG_WATCHDOG && CONFIG_STM32L4_IWDG */
