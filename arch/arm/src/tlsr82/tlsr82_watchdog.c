/****************************************************************************
 * arch/arm/src/tlsr82/tlsr82_watchdog.c
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

#include "hardware/tlsr82_clock.h"
#include "hardware/tlsr82_timer.h"

#if defined(CONFIG_WATCHDOG) && defined(CONFIG_TLSR82_WATCHDOG)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The timer2 clock in watchdog mode is always system clock, and the watchdog
 * capture bits number is only 14 (high 14 bit valid, mask = 0xfffc0000)
 * so the max timeout value = 0xfffc0000 / WDOG_CLK_MHZ / 1000 (ms)
 *    the resolution value  = 0x0003ffff / WDOG_CLK_MHZ / 1000 (ms)
 */

#define WDOG_CLK_MHZ           CONFIG_TLSR82_CPU_CLK_MHZ
#define WDOG_VALID_SHIFT       18
#define WDOG_VALID_MASK        (0x00003fff << WDOG_VALID_SHIFT)
#define WDOG_RES_MASK          (~WDOG_VALID_MASK)
#define WDOG_MINTIMEOUT        1
#define WDOG_MAXTIMEOUT        (WDOG_VALID_MASK / WDOG_CLK_MHZ / 1000)
#define WDOG_RESOLUTION        (WDOG_RES_MASK / WDOG_CLK_MHZ / 1000)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure provides the private representation of the "lower-half"
 * driver state structure.  This structure must be cast-compatible with the
 * well-known watchdog_lowerhalf_s structure.
 */

struct tlsr82_lowerhalf_s
{
  const struct watchdog_ops_s *ops;  /* Lower half operations */
  xcpt_t   handler;                  /* Current EWI interrupt handler */
  uint32_t timeout;                  /* The actual timeout value */
  uint32_t capture;                  /* Watchdog capture register value */
  bool     started;                  /* The timer has been started */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register operations ******************************************************/

#ifdef CONFIG_DEBUG_WATCHDOG_INFO
static void tlsr82_dumpregs(const char *msg);
#else
#  define tlsr82_dumpregs(msg)
#endif

/* "Lower half" driver methods **********************************************/

static int tlsr82_start(struct watchdog_lowerhalf_s *lower);
static int tlsr82_stop(struct watchdog_lowerhalf_s *lower);
static int tlsr82_keepalive(struct watchdog_lowerhalf_s *lower);
static int tlsr82_getstatus(struct watchdog_lowerhalf_s *lower,
                            struct watchdog_status_s *status);
static int tlsr82_settimeout(struct watchdog_lowerhalf_s *lower,
                             uint32_t timeout);
static xcpt_t tlsr82_capture(struct watchdog_lowerhalf_s *lower,
                             xcpt_t handler);
static int tlsr82_ioctl(struct watchdog_lowerhalf_s *lower, int cmd,
                        unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* "Lower half" driver methods */

static const struct watchdog_ops_s g_wdgops =
{
  .start      = tlsr82_start,
  .stop       = tlsr82_stop,
  .keepalive  = tlsr82_keepalive,
  .getstatus  = tlsr82_getstatus,
  .settimeout = tlsr82_settimeout,
  .capture    = tlsr82_capture,
  .ioctl      = tlsr82_ioctl,
};

/* "Lower half" driver state */

static struct tlsr82_lowerhalf_s g_wdgdev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tlsr82_dumpregs
 *
 * Description:
 *   Start the watchdog timer, resetting the time to the current timeout,
 *
 * Input Parameters:
 *   msg - the message that caller want to print
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_WATCHDOG_INFO
static void tlsr82_dumpregs(const char *msg)
{
  uint8_t status = WDOG_STATUS_REG;
  uint32_t ctrl  = WDOG_CTRL_REG;
  uint32_t tick2 = TIMER_TICK2_REG;

  wdinfo("%s\n", msg);

  /* Watchdog status register */

  wdinfo("  WDOG_STATUS_REG        : 0x%02x\n", status);

  /* Watchdog control register, extract the key information */

  wdinfo("  WDOG_CTRL_REG, CAPTURE : 0x%08lx\n",
         (ctrl & WDOG_CTRL_CAPT_MASK) >> WDOG_CTRL_CAPT_SHIFT);
  wdinfo("  WDOG_CTRL_REG, WDOG EN : 0x%08lx\n",
         (ctrl & WDOG_CTRL_ENABLE_MASK) >> WDOG_CTRL_ENABLE_SHIFT);
  wdinfo("  WDOG_CTRL_REG, TIM2 EN : 0x%08lx\n",
         (ctrl & TIMER_CTRL_T2_ENABLE) >> TIMER_CTRL_T2_ENABLE_SHIFT);

  /* Timer2 tick register */

  wdinfo("  TIMER_TICK2_REG        : 0x%08lx\n", tick2);
}
#endif

/****************************************************************************
 * Name: tlsr82_start
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

static int tlsr82_start(struct watchdog_lowerhalf_s *lower)
{
  struct tlsr82_lowerhalf_s *priv = (struct tlsr82_lowerhalf_s *)lower;

  wdinfo("Entry\n");
  DEBUGASSERT(priv);

  /* Write 1 to TIMER_STATUS_REG to clear the watch dog */

  BM_SET(TIMER_STATUS_REG, TIMER_STATUS_WDOG_CLR);

  /* Enable watchdog capture and timer2 */

  BM_SET(WDOG_CTRL_REG, WDOG_CTRL_ENABLE_MASK);
  BM_SET(TIMER_CTRL_REG, TIMER_CTRL_T2_ENABLE);

  tlsr82_dumpregs("Watchdog start");

  priv->started = true;
  return OK;
}

/****************************************************************************
 * Name: tlsr82_stop
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

static int tlsr82_stop(struct watchdog_lowerhalf_s *lower)
{
  struct tlsr82_lowerhalf_s *priv = (struct tlsr82_lowerhalf_s *)lower;

  wdinfo("Entry\n");

  /* Disable watchdog capture and timer2 */

  BM_CLR(WDOG_CTRL_REG, WDOG_CTRL_ENABLE_MASK);
  BM_CLR(TIMER_CTRL_REG, TIMER_CTRL_T2_ENABLE);

  /* Write 1 to TIMER_STATUS_REG to clear the watch dog */

  BM_SET(TIMER_STATUS_REG, TIMER_STATUS_WDOG_CLR);

  tlsr82_dumpregs("Watchdog stop");

  priv->started = false;

  return OK;
}

/****************************************************************************
 * Name: tlsr82_keepalive
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

static int tlsr82_keepalive(struct watchdog_lowerhalf_s *lower)
{
  struct tlsr82_lowerhalf_s *priv = (struct tlsr82_lowerhalf_s *)lower;

  wdinfo("Entry\n");
  DEBUGASSERT(priv);

  /* Write 1 to TIMER_STATUS_REG to clear the watch dog */

  BM_SET(TIMER_STATUS_REG, TIMER_STATUS_WDOG_CLR);

  tlsr82_dumpregs("Watchdog keepalive");

  return OK;
}

/****************************************************************************
 * Name: tlsr82_getstatus
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

static int tlsr82_getstatus(struct watchdog_lowerhalf_s *lower,
                           struct watchdog_status_s *status)
{
  struct tlsr82_lowerhalf_s *priv = (struct tlsr82_lowerhalf_s *)lower;
  uint32_t lefttick;

  wdinfo("Entry\n");
  DEBUGASSERT(priv);

  /* Return the status bit */

  status->flags = WDFLAGS_RESET;
  if (priv->started)
    {
      status->flags |= WDFLAGS_ACTIVE;
    }

  /* Return the actual timeout is milliseconds */

  status->timeout = priv->timeout;

  /* Get the time remaining until the watchdog expires (in milliseconds) */

  lefttick = priv->capture - TIMER_TICK2_REG;
  status->timeleft = lefttick / (WDOG_CLK_MHZ * 1000);

  tlsr82_dumpregs("Watchdog getstatus");

  wdinfo("Status     :\n");
  wdinfo("  flags    : %08lx\n", status->flags);
  wdinfo("  timeout  : %lu\n", status->timeout);
  wdinfo("  timeleft : %lu\n", status->timeleft);
  return OK;
}

/****************************************************************************
 * Name: tlsr82_settimeout
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

static int tlsr82_settimeout(struct watchdog_lowerhalf_s *lower,
                             uint32_t timeout)
{
  struct tlsr82_lowerhalf_s *priv = (struct tlsr82_lowerhalf_s *)lower;
  uint32_t capture;
  uint32_t tick;

  DEBUGASSERT(priv);
  wdinfo("Entry: timeout=%lu\n", timeout);

  /* Can this timeout be represented? */

  if (timeout < WDOG_MINTIMEOUT || timeout > WDOG_MAXTIMEOUT)
    {
      wderr("ERROR: Cannot represent timeout=%u > %u\n",
            (unsigned)timeout, WDOG_MAXTIMEOUT);
      return -ERANGE;
    }

  /* Calculate the watchdog capture value */

  /* Get the ideal tick value */

  tick = timeout * 1000 * WDOG_CLK_MHZ;

  /* Transfer tick to the watchdog capture value, make sure the real
   * timeout value is larger than the set timeout value
   * Note: loss some accuracy
   */

  capture = (tick + WDOG_RESOLUTION - 1) & WDOG_VALID_MASK;

  /* Calculate and save the actual timeout value in milliseconds:
   *
   * timeout =  1000 * (reload + 1) / Fwwdg
   */

  priv->timeout = capture / (1000 * WDOG_CLK_MHZ);
  priv->capture = capture;

  wdinfo("tick=%lu capture=%lu, actual timeout=%lu\n",
         tick, capture, priv->timeout);

  /* Set capture value to watchdog capture register */

  WDOG_CTRL_REG = (WDOG_CTRL_REG & (~WDOG_CTRL_CAPT_MASK)) |
                  ((capture >> WDOG_VALID_SHIFT) << WDOG_CTRL_CAPT_SHIFT);

  /* Write 1 to TIMER_STATUS_REG to clear the watch dog */

  BM_SET(TIMER_STATUS_REG, TIMER_STATUS_WDOG_CLR);

  tlsr82_dumpregs("Watchdog settimeout");

  return OK;
}

/****************************************************************************
 * Name: tlsr82_capture
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

static xcpt_t tlsr82_capture(struct watchdog_lowerhalf_s *lower,
                             xcpt_t handler)
{
  /* Not support early watchdog expiration interrupt */

  return NULL;
}

/****************************************************************************
 * Name: tlsr82_ioctl
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

static int tlsr82_ioctl(struct watchdog_lowerhalf_s *lower, int cmd,
                        unsigned long arg)
{
  struct tlsr82_lowerhalf_s *priv = (struct tlsr82_lowerhalf_s *)lower;
  int ret = -ENOTTY;

  DEBUGASSERT(priv);
  wdinfo("Entry: cmd=%d arg=%ld\n", cmd, arg);

  /* WDIOC_MINTIME: Set the minimum ping time.  If two keepalive ioctls
   * are received within this time, a reset event will be generated.
   * Argument: A 32-bit time value in milliseconds.
   */

  /* if (cmd == WDIOC_MINTIME) */

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tlsr82_wdginitialize
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
 *   0 on Success and Negative number on fail
 *
 ****************************************************************************/

int tlsr82_wdginitialize(const char *devpath)
{
  struct tlsr82_lowerhalf_s *priv = &g_wdgdev;
  void *driv;

  wdinfo("Entry: devpath=%s\n", devpath);

  if (WDOG_STATUS_REG & WDOG_STATUS_RESET_MASK)
    {
      syslog(LOG_DEBUG, "Reset by Watchdog\n");

      /* Write 1 to clear the watchdog reset bit */

      BM_SET(WDOG_STATUS_REG, WDOG_STATUS_RESET_MASK);
    }
  else
    {
      syslog(LOG_DEBUG, "Reset by Power\n");
    }

  /* Initialize the driver state structure.  Here we assume: (1) the state
   * structure lies in .bss and was zeroed at reset time.  (2) This function
   * is only called once so it is never necessary to re-zero the structure.
   */

  priv->ops     = &g_wdgops;
  priv->started = false;

  /* Select an arbitrary initial timeout value.  But don't start the watchdog
   * yet. NOTE: If the "Hardware watchdog" feature is enabled through the
   * device option bits, the watchdog is automatically enabled at power-on.
   */

  tlsr82_settimeout((struct watchdog_lowerhalf_s *)priv,
                    CONFIG_TLSR82_WDOG_DEFTIMOUT);

  /* Register the watchdog driver as /dev/watchdog0 */

  driv = watchdog_register(devpath, (struct watchdog_lowerhalf_s *)priv);
  if (driv == NULL)
    {
      wderr("Watchdog driver register failed, devpath=%s\n", devpath);
      return -ENOMEM;
    }

  /* When the microcontroller enters debug mode, the watchdog should be
   * halted, but there is no corresponding configuration in telink chip.
   */

  return OK;
}

#endif /* CONFIG_WATCHDOG && CONFIG_TLSR82_WATCHDOG */
