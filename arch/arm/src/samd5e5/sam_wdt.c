/****************************************************************************
 * arch/arm/src/samd5e5/sam_wdt.c
 *
 *   Copyright (C) 2020 Falker Automacao Agricola LTDA.
 *   Author: Leomar Mateus Radke <leomar@falker.com.br>
 *   Author: Ricardo Wartchow <wartchow@gmail.com>
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

#include <sys/types.h>

#include <stdint.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/timers/watchdog.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "arm_arch.h"
#include "sam_periphclks.h"
#include "sam_wdt.h"

#if defined(CONFIG_WATCHDOG) && defined(CONFIG_SAMD5E5_WDT)

#ifndef BOARD_SCLK_FREQUENCY
#  define BOARD_SCLK_FREQUENCY 32768
#endif

#define WDT_FCLK        (BOARD_SCLK_FREQUENCY / 128)
#define WDT_MAXTIMEOUT  ((1000 * (WDT_MR_WDV_MAX+1)) / WDT_FCLK)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Clocking *****************************************************************/

/** 8 clock cycles */
#define WDT_CLK_8CYCLE 8
/** 16 clock cycles */
#define WDT_CLK_16CYCLE 16
/** 32 clock cycles */
#define WDT_CLK_32CYCLE 32
/** 64 clock cycles */
#define WDT_CLK_64CYCLE 64
/** 128 clock cycles */
#define WDT_CLK_128CYCLE 128
/** 256 clock cycles */
#define WDT_CLK_256CYCLE 256
/** 512 clock cycles */
#define WDT_CLK_512CYCLE 512
/** 1024 clock cycles */
#define WDT_CLK_1024CYCLE 1024
/** 20488 clock cycles */
#define WDT_CLK_2048CYCLE 2048
/** 4096 clock cycles */
#define WDT_CLK_4096CYCLE 4096
/** 8192 clock cycles */
#define WDT_CLK_8192CYCLE 8192
/** 16384 clock cycles */
#define WDT_CLK_16384CYCLE 16384

/**
 * \brief Macro is used to indicate the rate of second/millisecond
 */
#define WDT_PERIOD_RATE 1000

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* This structure provides the private representation of the "lower-half"
 * driver state structure.  This structure must be cast-compatible with the
 * well-known watchdog_lowerhalf_s structure.
 */

struct sam_lowerhalf_s
{
  FAR const struct watchdog_ops_s  *ops;  /* Lower half operations */
  uint32_t timeout;   /* The (actual) selected timeout */
  uint32_t lastreset; /* The last reset time */
  bool     started;   /* true: The watchdog timer has been started */
  uint8_t  prescaler; /* Clock prescaler value */
  uint16_t reload;    /* Timer reload value */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
void sam_sync_wdt(int value);

/* Register operations ******************************************************/


/*static int      sam_interrupt(int irq, FAR void *context, FAR void *arg);*/

/* "Lower half" driver methods **********************************************/

static int      sam_start(FAR struct watchdog_lowerhalf_s *lower);
static int      sam_stop(FAR struct watchdog_lowerhalf_s *lower);
static int      sam_keepalive(FAR struct watchdog_lowerhalf_s *lower);
static int      sam_getstatus(FAR struct watchdog_lowerhalf_s *lower,
                  FAR struct watchdog_status_s *status);
static int      sam_settimeout(FAR struct watchdog_lowerhalf_s *lower,
                  uint32_t timeout);
/*static xcpt_t   sam_capture(FAR struct watchdog_lowerhalf_s *lower,
                  xcpt_t handler);
static int      sam_ioctl(FAR struct watchdog_lowerhalf_s *lower, int cmd,
                  unsigned long arg);*/

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* "Lower half" driver methods */

static const struct watchdog_ops_s g_wdgops =
{
  .start      = sam_start,
  .stop       = sam_stop,
  .keepalive  = sam_keepalive,
  .getstatus  = sam_getstatus,
  .settimeout = sam_settimeout,
  .capture    = NULL,
  .ioctl      = NULL,
};

/* "Lower half" driver state */

static struct sam_lowerhalf_s g_wdgdev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/
void sam_sync_wdt(int value)
{
  while ((getreg32(SAM_WDT_SYNCBUSY) & value) != 0);
}

void sam_wdt_dumpregs(void)
{
  wdinfo("WDT  Regs:\n");
  wdinfo("     INTENCLR:  %02x\n", getreg8(SAM_WDT_INTENCLR));
  wdinfo("     INTENSET:  %02x\n", getreg8(SAM_WDT_INTENSET));
  wdinfo("      INTFLAG:  %02x\n", getreg8(SAM_WDT_INTFLAG));
  wdinfo("        CTRLA:  %02x\n", getreg8(SAM_WDT_CTRLA));
  wdinfo("       CONFIG:  %02x\n", getreg8(SAM_WDT_CONFIG));
  wdinfo("          EWC:  %02x\n", getreg8(SAM_WDT_EWCTRL));
  wdinfo("        CLEAR:  %02x\n", getreg8(SAM_WDT_CLEAR));
}
/****************************************************************************
 * Name: sam_interrupt
 *
 * Description:
 *   WDT interrupt
 *
 * Input Parameters:
 *   Usual interrupt handler arguments.
 *
 * Returned Value:
 *   Always returns OK.
 *
 ****************************************************************************/

/****************************************************************************
 * Name: sam_start
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

static int sam_start(FAR struct watchdog_lowerhalf_s *lower)
{
  FAR struct sam_lowerhalf_s *priv = (FAR struct sam_lowerhalf_s *)lower;
  irqstate_t flags;

  wdinfo("Entry: started=%d\n");
  DEBUGASSERT(priv);

  /* Have we already been started? */

  if (!priv->started)
    {
      /* Set up prescaler and reload value for the selected timeout before
       * starting the watchdog timer.
       */

       /* Enable IWDG (the LSI oscillator will be enabled by hardware).  NOTE:
       * If the "Hardware watchdog" feature is enabled through the device option
       * bits, the watchdog is automatically enabled at power-on.
       */

      flags           = enter_critical_section();
      
      /*putreg8(WDT_CTRLA_ENABLE, SAM_WDT_CTRLA);
      sam_sync_wdt(WDT_SYNCBUSY_ENABLE);*/
      
      priv->lastreset = clock_systime_ticks();
      
      priv->started   = true;
      
      leave_critical_section(flags);
    }
  return OK;
}

/****************************************************************************
 * Name: sam_stop
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

static int sam_stop(FAR struct watchdog_lowerhalf_s *lower)
{
  /* The watchdog is always disabled after a reset. It is enabled by clearing
   * the WDDIS bit in the WDT_CR register, then it cannot be disabled again
   * except by a reset.
   */

  wdinfo("Entry\n");
  return -ENOSYS;
}

/****************************************************************************
 * Name: sam_keepalive
 *
 * Description:
 *   Reset the watchdog timer to the current timeout value, prevent any
 *   imminent watchdog timeouts.  This is sometimes referred as "pinging"
 *   the watchdog timer or "petting the dog".
 *
 *   The application program must write in the SAM_WDT_CLEAR register at regular
 *   intervals during normal operation to prevent an MCU reset.
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int sam_keepalive(FAR struct watchdog_lowerhalf_s *lower)
{
  FAR struct sam_lowerhalf_s *priv = (FAR struct sam_lowerhalf_s *)lower;
 irqstate_t flags;

  wdinfo("Entry\n");
  /* Reload the WDT timer */

  flags = enter_critical_section();

  putreg32(WDT_CLEAR_CLEAR, SAM_WDT_CLEAR);
  priv->lastreset = clock_systime_ticks();
  
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: sam_getstatus
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

static int sam_getstatus(FAR struct watchdog_lowerhalf_s *lower,
                           FAR struct watchdog_status_s *status)
{
  FAR struct sam_lowerhalf_s *priv = (FAR struct sam_lowerhalf_s *)lower;
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

  ticks   = clock_systime_ticks() - priv->lastreset;
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
 * Name: sam_settimeout
 *
 * Description:
 *   Set a new timeout value (and reset the watchdog timer)
 *
 * Input Parameters:
 *   lower   - A pointer the publicly visible representation of the "lower-half"
 *             driver state structure.
 *   timeout - The new timeout value in millisecnds.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int sam_settimeout(FAR struct watchdog_lowerhalf_s *lower,
                            uint32_t timeout)
{
  FAR struct sam_lowerhalf_s *priv = (FAR struct sam_lowerhalf_s *)lower;
  uint64_t            tmp;
  uint32_t            period_cycles;
  uint8_t             timeout_period = WDT_CONFIG_PER_16K;
  irqstate_t flags;
 
  DEBUGASSERT(priv);
  wdinfo("Entry: timeout=%d\n", timeout);

  /* Can this timeout be represented? */

  flags = enter_critical_section();
  
  /* calc the period cycles corresponding to timeout period */
  tmp = (uint64_t)timeout * WDT_PERIOD_RATE;

  /* check whether overflow*/
  if (tmp >> 32)
    return -ERANGE;

  period_cycles = (uint32_t)tmp;
  /* calc the register value corresponding to period cysles */
  switch (period_cycles) 
  {
    case WDT_CLK_8CYCLE *WDT_PERIOD_RATE:
      timeout_period = WDT_CONFIG_PER_8;
    break;
    
    case WDT_CLK_16CYCLE *WDT_PERIOD_RATE:
      timeout_period = WDT_CONFIG_PER_16;
    break;
    
    case WDT_CLK_32CYCLE *WDT_PERIOD_RATE:
      timeout_period = WDT_CONFIG_PER_32;
    break;
    
    case WDT_CLK_64CYCLE *WDT_PERIOD_RATE:
      timeout_period = WDT_CONFIG_PER_64;
    break;
    
    case WDT_CLK_128CYCLE *WDT_PERIOD_RATE:
      timeout_period = WDT_CONFIG_PER_128;
    break;
    
    case WDT_CLK_256CYCLE *WDT_PERIOD_RATE:
      timeout_period = WDT_CONFIG_PER_256;
    break;
    
    case WDT_CLK_512CYCLE *WDT_PERIOD_RATE:
      timeout_period = WDT_CONFIG_PER_512;
    break;
    
    case WDT_CLK_1024CYCLE *WDT_PERIOD_RATE:
      timeout_period = WDT_CONFIG_PER_1K;
    break;
    
    case WDT_CLK_2048CYCLE *WDT_PERIOD_RATE:
      timeout_period = WDT_CONFIG_PER_2K;
    break;
    
    case WDT_CLK_4096CYCLE *WDT_PERIOD_RATE:
      timeout_period = WDT_CONFIG_PER_4K;
    break;
    
    case WDT_CLK_8192CYCLE *WDT_PERIOD_RATE:
      timeout_period = WDT_CONFIG_PER_8K;
    break;
    
    case WDT_CLK_16384CYCLE *WDT_PERIOD_RATE:
      timeout_period = WDT_CONFIG_PER_16K;
    break;
  }

  if(!priv->started)
    putreg8(WDT_CONFIG_PER_16K, SAM_WDT_CONFIG);
  else
    putreg8(timeout_period, SAM_WDT_CONFIG);
  
  priv->reload = timeout_period;  
  wdinfo("fwdt=%d reload=%d timout=%d\n",
         WDT_FCLK, timeout_period, priv->timeout);
  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_wdt_initialize
 *
 * Description:
 *   Initialize the WDT watchdog timer.  The watchdog timer is initialized and
 *   registers as 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the watchdog.  This should be of the form
 *     /dev/watchdog0
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
void sam_wdt_initialize(FAR const char *devpath)
{
  FAR struct sam_lowerhalf_s *priv = &g_wdgdev;
  
  DEBUGASSERT((getreg8(SAM_WDT_CTRLA) & WDT_CTRLA_ENABLE) == 0);

  sam_apb_wdt_enableperiph();
  
  
  /* Initialize the driver state structure. */
   priv->ops = &g_wdgops;
   priv->started = false;  

  sam_settimeout((FAR struct watchdog_lowerhalf_s *)priv, BOARD_SCLK_FREQUENCY/2);

  (void)watchdog_register(devpath, (FAR struct watchdog_lowerhalf_s *)priv);

}

#endif /* CONFIG_WATCHDOG && CONFIG__WDT */
