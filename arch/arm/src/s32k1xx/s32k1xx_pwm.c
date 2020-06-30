/*****************************************************************************
 * arch/arm/src/s32k1xx/s32k1xx_pwm.c
 *
 *   Copyright (C) 2013, 2016, 2017 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Alan Carvalho de Assis <acassis@gmail.com>
 *            Ken Fazzone <kfazz01@gmail.com>
 *            David Sidrane <david_s5@nscdg.com>
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
 *****************************************************************************/

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/timers/pwm.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "arm_arch.h"

#include "chip.h"

#include "s32k1xx_clockconfig.h"
#include "s32k1xx_pin.h"
#include "s32k1xx_pwm.h"
#include "hardware/s32k1xx_gpio.h"
#include "hardware/s32k1xx_ftm.h"
#include "hardware/s32k1xx_sim.h"

/* This module then only compiles if there is at least one enabled timer
 * intended for use with the PWM upper half driver.
 */

#if defined(CONFIG_S32K1XX_FTM0_PWM) || defined(CONFIG_S32K1XX_FTM1_PWM) || \
    defined(CONFIG_S32K1XX_FTM2_PWM) || defined(CONFIG_S32K1XX_FTM3_PWM) || \
    defined(CONFIG_S32K1XX_FTM4_PWM) || defined(CONFIG_S32K1XX_FTM5_PWM) || \
    defined(CONFIG_S32K1XX_FTM6_PWM) || defined(CONFIG_S32K1XX_FTM7_PWM)

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

/* PWM/Timer Definitions *****************************************************/

/* Debug *********************************************************************/

#ifdef CONFIG_DEBUG_PWM_INFO
#  define pwm_dumpgpio(p,m) s32k1xx_pindump(p,m)
#else
#  define pwm_dumpgpio(p,m)
#endif

/*****************************************************************************
 * Private Types
 *****************************************************************************/

/* This structure represents the state of one PWM timer */

struct s32k1xx_pwmtimer_s
{
  FAR const struct pwm_ops_s *ops;     /* PWM operations */
  uint8_t                     tpmid;   /* Timer/PWM Module ID {0,..,7} */
  uint8_t                     channel; /* Timer/PWM Module channel: {0,...,7} */
  uint32_t                    base;    /* The base address of the timer */
  uint32_t                    pincfg;  /* Output pin configuration */
  uint32_t                    pclk;    /* The frequency of the peripheral clock */
};

/*****************************************************************************
 * Static Function Prototypes
 *****************************************************************************/

/* Register access */

static uint32_t pwm_getreg(struct s32k1xx_pwmtimer_s *priv, int offset);
static void pwm_putreg(struct s32k1xx_pwmtimer_s *priv, int offset,
                       uint32_t value);

#ifdef CONFIG_DEBUG_PWM_INFO
static void pwm_dumpregs(struct s32k1xx_pwmtimer_s *priv,
                         FAR const char *msg);
#else
#  define pwm_dumpregs(priv,msg)
#endif

/* Timer management */

static int pwm_timer(FAR struct s32k1xx_pwmtimer_s *priv,
                     FAR const struct pwm_info_s *info);

/* PWM driver methods */

static int pwm_setup(FAR struct pwm_lowerhalf_s *dev);
static int pwm_shutdown(FAR struct pwm_lowerhalf_s *dev);

static int pwm_start(FAR struct pwm_lowerhalf_s *dev,
                     FAR const struct pwm_info_s *info);

static int pwm_stop(FAR struct pwm_lowerhalf_s *dev);
static int pwm_ioctl(FAR struct pwm_lowerhalf_s *dev, int cmd,
                     unsigned long arg);

/*****************************************************************************
 * Private Data
 *****************************************************************************/

/* This is the list of lower half PWM driver methods used by the upper half
 * driver.
 */

static const struct pwm_ops_s g_pwmops =
{
  .setup    = pwm_setup,
  .shutdown = pwm_shutdown,
  .start    = pwm_start,
  .stop     = pwm_stop,
  .ioctl    = pwm_ioctl,
};

#ifdef CONFIG_S32K1XX_FTM0_PWM
static struct s32k1xx_pwmtimer_s g_pwm0dev =
{
  .ops        = &g_pwmops,
  .tpmid      = 0,
  .channel    = CONFIG_S32K1XX_FTM0_CHANNEL,
  .base       = S32K1XX_FTM0_BASE,
  .pincfg     = PWM_FTM0_PINCFG,
};
#endif

#ifdef CONFIG_S32K1XX_FTM1_PWM
static struct s32k1xx_pwmtimer_s g_pwm1dev =
{
  .ops        = &g_pwmops,
  .tpmid      = 1,
  .channel    = CONFIG_S32K1XX_FTM1_CHANNEL,
  .base       = S32K1XX_FTM1_BASE,
  .pincfg     = PWM_FTM1_PINCFG,
};
#endif

#ifdef CONFIG_S32K1XX_FTM2_PWM
static struct s32k1xx_pwmtimer_s g_pwm2dev =
{
  .ops        = &g_pwmops,
  .tpmid      = 2,
  .channel    = CONFIG_S32K1XX_FTM2_CHANNEL,
  .base       = S32K1XX_FTM2_BASE,
  .pincfg     = PWM_FTM2_PINCFG,
};
#endif

#ifdef CONFIG_S32K1XX_FTM3_PWM
static struct s32k1xx_pwmtimer_s g_pwm3dev =
{
  .ops        = &g_pwmops,
  .tpmid      = 3,
  .channel    = CONFIG_S32K1XX_FTM3_CHANNEL,
  .base       = S32K1XX_FTM3_BASE,
  .pincfg     = PWM_FTM3_PINCFG,
};
#endif

#ifdef CONFIG_S32K1XX_FTM4_PWM
static struct s32k1xx_pwmtimer_s g_pwm4dev =
{
  .ops        = &g_pwmops,
  .tpmid      = 4,
  .channel    = CONFIG_S32K1XX_FTM4_CHANNEL,
  .base       = S32K1XX_FTM4_BASE,
  .pincfg     = PWM_FTM4_PINCFG,
};
#endif

#ifdef CONFIG_S32K1XX_FTM5_PWM
static struct s32k1xx_pwmtimer_s g_pwm5dev =
{
  .ops        = &g_pwmops,
  .tpmid      = 5,
  .channel    = CONFIG_S32K1XX_FTM5_CHANNEL,
  .base       = S32K1XX_FTM5_BASE,
  .pincfg     = PWM_FTM5_PINCFG,
};
#endif

#ifdef CONFIG_S32K1XX_FTM6_PWM
static struct s32k1xx_pwmtimer_s g_pwm6dev =
{
  .ops        = &g_pwmops,
  .tpmid      = 6,
  .channel    = CONFIG_S32K1XX_FTM6_CHANNEL,
  .base       = S32K1XX_FTM6_BASE,
  .pincfg     = PWM_FTM6_PINCFG,
};
#endif

#ifdef CONFIG_S32K1XX_FTM7_PWM
static struct s32k1xx_pwmtimer_s g_pwm7dev =
{
  .ops        = &g_pwmops,
  .tpmid      = 7,
  .channel    = CONFIG_S32K1XX_FTM7_CHANNEL,
  .base       = S32K1XX_FTM7_BASE,
  .pincfg     = PWM_FTM7_PINCFG,
};
#endif

/*****************************************************************************
 * Private Functions
 *****************************************************************************/

/*****************************************************************************
 * Name: pwm_getreg
 *
 * Description:
 *   Read the value of an PWM timer register.
 *
 * Input Parameters:
 *   priv - A reference to the PWM block status
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *   The current contents of the specified register
 *
 *****************************************************************************/

static uint32_t pwm_getreg(struct s32k1xx_pwmtimer_s *priv, int offset)
{
  return getreg32(priv->base + offset);
}

/*****************************************************************************
 * Name: pwm_putreg
 *
 * Description:
 *   Read the value of an PWM timer register.
 *
 * Input Parameters:
 *   priv - A reference to the PWM block status
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *   None
 *
 *****************************************************************************/

static void pwm_putreg(struct s32k1xx_pwmtimer_s *priv, int offset,
                       uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/*****************************************************************************
 * Name: pwm_dumpregs
 *
 * Description:
 *   Dump all timer registers.
 *
 * Input Parameters:
 *   priv - A reference to the PWM block status
 *
 * Returned Value:
 *   None
 *
 *****************************************************************************/

#ifdef CONFIG_DEBUG_PWM_INFO
static void pwm_dumpregs(struct s32k1xx_pwmtimer_s *priv, FAR const char *msg)
{
  pwminfo("%s:\n", msg);
  pwminfo("  FTM%d_SC:     %04x   FTM%d_CNT:  %04x     FTM%d_MOD:  %04x\n",
          priv->tpmid, pwm_getreg(priv, S32K1XX_FTM_SC_OFFSET),
          priv->tpmid, pwm_getreg(priv, S32K1XX_FTM_CNT_OFFSET),
          priv->tpmid, pwm_getreg(priv, S32K1XX_FTM_MOD_OFFSET));
  pwminfo("  FTM%d_STATUS: %04x   FTM%d_CONF: %04x\n",
          priv->tpmid, pwm_getreg(priv, S32K1XX_FTM_STATUS_OFFSET),
          priv->tpmid, pwm_getreg(priv, S32K1XX_FTM_CONF_OFFSET));
  pwminfo("   FTM%d_C0SC:  %04x   FTM%d_C0V:  %04x\n",
          priv->tpmid, pwm_getreg(priv, S32K1XX_FTM_C0SC_OFFSET),
          priv->tpmid, pwm_getreg(priv, S32K1XX_FTM_C0V_OFFSET));
  pwminfo("   FTM%d_C1SC:  %04x   FTM%d_C1V:  %04x\n",
          priv->tpmid, pwm_getreg(priv, S32K1XX_FTM_C1SC_OFFSET),
          priv->tpmid, pwm_getreg(priv, S32K1XX_FTM_C1V_OFFSET));
  pwminfo("   FTM%d_C2SC:  %04x   FTM%d_C2V:  %04x\n",
          priv->tpmid, pwm_getreg(priv, S32K1XX_FTM_C2SC_OFFSET),
          priv->tpmid, pwm_getreg(priv, S32K1XX_FTM_C2V_OFFSET));
  pwminfo("   FTM%d_C3SC:  %04x   FTM%d_C3V:  %04x\n",
          priv->tpmid, pwm_getreg(priv, S32K1XX_FTM_C3SC_OFFSET),
          priv->tpmid, pwm_getreg(priv, S32K1XX_FTM_C3V_OFFSET));
  pwminfo("   FTM%d_C4SC:  %04x   FTM%d_C4V:  %04x\n",
          priv->tpmid, pwm_getreg(priv, S32K1XX_FTM_C4SC_OFFSET),
          priv->tpmid, pwm_getreg(priv, S32K1XX_FTM_C4V_OFFSET));
  pwminfo("   FTM%d_C5SC:  %04x   FTM%d_C5V:  %04x\n",
          priv->tpmid, pwm_getreg(priv, S32K1XX_FTM_C5SC_OFFSET),
          priv->tpmid, pwm_getreg(priv, S32K1XX_FTM_C5V_OFFSET));
  pwminfo("   FTM%d_C6SC:  %04x   FTM%d_C6V:  %04x\n",
          priv->tpmid, pwm_getreg(priv, S32K1XX_FTM_C6SC_OFFSET),
          priv->tpmid, pwm_getreg(priv, S32K1XX_FTM_C6V_OFFSET));
  pwminfo("   FTM%d_C7SC:  %04x   FTM%d_C7V:  %04x\n",
          priv->tpmid, pwm_getreg(priv, S32K1XX_FTM_C7SC_OFFSET),
          priv->tpmid, pwm_getreg(priv, S32K1XX_FTM_C7V_OFFSET));
}
#endif

/*****************************************************************************
 * Name: pwm_timer
 *
 * Description:
 *   (Re-)initialize the timer resources and start the pulsed output
 *
 * Input Parameters:
 *   priv - A reference to the lower half PWM driver state structure
 *   info - A reference to the characteristics of the pulsed output
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 *****************************************************************************/

static int pwm_timer(FAR struct s32k1xx_pwmtimer_s *priv,
                     FAR const struct pwm_info_s *info)
{
  /* Calculated values */

  uint32_t prescaler;
  uint32_t tpmclk;
  uint32_t modulo;
  uint32_t regval;
  uint32_t cv;
  uint8_t i;

  static const uint8_t presc_values[8] = {1, 2, 4, 8, 16, 32, 64, 128};

  /* Register contents */

  DEBUGASSERT(priv != NULL && info != NULL);

  pwminfo("FTM%d channel: %d frequency: %d duty: %08x\n",
          priv->tpmid, priv->channel, info->frequency, info->duty);

  DEBUGASSERT(info->frequency > 0 && info->duty > 0 &&
              info->duty < uitoub16(100));

  /* Calculate optimal values for the timer prescaler and for the timer modulo
   * register.  If' frequency' is the desired frequency, then
   *
   *   modulo = tpmclk / frequency
   *   tpmclk = pclk / presc
   *
   * Or,
   *
   *   modulo = pclk / presc / frequency
   *
   * There are many solutions to do this, but the best solution will be the
   * one that has the largest modulo value and the smallest prescaler value.
   * That is the solution that should give us the most accuracy in the timer
   * control.  Subject to:
   *
   *   1 <= presc  <= 128 (need to be 1, 2, 4, 8, 16, 32, 64, 128)
   *   1 <= modulo <= 65535
   *
   * So presc = pclk / 65535 / frequency would be optimal.
   *
   * Example:
   *
   *  pclk      = 24 MHz
   *  frequency = 100 Hz
   *
   *  prescaler = 24,000,000 / 65,535 / 100
   *            = 3.6 (or 4 -- taking the ceiling always)
   *  timclk    = 24,000,000 / 4
   *            = 6,000,000
   *  modulo    = 6,000,000 / 100
   *            = 60,000
   */

  prescaler = (priv->pclk / info->frequency + 65534) / 65535;

  for (i = 0; i < 7; i++)
    {
      if (prescaler <= presc_values[i])
        {
           break;
        }
    }

  prescaler = i;

  tpmclk = priv->pclk / presc_values[prescaler];

  modulo = tpmclk / info->frequency;
  if (modulo < 1)
    {
      modulo = 1;
    }
  else if (modulo > 65535)
    {
      modulo = 65535;
    }

  /* Duty cycle:
   *
   * duty cycle = cv / modulo (fractional value)
   */

  cv = b16toi(info->duty * modulo + b16HALF);

  pwminfo("FTM%d PCLK: %d frequency: %d FTMCLK: %d prescaler: %d modulo: %d \
           c0v: %d\n",
          priv->tpmid, priv->pclk, info->frequency, tpmclk,
          presc_values[prescaler], modulo, cv);

  /* Disable FTM and reset CNT before writing MOD and PS */

  pwm_putreg(priv, S32K1XX_FTM_SC_OFFSET, FTM_SC_CLKS_DIS);
  pwm_putreg(priv, S32K1XX_FTM_CNT_OFFSET, 0);

  /* Set the modulo value */

  pwm_putreg(priv, S32K1XX_FTM_MOD_OFFSET, (uint16_t)modulo);

  /* Set the duty cycle for channel specific */

  switch (priv->channel)
    {
      case 0:  /* PWM Mode configuration: Channel 0 */
        {
          pwm_putreg(priv, S32K1XX_FTM_C0SC_OFFSET,
                     FTM_CNSC_MSB | FTM_CNSC_ELSB);
          pwm_putreg(priv, S32K1XX_FTM_C0V_OFFSET, (uint16_t) cv);
        }
        break;

      case 1:  /* PWM Mode configuration: Channel 1 */
        {
          pwm_putreg(priv, S32K1XX_FTM_C1SC_OFFSET,
                     FTM_CNSC_MSB | FTM_CNSC_ELSB);
          pwm_putreg(priv, S32K1XX_FTM_C1V_OFFSET, (uint16_t) cv);
        }
        break;

      case 2:  /* PWM Mode configuration: Channel 2 */
        {
          pwm_putreg(priv, S32K1XX_FTM_C2SC_OFFSET,
                     FTM_CNSC_MSB | FTM_CNSC_ELSB);
          pwm_putreg(priv, S32K1XX_FTM_C2V_OFFSET, (uint16_t) cv);
        }
        break;

      case 3:  /* PWM Mode configuration: Channel 3 */
        {
          pwm_putreg(priv, S32K1XX_FTM_C3SC_OFFSET,
                     FTM_CNSC_MSB | FTM_CNSC_ELSB);
          pwm_putreg(priv, S32K1XX_FTM_C3V_OFFSET, (uint16_t) cv);
        }
        break;

      case 4:  /* PWM Mode configuration: Channel 4 */
        {
          pwm_putreg(priv, S32K1XX_FTM_C4SC_OFFSET,
                     FTM_CNSC_MSB | FTM_CNSC_ELSB);
          pwm_putreg(priv, S32K1XX_FTM_C4V_OFFSET, (uint16_t) cv);
        }
        break;

      case 5:  /* PWM Mode configuration: Channel 5 */
        {
          pwm_putreg(priv, S32K1XX_FTM_C5SC_OFFSET,
                     FTM_CNSC_MSB | FTM_CNSC_ELSB);
          pwm_putreg(priv, S32K1XX_FTM_C5V_OFFSET, (uint16_t) cv);
        }
        break;

      case 6:  /* PWM Mode configuration: Channel 6 */
        {
          pwm_putreg(priv, S32K1XX_FTM_C6SC_OFFSET,
                     FTM_CNSC_MSB | FTM_CNSC_ELSB);
          pwm_putreg(priv, S32K1XX_FTM_C6V_OFFSET, (uint16_t) cv);
        }
        break;
      case 7:  /* PWM Mode configuration: Channel 7 */
        {
          pwm_putreg(priv, S32K1XX_FTM_C7SC_OFFSET,
                     FTM_CNSC_MSB | FTM_CNSC_ELSB);
          pwm_putreg(priv, S32K1XX_FTM_C7V_OFFSET, (uint16_t) cv);
        }
        break;

      default:
        pwmerr("ERROR: No such channel: %d\n", priv->channel);
        return -EINVAL;
    }

  /* Set prescaler and enable clock */

  regval = pwm_getreg(priv, S32K1XX_FTM_SC_OFFSET);
  regval &= ~(FTM_SC_PS_MASK);
  regval &= ~(FTM_SC_CLKS_MASK);
  regval |= prescaler | FTM_SC_CLKS_FTM;
  regval |= FTM_SC_PWMEN(priv->channel);
  pwm_putreg(priv, S32K1XX_FTM_SC_OFFSET, regval);

  pwm_dumpregs(priv, "After starting");
  return OK;
}

/*****************************************************************************
 * Name: pwm_setup
 *
 * Description:
 *   This method is called when the driver is opened.  The lower half driver
 *   should configure and initialize the device so that it is ready for use.
 *   It should not, however, output pulses until the start method is called.
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 * Assumptions:
 *   AHB1 or 2 clocking for the GPIOs and timer has already been configured
 *   by the RCC logic at power up.
 *
 *****************************************************************************/

static int pwm_setup(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct s32k1xx_pwmtimer_s *priv = (FAR struct s32k1xx_pwmtimer_s *)dev;

  /* Note: The appropriate clock should for the right FTM device should
   * already be enabled in the board-specific s32k1xx_periphclocks.c file.
   */

  pwminfo("FTM%d pincfg: %08x\n", priv->tpmid, priv->pincfg);
  pwm_dumpregs(priv, "Initially");

  /* Configure the PWM output pin, but do not start the timer yet */

  s32k1xx_pinconfig(priv->pincfg);
  pwm_dumpgpio(priv->pincfg, "PWM setup");
  return OK;
}

/*****************************************************************************
 * Name: pwm_shutdown
 *
 * Description:
 *   This method is called when the driver is closed.  The lower half driver
 *   stop pulsed output, free any resources, disable the timer hardware, and
 *   put the system into the lowest possible power usage state
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 *****************************************************************************/

static int pwm_shutdown(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct s32k1xx_pwmtimer_s *priv = (FAR struct s32k1xx_pwmtimer_s *)dev;
  uint32_t pincfg;

  pwminfo("FTM%d pincfg: %08x\n", priv->tpmid, priv->pincfg);

  /* Make sure that the output has been stopped */

  pwm_stop(dev);

  /* Then put the GPIO pin back to the default state */

  pincfg  = (priv->pincfg & ~(_PIN_MODE_MASK));
  pincfg |= GPIO_INPUT;
  s32k1xx_pinconfig(pincfg);
  return OK;
}

/*****************************************************************************
 * Name: pwm_start
 *
 * Description:
 *   (Re-)initialize the timer resources and start the pulsed output
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *   info - A reference to the characteristics of the pulsed output
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 *****************************************************************************/

static int pwm_start(FAR struct pwm_lowerhalf_s *dev,
                     FAR const struct pwm_info_s *info)
{
  FAR struct s32k1xx_pwmtimer_s *priv = (FAR struct s32k1xx_pwmtimer_s *)dev;
  return pwm_timer(priv, info);
}

/*****************************************************************************
 * Name: pwm_stop
 *
 * Description:
 *   Stop the pulsed output and reset the timer resources
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 * Assumptions:
 *   This function is called to stop the pulsed output at anytime.  This
 *   method is also called from the timer interrupt handler when a repetition
 *   count expires... automatically stopping the timer.
 *
 *****************************************************************************/

static int pwm_stop(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct s32k1xx_pwmtimer_s *priv = (FAR struct s32k1xx_pwmtimer_s *)dev;
  irqstate_t flags;

  pwminfo("FTM%d\n", priv->tpmid);

  /* Disable interrupts momentary to stop any ongoing timer processing and
   * to prevent any concurrent access to the reset register.
   */

  flags = enter_critical_section();

  /* Disable further interrupts and stop the timer */

  pwm_putreg(priv, S32K1XX_FTM_SC_OFFSET, FTM_SC_CLKS_DIS);
  pwm_putreg(priv, S32K1XX_FTM_CNT_OFFSET, 0);

  /* Determine which timer channel to clear */

  switch (priv->channel)
    {
      case 0:
        pwm_putreg(priv, S32K1XX_FTM_C0V_OFFSET, 0);
        break;

      case 1:
        pwm_putreg(priv, S32K1XX_FTM_C1V_OFFSET, 0);
        break;

      case 2:
        pwm_putreg(priv, S32K1XX_FTM_C2V_OFFSET, 0);
        break;

      case 3:
        pwm_putreg(priv, S32K1XX_FTM_C3V_OFFSET, 0);
        break;

      case 4:
        pwm_putreg(priv, S32K1XX_FTM_C4V_OFFSET, 0);
        break;

      case 5:
        pwm_putreg(priv, S32K1XX_FTM_C5V_OFFSET, 0);
        break;

      case 6:
        pwm_putreg(priv, S32K1XX_FTM_C6V_OFFSET, 0);
        break;

      case 7:
        pwm_putreg(priv, S32K1XX_FTM_C7V_OFFSET, 0);
        break;

      default:
        pwmerr("ERROR: No such channel: %d\n", priv->channel);
        return -EINVAL;
    }

  leave_critical_section(flags);

  pwm_dumpregs(priv, "After stop");
  return OK;
}

/*****************************************************************************
 * Name: pwm_ioctl
 *
 * Description:
 *   Lower-half logic may support platform-specific ioctl commands
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *   cmd - The ioctl command
 *   arg - The argument accompanying the ioctl command
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 *****************************************************************************/

static int pwm_ioctl(FAR struct pwm_lowerhalf_s *dev, int cmd,
                     unsigned long arg)
{
#ifdef CONFIG_DEBUG_PWM_INFO
  FAR struct s32k1xx_pwmtimer_s *priv = (FAR struct s32k1xx_pwmtimer_s *)dev;

  /* There are no platform-specific ioctl commands */

  pwminfo("FTM%d\n", priv->tpmid);
#endif
  return -ENOTTY;
}

/*****************************************************************************
 * Public Functions
 *****************************************************************************/

/*****************************************************************************
 * Name: s32k1xx_pwminitialize
 *
 * Description:
 *   Initialize one timer for use with the upper_level PWM driver.
 *
 * Input Parameters:
 *   timer - A number identifying the timer use.
 *
 * Returned Value:
 *   On success, a pointer to the S32K1XX lower half PWM driver is returned.
 *   NULL is returned on any failure.
 *
 *****************************************************************************/

FAR struct pwm_lowerhalf_s *s32k1xx_pwminitialize(int timer)
{
  FAR struct s32k1xx_pwmtimer_s *lower;
  int ret;
  uint32_t sysclk;

  sysclk = s32k1xx_get_coreclk();
  ret = (sysclk == 0) ? -ENODEV : OK;

  pwminfo("FTM%d\n", timer);

  switch (timer)
    {
#ifdef CONFIG_S32K1XX_FTM0_PWM
      case 0:
        g_pwm0dev.pclk = sysclk;
        lower = &g_pwm0dev;
        break;
#endif
#ifdef CONFIG_S32K1XX_FTM1_PWM
      case 1:
        g_pwm1dev.pclk = sysclk;
        lower = &g_pwm1dev;
        break;
#endif
#ifdef CONFIG_S32K1XX_FTM2_PWM
      case 2:
        g_pwm2dev.pclk = sysclk;
        lower = &g_pwm2dev;
        break;
#endif
#ifdef CONFIG_S32K1XX_FTM3_PWM
      case 3:
        g_pwm3dev.pclk = sysclk;
        lower = &g_pwm3dev;
        break;
#endif
#ifdef CONFIG_S32K1XX_FTM4_PWM
      case 4:
        g_pwm4dev.pclk = sysclk;
        lower = &g_pwm4dev;
        break;
#endif
#ifdef CONFIG_S32K1XX_FTM5_PWM
      case 5:
        g_pwm5dev.pclk = sysclk;
        lower = &g_pwm5dev;
        break;
#endif
#ifdef CONFIG_S32K1XX_FTM6_PWM
      case 6:
        g_pwm6dev.pclk = sysclk;
        lower = &g_pwm6dev;
        break;
#endif
#ifdef CONFIG_S32K1XX_FTM7_PWM
      case 7:
        g_pwm7dev.pclk = sysclk;
        lower = &g_pwm7dev;
        break;
#endif
      default:
        pwmerr("ERROR: No such timer configured\n");
        return NULL;
    }

    if (ret != OK)
      {
        pwmerr("ERROR: FTM%d peripheral clock not available\n", timer);
        return NULL;
      }

  return (FAR struct pwm_lowerhalf_s *)lower;
}

#endif /* CONFIG_S32K1XX_FTMn_PWM, n = 0,...,7 */
