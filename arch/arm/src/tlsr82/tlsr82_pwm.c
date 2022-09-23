/****************************************************************************
 * arch/arm/src/tlsr82/tlsr82_pwm.c
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

#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/timers/pwm.h>
#include <arch/board/board.h>

#include "arm_internal.h"

#include "chip.h"
#include "tlsr82_pwm.h"
#include "tlsr82_gpio.h"
#include "tlsr82_gpio_cfg.h"

#include "hardware/tlsr82_pwm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The pwm clock source is system clock */

#define PWM_SRC_CLK_HZ        (CONFIG_TLSR82_CPU_CLK_MHZ * 1000000)

/* The default clkdiv = 9, default pwm clk = PWM_SRC_CLK_HZ / 10 */

#define PWM_INIT_CLKDIV       9

/* Make sure the max frequency has at least 1% duty accruay */

#define PWM_MAX_FREQ          (g_pwmclk / 100)
#define PWM_MIN_FREQ          (g_pwmclk / UINT16_MAX + 1)

/* PWM max pulse count number, bit 0 ~ 13 */

#define PWM_MAX_COUNT         (0x3fff)

/* PWM simple operation definition */

#define PWM_INT_PEND(id)      BM_IS_SET(PWM_IRQ_STA_REG, 1 << ((id) + 2))
#define PWM_INT_CLEAR(id)     BM_CLR(PWM_IRQ_STA_REG, 1 << ((id) + 2))
#define PWM_INT_ENABLE(id)    BM_SET(PWM_IRQ_CTRL_REG, 1 << ((id) + 2))
#define PWM_INT_DISABLE(id)   BM_CLR(PWM_IRQ_CTRL_REG, 1 << ((id) + 2))

#define PWM0_PNUM_INT_PEND()      BM_IS_SET(PWM_IRQ_STA_REG, 1 << 0)
#define PWM0_PNUM_INT_CLEAR()     BM_CLR(PWM_IRQ_STA_REG, 1 << 0)
#define PWM0_PNUM_INT_ENABLE()    BM_SET(PWM_IRQ_CTRL_REG, 1 << 0)
#define PWM0_PNUM_INT_ISENABLE()  BM_IS_SET(PWM_IRQ_CTRL_REG, 1 << 0)
#define PWM0_PNUM_INT_DISABLE()   BM_CLR(PWM_IRQ_CTRL_REG, 1 << 0)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure represents the state of one PWM timer */

struct tlsr82_pwmtimer_s
{
  const struct pwm_ops_s *ops;   /* PWM operations */
  uint32_t pincfg;               /* PWM pin config */
  uint8_t id;                    /* PWM hardware id */
  bool invert;                   /* PWM output is inverted or not */
  bool started;                  /* PWM output has started or not */
#ifdef CONFIG_TLSR82_PWM0_PULSECOUNT
  uint8_t irq;                   /* Timer update IRQ */
  uint32_t count;                /* Remaining pulse count */
#endif
  uint32_t frequency;            /* Current frequency setting */
  uint16_t max;
  uint16_t cmp;
#ifdef CONFIG_TLSR82_PWM0_PULSECOUNT
  void *handle;                  /* Handle used for upper-half callback */
#endif
};

/****************************************************************************
 * Static Function Prototypes
 ****************************************************************************/

/* Register access */

#ifdef CONFIG_DEBUG_PWM_INFO
static void pwm_dumpregs(const char *msg);
#else
#  define pwm_dumpregs(msg)
#endif

/* Timer management */

static void pwm_enable(struct tlsr82_pwmtimer_s *priv, bool en);
static int pwm_cfg_check(struct tlsr82_pwmtimer_s *priv);
static int pwm_config(struct tlsr82_pwmtimer_s *priv,
                      const struct pwm_info_s *info);

#ifdef CONFIG_TLSR82_PWM0_PULSECOUNT
static int pwm_interrupt(int irq, void *context, void *arg);
#endif

/* PWM driver methods */

static int pwm_setup(struct pwm_lowerhalf_s *dev);
static int pwm_shutdown(struct pwm_lowerhalf_s *dev);

#ifdef CONFIG_TLSR82_PWM0_PULSECOUNT
static int pwm_start(struct pwm_lowerhalf_s *dev,
                     const struct pwm_info_s *info,
                     void *handle);
#else
static int pwm_start(struct pwm_lowerhalf_s *dev,
                     const struct pwm_info_s *info);
#endif

static int pwm_stop(struct pwm_lowerhalf_s *dev);
static int pwm_ioctl(struct pwm_lowerhalf_s *dev,
                     int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t g_pwmclk;

/* This is the list of lower half PWM driver methods used by the upper half
 * driver
 */

static const struct pwm_ops_s g_pwmops =
{
  .setup       = pwm_setup,
  .shutdown    = pwm_shutdown,
  .start       = pwm_start,
  .stop        = pwm_stop,
  .ioctl       = pwm_ioctl,
};

#ifdef CONFIG_TLSR82_PWM0
static struct tlsr82_pwmtimer_s g_pwm0dev =
{
  .ops         = &g_pwmops,
  .pincfg      = BOARD_PWM0_PIN,
  .id          = 0,
  .invert      = false,
  .started     = false,
#ifdef CONFIG_TLSR82_PWM0_PULSECOUNT
  .irq         = NR_SW_PWM_IRQ,
#endif
};
#endif

#ifdef CONFIG_TLSR82_PWM1
static struct tlsr82_pwmtimer_s g_pwm1dev =
{
  .ops         = &g_pwmops,
  .pincfg      = BOARD_PWM1_PIN,
  .id          = 1,
  .invert      = true,
  .started     = false,
};
#endif

#ifdef CONFIG_TLSR82_PWM2
static struct tlsr82_pwmtimer_s g_pwm2dev =
{
  .ops         = &g_pwmops,
  .pincfg      = BOARD_PWM2_PIN,
  .id          = 2,
  .invert      = true,
  .started     = false,
};
#endif

#ifdef CONFIG_TLSR82_PWM3
static struct tlsr82_pwmtimer_s g_pwm3dev =
{
  .ops         = &g_pwmops,
  .pincfg      = BOARD_PWM3_PIN,
  .id          = 3,
  .invert      = false,
  .started     = false,
};
#endif

#ifdef CONFIG_TLSR82_PWM4
static struct tlsr82_pwmtimer_s g_pwm4dev =
{
  .ops         = &g_pwmops,
  .pincfg      = BOARD_PWM4_PIN,
  .id          = 4,
  .invert      = false,
  .started     = false,
};
#endif

#ifdef CONFIG_TLSR82_PWM5
static struct tlsr82_pwmtimer_s g_pwm5dev =
{
  .ops         = &g_pwmops,
  .pincfg      = BOARD_PWM5_PIN,
  .id          = 5,
  .invert      = false,
  .started     = false,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pwm_dumpregs
 *
 * Description:
 *   Dump all pwm registers.
 *
 * Input Parameters:
 *   msg - message.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_PWM_INFO
static void pwm_dumpregs(const char *msg)
{
  int i;
  pwminfo("%s:\n", msg);
  pwminfo("PWM_CLKDIV_REG   : 0x%x\n", PWM_CLKDIV_REG);
  pwminfo("PWM_ENABLE0_REG  : 0x%x\n", PWM_ENABLE0_REG);
  pwminfo("PWM_MODE0_REG    : 0x%x\n", PWM_MODE0_REG);
  pwminfo("PWM_ENABLE_REG   : 0x%x\n", PWM_ENABLE_REG);
  pwminfo("PWM_INVERT_REG   : 0x%x\n", PWM_INVERT_REG);
  pwminfo("PWM_N_INVERT_REG : 0x%x\n", PWM_N_INVERT_REG);
  pwminfo("PWM_POL_REG      : 0x%x\n", PWM_POL_REG);

  for (i = 0; i < 5; i++)
    {
      pwminfo("PWM_CYCLE_REG(%d): 0x%lx\n", i, PWM_CYCLE_REG(i));
      pwminfo("PWM_CMP_REG(%d)  : 0x%x\n", i, PWM_CMP_REG(i));
      pwminfo("PWM_MAX_REG(%d)  : 0x%x\n", i, PWM_MAX_REG(i));
    }

  pwminfo("PWM_PLUSE_NUM_RE : 0x%x\n", PWM_PLUSE_NUM_REG);
  pwminfo("PWM_IRQ_CTRL_REG : 0x%x\n", PWM_IRQ_CTRL_REG);
  pwminfo("PWM_IRQ_STA_REG  : 0x%x\n", PWM_IRQ_STA_REG);
}
#endif

/****************************************************************************
 * Name: pwm_config
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
 ****************************************************************************/

static int pwm_config(struct tlsr82_pwmtimer_s *priv,
                      const struct pwm_info_s *info)
{
  /* Calculated values */

  uint16_t max;
  uint16_t cmp;

  if (priv == NULL || info == NULL)
    {
      pwmerr("priv or info is null, priv=%p info=%p\n", priv, info);
      return -EINVAL;
    }

  pwminfo("PWM%d invert: %d frequency: %lu duty: %08lx\n",
          (int)priv->id, (int)priv->invert, info->frequency,
          info->duty);

  if (info->frequency == 0 || info->duty >= uitoub16(100))
    {
      pwrerr("pwm info is invalid, fre: %lu duty: %08lx\n",
             info->frequency, info->duty);
      return -EINVAL;
    }

  if (info->frequency > PWM_MAX_FREQ || info->frequency < PWM_MIN_FREQ)
    {
      pwmerr("pwm freq out of range, freq: %ld, max: %ld, min: %ld\n",
             info->frequency, PWM_MAX_FREQ, PWM_MIN_FREQ);
      return -EINVAL;
    }

  /* Calculate the MAX and CMP register value
   * period = max_reg / g_pwmclk
   * freq   = g_pwmclk / max_reg
   */

  max = (uint16_t)((g_pwmclk / info->frequency));

  cmp = (uint16_t)(((uint64_t)max * (uint64_t)info->duty) >> 16);

  pwminfo("PWM%d PCLK: %lu freq: %lu duty: %lu max: %u cmp: %u\n",
          priv->id, g_pwmclk, info->frequency, info->duty, max, cmp);

  priv->max = max;
  priv->cmp = cmp;

  PWM_MAX_REG(priv->id) = priv->max;
  PWM_CMP_REG(priv->id) = priv->cmp;

#ifdef CONFIG_TLSR82_PWM0_PULSECOUNT
  if (priv->count > 0)
    {
      PWM_MODE0_REG = PWM_MODE0_COUNT;
      PWM_PLUSE_NUM_REG = priv->count;
      PWM0_PNUM_INT_ENABLE();
      up_enable_irq(priv->irq);
    }
  else
    {
      PWM_MODE0_REG = PWM_MODE0_NORMAL;
    }
#endif

  pwm_enable(priv, true);

  return OK;
}

/****************************************************************************
 * Name: pwm_interrupt
 *
 * Description:
 *   Handle timer interrupts.
 *
 * Input Parameters:
 *   priv - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

#ifdef CONFIG_TLSR82_PWM0_PULSECOUNT
static int pwm_interrupt(int irq, void *context, void *arg)
{
  struct tlsr82_pwmtimer_s *priv = (struct tlsr82_pwmtimer_s *)arg;

  if (PWM0_PNUM_INT_PEND() && PWM0_PNUM_INT_ISENABLE())
    {
      /* Disable first interrupts, stop and reset the timer */

      pwm_enable(priv, false);

      /* Disable PWMn interrupt and clear interrupt flag */

      PWM_INT_DISABLE(priv->id);
      PWM_INT_CLEAR(priv->id);

      /* Then perform the callback into the upper half driver */

      pwm_expired(priv->handle);

      priv->handle = NULL;
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: pwm_enable
 *
 * Description:
 *   Enable or disable pwm
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *   en  - Enable clock if 'en' is 'true' and disable if 'false'
 *
 ****************************************************************************/

static void pwm_enable(struct tlsr82_pwmtimer_s *priv, bool en)
{
  int id = (int)priv->id;

  if (en)
    {
      if (priv->started)
        {
          return;
        }

      if (id == 0)
        {
          BM_SET(PWM_ENABLE0_REG, 1);
        }
      else
        {
          BM_SET(PWM_ENABLE_REG, 1 << id);
        }

      priv->started = true;
    }
  else
    {
      if (!priv->started)
        {
          return;
        }

      if (id == 0)
        {
          BM_CLR(PWM_ENABLE0_REG, 1);
        }
      else
        {
          BM_CLR(PWM_ENABLE_REG, 1 << id);
        }

      priv->started = false;
    }
}

/****************************************************************************
 * Name: pwm_cfg_check
 *
 * Description:
 *   This method is called when the driver intialize. This function will
 *   check the pincfg, if pincfg is not valid or current pin can not be used
 *   as PWM, this function will call
 *   PANIC() to assert here.
 *
 * Input Parameters:
 *   dev    - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 *
 ****************************************************************************/

static int pwm_cfg_check(struct tlsr82_pwmtimer_s *priv)
{
  uint32_t pincfg = priv->pincfg;

  if (pincfg == GPIO_INVLD_CFG)
    {
      pwmerr("pwm pincfg is not valid, pincfg=0x%lx\n", pincfg);
      PANIC();
      return -EINVAL;
    }

  if (tlsr82_gpio_cfg_check(pincfg, TLSR82_MUX_PWM) != 0)
    {
      pwmerr("pincfg=0x%lx not support pwm mux\n", pincfg);
      PANIC();
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
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
 ****************************************************************************/

static int pwm_setup(struct pwm_lowerhalf_s *dev)
{
  struct tlsr82_pwmtimer_s *priv = (struct tlsr82_pwmtimer_s *)dev;
  uint32_t pincfg;
  int ret = OK;

  pwm_dumpregs("Initially");

  /* Configure the PWM output pins, but do not start the timer yet */

  pincfg = priv->pincfg;

  pwminfo("pincfg: %08lx\n", pincfg);

  /* Set the pwm_id_n be inverted to unify the pwm duty config, this
   * configuration must precede the gpio configuration, otherwise, the
   * gpio output is high between the invert and gpio config, and become
   * low after the invert configuration.
   */

  if (priv->invert)
    {
      BM_SET(PWM_N_INVERT_REG, 1 << priv->id);
    }

#ifdef CONFIG_TLSR82_PWM0_PULSECOUNT
  if (priv->id == 0)
    {
      ret = irq_attach(priv->irq, pwm_interrupt, dev);
      if (ret < 0)
        {
          pwmerr("ERROR: PWM0 plusecount interrupt attach failed, ret=%d\n",
                  ret);
          return ret;
        }
    }
#endif

  tlsr82_gpioconfig(pincfg);

  return ret;
}

/****************************************************************************
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
 ****************************************************************************/

static int pwm_shutdown(struct pwm_lowerhalf_s *dev)
{
  struct tlsr82_pwmtimer_s *priv = (struct tlsr82_pwmtimer_s *)dev;

  /* Make sure that the output has been stopped */

  pwm_stop(dev);

  /* Then put the GPIO pins back to the default state */

  tlsr82_gpiounconfig(priv->pincfg);

  return OK;
}

/****************************************************************************
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
 ****************************************************************************/

#ifdef CONFIG_PWM_PULSECOUNT
static int pwm_start(struct pwm_lowerhalf_s *dev,
                     const struct pwm_info_s *info,
                     void *handle)
#else
static int pwm_start(struct pwm_lowerhalf_s *dev,
                     const struct pwm_info_s *info)
#endif
{
  int ret = OK;
  struct tlsr82_pwmtimer_s *priv = (struct tlsr82_pwmtimer_s *)dev;

#ifdef CONFIG_PWM_PULSECOUNT

  /* Check if a pulsecount has been selected */

  if (info->count > 0)
    {
      /* Only the PWM0 support the pulse counting */

      if (priv->id != 0)
        {
          pwmerr("ERROR: PWM%d cannot support pulse count: %lu\n",
                 priv->id, info->count);
          return -EPERM;
        }

      if (info->count > PWM_MAX_COUNT)
        {
          pwmerr("ERROR: PWM0 count out of range, count: %lu, max: %d",
                info->count, PWM_MAX_COUNT);
          return -EINVAL;
        }

      priv->count = info->count;
    }

  /* Save the handle */

  priv->handle = handle;

#endif

  /* Config the PWM */

  ret = pwm_config(priv, info);

  /* Save current frequency */

  if (ret == OK)
    {
      priv->frequency = info->frequency;
    }

  return ret;
}

/****************************************************************************
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
 ****************************************************************************/

static int pwm_stop(struct pwm_lowerhalf_s *dev)
{
  struct tlsr82_pwmtimer_s *priv = (struct tlsr82_pwmtimer_s *)dev;
  irqstate_t flags;

  pwminfo("PWM%u\n", priv->id);

  /* Disable interrupts momentary to stop any ongoing pwm processing and
   * to prevent any concurrent access to the reset register.
   */

  flags = enter_critical_section();

  /* Disable PWMn output */

  pwm_enable(priv, false);

  /* Disable PWMn interrupt and clear interrupt flag */

  PWM_INT_DISABLE(priv->id);
  PWM_INT_CLEAR(priv->id);

#ifdef CONFIG_TLSR82_PWM0_PULSECOUNT
  /* Diable and clear PWM0 count interrupt flag */

  if (priv->count > 0)
    {
      priv->count = 0;
      up_disable_irq(priv->irq);
      PWM0_PNUM_INT_DISABLE();
      PWM0_PNUM_INT_CLEAR();
    }
#endif

  leave_critical_section(flags);

  pwm_dumpregs("After stop");

  return OK;
}

/****************************************************************************
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
 ****************************************************************************/

static int pwm_ioctl(struct pwm_lowerhalf_s *dev, int cmd,
                     unsigned long arg)
{
#ifdef CONFIG_DEBUG_PWM_INFO
  struct tlsr82_pwmtimer_s *priv = (struct tlsr82_pwmtimer_s *)dev;

  /* There are no platform-specific ioctl commands */

  pwminfo("PWM%d\n", priv->id);
#endif
  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tlsr82_pwminitialize
 *
 * Description:
 *   Initialize one timer for use with the upper_level PWM driver.
 *
 * Input Parameters:
 *   timer - A number identifying the timer use.  The number of valid timer
 *     IDs varies with the STM32 MCU and MCU family but is somewhere in
 *     the range of {1,..,17}.
 *
 * Returned Value:
 *   On success, a pointer to the STM32 lower half PWM driver is returned.
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

int tlsr82_pwminitialize(const char *devpath, int miror)
{
  struct tlsr82_pwmtimer_s *lower;
  int ret = OK;

  static bool pwm_initialized = false;

  if (!pwm_initialized)
    {
      pwm_initialized = true;

      /* Disable all the output */

      BM_CLR(PWM_ENABLE0_REG, 0xff);
      BM_CLR(PWM_ENABLE_REG, 0xff);

      /* Config the pwm clock */

      g_pwmclk = PWM_SRC_CLK_HZ / (PWM_INIT_CLKDIV + 1);
      PWM_CLKDIV_REG = PWM_INIT_CLKDIV;

      /* Disable all the pwm interrupt and clear all the interrupt flags */

      BM_CLR(PWM_IRQ_CTRL_REG, 0xff);
      BM_SET(PWM_IRQ_STA_REG, 0xff);

      /* Enable the pwm total interrupt */

      up_enable_irq(NR_SW_PWM_IRQ);
    }

  pwminfo("PWM%u\n", miror);

  switch (miror)
    {
#ifdef CONFIG_TLSR82_PWM0
      case 0:
        lower = &g_pwm0dev;
        break;
#endif

#ifdef CONFIG_TLSR82_PWM1
      case 1:
        lower = &g_pwm1dev;
        break;
#endif

#ifdef CONFIG_TLSR82_PWM2
      case 2:
        lower = &g_pwm2dev;
        break;
#endif

#ifdef CONFIG_TLSR82_PWM3
      case 3:
        lower = &g_pwm3dev;
        break;
#endif

#ifdef CONFIG_TLSR82_PWM4
      case 4:
        lower = &g_pwm4dev;
        break;
#endif

#ifdef CONFIG_TLSR82_PWM5
      case 5:
        lower = &g_pwm5dev;
        break;
#endif

      default:
        pwmerr("ERROR: No such pwm configured\n");
        return -EINVAL;
    }

  ret = pwm_cfg_check(lower);
  if (ret < 0)
    {
      return ret;
    }

  ret = pwm_register(devpath, (struct pwm_lowerhalf_s *)lower);
  if (ret < 0)
    {
      pwmerr("%s has existed\n", devpath);
      return ret;
    }

  pwm_dumpregs("tlsr82_pwminitialize");

  return ret;
}
