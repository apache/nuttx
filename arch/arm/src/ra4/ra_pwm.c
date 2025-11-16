/****************************************************************************
 * arch/arm/src/ra4/ra_pwm.c
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
#include "ra_gpio.h"
#include "ra_pwm.h"
#include "hardware/ra_gpt.h"
#include "hardware/ra_mstp.h"
#include "hardware/ra_system.h"

#ifdef CONFIG_RA_PWM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PWM output frequency */
#define RA_PWM_CLOCK_FREQ  RA_PCKA_FREQUENCY

/* Number of GPT channels available on RA4M1 */
#define RA_PWM_NUM_CHANNELS 8

typedef enum
{
  R_GPT16 = 0,
  R_GPT32,
} ra_gpt_type_t;

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure represents the state of one PWM timer */

struct ra_pwmtimer_s
{
  const struct pwm_ops_s *ops;          /* PWM operations */
  uint8_t                 channel;      /* Timer channel: 0-7 */
  ra_gpt_type_t           gtp;          /* GPT type */
  uint32_t                base;         /* Base address of GPT registers */
  uint32_t                mstp;         /* Module stop bit */
  uint32_t                pclk;         /* Input clock frequency */
  uint32_t                frequency;    /* Current frequency setting */
  uint32_t                duty;         /* Current duty cycle setting */
  gpio_pinset_t           gtioa;        /* GPIO pin configuration for GTIOA */
  gpio_pinset_t           gtiob;        /* GPIO pin configuration for GTIOB */
};

/****************************************************************************
 * Static Function Prototypes
 ****************************************************************************/

/* PWM driver methods */

static int pwm_setup(struct pwm_lowerhalf_s *dev);
static int pwm_shutdown(struct pwm_lowerhalf_s *dev);
static int pwm_start(struct pwm_lowerhalf_s *dev,
                        const struct pwm_info_s *info);
static int pwm_stop(struct pwm_lowerhalf_s *dev);
static int pwm_ioctl(struct pwm_lowerhalf_s *dev, int cmd,
                        unsigned long arg);
#ifdef CONFIG_DEBUG_PWM_INFO
static void pwm_dumpregs(struct pwm_lowerhalf_s *dev,
                         const char *msg);
#else
#  define pwm_dumpregs(priv,msg)
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* PWM operations */

static const struct pwm_ops_s g_pwmops =
{
  .setup      = pwm_setup,
  .shutdown   = pwm_shutdown,
  .start      = pwm_start,
  .stop       = pwm_stop,
  .ioctl      = pwm_ioctl
};

#ifdef CONFIG_RA_GPT0_PWM
static struct ra_pwmtimer_s g_pwm0dev =
{
  .ops        = &g_pwmops,
  .channel    = 0,
  .gtp        = R_GPT32,
  .base       = R_GPT0_BASE,
  .mstp       = R_MSTP_MSTPCRD_GPT_32,
  .pclk       = RA_PWM_CLOCK_FREQ,
  #ifdef CONFIG_RA_GPT0_OUTPUTA
  .gtioa    = GPIO_GPT0_GTIOCA,
  #else
  .gtioa    = GPIO_PIN_INVALID,
  #endif
  #ifdef CONFIG_RA_GPT0_OUTPUTB
  .gtiob    = GPIO_GPT0_GTIOCB,
  #else
  .gtiob    = GPIO_PIN_INVALID,
  #endif
};
#endif

#ifdef CONFIG_RA_GPT1_PWM
static struct ra_pwmtimer_s g_pwm1dev =
{
  .ops        = &g_pwmops,
  .channel    = 1,
  .gtp        = R_GPT32,
  .base       = R_GPT1_BASE,
  .mstp       = R_MSTP_MSTPCRD_GPT_32,
  .pclk       = RA_PWM_CLOCK_FREQ,
  #ifdef CONFIG_RA_GPT1_OUTPUTA
  .gtioa    = GPIO_GPT1_GTIOCA,
  #else
  .gtiob    = GPIO_PIN_INVALID,
  #endif
  #ifdef CONFIG_RA_GPT1_OUTPUTB
  .gtiob    = GPIO_GPT1_GTIOCB,
  #else
  .gtiob    = GPIO_PIN_INVALID,
  #endif
};
#endif

#ifdef CONFIG_RA_GPT2_PWM
static struct ra_pwmtimer_s g_pwm2dev =
{
  .ops        = &g_pwmops,
  .channel    = 2,
  .gtp        = R_GPT16,
  .base       = R_GPT2_BASE,
  .mstp       = R_MSTP_MSTPCRD_GPT_16,
  .pclk       = RA_PWM_CLOCK_FREQ,
  #ifdef CONFIG_RA_GPT2_OUTPUTA
  .gtioa    = GPIO_GPT2_GTIOCA,
  #endif
  #ifdef CONFIG_RA_GPT2_OUTPUTB
  .gtiob    = GPIO_GPT2_GTIOCB,
  #endif
};
#endif

#ifdef CONFIG_RA_GPT3_PWM
static struct ra_pwmtimer_s g_pwm3dev =
{
  .ops        = &g_pwmops,
  .channel    = 3,
  .gtp        = R_GPT16,
  .base       = R_GPT3_BASE,
  .mstp       = R_MSTP_MSTPCRD_GPT_16,
  .pclk       = RA_PWM_CLOCK_FREQ,
  #ifdef CONFIG_RA_GPT3_OUTPUTA
  .gtioa    = GPIO_GPT3_GTIOCA,
  #endif
  #ifdef CONFIG_RA_GPT3_OUTPUTB
  .gtiob    = GPIO_GPT3_GTIOCB,
  #endif
};
#endif

#ifdef CONFIG_RA_GPT4_PWM
static struct ra_pwmtimer_s g_pwm4dev =
{
  .ops        = &g_pwmops,
  .channel    = 4,
  .gtp        = R_GPT16,
  .base       = R_GPT4_BASE,
  .mstp       = R_MSTP_MSTPCRD_GPT_16,
  .pclk       = RA_PWM_CLOCK_FREQ,
  #ifdef CONFIG_RA_GPT4_OUTPUTA
  .gtioa    = GPIO_GPT4_GTIOCA,
  #endif
  #ifdef CONFIG_RA_GPT4_OUTPUTB
  .gtiob    = GPIO_GPT4_GTIOCB,
  #endif
};
#endif

#ifdef CONFIG_RA_GPT5_PWM
static struct ra_pwmtimer_s g_pwm5dev =
{
  .ops        = &g_pwmops,
  .channel    = 5,
  .gtp        = R_GPT16,
  .base       = R_GPT5_BASE,
  .mstp       = R_MSTP_MSTPCRD_GPT_16,
  .pclk       = RA_PWM_CLOCK_FREQ,
  #ifdef CONFIG_RA_GPT5_OUTPUTA
  .gtioa    = GPIO_GPT5_GTIOCA,
  #endif
  #ifdef CONFIG_RA_GPT5_OUTPUTB
  .gtiob    = GPIO_GPT5_GTIOCB,
  #endif
};
#endif

#ifdef CONFIG_RA_GPT6_PWM
static struct ra_pwmtimer_s g_pwm6dev =
{
  .ops        = &g_pwmops,
  .channel    = 6,
  .gtp        = R_GPT16,
  .base       = R_GPT6_BASE,
  .mstp       = R_MSTP_MSTPCRD_GPT_16,
  .pclk       = RA_PWM_CLOCK_FREQ,
  #ifdef CONFIG_RA_GPT6_OUTPUTA
  .gtioa    = GPIO_GPT6_GTIOCA,
  #endif
  #ifdef CONFIG_RA_GPT6_OUTPUTB
  .gtiob    = GPIO_GPT6_GTIOCB,
  #endif
};
#endif

#ifdef CONFIG_RA_GPT7_PWM
static struct ra_pwmtimer_s g_pwm7dev =
{
  .ops        = &g_pwmops,
  .channel    = 7,
  .base       = R_GPT7_BASE,
  .gtp        = R_GPT16,
  .mstp       = R_MSTP_MSTPCRD_GPT_16,
  .pclk       = RA_PWM_CLOCK_FREQ,
  #ifdef CONFIG_RA_GPT7_OUTPUTA
  .gtioa    = GPIO_GPT7_GTIOCA,
  #endif
  #ifdef CONFIG_RA_GPT7_OUTPUTB
  .gtiob    = GPIO_GPT7_GTIOCB,
  #endif
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pwm_getreg
 *
 * Description:
 *   Read the value of a PWM timer register
 *
 * Input Parameters:
 *   priv - A reference to the PWM timer state structure
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *   The current contents of the specified register
 *
 ****************************************************************************/

static uint32_t pwm_getreg(struct ra_pwmtimer_s *priv, uint32_t offset)
{
  return getreg32(priv->base + offset);
}

/****************************************************************************
 * Name: pwm_putreg
 *
 * Description:
 *   Write a value to a PWM timer register
 *
 * Input Parameters:
 *   priv - A reference to the PWM timer state structure
 *   offset - The offset to the register to write
 *   value - The value to write to the register
 *
 ****************************************************************************/

static void pwm_putreg(struct ra_pwmtimer_s *priv, uint32_t offset,
                          uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: pwm_dumpregs
 *
 * Description:
 *   Dump all timer registers.
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_PWM_INFO
static void pwm_dumpregs(struct pwm_lowerhalf_s *dev, const char *msg)
{
  struct ra_pwmtimer_s *priv = (struct ra_pwmtimer_s *)dev;

  pwminfo("PWM: %s\n", msg);
  pwminfo("   GTCR: %08x \n",
    pwm_getreg(priv, R_GPT_GTCR_OFFSET));
  pwminfo("   GTIOR: %08x \n",
      pwm_getreg(priv, R_GPT_GTIOR_OFFSET));
  pwminfo("   GTPR: %08x \n",
      pwm_getreg(priv, R_GPT_GTPR_OFFSET));
  pwminfo("   GTCCRA: %08x \n",
      pwm_getreg(priv, R_GPT_GTCCRA_OFFSET));
  pwminfo("   GTCCRB: %08x \n",
      pwm_getreg(priv, R_GPT_GTCCRB_OFFSET));
}
#endif

/****************************************************************************
 * Name: pwm_setup
 *
 * Description:
 *   This method is called when the driver is opened. The lower half driver
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
  struct ra_pwmtimer_s *priv = (struct ra_pwmtimer_s *)dev;
  uint32_t regval = 0;

  /* Enable clock to the GPT module */

  putreg16((R_SYSTEM_PRCR_PRKEY_VALUE | R_SYSTEM_PRCR_PRC1), R_SYSTEM_PRCR);
  modifyreg32(R_MSTP_MSTPCRD, priv->mstp, 0);
  putreg16(R_SYSTEM_PRCR_PRKEY_VALUE, R_SYSTEM_PRCR);

  /* Reset the timer */

  pwm_putreg(priv, R_GPT_GTCR_OFFSET, 0);

  /* Configure the timer for PWM mode */

  regval &= ~R_GPT_GTCR_CST;
  regval |= R_GPT_GTCR_MD_MOD0;
  pwm_putreg(priv, R_GPT_GTCR_OFFSET, regval);

  /* Set default frequency and duty cycle */

  priv->frequency = 0;
  priv->duty = 0;

  /* Configure selected PWM pins */

  regval = pwm_getreg(priv, R_GPT_GTIOR_OFFSET);
  if (priv->gtioa.port != PORT_INVALID)
    {
      regval &= ~R_GPT_GTIOR_GTIOA_MASK;
      regval |= (R_GPT_GTIOR_GTIOA_SET25 | R_GPT_GTIOR_OAE);
      ra_configgpio(priv->gtioa);
    }

  if (priv->gtiob.port != PORT_INVALID)
    {
      regval &= ~R_GPT_GTIOR_GTIOB_MASK;
      regval |= (R_GPT_GTIOR_GTIOB_SET25 | R_GPT_GTIOR_OBE);
      ra_configgpio(priv->gtiob);
    }

  pwm_putreg(priv, R_GPT_GTIOR_OFFSET, regval);

  pwm_dumpregs(dev, "After setup");
  return OK;
}

/****************************************************************************
 * Name: pwm_shutdown
 *
 * Description:
 *   This method is called when the driver is closed. The lower half driver
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
  struct ra_pwmtimer_s *priv = (struct ra_pwmtimer_s *)dev;

  /* Stop the timer */

  pwm_putreg(priv, R_GPT_GTCR_OFFSET, 0);

  /* Disable the output */

  pwm_putreg(priv, R_GPT_GTIOR_OFFSET, 0);

  /* Disable clock to the GPT module */

  putreg16((R_SYSTEM_PRCR_PRKEY_VALUE | R_SYSTEM_PRCR_PRC1), R_SYSTEM_PRCR);
  modifyreg32(R_MSTP_MSTPCRD, 0, priv->mstp);
  putreg16(R_SYSTEM_PRCR_PRKEY_VALUE, R_SYSTEM_PRCR);
  pwm_dumpregs(dev, "After shutdown");
  return OK;
}

/****************************************************************************
 * Name: pwm_start
 *
 * Description:
 *   Start the pulsed output with the given frequency and duty cycle. This
 *   method is called after setup() when the PWM device is opened. This
 *   method is called after stop() when a new pulse train is started.
 *
 * Input Parameters:
 *   dev  - A reference to the lower half PWM driver state structure
 *   info - A reference to the characteristics of the pulsed output
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pwm_start(struct pwm_lowerhalf_s *dev,
                        const struct pwm_info_s *info)
{
  struct ra_pwmtimer_s *priv = (struct ra_pwmtimer_s *)dev;
  uint32_t period;
  uint32_t duty;
  uint32_t regval;
  uint32_t hi;
  uint32_t lo;
  uint32_t duty16;

  /* Calculate the period and duty cycle register values */

  if (info->frequency == 0)
    {
      return -EINVAL;
    }

  /* Calculate the period register value */

  period = priv->pclk / info->frequency;
  if (period < 2 || period > UINT32_MAX)
    {
      return -EINVAL;
    }

  if (period > UINT16_MAX && priv->gtp == R_GPT16)
    {
      uint8_t prescaler = 0;

      /* Try to find a suitable prescaler */

      while (prescaler < 6)
        {
          prescaler++;
          switch (prescaler)
            {
              case 1:
                period = priv->pclk / 4 / info->frequency;
                break;
              case 2:
                period = priv->pclk / 16 / info->frequency;
                break;
              case 3:
                period = priv->pclk / 64 / info->frequency;
                break;
              case 4:
                period = priv->pclk / 256 / info->frequency;
                break;
              case 5:
                period = priv->pclk / 1024 / info->frequency;
                break;
            }

          if (period <= UINT16_MAX)
            {
              break;
            }
        }

      if (prescaler == 6)
        {
          return -EINVAL;
        }

      /* Set the prescaler */

      regval = pwm_getreg(priv, R_GPT_GTCR_OFFSET);
      regval &= ~R_GPT_GTCR_TPCS_MASK;
      switch (prescaler)
        {
          case 1:
            regval |= R_GPT_GTCR_TPCS_DIV4;
            break;
          case 2:
            regval |= R_GPT_GTCR_TPCS_DIV16;
            break;
          case 3:
            regval |= R_GPT_GTCR_TPCS_DIV64;
            break;
          case 4:
            regval |= R_GPT_GTCR_TPCS_DIV256;
            break;
          case 5:
            regval |= R_GPT_GTCR_TPCS_DIV1024;
            break;
        }

      pwminfo("prescaler=%" PRIu32 "\n",
            regval);
      pwm_putreg(priv, R_GPT_GTCR_OFFSET, regval);
    }

  /* Calculate the duty cycle register value.
   * info->duty is a 16-bit value (0..65535). To avoid 64-bit math we
   * can split the 32-bit period into high and low 16-bit parts
   */

  hi = period >> 16;
  lo = period & 0xffff;
  duty16 = (uint32_t)info->duty;

  duty = hi * duty16;

  /* Add the scaled low part. Use truncating shift; add (1<<15) to round. */

  duty += ((lo * duty16) >> 16);

  if (duty >= period)
    {
      duty = period - 1;
    }

  /* Save the frequency and duty cycle settings */

  priv->frequency = info->frequency;
  priv->duty = info->duty;

  /* Stop the timer */

  regval = pwm_getreg(priv, R_GPT_GTCR_OFFSET);
  regval &= ~R_GPT_GTCR_CST;
  pwm_putreg(priv, R_GPT_GTCR_OFFSET, regval);

  /* Set the period and duty cycle */

  pwm_putreg(priv, R_GPT_GTPR_OFFSET, period);
  regval = pwm_getreg(priv, R_GPT_GTIOR_OFFSET);
  if (regval & R_GPT_GTIOR_OAE)
    {
      pwm_putreg(priv, R_GPT_GTCCRA_OFFSET, duty - 1);
    }

  if (regval & R_GPT_GTIOR_OBE)
    {
      pwm_putreg(priv, R_GPT_GTCCRB_OFFSET, duty - 1);
    }

  /* Start the timer */

  regval = pwm_getreg(priv, R_GPT_GTCR_OFFSET);
  regval |= R_GPT_GTCR_CST;
  pwm_putreg(priv, R_GPT_GTCR_OFFSET, regval);
  pwm_dumpregs(dev, "After start");
  return OK;
}

/****************************************************************************
 * Name: pwm_stop
 *
 * Description:
 *   Stop the pulsed output and reset the timer resources. This method is
 *   called when the PWM device is closed. This method is also called when a
 *   new PWM output is started to stop the previous output before starting
 *   the new one.
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pwm_stop(struct pwm_lowerhalf_s *dev)
{
  struct ra_pwmtimer_s *priv = (struct ra_pwmtimer_s *)dev;
  uint32_t regval;

  /* Stop the timer */

  regval = pwm_getreg(priv, R_GPT_GTCR_OFFSET);
  regval &= ~R_GPT_GTCR_CST;
  pwm_putreg(priv, R_GPT_GTCR_OFFSET, regval);
  pwm_dumpregs(dev, "After stop");
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
  /* No ioctl commands supported */

  pwm_dumpregs(dev, "After ioctl");
  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_pwminitialize
 *
 * Description:
 *   Initialize one PWM channel for use with the upper_level PWM driver.
 *
 * Input Parameters:
 *   channel - A number identifying the PWM channel use.
 *
 * Returned Value:
 *   On success, a pointer to the RA PWM lower half driver is returned.
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

struct pwm_lowerhalf_s *ra_pwminitialize(int channel)
{
  struct ra_pwmtimer_s *lower;

  /* Find the requested channel */

  printf("pwminitialize passed\n");
  switch (channel)
    {
#ifdef CONFIG_RA_GPT0_PWM
      case 0:
        lower = &g_pwm0dev;
        break;
#endif

#ifdef CONFIG_RA_GPT1_PWM
      case 1:
        lower = &g_pwm1dev;
        break;
#endif

#ifdef CONFIG_RA_GPT2_PWM
      case 2:
        lower = &g_pwm2dev;
        break;
#endif

#ifdef CONFIG_RA_GPT3_PWM
      case 3:
        lower = &g_pwm3dev;
        break;
#endif

#ifdef CONFIG_RA_GPT4_PWM
      case 4:
        lower = &g_pwm4dev;
        break;
#endif

#ifdef CONFIG_RA_GPT5_PWM
      case 5:
        lower = &g_pwm5dev;
        break;
#endif

#ifdef CONFIG_RA_GPT6_PWM
      case 6:
        lower = &g_pwm6dev;
        break;
#endif

#ifdef CONFIG_RA_GPT7_PWM
      case 7:
        lower = &g_pwm7dev;
        break;
#endif

      default:
        pwmerr("ERROR: No such PWM channel: %d\n", channel);
        return NULL;
    }

  return (struct pwm_lowerhalf_s *)lower;
}
#endif /* CONFIG_RA_PWM */
