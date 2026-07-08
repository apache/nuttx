/****************************************************************************
 * arch/arm64/src/bcm2711/bcm2711_pwm.c
 *
 * Contributed by: Matteo Golin <linguini@apache.org>
 *
 * SPDX-License-Identifer: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements. See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership. The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
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

#include <nuttx/mutex.h>
#include <nuttx/timers/pwm.h>

#include "bcm2711_gpio.h"
#include "bcm2711_mailbox.h"
#include "bcm2711_pwm.h"

#include "hardware/bcm2711_gpclk.h"
#include "hardware/bcm2711_pwm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PWM0 has several options for the channels. */

#if CONFIG_BCM2711_PWM0_CHAN1_PIN == 12
#define BCM2711_PWM0_CHAN1_PIN_ALT BCM_GPIO_FUNC0
#elif CONFIG_BCM2711_PWM0_CHAN1_PIN == 18
#define BCM2711_PWM0_CHAN1_PIN_ALT BCM_GPIO_FUNC5
#else
#error "Invalid pin selection for PWM0 channel 1!"
#endif

#if CONFIG_BCM2711_PWM0_CHAN2_PIN == 13
#define BCM2711_PWM0_CHAN2_PIN_ALT BCM_GPIO_FUNC0
#elif CONFIG_BCM2711_PWM0_CHAN2_PIN == 19
#define BCM2711_PWM0_CHAN2_PIN_ALT BCM_GPIO_FUNC5
#elif CONFIG_BCM2711_PWM0_CHAN2_PIN == 45
#define BCM2711_PWM0_CHAN2_PIN_ALT BCM_GPIO_FUNC0
#else
#error "Invalid pin selection for PWM0 channel 2!"
#endif

/* PWM1 only has one option for the channels */

#define BCM2711_PWM1_CHAN1_PIN (40)
#define BCM2711_PWM1_CHAN1_PIN_ALT BCM_GPIO_FUNC0
#define BCM2711_PWM1_CHAN2_PIN (41)
#define BCM2711_PWM1_CHAN2_PIN_ALT BCM_GPIO_FUNC0

/* PWM clock runs at 54MHz */

#define PWM_CLKFREQ (54000000)

/* GPCLK for PWM */

#define PWM_GPCLKCTL (BCM_GPCLK_BASEADDR + 0xa0)
#define PWM_GPCLKDIV (BCM_GPCLK_BASEADDR + 0xa4)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct bcm2711_pwm_s
{
  struct pwm_lowerhalf_s dev;
  mutex_t lock;  /* We lock both channels at once, no granularity */
  uint32_t base; /* Interface base address */
  uint8_t port;  /* Interface number */
  bool init;     /* Initialized yet? */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int bcm2711_pwm_setup(FAR struct pwm_lowerhalf_s *dev);
static int bcm2711_pwm_shutdown(FAR struct pwm_lowerhalf_s *dev);
static int bcm2711_pwm_start(FAR struct pwm_lowerhalf_s *dev,
                             FAR const struct pwm_info_s *info);
static int bcm2711_pwm_stop(FAR struct pwm_lowerhalf_s *dev);
static int bcm2711_pwm_ioctl(FAR struct pwm_lowerhalf_s *dev, int cmd,
                             unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct pwm_ops_s g_pwmops =
{
  .setup = bcm2711_pwm_setup,
  .shutdown = bcm2711_pwm_shutdown,
  .start = bcm2711_pwm_start,
  .stop = bcm2711_pwm_stop,
  .ioctl = bcm2711_pwm_ioctl,
};

#ifdef CONFIG_BCM2711_PWM0
struct bcm2711_pwm_s g_pwm0 =
{
  .dev =
  {
    .ops = &g_pwmops,
  },
  .lock = NXMUTEX_INITIALIZER,
  .base = BCM_PWM0_BASEADDR,
  .port = 0,
  .init = false,
};
#endif /* CONFIG_BCM2711_PWM0 */

#ifdef CONFIG_BCM2711_PWM1
struct bcm2711_pwm_s g_pwm1 =
{
  .dev =
    {
      .ops = &g_pwmops,
    },
  .lock = NXMUTEX_INITIALIZER,
  .base = BCM_PWM1_BASEADDR,
  .port = 1,
  .init = false,
};
#endif /* CONFIG_BCM2711_PWM1 */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bcm2711_channel_pol
 *
 * Description:
 *   Set the polarity of a PWM channel. NOTE: CPOL and DCPOL settings have
 *   the same values, so this function works for both.
 *
 * Input Parameters:
 *   priv - The PWM device reference
 *   mask - The mask for the channel's POLA register
 *   pol - The PWM upper-half driver polarity setting
 *
 ****************************************************************************/

static void bcm2711_channel_pol(struct bcm2711_pwm_s *priv, uint32_t mask,
                                uint8_t pol)
{
  uint32_t val = 0; /* Default: logical high */

  if (pol == PWM_CPOL_LOW)
    {
      val = 0xffffffff; /* Logical low (this will be masked) */
    }

  modreg32(val, mask, BCM_PWM_CTL(priv->base));
}

/****************************************************************************
 * Name: bcm2711_setpulse
 *
 * Description:
 *   Set the duty cycle and frequency of the PWM interface
 *
 * Input Parameters:
 *   rangereg - The RNG register of the PWM channel in which to set the duty
 *   datreg - The DAT register of the PWM channel in which to set the duty
 *   duty - The duty value from the upper-half driver
 *   frequency - The frequency of the pulse in Hz
 *
 ****************************************************************************/

static void bcm2711_setpulse(uint32_t rangereg, uint32_t datreg, ub16_t duty,
                             uint32_t freq)
{
  uint32_t val = PWM_CLKFREQ / freq;

  putreg32(val, rangereg);

  /* Multiply the duty fractional amount by the frequency we set in our
   * range.
   */

  val = ((uint64_t)val * (uint64_t)duty) / 0xffff;
  pwminfo("Range: %08x\n", val);
  putreg32(val, datreg);
}

/****************************************************************************
 * Name: bcm2711_pwm_setup
 *
 * Description:
 *   This method is called when the driver is opened. The lower half driver
 *   should configure and initialize the device so that it is ready for use.
 *   It should not, however, output pulses until the start method is called.
 *
 * Input Parameters:
 *   dev - The device reference
 *
 * Returned Value:
 *   0 on success, negated errno on failure.
 *
 ****************************************************************************/

static int bcm2711_pwm_setup(FAR struct pwm_lowerhalf_s *dev)
{
  struct bcm2711_pwm_s *priv = (struct bcm2711_pwm_s *)dev;
  uint32_t errmask = BCM_PWM_STA_BERR | BCM_PWM_STA_GAPO2 |
                     BCM_PWM_STA_GAPO1 | BCM_PWM_STA_RERR1 |
                     BCM_PWM_STA_WERR1;

  nxmutex_lock(&priv->lock);

  /* Ensure channels are disabled so there is no output started */

  modreg32(0, BCM_PWM_CTL_PWEN1, BCM_PWM_CTL(priv->base));
  modreg32(0, BCM_PWM_CTL_PWEN2, BCM_PWM_CTL(priv->base));

  /* Clear all errors */

  putreg32(errmask, BCM_PWM_STA(priv->base));

  /* Put channels in PWM mode */

  modreg32(0, BCM_PWM_CTL_MODE1, BCM_PWM_CTL(priv->base));
  modreg32(0, BCM_PWM_CTL_MODE2, BCM_PWM_CTL(priv->base));

  /* Use marked-space mode */

  modreg32(BCM_PWM_CTL_MSEN1, BCM_PWM_CTL_MSEN1, BCM_PWM_CTL(priv->base));
  modreg32(BCM_PWM_CTL_MSEN2, BCM_PWM_CTL_MSEN2, BCM_PWM_CTL(priv->base));

  /* The way NuttX's PWM interface is set up, we should never have to use
   * FIFO mode. This is because the PWM interface seems to be intended to
   * just directly set a duty cycle that runs until stopped, not output a
   * data stream. Hence, we disable FIFO usage.
   */

  modreg32(0, BCM_PWM_CTL_USEF1, BCM_PWM_CTL(priv->base));
  modreg32(0, BCM_PWM_CTL_USEF2, BCM_PWM_CTL(priv->base));
  nxmutex_unlock(&priv->lock);
  return 0;
}

/****************************************************************************
 * Name: bcm2711_pwm_shutdown
 *
 * Description:
 *   This method is called when the driver is closed. The lower half driver
 *   should stop pulsed output, free any resources, disable the timer
 *   hardware, and put the system into the lowest possible power usage state
 *
 * Input Parameters:
 *   dev - The device reference
 *
 * Returned Value:
 *   0 on success, negated errno on failure.
 *
 ****************************************************************************/

static int bcm2711_pwm_shutdown(FAR struct pwm_lowerhalf_s *dev)
{
  struct bcm2711_pwm_s *priv = (struct bcm2711_pwm_s *)dev;

  nxmutex_lock(&priv->lock);

  /* Clear FIFOs */

  modreg32(BCM_PWM_CTL_CLRF, BCM_PWM_CTL_CLRF, BCM_PWM_CTL(priv->base));

  /* Ensure channels are disabled */

  modreg32(0, BCM_PWM_CTL_PWEN1, BCM_PWM_CTL(priv->base));
  modreg32(0, BCM_PWM_CTL_PWEN2, BCM_PWM_CTL(priv->base));

  nxmutex_unlock(&priv->lock);
  return 0;
}

/****************************************************************************
 * Name: bcm2711_pwm_start
 *
 * Description:
 *   (Re-)initialize the timer resources and start the pulsed output. The
 *   start method should return an error if it cannot start the timer with
 *   the given parameter (frequency or duty)
 *
 * Input Parameters:
 *   dev - The device reference
 *   info - Characteristics for the pulsed output
 *
 * Returned Value:
 *   0 on success, negated errno on failure
 *
 ****************************************************************************/

static int bcm2711_pwm_start(FAR struct pwm_lowerhalf_s *dev,
                             FAR const struct pwm_info_s *info)
{
  struct bcm2711_pwm_s *priv = (struct bcm2711_pwm_s *)dev;
  unsigned i = 0;

  pwminfo("Frequency %u Hz", info->frequency);

  for (i = 0; i < CONFIG_PWM_NCHANNELS; i++)
    {
      pwminfo("Channel %u duty %08x", info->channels[i].channel,
              info->channels[i].duty);
      pwminfo("Channel %u polarity %u", info->channels[i].channel,
              info->channels[i].cpol);

      nxmutex_lock(&priv->lock); /* Lock for this iteration/channel */

      switch (info->channels[i].channel)
        {
        case 1:
          bcm2711_setpulse(BCM_PWM_RNG1(priv->base),
                           BCM_PWM_DAT1(priv->base),
                           info->channels[i].duty, info->frequency);
          bcm2711_channel_pol(priv, BCM_PWM_CTL_POLA1,
                              info->channels[i].cpol);

          /* Enable channel output */

          modreg32(BCM_PWM_CTL_PWEN1, BCM_PWM_CTL_PWEN1,
                   BCM_PWM_CTL(priv->base));
          break;
        case 2:
          bcm2711_setpulse(BCM_PWM_RNG2(priv->base),
                           BCM_PWM_DAT2(priv->base),
                           info->channels[i].duty, info->frequency);
          bcm2711_channel_pol(priv, BCM_PWM_CTL_POLA2,
                              info->channels[i].cpol);

          /* Enable channel output */

          modreg32(BCM_PWM_CTL_PWEN2, BCM_PWM_CTL_PWEN2,
                   BCM_PWM_CTL(priv->base));
          break;
        default:
          pwmerr("Invalid channel %d\n", info->channels[i].channel);
          nxmutex_unlock(&priv->lock); /* Unlock before return */
          return -EINVAL;
        }

      nxmutex_unlock(&priv->lock); /* Unlock for next iteration */
    }

  pwminfo("PWM%d status: %08x", priv->port,
          getreg32(BCM_PWM_STA(priv->base)));
  return 0;
}

/****************************************************************************
 * Name: bcm2711_pwm_stop
 *
 * Description:
 *  Stop the pulsed output and reset the timer resources
 *
 * Input Parameters:
 *   dev - The device reference
 *
 * Returned Value:
 *   0 on success, negated errno on failure
 *
 ****************************************************************************/

static int bcm2711_pwm_stop(FAR struct pwm_lowerhalf_s *dev)
{
  struct bcm2711_pwm_s *priv = (struct bcm2711_pwm_s *)dev;

  nxmutex_lock(&priv->lock);

  /* Ensure channels are disabled */

  modreg32(0, BCM_PWM_CTL_PWEN1, BCM_PWM_CTL(priv->base));
  modreg32(0, BCM_PWM_CTL_PWEN2, BCM_PWM_CTL(priv->base));
  nxmutex_unlock(&priv->lock);
  return 0;
}

/****************************************************************************
 * Name: bcm2711_pwm_ioctl
 *
 * Description:
 *  Lower-half logic may support platform-specific ioctl commands
 *
 * Input Parameters:
 *   dev - The device reference
 *   cmd - IOCTL command
 *   arg - Opaque command argument
 *
 * Returned Value:
 *   0 on success, negated errno on on failure
 *
 ****************************************************************************/

static int bcm2711_pwm_ioctl(FAR struct pwm_lowerhalf_s *dev, int cmd,
                             unsigned long arg)
{
  /* There are no lower-half specific commands needed here. */

  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bcm2711_pwminitialize
 *
 * Description:
 *   Initialize one PWM interface for use with the upper_level PWM driver.
 *
 * Input Parameters:
 *   timer - Which PWM interface to initialize (0 or 1)
 *
 * Returned Value:
 *   On success, a pointer to the lower half PWM driver is returned.
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

struct pwm_lowerhalf_s *bcm2711_pwminitialize(int timer)
{
  int err;
  uint8_t count;
  struct bcm2711_pwm_s *priv;

  switch (timer)
    {
#ifdef CONFIG_BCM2711_PWM0
    case 0:
      priv = &g_pwm0;
      bcm2711_gpio_set_func(CONFIG_BCM2711_PWM0_CHAN1_PIN,
                            BCM2711_PWM0_CHAN1_PIN_ALT);
      bcm2711_gpio_set_pulls(CONFIG_BCM2711_PWM0_CHAN1_PIN, false, false);
      bcm2711_gpio_set_func(CONFIG_BCM2711_PWM0_CHAN2_PIN,
                            BCM2711_PWM0_CHAN2_PIN_ALT);
      bcm2711_gpio_set_pulls(CONFIG_BCM2711_PWM0_CHAN2_PIN, false, false);
      break;
#endif
#ifdef CONFIG_BCM2711_PWM1
    case 1:
      priv = &g_pwm1;
      bcm2711_gpio_set_func(BCM2711_PWM1_CHAN1_PIN,
                            BCM2711_PWM1_CHAN1_PIN_ALT);
      bcm2711_gpio_set_pulls(BCM2711_PWM1_CHAN1_PIN, false, false);
      bcm2711_gpio_set_func(BCM2711_PWM1_CHAN2_PIN,
                            BCM2711_PWM1_CHAN2_PIN_ALT);
      bcm2711_gpio_set_pulls(BCM2711_PWM1_CHAN2_PIN, false, false);
      break;
#endif
    default:
      pwmerr("No PWM%d interface.", timer);
      return NULL;
    }

  /* Don't re-initialize */

  if (priv->init)
    {
      pwmwarn("PWM%d already initialized, skipping...", priv->port);
      return &priv->dev;
    }

  /* Start PWM clock via mailbox domain */

  count = 0;
  do
    {
      err = bcm2711_mbox_setclken(MBOX_CLK_PWM, true);
      count++;
    }
  while (err < 0 && count < 3);

  if (err < 0)
    {
      pwmerr("Couldn't enable PWM clock via mailbox: %d\n", err);
    }

  /* NOTE: initialization sequence determined from rpi4-osdev project.
   * They have CNTL and DIV offsets at 40, 41 from the GPCLK base. This
   * offsets are in multiples of sizeof(unsigned) since they are added to an
   * `unsigned *`.
   */

  /* Stops clock */

  putreg32(BCM_GPCLK_PASSWD | BCM_GPCLK_CM_CTL_KILL, PWM_GPCLKCTL);

  /* Wait for clock to no longer be busy */

  while (getreg32(PWM_GPCLKCTL) & BCM_GPCLK_CM_CTL_BUSY)
    ;

  putreg32(BCM_GPCLK_PASSWD | BCM_PWMCLK_DIV_VAL, PWM_GPCLKDIV);
  putreg32(BCM_GPCLK_PASSWD | BCM_GPCLK_CLKSRC_OSC, PWM_GPCLKCTL);

  /* Enable clock with the same settings. */

  putreg32(BCM_GPCLK_PASSWD | BCM_GPCLK_CLKSRC_OSC | BCM_GPCLK_CM_CTL_ENAB,
           PWM_GPCLKCTL);

  pwminfo("Initialized PWM%d", priv->port);
  priv->init = true;
  return &priv->dev;
}
