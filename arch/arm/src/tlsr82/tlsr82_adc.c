/****************************************************************************
 * arch/arm/src/tlsr82/tlsr82_adc.c
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

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>
#include <nuttx/semaphore.h>

#include "tlsr82_adc.h"
#include "tlsr82_gpio.h"
#include "tlsr82_analog.h"
#include "tlsr82_flash.h"
#include "tlsr82_timer.h"
#include "hardware/tlsr82_dfifo.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ADC calibration max count */

#ifndef MIN
#  define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

#ifndef MAX
#  define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

/* Default reference voltage 1175 mV */

#define ADC_DEFAULT_VREF     1175

#define ADC_FILT_NUM         CONFIG_TLSR82_ADC_FILT_NUM

#if (ADC_FILT_NUM & 0x3) != 0
#  error "The filter number must be multiple of 4 !"
#endif

/* ADC Channel type definition */

#define ADC_CHAN_TYPE_NONE   0
#define ADC_CHAN_TYPE_BASE   1
#define ADC_CHAN_TYPE_VBAT   2
#define ADC_CHAN_TYPE_TEMP   3

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* ADC Information data */

struct adc_info_s
{
  uint32_t   base_vref;    /* The reference voltage (mV) or gain for base/gpio mode */
  int        base_off;     /* The offset for base/gpio mode two-point calibration */
  uint32_t   vbat_vref;    /* The reference voltage (mV) for vbat mode */
  bool       base_two;     /* Base/Gpio mode two-point calibration or not */
  bool       registered;   /* Have registered a adc device */
  bool       configed;     /* Adc has been configured or not */
  uint8_t    channel;      /* Adc current channel */
  uint8_t    channeltype;  /* Adc current channel type */
};

/* ADC Private Data */

struct adc_chan_s
{
  uint32_t                     ref;          /* Reference count */
  struct adc_info_s           *info;         /* Adc information */
  const uint8_t                channeltype;  /* Channel number */
  const uint8_t                channel;      /* Channel number */
  const uint32_t               pinset;       /* GPIO pin number */
  const struct adc_callback_s *cb;           /* Upper driver callback */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void tlsr82_adc_reset(void);
static void tlsr82_adc_power_ctrl(bool enable);
static void tlsr82_adc_clk_ctrl(bool enable);
static void tlsr82_adc_config(struct adc_chan_s *priv);
static void tlsr82_adc_pin_config(uint32_t pinset);
static void tlsr82_adc_chan_config(uint32_t pinset);

static void adc_dump(const char *msg);
static int  adc_bind(struct adc_dev_s *dev,
                     const struct adc_callback_s *callback);
static void adc_reset(struct adc_dev_s *dev);
static int  adc_setup(struct adc_dev_s *dev);
static void adc_shutdown(struct adc_dev_s *dev);
static void adc_rxint(struct adc_dev_s *dev, bool enable);
static int  adc_ioctl(struct adc_dev_s *dev, int cmd, unsigned long arg);
static void adc_read_work(struct adc_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* ADC module information */

static struct adc_info_s g_adc_module0_info =
{
  .base_vref    = ADC_DEFAULT_VREF,
  .base_off     = 0,
  .vbat_vref    = ADC_DEFAULT_VREF,
  .base_two     = false,
  .registered   = false,
  .configed     = false,
  .channel      = ADC_CHAN_NONE,
  .channeltype  = ADC_CHAN_TYPE_NONE,
};

/* ADC interface operations */

static const struct adc_ops_s g_adcops =
{
  .ao_bind        = adc_bind,
  .ao_reset       = adc_reset,
  .ao_setup       = adc_setup,
  .ao_shutdown    = adc_shutdown,
  .ao_rxint       = adc_rxint,
  .ao_ioctl       = adc_ioctl,
};

/* ADC normal channel, for gpio sample */

#ifdef CONFIG_TLSR82_ADC_CHAN0
static struct adc_chan_s g_adc_chan0 =
{
  .info        = &g_adc_module0_info,
  .channeltype = ADC_CHAN_TYPE_BASE,
  .channel     = ADC_CHAN_0,
  .pinset      = GPIO_PIN_PB2,
};

static struct adc_dev_s g_adc_chan0_dev =
{
  .ad_ops  = &g_adcops,
  .ad_priv = &g_adc_chan0,
};
#endif

#ifdef CONFIG_TLSR82_ADC_CHAN1
static struct adc_chan_s g_adc_chan1 =
{
  .info        = &g_adc_module0_info,
  .channeltype = ADC_CHAN_TYPE_BASE,
  .channel     = ADC_CHAN_1,
  .pinset      = GPIO_PIN_PB3,
};

static struct adc_dev_s g_adc_chan1_dev =
{
  .ad_ops  = &g_adcops,
  .ad_priv = &g_adc_chan1,
};
#endif

#ifdef CONFIG_TLSR82_ADC_CHAN2
static struct adc_chan_s g_adc_chan2 =
{
  .info        = &g_adc_module0_info,
  .channeltype = ADC_CHAN_TYPE_BASE,
  .channel     = ADC_CHAN_2,
  .pinset      = GPIO_PIN_PB5,
};

static struct adc_dev_s g_adc_chan2_dev =
{
  .ad_ops  = &g_adcops,
  .ad_priv = &g_adc_chan2,
};
#endif

/* ADC Bat channel, for chip battery sample */

#ifdef CONFIG_TLSR82_ADC_VBAT
static struct adc_chan_s g_adc_chanbat =
{
  .info        = &g_adc_module0_info,
  .channeltype = ADC_CHAN_TYPE_VBAT,
  .channel     = ADC_CHAN_VBAT,
  .pinset      = GPIO_INVLD_CFG,
};

static struct adc_dev_s g_adc_chanbat_dev =
{
  .ad_ops  = &g_adcops,
  .ad_priv = &g_adc_chanbat,
};
#endif

static sem_t g_sem_excl = SEM_INITIALIZER(1);

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tlsr82_adc_dfifo_enable
 *
 * Description:
 *   Enable the adc dfifo/dfifo2, the dfifo2 will copy the adc sample data
 *   to the address in DFIFO_ADC_ADDR_REG.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void tlsr82_adc_dfifo_enable(void)
{
  DFIFO_MODE_REG |= DFIFO_MODE_DFIFO2_IN;
}

/****************************************************************************
 * Name: tlsr82_adc_dfifo_disable
 *
 * Description:
 *   Disable the adc dfifo/dfifo2
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void tlsr82_adc_dfifo_disable(void)
{
  DFIFO_MODE_REG &= ~DFIFO_MODE_DFIFO2_IN;
}

/****************************************************************************
 * Name: tlsr82_adc_dfifo_config
 *
 * Description:
 *   Config the data buffer, so DFIFO2 can copy sample value to buffer
 *
 * Input Parameters:
 *   buffer - the buffer to store the adc sample data
 *   size   - the buffer size, this size must be multiple of 4
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void tlsr82_adc_dfifo_config(uint8_t *buffer, size_t size)
{
  /* Config the data buffer, so DFIFO2 can copy sample value to buffer
   * DFIFO buffer address : only need low 16 bit, beacause the high 16 bit
   *                        must be 0x0084
   * DFIFO buffer size    : DFIFO_ADC_SIZE_REG = n ==> 4 * (n + 1) size
   *                        DFIFO_ADC_SIZE_REG = size / 4 - 1
   */

  DFIFO_ADC_ADDR_REG = (uint16_t)((uint32_t)buffer & 0xffff);
  DFIFO_ADC_SIZE_REG = (size >> 2) - 1;

  /* Clear the dfifo write pointer */

  DFIFO2_WPTR_REG    = 0;
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tlsr82_adc_reset
 *
 * Description:
 *   Reset the adc, reconfig the adc.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void tlsr82_adc_reset(void)
{
  BM_SET(RESET_RST1_REG, RESET_RST1_ADC);
  BM_CLR(RESET_RST1_REG, RESET_RST1_ADC);
}

/****************************************************************************
 * Name: tlsr82_adc_power_ctrl
 *
 * Description:
 *   Enable/Disable the ADC power, the power should be power up before
 *   config the adc.
 *
 * Input Parameters:
 *   enable  -  true : power up
 *              false: power down
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void tlsr82_adc_power_ctrl(bool enable)
{
  tlsr82_analog_write(ADC_PAG_CTRL_REG,
                      (tlsr82_analog_read(ADC_PAG_CTRL_REG) &
                       (~ADC_POWER_MASK)) |
                      (!enable) << ADC_POWER_SHIFT);
}

/****************************************************************************
 * Name: tlsr82_adc_clk_ctrl
 *
 * Description:
 *   Enable/Disable the ADC clock.
 *
 * Input Parameters:
 *   enable  -  true : enable adc clock
 *              false: disable adc clock
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void tlsr82_adc_clk_ctrl(bool enable)
{
  if (enable)
    {
      tlsr82_analog_write(ADC_CLK_REG,
                          tlsr82_analog_read(ADC_CLK_REG) |
                          ADC_CLK_24M_EN);
    }
  else
    {
      tlsr82_analog_write(ADC_CLK_REG,
                          tlsr82_analog_read(ADC_CLK_REG) &
                          (~ADC_CLK_24M_EN));
    }
}

/****************************************************************************
 * Name: tlsr82_adc_config
 *
 * Description:
 *   Config the adc to different mode, after this, the adc can start sample
 *   the voltage in the gpio pin (Base mode) or the chip volatge (Vbat
 *   channel mode).
 *   Five configuration conditions:
 *   1. Same channel, do not need do not need re-configuration;
 *   2. adc module has not been configured, must configure it;
 *   3. adc module has been configured, but current channel and configured
 *      channel are both vbat channel, do not need re-configuration;
 *   4. adc module has been configured, but current channel and configured
 *      channel are both base channel, only configure the channel;
 *   5. others, need re-configuration.
 *
 * Input Parameters:
 *   priv  - adc channel handler
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void tlsr82_adc_config(struct adc_chan_s *priv)
{
  uint8_t channel     = priv->info->channel;
  uint8_t channeltype = priv->info->channeltype;

  ainfo("Current channel=%u, channeltype=%u\n", channel, channeltype);
  ainfo("Input   channel=%u, channeltype=%u\n",
        priv->channel, priv->channeltype);

  DEBUGASSERT(priv != NULL && priv->channel != ADC_CHAN_NONE);

  /* If current channel type is same as the priv channel type, do not
   * need re-configure all the register.
   */

  if (channeltype == priv->channeltype)
    {
      if (channeltype == ADC_CHAN_TYPE_BASE && channel != priv->channel)
        {
          /* The channel type is base and the input channel (GPIO pin) is
           * different, only need re-configure the adc input channel.
           */

          tlsr82_adc_chan_config(priv->pinset);
          priv->info->channel = priv->channel;
        }

      return;
    }

  ainfo("Start config\n");

  /* Follow the sdk code adc_base_init() and adc_vbat_channel_init()
   * to config the adc, VBAT_CHAN mode is samiler to BASE mode, the
   * differences are:
   * 1. adc divider : VBAT_CHAN mode, 1/3
   *                  BASE mode, 1
   * 2. pre-scale   : VBAT_CHAN mode, 1
   *                  BASE mode, 1/8
   */

  /* Enable misc chanel and set totaol length for sampling state be 2 */

  tlsr82_analog_write(ADC_CTRL0_REG, ADC_CTRL0_CHANEN_ENABLE |
                                     (2 << ADC_CTRL0_SAMPLEN_SHIFT));

  /* Set the max capture time and max set time, time = count / 24Mhz
   * r_max_s (Set)     : ADC_SAMP3_REG_0xf1<3:0>
   * r_max_mc(Capture) : ADC_SAMP3_REG_0xf1<7:6>, ADC_SAMP1_REG_0xef<7:0>
   * Here, set r_max_s = 10 (0.417us), r_max_mc = 240 (10us)
   * So, total duration = 10.417us, sample frequency = 24Mhz / 250 = 96Khz
   */

  tlsr82_analog_write(ADC_SAMP1_REG, 240 & 0xff);
  tlsr82_analog_write(ADC_SAMP3_REG, ((240 >> 8) << 6) | (10 & 0xff));

  /* Divider select 1/3 or OFF */

#ifdef CONFIG_TLSR82_ADC_VBAT
  if (priv->channeltype == ADC_CHAN_TYPE_VBAT)
    {
      tlsr82_analog_modify(ADC_DIVIDER_REG, ADC_DIVIDER_SEL_MASK,
                           ADC_DIVIDER_SEL_1F3);
    }
  else
#endif
    {
      tlsr82_analog_modify(ADC_DIVIDER_REG, ADC_DIVIDER_SEL_MASK,
                           ADC_DIVIDER_SEL_OFF);
    }

  /* Set the adc differential channel */

#ifdef CONFIG_TLSR82_ADC_VBAT
  if (priv->channeltype == ADC_CHAN_TYPE_VBAT)
    {
      tlsr82_analog_write(ADC_CHAN_REG, ADC_CHAN_POS_VBAT |
                                        ADC_CHAN_NEG_GND);
    }
  else
#endif
    {
      tlsr82_adc_chan_config(priv->pinset);
    }

  /* Enable the Different input mode */

  tlsr82_analog_modify(ADC_MODE_REG, ADC_MODE_INPUT_MASK,
                       ADC_MODE_INPUT_DIFF);

  /* Reference voltage select, 1.2V */

  tlsr82_analog_modify(ADC_VREF_REG, ADC_VREF_MASK, ADC_VREF_1P2V);

  /* Adc resolution select, 14bit */

  tlsr82_analog_modify(ADC_MODE_REG, ADC_MODE_RES_MASK,
                       ADC_MODE_RES_14BIT);

  /* Adc sample cycle number, 6 */

  tlsr82_analog_modify(ADC_SAMP0_REG, ADC_SAMP0_CYCLE_MASK,
                       ADC_SAMP0_CYCLE_6);

  /* Adc pre-scale select, 1/8
   * When pre-scaling is 1  , the ADC_DIVIDER_REG_0xf9 <4:5> must be 0
   * When pre-scaling is 1/8, the ADC_DIVIDER_REG_0xf9 <4> must be 1
   */

#ifdef CONFIG_TLSR82_ADC_VBAT
  if (priv->channeltype == ADC_CHAN_TYPE_VBAT)
    {
      tlsr82_analog_modify(ADC_SCALE_REG, ADC_SCALE_MASK, ADC_SCALE_1);
      tlsr82_analog_modify(ADC_DIVIDER_REG, BIT_RNG(4, 5), 0);
    }
  else
#endif
    {
      tlsr82_analog_modify(ADC_SCALE_REG, ADC_SCALE_MASK, ADC_SCALE_1F8);
      tlsr82_analog_modify(ADC_DIVIDER_REG, BIT(4), BIT(4));
    }

  /* Set current channel and current type */

  priv->info->channel     = priv->channel;
  priv->info->channeltype = priv->channeltype;
}

/****************************************************************************
 * Name: tlsr82_adc_pin_config
 *
 * Description:
 *   Configure the the input pin as adc function, this function should be
 *   called before used this pin as adc.
 *
 * Input Parameters:
 *   pinset  -  adc pinset
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void tlsr82_adc_pin_config(uint32_t pinset)
{
  uint32_t cfg = GPIO_CFG2PIN(pinset);

  GPIO_SET_AS_GPIO(GPIO_GET(GROUP, cfg), GPIO_GET(PIN, cfg));

  /* Base mode pin config, diable input, disable output, output set low */

  tlsr82_gpio_input_ctrl(cfg, false);

  tlsr82_gpio_output_ctrl(cfg, false);

  tlsr82_gpiowrite(cfg, false);
}

/****************************************************************************
 * Name: tlsr82_adc_chan_config
 *
 * Description:
 *   Configure the the input pin as the adc channel input, tlsr82 only have
 *   one adc conversion channel.
 *
 * Input Parameters:
 *   pinset  -  adc pinset
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void tlsr82_adc_chan_config(uint32_t pinset)
{
  uint16_t pinnum = GPIO_PIN2NUM(pinset);
  uint16_t pinadc;

  /* Convert the gpio pin index to adc register defined pin index */

  if (pinnum >= 8 && pinnum <= 15)
    {
      /* PB0 ~ PB7 */

      pinadc = pinnum - 7;
    }
  else if (pinnum >= 20 && pinnum <= 21)
    {
      /* PC4 ~ PC5 */

      pinadc = pinnum - 11;
    }
  else
    {
      aerr("Adc pin number error, pinnum=%u\n", pinnum);
      return;
    }

  /* Config gpio */

  tlsr82_adc_pin_config(pinset);

  /* Config the positive and negative input, here, the negative input
   * is always configured to GND (0x0f).
   */

  tlsr82_analog_write(ADC_CHAN_REG, (pinadc << ADC_CHAN_POS_SHIFT) |
                                    ADC_CHAN_NEG_GND);
}

/****************************************************************************
 * Name: adc_dump
 *
 * Description:
 *   Dump all the adc register value.
 *
 * Input Parameters:
 *   msg - Message need to print.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void adc_dump(const char *msg)
{
  if (msg)
    {
      ainfo("%s, dump all adc regs:\n", msg);
    }
  else
    {
      ainfo("Dump all adc regs:\n");
    }

  ainfo("ADC_VREF_REG     : addr=0x%x, val=0x%x\n",
         ADC_VREF_REG, tlsr82_analog_read(ADC_VREF_REG));
  ainfo("ADC_CHAN_REG     : addr=0x%x, val=0x%x\n",
         ADC_CHAN_REG, tlsr82_analog_read(ADC_CHAN_REG));
  ainfo("ADC_MODE_REG     : addr=0x%x, val=0x%x\n",
         ADC_MODE_REG, tlsr82_analog_read(ADC_MODE_REG));
  ainfo("ADC_SAMP0_REG    : addr=0x%x, val=0x%x\n",
        ADC_SAMP0_REG, tlsr82_analog_read(ADC_SAMP0_REG));
  ainfo("ADC_SAMP1_REG    : addr=0x%x, val=0x%x\n",
        ADC_SAMP1_REG, tlsr82_analog_read(ADC_SAMP1_REG));
  ainfo("ADC_SAMP2_REG    : addr=0x%x, val=0x%x\n",
        ADC_SAMP2_REG, tlsr82_analog_read(ADC_SAMP2_REG));
  ainfo("ADC_SAMP3_REG    : addr=0x%x, val=0x%x\n",
        ADC_SAMP3_REG, tlsr82_analog_read(ADC_SAMP3_REG));
  ainfo("ADC_CTRL0_REG    : addr=0x%x, val=0x%x\n",
        ADC_CTRL0_REG, tlsr82_analog_read(ADC_CTRL0_REG));
  ainfo("ADC_CTRL1_REG    : addr=0x%x, val=0x%x\n",
        ADC_CTRL1_REG, tlsr82_analog_read(ADC_CTRL1_REG));
  ainfo("ADC_CLKDIV_REG   : addr=0x%x, val=0x%x\n",
        ADC_CLKDIV_REG, tlsr82_analog_read(ADC_CLKDIV_REG));
  ainfo("ADC_STATUS_REG   : addr=0x%x, val=0x%x\n",
        ADC_STATUS_REG, tlsr82_analog_read(ADC_STATUS_REG));
  ainfo("ADC_DATAL_REG    : addr=0x%x, val=0x%x\n",
        ADC_DATAL_REG, tlsr82_analog_read(ADC_DATAL_REG));
  ainfo("ADC_DATAH_REG    : addr=0x%x, val=0x%x\n",
        ADC_DATAH_REG, tlsr82_analog_read(ADC_DATAH_REG));
  ainfo("ADC_DIVIDER_REG  : addr=0x%x, val=0x%x\n",
        ADC_DIVIDER_REG, tlsr82_analog_read(ADC_DIVIDER_REG));
  ainfo("ADC_SCALE_REG    : addr=0x%x, val=0x%x\n",
        ADC_SCALE_REG, tlsr82_analog_read(ADC_SCALE_REG));
  ainfo("ADC_PAG_CTRL_REG : addr=0x%x, val=0x%x\n",
        ADC_PAG_CTRL_REG, tlsr82_analog_read(ADC_PAG_CTRL_REG));
  ainfo("ADC_CLK_REG      : addr=0x%x, val=0x%x\n",
        ADC_CLK_REG, tlsr82_analog_read(ADC_CLK_REG));
}

/****************************************************************************
 * Name: adc_read
 *
 * Description:
 *   Start ADC sampling and read ADC value.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Read ADC value.
 *
 ****************************************************************************/

static uint16_t adc_read(void)
{
  volatile uint16_t data_buf[ADC_FILT_NUM];
  uint16_t tmp;
  uint16_t max = 0;
  uint16_t min = UINT16_MAX;
  uint32_t sum = 0;
  uint32_t currtime;
  int i;

  /* Clear the data buffer and config */

  memset((void *)data_buf, 0, ADC_FILT_NUM);

  tlsr82_adc_dfifo_config((uint8_t *)data_buf, ADC_FILT_NUM);

  /* Enable DFIFO 2 */

  tlsr82_adc_dfifo_enable();

  /* Wait at least 2 sample cycle, frequency = 96k, T = 10.4us */

  currtime = tlsr82_time();
  while (!tlsr82_time_exceed(currtime, 25));

  for (i = 0; i < ADC_FILT_NUM; i++)
    {
      while (data_buf[i] == 0 && !tlsr82_time_exceed(currtime, 25));
      currtime = tlsr82_time();

      /* Bit13 is the sign bit, bit13 = 1 indicates the data
       * is negative but we only get the low 13bit data.
       */

      tmp = data_buf[i];
      if (tmp & BIT(13))
        {
          tmp = 0;
        }
      else
        {
          tmp &= 0x1fff;
        }

      sum += tmp;

      if (tmp > max)
        {
          max = tmp;
        }

      if (tmp < min)
        {
          min = tmp;
        }

      ainfo("data_buf[%d]=%u\n", i, tmp);
    }

  /* Disable DFIFO 2 */

  tlsr82_adc_dfifo_disable();

  ainfo("sum=%lu, max=%u, min=%u, filt_num=%d\n",
        sum, max, min, ADC_FILT_NUM);

  /* Remove max, min value and get the average value */

  tmp = (sum - min - max) / (ADC_FILT_NUM - 2);

  return tmp;
}

/****************************************************************************
 * Name: tlsr82_adc_calibrate
 *
 * Description:
 *   ADC calibration.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_TLSR82_ADC_CALI
static void tlsr82_adc_calibrate(struct adc_chan_s *priv)
{
  uint8_t cali_data[7];

  uint32_t base_vref;
  uint32_t vbat_vref;

  memset((void *)cali_data, 0, sizeof(cali_data));
  tlsr82_flash_read_data(CONFIG_TLSR82_ADC_CALI_PARA_ADDR, cali_data,
                         sizeof(cali_data));

  ainfo("Calibration data: 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n",
        cali_data[0], cali_data[1], cali_data[2], cali_data[3],
        cali_data[4], cali_data[5], cali_data[6]);

  /* Calibration parameters format follow the telink sdk code */

  /* Base/gpio mode calibration parameters read */

  if (cali_data[4] <= 0x7f && cali_data[5] != 0xff &&
      cali_data[6] != 0xff)
    {
      /* Two-point calibration exist, the base_vref as the gain to use
       * Method of calculating two-point gpio calibration Flash_gain
       * and Flash_offset:
       * gain   = (data[6] << 8) + data[5] + 1000 mv
       * offset = data[4] - 20 mv
       */

      priv->info->base_vref = (cali_data[6] << 8) + cali_data[5] + 1000;
      priv->info->base_off  = (int)cali_data[4] - 20;
      priv->info->base_two  = true;
    }
  else
    {
      /* One-point calibration exist
       * Method of calculating calibration Flash_gpio_vref value:
       * vref = 1175 + data[0] - 255 + data[1] mV
       *      = 920 + data[0] + data[1] mV
       */

      base_vref = 920 + cali_data[0] + cali_data[1];
      if (base_vref >= 1047 && base_vref <= 1302)
        {
          priv->info->base_vref = base_vref;
        }

      priv->info->base_two = false;
    }

  /* Vbat mode calibration parameters read */

  if (cali_data[2] != 0xff || cali_data[3] != 0xff)
    {
      /* One-point calibration exist
       * Method of calculating calibration Flash_vbat_vref value:
       * vref = 1175 + data[2] - 255 + data[3] mV
       *      = 920 + data[2] + data[3] mV
       */

      vbat_vref = 920 + cali_data[2] + cali_data[3];
      if (vbat_vref >= 1047 && vbat_vref <= 1302)
        {
          priv->info->vbat_vref = vbat_vref;
        }
    }

    ainfo("Calibration paramters:\n");
    ainfo("  base two-point: gain=%d, offset=%d\n",
          priv->info->base_vref, priv->info->base_off);
    ainfo("  base one-point: vref=%lu\n", priv->info->base_vref);
    ainfo("  vbat one-point: vref=%lu\n", priv->info->vbat_vref);
}
#endif

/****************************************************************************
 * Name: adc_read_work
 *
 * Description:
 *   Read ADC value and pass it to up.
 *
 * Input Parameters:
 *   dev - ADC device pointer
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void adc_read_work(struct adc_dev_s *dev)
{
  int ret;
  uint32_t value;
  int32_t adc;
  struct adc_chan_s *priv = (struct adc_chan_s *)dev->ad_priv;

  ret = nxsem_wait(&g_sem_excl);
  if (ret < 0)
    {
      aerr("Failed to wait sem ret=%d\n", ret);
      return;
    }

  /* Config the adc */

  tlsr82_adc_config(priv);

  /* Dump adc register for debug */

  adc_dump("adc_read_work");

  value = adc_read();

  /* Transfer the adc count into voltage (mV)
   * ADC resolution is 14bit, bit13 is a signed bit and we set the
   * negative input be gnd, so the actual adc resolution is 13bit.
   */

#ifdef CONFIG_TLSR82_ADC_VBAT
  if (priv->channeltype == ADC_CHAN_TYPE_VBAT)
    {
      /* Vbat channel mode, divider = 1/3, scale = 1 ==> factor = 3 */

      adc = (value * priv->info->vbat_vref * 3) >> 13;
    }
  else
#endif
    {
      /* Base/gpio mode, divider = 1, scale = 1/8 ==> factor = 8 */

      adc = (value * priv->info->base_vref * 8) >> 13;

      if (priv->info->base_two)
        {
          adc += priv->info->base_off;
        }
    }

  /* Put adc value to the adc buffer */

  priv->cb->au_receive(dev, priv->channel, adc);

  ainfo("channel: %" PRIu8 ", voltage: %" PRIu32 " mV\n", priv->channel,
        adc);

  nxsem_post(&g_sem_excl);
}

/****************************************************************************
 * Name: adc_bind
 *
 * Description:
 *   Bind the upper-half driver callbacks to the lower-half implementation.
 *   This must be called early in order to receive ADC event notifications.
 *
 ****************************************************************************/

static int adc_bind(struct adc_dev_s *dev,
                    const struct adc_callback_s *callback)
{
  struct adc_chan_s *priv = (struct adc_chan_s *)dev->ad_priv;

  DEBUGASSERT(priv != NULL);

  ainfo("channel: %" PRIu8 "\n", priv->channel);

  priv->cb = callback;

  return OK;
}

/****************************************************************************
 * Name: adc_reset
 *
 * Description:
 *   Reset the ADC device.  Called early to initialize the hardware.
 *   This is called, before adc_setup() and on error conditions.
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

static void adc_reset(struct adc_dev_s *dev)
{
  irqstate_t flags;
  struct adc_chan_s *priv = (struct adc_chan_s *)dev->ad_priv;

  ainfo("channel: %" PRIu8 "\n", priv->channel);

  flags = enter_critical_section();

  /* Do nothing if ADC instance is currently in use */

  if (priv->ref > 0)
    {
      goto out;
    }

  if (priv->info->registered)
    {
      goto out;
    }

  /* Reset ADC hardware */

  tlsr82_adc_power_ctrl(true);

  tlsr82_adc_reset();

  tlsr82_adc_clk_ctrl(false);

  /* adc_reset() will be called in adc_register(), the same one adc
   * device should be resetted only once.
   */

  priv->info->registered = true;

out:
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: adc_setup
 *
 * Description:
 *   Configure the ADC. This method is called the first time that the ADC
 *   device is opened.  This will occur when the port is first opened.
 *   This setup includes configuring.
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

static int adc_setup(struct adc_dev_s *dev)
{
  struct adc_chan_s *priv = (struct adc_chan_s *)dev->ad_priv;

  ainfo("channel: %" PRIu8 "\n", priv->channel);

  /* Do nothing when the ADC device is already set up */

  if (priv->ref > 0)
    {
      priv->ref++;
      return OK;
    }

  /* The same one adc device should be configgured only once */

  if (priv->info->configed)
    {
      return OK;
    }

  /* Enable ADC clock */

  tlsr82_adc_clk_ctrl(true);

  /* Read the calibration parameters */

#ifdef CONFIG_TLSR82_ADC_CALI
  tlsr82_adc_calibrate(priv);
#endif

  /* Config ADC hardware */

  ainfo("pin: 0x%" PRIx32 ", channel: %" PRIu8 "\n",
        priv->pinset, priv->channel);

  tlsr82_adc_config(priv);

  /* The ADC device is ready */

  priv->info->configed = true;
  priv->ref++;

  return OK;
}

/****************************************************************************
 * Name: adc_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts.
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

static void adc_rxint(struct adc_dev_s *dev, bool enable)
{
  UNUSED(dev);
  UNUSED(enable);
}

/****************************************************************************
 * Name: adc_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method.
 *
 * Input Parameters:
 *   dev - pointer to device structure used by the driver
 *   cmd - command
 *   arg - arguments passed with command
 *
 * Returned Value:
 *
 ****************************************************************************/

static int adc_ioctl(struct adc_dev_s *dev, int cmd, unsigned long arg)
{
  int ret;
  struct adc_chan_s *priv = (struct adc_chan_s *)dev->ad_priv;

  ainfo("channel: %" PRIu8 " cmd=%d\n", priv->channel, cmd);

  switch (cmd)
    {
      case ANIOC_TRIGGER:
        {
          /* Start sampling and read ADC value here */

          adc_read_work(dev);
          ret = OK;
        }
      break;

      case ANIOC_GET_NCHANNELS:
        {
          /* Return the number of configured channels */

          ret = 1;
        }
        break;

      default:
        {
          aerr("ERROR: Unknown cmd: %d\n", cmd);
          ret = -ENOTTY;
        }
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: adc_shutdown
 *
 * Description:
 *   Disable the ADC.  This method is called when the ADC device is closed.
 *   This method reverses the operation the setup method.
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

static void adc_shutdown(struct adc_dev_s *dev)
{
  struct adc_chan_s *priv = (struct adc_chan_s *)dev->ad_priv;

  ainfo("channel: %" PRIu8 "\n", priv->channel);

  /* Decrement count only when ADC device is in use */

  if (priv->ref > 0)
    {
      priv->ref--;

      /* Shutdown the ADC device only when not in use */

      if (priv->ref == 0)
        {
          /* Disable ADC clock */

          tlsr82_adc_clk_ctrl(false);

          /* Clear the configed flag */

          priv->info->configed = false;
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tlsr82_adc_init
 *
 * Description:
 *   Initialize the ADC.
 *
 * Input Parameters:
 *   channel - ADC channel number
 *
 * Returned Value:
 *   ADC device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

int tlsr82_adc_init(const char *devpath, int miror)
{
  int ret = OK;
  struct adc_dev_s *dev;

  ainfo("ADC channel: %" PRIu8 "\n", miror);

  switch (miror)
    {
#ifdef CONFIG_TLSR82_ADC_CHAN0
      case ADC_CHAN_0:
        dev = &g_adc_chan0_dev;
        break;
#endif

#ifdef CONFIG_TLSR82_ADC_CHAN1
      case ADC_CHAN_1:
        dev = &g_adc_chan1_dev;
        break;
#endif

#ifdef CONFIG_TLSR82_ADC_CHAN2
      case ADC_CHAN_2:
        dev = &g_adc_chan2_dev;
        break;
#endif

#ifdef CONFIG_TLSR82_ADC_VBAT
      case ADC_CHAN_VBAT:
        dev = &g_adc_chanbat_dev;
        break;
#endif

      default:
        {
          aerr("ERROR: No ADC interface defined\n");
          return -ENOTTY;
        }
    }

  ret = adc_register(devpath, dev);
  if (ret < 0)
    {
      aerr("Adc register fail, devpath=%s, ret=%d\n", devpath, ret);
    }

  return ret;
}
