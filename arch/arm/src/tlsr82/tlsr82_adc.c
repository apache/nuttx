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

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* ADC Information data */

struct adc_info_s
{
  uint32_t   vref;        /* The reference voltage (mV) */
  bool       calied;      /* Calibration finished or not */
  bool       registered;  /* Have registered a adc device */
  bool       configed;    /* Adc has been configured or not */
  const bool cali;        /* Calibration enable/disable, default enable */
};

/* ADC Private Data */

struct adc_chan_s
{
  uint32_t                     ref;     /* Reference count */
  struct adc_info_s           *info;    /* Adc information */
  const uint8_t                channel; /* Channel number */
  const uint32_t               pinset;  /* GPIO pin number */
  const struct adc_callback_s *cb;      /* Upper driver callback */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void tlsr82_adc_reset(void);
static void tlsr82_adc_power_ctrl(bool enable);
static void tlsr82_adc_clk_ctrl(bool enable);
static void tlsr82_adc_set_sampleclk(uint32_t clk);
static void tlsr82_adc_config(uint32_t cfg);
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

/* ADC information */

static struct adc_info_s g_adc_chan0_info =
{
  .vref       = 1175,  /* Default reference voltage is 1175mV (1.2V) */
  .registered = false,
  .configed   = false,
  .cali       = true,  /* Default calibration switch is enable */
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

static struct adc_chan_s g_adc_chan0 =
{
  .info    = &g_adc_chan0_info,
  .channel = 0,
  .pinset  = GPIO_PIN_PB2,
};

static struct adc_dev_s g_adc_chan0_dev =
{
  .ad_ops      = &g_adcops,
  .ad_priv     = &g_adc_chan0,
};

static sem_t g_sem_excl = SEM_INITIALIZER(1);

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
 *   Config the adc, after this, the adc can start sample.
 *
 * Input Parameters:
 *   cfg  -  adc pinset
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void tlsr82_adc_config(uint32_t cfg)
{
  /* Follow the datasheet and sdk adc_vbat_init() to config the adc */

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

  /* Divider select OFF */

  tlsr82_analog_modify(ADC_DIVIDER_REG, ADC_DIVIDER_SEL_MASK,
                       ADC_DIVIDER_SEL_OFF);

  /* Set the adc differential channel */

  tlsr82_adc_chan_config(cfg);

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

  tlsr82_analog_modify(ADC_SCALE_REG, ADC_SCALE_MASK, ADC_SCALE_1F8);
  tlsr82_analog_modify(ADC_DIVIDER_REG, 0, (0x1 << 4));
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

  /* Vbat mode pin config, disable input, enable output, output set high */

  tlsr82_gpio_input_ctrl(cfg, false);

  tlsr82_gpio_output_ctrl(cfg, true);

  tlsr82_gpiowrite(cfg, true);

  /* Base mode pin config, diable input, disable output, output set low */

  /* nothing */
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

  /* Config the positive and negative input, here, the negative input
   * is always configured to GND (0x0f).
   */

  tlsr82_analog_write(ADC_CHAN_REG, (pinadc << 4) | 0x0f);
}

/****************************************************************************
 * Name: read_efuse
 *
 * Description:
 *   Read Efuse data.
 *
 * Input Parameters:
 *   addr   - register address
 *   b_off  - bit offset
 *   b_size - bit size
 *
 * Returned Value:
 *  Efuse data.
 *
 ****************************************************************************/

static uint32_t read_efuse(uint32_t addr, uint32_t b_off, uint32_t b_size)
{
  uint32_t data;
  uint32_t regval;
  uint32_t shift = 32 - b_size;
  uint32_t mask = UINT32_MAX >> shift;
  uint32_t res = b_off % 32;
  uint32_t regaddr = addr + (b_off / 32 * 4);

  regval = getreg32(regaddr);
  data = regval >> res;
  if (res <= shift)
    {
      data &= mask;
    }
  else
    {
      shift = 32 - res;

      regval = getreg32(regaddr + 4);
      data |= (regval & (mask >> shift)) << shift;
    }

  return data;
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
  volatile uint8_t adc_misc_data_l;
  volatile uint8_t adc_misc_data_h;
  volatile uint16_t adc_misc_data;

  /* Stop the adc sample */

  tlsr82_analog_modify(ADC_CTRL1_REG, ADC_CTRL1_SAMP_MASK,
                       ADC_CTRL1_SAMP_OFF);

  /* Get adc sample value */

  adc_misc_data_l = tlsr82_analog_read(ADC_DATAL_REG);
  adc_misc_data_h = tlsr82_analog_read(ADC_DATAH_REG);

  /* Restart the adc sample */

  tlsr82_analog_modify(ADC_CTRL1_REG, ADC_CTRL1_SAMP_MASK,
                       ADC_CTRL1_SAMP_ON);

  /* Combine the adc sample value and process */

  adc_misc_data = (adc_misc_data_h << 8 | adc_misc_data_l);

  if (adc_misc_data & BIT(13))
    {
      /* Bit13 is the sign bit, bit13 = 1 indicates the data
       * is negative.
       */

      adc_misc_data = 0;
    }
  else
    {
      /* Only get the low 13bit data */

      adc_misc_data &= 0x1fff;
    }

  return adc_misc_data;
}

/****************************************************************************
 * Name: adc_calibrate
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

#if 0

static void adc_calibrate(void)
{
  uint16_t cali_val;
  uint16_t adc;
  uint16_t adc_max = 0;
  uint16_t adc_min = UINT16_MAX;
  uint32_t adc_sum = 0;
  uint32_t regval;

  regval = read_efuse(ADC_CAL_BASE_REG, ADC_CAL_VER_OFF, ADC_CAL_VER_LEN);
  if (regval == 1)
    {
      ainfo("Calibrate based on efuse data\n");

      regval = read_efuse(ADC_CAL_BASE_REG, ADC_CAL_DATA_OFF,
                          ADC_CAL_DATA_LEN);
      cali_val = regval + ADC_CAL_DATA_COMP;
    }
  else
    {
      ainfo("Calibrate based on GND voltage\n");

      /* Enable Vdef */

      rom_i2c_writereg_mask(I2C_ADC, I2C_ADC_HOSTID,
                            I2C_ADC1_DEF, I2C_ADC1_DEF_MSB,
                            I2C_ADC1_DEF_LSB, 1);

      /* Start sampling */

      adc_samplecfg(ADC_CAL_CHANNEL);

      /* Enable internal connect GND (for calibration). */

      rom_i2c_writereg_mask(I2C_ADC, I2C_ADC_HOSTID,
                            I2C_ADC1_ENCAL_GND, I2C_ADC1_ENCAL_GND_MSB,
                            I2C_ADC1_ENCAL_GND_LSB, 1);

      for (int i = 1; i < ADC_CAL_CNT_MAX ; i++)
        {
          adc_set_calibration(0);
          adc = adc_read();

          adc_sum += adc;
          adc_max  = MAX(adc, adc_max);
          adc_min  = MIN(adc, adc_min);
        }

      cali_val = (adc_sum - adc_max - adc_min) / (ADC_CAL_CNT_MAX - 2);

      /* Disable internal connect GND (for calibration). */

      rom_i2c_writereg_mask(I2C_ADC, I2C_ADC_HOSTID,
                            I2C_ADC1_ENCAL_GND,
                            I2C_ADC1_ENCAL_GND_MSB,
                            I2C_ADC1_ENCAL_GND_LSB, 0);
    }

  ainfo("calibration value: %" PRIu16 "\n", cali_val);

  /* Set final calibration parameters */

  adc_set_calibration(cali_val);

  /* Set calibration digital parameters */

  regval = read_efuse(ADC_CAL_BASE_REG, ADC_CAL_VOL_OFF, ADC_CAL_VOL_LEN);
  if (regval & BIT(ADC_CAL_VOL_LEN - 1))
    {
      g_cal_digit = 2000 - (regval & ~(BIT(ADC_CAL_VOL_LEN - 1)));
    }
  else
    {
      g_cal_digit = 2000 + regval;
    }
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

  ret = sem_wait(&g_sem_excl);
  if (ret < 0)
    {
      aerr("Failed to wait sem ret=%d\n", ret);
      return;
    }

  tlsr82_adc_chan_config(priv->pinset);

  /* Dump adc register for debug */

  adc_dump("adc_read_work");

  value = adc_read();

  /* Transfer the adc count into voltage (mV)
   * ADC resolution is 14bit, bit13 is a signed bit and we set the
   * negative input be gnd, so the actual adc resolution is 13bit.
   */

  adc = (value * priv->info->vref * 8) >> 13;

  /* Calibration */

  priv->cb->au_receive(dev, priv->channel, adc);

  ainfo("channel: %" PRIu8 ", voltage: %" PRIu32 " mV\n", priv->channel,
        adc);

  sem_post(&g_sem_excl);
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

  /* Config ADC hardware (Calibration and gpio) */

  ainfo("pin: 0x%" PRIx32 "\n", priv->pinset);

  tlsr82_adc_config(priv->pinset);

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
      case 0:
        dev = &g_adc_chan0_dev;
        break;

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
