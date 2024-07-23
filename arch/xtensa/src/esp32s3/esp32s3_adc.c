/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_adc.c
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
#include <sys/param.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/mutex.h>

#include "esp32s3_gpio.h"
#include "esp32s3_dma.h"
#include "esp32s3_irq.h"
#include "esp32s3_adc.h"

#include "xtensa.h"
#include "hardware/esp32s3_system.h"
#include "hardware/esp32s3_efuse.h"
#include "hardware/esp32s3_sens.h"
#include "hardware/esp32s3_gpio_sigmap.h"
#include "hardware/regi2c_ctrl.h"
#include "hardware/regi2c_saradc.h"
#include "hardware/esp32s3_rtc_io.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ADC calibration max count */

#define ADC_CAL_CNT_MAX         (32)

/* ADC calibration max value */

#define ADC_CAL_VAL_MAX         (4096 - 1)

/* ADC calibration sampling channel */

#define ADC_CAL_CHANNEL         (0xf)

/* ADC max value mask */

#define ADC_VAL_MASK            (0xfff)

#define ADC_CAL_BASE_REG        EFUSE_RD_SYS_PART1_DATA0_REG

#define ADC_CAL_VER_OFF         (128)
#define ADC_CAL_VER_LEN         (2)

#define ADC_CAL_DATA_COMP       (1550)

#define ADC_CAL_VOL_LEN         (8)

/* ADC input voltage attenuation, this affects measuring range */

#define ADC_ATTEN_DB_0          (0)     /* Vmax = 950 mV  */
#define ADC_ATTEN_DB_2_5        (1)     /* Vmax = 1250 mV */
#define ADC_ATTEN_DB_6          (2)     /* Vmax = 1750 mV */
#define ADC_ATTEN_DB_12         (3)     /* Vmax = 3100 mV */

/* ADC attenuation */

#if defined(CONFIG_ESP32S3_ADC_VOL_950)
#  define ADC_ATTEN_DEF         ADC_ATTEN_DB_0
#  define ADC_CAL_DATA_LEN        (8)

#  define ADC_CAL_DATA_OFF      (149)
#  define ADC_CAL_VOL_OFF       (201)

#  define ADC_CAL_VOL_DEF       (488)
#elif defined(CONFIG_ESP32S3_ADC_VOL_1250)
#  define ADC_ATTEN_DEF         ADC_ATTEN_DB_2_5
#  define ADC_CAL_DATA_LEN        (6)

#  define ADC_CAL_DATA_OFF      (157)
#  define ADC_CAL_VOL_OFF       (209)

#  define ADC_CAL_VOL_DEF       (641)
#elif defined(CONFIG_ESP32S3_ADC_VOL_1750)
#  define ADC_ATTEN_DEF         ADC_ATTEN_DB_6
#  define ADC_CAL_DATA_LEN        (6)

#  define ADC_CAL_DATA_OFF      (163)
#  define ADC_CAL_VOL_OFF       (217)

#  define ADC_CAL_VOL_DEF       (892)
#elif defined(CONFIG_ESP32S3_ADC_VOL_3100)
#  define ADC_ATTEN_DEF         ADC_ATTEN_DB_12
#  define ADC_CAL_DATA_LEN        (6)

#  define ADC_CAL_DATA_OFF      (169)
#  define ADC_CAL_VOL_OFF       (225)

#  define ADC_CAL_VOL_DEF       (1592)
#endif

#define setbits(bs, a)     modifyreg32(a, 0, bs)
#define resetbits(bs, a)   modifyreg32(a, bs, 0)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* ADC Private Data */

struct adc_chan_s
{
  uint32_t ref;           /* Reference count */

  const uint8_t channel;  /* Channel number */
  const uint8_t pin;      /* GPIO pin number */

  const struct adc_callback_s *cb;  /* Upper driver callback */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  adc_bind(struct adc_dev_s *dev,
                     const struct adc_callback_s *callback);
static void adc_reset(struct adc_dev_s *dev);
static int  adc_setup(struct adc_dev_s *dev);
static void adc_shutdown(struct adc_dev_s *dev);
static void adc_rxint(struct adc_dev_s *dev, bool enable);
static int  adc_ioctl(struct adc_dev_s *dev, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

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

#ifdef CONFIG_ESP32S3_ADC1_CHANNEL0
static struct adc_chan_s g_adc1_chan0 =
{
  .channel = 0,
  .pin = 1
};
#endif

#ifdef CONFIG_ESP32S3_ADC1_CHANNEL1
static struct adc_chan_s g_adc1_chan1 =
{
  .channel = 1,
  .pin = 2
};
#endif

#ifdef CONFIG_ESP32S3_ADC1_CHANNEL2
static struct adc_chan_s g_adc1_chan2 =
{
  .channel = 2,
  .pin = 3
};
#endif

#ifdef CONFIG_ESP32S3_ADC1_CHANNEL3
static struct adc_chan_s g_adc1_chan3 =
{
  .channel = 3,
  .pin = 4
};
#endif

#ifdef CONFIG_ESP32S3_ADC1_CHANNEL4
static struct adc_chan_s g_adc1_chan4 =
{
  .channel = 4,
  .pin = 5
};
#endif

#ifdef CONFIG_ESP32S3_ADC1_CHANNEL5
static struct adc_chan_s g_adc1_chan5 =
{
  .channel = 5,
  .pin = 6
};
#endif

#ifdef CONFIG_ESP32S3_ADC1_CHANNEL6
static struct adc_chan_s g_adc1_chan6 =
{
  .channel = 6,
  .pin = 7
};
#endif

#ifdef CONFIG_ESP32S3_ADC1_CHANNEL7
static struct adc_chan_s g_adc1_chan7 =
{
  .channel = 7,
  .pin = 8
};
#endif

#ifdef CONFIG_ESP32S3_ADC1_CHANNEL8
static struct adc_chan_s g_adc1_chan8 =
{
  .channel = 8,
  .pin = 9
};
#endif

#ifdef CONFIG_ESP32S3_ADC1_CHANNEL9
static struct adc_chan_s g_adc1_chan9 =
{
  .channel = 9,
  .pin = 10
};
#endif

/* ADC calibration mark */

static bool g_adc_switch;

/* ADC calibration mark */

static bool g_calibrated;

/* ADC calibration digital parameter */

static uint16_t g_cal_digit;

/* ADC clock reference */

static uint32_t g_clk_ref;

static mutex_t g_lock = NXMUTEX_INITIALIZER;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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
 * Name: adc_enable_clk
 *
 * Description:
 *   Enable ADC clock.
 *
 * Input Parameters:
 *   NOne
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void adc_enable_clk(void)
{
  irqstate_t flags;

  flags = enter_critical_section();

  if (!g_clk_ref)
    {
      setbits(SENS_SARADC_CLK_EN, SENS_SAR_PERI_CLK_GATE_CONF_REG);
    }

  g_clk_ref++;

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: adc_disable_clk
 *
 * Description:
 *   Disable ADC clock.
 *
 * Input Parameters:
 *   NOne
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void adc_disable_clk(void)
{
  irqstate_t flags;

  flags = enter_critical_section();

  g_clk_ref--;

  if (!g_clk_ref)
    {
      resetbits(SENS_SARADC_CLK_EN, SENS_SAR_PERI_CLK_GATE_CONF_REG);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: adc_set_calibration
 *
 * Description:
 *   Set calibration parameter to ADC hardware.
 *
 * Input Parameters:
 *   data - Calibration parameter
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void adc_set_calibration(uint16_t data)
{
  uint8_t h_data = data >> 8;
  uint8_t l_data = data & 0xff;

  esp_rom_regi2c_write_mask(I2C_ADC, I2C_ADC_HOSTID,
                        I2C_ADC1_INITVAL_H,
                        I2C_ADC1_INITVAL_H_MSB,
                        I2C_ADC1_INITVAL_H_LSB, h_data);

  esp_rom_regi2c_write_mask(I2C_ADC, I2C_ADC_HOSTID,
                        I2C_ADC1_INITVAL_L,
                        I2C_ADC1_INITVAL_L_MSB,
                        I2C_ADC1_INITVAL_L_LSB, l_data);
}

/****************************************************************************
 * Name: adc_samplecfg
 *
 * Description:
 *   Set ADC sampling with given channel.
 *
 * Input Parameters:
 *   channel - Sampling channel number
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void adc_samplecfg(int channel)
{
  uint32_t regval;

  /* set (Frequency division) (inversion adc) */

  regval = getreg32(SENS_SAR_READER1_CTRL_REG);
  regval &= ~(SENS_SAR1_CLK_DIV_M);
  regval |= (1 << SENS_SAR1_CLK_DIV_S);
  putreg32(regval, SENS_SAR_READER1_CTRL_REG);

  /* Enable ADC1, its sampling attenuation */

  regval = getreg32(SENS_SAR_ATTEN1_REG);
  regval &= ~(ADC_ATTEN_DEF << (channel * 2));
  regval |= ADC_ATTEN_DEF << (channel * 2);
  putreg32(regval, SENS_SAR_ATTEN1_REG);

  /* Enable ADC1, its sampling channel and attenuation */

  regval  = getreg32(SENS_SAR_MEAS1_CTRL2_REG);
  regval &= ~(SENS_SAR1_EN_PAD_M | SENS_SAR1_EN_PAD_FORCE_M |
              SENS_MEAS1_START_FORCE_M);
  regval |= ((1 << channel) << SENS_SAR1_EN_PAD_S) |
              SENS_SAR1_EN_PAD_FORCE | SENS_MEAS1_START_FORCE;
  putreg32(regval, SENS_SAR_MEAS1_CTRL2_REG);
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
  uint16_t adc;
  uint32_t regval;

  /* Trigger ADC1 sampling */

  setbits(SENS_MEAS1_START_SAR, SENS_SAR_MEAS1_CTRL2_REG);

  /* Wait until ADC1 sampling is done */

  do
    {
      regval = getreg32(SENS_SAR_MEAS1_CTRL2_REG);
    }
  while (!(regval & SENS_MEAS1_DONE_SAR_M));

  regval = getreg32(SENS_SAR_MEAS1_CTRL2_REG) & ADC_VAL_MASK;
  ainfo("SENS_MEAS1_DATA_SAR adc_read: %d\n", regval);

  /* Disable ADC sampling */

  resetbits(SENS_MEAS1_START_SAR, SENS_SAR_MEAS1_CTRL2_REG);

  return regval;
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

      esp_rom_regi2c_write_mask(I2C_ADC, I2C_ADC_HOSTID,
                            I2C_ADC1_DEF, I2C_ADC1_DEF_MSB,
                            I2C_ADC1_DEF_LSB, 1);

      /* Start sampling */

      adc_samplecfg(ADC_CAL_CHANNEL);

      /* Enable internal connect GND (for calibration). */

      esp_rom_regi2c_write_mask(I2C_ADC, I2C_ADC_HOSTID,
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

      esp_rom_regi2c_write_mask(I2C_ADC, I2C_ADC_HOSTID,
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

  ainfo("calibration read_efuse g_cal_digit: %" PRIu16 "\n", g_cal_digit);
}

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

  ret = nxmutex_lock(&g_lock);
  if (ret < 0)
    {
      aerr("Failed to lock ret=%d\n", ret);
      return;
    }

  adc_samplecfg(priv->channel);
  value = adc_read();

  adc = (int32_t)(value * (UINT16_MAX * ADC_CAL_VOL_DEF / g_cal_digit) /
                  UINT16_MAX);

  priv->cb->au_receive(dev, priv->channel, adc);

  ainfo("channel: %" PRIu8 ", voltage: %" PRIu32 " mV\n", priv->channel,
        adc);

  nxmutex_unlock(&g_lock);
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

  /* Reset ADC hardware */

  adc_enable_clk();

  adc_disable_clk();

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
  int ret;
  uint32_t regval;
  struct adc_chan_s *priv = (struct adc_chan_s *)dev->ad_priv;

  ainfo("channel: %" PRIu8 "\n", priv->channel);

  /* Do nothing when the ADC device is already set up */

  if (priv->ref > 0)
    {
      priv->ref++;
      return OK;
    }

  /* Enable ADC clock */

  adc_enable_clk();

  /* Disable GPIO input and output */

  ainfo("pin: %" PRIu8 "\n", priv->pin);

  esp32s3_configgpio(priv->pin, INPUT | FUNCTION_1);

  /* Start calibration only once  */

  ret = nxmutex_lock(&g_lock);
  if (ret < 0)
    {
      adc_disable_clk();
      aerr("Failed to lock ret=%d\n", ret);
      return ret;
    }

  if (!g_calibrated)
    {
      adc_calibrate();
      g_calibrated = true;
    }

  nxmutex_unlock(&g_lock);

  /* The ADC device is ready */

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

          ret = priv->channel;
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

      if (!priv->ref)
        {
          adc_rxint(dev, false);

          /* Disable ADC clock */

          adc_disable_clk();
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_adc_init
 *
 * Description:
 *   Initialize the ADC.
 *
 * Input Parameters:
 *   adc_index - ADC channel number
 *   dev       - pointer to device structure used by the driver
 *
 * Returned Value:
 *
 ****************************************************************************/

void esp32s3_adc_init(int adc_index, struct adc_dev_s *dev)
{
  ainfo("ADC index: %" PRIu8 "\n",  adc_index);

  dev->ad_ops = &g_adcops;

  switch (adc_index)
    {
#if defined(CONFIG_ESP32S3_ADC1_CHANNEL0)
      case 0:
          dev->ad_priv = &g_adc1_chan0;
        break;
#endif

#if defined(CONFIG_ESP32S3_ADC1_CHANNEL1)
      case 1:
          dev->ad_priv = &g_adc1_chan1;
        break;
#endif

#if defined(CONFIG_ESP32S3_ADC1_CHANNEL2)
      case 2:
          dev->ad_priv = &g_adc1_chan2;
        break;
#endif

#if defined(CONFIG_ESP32S3_ADC1_CHANNEL3)
      case 3:
          dev->ad_priv = &g_adc1_chan3;
        break;
#endif

#if defined(CONFIG_ESP32S3_ADC1_CHANNEL4)
      case 4:
          dev->ad_priv = &g_adc1_chan4;
        break;
#endif

#if defined(CONFIG_ESP32S3_ADC1_CHANNEL5)
      case 5:
          dev->ad_priv = &g_adc1_chan5;
        break;
#endif

#if defined(CONFIG_ESP32S3_ADC1_CHANNEL6)
      case 6:
          dev->ad_priv = &g_adc1_chan6;
        break;
#endif

#if defined(CONFIG_ESP32S3_ADC1_CHANNEL7)
      case 7:
          dev->ad_priv = &g_adc1_chan7;
        break;
#endif

#if defined(CONFIG_ESP32S3_ADC1_CHANNEL8)
      case 8:
          dev->ad_priv = &g_adc1_chan8;
        break;
#endif

#if defined(CONFIG_ESP32S3_ADC1_CHANNEL9)
      case 9:
          dev->ad_priv = &g_adc1_chan9;
        break;
#endif

      default:
        {
          aerr("ERROR: No ADC interface defined\n");
        }
    }
}
