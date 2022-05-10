/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_adc.c
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
#include <assert.h>
#include <debug.h>

#include <nuttx/semaphore.h>

#include "riscv_internal.h"
#include "esp32c3.h"
#include "esp32c3_gpio.h"
#include "esp32c3_adc.h"

#include "hardware/esp32c3_system.h"
#include "hardware/esp32c3_efuse.h"
#include "hardware/esp32c3_saradc.h"
#include "hardware/esp32c3_gpio_sigmap.h"
#include "hardware/regi2c_ctrl.h"
#include "hardware/regi2c_saradc.h"

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

#define ADC_CAL_BASE_REG        EFUSE_RD_SYS_DATA_PART1_0_REG

#define ADC_CAL_VER_OFF         (128)
#define ADC_CAL_VER_LEN         (3)

#define ADC_CAL_DATA_LEN        (10)
#define ADC_CAL_DATA_COMP       (1000)

#define ADC_CAL_VOL_LEN         (10)

/* ADC input voltage attenuation, this affects measuring range */

#define ADC_ATTEN_DB_0          (0)     /* Vmax = 800 mV  */
#define ADC_ATTEN_DB_2_5        (1)     /* Vmax = 1100 mV */
#define ADC_ATTEN_DB_6          (2)     /* Vmax = 1350 mV */
#define ADC_ATTEN_DB_11         (3)     /* Vmax = 2600 mV */

/* ADC attenuation */

#if defined(CONFIG_ESP32C3_ADC_VOL_750)
#  define ADC_ATTEN_DEF         ADC_ATTEN_DB_0

#  define ADC_CAL_DATA_OFF      (148)
#  define ADC_CAL_VOL_OFF       (188)

#  define ADC_CAL_VOL_DEF       (400)
#elif defined(CONFIG_ESP32C3_ADC_VOL_1050)
#  define ADC_ATTEN_DEF         ADC_ATTEN_DB_2_5

#  define ADC_CAL_DATA_OFF      (158)
#  define ADC_CAL_VOL_OFF       (198)

#  define ADC_CAL_VOL_DEF       (550)
#elif defined(CONFIG_ESP32C3_ADC_VOL_1300)
#  define ADC_ATTEN_DEF         ADC_ATTEN_DB_6

#  define ADC_CAL_DATA_OFF      (168)
#  define ADC_CAL_VOL_OFF       (208)

#  define ADC_CAL_VOL_DEF       (750)
#elif defined(CONFIG_ESP32C3_ADC_VOL_2500)
#  define ADC_ATTEN_DEF         ADC_ATTEN_DB_11

#  define ADC_CAL_DATA_OFF      (178)
#  define ADC_CAL_VOL_OFF       (218)

#  define ADC_CAL_VOL_DEF       (1370)
#endif

#define ADC_WORK_DELAY          (1)

#ifndef MIN
#  define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

#ifndef MAX
#  define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

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

#ifdef CONFIG_ESP32C3_ADC1_CHANNEL0
static struct adc_chan_s g_adc1_chan0 =
{
  .channel = 0,
  .pin     = 0
};

static struct adc_dev_s g_adc1_chan0_dev =
{
  .ad_ops      = &g_adcops,
  .ad_priv     = &g_adc1_chan0
};
#endif

#ifdef CONFIG_ESP32C3_ADC1_CHANNEL1
static struct adc_chan_s g_adc1_chan1 =
{
  .channel = 1,
  .pin     = 1
};

static struct adc_dev_s g_adc1_chan1_dev =
{
  .ad_ops      = &g_adcops,
  .ad_priv     = &g_adc1_chan1,
};
#endif

#ifdef CONFIG_ESP32C3_ADC1_CHANNEL2
static struct adc_chan_s g_adc1_chan2 =
{
  .channel = 2,
  .pin     = 2
};

static struct adc_dev_s g_adc1_chan2_dev =
{
  .ad_ops      = &g_adcops,
  .ad_priv     = &g_adc1_chan2,
};
#endif

#ifdef CONFIG_ESP32C3_ADC1_CHANNEL3
static struct adc_chan_s g_adc1_chan3 =
{
  .channel = 3,
  .pin     = 3
};

static struct adc_dev_s g_adc1_chan3_dev =
{
  .ad_ops      = &g_adcops,
  .ad_priv     = &g_adc1_chan3,
};
#endif

#ifdef CONFIG_ESP32C3_ADC1_CHANNEL4
static struct adc_chan_s g_adc1_chan4 =
{
  .channel = 4,
  .pin     = 4
};

static struct adc_dev_s g_adc1_chan4_dev =
{
  .ad_ops      = &g_adcops,
  .ad_priv     = &g_adc1_chan4,
};
#endif

/* ADC calibration mark */

static bool g_calibrated;

/* ADC calibration digital parameter */

static uint16_t g_cal_digit;

/* ADC clock reference */

static uint32_t g_clk_ref;

static sem_t g_sem_excl = SEM_INITIALIZER(1);

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
      setbits(SYSTEM_APB_SARADC_CLK_EN, SYSTEM_PERIP_CLK_EN0_REG);
      resetbits(SYSTEM_APB_SARADC_RST, SYSTEM_PERIP_RST_EN0_REG);
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
      setbits(SYSTEM_APB_SARADC_RST, SYSTEM_PERIP_RST_EN0_REG);
      resetbits(SYSTEM_APB_SARADC_CLK_EN, SYSTEM_PERIP_CLK_EN0_REG);
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

  rom_i2c_writereg_mask(I2C_ADC, I2C_ADC_HOSTID,
                        I2C_ADC1_INITVAL_H,
                        I2C_ADC1_INITVAL_H_MSB,
                        I2C_ADC1_INITVAL_H_LSB, h_data);

  rom_i2c_writereg_mask(I2C_ADC, I2C_ADC_HOSTID,
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

  /* Enable ADC1, its sampling channel and attenuation */

  regval  = getreg32(APB_SARADC_ONETIME_SAMPLE_REG);
  regval &= ~(APB_SARADC1_ONETIME_SAMPLE | APB_SARADC2_ONETIME_SAMPLE |
              APB_SARADC_ONETIME_CHANNEL_M | APB_SARADC_ONETIME_ATTEN_M);
  regval |= (channel << APB_SARADC_ONETIME_CHANNEL_S) |
            (ADC_ATTEN_DEF << APB_SARADC_ONETIME_ATTEN_S) |
            APB_SARADC1_ONETIME_SAMPLE;
  putreg32(regval, APB_SARADC_ONETIME_SAMPLE_REG);
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

  /* Trigger ADC sampling */

  setbits(APB_SARADC_ONETIME_START, APB_SARADC_ONETIME_SAMPLE_REG);

  /* Wait until ADC1 sampling is done */

  do
    {
      regval = getreg32(APB_SARADC_INT_ST_REG);
    }
  while (!(regval & APB_SARADC_ADC1_DONE_INT_ST));

  adc = getreg32(APB_SARADC_1_DATA_STATUS_REG) & ADC_VAL_MASK;

  /* Disable ADC sampling */

  resetbits(APB_SARADC_ONETIME_START, APB_SARADC_ONETIME_SAMPLE_REG);

  /* Clear ADC1 sampling done interrupt bit */

  setbits(APB_SARADC_ADC1_DONE_INT_CLR, APB_SARADC_INT_CLR_REG);

  return adc;
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
      return ;
    }

  adc_samplecfg(priv->channel);
  value = adc_read();

  adc = (int32_t)(value * (UINT16_MAX * ADC_CAL_VOL_DEF / g_cal_digit) /
                  UINT16_MAX);

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

  esp32c3_configgpio(priv->pin, INPUT);

  /* Config ADC hardware */

  regval = APB_SARADC_SAR_CLK_GATED_M | APB_SARADC_XPD_SAR_FORCE_M;
  setbits(regval, APB_SARADC_CTRL_REG);

  regval = APB_SARADC_ADC1_DONE_INT_ENA;
  setbits(regval, APB_SARADC_INT_ENA_REG);

  /* Start calibration only once  */

  ret = nxsem_wait(&g_sem_excl);
  if (ret < 0)
    {
      adc_disable_clk();
      aerr("Failed to wait sem ret=%d\n", ret);
      return ret;
    }

  if (!g_calibrated)
    {
      adc_calibrate();
      g_calibrated = true;
    }

  nxsem_post(&g_sem_excl);

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
 * Name: esp32c3_adc_init
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

struct adc_dev_s *esp32c3_adc_init(int channel)
{
  struct adc_dev_s *dev;

  ainfo("ADC channel: %" PRIu8 "\n", channel);

  switch (channel)
    {
#ifdef CONFIG_ESP32C3_ADC1_CHANNEL0
      case 0:
        dev = &g_adc1_chan0_dev;
        break;
#endif

#ifdef CONFIG_ESP32C3_ADC1_CHANNEL1
      case 1:
        dev = &g_adc1_chan1_dev;
        break;
#endif

#ifdef CONFIG_ESP32C3_ADC1_CHANNEL2
      case 2:
        dev = &g_adc1_chan2_dev;
        break;
#endif

#ifdef CONFIG_ESP32C3_ADC1_CHANNEL3
      case 3:
        dev = &g_adc1_chan3_dev;
        break;
#endif

#ifdef CONFIG_ESP32C3_ADC1_CHANNEL4
      case 4:
        dev = &g_adc1_chan4_dev;
        break;
#endif

      default:
        {
          aerr("ERROR: No ADC interface defined\n");
          return NULL;
        }
    }

  return dev;
}
