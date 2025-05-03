/****************************************************************************
 * arch/xtensa/src/common/espressif/esp_adc.c
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

#include <nuttx/analog/adc.h>
#include <nuttx/spinlock.h>

#include "esp_adc.h"

#include "adc_cali_interface.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_private/adc_share_hw_ctrl.h"
#include "esp_private/esp_sleep_internal.h"
#include "esp_private/periph_ctrl.h"
#include "esp_private/sar_periph_ctrl.h"
#include "hal/adc_types.h"
#include "hal/adc_oneshot_hal.h"
#include "hal/adc_ll.h"
#include "hal/sar_ctrl_ll.h"
#include "soc/adc_periph.h"
#include "soc/periph_defs.h"
#include "esp_clk_tree.h"

#ifdef CONFIG_ARCH_CHIP_ESP32
#include "esp32_gpio.h"
#elif defined(CONFIG_ARCH_CHIP_ESP32S2)
#include "esp32s2_gpio.h"
#elif defined(CONFIG_ARCH_CHIP_ESP32S3)
#include "esp32s3_gpio.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_ADC_1_MODE_CONTINUOUS
#  error "Continuous mode not implemented"
#endif

#ifdef CONFIG_ARCH_CHIP_ESP32
#  define esp_configgpio       esp32_configgpio
#  define GPIO_ADC_FUNCTION    FUNCTION_3
#endif
#ifdef CONFIG_ARCH_CHIP_ESP32S2
#  define esp_configgpio       esp32s2_configgpio
#  define GPIO_ADC_FUNCTION    FUNCTION_2
#endif
#ifdef CONFIG_ARCH_CHIP_ESP32S3
#  define esp_configgpio       esp32s3_configgpio
#  define GPIO_ADC_FUNCTION    FUNCTION_2
#endif

#define ESP_ADC_BITWIDTH_DEFAULT ADC_BITWIDTH_DEFAULT

/* Default internal reference voltage */

#define ADC_ESP32_DEFAULT_VREF_INTERNAL 1100

#define ADC_GET_IO_NUM(unit, channel) (adc_channel_io_map[unit][channel])
#define COUNT_NON_ZERO(arr, len) ({  \
    size_t _count = 0;               \
    for (size_t _i = 0; _i < (len); _i++) \
        if ((arr)[_i] != 0) _count++; \
    _count;                           \
})

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum esp_adc_mode_e
{
  ESP_ADC_MODE_ONE_SHOT = 0,
  ESP_ADC_MODE_CONTINUOUS,
};

struct esp_adc_dev_common_s
{
  spinlock_t esp_adc_spinlock;
  bool initialized;             /* ADC peripheral initialized */
};

struct esp_adc_oneshot_ch_s
{
  uint8_t channel;                 /* Channel number */
  adc_cali_handle_t  cali_handle;  /* Handle for calibration */
  bool calibrated;                 /* Channel has been calibrated */
};

/* One-shot ADC device struct */

struct esp_adc_oneshot_s
{
  adc_oneshot_hal_ctx_t hal;   /* ADC unit low-level context */
  struct esp_adc_oneshot_ch_s ch_list[SOC_ADC_MAX_CHANNEL_NUM];
};

/* Continuous ADC device struct */

struct esp_adc_continuous_s
{
};

/* Generic ADC unit struct to be used by one-shot or continuous mode */

struct esp_adc_dev_s
{
  struct adc_dev_s            *upper_dev;  /* Upper-half ADC reference */
  const struct adc_callback_s *cb;         /* Upper driver callback */

  struct esp_adc_dev_common_s *common;     /* Common ADC driver data */

  enum esp_adc_mode_e mode;  /* ADC mode */
  adc_atten_t atten_mode;    /* Attenuation parameter */
  uint32_t atten_k;          /* Attenuation factor */
  uint8_t channels;          /* Total channels for this ADC */
  uint8_t unit;              /* ADC unit number */
  bool    initialized;       /* ADC unit initialized */

  union
    {
      struct esp_adc_oneshot_s os_dev;
      struct esp_adc_continuous_s cnt_dev;
    };
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void esp_adc_reset(struct adc_dev_s *dev);
static void esp_adc_shutdown(struct adc_dev_s *dev);
static void esp_adc_rxint(struct adc_dev_s *dev, bool enable);
static int esp_adc_setup(struct adc_dev_s *dev);
static int esp_adc_ioctl(struct adc_dev_s *dev, int cmd,
                         unsigned long arg);
static int esp_adc_bind(struct adc_dev_s *dev,
                        const struct adc_callback_s *callback);

static int esp_adc_oneshot_read(struct adc_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct adc_ops_s g_adcops =
{
  .ao_bind        = esp_adc_bind,
  .ao_reset       = esp_adc_reset,
  .ao_setup       = esp_adc_setup,
  .ao_shutdown    = esp_adc_shutdown,
  .ao_rxint       = esp_adc_rxint,
  .ao_ioctl       = esp_adc_ioctl,
};

static struct esp_adc_dev_common_s g_adc_common =
{
  .esp_adc_spinlock = SP_UNLOCKED,
};

#ifdef CONFIG_ESPRESSIF_ADC_1
static struct esp_adc_dev_s g_adcpriv1 =
{
  .common = &g_adc_common,
  .initialized = false,
  .unit = ADC_UNIT_1,
  .atten_mode = CONFIG_ESPRESSIF_ADC_1_ATTENUATION,
#ifdef CONFIG_ESPRESSIF_ADC_1_MODE_ONE_SHOT
  .mode = ESP_ADC_MODE_ONE_SHOT,
#else
  .mode = ESP_ADC_MODE_CONTINUOUS,
#endif
};

static struct adc_dev_s g_adcdev1 =
{
  .ad_ops      = &g_adcops,
  .ad_priv     = &g_adcpriv1,
};
#endif  /* CONFIG_ESPRESSIF_ADC_1 */

#ifdef CONFIG_ESPRESSIF_ADC_2
static struct esp_adc_dev_s g_adcpriv2 =
{
  .common = &g_adc_common,
  .initialized = false,
  .unit = ADC_UNIT_2,
  .atten_mode = CONFIG_ESPRESSIF_ADC_2_ATTENUATION,
#ifdef CONFIG_ESPRESSIF_ADC_2_MODE_ONE_SHOT
  .mode = ESP_ADC_MODE_ONE_SHOT,
#else
  .mode = ESP_ADC_MODE_CONTINUOUS,
#endif
};

static struct adc_dev_s g_adcdev2 =
{
  .ad_ops      = &g_adcops,
  .ad_priv     = &g_adcpriv2,
};
#endif  /* CONFIG_ESPRESSIF_ADC_2 */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_adc_bind
 *
 * Description:
 *   This function binds the upper-half driver callback to the ADC device.
 *
 * Input Parameters:
 *   dev      - Pointer to the ADC device structure.
 *   callback - Pointer to the upper-half driver callback structure.
 *
 * Returned Value:
 *   Returns OK on successful binding.
 *
 ****************************************************************************/

static int esp_adc_bind(struct adc_dev_s *dev,
                        const struct adc_callback_s *callback)
{
  struct esp_adc_dev_s *priv = (struct esp_adc_dev_s *)dev->ad_priv;

  DEBUGASSERT(priv);

  priv->cb = callback;

  return OK;
}

/****************************************************************************
 * Name: esp_adc_reset
 *
 * Description:
 *   This function resets the ADC device.
 *
 * Input Parameters:
 *   dev - Pointer to the ADC device structure.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void esp_adc_reset(struct adc_dev_s *dev)
{
  struct esp_adc_dev_s *priv = (struct esp_adc_dev_s *)dev->ad_priv;

  DEBUGASSERT(priv);
}

/****************************************************************************
 * Name: esp_adc_setup
 *
 * Description:
 *   This function sets up the ADC device.
 *
 * Input Parameters:
 *   dev - Pointer to the ADC device structure.
 *
 * Returned Value:
 *   Returns OK on successful setup; ERROR if the peripheral is already
 *   initialized.
 *
 ****************************************************************************/

static int esp_adc_setup(FAR struct adc_dev_s *dev)
{
  struct esp_adc_dev_s *priv = (struct esp_adc_dev_s *) dev->ad_priv;
  int ret = OK;

  DEBUGASSERT(priv);

  if (priv->common->initialized)
    {
      awarn("peripheral already initialized\n");
      return ERROR;
    }

  priv->common->initialized = true;

  return ret;
}

/****************************************************************************
 * Name: esp_adc_shutdown
 *
 * Description:
 *   This function shuts down the ADC device.
 *
 * Input Parameters:
 *   dev - Pointer to the ADC device structure.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void esp_adc_shutdown(FAR struct adc_dev_s *dev)
{
  struct esp_adc_dev_s *priv = (struct esp_adc_dev_s *) dev->ad_priv;

  DEBUGASSERT(priv);

  if (!priv->common->initialized)
    {
      return;
    }

  priv->common->initialized = false;
}

/****************************************************************************
 * Name: esp_adc_rxint
 *
 * Description:
 *   This function enables or disables ADC receive interrupts.
 *
 * Input Parameters:
 *   dev    - Pointer to the ADC device structure.
 *   enable - Boolean indicating whether to enable (true) or disable (false)
 *            the receive interrupts.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void esp_adc_rxint(FAR struct adc_dev_s *dev, bool enable)
{
  struct esp_adc_dev_s *priv = (struct esp_adc_dev_s *) dev->ad_priv;

  DEBUGASSERT(priv);
}

/****************************************************************************
 * Name: esp_adc_ioctl
 *
 * Description:
 *   This function handles ADC ioctl commands.
 *
 * Input Parameters:
 *   dev  - Pointer to the ADC device structure.
 *   cmd  - The ioctl command.
 *   arg  - The argument for the ioctl command.
 *
 * Returned Value:
 *   Returns the number of channels on ANIOC_GET_NCHANNELS command; a negated
 *   errno value is returned on any failure.
 *
 ****************************************************************************/

static int esp_adc_ioctl(struct adc_dev_s *dev, int cmd,
                         unsigned long arg)
{
  struct esp_adc_dev_s *priv = (struct esp_adc_dev_s *)dev->ad_priv;
  int i;
  int ret = OK;

  DEBUGASSERT(priv);

  switch (cmd)
    {
      case ANIOC_TRIGGER:
        esp_adc_oneshot_read(dev);
        break;

      case ANIOC_GET_NCHANNELS:
        ret = priv->channels;
        break;

      default:
        aerr("ERROR: Unknown cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: esp_adc_oneshot_read
 *
 * Description:
 *   This function reads data from the ADC device in one-shot mode.
 *
 * Input Parameters:
 *   dev - Pointer to the ADC device structure.
 *
 * Returned Value:
 *   Returns OK on successful read; ERROR if the ADC is not initialized.
 *
 ****************************************************************************/

static int esp_adc_oneshot_read(struct adc_dev_s *dev)
{
  struct esp_adc_dev_s *priv = (struct esp_adc_dev_s *)dev->ad_priv;
  adc_oneshot_hal_ctx_t *hal = &priv->os_dev.hal;
  struct esp_adc_oneshot_ch_s *ch;
  irqstate_t flags;
  int raw_value;
  int voltage;
  int i;
  int ret = OK;
#ifndef CONFIG_ARCH_CHIP_ESP32
    adc_atten_t atten;
#endif

  DEBUGASSERT(priv);
  DEBUGASSERT(priv->mode == ESP_ADC_MODE_ONE_SHOT);

  if (!priv->initialized)
    {
      aerr("ADC %d not initialized\n", priv->unit);
      return ERROR;
    }

  /* Read all configured channels */

  for (i = 0; i < priv->channels; i++)
    {
      ch = &priv->os_dev.ch_list[i];
      flags = spin_lock_irqsave(&g_adc_common.esp_adc_spinlock);

      /* Read the ADC value */

      adc_oneshot_hal_setup(hal, ch->channel);
#ifndef CONFIG_ARCH_CHIP_ESP32
      if (ch->calibrated)
        {
          atten = adc_ll_get_atten(priv->unit, ch->channel);
          adc_hal_calibration_init(priv->unit);
          adc_set_hw_calibration_code(priv->unit, atten);
        }
#endif

      ret = adc_oneshot_hal_convert(hal, &raw_value);
      if (!ret)
        {
          aerr("invalid one-shot ADC read\n");
        }

      /* Apply calibration if possible */

      if (ch->calibrated)
        {
          ret = adc_cali_raw_to_voltage(ch->cali_handle,
                                        raw_value,
                                        &voltage);
          if (ret != OK)
            {
              voltage = 0;
              aerr("apply calibration failed\n");
            }
        }
      else
        {
          voltage = priv->atten_k * raw_value / 4095;
        }

      /* Inform upper layer that new data for a channel is available */

      priv->cb->au_receive(dev, ch->channel, (int32_t)voltage);

      spin_unlock_irqrestore(&g_adc_common.esp_adc_spinlock, flags);

      ainfo("read adc %d ch %d (cali: %d): value %" PRId32 " (raw %d)\n",
            priv->unit, ch->channel, ch->calibrated,
            (int32_t)voltage, raw_value);
      usleep(1000);
    }

  return OK;
}

/****************************************************************************
 * Name: esp_adc_oneshot_new_unit
 *
 * Description:
 *   This function initializes a new ADC unit in one-shot mode.
 *
 * Input Parameters:
 *   dev - Pointer to the ADC device structure.
 *
 * Returned Value:
 *   Returns OK on successful initialization; ERROR if the ADC is already
 *   initialized.
 *
 ****************************************************************************/

static int esp_adc_oneshot_new_unit(struct adc_dev_s *dev)
{
  struct esp_adc_dev_s *priv = (struct esp_adc_dev_s *)dev->ad_priv;
  adc_oneshot_hal_ctx_t *hal = &priv->os_dev.hal;
  adc_oneshot_hal_cfg_t config;
  irqstate_t flags;
  uint32_t clk_src_freq_hz = 0;

  DEBUGASSERT(priv);
  DEBUGASSERT(priv->mode == ESP_ADC_MODE_ONE_SHOT);

  if (priv->initialized)
    {
      aerr("ADC %d already initialized\n", priv->unit);
      return ERROR;
    }

  flags = spin_lock_irqsave(&g_adc_common.esp_adc_spinlock);

  esp_clk_tree_src_get_freq_hz(ADC_DIGI_CLK_SRC_DEFAULT,
                               ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED,
                               &clk_src_freq_hz);

  config.unit = priv->unit;
  config.work_mode = ADC_HAL_SINGLE_READ_MODE;
  config.clk_src = ADC_DIGI_CLK_SRC_DEFAULT;
  config.clk_src_freq_hz = clk_src_freq_hz;

  adc_oneshot_hal_init(hal, &config);

  /* Enable peripheral and power ADC */

  adc_apb_periph_claim();

  sar_periph_ctrl_adc_oneshot_power_acquire();

  /* Acquire oneshot mode power */

#ifndef ARCH_CHIP_ESP32S2
  sar_ctrl_ll_set_power_mode(SAR_CTRL_LL_POWER_ON);
#endif

  priv->initialized = true;

  spin_unlock_irqrestore(&g_adc_common.esp_adc_spinlock, flags);

  ainfo("unit %d freq %" PRIu32 "\n", priv->unit, clk_src_freq_hz);
  return OK;
}

/****************************************************************************
 * Name: esp_adc_oneshot_config_channel
 *
 * Description:
 *   This function configures a specific channel for the ADC device in
 *   one-shot mode.
 *
 * Input Parameters:
 *   dev     - Pointer to the ADC device structure.
 *   channel - The channel number to configure.
 *
 * Returned Value:
 *   Returns OK on successful configuration; a negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

static int esp_adc_oneshot_config_channel(struct adc_dev_s *dev,
                                          uint8_t channel)
{
  struct esp_adc_dev_s *priv = (struct esp_adc_dev_s *)dev->ad_priv;
  adc_oneshot_hal_ctx_t *hal = &priv->os_dev.hal;
  adc_oneshot_hal_chan_cfg_t config;
  irqstate_t flags;
  int gpio;
  int ret;

  DEBUGASSERT(priv);
  DEBUGASSERT(priv->mode == ESP_ADC_MODE_ONE_SHOT);

  config.atten = priv->atten_mode;
  config.bitwidth = ESP_ADC_BITWIDTH_DEFAULT;

  /* Configure GPIO for ADC */

  gpio = ADC_GET_IO_NUM(priv->unit, channel);
  ret = esp_configgpio(gpio, GPIO_ADC_FUNCTION);
  if (ret < 0)
    {
      aerr("ERROR: Failed to configure GPIO %d\n", gpio);
      return ret;
    }

  /* Config ADC channel */

  flags = spin_lock_irqsave(&g_adc_common.esp_adc_spinlock);
  adc_oneshot_hal_channel_config(hal, &config, channel);
  spin_unlock_irqrestore(&g_adc_common.esp_adc_spinlock, flags);

  ainfo("init adc unit %u, ch %u (gpio %d), atten %d, bitwidth %d",
        priv->unit, channel, gpio, config.atten, config.bitwidth);

  return OK;
}

/****************************************************************************
 * Name: esp_adc_calibrate
 *
 * Description:
 *   This function calibrates the ADC device.
 *
 * Input Parameters:
 *   dev - Pointer to the ADC device structure.
 *
 * Returned Value:
 *   Returns OK on successful calibration; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

static int esp_adc_calibrate(struct adc_dev_s *dev)
{
  struct esp_adc_dev_s *priv = (struct esp_adc_dev_s *)dev->ad_priv;
  adc_cali_handle_t *handle;
#ifdef ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
  adc_cali_curve_fitting_config_t cali_config;
#else
  adc_cali_line_fitting_config_t cali_config;
#endif
  int i;
  int ret = OK;

  DEBUGASSERT(priv);
  DEBUGASSERT(priv->mode == ESP_ADC_MODE_ONE_SHOT);

  /* Init calibration for this ADC unit */

#ifndef CONFIG_ARCH_CHIP_ESP32
  adc_hal_calibration_init(priv->unit);
  adc_calc_hw_calibration_code(priv->unit, priv->atten_mode);
#endif

  cali_config.unit_id = priv->unit;
  cali_config.atten = priv->atten_mode;
  cali_config.bitwidth = ESP_ADC_BITWIDTH_DEFAULT;

  for (i = 0; i < priv->channels; i++)
    {
      handle = &priv->os_dev.ch_list[i].cali_handle;
#ifdef ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
      cali_config.chan = priv->os_dev.ch_list[i].channel;

      ainfo("curve fitting unit %d, chan %u, atten %u, bitwidth %u\n",
            cali_config.unit_id, cali_config.chan, cali_config.atten,
            cali_config.bitwidth);

      /* Make sure calibration handle is clear */

      if (*handle != NULL)
        {
          adc_cali_delete_scheme_curve_fitting(*handle);
        }

      ret = adc_cali_create_scheme_curve_fitting(&cali_config, handle);
#endif
#ifdef ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
#ifdef CONFIG_ARCH_CHIP_ESP32
      cali_config.default_vref = ADC_ESP32_DEFAULT_VREF_INTERNAL;
#endif
      ainfo("line fitting unit %d, atten %u, bitwidth %u\n",
            cali_config.unit_id, cali_config.atten, cali_config.bitwidth);

      /* Make sure calibration handle is clear */

      if (*handle != NULL)
        {
          adc_cali_delete_scheme_line_fitting(*handle);
        }

      ret = adc_cali_create_scheme_line_fitting(&cali_config, handle);
#endif
      if (ret == OK)
        {
          priv->os_dev.ch_list[i].calibrated = true;
        }
      else
        {
          aerr("calibration failed\n");
        }
    }

  /* This attenuation constant is used when calibration fails,
   * returning a voltage obtained from simple calibration based
   * on: Vdata = Vref * raw_data / 4095
   */

  switch (priv->atten_mode)
    {
      case ADC_ATTEN_DB_12:
        priv->atten_k = 4400;
        break;

      case ADC_ATTEN_DB_6:
        priv->atten_k = 2200;
        break;

      case ADC_ATTEN_DB_2_5:
        priv->atten_k = 1470;
        break;

      case ADC_ATTEN_DB_0:
        priv->atten_k = 1100;
        break;

      default:
        priv->atten_k = 0;
        aerr("invalid attenuation mode\n");
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: esp_adc_initialize
 *
 * Description:
 *   This function initializes the specified ADC device with the provided
 *   configuration.
 *
 * Input Parameters:
 *   adc_num      - The ADC unit number.
 *   channel_list - List of channels to be configured for the ADC unit.
 *
 * Returned Value:
 *   Returns a valid pointer to the ADC device structure on success; NULL on
 *   any failure.
 *
 ****************************************************************************/

struct adc_dev_s *esp_adc_initialize(int adc_num,
                                     const uint8_t *channel_list)
{
  struct adc_dev_s *dev;
  struct esp_adc_dev_s *priv;
  uint8_t channel;
  int gpio;
  int i;
  int ret;

  ainfo("initialize SAR ADC %d\n", adc_num);

  switch (adc_num)
    {
      case 1:
        {
#ifdef CONFIG_ESPRESSIF_ADC_1
          dev = &g_adcdev1;
          priv = &g_adcpriv1;
          break;
#endif
        }

      case 2:
        {
#ifdef CONFIG_ESPRESSIF_ADC_2
          dev = &g_adcdev2;
          priv = &g_adcpriv2;
#endif
          break;
        }

      default:
        {
          aerr("ERROR: Unsupported ADC number: %d\n", adc_num);
          return NULL;
        }
    }

  /* Get number of channels used. ESP32 is an exception since unit 1
   * has 8 channels instead of 10.
   */

#ifdef CONFIG_ARCH_CHIP_ESP32
  if (priv->unit == ADC_UNIT_1)
    {
      priv->channels = COUNT_NON_ZERO(channel_list,
                                      SOC_ADC_MAX_CHANNEL_NUM - 2);
    }
  else
    {
      priv->channels = COUNT_NON_ZERO(channel_list, SOC_ADC_MAX_CHANNEL_NUM);
    }
#else
  priv->channels = COUNT_NON_ZERO(channel_list, SOC_ADC_MAX_CHANNEL_NUM);
#endif

  if (priv->channels == 0)
    {
      aerr("ERROR: No channels configured\n");
      return NULL;
    }

  /* Setup this ADC unit to one-shot mode */

  esp_adc_oneshot_new_unit(dev);

  /* Configure channels that will be used for this ADC unit. */

  for (i = 0; i < priv->channels; i++)
    {
      /* Decrement channel number by 1 to get correct index */

      channel = channel_list[i] - 1;
      priv->os_dev.ch_list[i].channel = channel;

      esp_adc_oneshot_config_channel(dev, channel);
    }

  esp_adc_calibrate(dev);

  return dev;
}
