/****************************************************************************
 * arch/xtensa/src/common/espressif/esp_sdm.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <sys/param.h>
#include <sys/types.h>
#include <syslog.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/analog/dac.h>
#include <nuttx/mutex.h>
#include <nuttx/nuttx.h>
#include <nuttx/kmalloc.h>
#include <nuttx/spinlock.h>

#include "esp_sdm.h"
#if defined(CONFIG_ARCH_CHIP_ESP32S3)
#include "esp32s3_gpio.h"
#include "hardware/esp32s3_soc.h"
#elif defined(CONFIG_ARCH_CHIP_ESP32S2)
#include "esp32s2_gpio.h"
#include "hardware/esp32s2_soc.h"
#elif defined(CONFIG_ARCH_CHIP_ESP32)
#include "esp32_gpio.h"
#include "hardware/esp32_soc.h"
#endif

#include "esp_clk_tree.h"
#include "hal/sdm_hal.h"
#include "hal/sdm_ll.h"
#include "soc/sdm_periph.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_ARCH_CHIP_ESP32S3)
#define esp_configgpio            esp32s3_configgpio
#define esp_gpio_matrix_in        esp32s3_gpio_matrix_in
#define esp_gpio_matrix_out       esp32s3_gpio_matrix_out
#elif defined(CONFIG_ARCH_CHIP_ESP32S2)
#define esp_configgpio            esp32s2_configgpio
#define esp_gpio_matrix_in        esp32s2_gpio_matrix_in
#define esp_gpio_matrix_out       esp32s2_gpio_matrix_out
#elif defined(CONFIG_ARCH_CHIP_ESP32)
#define esp_configgpio            esp32_configgpio
#define esp_gpio_matrix_in        esp32_gpio_matrix_in
#define esp_gpio_matrix_out       esp32_gpio_matrix_out
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp_sdm_channel_priv_s
{
  int group_id;             /* Group number */
  int chan_id;              /* Channel number */
  int gpio_num;             /* GPIO number for output */
  uint32_t sample_rate_hz;  /* Over sample rate in Hz, it determines the frequency of the carrier pulses */
  spinlock_t spinlock;      /* Spinlock for protecting concurrent operations */
  int flags;                /* Configuration flags for channel */
};

struct esp_sdm_group_priv_s
{
  int group_id;                                                         /* Group ID, index from 0 */
  spinlock_t spinlock;                                                  /* Spinlock for protecting concurrent operations */
  sdm_hal_context_t hal;                                                /* Common layer context */
  soc_periph_sdm_clk_src_t clk_src;                                     /* Clock source */
  struct esp_sdm_channel_priv_s *channels[SOC_SDM_CHANNELS_PER_GROUP];  /* Array of SDM channels */
};

struct esp_sdm_priv_s
{
  rmutex_t lock;                                       /* Lock for protecting concurrent operations */
  struct esp_sdm_group_priv_s *groups[SOC_SDM_GROUPS]; /* SDM group pool */
  int group_ref_counts[SOC_SDM_GROUPS];                /* Reference count used to protect group install/uninstall */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void dac_reset(struct dac_dev_s *dev);
static int dac_setup(struct dac_dev_s *dev);
static void dac_shutdown(struct dac_dev_s *dev);
static void dac_txint(struct dac_dev_s *dev, bool enable);
static int dac_send(struct dac_dev_s *dev, struct dac_msg_s *msg);
static int dac_ioctl(struct dac_dev_s *dev, int cmd, unsigned long arg);
static int esp_sdm_channel_set_pulse_density(
  struct esp_sdm_channel_priv_s *chan,
  int8_t density);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct dac_ops_s ops =
{
  .ao_reset = dac_reset,
  .ao_setup = dac_setup,
  .ao_shutdown = dac_shutdown,
  .ao_txint = dac_txint,
  .ao_send = dac_send,
  .ao_ioctl = dac_ioctl,
};

struct esp_sdm_priv_s g_esp_sdm =
{
  .lock = NXRMUTEX_INITIALIZER,
  .groups =
  {
    0
  },
  .group_ref_counts =
  {
    0
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dac_reset
 *
 * Description:
 *   This function resets the DAC channel.
 *
 * Input Parameters:
 *   dev - DAC structure pointer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void dac_reset(struct dac_dev_s *dev)
{
  struct esp_sdm_group_priv_s *group =
    (struct esp_sdm_group_priv_s *) dev->ad_priv;
  sdm_ll_enable_clock(group->hal.dev, false);
  sdm_ll_enable_clock(group->hal.dev, true);
}

/****************************************************************************
 * Name: dac_setup
 *
 * Description:
 *   This function configures the DAC.
 *
 * Input Parameters:
 *   dev - DAC structure pointer
 *
 * Returned Value:
 *   OK.
 *
 ****************************************************************************/

static int dac_setup(struct dac_dev_s *dev)
{
  struct esp_sdm_group_priv_s *group =
    (struct esp_sdm_group_priv_s *) dev->ad_priv;
  sdm_ll_enable_clock(group->hal.dev, true);
  return OK;
}

/****************************************************************************
 * Name: dac_shutdown
 *
 * Description:
 *   This function disables the DAC.
 *
 * Input Parameters:
 *   dev - DAC structure pointer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void dac_shutdown(struct dac_dev_s *dev)
{
  struct esp_sdm_group_priv_s *group =
    (struct esp_sdm_group_priv_s *) dev->ad_priv;
  sdm_ll_enable_clock(group->hal.dev, false);
}

/****************************************************************************
 * Name: dac_txint
 *
 * Description:
 *   This function enables/disables TX interrupts
 *
 * Input Parameters:
 *   dev    - DAC structure pointer
 *   enable - True to enable interrupts, false to disable.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void dac_txint(struct dac_dev_s *dev, bool enable)
{
  ainfo("ERROR! Interrupt not supported by SDM");
}

/****************************************************************************
 * Name: dac_send
 *
 * Description:
 *   This function sets the DAC output.
 *
 * Input Parameters:
 *   dev - DAC structure pointer
 *   msg - Parameter includes dac message details
 *
 * Returned Value:
 *   OK on success; ERROR on failure.
 *
 ****************************************************************************/

static int dac_send(struct dac_dev_s *dev, struct dac_msg_s *msg)
{
  struct esp_sdm_group_priv_s *group =
    (struct esp_sdm_group_priv_s *) dev->ad_priv;
  int channel = msg->am_channel;
  irqstate_t flags = spin_lock_irqsave(&group->spinlock);
  int ret = ERROR;

  if (group->channels[channel] != NULL)
    {
      ret = esp_sdm_channel_set_pulse_density(group->channels[channel],
                                              (int8_t)msg->am_data);
    }
  else
    {
      aerr("ERROR! Selected channel(%d) on group(%d) not initialized\n",
            msg->am_channel, group->group_id);
    }

  spin_unlock_irqrestore(&group->spinlock, flags);

  dac_txdone(dev);
  return ret;
}

/****************************************************************************
 * Name: dac_ioctl
 *
 * Description:
 *   Lower-half logic may support platform-specific ioctl commands
 *
 * Description:
 *   Lower-half logic may support platform-specific ioctl commands
 *
 * Input Parameters:
 *   dev - DAC structure pointer
 *   cmd - The ioctl command
 *   arg - The argument accompanying the ioctl command
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int dac_ioctl(struct dac_dev_s *dev, int cmd, unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Name: esp_sdm_channel_set_pulse_density
 *
 * Description:
 *   Set the pulse density of the PDM output signal. Output can be calculated
 *   with "Vout = (VDD_IO / 256 * density) + VDD_IO / 2" formula
 *   (e.g 0 density will output %50 duty).
 *
 * Input Parameters:
 *   chan    - SDM channel struct pointer
 *   density - Quantized pulse density of the PDM output signal ranges from
 *             -128 to 127. But the range of [-90, 90] can provide a better
 *             randomness.
 *
 * Returned Value:
 *   OK
 *
 ****************************************************************************/

static int esp_sdm_channel_set_pulse_density(
  struct esp_sdm_channel_priv_s *chan,
  int8_t density)
{
  DEBUGASSERT(chan != NULL);
  struct esp_sdm_group_priv_s *group = g_esp_sdm.groups[chan->group_id];
  int chan_id = chan->chan_id;
  irqstate_t flags;

  flags = spin_lock_irqsave(&chan->spinlock);
  sdm_ll_set_pulse_density(group->hal.dev, chan_id, density);
  spin_unlock_irqrestore(&chan->spinlock, flags);

  return ESP_OK;
}

/****************************************************************************
 * Name: esp_sdm_create_config_channel
 *
 * Description:
 *   This function tries to allocate a channel and configures it with
 *   given parameters
 *
 * Input Parameters:
 *   config   - A structure containing the configuration settings
 *   group_id - Group number to allocate channel
 *   chan_id  - Channel number to allocate
 *
 * Returned Value:
 *   On success, this function returns a valid pointer to the SDM channel
 *   structure. If the initialization fails, it returns NULL.
 *
 ****************************************************************************/

struct esp_sdm_channel_priv_s *esp_sdm_create_config_channel(
  struct esp_sdm_chan_config_s config, int group_id, int chan_id)
{
  int attr;
  uint32_t prescale;
  uint32_t src_clk_hz;
  struct esp_sdm_channel_priv_s *ret =
    kmm_zalloc(sizeof(struct esp_sdm_channel_priv_s));
  if (ret == NULL)
    {
      aerr("Error! No mem for group (%d) - channel (%d)\n",
            group_id, chan_id);
      return ret;
    }

  ret->group_id = group_id;
  ret->chan_id = chan_id;
  ret->flags = config.flags;
  ret->gpio_num = config.gpio_num;
  ret->sample_rate_hz = config.sample_rate_hz;

  attr = OUTPUT | PULLUP;
  if ((config.flags & IO_LOOPBACK) != 0)
    {
      attr |= INPUT;
    }

  esp_configgpio(config.gpio_num, attr);
  esp_gpio_matrix_out(config.gpio_num,
      sigma_delta_periph_signals.channels[ret->chan_id].sd_sig,
      (config.flags && INVERT_OUT), false);

  esp_clk_tree_src_get_freq_hz(g_esp_sdm.groups[ret->group_id]->clk_src,
    ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED, &src_clk_hz);
  prescale = src_clk_hz / config.sample_rate_hz;
  sdm_ll_set_prescale(g_esp_sdm.groups[ret->group_id]->hal.dev,
                      ret->chan_id,
                      prescale);
  ret->sample_rate_hz = prescale;
  sdm_ll_set_pulse_density(g_esp_sdm.groups[ret->group_id]->hal.dev,
                           ret->chan_id,
                           0);

  ainfo("New SDM channel (%d,%d) at %p, gpio=%d,sample rate=%"PRIu32"Hz\n",
        ret->group_id, ret->chan_id, ret,
        config.gpio_num, ret->sample_rate_hz);
  return ret;
}

/****************************************************************************
 * Name: esp_sdm_init
 *
 * Description:
 *   This function tries to find an unallocated channel.
 *   After that configures it with given parameters
 *
 * Input Parameters:
 *   config - A structure containing the configuration settings
 *
 * Returned Value:
 *   On success, this function returns a valid pointer to the SDM group
 *   device structure. If the initialization fails, it returns NULL.
 *
 ****************************************************************************/

static struct esp_sdm_group_priv_s *esp_sdm_init(
  struct esp_sdm_chan_config_s config)
{
  struct esp_sdm_group_priv_s *group = NULL;
  struct esp_sdm_channel_priv_s *ret = NULL;
  int i = 0;
  int j = 0;
  int group_id = 0;
  int chan_id = -1;
  irqstate_t flags;

  DEBUGASSERT(GPIO_IS_VALID_GPIO(config.gpio_num));

  for (i = 0; i < SOC_SDM_GROUPS; i++)
    {
      nxrmutex_lock(&(g_esp_sdm.lock));
      if (g_esp_sdm.groups[i] == NULL)
        {
          g_esp_sdm.groups[i] =
            kmm_zalloc(sizeof(struct esp_sdm_group_priv_s));
          if (g_esp_sdm.groups[i] == NULL)
            {
              aerr("Error! No mem for group (%d)\n", i);
              return NULL;
            }
          else
            {
              g_esp_sdm.groups[i]->group_id = i;
              g_esp_sdm.groups[i]->clk_src = SDM_CLK_SRC_DEFAULT;
              group_id = i;
              sdm_hal_init(&g_esp_sdm.groups[i]->hal, i);
              sdm_ll_enable_clock(g_esp_sdm.groups[i]->hal.dev, true);
              ainfo("new group (%d) at %p\n", i, g_esp_sdm.groups[i]);
              break;
            }
        }
      else if (g_esp_sdm.group_ref_counts[i] < SOC_SDM_CHANNELS_PER_GROUP)
        {
          group_id = i;
          break;
        }
    }

  if (g_esp_sdm.group_ref_counts[group_id] >=
        SOC_SDM_CHANNELS_PER_GROUP)
    {
      aerr("ERROR! No free slot available\n");
      return NULL;
    }

  group = g_esp_sdm.groups[group_id];
  g_esp_sdm.group_ref_counts[group_id]++;
  nxrmutex_unlock(&(g_esp_sdm.lock));

  flags = spin_lock_irqsave(&group->spinlock);
  for (j = 0; j < SOC_SDM_CHANNELS_PER_GROUP; j++)
    {
      if (group->channels[j] == NULL)
        {
          ret = esp_sdm_create_config_channel(config, i, j);
          if (ret == NULL)
            {
              aerr("Error! No mem for group (%d) - channel (%d)\n",
                    i, j);
              return NULL;
            }

          group->channels[j] = ret;
          break;
        }
    }

  spin_unlock_irqrestore(&group->spinlock, flags);
  if (j < 0)
    {
      nxrmutex_lock(&(g_esp_sdm.lock));
      g_esp_sdm.group_ref_counts[i]--;
      if (g_esp_sdm.group_ref_counts[i] == 0)
        {
          sdm_ll_enable_clock(group->hal.dev, false);
          free(g_esp_sdm.groups[i]);
          g_esp_sdm.groups[i] = NULL;
        }

      group = NULL;
      nxrmutex_unlock(&(g_esp_sdm.lock));
    }

  return group;
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_sdm_create_channel
 *
 * Description:
 *   This function initializes a SDM (Sigma Delta Modulator) channel and
 *   register to related group with the provided configuration.
 *   Each group can be used as independent DAC and each channel in a group
 *   can be used as DAC channel.
 *
 * Input Parameters:
 *   config  - A structure containing the configuration settings
 *   dac_dev - Pointer to DAC structure which
 *             is returned from esp_sdminitialize
 *
 * Returned Value:
 *   OK on success, ERROR on failure
 *
 ****************************************************************************/

int esp_sdm_create_channel(struct esp_sdm_chan_config_s config,
                           struct dac_dev_s *dac_dev)
{
  struct esp_sdm_group_priv_s *group = dac_dev->ad_priv;
  int chan_num = -1;
  int i;

  DEBUGASSERT(GPIO_IS_VALID_GPIO(config.gpio_num));

  for (i = 0; i < SOC_SDM_CHANNELS_PER_GROUP; i++)
    {
      if (group->channels[i] == NULL)
        {
          chan_num = i;
          break;
        }
    }

  if (chan_num < 0)
    {
      aerr("ERROR! No free slots on group(%d)\n", group->group_id);
      return ERROR;
    }

  group->channels[chan_num] = esp_sdm_create_config_channel(config,
                                                            group->group_id,
                                                            chan_num);
  if (group->channels[chan_num] == NULL)
    {
      aerr("ERROR! Failed to create channel(%d) on group(%d)\n", chan_num,
            group->group_id);
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: esp_sdminitialize
 *
 * Description:
 *   This function initializes SDM (Sigma Delta Modulator) group and a
 *   channel with the provided configuration.
 *
 * Input Parameters:
 *   config - A structure containing the configuration settings
 *
 * Returned Value:
 *   On success, this function returns a valid pointer to the SDM device
 *   structure. If the initialization fails, it returns NULL.
 *
 ****************************************************************************/

struct dac_dev_s *esp_sdminitialize(struct esp_sdm_chan_config_s config)
{
  struct dac_dev_s *ret = kmm_zalloc(sizeof(struct dac_dev_s));
  struct esp_sdm_group_priv_s *sdm = esp_sdm_init(config);
  if (ret == NULL)
    {
      aerr("Error! Not enough memory");
      if (sdm != NULL)
        {
          free(sdm);
          return NULL;
        }
    }
  else
    {
      ret->ad_priv = (void *)sdm;
      ret->ad_ops = &ops;
      ret->ad_nchannel = SOC_SDM_CHANNELS_PER_GROUP;
    }

  return (struct dac_dev_s *)ret;
}
