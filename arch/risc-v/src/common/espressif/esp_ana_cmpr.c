/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_ana_cmpr.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this args for additional information regarding copyright ownership.  The
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

#include <nuttx/analog/comp.h>
#include <nuttx/debug.h>

#include "espressif/esp_ana_cmpr.h"

#include "soc/clk_tree_defs.h"
#include "driver/ana_cmpr.h"
#include "hal/ana_cmpr_ll.h"
#include "hal/ana_cmpr_periph.h"
#include "hal/ana_cmpr_types.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp_ana_cmpr_priv_s
{
  ana_cmpr_handle_t handle;                   /* Peripheral handler */
  ana_cmpr_config_t *config;                  /* Peripheral configuration */
  ana_cmpr_internal_ref_config_t *ref_cfg;    /* Comparator configuration */
  ana_cmpr_debounce_config_t debounce_config; /* Debounce filter configuration */
  const struct comp_callback_s *cb;           /* Upper driver callback */
  bool initialized;                           /* Comparator unit initialized */
  bool cross_pos;                             /* Indication of cross type */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int esp_ana_cmpr_bind(FAR struct comp_dev_s *dev,
                             FAR const struct comp_callback_s *callback);
static int esp_ana_cmpr_setup(FAR struct comp_dev_s *dev);
static void esp_ana_cmpr_shutdown(FAR struct comp_dev_s *dev);
static int esp_ana_cmpr_read(FAR struct comp_dev_s *dev);
static int esp_ana_cmpr_ioctl(FAR struct comp_dev_s *dev,
                              int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct comp_ops_s g_esp_ana_ops =
{
  .ao_bind = esp_ana_cmpr_bind,
  .ao_setup = esp_ana_cmpr_setup,
  .ao_shutdown = esp_ana_cmpr_shutdown,
  .ao_read = esp_ana_cmpr_read,
  .ao_ioctl = esp_ana_cmpr_ioctl,
};

#ifdef CONFIG_ESPRESSIF_ANA_COMPR0

ana_cmpr_internal_ref_config_t ana_cmpr0_ref_cfg =
{
#if defined(CONFIG_ESPRESSIF_ANA_COMPR0_REF_VOLT_0_VDD)
  .ref_volt = ANA_CMPR_REF_VOLT_0_PCT_VDD
#elif defined(CONFIG_ESPRESSIF_ANA_COMPR0_REF_VOLT_10_VDD)
  .ref_volt = ANA_CMPR_REF_VOLT_10_PCT_VDD
#elif defined(CONFIG_ESPRESSIF_ANA_COMPR0_REF_VOLT_20_VDD)
  .ref_volt = ANA_CMPR_REF_VOLT_20_PCT_VDD
#elif defined(CONFIG_ESPRESSIF_ANA_COMPR0_REF_VOLT_30_VDD)
  .ref_volt = ANA_CMPR_REF_VOLT_30_PCT_VDD
#elif defined(CONFIG_ESPRESSIF_ANA_COMPR0_REF_VOLT_40_VDD)
  .ref_volt = ANA_CMPR_REF_VOLT_40_PCT_VDD
#elif defined(CONFIG_ESPRESSIF_ANA_COMPR0_REF_VOLT_50_VDD)
  .ref_volt = ANA_CMPR_REF_VOLT_50_PCT_VDD
#elif defined(CONFIG_ESPRESSIF_ANA_COMPR0_REF_VOLT_60_VDD)
  .ref_volt = ANA_CMPR_REF_VOLT_60_PCT_VDD
#elif defined(CONFIG_ESPRESSIF_ANA_COMPR0_REF_VOLT_70_VDD)
  .ref_volt = ANA_CMPR_REF_VOLT_70_PCT_VDD
#endif
};

ana_cmpr_config_t g_esp_ana_cmpr0_config =
{
  .unit = 0,
  .clk_src = ANA_CMPR_CLK_SRC_DEFAULT,
#ifdef CONFIG_ESPRESSIF_ANA_COMPR0_REF_SRC_INTERNAL
  .ref_src = ANA_CMPR_REF_SRC_INTERNAL,
#else
  .ref_src = ANA_CMPR_REF_SRC_EXTERNAL,
#endif
  .cross_type = ANA_CMPR_CROSS_ANY,
};

struct esp_ana_cmpr_priv_s g_esp_ana_cmpr0_priv =
{
  .handle = NULL,
  .config = &g_esp_ana_cmpr0_config,
  .ref_cfg = &ana_cmpr0_ref_cfg,
  .debounce_config =
  {
    .wait_us = CONFIG_ESPRESSIF_ANA_COMPR0_DEBOUNCE * 1000,
  },
  .cb = NULL,
  .initialized = false,
  .cross_pos = false,
};

static struct comp_dev_s g_esp_ana_cmpr0 =
{
  .ad_ops = &g_esp_ana_ops,
  .ad_priv = &g_esp_ana_cmpr0_priv,
};
#endif

#ifdef CONFIG_ESPRESSIF_ANA_COMPR1
ana_cmpr_internal_ref_config_t ana_cmpr1_ref_cfg =
{
#if defined(CONFIG_ESPRESSIF_ANA_COMPR1_REF_VOLT_0_VDD)
  .ref_volt = ANA_CMPR_REF_VOLT_0_PCT_VDD
#elif defined(CONFIG_ESPRESSIF_ANA_COMPR1_REF_VOLT_10_VDD)
  .ref_volt = ANA_CMPR_REF_VOLT_10_PCT_VDD
#elif defined(CONFIG_ESPRESSIF_ANA_COMPR1_REF_VOLT_20_VDD)
  .ref_volt = ANA_CMPR_REF_VOLT_20_PCT_VDD
#elif defined(CONFIG_ESPRESSIF_ANA_COMPR1_REF_VOLT_30_VDD)
  .ref_volt = ANA_CMPR_REF_VOLT_30_PCT_VDD
#elif defined(CONFIG_ESPRESSIF_ANA_COMPR1_REF_VOLT_40_VDD)
  .ref_volt = ANA_CMPR_REF_VOLT_40_PCT_VDD
#elif defined(CONFIG_ESPRESSIF_ANA_COMPR1_REF_VOLT_50_VDD)
  .ref_volt = ANA_CMPR_REF_VOLT_50_PCT_VDD
#elif defined(CONFIG_ESPRESSIF_ANA_COMPR1_REF_VOLT_60_VDD)
  .ref_volt = ANA_CMPR_REF_VOLT_60_PCT_VDD
#elif defined(CONFIG_ESPRESSIF_ANA_COMPR1_REF_VOLT_70_VDD)
  .ref_volt = ANA_CMPR_REF_VOLT_70_PCT_VDD
#endif
};

ana_cmpr_config_t g_esp_ana_cmpr1_config =
{
  .unit = 1,
  .clk_src = ANA_CMPR_CLK_SRC_DEFAULT,
#ifdef CONFIG_ESPRESSIF_ANA_COMPR1_REF_SRC_INTERNAL
  .ref_src = ANA_CMPR_REF_SRC_INTERNAL,
#else
  .ref_src = ANA_CMPR_REF_SRC_EXTERNAL,
#endif
  .cross_type = ANA_CMPR_CROSS_ANY,
};

struct esp_ana_cmpr_priv_s g_esp_ana_cmpr1_priv =
{
  .handle = NULL,
  .config = &g_esp_ana_cmpr1_config,
  .ref_cfg = &ana_cmpr1_ref_cfg,
  .debounce_config =
  {
    .wait_us = CONFIG_ESPRESSIF_ANA_COMPR1_DEBOUNCE * 1000,
  },
  .cb = NULL,
  .initialized = false,
  .cross_pos = false,
};

static struct comp_dev_s g_esp_ana_cmpr1 =
{
  .ad_ops = &g_esp_ana_ops,
  .ad_priv = &g_esp_ana_cmpr1_priv
};
#endif

/****************************************************************************
 * Private functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_ana_cmpr_bind
 *
 * Description:
 *   Bind the upper-half driver callbacks to the lower-half implementation.
 *
 * Input Parameters:
 *   dev      - Comparator upper layer struct.
 *   callback - Pointer to the upper-half driver callback structure.
 *
 * Returned Value:
 *   OK.
 *
 ****************************************************************************/

static int esp_ana_cmpr_bind(FAR struct comp_dev_s *dev,
                             FAR const struct comp_callback_s *callback)
{
  struct esp_ana_cmpr_priv_s *priv = dev->ad_priv;
  DEBUGASSERT(priv);

  priv->cb = callback;
  return OK;
}

/****************************************************************************
 * Name: esp_ana_cmpr_setup
 *
 * Description:
 *   Configure the COMP.
 *
 * Input Parameters:
 *   dev - Comparator upper layer struct.
 *
 * Returned Value:
 *   OK.
 *
 ****************************************************************************/

static int esp_ana_cmpr_setup(FAR struct comp_dev_s *dev)
{
  struct esp_ana_cmpr_priv_s *priv = dev->ad_priv;
  DEBUGASSERT(priv);

  if (priv->initialized != true)
    {
      ana_cmpr_enable(priv->handle);
      priv->initialized = true;
    }

  return OK;
}

/****************************************************************************
 * Name: esp_ana_cmpr_shutdown
 *
 * Description:
 *   Disable the COMP.
 *
 * Input Parameters:
 *   dev - Comparator upper layer struct.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void esp_ana_cmpr_shutdown(FAR struct comp_dev_s *dev)
{
  struct esp_ana_cmpr_priv_s *priv = dev->ad_priv;
  DEBUGASSERT(priv);

  if (priv->initialized == true)
    {
      ana_cmpr_disable(priv->handle);
      priv->initialized = false;
    }
}

/****************************************************************************
 * Name: esp_ana_cmpr_read
 *
 * Description:
 *   Read COMP output state.
 *
 * Input Parameters:
 *   dev - Comparator upper layer struct.
 *
 * Returned Value:
 *   True if channel is higher than reference, false otherwise.
 *
 ****************************************************************************/

static int esp_ana_cmpr_read(FAR struct comp_dev_s *dev)
{
  struct esp_ana_cmpr_priv_s *priv = dev->ad_priv;
  DEBUGASSERT(priv);

  return priv->cross_pos;
}

/****************************************************************************
 * Name: esp_ana_cmpr_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 * Input Parameters:
 *   dev  - Pointer to the comparator device structure.
 *   cmd  - The ioctl command.
 *   arg  - The argument for the ioctl command.
 *
 * Returned Value:
 *   OK.
 *
 ****************************************************************************/

static int esp_ana_cmpr_ioctl(FAR struct comp_dev_s *dev,
                              int cmd, unsigned long arg)
{
  struct esp_ana_cmpr_priv_s *priv = dev->ad_priv;
  DEBUGASSERT(priv);

  ainfo("Analog comparator ioctl calls did not implement\n");
  return OK;
}

/****************************************************************************
 * Name: esp_ana_cmpr_callback
 *
 * Description:
 *   Callaback function for comparator.
 *
 * Input Parameters:
 *   cmpr     - Handle of comparator unit.
 *   edata    - Interrupt event information
 *   user_ctx - User callback data
 *
 * Returned Value:
 *   False.
 *
 ****************************************************************************/

static bool esp_ana_cmpr_callback(ana_cmpr_handle_t cmpr,
                                  const ana_cmpr_cross_event_data_t *edata,
                                  void *user_ctx)
{
  struct comp_dev_s *dev = (struct comp_dev_s *)user_ctx;
  struct esp_ana_cmpr_priv_s *priv = dev->ad_priv;
  DEBUGASSERT(priv);

#ifdef SOC_ANA_CMPR_CAN_DISTINGUISH_EDGE
  if (edata->cross_type == ANA_CMPR_CROSS_POS)
    {
      ainfo("Comparator unit%d: Positive cross event triggered\n",
             priv->config->unit);
      priv->cross_pos = true;
    }
  else
    {
      ainfo("Comparator unit%d: Negative cross event triggered\n",
            priv->config->unit);
      priv->cross_pos = false;
    }
#else
  ainfo("Comparator unit%d: Any cross event triggered\n",
        priv->config->unit);
  priv->cross_pos = true;
#endif

  if (priv->cb != NULL)
    {
      priv->cb->au_notify(dev, priv->cross_pos);
    }

  return false;
}

/****************************************************************************
 * Name: esp_ana_cmpr_init
 *
 * Description:
 *   Initialize analog comparator hardware and then register.
 *
 * Input Parameters:
 *   dev - Comparator upper layer struct.
 *
 * Returned Value:
 *   Zero (OK) is returned on success. Otherwise -1 (ERROR).
 *
 ****************************************************************************/

static int esp_ana_cmpr_init(struct comp_dev_s *dev)
{
  int ret;
  char path[32];
  struct esp_ana_cmpr_priv_s *priv = dev->ad_priv;
  ana_cmpr_event_callbacks_t cbs =
    {
      .on_cross = esp_ana_cmpr_callback,
    };

  ret = ana_cmpr_new_unit(priv->config, &priv->handle);
  if (ret != OK)
    {
      return ret;
    }

  ana_cmpr_set_internal_reference(priv->handle, priv->ref_cfg);

  snprintf(path, sizeof(path), "/dev/anacmpr%d", priv->config->unit);
  ret = comp_register(path, dev);
  if (ret != OK)
    {
      aerr("Comparator register failed for %s\n", path);
      return ret;
    }

  ana_cmpr_set_debounce(priv->handle, &priv->debounce_config);
  ana_cmpr_register_event_callbacks(priv->handle, &cbs, dev);
  ana_cmpr_enable(priv->handle);
  priv->initialized = true;

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_cmprinitialize
 *
 * Description:
 *   Initialize and register the analog comparator driver.
 *
 * Input Parameters:
 *   unit - Unit number of the comparator interface to be initialized.
 *
 * Returned Value:
 *   Zero (OK) is returned on success. Otherwise -1 (ERROR).
 *
 ****************************************************************************/

int esp_cmprinitialize(int unit)
{
  struct comp_dev_s *dev;
  int ret;

  switch (unit)
    {
#ifdef CONFIG_ESPRESSIF_ANA_COMPR0
    case ESPRESSIF_COMP0:
      dev = &g_esp_ana_cmpr0;
      break;
#endif
#ifdef CONFIG_ESPRESSIF_ANA_COMPR1
    case ESPRESSIF_COMP1:
      dev = &g_esp_ana_cmpr1;
      break;
#endif
    default:
      aerr("Invalid unit number\n");
      return ERROR;
    }

  ret = esp_ana_cmpr_init(dev);
  if (ret != OK)
    {
      aerr("Analog comparator unit%d initialization failed!\n", unit);
      return ret;
    }

  ainfo("Analog comparator unit%d initialized! Handler: %p\n",
        unit, dev);
  return ret;
}
