/****************************************************************************
 * arch/arm/src/rzv2h/rzv2h_adc.c
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

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>
#include <nuttx/arch.h>
#include <nuttx/debug.h>
#include <nuttx/irq.h>

#include <arch/chip/adc.h>

#include "r_adc_e.h"
#include "rzv2h_adc.h"
#include "rzv2h_fsp_err.h"
#include "rzv2h_irq.h"

#ifdef CONFIG_RZV2H_ADC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_RZV2H_ADC_WINDOW_A
#  if CONFIG_RZV2H_ADC_WINDOW_A_LOWER > \
      CONFIG_RZV2H_ADC_WINDOW_A_UPPER
#    error "Window A lower threshold exceeds upper threshold"
#  endif

#  define RZV2H_ADC_WINDOW_A_FSP_IRQ ADC0_ADA_COMPAI_N_IRQn
#  define RZV2H_ADC_WINDOW_A_IRQ \
  RZV2H_FSP_TO_GIC_IRQ(RZV2H_ADC_WINDOW_A_FSP_IRQ)
#endif

#if defined(CONFIG_RZV2H_ADC_ADD_TWO)
#  define RZV2H_ADC_ADD_AVERAGE_COUNT ADC_E_ADD_TWO
#elif defined(CONFIG_RZV2H_ADC_ADD_THREE)
#  define RZV2H_ADC_ADD_AVERAGE_COUNT ADC_E_ADD_THREE
#elif defined(CONFIG_RZV2H_ADC_ADD_FOUR)
#  define RZV2H_ADC_ADD_AVERAGE_COUNT ADC_E_ADD_FOUR
#elif defined(CONFIG_RZV2H_ADC_ADD_SIXTEEN)
#  define RZV2H_ADC_ADD_AVERAGE_COUNT ADC_E_ADD_SIXTEEN
#elif defined(CONFIG_RZV2H_ADC_AVERAGE_TWO)
#  define RZV2H_ADC_ADD_AVERAGE_COUNT ADC_E_ADD_AVERAGE_TWO
#elif defined(CONFIG_RZV2H_ADC_AVERAGE_FOUR)
#  define RZV2H_ADC_ADD_AVERAGE_COUNT ADC_E_ADD_AVERAGE_FOUR
#elif defined(CONFIG_RZV2H_ADC_ADD_AVERAGE_OFF)
#  define RZV2H_ADC_ADD_AVERAGE_COUNT ADC_E_ADD_OFF
#else
#  error "No RZ/V2H ADC add/average count selected"
#endif

#ifndef CONFIG_RZV2H_ADC_SCAN_END_INTERRUPT
#define RZV2H_ADC_TIMEOUT_US     100000
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rzv2h_adc_priv_s
{
  const struct adc_callback_s *cb;
  adc_e_instance_ctrl_t ctrl;
  uint32_t channel_mask;
  uint8_t nchannels;
  bool opened;
  bool rxenabled;
#ifdef CONFIG_RZV2H_ADC_TRIGGER_EXTERNAL
  bool scan_armed;
#endif
#ifdef CONFIG_RZV2H_ADC_SCAN_END_INTERRUPT
  bool irqattached;
#endif
#ifdef CONFIG_RZV2H_ADC_WINDOW_A
  bool windowairqattached;
  struct rzv2h_adc_window_status_s windowastatus;
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_RZV2H_ADC_SCAN_END_INTERRUPT
void adc_e_scan_end_isr(void);
#endif
#ifdef CONFIG_RZV2H_ADC_WINDOW_A
void adc_e_window_compare_isr(void);
#endif

static int rzv2h_adc_bind(struct adc_dev_s *dev,
                          const struct adc_callback_s *callback);
static void rzv2h_adc_reset(struct adc_dev_s *dev);
static int rzv2h_adc_setup(struct adc_dev_s *dev);
static void rzv2h_adc_shutdown(struct adc_dev_s *dev);
static void rzv2h_adc_rxint(struct adc_dev_s *dev, bool enable);
static int rzv2h_adc_ioctl(struct adc_dev_s *dev, int cmd,
                           unsigned long arg);
static int rzv2h_adc_receive(struct adc_dev_s *dev);

#ifdef CONFIG_RZV2H_ADC_SCAN_END_INTERRUPT
static int rzv2h_adc_interrupt(int irq, void *context, void *arg);
static int rzv2h_adc_irq_attach(struct adc_dev_s *dev);
static void rzv2h_adc_irq_detach(struct rzv2h_adc_priv_s *priv);
static void rzv2h_adc_fsp_rxint(struct rzv2h_adc_priv_s *priv,
                                bool enable);
#endif
#if defined(CONFIG_RZV2H_ADC_SCAN_END_INTERRUPT) || \
    defined(CONFIG_RZV2H_ADC_WINDOW_A)
static void rzv2h_adc_fsp_callback(adc_callback_args_t *args);
#endif
#ifdef CONFIG_RZV2H_ADC_WINDOW_A
static int rzv2h_adc_window_a_interrupt(int irq, void *context, void *arg);
static void rzv2h_adc_window_a_irq_detach(
  struct rzv2h_adc_priv_s *priv);
static void rzv2h_adc_window_a_reset(struct rzv2h_adc_priv_s *priv);
static int rzv2h_adc_window_a_status(struct rzv2h_adc_priv_s *priv,
                                     unsigned long arg);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct adc_ops_s g_rzv2h_adc_ops =
{
  .ao_bind     = rzv2h_adc_bind,
  .ao_reset    = rzv2h_adc_reset,
  .ao_setup    = rzv2h_adc_setup,
  .ao_shutdown = rzv2h_adc_shutdown,
  .ao_rxint    = rzv2h_adc_rxint,
  .ao_ioctl    = rzv2h_adc_ioctl,
};

static struct rzv2h_adc_priv_s g_rzv2h_adc_priv;
static struct adc_dev_s g_rzv2h_adc_dev =
{
  .ad_ops  = &g_rzv2h_adc_ops,
  .ad_priv = &g_rzv2h_adc_priv,
};

static const adc_e_extended_cfg_t g_rzv2h_adc_extend =
{
  .add_average_count           = RZV2H_ADC_ADD_AVERAGE_COUNT,
#ifdef CONFIG_RZV2H_ADC_CLEAR_AFTER_READ
  .clearing                    = ADC_E_CLEAR_AFTER_READ_ON,
#else
  .clearing                    = ADC_E_CLEAR_AFTER_READ_OFF,
#endif
  .trigger_group_b             = ADC_TRIGGER_SOFTWARE,
  .double_trigger_mode         = ADC_E_DOUBLE_TRIGGER_DISABLED,
#ifdef CONFIG_RZV2H_ADC_TRIGGER_EXTERNAL
  .adc_start_trigger_a         = ADC_E_ACTIVE_TRIGGER_EXTERNAL,
#else
  .adc_start_trigger_a         = ADC_E_ACTIVE_TRIGGER_DISABLED,
#endif
  .adc_start_trigger_b         = ADC_E_ACTIVE_TRIGGER_DISABLED,
  .adc_start_trigger_c_enabled = false,
  .adc_start_trigger_c         = ADC_E_ACTIVE_TRIGGER_DISABLED,
  .adc_elc_ctrl                = ADC_E_ELC_GROUP_A_SCAN,
#ifdef CONFIG_RZV2H_ADC_WINDOW_A
  .window_a_irq                = RZV2H_ADC_WINDOW_A_FSP_IRQ,
  .window_a_ipl                = CONFIG_RZV2H_ADC_WINDOW_A_IRQ_PRIORITY,
#else
  .window_a_irq                = FSP_INVALID_VECTOR,
  .window_a_ipl                = 0,
#endif
  .window_b_irq                = FSP_INVALID_VECTOR,
  .window_b_ipl                = 0,
};

static const adc_cfg_t g_rzv2h_adc_cfg =
{
  .unit           = 0,
  .mode           = ADC_MODE_SINGLE_SCAN,
#if defined(CONFIG_RZV2H_ADC_RESOLUTION_8BIT)
  .resolution     = ADC_RESOLUTION_8_BIT,
#elif defined(CONFIG_RZV2H_ADC_RESOLUTION_12BIT)
  .resolution     = ADC_RESOLUTION_12_BIT,
#else
#  error "No RZ/V2H ADC resolution selected"
#endif
  .alignment      = ADC_ALIGNMENT_RIGHT,
#ifdef CONFIG_RZV2H_ADC_TRIGGER_EXTERNAL
  .trigger        = ADC_TRIGGER_ASYNC_EXTERNAL,
#else
  .trigger        = ADC_TRIGGER_SOFTWARE,
#endif
#ifdef CONFIG_RZV2H_ADC_SCAN_END_INTERRUPT
  .scan_end_irq   = (IRQn_Type)CONFIG_RZV2H_ADC_SCAN_END_INTSEL,
  .scan_end_ipl   = CONFIG_RZV2H_ADC_SCAN_END_IRQ_PRIORITY,
#else
  .scan_end_irq   = FSP_INVALID_VECTOR,
  .scan_end_ipl   = BSP_IRQ_DISABLED,
#endif
  .scan_end_b_irq = FSP_INVALID_VECTOR,
  .scan_end_c_irq = FSP_INVALID_VECTOR,
  .scan_end_b_ipl = BSP_IRQ_DISABLED,
  .scan_end_c_ipl = BSP_IRQ_DISABLED,
#if defined(CONFIG_RZV2H_ADC_SCAN_END_INTERRUPT) || \
    defined(CONFIG_RZV2H_ADC_WINDOW_A)
  .p_callback     = rzv2h_adc_fsp_callback,
  .p_context      = &g_rzv2h_adc_dev,
#else
  .p_callback     = NULL,
  .p_context      = NULL,
#endif
  .p_extend       = &g_rzv2h_adc_extend,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_RZV2H_ADC_SCAN_END_INTERRUPT
/****************************************************************************
 * Name: rzv2h_adc_irq_attach
 *
 * Description:
 *   Configure and attach the ADC group-A scan-end interrupt. The scan-end
 *   IRQ in the FSP configuration identifies an INTSEL destination. Keep
 *   its raw GIC interrupt disabled while routing the ADC event and attaching
 *   the NuttX wrapper. R_ADC_E_Open() configures and enables the interrupt
 *   after this function returns.
 *
 * Input Parameters:
 *   dev - ADC lower-half device.
 *
 * Returned Value:
 *   OK on success; a negative errno value on failure.
 *
 ****************************************************************************/

static int rzv2h_adc_irq_attach(struct adc_dev_s *dev)
{
  struct rzv2h_adc_priv_s *priv = dev->ad_priv;
  rzv2h_irqn_t fsp_irq = (rzv2h_irqn_t)g_rzv2h_adc_cfg.scan_end_irq;
  int gic_irq = RZV2H_FSP_TO_GIC_IRQ(fsp_irq);
  int ret;

  up_disable_irq(gic_irq);

  ret = rzv2h_intsel_connect_event(fsp_irq,
                                   RZV2H_IRQSEL_ADC0_ADIREQ,
                                   BSP_GIC_SPI_DETECT_EDGE);
  if (ret < 0)
    {
      aerr("ERROR: Failed to route ADC scan-end event: %d\n", ret);
      return ret;
    }

  ret = irq_attach(gic_irq, rzv2h_adc_interrupt, dev);
  if (ret < 0)
    {
      aerr("ERROR: Failed to attach ADC scan-end IRQ %d: %d\n",
           gic_irq, ret);
      rzv2h_intsel_disconnect_event(fsp_irq);
      return ret;
    }

  priv->irqattached = true;
  return OK;
}

/****************************************************************************
 * Name: rzv2h_adc_irq_detach
 *
 * Description:
 *   Disable and detach the ADC group-A scan-end interrupt, then release its
 *   INTSEL destination.
 *
 * Input Parameters:
 *   priv - RZ/V2H ADC private state.
 *
 ****************************************************************************/

static void rzv2h_adc_irq_detach(struct rzv2h_adc_priv_s *priv)
{
  rzv2h_irqn_t fsp_irq = (rzv2h_irqn_t)g_rzv2h_adc_cfg.scan_end_irq;
  int gic_irq = RZV2H_FSP_TO_GIC_IRQ(fsp_irq);

  if (!priv->irqattached)
    {
      return;
    }

  up_disable_irq(gic_irq);
  irq_detach(gic_irq);
  rzv2h_intsel_disconnect_event(fsp_irq);
  priv->irqattached = false;
}
#endif

#ifdef CONFIG_RZV2H_ADC_WINDOW_A
/****************************************************************************
 * Name: rzv2h_adc_window_a_irq_detach
 *
 * Description:
 *   Disable and detach the ADC Window A interrupt.
 *
 * Input Parameters:
 *   priv - RZ/V2H ADC private state.
 *
 ****************************************************************************/

static void rzv2h_adc_window_a_irq_detach(
  struct rzv2h_adc_priv_s *priv)
{
  if (!priv->windowairqattached)
    {
      return;
    }

  up_disable_irq(RZV2H_ADC_WINDOW_A_IRQ);
  irq_detach(RZV2H_ADC_WINDOW_A_IRQ);
  priv->windowairqattached = false;
}

/****************************************************************************
 * Name: rzv2h_adc_window_a_reset
 *
 * Description:
 *   Atomically clear all accumulated Window A event state.
 *
 * Input Parameters:
 *   priv - RZ/V2H ADC private state.
 *
 ****************************************************************************/

static void rzv2h_adc_window_a_reset(struct rzv2h_adc_priv_s *priv)
{
  struct rzv2h_adc_window_status_s empty =
    {
      0
    };

  irqstate_t flags;

  flags = enter_critical_section();
  priv->windowastatus = empty;
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: rzv2h_adc_window_a_status
 *
 * Description:
 *   Copy the accumulated Window A status to the caller and atomically clear
 *   the driver copy. Events arriving after the critical section are kept
 *   for the next ioctl call.
 *
 * Input Parameters:
 *   priv - RZ/V2H ADC private state.
 *   arg  - Pointer to a struct rzv2h_adc_window_status_s supplied by the
 *          application.
 *
 * Returned Value:
 *   OK on success; -EINVAL if arg is NULL.
 *
 ****************************************************************************/

static int rzv2h_adc_window_a_status(struct rzv2h_adc_priv_s *priv,
                                     unsigned long arg)
{
  struct rzv2h_adc_window_status_s empty =
    {
      0
    };

  struct rzv2h_adc_window_status_s status;
  struct rzv2h_adc_window_status_s *userstatus;
  irqstate_t flags;

  if (arg == 0)
    {
      return -EINVAL;
    }

  userstatus = (struct rzv2h_adc_window_status_s *)(uintptr_t)arg;

  flags = enter_critical_section();
  status = priv->windowastatus;
  priv->windowastatus = empty;
  leave_critical_section(flags);

  *userstatus = status;
  return OK;
}
#endif

/****************************************************************************
 * Name: rzv2h_adc_bind
 *
 * Description:
 *   Bind the NuttX ADC upper-half callback to the lower-half driver.
 *
 * Input Parameters:
 *   dev      - ADC lower-half device.
 *   callback - Upper-half callback used to deliver converted samples.
 *
 * Returned Value:
 *   OK on success.
 *
 ****************************************************************************/

static int rzv2h_adc_bind(struct adc_dev_s *dev,
                          const struct adc_callback_s *callback)
{
  struct rzv2h_adc_priv_s *priv = dev->ad_priv;

  priv->cb = callback;
  return OK;
}

/****************************************************************************
 * Name: rzv2h_adc_reset
 *
 * Description:
 *   Stop any active ADC scan and return the opened ADC to idle state. This
 *   method does not close the FSP ADC instance. It is safe to call before
 *   setup, in which case no hardware operation is required.
 *
 * Input Parameters:
 *   dev - ADC lower-half device.
 *
 ****************************************************************************/

static void rzv2h_adc_reset(struct adc_dev_s *dev)
{
  struct rzv2h_adc_priv_s *priv = dev->ad_priv;
#ifdef CONFIG_RZV2H_ADC_TRIGGER_EXTERNAL
  irqstate_t flags;

  flags = enter_critical_section();
  priv->scan_armed = false;
  leave_critical_section(flags);
#endif

#ifdef CONFIG_RZV2H_ADC_WINDOW_A
  rzv2h_adc_window_a_reset(priv);
#endif

  if (priv->opened)
    {
      (void)R_ADC_E_ScanStop(&priv->ctrl);
    }
}

/****************************************************************************
 * Name: rzv2h_adc_setup
 *
 * Description:
 *   Open and configure the single RZ/V2H ADC controller. The function
 *   configures all channels selected by the device channel mask. In
 *   interrupt mode it also attaches the NuttX scan-end wrapper and the
 *   optional Window A wrapper.
 *
 * Input Parameters:
 *   dev - ADC lower-half device.
 *
 * Returned Value:
 *   OK on success; a negative errno value on failure.
 *
 ****************************************************************************/

static int rzv2h_adc_setup(struct adc_dev_s *dev)
{
  struct rzv2h_adc_priv_s *priv = dev->ad_priv;
  adc_e_channel_cfg_t channel_cfg;
#ifdef CONFIG_RZV2H_ADC_WINDOW_A
  adc_e_window_cfg_t window_cfg =
    {
      .compare_mask       = priv->channel_mask,
#ifdef CONFIG_RZV2H_ADC_WINDOW_A_INSIDE
      .compare_mode_mask  = priv->channel_mask,
#else
      .compare_mode_mask  = ADC_E_MASK_OFF,
#endif
      .compare_cfg        = (adc_e_compare_cfg_t)
                            (ADC_E_COMPARE_CFG_A_ENABLE |
                             ADC_E_COMPARE_CFG_WINDOW_ENABLE |
                             ADC_E_COMPARE_CFG_EVENT_OUTPUT_OR),
      .compare_ref_low    = CONFIG_RZV2H_ADC_WINDOW_A_LOWER,
      .compare_ref_high   = CONFIG_RZV2H_ADC_WINDOW_A_UPPER,
      .compare_b_ref_low  = 0,
      .compare_b_ref_high = 0,
      .compare_b_channel  = ADC_E_WINDOW_B_CHANNEL_NONE,
      .compare_b_mode     = ADC_E_WINDOW_B_MODE_LESS_THAN_OR_OUTSIDE,
    };

#endif

  fsp_err_t err;
  int ret;

  if (priv->opened)
    {
      return OK;
    }

#ifdef CONFIG_RZV2H_ADC_WINDOW_A
  rzv2h_adc_window_a_reset(priv);
#endif

#ifdef CONFIG_RZV2H_ADC_SCAN_END_INTERRUPT
  ret = rzv2h_adc_irq_attach(dev);
  if (ret < 0)
    {
      return ret;
    }
#endif

#ifdef CONFIG_RZV2H_ADC_WINDOW_A
  up_disable_irq(RZV2H_ADC_WINDOW_A_IRQ);
  ret = irq_attach(RZV2H_ADC_WINDOW_A_IRQ,
                   rzv2h_adc_window_a_interrupt, dev);
  if (ret < 0)
    {
      aerr("ERROR: Failed to attach ADC Window A IRQ %d: %d\n",
           RZV2H_ADC_WINDOW_A_IRQ, ret);
      goto errout;
    }

  priv->windowairqattached = true;
#endif

  err = R_ADC_E_Open(&priv->ctrl, &g_rzv2h_adc_cfg);
  if (err != FSP_SUCCESS)
    {
      aerr("ERROR: R_ADC_E_Open failed: %d\n", err);
      ret = rzv2h_fsp_err_to_errno(err);
      goto errout;
    }

#ifdef CONFIG_RZV2H_ADC_SCAN_END_INTERRUPT
  /* ao_setup() must return with sample interrupts disabled. */

  up_disable_irq(RZV2H_FSP_TO_GIC_IRQ(g_rzv2h_adc_cfg.scan_end_irq));
#endif
#ifdef CONFIG_RZV2H_ADC_WINDOW_A
  up_disable_irq(RZV2H_ADC_WINDOW_A_IRQ);
#endif

  channel_cfg.scan_mask         = priv->channel_mask;
  channel_cfg.scan_mask_group_b = ADC_E_MASK_OFF;
  channel_cfg.scan_mask_group_c = ADC_E_MASK_OFF;
#ifdef CONFIG_RZV2H_ADC_ADD_AVERAGE_OFF
  channel_cfg.add_mask          = ADC_E_MASK_OFF;
#else
  channel_cfg.add_mask          = priv->channel_mask;
#endif
#ifdef CONFIG_RZV2H_ADC_WINDOW_A
  channel_cfg.p_window_cfg      = &window_cfg;
#else
  channel_cfg.p_window_cfg      = NULL;
#endif
  channel_cfg.priority_group_a  = ADC_E_GRPA_PRIORITY_OFF;

  err = R_ADC_E_ScanCfg(&priv->ctrl, &channel_cfg);
  if (err != FSP_SUCCESS)
    {
      aerr("ERROR: R_ADC_E_ScanCfg failed: %d\n", err);
      ret = rzv2h_fsp_err_to_errno(err);
      goto errout_close;
    }

  priv->opened = true;

  ainfo("ADC setup complete\n");
  return OK;

errout_close:
  (void)R_ADC_E_Close(&priv->ctrl);

errout:
#ifdef CONFIG_RZV2H_ADC_WINDOW_A
  rzv2h_adc_window_a_irq_detach(priv);
#endif
#ifdef CONFIG_RZV2H_ADC_SCAN_END_INTERRUPT
  rzv2h_adc_irq_detach(priv);
#endif
  return ret;
}

/****************************************************************************
 * Name: rzv2h_adc_shutdown
 *
 * Description:
 *   Disable interrupts, stop scanning, and close the FSP ADC instance.
 *
 * Input Parameters:
 *   dev - ADC lower-half device.
 *
 ****************************************************************************/

static void rzv2h_adc_shutdown(struct adc_dev_s *dev)
{
  struct rzv2h_adc_priv_s *priv = dev->ad_priv;
#ifdef CONFIG_RZV2H_ADC_TRIGGER_EXTERNAL
  irqstate_t flags;

  flags = enter_critical_section();
  priv->scan_armed = false;
  leave_critical_section(flags);
#endif

  if (priv->opened)
    {
#ifdef CONFIG_RZV2H_ADC_SCAN_END_INTERRUPT
      rzv2h_adc_irq_detach(priv);
#endif

#ifdef CONFIG_RZV2H_ADC_WINDOW_A
      rzv2h_adc_window_a_irq_detach(priv);
#endif

      (void)R_ADC_E_Close(&priv->ctrl);
      priv->opened = false;
      priv->rxenabled = false;
    }

  ainfo("ADC shutdown complete\n");
}

#ifdef CONFIG_RZV2H_ADC_SCAN_END_INTERRUPT
/****************************************************************************
 * Name: rzv2h_adc_fsp_rxint
 *
 * Description:
 *   Enable or disable the ADC group-A scan-end request. Keep the FSP cached
 *   scan-start register synchronized and change the live ADIE bit only while
 *   the converter is idle.
 *
 * Input Parameters:
 *   priv   - RZ/V2H ADC private state.
 *   enable - True to enable scan-end notification; false to disable it.
 *
 ****************************************************************************/

static void rzv2h_adc_fsp_rxint(struct rzv2h_adc_priv_s *priv,
                                bool enable)
{
  uint16_t adcsr;

  /* R_ADC_E_ScanStart() writes scan_start_adcsr to the peripheral. Keep
   * that cached value synchronized so a later scan does not undo ao_rxint().
   */

  if (enable)
    {
      priv->ctrl.scan_start_adcsr |=
        (uint16_t)R_ADC_E_ADCSR_ADIE_Msk;
    }
  else
    {
      priv->ctrl.scan_start_adcsr &=
        (uint16_t)~R_ADC_E_ADCSR_ADIE_Msk;
    }

  /* ADCSR contains the hardware-controlled ADST bit. A read-modify-
   * write while a scan is active could write a stale ADST value back to the
   * peripheral. Update the live ADIE bit only while the ADC is idle.
   */

  adcsr = priv->ctrl.p_reg->ADCSR;
  if ((adcsr & R_ADC_E_ADCSR_ADST_Msk) == 0)
    {
      adcsr &= (uint16_t)~R_ADC_E_ADCSR_ADIE_Msk;
      if (enable)
        {
          adcsr |= (uint16_t)R_ADC_E_ADCSR_ADIE_Msk;
        }

      priv->ctrl.p_reg->ADCSR = adcsr;
    }
}
#endif

/****************************************************************************
 * Name: rzv2h_adc_rxint
 *
 * Description:
 *   Enable or disable ADC sample notification. Scan-end interrupt mode
 *   controls the ADC ADIE bit and scan-end GIC interrupt. Window A controls
 *   its independent GIC interrupt in either completion mode. Polling mode
 *   gates sample delivery to the upper half.
 *
 * Input Parameters:
 *   dev    - ADC lower-half device.
 *   enable - True to enable scan-end notification; false to disable it.
 *
 ****************************************************************************/

static void rzv2h_adc_rxint(struct adc_dev_s *dev, bool enable)
{
  struct rzv2h_adc_priv_s *priv = dev->ad_priv;

  priv->rxenabled = enable;

#ifdef CONFIG_RZV2H_ADC_SCAN_END_INTERRUPT
  if (priv->irqattached)
    {
      if (enable)
        {
          rzv2h_adc_fsp_rxint(priv, true);
          up_enable_irq(
            RZV2H_FSP_TO_GIC_IRQ(g_rzv2h_adc_cfg.scan_end_irq));
        }
      else
        {
          up_disable_irq(
            RZV2H_FSP_TO_GIC_IRQ(g_rzv2h_adc_cfg.scan_end_irq));
          rzv2h_adc_fsp_rxint(priv, false);
        }
    }
#endif

#ifdef CONFIG_RZV2H_ADC_WINDOW_A
  if (priv->windowairqattached)
    {
      if (enable)
        {
          up_enable_irq(RZV2H_ADC_WINDOW_A_IRQ);
        }
      else
        {
          up_disable_irq(RZV2H_ADC_WINDOW_A_IRQ);
        }
    }
#endif
}

/****************************************************************************
 * Name: rzv2h_adc_receive
 *
 * Description:
 *   Read all channels selected for the completed scan, then deliver the
 *   complete channel set to the NuttX ADC upper half in one batch.
 *
 * Input Parameters:
 *   dev - ADC lower-half device.
 *
 * Returned Value:
 *   OK on success; a negative errno value on failure.
 *
 ****************************************************************************/

static int rzv2h_adc_receive(struct adc_dev_s *dev)
{
  struct rzv2h_adc_priv_s *priv = dev->ad_priv;
  uint32_t samples[RZV2H_ADC_NCHANNELS];
  uint8_t channels[RZV2H_ADC_NCHANNELS];
  fsp_err_t err;
  uint16_t sample;
  uint8_t channel;
  size_t count = 0;

  if (!priv->rxenabled || priv->cb == NULL)
    {
      return OK;
    }

  for (channel = 0; channel < RZV2H_ADC_NCHANNELS; channel++)
    {
      if ((priv->channel_mask & RZV2H_ADC_CHANNEL_BIT(channel)) == 0)
        {
          continue;
        }

      err = R_ADC_E_Read(&priv->ctrl, (adc_channel_t)channel, &sample);
      if (err != FSP_SUCCESS)
        {
          return rzv2h_fsp_err_to_errno(err);
        }

      channels[count] = channel;
      samples[count] = sample;
      count++;
    }

  return priv->cb->au_receive_batch(dev, channels, samples, count);
}

#if defined(CONFIG_RZV2H_ADC_SCAN_END_INTERRUPT) || \
    defined(CONFIG_RZV2H_ADC_WINDOW_A)
/****************************************************************************
 * Name: rzv2h_adc_fsp_callback
 *
 * Description:
 *   Handle scan-complete and Window A notifications from the Renesas FSP
 *   ISRs. Scan completion delivers configured channel results to the
 *   NuttX ADC upper half. A Window A match updates the event state exposed
 *   through ANIOC_RZV2H_WINDOW_A_STATUS without reading or clearing the
 *   channel result register.
 *
 * Input Parameters:
 *   args - FSP callback arguments containing the event and ADC context.
 *
 ****************************************************************************/

static void rzv2h_adc_fsp_callback(adc_callback_args_t *args)
{
  struct adc_dev_s *dev;
#if defined(CONFIG_RZV2H_ADC_TRIGGER_EXTERNAL) || \
    defined(CONFIG_RZV2H_ADC_WINDOW_A)
  struct rzv2h_adc_priv_s *priv;
#endif
#ifdef CONFIG_RZV2H_ADC_WINDOW_A
  uint32_t channel;
#endif
#ifdef CONFIG_RZV2H_ADC_TRIGGER_EXTERNAL
  irqstate_t flags;
#endif
  int ret;

  DEBUGASSERT(args != NULL);

  dev = (struct adc_dev_s *)args->p_context;
  DEBUGASSERT(dev != NULL);

#if defined(CONFIG_RZV2H_ADC_TRIGGER_EXTERNAL) || \
    defined(CONFIG_RZV2H_ADC_WINDOW_A)
  priv = dev->ad_priv;
#endif

#ifdef CONFIG_RZV2H_ADC_WINDOW_A
  if (args->event == ADC_EVENT_WINDOW_COMPARE_A)
    {
      channel = (uint32_t)args->channel;
      if (channel < RZV2H_ADC_NCHANNELS)
        {
          priv->windowastatus.pending_mask |=
            RZV2H_ADC_CHANNEL_BIT(channel);
          priv->windowastatus.total_events++;
          priv->windowastatus.channel_events[channel]++;
        }

      return;
    }
#endif

  if (args->event != ADC_EVENT_SCAN_COMPLETE)
    {
      return;
    }

#ifdef CONFIG_RZV2H_ADC_TRIGGER_EXTERNAL
  /* R_ADC_E_ScanStart() leaves an external hardware trigger enabled.
   * ANIOC_TRIGGER requests one conversion, so disable ADTRG after this
   * completed scan. Consume the one-shot arm before stopping the scan so
   * duplicate scan-end notifications from the same external trigger cannot
   * deliver duplicate samples. A subsequent ANIOC_TRIGGER call arms it
   * again.
   */

  flags = enter_critical_section();
  if (!priv->scan_armed)
    {
      leave_critical_section(flags);
      return;
    }

  (void)R_ADC_E_ScanStop(&priv->ctrl);
  priv->scan_armed = false;
  leave_critical_section(flags);
#endif

  ret = rzv2h_adc_receive(dev);
  if (ret < 0)
    {
      aerr("ERROR: ADC scan-end sample delivery failed: %d\n", ret);
    }
}
#endif

#ifdef CONFIG_RZV2H_ADC_SCAN_END_INTERRUPT
/****************************************************************************
 * Name: rzv2h_adc_interrupt
 *
 * Description:
 *   Adapt a NuttX scan-end interrupt to the Renesas FSP ADC ISR.
 *
 * Input Parameters:
 *   irq     - Raw GIC INTID supplied by the NuttX ARMv7-R dispatcher.
 *   context - Saved processor context supplied by NuttX.
 *   arg     - ADC device pointer supplied when the ISR was attached.
 *
 * Returned Value:
 *   OK after the interrupt has been handled.
 *
 ****************************************************************************/

static int rzv2h_adc_interrupt(int irq, void *context, void *arg)
{
  struct adc_dev_s *dev = arg;
  rzv2h_irqn_t fsp_irq;

  UNUSED(context);

  if (dev == NULL)
    {
      return -EINVAL;
    }

  fsp_irq = (rzv2h_irqn_t)(irq - BSP_CORTEX_VECTOR_TABLE_ENTRIES);
  if (fsp_irq != (rzv2h_irqn_t)g_rzv2h_adc_cfg.scan_end_irq)
    {
      return -EINVAL;
    }

  rzv2h_interrupt_common_handler(fsp_irq, adc_e_scan_end_isr);

  return OK;
}
#endif

#ifdef CONFIG_RZV2H_ADC_WINDOW_A
/****************************************************************************
 * Name: rzv2h_adc_window_a_interrupt
 *
 * Description:
 *   Adapt a NuttX Window A interrupt to the Renesas FSP window-compare ISR.
 *
 * Input Parameters:
 *   irq     - Raw GIC INTID supplied by the NuttX ARMv7-R dispatcher.
 *   context - Saved processor context supplied by NuttX.
 *   arg     - ADC device pointer supplied when the ISR was attached.
 *
 * Returned Value:
 *   OK after the interrupt has been handled.
 *
 ****************************************************************************/

static int rzv2h_adc_window_a_interrupt(int irq, void *context, void *arg)
{
  struct adc_dev_s *dev = arg;

  (void)context;

  DEBUGASSERT(irq == RZV2H_ADC_WINDOW_A_IRQ);
  DEBUGASSERT(dev != NULL);

  rzv2h_interrupt_common_handler(RZV2H_ADC_WINDOW_A_FSP_IRQ,
                                 adc_e_window_compare_isr);

  return OK;
}
#endif

/****************************************************************************
 * Name: rzv2h_adc_trigger
 *
 * Description:
 *   Start one conversion scan of all selected ADC channels. In
 *   software-trigger mode, conversion starts immediately. In
 *   external-trigger mode, the ADC is armed and the scan starts when an
 *   ADTRG event arrives. Synchronous ELC triggering is not supported.
 *
 *   Interrupt mode returns after the scan starts or is armed. Polling mode
 *   is supported only for the software trigger and waits for completion
 *   before delivering the samples.
 *
 * Input Parameters:
 *   dev - ADC lower-half device.
 *
 * Returned Value:
 *   OK on success; a negative errno value on failure.
 *
 ****************************************************************************/

static int rzv2h_adc_trigger(struct adc_dev_s *dev)
{
  struct rzv2h_adc_priv_s *priv = dev->ad_priv;
  fsp_err_t err;
#ifdef CONFIG_RZV2H_ADC_TRIGGER_EXTERNAL
  irqstate_t flags;
#endif

#ifndef CONFIG_RZV2H_ADC_SCAN_END_INTERRUPT
  adc_status_t status;
  uint32_t timeout;
  int ret;
#endif

  if (!priv->opened)
    {
      return -ENODEV;
    }

  switch (g_rzv2h_adc_cfg.trigger)
    {
      case ADC_TRIGGER_SOFTWARE:
        break;

      case ADC_TRIGGER_ASYNC_EXTERNAL:
#ifdef CONFIG_RZV2H_ADC_SCAN_END_INTERRUPT
        break;
#else
        return -ENOTSUP;
#endif

      case ADC_TRIGGER_SYNC_ELC:
        return -ENOTSUP;

      default:
        return -EINVAL;
    }

#ifdef CONFIG_RZV2H_ADC_TRIGGER_EXTERNAL
  /* Serialize the application arm operation with scan-end callbacks. The
   * flag is set before enabling ADTRG so even an immediate external event
   * belongs to this ANIOC_TRIGGER request. Holding the critical section
   * across R_ADC_E_ScanStart() also prevents an old pending callback from
   * consuming the new arm before the hardware has been enabled.
   */

  flags = enter_critical_section();
  if (priv->scan_armed)
    {
      leave_critical_section(flags);
      return -EBUSY;
    }

  priv->scan_armed = true;
#endif

  err = R_ADC_E_ScanStart(&priv->ctrl);
#ifdef CONFIG_RZV2H_ADC_TRIGGER_EXTERNAL
  if (err != FSP_SUCCESS)
    {
      priv->scan_armed = false;
    }

  leave_critical_section(flags);
#endif

  if (err != FSP_SUCCESS)
    {
      return rzv2h_fsp_err_to_errno(err);
    }

#ifdef CONFIG_RZV2H_ADC_SCAN_END_INTERRUPT
  return OK;
#else
  for (timeout = 0; timeout < RZV2H_ADC_TIMEOUT_US; timeout++)
    {
      err = R_ADC_E_StatusGet(&priv->ctrl, &status);
      if (err != FSP_SUCCESS)
        {
          ret = rzv2h_fsp_err_to_errno(err);
          (void)R_ADC_E_ScanStop(&priv->ctrl);
          return ret;
        }

      if (status.state == ADC_STATE_IDLE)
        {
          break;
        }

      up_udelay(1);
    }

  if (timeout == RZV2H_ADC_TIMEOUT_US)
    {
      (void)R_ADC_E_ScanStop(&priv->ctrl);
      return -ETIMEDOUT;
    }

  return rzv2h_adc_receive(dev);
#endif
}

/****************************************************************************
 * Name: rzv2h_adc_ioctl
 *
 * Description:
 *   Process ADC lower-half ioctl commands. ANIOC_TRIGGER and
 *   ANIOC_GET_NCHANNELS are supported. Window A builds also support
 *   ANIOC_RZV2H_WINDOW_A_STATUS.
 *
 * Input Parameters:
 *   dev - ADC lower-half device.
 *   cmd - NuttX ADC ioctl command.
 *   arg - Command-specific argument.
 *
 * Returned Value:
 *   A command-specific non-negative value on success; a negative errno value
 *   on failure.
 *
 ****************************************************************************/

static int rzv2h_adc_ioctl(struct adc_dev_s *dev, int cmd,
                           unsigned long arg)
{
  struct rzv2h_adc_priv_s *priv = dev->ad_priv;

  (void)arg;

  switch (cmd)
    {
      case ANIOC_TRIGGER:
        return rzv2h_adc_trigger(dev);
      case ANIOC_GET_NCHANNELS:
        return priv->nchannels;
#ifdef CONFIG_RZV2H_ADC_WINDOW_A
      case ANIOC_RZV2H_WINDOW_A_STATUS:
        return rzv2h_adc_window_a_status(priv, arg);
#endif
      default:
        aerr("ERROR: Unknown command: %d\n", cmd);
        return -ENOTTY;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rzv2h_adc_initialize
 *
 * Description:
 *   Initialize the single RZ/V2H ADC lower-half device. Hardware
 *   initialization is deferred until ao_setup().
 *
 * Input Parameters:
 *   channel_mask - Bits 0 through 7 select ADC channels ANI000 through
 *                  ANI007.
 *
 * Returned Value:
 *   Pointer to the ADC lower-half device on success; NULL on failure.
 *
 ****************************************************************************/

struct adc_dev_s *rzv2h_adc_initialize(uint32_t channel_mask)
{
  struct rzv2h_adc_priv_s *priv = &g_rzv2h_adc_priv;
  uint32_t mask;

  if (channel_mask == 0 ||
      (channel_mask & ~RZV2H_ADC_CHANNEL_MASK) != 0 ||
      priv->opened)
    {
      return NULL;
    }

  priv->channel_mask = channel_mask;
  priv->nchannels = 0;

  for (mask = channel_mask; mask != 0; mask >>= 1)
    {
      priv->nchannels += mask & 1u;
    }

  return &g_rzv2h_adc_dev;
}

#endif /* CONFIG_RZV2H_ADC */
