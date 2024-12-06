/*****************************************************************************
 * arch/arm/src/xmc4/xmc4_pwm.c
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
 *****************************************************************************/

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <nuttx/config.h>

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "xmc4_pwm.h"
#include "xmc4_gpio.h"
#include "xmc4_clockconfig.h"
#include "hardware/xmc4_ccu4.h"
#include "hardware/xmc4_scu.h"
#include "hardware/xmc4_pinmux.h"

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

/*****************************************************************************
 * Private Types
 *****************************************************************************/

/* This structure represents the state of one PWM timer */

struct xmc4_pwm_s
{
  const struct pwm_ops_s *ops; /* PWM operations */
  uint8_t module;              /* CCU4x Module number {0,...,4} */
  uint8_t slice;               /* CC4y Slice number {0,...,4} */
  uint8_t prescaler;           /* Clock division for f_tclk */
  uint32_t frequency;          /* Current frequency setting */
  ub16_t duty;                 /* Current duty setting */
  uint32_t base;               /* The base address of the CCU4x module */
  uint32_t f_tclk;             /* The frequency of the module clock */
  uint32_t outgpio;            /* The output pin config (set in board.h) */
};

/*****************************************************************************
 * Private Function Prototypes
 *****************************************************************************/

/* PWM Register access */

static inline void xmc4_pwm_putreg32(struct xmc4_pwm_s *priv,
                                     uint32_t offset,
                                     uint32_t value);
static inline uint32_t xmc4_pwm_getreg32(struct xmc4_pwm_s *priv,
                                         uint32_t offset);
static void xmc4_pwm_modifyreg32(struct xmc4_pwm_s *priv,
                                 uint32_t offset,
                                 uint32_t clearbits,
                                 uint32_t setbits);

/* PWM driver methods */

static int pwm_setup(struct pwm_lowerhalf_s *dev);
static int pwm_shutdown(struct pwm_lowerhalf_s *dev);
static int pwm_start(struct pwm_lowerhalf_s *dev,
                     const struct pwm_info_s *info);
static int pwm_stop(struct pwm_lowerhalf_s *dev);
static int pwm_ioctl(struct pwm_lowerhalf_s *dev, int cmd, unsigned long arg);

/* PWM helper method */

static int pwm_set_period_match(struct xmc4_pwm_s *priv, uint16_t period_val);
static int pwm_set_compare_match(struct xmc4_pwm_s *priv,
                                 uint16_t compare_val);
static int pwm_set_passive_level(struct xmc4_pwm_s *priv, uint8_t level);
static int pwm_shadow_transfert(struct xmc4_pwm_s *priv);
static int pwm_enable_slice_clock(struct xmc4_pwm_s *priv);
static int pwm_disable_slice_clock(struct xmc4_pwm_s *priv);
static int pwm_start_slice_timer(struct xmc4_pwm_s *priv);
static int pwm_stop_slice_timer(struct xmc4_pwm_s *priv);
static bool pwm_is_slice_timer_running(struct xmc4_pwm_s *priv);
static int pwm_set_slice_prescaler(struct xmc4_pwm_s *priv,
                                   uint8_t prescaler);
static int pwm_start_module_prescaler(struct xmc4_pwm_s *priv);
static int pwm_stop_module_prescaler(struct xmc4_pwm_s *priv);
static int pwm_enable_module(struct xmc4_pwm_s *priv);
static int pwm_disable_module(struct xmc4_pwm_s *priv);
static bool pwm_is_module_used(struct xmc4_pwm_s *priv);
static int pwm_compute_config(struct xmc4_pwm_s *priv,
                          const struct pwm_info_s *info,
                          uint8_t *prescaler,
                          uint16_t *period,
                          uint16_t *compare);
static int pwm_timer(struct xmc4_pwm_s *priv, const struct pwm_info_s *info);

/*****************************************************************************
 * Private Data
 *****************************************************************************/

static const struct pwm_ops_s g_pwmops =
    {
        .setup = pwm_setup,
        .shutdown = pwm_shutdown,
        .start = pwm_start,
        .stop = pwm_stop,
        .ioctl = pwm_ioctl,
};

#ifdef CONFIG_XMC4_CCU40
#ifdef CONFIG_XMC4_CCU40_CC40
static struct xmc4_pwm_s g_pwm00 =
    {
        .ops = &g_pwmops,
        .module = 0,
        .slice = 0,
        .base = XMC4_CCU40_BASE,
        .outgpio = GPIO_CCU40_OUT0,
};
#endif

#ifdef CONFIG_XMC4_CCU40_CC41
static struct xmc4_pwm_s g_pwm01 =
    {
        .ops = &g_pwmops,
        .module = 0,
        .slice = 1,
        .base = XMC4_CCU40_BASE,
        .outgpio = GPIO_CCU40_OUT1,
};
#endif

#ifdef CONFIG_XMC4_CCU40_CC42
static struct xmc4_pwm_s g_pwm02 =
    {
        .ops = &g_pwmops,
        .module = 0,
        .slice = 2,
        .base = XMC4_CCU40_BASE,
        .outgpio = GPIO_CCU40_OUT2,
};
#endif

#ifdef CONFIG_XMC4_CCU40_CC43
static struct xmc4_pwm_s g_pwm03 =
    {
        .ops = &g_pwmops,
        .module = 0,
        .slice = 3,
        .base = XMC4_CCU40_BASE,
        .outgpio = GPIO_CCU40_OUT3,
};
#endif
#endif /* CONFIG_XMC4_CCU40 */

#ifdef CONFIG_XMC4_CCU41
#ifdef CONFIG_XMC4_CCU41_CC40
static struct xmc4_pwm_s g_pwm10 =
    {
        .ops = &g_pwmops,
        .module = 1,
        .slice = 0,
        .base = XMC4_CCU41_BASE,
        .outgpio = GPIO_CCU41_OUT0,
};
#endif

#ifdef CONFIG_XMC4_CCU41_CC41
static struct xmc4_pwm_s g_pwm11 =
    {
        .ops = &g_pwmops,
        .module = 1,
        .slice = 1,
        .base = XMC4_CCU41_BASE,
        .outgpio = GPIO_CCU41_OUT1,
};
#endif

#ifdef CONFIG_XMC4_CCU41_CC42
static struct xmc4_pwm_s g_pwm12 =
    {
        .ops = &g_pwmops,
        .module = 1,
        .slice = 2,
        .base = XMC4_CCU41_BASE,
        .outgpio = GPIO_CCU41_OUT2,
};
#endif

#ifdef CONFIG_XMC4_CCU41_CC43
static struct xmc4_pwm_s g_pwm13 =
    {
        .ops = &g_pwmops,
        .module = 1,
        .slice = 3,
        .base = XMC4_CCU41_BASE,
        .outgpio = GPIO_CCU41_OUT3,
};
#endif
#endif /* CONFIG_XMC4_CCU41 */

#ifdef CONFIG_XMC4_CCU42
#ifdef CONFIG_XMC4_CCU42_CC40
static struct xmc4_pwm_s g_pwm20 =
    {
        .ops = &g_pwmops,
        .module = 2,
        .slice = 0,
        .base = XMC4_CCU42_BASE,
        .outgpio = GPIO_CCU42_OUT0,
};
#endif

#ifdef CONFIG_XMC4_CCU42_CC41
static struct xmc4_pwm_s g_pwm21 =
    {
        .ops = &g_pwmops,
        .module = 2,
        .slice = 1,
        .base = XMC4_CCU42_BASE,
        .outgpio = GPIO_CCU42_OUT1,
};
#endif

#ifdef CONFIG_XMC4_CCU42_CC42
static struct xmc4_pwm_s g_pwm22 =
    {
        .ops = &g_pwmops,
        .module = 2,
        .slice = 2,
        .base = XMC4_CCU42_BASE,
        .outgpio = GPIO_CCU42_OUT2,
};
#endif

#ifdef CONFIG_XMC4_CCU42_CC43
static struct xmc4_pwm_s g_pwm23 =
    {
        .ops = &g_pwmops,
        .module = 2,
        .slice = 3,
        .base = XMC4_CCU42_BASE,
        .outgpio = GPIO_CCU42_OUT3,
};
#endif
#endif /* CONFIG_XMC4_CCU42 */

#ifdef CONFIG_XMC4_CCU43
#ifdef CONFIG_XMC4_CCU43_CC40
static struct xmc4_pwm_s g_pwm30 =
    {
        .ops = &g_pwmops,
        .module = 3,
        .slice = 0,
        .base = XMC4_CCU43_BASE,
        .outgpio = GPIO_CCU43_OUT0,
};
#endif

#ifdef CONFIG_XMC4_CCU43_CC41
static struct xmc4_pwm_s g_pwm31 =
    {
        .ops = &g_pwmops,
        .module = 3,
        .slice = 1,
        .base = XMC4_CCU43_BASE,
        .outgpio = GPIO_CCU43_OUT1,
};
#endif

#ifdef CONFIG_XMC4_CCU43_CC42
static struct xmc4_pwm_s g_pwm32 =
    {
        .ops = &g_pwmops,
        .module = 3,
        .slice = 2,
        .base = XMC4_CCU43_BASE,
        .outgpio = GPIO_CCU43_OUT2,
};
#endif

#ifdef CONFIG_XMC4_CCU43_CC43
static struct xmc4_pwm_s g_pwm33 =
    {
        .ops = &g_pwmops,
        .module = 3,
        .slice = 3,
        .base = XMC4_CCU43_BASE,
        .outgpio = GPIO_CCU43_OUT3,
};
#endif
#endif /* CONFIG_XMC4_CCU43 */

/*****************************************************************************
 * Private Functions
 *****************************************************************************/

/*****************************************************************************
 * Name: xmc4_pwm_putreg32
 *
 * Description:
 *   Put a 32-bit register value by offset
 *
 *****************************************************************************/

static inline void xmc4_pwm_putreg32(struct xmc4_pwm_s *priv,
                                     uint32_t offset,
                                     uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/*****************************************************************************
 * Name: xmc4_pwm_getreg32
 *
 * Description:
 *   Get a 32-bit register value by offset
 *
 *****************************************************************************/

static inline uint32_t xmc4_pwm_getreg32(struct xmc4_pwm_s *priv,
                                         uint32_t offset)
{
  return getreg32(priv->base + offset);
}

/*****************************************************************************
 * Name: xmc4_pwm_modifyreg32
 *
 * Description:
 *   Modify a 32-bit register value by offset
 *
 *****************************************************************************/

static void xmc4_pwm_modifyreg32(struct xmc4_pwm_s *priv,
                                 uint32_t offset,
                                 uint32_t clearbits,
                                 uint32_t setbits)
{
  modifyreg32(priv->base + offset, clearbits, setbits);
}

/*****************************************************************************
 * Name: pwm_set_period_match
 *
 * Description:
 *   Set the period match register (CC4yPRS.PRS).
 *   Must call pwm_shadow_transfert() after.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 *****************************************************************************/

static int pwm_set_period_match(struct xmc4_pwm_s *priv, uint16_t period_val)
{
  DEBUGASSERT(priv != NULL);

  uint32_t cc4yprs_offset =
      (uint32_t)(XMC4_CCU4_CC40PRS_OFFSET + 0x0100 * priv->slice);

  xmc4_pwm_putreg32(priv, cc4yprs_offset, period_val);

  pwminfo("PWM CCU4%d,CC4%d period is : %d\n",
          priv->module, priv->slice, period_val);

  return OK;
}

/*****************************************************************************
 * Name: pwm_set_compare_match
 *
 * Description:
 *   Set the comapre match register (CC4yCRS.CRS).
 *   Must call pwm_shadow_transfert() after.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 *****************************************************************************/

static int pwm_set_compare_match(struct xmc4_pwm_s *priv,
                                  uint16_t compare_val)
{
  DEBUGASSERT(priv != NULL);

  uint32_t cc4ycrs_offset =
      (uint32_t)(XMC4_CCU4_CC40CRS_OFFSET + 0x0100 * priv->slice);

  xmc4_pwm_putreg32(priv, cc4ycrs_offset, compare_val);

  pwminfo("PWM CCU4%d,CC4%d compare is : %d\n",
          priv->module, priv->slice, compare_val);

  return OK;
}

/*****************************************************************************
 * Name: pwm_set_passive_level
 *
 * Description:
 *   Set the passive level of the PWM slice.
 *   Must call pwm_shadow_transfert() after.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 *****************************************************************************/

static int pwm_set_passive_level(struct xmc4_pwm_s *priv, uint8_t level)
{
  DEBUGASSERT(priv != NULL);

  uint32_t passive_level = (uint32_t)(level == 1);

  uint32_t cc4ypsl_offset =
      (uint32_t)(XMC4_CCU4_CC40PSL_OFFSET + 0x0100 * priv->slice);

  xmc4_pwm_putreg32(priv, cc4ypsl_offset, passive_level);

  pwminfo("PWM CCU4%d,CC4%d passive level is : %d\n",
          priv->module, priv->slice, passive_level);

  return OK;
}

/*****************************************************************************
 * Name: pwm_shadow_transfert
 *
 * Description:
 *   Enable the transfert of the CRS, PRS and PSL registers for the next
 *   period. Must be called if one of these register is changed.
 *   Must call pwm_shadow_transfert() after.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 *****************************************************************************/

static int pwm_shadow_transfert(struct xmc4_pwm_s *priv)
{
  DEBUGASSERT(priv != NULL);

  uint32_t shadow_transfert_mask = 1 << (priv->slice * 4);

  xmc4_pwm_modifyreg32(priv, XMC4_CCU4_GCSS_OFFSET, 0,
                       shadow_transfert_mask);

  return OK;
}

/*****************************************************************************
 * Name: pwm_enable_slice_clock
 *
 * Description:
 *   Enable the prescaller clock for the given slice.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 *****************************************************************************/

static int pwm_enable_slice_clock(struct xmc4_pwm_s *priv)
{
  DEBUGASSERT(priv != NULL);

  uint32_t mask = (1 << priv->slice);

  xmc4_pwm_modifyreg32(priv, XMC4_CCU4_GIDLC_OFFSET, 0, mask);

  pwminfo("PWM CCU4%d,CC4%d clock is enabled\n", priv->module, priv->slice);

  return OK;
}

/*****************************************************************************
 * Name: pwm_disable_slice_clock
 *
 * Description:
 *   Disable the prescaller clock for the given slice.
 *
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 *****************************************************************************/

static int pwm_disable_slice_clock(struct xmc4_pwm_s *priv)
{
  DEBUGASSERT(priv != NULL);

  uint32_t mask = (1 << priv->slice);

  xmc4_pwm_modifyreg32(priv, XMC4_CCU4_GIDLS_OFFSET, 0, mask);

  pwminfo("PWM CCU4%d,CC4%d clock is disabled\n",
          priv->module, priv->slice);

  return OK;
}

/*****************************************************************************
 * Name: pwm_start_slice_timer
 *
 * Description:
 *   Start the timer counter for the given slice.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 *****************************************************************************/

static int pwm_start_slice_timer(struct xmc4_pwm_s *priv)
{
  DEBUGASSERT(priv != NULL);

  uint32_t cc4ytcset_offset =
      (uint32_t)(XMC4_CCU4_CC40TCSET_OFFSET + 0x0100 * priv->slice);

  xmc4_pwm_putreg32(priv, cc4ytcset_offset,
                    (uint32_t)CCU4_CC4_TCSET_TRBS_MASK);

  pwminfo("PWM CCU4%d,CC4%d timer is running\n", priv->module, priv->slice);

  return OK;
}

/*****************************************************************************
 * Name: pwm_stop_slice_timer
 *
 * Description:
 *   Stop the timer counter for the given slice. Reset the counter.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 *****************************************************************************/

static int pwm_stop_slice_timer(struct xmc4_pwm_s *priv)
{
  DEBUGASSERT(priv != NULL);

  uint32_t cc4ytcclr_offset =
      (uint32_t)(XMC4_CCU4_CC40TCCLR_OFFSET + 0x0100 * priv->slice);

  xmc4_pwm_putreg32(priv, cc4ytcclr_offset,
                    (uint32_t)(CCU4_CC4_TCCLR_TRBC_MASK |
                    CCU4_CC4_TCCLR_TCC_MASK));

  pwminfo("PWM CCU4%d,CC4%d timer is stopped\n", priv->module, priv->slice);

  return OK;
}

/*****************************************************************************
 * Name: pwm_is_slice_timer_running
 *
 * Description:
 *   Get the state (running timer or not) of the slice.
 *
 *
 * Returned Value:
 *   0 if slice timer is stopped else true if timmer running.
 *
 *****************************************************************************/

static bool pwm_is_slice_timer_running(struct xmc4_pwm_s *priv)
{
  DEBUGASSERT(priv != NULL);

  uint32_t cc4ytst_offset =
      (uint32_t)(XMC4_CCU4_CC40TST_OFFSET + 0x0100 * priv->slice);

  bool status = (bool)((xmc4_pwm_getreg32(priv, cc4ytst_offset) &
                        (uint32_t)CCU4_CC4_TCST_TRB_MASK) ==
                        (uint32_t)CCU4_CC4_TCST_TRB_MASK);

  pwminfo("PWM CCU4%d,CC4%d timer status is :%d\n",
            priv->module, priv->slice, status);

  return status;
}

/*****************************************************************************
 * Name: pwm_set_slice_prescaler
 *
 * Description:
 *   Set the value of the prescaler [0-14] in CC4yPSC.PSIV.
 *   Slice must be restarted for effective change.
 *   Update the value of prescaler in related xmc4_pwm_s.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 *****************************************************************************/

static int pwm_set_slice_prescaler(struct xmc4_pwm_s *priv, uint8_t prescaler)
{
  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(prescaler < 15);

  if (prescaler >= 15)
    {
      return -EINVAL;
    }

  uint32_t cc4ypsc_offset =
      (uint32_t)(XMC4_CCU4_CC40PSC_OFFSET + 0x0100 * priv->slice);

  xmc4_pwm_putreg32(priv, cc4ypsc_offset, (uint32_t)prescaler);

  priv->prescaler = prescaler;

  pwminfo("PWM CCU4%d,CC4%d prescaler is set to : %d\n",
            priv->module, priv->slice, prescaler);

  return OK;
}

/*****************************************************************************
 * Name: pwm_start_module_prescaler
 *
 * Description:
 *   Start the CCU4x module prescaler (CCU4xGIDLC.SPRB)
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 *****************************************************************************/

static int pwm_start_module_prescaler(struct xmc4_pwm_s *priv)
{
  DEBUGASSERT(priv != NULL);

  xmc4_pwm_modifyreg32(priv, XMC4_CCU4_GIDLC_OFFSET, 0, CCU4_GIDLC_SPRB_MASK);

  pwminfo("PWM CCU4%d prescaler is started\n", priv->module);

  return OK;
}

/*****************************************************************************
 * Name: pwm_stop_module_prescaler
 *
 * Description:
 *   Stop the CCU4x module prescaler (CCU4xGIDLS.CPRB)
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 *****************************************************************************/

static int pwm_stop_module_prescaler(struct xmc4_pwm_s *priv)
{
  DEBUGASSERT(priv != NULL);

  xmc4_pwm_modifyreg32(priv, XMC4_CCU4_GIDLS_OFFSET, 0, CCU4_GIDLS_CPRB_MASK);

  pwminfo("PWM CCU4%d prescaler is stopped\n", priv->module);

  return OK;
}

/*****************************************************************************
 * Name: pwm_enable_module
 *
 * Description:
 *   Enable the CCU4x module (ungate and de-assert reset).
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 *****************************************************************************/

static int pwm_enable_module(struct xmc4_pwm_s *priv)
{
  DEBUGASSERT(priv != NULL);

  if (priv->module == 3)
    {
#ifdef XMC4_SCU_GATING
      /* Check if peripheral is gated */

      if ((getreg32(XMC4_SCU_CGATSTAT1) && SCU_CGAT1_CCU43))
        {
          putreg32(SCU_CGAT1_CCU43, XMC4_SCU_CGATCLR1); /* Ungate it */
        }

#endif
      /* Check if peripheral reset is asserted */

      if ((getreg32(XMC4_SCU_PRSTAT1) && SCU_PR1_CCU43RS))
        {
          putreg32(SCU_PR1_CCU43RS, XMC4_SCU_PRCLR1); /* De-assert reset */
        }
    }
  else
    {
      uint32_t scu_ccu4x_mask = (uint32_t)(1 << (priv->module + 2));

#ifdef XMC4_SCU_GATING
      if ((getreg32(XMC4_SCU_CGATSTAT0) && scu_ccu4x_mask))
        {
          putreg32(scu_ccu4x_mask, XMC4_SCU_CGATCLR0);
        }

#endif
      if ((getreg32(XMC4_SCU_PRSTAT0) && scu_ccu4x_mask))
        {
          putreg32(scu_ccu4x_mask, XMC4_SCU_PRCLR0);
        }
    }

  pwminfo("PWM CCU4%d module is enabled\n", priv->module);

  return OK;
}

/*****************************************************************************
 * Name: pwm_enable_module
 *
 * Description:
 *   Disable the CCU4x module (gate and assert reset).
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 *****************************************************************************/

static int pwm_disable_module(struct xmc4_pwm_s *priv)
{
  DEBUGASSERT(priv != NULL);

  if (priv->module == 3)
    {
      /* Assert reset */

      putreg32(SCU_PR1_CCU43RS, XMC4_SCU_PRCLR1);

#ifdef XMC4_SCU_GATING
      /* Gate clock */

      putreg32(SCU_CGAT1_CCU43, XMC4_SCU_CGATSET1);
#endif
    }
  else
    {
      uint32_t scu_ccu4x_mask = (uint32_t)(1 << (priv->module + 2));

      putreg32(scu_ccu4x_mask, XMC4_SCU_PRSET0);

#ifdef XMC4_SCU_GATING
      putreg32(scu_ccu4x_mask, XMC4_SCU_CGATSET0);
#endif
    }

  pwminfo("PWM CCU4%d module is disabled\n", priv->module);

  return OK;
}

/*****************************************************************************
 * Name: pwm_is_module_used
 *
 * Description:
 *   Get the state (running timer or not) of the slice.
 *
 * Returned Value:
 *   0 if module is idle, else true if one slice or more is running.
 *
 *****************************************************************************/

static bool pwm_is_module_used(struct xmc4_pwm_s *priv)
{
  DEBUGASSERT(priv != NULL);

  uint32_t idle_status = xmc4_pwm_getreg32(priv, XMC4_CCU4_GSTAT_OFFSET);
  uint32_t mask = (CCU4_GSTAT_S0I_MASK |
                   CCU4_GSTAT_S1I_MASK |
                   CCU4_GSTAT_S2I_MASK |
                   CCU4_GSTAT_S3I_MASK);

  idle_status &= mask;

  bool is_used = !(idle_status == mask);

  pwminfo("PWM CCU4%d is used ? : %d\n", priv->module, is_used);

  return is_used;
}

/*****************************************************************************
 * Name: pwm_compute_config
 *
 * Description:
 *   Compute the prescaler value, compare and period match for the given
 *   info config.
 *
 * Input Parameters:
 *   priv - A reference to the lower half PWM driver state structure
 *   info - A reference to the characteristics of the pulsed output
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 *****************************************************************************/

static int pwm_compute_config(struct xmc4_pwm_s *priv,
                          const struct pwm_info_s *info,
                          uint8_t *prescaler,
                          uint16_t *period,
                          uint16_t *compare)
{
  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(info != NULL);
  DEBUGASSERT(prescaler != NULL);
  DEBUGASSERT(period != NULL);
  DEBUGASSERT(compare != NULL);

  uint32_t f_ccu = xmc4_get_ccuclock();

  pwminfo("PWM f_ccu is %d\n", f_ccu);

  uint32_t f_tclk = 0;
  uint8_t new_prescaler = 0;
  uint32_t prs = 0;
  uint32_t crs = 0;

  uint32_t f_pwm = info->frequency;
  float duty_cycle = (1.0 - b16tof(info->duty));

  do
    {
      f_tclk = f_ccu >> new_prescaler;
      new_prescaler += 1;
      prs = f_tclk / f_pwm - 1;
      crs = (uint32_t)(duty_cycle * (prs + 1));
    }
  while ((prs > UINT16_MAX) || (crs > UINT16_MAX));

  if (new_prescaler > 15)
    {
      return -EINVAL;
    }

  *period = (uint16_t)prs;
  *compare = (uint16_t)crs;
  *prescaler = (uint8_t)(new_prescaler - 1);
  priv->f_tclk = f_tclk;

  return OK;
}

/*****************************************************************************
 * Name: pwm_timer
 *
 * Description:
 *   (Re-)initialize the timer resources and start the pulsed output
 *
 * Input Parameters:
 *   priv - A reference to the lower half PWM driver state structure
 *   info - A reference to the characteristics of the pulsed output
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 *****************************************************************************/

static int pwm_timer(struct xmc4_pwm_s *priv, const struct pwm_info_s *info)
{
  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(info != NULL);

  uint8_t new_prescaler;
  uint16_t crs;
  uint16_t prs;

  int ret = pwm_compute_config(priv, info, &new_prescaler, &prs, &crs);
  if (ret < 0)
    {
      return -EINVAL;
    }

  if (info->frequency != priv->frequency)
    {
      /* Is slice timer running ? */

      if (pwm_is_slice_timer_running(priv))
        {
          /* Slice is running */

          if (new_prescaler == priv->prescaler)
            {
              /* Prescaller doesn't change, update shadow transfert */

              pwm_set_period_match(priv, prs);
              pwm_set_compare_match(priv, crs);
              pwm_set_passive_level(priv, info->cpol);
              pwm_shadow_transfert(priv);
            }
          else
            {
              /* Stop slice to update prescaler */

              pwm_stop_slice_timer(priv);
              pwm_disable_slice_clock(priv);

              pwm_set_slice_prescaler(priv, new_prescaler);

              pwm_set_compare_match(priv, crs);
              pwm_set_period_match(priv, prs);
              pwm_set_passive_level(priv, info->cpol);
              pwm_shadow_transfert(priv);

              pwm_enable_slice_clock(priv);
              pwm_start_slice_timer(priv);
            }
        }
      else
        {
          /* Slice isn't running, start it */

          pwm_set_slice_prescaler(priv, new_prescaler);

          pwm_set_compare_match(priv, crs);
          pwm_set_period_match(priv, prs);
          pwm_set_passive_level(priv, info->cpol);
          pwm_shadow_transfert(priv);

          pwm_enable_slice_clock(priv);
          pwm_start_slice_timer(priv);
        }

      priv->frequency = info->frequency;
      priv->duty = info->duty;
    }
  else
    {
      /* Frequency doesn't change, update shadow transfert */

      pwm_set_compare_match(priv, crs);
      pwm_set_passive_level(priv, info->cpol);
      pwm_shadow_transfert(priv);

      priv->duty = info->duty;
    }

  return OK;
}

/*****************************************************************************
 * Name: pwm_setup
 *
 * Description:
 *   This method is called when the driver is opened.  The lower half driver
 *   should configure and initialize the device so that it is ready for use.
 *   It should not, however, output pulses until the start method is called.
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 *****************************************************************************/

static int pwm_setup(struct pwm_lowerhalf_s *dev)
{
  struct xmc4_pwm_s *priv = (struct xmc4_pwm_s *)dev;

  DEBUGASSERT(priv != NULL);

  int ret = OK;

  pwminfo("PWM CCU4%d,CC4%d is being used\n", priv->module, priv->slice);

  /* Set the selected GPIO as the CCU4 output alternate function */

  ret = xmc4_gpio_config((gpioconfig_t)priv->outgpio);

  /* Check if CCU4 is clocked in SCU */

  if ((getreg32(XMC4_SCU_CLKSTAT) & (uint32_t)SCU_CLK_CCUC) == 0)
    {
      /* Enable CCU4 clock */

      putreg32(SCU_CLK_CCUC, XMC4_SCU_CLKSET);
    }

  /* Enable CCU clock during sleep */

  if ((getreg32(XMC4_SCU_SLEEPCR) & (uint32_t)(SCU_SLEEPCR_CCUCR)) == 0)
    {
      putreg32(SCU_SLEEPCR_CCUCR | SCU_SLEEPCR_SYSSEL, XMC4_SCU_SLEEPCR);
    }

  /* Activate CCU4x module */

  ret |= pwm_enable_module(priv);

  /* Start the prescaller of the module */

  ret |= pwm_start_module_prescaler(priv);

  return ret;
}

static int pwm_start(struct pwm_lowerhalf_s *dev,
                     const struct pwm_info_s *info)
{
  struct xmc4_pwm_s *priv = (struct xmc4_pwm_s *)dev;

  DEBUGASSERT(priv != NULL);

  pwminfo("PWM CCU4%d,CC4%d is started\n", priv->module, priv->slice);

  return pwm_timer(priv, info);
}

/*****************************************************************************
 * Name: pwm_stop
 *
 * Description:
 *   Stop the timer resources.
 *
 * Input Parameters:
 *   priv - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 *****************************************************************************/

static int pwm_stop(struct pwm_lowerhalf_s *dev)
{
  struct xmc4_pwm_s *priv = (struct xmc4_pwm_s *)dev;

  DEBUGASSERT(priv != NULL);

  pwm_stop_slice_timer(priv);
  pwm_disable_slice_clock(priv);

  priv->frequency = 0;
  priv->duty = 0;

  pwminfo("PWM CCU4%d,CC4%d is stopped\n", priv->module, priv->slice);

  return OK;
}

/*****************************************************************************
 * Name: pwm_stop
 *
 * Description:
 *   Stop the timer resources and disable hardware module.
 *
 * Input Parameters:
 *   priv - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 *****************************************************************************/

static int pwm_shutdown(struct pwm_lowerhalf_s *dev)
{
  struct xmc4_pwm_s *priv = (struct xmc4_pwm_s *)dev;

  DEBUGASSERT(priv != NULL);

  /* Disable slice */

  pwm_stop(dev);

  /* Disable module if not used anymore */

  if (!pwm_is_module_used(priv))
    {
      pwm_disable_module(priv);
    }

  priv->f_tclk = 0;
  priv->prescaler = 0;

  return OK;
}

/*****************************************************************************
 * Name: pwm_ioctl
 *
 * Description:
 *   Lower-half logic may support platform-specific ioctl commands
 *
 *****************************************************************************/

static int pwm_ioctl(struct pwm_lowerhalf_s *dev, int cmd, unsigned long arg)
{
  struct xmc4_pwm_s *priv = (struct xmc4_pwm_s *)dev;

  DEBUGASSERT(dev);

  /* There are no platform-specific ioctl commands */

  UNUSED(priv);

  return -ENOTTY;
}

/*****************************************************************************
 * Public Functions
 *****************************************************************************/

/*****************************************************************************
 * Name: xmc4_pwminitialize
 *
 * Description:
 *   Initialize one timer for use with the upper_level PWM driver.
 *
 * Input Parameters:
 *   module - A number identifying the CCU4x use, in the range of {0,..,3}.
 *   slice - A number identifying the CC4y use, in the range of {0,..,3}.
 *
 * Returned Value:
 *   On success, a pointer to the XMC4 lower half PWM driver is returned.
 *   NULL is returned on any failure.
 *
 *****************************************************************************/

struct pwm_lowerhalf_s *xmc4_pwminitialize(int module, int slice)
{
  struct xmc4_pwm_s *lower = NULL;

  pwminfo("CCU4%u,CC4%u\n", module, slice);

  switch (module)
  {
#ifdef CONFIG_XMC4_CCU40
  case 0:
  {
    switch (slice)
    {
#ifdef CONFIG_XMC4_CCU40_CC40
    case 0:
    {
      lower = &g_pwm00;
      break;
    }
#endif

#ifdef CONFIG_XMC4_CCU40_CC41
    case 1:
    {
      lower = &g_pwm01;
      break;
    }
#endif

#ifdef CONFIG_XMC4_CCU40_CC42
    case 2:
    {
      lower = &g_pwm02;
      break;
    }
#endif

#ifdef CONFIG_XMC4_CCU40_CC43
    case 3:
    {
      lower = &g_pwm03;
      break;
    }
#endif

    default:
    {
      pwmerr("ERROR: No such CCU4%d,CC4%d existing %d\n", module, slice);
      lower = NULL;
      goto errout;
    }
    }

    break;
  }
#endif /* CONFIG_XMC4_CCU40 */

#ifdef CONFIG_XMC4_CCU41
  case 1:
  {
    switch (slice)
    {
#ifdef CONFIG_XMC4_CCU41_CC40
    case 0:
    {
      lower = &g_pwm10;
      break;
    }
#endif

#ifdef CONFIG_XMC4_CCU41_CC41
    case 1:
    {
      lower = &g_pwm11;
      break;
    }
#endif

#ifdef CONFIG_XMC4_CCU41_CC42
    case 2:
    {
      lower = &g_pwm12;
      break;
    }
#endif

#ifdef CONFIG_XMC4_CCU41_CC43
    case 3:
    {
      lower = &g_pwm13;
      break;
    }
#endif

    default:
    {
      pwmerr("ERROR: No such CCU4%d,CC4%d existing %d\n", module, slice);
      lower = NULL;
      goto errout;
    }
    }

    break;
  }
#endif /* CONFIG_XMC4_CCU41 */

#ifdef CONFIG_XMC4_CCU42
  case 2:
  {
    switch (slice)
    {
#ifdef CONFIG_XMC4_CCU42_CC40
    case 0:
    {
      lower = &g_pwm20;
      break;
    }
#endif

#ifdef CONFIG_XMC4_CCU42_CC41
    case 1:
    {
      lower = &g_pwm21;
      break;
    }
#endif

#ifdef CONFIG_XMC4_CCU42_CC42
    case 2:
    {
      lower = &g_pwm22;
      break;
    }
#endif

#ifdef CONFIG_XMC4_CCU42_CC43
    case 3:
    {
      lower = &g_pwm23;
      break;
    }
#endif

    default:
    {
      pwmerr("ERROR: No such CCU4%d,CC4%d existing %d\n", module, slice);
      lower = NULL;
      goto errout;
    }
    }

    break;
  }
#endif /* CONFIG_XMC4_CCU42 */

#ifdef CONFIG_XMC4_CCU43
  case 3:
  {
    switch (slice)
    {
#ifdef CONFIG_XMC4_CCU43_CC40
    case 0:
    {
      lower = &g_pwm30;
      break;
    }
#endif

#ifdef CONFIG_XMC4_CCU43_CC41
    case 1:
    {
      lower = &g_pwm31;
      break;
    }
#endif

#ifdef CONFIG_XMC4_CCU43_CC42
    case 2:
    {
      lower = &g_pwm32;
      break;
    }
#endif

#ifdef CONFIG_XMC4_CCU43_CC43
    case 3:
    {
      lower = &g_pwm33;
      break;
    }
#endif

    default:
    {
      pwmerr("ERROR: No such CCU4%d,CC4%d existing %d\n", module, slice);
      lower = NULL;
      goto errout;
    }
    }

    break;
  }
#endif /* CONFIG_XMC4_CCU43 */

  default:
  {
    pwmerr("ERROR: No such CCU4%d,CC4%d existing %d\n", module, slice);
    lower = NULL;
    goto errout;
  }
  }

errout:
  return (struct pwm_lowerhalf_s *)lower;
}
