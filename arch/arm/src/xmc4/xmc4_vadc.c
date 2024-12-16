/**********************************************************************************************
 * arch/arm/src/xmc4/xmc4_vadc.c
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
 **********************************************************************************************/

/**********************************************************************************************
 * May include some logic from sample code provided by Infineon:
 *
 *   Copyright (C) 2011-2015 Infineon Technologies AG. All rights reserved.
 *
 * Infineon Technologies AG (Infineon) is supplying this software for use
 * with Infineon's microcontrollers.  This file can be freely distributed
 * within development tools that are supporting such microcontrollers.
 *
 * THIS SOFTWARE IS PROVIDED AS IS. NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS
 * SOFTWARE. INFINEON SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL,
 * INCIDENTAL, OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 **********************************************************************************************/

/**********************************************************************************************
 * Included Files
 **********************************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <errno.h>
#include <assert.h>

#include <arch/xmc4/chip.h>

#include "arm_internal.h"
#include "hardware/xmc4_scu.h"
#include "xmc4_vadc.h"

/**********************************************************************************************
 * Private Data
 **********************************************************************************************/

static vadc_group_t *const g_vadc_group_array[XMC_VADC_MAXIMUM_NUM_GROUPS] =
{
  (vadc_group_t *)VADC_G0,
  (vadc_group_t *)VADC_G1,
  (vadc_group_t *)VADC_G2,
  (vadc_group_t *)VADC_G3
};

/**********************************************************************************************
 * Public Functions
 **********************************************************************************************/

/**********************************************************************************************
 * Name: xmc4_vadc_global_enable
 *
 * Description:
 *   Ungate the clock to the VADC module (if applicable) and bring the VADC
 *   module out of reset state.
 *   Called in xmc4_vadc_global_initialize.
 *
 **********************************************************************************************/

void xmc4_vadc_global_enable(void)
{
  #ifdef XMC4_SCU_GATING
  /* Check if VADC is gated */

  if ((getreg32(XMC4_SCU_CGATSTAT0) & SCU_CGAT0_VADC) == 1)
    {
      /* Disable VADC gating */

      putreg32(SCU_CGAT0_VADC, XMC4_SCU_CGATCLR0);
    }

  /* Set bit in PRCLR0 to de-assert VADC peripheral reset */

  putreg32(SCU_PR0_VADCRS, XMC4_SCU_PRCLR0);
  #else

  /* Set bit in PRCLR0 to de-assert VADC peripheral reset */

  putreg32(SCU_PR0_VADCRS, XMC4_SCU_PRCLR0);
  #endif
}

/**********************************************************************************************
 * Name: xmc4_vadc_global_disable
 *
 * Description:
 *   Gate the clock to the VADC module (if applicable) and put the VADC
 *   module into the reset state
 *
 **********************************************************************************************/

void xmc4_vadc_global_disable(void)
{
  /* Set bit in PRSET0 to assert VADC peripheral reset */

  putreg32(SCU_PR0_VADCRS, XMC4_SCU_PRSET0);

  #ifdef XMC4_SCU_GATING
  /* Check if VADC is ungated */

  if ((getreg32(XMC4_SCU_CGATSTAT0) & SCU_CGAT0_VADC) == 0)
    {
      /* Enable VADC gating */

      putreg32(SCU_CGAT0_VADC, XMC4_SCU_CGATSET0);
    }
  #endif
}

/**********************************************************************************************
 * Name: xmc4_vadc_global_initialize
 *
 * Description:
 *   Initializes the VADC global module with given configuration structure.
 *   It initializes global input classes, boundaries , result resources.
 *   Configures GLOBICLASS,GLOBBOUND,GLOBRCR registers.
 *   It also configures the global analog and digital clock dividers
 *   by setting GLOBCFG register.
 *
 **********************************************************************************************/

void xmc4_vadc_global_initialize(const vadc_global_config_t *config)
{
  /* Enable the VADC module */

  xmc4_vadc_global_enable();

  VADC->CLC = (uint32_t)(config->clc);

  /* Clock configuration, use DIVWC (write control) to write register */

  VADC->GLOBCFG = (uint32_t)(config->globcfg
                     | (uint32_t)(VADC_GLOBCFG_DIVWC_MASK));

  /* Global input class 0 configuration */

  VADC->GLOBICLASS[0] = (uint32_t)(config->class0.iclass);

  /* Global input class 1 configuration */

  VADC->GLOBICLASS[1] = (uint32_t)(config->class1.iclass);

  /* Global result generation configuration */

  VADC->GLOBRCR = (uint32_t)(config->globrcr);

  /* Set global boundaries values */

  VADC->GLOBBOUND = (uint32_t)(config->globbound);
}

/**********************************************************************************************
 * Name: xmc4_vadc_group_initialize
 *
 * Description:
 *   Initializes the VADC group module with the associated
 *   configuration structure pointed by config. Initializes group conversion
 *   class, arbiter configuration, boundary configuration by setting
 *   GxICLASS,GxARBCFG,GxBOUND, registers.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negative errno value is returned to
 *   indicate the nature of any failure.
 *
 **********************************************************************************************/

int xmc4_vadc_group_initialize(vadc_group_t *const group_ptr,
                                const vadc_group_config_t *config)
{
  if (!xmc_vadc_check_group_ptr(group_ptr))
    {
      return -EINVAL;
    }

  /* Configures the group-specific input classes,
   * each channel is assigned to a class in GxCHCTRy
   */

  group_ptr->ICLASS[0] = config->class0.iclass;
  group_ptr->ICLASS[1] = config->class1.iclass;

  /* Configures the group arbitration behavior */

  group_ptr->ARBCFG = config->g_arbcfg;

  /* Configures the group-specific boundaries */

  group_ptr->BOUND = config->g_bound;

  return OK;
}

/**********************************************************************************************
 * Name: xmc4_vadc_group_set_powermode
 *
 * Description:
 *   Configures the power mode of a VADC group. For a VADC group to
 *   actually convert an analog signal, its analog converter must be on.
 *   Configure the register bit field GxARBCFG.ANONC
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negative errno value is returned to
 *   indicate the nature of any failure.
 *
 **********************************************************************************************/

int xmc4_vadc_group_set_powermode(vadc_group_t *const group_ptr,
                                    const vadc_group_powermode_t power_mode)
{
  if (!xmc_vadc_check_group_ptr(group_ptr)
        || (power_mode > XMC_VADC_GROUP_POWERMODE_NORMAL))
    {
      return -EINVAL;
    }

  uint32_t arbcfg = group_ptr->ARBCFG;

  arbcfg &= ~((uint32_t)VADC_GXARBCFG_ANONC_MASK);
  arbcfg |= (uint32_t)power_mode;

  group_ptr->ARBCFG = arbcfg;

  return OK;
}

/**********************************************************************************************
 * Name: xmc4_vadc_global_start_calibration
 *
 * Description:
 *   Start the calibration process and loops until all active groups
 *   finish calibration. Call xmc4_vadc_global_enable and
 *   xmc4_vadc_global_initialize before calibration.
 *   Configures the register bit field GLOBCFG.SUCAL.
 *
 **********************************************************************************************/

void xmc4_vadc_global_start_calibration(void)
{
  vadc_group_t * group_ptr;

  /* Start Calibration */

  VADC->GLOBCFG |= (uint32_t) VADC_GLOBCFG_SUCAL_MASK;

  /* Loop until all groups to finish calibration */

  for (int i = 0; i < XMC_VADC_MAXIMUM_NUM_GROUPS; i++)
    {
      group_ptr = g_vadc_group_array[i];

      /* Check if group is active */

      if ((group_ptr->ARBCFG) & (uint32_t)(VADC_GXARBCFG_ANONS_MASK))
        {
          /* Loop until it finish calibration */

          while ((group_ptr->ARBCFG) & (uint32_t)VADC_GXARBCFG_CAL_MASK)
            {
              /* NOP */
            }
        }
    }
}

/**********************************************************************************************
 * Name: xmc4_vadc_group_background_enable_arbitrationslot
 *
 * Description:
 *   Enables arbitration slot of the Background request source to
 *   participate in the arbitration round. Even if a load event occurs the
 *   Background channel can only be converted when the arbiter comes to the
 *   Background slot. Thus this must be enabled if any conversion need to
 *   take place.
 *   Configure the register bit field GxARBPR.ASEN2.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negative errno value is returned to
 *   indicate the nature of any failure.
 *
 **********************************************************************************************/

int xmc4_vadc_group_background_enable_arbitrationslot(vadc_group_t *const group_ptr)
{
  if (!xmc_vadc_check_group_ptr(group_ptr))
    {
      return -EINVAL;
    }

  group_ptr->ARBPR |= (uint32_t)VADC_GXARBPR_ASEN2_MASK;

  return OK;
}

/**********************************************************************************************
 * Name: xmc4_vadc_group_background_disable_arbitrationslot
 *
 * Description:
 *   Disables arbitration slot of the Background request source to
 *   participate in the arbitration round. Even if a load event occurs the
 *   Background channel can only be converted when the arbiter comes to the
 *   Background slot. Thus this must be enabled if any conversion need to
 *   take place.
 *   Configure the register bit field GxARBPR.ASEN2.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negative errno value is returned to
 *   indicate the nature of any failure.
 *
 **********************************************************************************************/

int xmc4_vadc_group_background_disable_arbitrationslot(
                                              vadc_group_t *const group_ptr)
{
  if (!xmc_vadc_check_group_ptr(group_ptr))
    {
      return -EINVAL;
    }

  group_ptr->ARBPR &= ~((uint32_t)VADC_GXARBPR_ASEN2_MASK);

  return OK;
}

/**********************************************************************************************
 * Name: xmc4_vadc_global_background_initialize
 *
 * Description:
 *   Initializes the Background scan functional block. The BACKGROUND
 *   SCAN request source functional block converts channels of all VADC groups
 *   that have not been assigned as a priority channel (priority channels
 *   can be converted only by queue and scan). Related arbitration slot must
 *   be disabled to configure background request, then re-enabled.
 *
 **********************************************************************************************/

void xmc4_vadc_global_background_initialize(const vadc_background_config_t *config)
{
  uint32_t reg;
  uint32_t conv_start_mask;

  /* Disable background request source to change its parameters */

  for (int i = 0; i < XMC_VADC_MAXIMUM_NUM_GROUPS; i++)
    {
      xmc4_vadc_group_background_disable_arbitrationslot(g_vadc_group_array[i]);
    }

  /* Set start mode */

  conv_start_mask = (uint32_t) 0;
  if (XMC_VADC_STARTMODE_WFS != (vadc_startmode_t)config->conv_start_mode)
    {
      conv_start_mask = (uint32_t)VADC_GXARBPR_CSM2_MASK;
    }

  /* Configures GxARBPR register for each group */

  for (int i = 0; i < XMC_VADC_MAXIMUM_NUM_GROUPS; i++)
    {
      reg = g_vadc_group_array[i]->ARBPR;

      /* Program the priority of the request source : Background Scan is source 2 */

      reg &= ~(uint32_t)(VADC_GXARBPR_PRIO2_MASK);
      reg |= (uint32_t)((uint32_t)config->req_src_priority << VADC_GXARBPR_PRIO2_SHIFT);

      /* Program the start mode */

      reg |= conv_start_mask;

      g_vadc_group_array[i]->ARBPR = reg;
    }

  /* Program BackgroundRequestSourceControl register, use XT and GT write control bitfields */

  VADC->BRSCTRL = (uint32_t)(config->asctrl | (uint32_t)VADC_BRSCTRL_XTWC_MASK
                                            | (uint32_t)VADC_BRSCTRL_GTWC_MASK);

  /* Program Background Request Source Mode register */

  VADC->BRSMR = (uint32_t)((config->asmr)
                    | (uint32_t)((uint32_t)XMC_VADC_GATEMODE_IGNORE << VADC_BRSMR_ENGT_SHIFT));

  if (XMC_VADC_STARTMODE_CNR == (vadc_startmode_t)(config->conv_start_mode))
    {
      VADC->BRSMR |= (uint32_t)VADC_BRSMR_RPTDIS_MASK;
    }

  /* Re enable request source */

  for (int i = 0; i < XMC_VADC_MAXIMUM_NUM_GROUPS; i++)
    {
      xmc4_vadc_group_background_enable_arbitrationslot(g_vadc_group_array[i]);
    }
}

/**********************************************************************************************
 * Name: xmc4_vadc_group_channel_initialize
 *
 * Description:
 *   Initializes the ADC channel for conversion. Must be called after
 *   request source initialization for each channel to enable their conversion.
 *   Configures registers GxCHCTRy and boundary flag GxBFL, GxBFLC and GxCHASS.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negative errno value is returned to
 *   indicate the nature of any failure.
 *
 **********************************************************************************************/

int xmc4_vadc_group_channel_initialize(vadc_group_t *const group_ptr, const uint32_t ch_num,
                                        const vadc_channel_config_t *config)
{
  if (!xmc_vadc_check_group_ptr(group_ptr) || (ch_num > XMC_VADC_NUM_CHANNELS_PER_GROUP))
    {
      return -EINVAL;
    }

  uint32_t prio  = (uint32_t)config->channel_priority;

  /* Set channel priority, channel_priority must be 0 for background channel */

  uint32_t ch_assign  = group_ptr->CHASS;
  ch_assign &= ~((uint32_t)((uint32_t)1 << ch_num));
  ch_assign |= (uint32_t)(prio << ch_num);
  group_ptr->CHASS = ch_assign;

  /* Alias channel, can replace CH0 and CH1 with another channel number. */

  if (config->alias_channel >= (int32_t)0)
    {
      uint32_t mask = (uint32_t)0;
      if ((uint32_t)1 == ch_num)
        {
          mask = VADC_GXALIAS_ALIAS1_SHIFT;
          group_ptr->ALIAS &= ~(uint32_t)(VADC_GXALIAS_ALIAS1_MASK);
        }
      else if ((uint32_t)0 == ch_num)
        {
          mask = VADC_GXALIAS_ALIAS0_SHIFT;
          group_ptr->ALIAS &= ~(uint32_t)(VADC_GXALIAS_ALIAS0_MASK);
        }

      group_ptr->ALIAS |= (uint32_t)(config->alias_channel << mask);
    }

  /* Set channel boundaries flag */

  group_ptr->BFL |= config->bfl;
  #if (XMC_VADC_BOUNDARY_FLAG_SELECT == 1U)
    group_ptr->BFLC |= config->bflc;
  #endif

  /* Program the CHCTR register */

  group_ptr->CHCTR[ch_num] = config->chctr;

  return OK;
}

/**********************************************************************************************
 * Name: xmc4_vadc_global_background_add_channel_to_sequence
 *
 * Description:
 *   Adds a channel to the background scan sequence. The pending
 *   register are updated only after a new load event occured.
 *   Configures the register bit fields of BRSSEL.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negative errno value is returned to
 *   indicate the nature of any failure.
 *
 **********************************************************************************************/

int xmc4_vadc_global_background_add_channel_to_sequence(const uint32_t grp_num,
                                                        const uint32_t ch_num)
{
  if ((grp_num > XMC_VADC_MAXIMUM_NUM_GROUPS) || (ch_num > XMC_VADC_NUM_CHANNELS_PER_GROUP))
    {
      return -EINVAL;
    }

  VADC->BRSSEL[grp_num] |= (uint32_t)(1 << ch_num);

  return OK;
}

/**********************************************************************************************
 * Name: xmc4_vadc_global_background_enable_autoscan
 *
 * Description:
 *   Enables continuous conversion mode (autoscan). Once all channels
 *   belonging to a Background request source have been converted, a new  load
 *   event occurs.
 *   Configures the register bit field BRSMR.SCAN.
 *
 **********************************************************************************************/

void xmc4_vadc_global_background_enable_autoscan(void)
{
  VADC->BRSMR |= (uint32_t)VADC_BRSMR_SCAN_MASK;
}

/**********************************************************************************************
 * Name: xmc4_vadc_global_background_start_conversion
 *
 * Description:
 *   Generates conversion request (Software initiated conversion).
 *   The background scan must been init.
 *   Configures the register bit field BRSMR.LDEV.
 *
 **********************************************************************************************/

void xmc4_vadc_global_background_start_conversion(void)
{
  VADC->BRSMR |= (uint32_t)VADC_BRSMR_LDEV_MASK;
}

/**********************************************************************************************
 * Name: xmc4_vadc_group_get_result
 *
 * Description:
 *   Returns the result of the conversion in the given group result register.
 *   Get the register bit field GxRES.RESULT.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negative errno value is returned to
 *   indicate the nature of any failure.
 *
 **********************************************************************************************/

int xmc4_vadc_group_get_result(vadc_group_t *const group_ptr,
                                const uint32_t res_reg,
                                uint16_t *result_ptr)
{
  if (!xmc_vadc_check_group_ptr(group_ptr) || (res_reg > XMC_VADC_NUM_RESULT_REGISTERS))
    {
      return -EINVAL;
    }

  *result_ptr = (uint16_t)(group_ptr->RES[res_reg]);

  return OK;
}

/**********************************************************************************************
 * Name: xmc4_vadc_group_channel_get_result
 *
 * Description:
 *   Returns the result of the conversion of the given group channel.
 *   Get the register bit field GxRES.RESULT.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negative errno value is returned to
 *   indicate the nature of any failure.
 *
 **********************************************************************************************/

int xmc4_vadc_group_get_channel_result(vadc_group_t *const group_ptr,
                                        const uint32_t ch_num,
                                        uint16_t *result_ptr)
{
  if (!xmc_vadc_check_group_ptr(group_ptr) || (ch_num > XMC_VADC_NUM_CHANNELS_PER_GROUP))
    {
      return -EINVAL;
    }

  uint8_t resreg = (uint8_t)((group_ptr->CHCTR[ch_num]
                    & (uint32_t)VADC_GXCHCTR_RESREG_MASK) >> VADC_GXCHCTR_RESREG_SHIFT);

  *result_ptr = (uint16_t)(group_ptr->RES[resreg]);

  return OK;
}
