/****************************************************************************
 * arch/arm/src/stm32/stm32_comp_v2.h
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

#ifndef __ARCH_ARM_SRC_STM32_STM32_COMP_V2_H
#define __ARCH_ARM_SRC_STM32_STM32_COMP_V2_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifdef CONFIG_STM32_COMP

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Inverting input. See Table 196 in RM0440 */

enum stm32_comp_inm_e
{
  COMP_INM_1_4_VREF,
  COMP_INM_1_2_VREF,
  COMP_INM_3_4_VREF,
  COMP_INM_VREF,
  COMP_INM_DAC_1,
  COMP_INM_DAC_2,
  COMP_INM_PIN_1,
  COMP_INM_PIN_2,
};

/* Non-inverting input. See Table 195 in RM0440 */

enum stm32_comp_inp_e
{
  COMP_INP_PIN_1,
  COMP_INP_PIN_2,
};

/* Output polarity */

enum stm32_comp_pol_e
{
  COMP_POL_NONINVERT,
  COMP_POL_INVERTED
};

/* Hysteresis  */

enum stm32_comp_hyst_e
{
  COMP_HYST_DIS,
  COMP_HYST_10MV,
  COMP_HYST_20MV,
  COMP_HYST_30MV,
  COMP_HYST_40MV,
  COMP_HYST_50MV,
  COMP_HYST_60MV,
  COMP_HYST_70MV,
};

/* Blanking source */

enum stm32_comp_blanking_e
{
  COMP_BLANKING_DIS,
  COMP_BLANKING_TIMX_OCY_1,
  COMP_BLANKING_TIMX_OCY_2,
  COMP_BLANKING_TIMX_OCY_3,
  COMP_BLANKING_TIMX_OCY_4,
  COMP_BLANKING_TIMX_OCY_5,
  COMP_BLANKING_TIMX_OCY_6,
  COMP_BLANKING_TIMX_OCY_7,
};

#endif /* CONFIG_STM32_COMP */
#endif /* __ARCH_ARM_SRC_STM32_STM32_COMP_V2_H */
