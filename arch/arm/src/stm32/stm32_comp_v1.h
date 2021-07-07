/****************************************************************************
 * arch/arm/src/stm32/stm32_comp_v1.h
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

#ifndef __ARCH_ARM_SRC_STM32_STM32_COMP_V1_H
#define __ARCH_ARM_SRC_STM32_STM32_COMP_V1_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifdef CONFIG_STM32_COMP

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#define COMP_BLANKING_DEFAULT COMP_BLANKING_DIS   /* No blanking */
#define COMP_POL_DEFAULT      COMP_POL_NONINVERT  /* Output is not inverted */
#define COMP_INM_DEFAULT      COMP_INMSEL_1P4VREF /* 1/4 of Vrefint as INM */
#define COMP_OUTSEL_DEFAULT   COMP_OUTSEL_NOSEL   /* Output not selected */
#define COMP_LOCK_DEFAULT     COMP_LOCK_RW        /* Do not lock CSR register */

#ifndef CONFIG_STM32_STM32F33XX
#define COMP_MODE_DEFAULT
#define COMP_HYST_DEFAULT
#define COMP_WINMODE_DEFAULT
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Blanking source */

enum stm32_comp_blanking_e
{
  COMP_BLANKING_DIS,
#if defined(CONFIG_STM32_STM32F33XX)
  COMP_BLANKING_T1OC5,
  COMP_BLANKING_T3OC4,
  COMP_BLANKING_T2OC3,
  COMP_BLANKING_T3OC3,
  COMP_BLANKING_T15OC1,
  COMP_BLANKING_T2OC4,
  COMP_BLANKING_T15OC2,
#endif
};

/* Output polarisation */

enum stm32_comp_pol_e
{
  COMP_POL_NONINVERT,
  COMP_POL_INVERTED
};

/* Inverting input */

enum stm32_comp_inm_e
{
  COMP_INMSEL_1P4VREF,
  COMP_INMSEL_1P2VREF,
  COMP_INMSEL_3P4VREF,
  COMP_INMSEL_VREF,
  COMP_INMSEL_DAC1CH1,
  COMP_INMSEL_DAC1CH2,
  COMP_INMSEL_PIN
};

/* Output selection */

enum stm32_comp_outsel_e
{
  COMP_OUTSEL_NOSEL,
#if defined(CONFIG_STM32_STM32F33XX)
  COMP_OUTSEL_BRKACTH,
  COMP_OUTSEL_BRK2,
  COMP_OUTSEL_T1OCC,          /* COMP2 only */
  COMP_OUTSEL_T3CAP3,         /* COMP4 only */
  COMP_OUTSEL_T2CAP2,         /* COMP6 only */
  COMP_OUTSEL_T1CAP1,         /* COMP2 only */
  COMP_OUTSEL_T2CAP4,         /* COMP2 only */
  COMP_OUTSEL_T15CAP2,        /* COMP4 only */
  COMP_OUTSEL_T2OCC,          /* COMP6 only */
  COMP_OUTSEL_T16OCC,         /* COMP2 only */
  COMP_OUTSEL_T3CAP1,         /* COMP2 only */
  COMP_OUTSEL_T15OCC,         /* COMP4 only */
  COMP_OUTSEL_T16CAP1,        /* COMP6 only */
  COMP_OUTSEL_T3OCC,          /* COMP2 and COMP4 only */
#endif
};

/* CSR register lock state */

enum stm32_comp_lock_e
{
  COMP_LOCK_RW,
  COMP_LOCK_RO
};

#ifndef CONFIG_STM32_STM32F33XX

/* Hysteresis  */

enum stm32_comp_hyst_e
{
  COMP_HYST_DIS,
  COMP_HYST_LOW,
  COMP_HYST_MEDIUM,
  COMP_HYST_HIGH
};

/* Power/Speed Modes */

enum stm32_comp_mode_e
{
  COMP_MODE_HIGHSPEED,
  COMP_MODE_MEDIUMSPEED,
  COMP_MODE_LOWPOWER,
  COMP_MODE_ULTRALOWPOWER
};

/* Window mode */

enum stm32_comp_winmode_e
{
  COMP_WINMODE_DIS,
  COMP_WINMODE_EN
};

#endif

#endif /* CONFIG_STM23_COMP */
#endif /* __ARCH_ARM_SRC_STM32_STM32_COMP_V1_H */
