/****************************************************************************
 * arch/arm/src/stm32l4/stm32l4_comp.h
 *
 * Copyright (c) 2016 Motorola Mobility, LLC.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32L4_STM32L4_COMP_H
#define __ARCH_ARM_SRC_STM32L4_STM32L4_COMP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#include "hardware/stm32l4_comp.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

#if defined(CONFIG_STM32L4_STM32L4X3)

/* Comparators */

enum stm32l4_comp_e
{
  STM32L4_COMP1,
  STM32L4_COMP2,
  STM32L4_COMP_NUM          /* Number of comparators */
};

/* Plus input */

enum stm32l4_comp_inp_e
{
  STM32L4_COMP_INP_PIN_1,   /* COMP1: PC5, COMP2: PB4 */
  STM32L4_COMP_INP_PIN_2,   /* COMP1: PB2, COMP2: PB6 */
  STM32L4_COMP_INP_PIN_3    /* COMP1: PA1, COMP2: PA3 */
};

/* Minus input */

enum stm32l4_comp_inm_e
{
  STM32L4_COMP_INM_1_4_VREF,
  STM32L4_COMP_INM_1_2_VREF,
  STM32L4_COMP_INM_3_4_VREF,
  STM32L4_COMP_INM_VREF,
  STM32L4_COMP_INM_DAC_1,
  STM32L4_COMP_INM_DAC_2,
  STM32L4_COMP_INM_PIN_1,   /* COMP1: PB1, COMP2: PB3 */
  STM32L4_COMP_INM_PIN_2,   /* COMP1: PC4, COMP2: PB7 */
  STM32L4_COMP_INM_PIN_3,   /* COMP1: PA0, COMP2: PA2 */
  STM32L4_COMP_INM_PIN_4,   /* COMP1: PA4, COMP2: PA4 */
  STM32L4_COMP_INM_PIN_5    /* COMP1: PA5, COMP2: PA5 */
};

#else

/* Comparators */

enum stm32l4_comp_e
{
  STM32L4_COMP1,
  STM32L4_COMP2,
  STM32L4_COMP_NUM          /* Number of comparators */
};

/* Plus input */

enum stm32l4_comp_inp_e
{
  STM32L4_COMP_INP_PIN_1,   /* COMP1: PC5, COMP2: PB4 */
  STM32L4_COMP_INP_PIN_2    /* COMP1: PB2, COMP2: PB6 */
};

/* Minus input */

enum stm32l4_comp_inm_e
{
  STM32L4_COMP_INM_1_4_VREF,
  STM32L4_COMP_INM_1_2_VREF,
  STM32L4_COMP_INM_3_4_VREF,
  STM32L4_COMP_INM_VREF,
  STM32L4_COMP_INM_DAC_1,
  STM32L4_COMP_INM_DAC_2,
  STM32L4_COMP_INM_PIN_1,   /* COMP1: PB1, COMP2: PB3 */
  STM32L4_COMP_INM_PIN_2    /* COMP1: PC4, COMP2: PB7 */
};
#endif

/* Hysteresis */

enum stm32l4_comp_hyst_e
{
  STM32L4_COMP_HYST_NONE,
  STM32L4_COMP_HYST_LOW,
  STM32L4_COMP_HYST_MEDIUM,
  STM32L4_COMP_HYST_HIGH
};

/* Power/Speed Modes */

enum stm32l4_comp_speed_e
{
  STM32L4_COMP_SPEED_HIGH,
  STM32L4_COMP_SPEED_MEDIUM,
  STM32L4_COMP_SPEED_LOW
};

/* Comparator configuration *************************************************/

struct stm32l4_comp_config_s
{
  struct
  {
    const struct comp_callback_s *cb;
    bool                          rising;
    bool                          falling;
  } interrupt;

  uint8_t  inp;                 /* Plus input pin (see enum stm32l4_comp_inp_e) */
  uint8_t  inm;                 /* Minus input pin (see enum stm32l4_comp_inm_e) */
  uint8_t  hyst;                /* Hysteresis (see enum stm32l4_comp_hyst_e) */
  uint8_t  speed;               /* Speed (see stm32l4_comp_speed_e) */
  bool     inverted;            /* Invert output? */
  uint32_t csr;                 /* Control and status register */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: stm32l4_compinitialize
 *
 * Description:
 *   Initialize the COMP.
 *
 * Input Parameters:
 *   intf - The COMP interface number.
 *   cfg  - configuration
 *
 * Returned Value:
 *   Valid COMP device structure reference on success; a NULL on failure.
 *
 * Assumptions:
 *   1. Clock to the COMP block has enabled,
 *   2. Board-specific logic has already configured
 *
 ****************************************************************************/

struct
comp_dev_s *stm32l4_compinitialize(int intf,
                                   const struct stm32l4_comp_config_s *cfg);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_STM32L4_STM32L4_COMP_H */
