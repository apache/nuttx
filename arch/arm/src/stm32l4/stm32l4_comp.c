/************************************************************************************
 * arch/arm/src/stm32l4/stm32l4_comp.c
 *
 *   Copyright (c) 2017 Gregory Nutt. All rights reserved.
 *
 * Based on COMP driver from the Motorola MDK:
 *
 *   Copyright (c) 2016 Motorola Mobility, LLC. All rights reserved.
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include "chip.h"
#include "stm32l4_comp.h"
#include "stm32l4_gpio.h"
#include "up_arch.h"

#include <errno.h>

#if !(defined(CONFIG_STM32L4_STM32L4X3) || defined(CONFIG_STM32L4_STM32L4X6))
#  error "Unrecognized STM32 chip"
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: modify_csr
 ************************************************************************************/

static inline void modify_csr(int cmp, uint32_t clearbits, uint32_t setbits)
{
  modifyreg32(cmp == STM32L4_COMP1 ? STM32L4_COMP1_CSR : STM32L4_COMP2_CSR,
              clearbits, setbits);
}

/************************************************************************************
 * Name: get_csr
 ************************************************************************************/

static inline uint32_t get_csr(int cmp)
{
  return getreg32(cmp == STM32L4_COMP1 ? STM32L4_COMP1_CSR : STM32L4_COMP2_CSR);
}

/*************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32l4_compconfig
 *
 * Description:
 *   Configure comparator and I/Os used as comparators inputs
 *
 * Parameters:
 *  cmp - comparator
 *  cfg - configuration
 *
 * Returns:
 *  0 on success, a negated errno value on failure
 *
 ************************************************************************************/

int stm32l4_compconfig(int cmp, const struct stm32l4_comp_config_s *cfg)
{
  uint32_t regval = 0;
  uint32_t mask = 0;
  uint32_t clearbits;
  uint32_t setbits;

  /* Input plus */

  mask |= COMP_CSR_INPSEL_MASK;
  switch (cfg->inp)
    {
    case STM32L4_COMP_INP_PIN_1:
      stm32l4_configgpio(cmp == STM32L4_COMP1 ? GPIO_COMP1_INP_1 : GPIO_COMP2_INP_1);
      regval |= COMP_CSR_INPSEL_PIN1;
      break;

    case STM32L4_COMP_INP_PIN_2:
      stm32l4_configgpio(cmp == STM32L4_COMP1 ? GPIO_COMP1_INP_2 : GPIO_COMP2_INP_2);
      regval |= COMP_CSR_INPSEL_PIN2;
      break;

#if defined(CONFIG_STM32L4_STM32L4X3)
    case STM32L4_COMP_INP_PIN_3:
      stm32l4_configgpio(cmp == STM32L4_COMP1 ? GPIO_COMP1_INP_3 : GPIO_COMP2_INP_3);
      regval |= COMP_CSR_INPSEL_PIN3;
      break;
#endif

    default:
      return -EINVAL;
    }

  /* Input minus */

  mask |= COMP_CSR_INMSEL_MASK;
  switch (cfg->inm)
    {
    case STM32L4_COMP_INM_1_4_VREF:
      regval |= COMP_CSR_INMSEL_25PCT;
      mask   |= (COMP_CSR_SCALEN | COMP_CSR_BRGEN);
      regval |= (COMP_CSR_SCALEN | COMP_CSR_BRGEN);
      break;

    case STM32L4_COMP_INM_1_2_VREF:
      regval |= COMP_CSR_INMSEL_50PCT;
      mask   |= (COMP_CSR_SCALEN | COMP_CSR_BRGEN);
      regval |= (COMP_CSR_SCALEN | COMP_CSR_BRGEN);
      break;

    case STM32L4_COMP_INM_3_4_VREF:
      regval |= COMP_CSR_INMSEL_75PCT;
      mask   |= (COMP_CSR_SCALEN | COMP_CSR_BRGEN);
      regval |= (COMP_CSR_SCALEN | COMP_CSR_BRGEN);
      break;

    case STM32L4_COMP_INM_VREF:
      regval |= COMP_CSR_INMSEL_VREF;
      mask   |= (COMP_CSR_SCALEN | COMP_CSR_BRGEN);
      regval |= COMP_CSR_SCALEN;
      break;

    case STM32L4_COMP_INM_DAC_1:
      regval |= COMP_CSR_INMSEL_DAC1;
      break;

    case STM32L4_COMP_INM_DAC_2:
      regval |= COMP_CSR_INMSEL_DAC2;
      break;

  case STM32L4_COMP_INM_PIN_1:
      stm32l4_configgpio(cmp == STM32L4_COMP1 ? GPIO_COMP1_INM_1 : GPIO_COMP2_INM_1);
      regval |= COMP_CSR_INMSEL_PIN1;
      break;

  case STM32L4_COMP_INM_PIN_2:
      stm32l4_configgpio(cmp == STM32L4_COMP1 ? GPIO_COMP1_INM_2 : GPIO_COMP2_INM_2);
#if defined(CONFIG_STM32L4_STM32L4X6)
      regval |= COMP_CSR_INMSEL_PIN2;
#else
      regval |= COMP_CSR_INMSEL_INMESEL;
      mask   |= COMP_CSR_INMESEL_MASK;
      regval |= COMP_CSR_INMSEL_PIN2;
#endif
      break;

#if defined(CONFIG_STM32L4_STM32L4X3)
    case STM32L4_COMP_INM_PIN_3:
      stm32l4_configgpio(cmp == STM32L4_COMP1 ? GPIO_COMP1_INM_3 : GPIO_COMP2_INM_3);
      regval |= COMP_CSR_INMSEL_INMESEL;
      mask   |= COMP_CSR_INMESEL_MASK;
      regval |= COMP_CSR_INMSEL_PIN3;
      break;

    case STM32L4_COMP_INM_PIN_4:
      stm32l4_configgpio(cmp == STM32L4_COMP1 ? GPIO_COMP1_INM_4 : GPIO_COMP2_INM_4);
      regval |= COMP_CSR_INMSEL_INMESEL;
      mask   |= COMP_CSR_INMESEL_MASK;
      regval |= COMP_CSR_INMSEL_PIN4;
      break;

    case STM32L4_COMP_INM_PIN_5:
      stm32l4_configgpio(cmp == STM32L4_COMP1 ? GPIO_COMP1_INM_5 : GPIO_COMP2_INM_5);
      regval |= COMP_CSR_INMSEL_INMESEL;
      mask   |= COMP_CSR_INMESEL_MASK;
      regval |= COMP_CSR_INMSEL_PIN5;
      break;

#endif
    default:
      return -EINVAL;
    }

  /* Hysteresis */

  mask |= COMP_CSR_HYST_MASK;
  switch (cfg->hyst)
    {
    case STM32L4_COMP_HYST_NONE:
      regval |= COMP_CSR_HYST_NONE;
      break;

    case STM32L4_COMP_HYST_LOW:
      regval |= COMP_CSR_HYST_LOW;
      break;

    case STM32L4_COMP_HYST_MEDIUM:
      regval |= COMP_CSR_HYST_MEDIUM;
      break;

    case STM32L4_COMP_HYST_HIGH:
      regval |= COMP_CSR_HYST_HIGH;
      break;

    default:
      return -EINVAL;
    }

  /* Power/speed Mode */

  mask |= COMP_CSR_PWRMODE_MASK;
  switch(cfg->speed)
    {
    case STM32L4_COMP_SPEED_HIGH:
      regval |= COMP_CSR_PWRMODE_HIGH;
      break;

    case STM32L4_COMP_SPEED_MEDIUM:
      regval |= COMP_CSR_PWRMODE_MEDIUM;
      break;

    case STM32L4_COMP_SPEED_LOW:
      regval |= COMP_CSR_PWRMODE_LOW;
      break;

    default:
      return -EINVAL;
    }

  /* Polarity */

  mask |= COMP_CSR_POLARITY_MASK;
  if (cfg->inverted)
    {
        regval |= COMP_CSR_POLARITY_INVERT;
    }

  /* Disable blanking */

  mask   |= COMP_CSR_BLANK_MASK;
  regval |= COMP_CSR_BLANK_NONE;

  clearbits = regval ^ mask;
  setbits = regval;

  modify_csr(cmp, clearbits, setbits);
  return 0;
}

/************************************************************************************
 * Name: stm32l4_compenable
 *
 * Description:
 *   Enable/disable comparator
 *
 * Parameters:
 *  cmp - comparator
 *  cfg - enable/disable flag
 *
 * Returns:
 *  0 on success, a negated errno value on failure
 *
 ************************************************************************************/

int stm32l4_compenable(int cmp, bool en)
{
  uint32_t clearbits = en ? 0 : COMP_CSR_EN;
  uint32_t setbits = en ? COMP_CSR_EN : 0;

  modify_csr(cmp, clearbits, setbits);
  return 0;
}

/************************************************************************************
 * Name: stm32l4_compread
 *
 * Description:
 *   Read comparator output
 *
 * Parameters:
 *  - cmp: comparator
 *
 * Returns:
 *  true for high, false for low
 *
 ************************************************************************************/

bool stm32l4_compread(int cmp)
{
  return !!(get_csr(cmp) & COMP_CSR_VALUE);
}
