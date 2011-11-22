/****************************************************************************
 * arch/arm/src/stm32/stm32_gpio.c
 *
 *   Copyright (C) 2009, 2011 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Uros Platise <uros.platise@isotel.eu>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include "up_arch.h"

#include "chip.h"
#include "stm32_gpio.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/
/* Base addresses for each GPIO block */

const uint32_t g_gpiobase[STM32_NGPIO_PORTS] =
{
#if STM32_NGPIO_PORTS > 0
  STM32_GPIOA_BASE,
#endif
#if STM32_NGPIO_PORTS > 1
  STM32_GPIOB_BASE,
#endif
#if STM32_NGPIO_PORTS > 2
  STM32_GPIOC_BASE,
#endif
#if STM32_NGPIO_PORTS > 3
  STM32_GPIOD_BASE,
#endif
#if STM32_NGPIO_PORTS > 4
  STM32_GPIOE_BASE,
#endif
#if STM32_NGPIO_PORTS > 5
  STM32_GPIOF_BASE,
#endif
#if STM32_NGPIO_PORTS > 6
  STM32_GPIOG_BASE,
#endif
#if STM32_NGPIO_PORTS > 7
  STM32_GPIOH_BASE,
#endif
#if STM32_NGPIO_PORTS > 8
  STM32_GPIOI_BASE,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_gpio_configlock (for the STM32F10xxx family
 ****************************************************************************/

#if defined(CONFIG_STM32_STM32F10XX)
static int stm32_gpio_configlock(uint32_t cfgset, bool altlock)
{
  uint32_t base;
  uint32_t cr;
  uint32_t regval;
  uint32_t regaddr;
  unsigned int port;
  unsigned int pin;
  unsigned int pos;
  unsigned int modecnf;
  bool input;
 
  /* Verify that this hardware supports the select GPIO port */

  port = (cfgset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  if (port >= STM32_NGPIO_PORTS)
    {
      return -EINVAL;
    }
    
  /* Get the port base address */

  base = g_gpiobase[port];

  /* Get the pin number and select the port configuration register for that
   * pin
   */

  pin = (cfgset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  if (pin < 8)
    {
      cr  = base + STM32_GPIO_CRL_OFFSET;
      pos = pin;
    }
  else
    {
      cr  = base + STM32_GPIO_CRH_OFFSET;
      pos = pin - 8;
    }
    
  /* Input or output? */

  input = ((cfgset & GPIO_INPUT) != 0);

  /* Decode the mode and configuration */
  
  regval  = getreg32(cr);
  
  /* Is present (old) config already in GPIO_ALT? and we got request to
   * lock the alternative configuration. If so we allow the following
   * changes only:
   *  - to HiZ (unlocking the configuration)
   *  - AFPP
   *  - AFOD
   */

  uint32_t oldmode = (regval >> GPIO_CR_MODECNF_SHIFT(pos));

  if (altlock && 
        (oldmode & (GPIO_MODE_MASK >> GPIO_MODE_SHIFT)) &&                  /* previous state was output? */
        ((oldmode>>2) & GPIO_CR_CNF_ALTOD) > GPIO_CR_CNF_OUTOD &&           /* previous state is ALT? */
      ( ((cfgset & GPIO_CNF_MASK) >> GPIO_CNF_SHIFT) < GPIO_CR_CNF_ALTPP || /* new state is not output ALT? */
        input ) )                                                           /* or it is input */
    {
      return -EINVAL;
    }
      
  if (input)
    {
      /* Input.. force mode = INPUT */

      modecnf = 0;
    }
  else
    {
      /* Output or alternate function */
              
      modecnf = (cfgset & GPIO_MODE_MASK) >> GPIO_MODE_SHIFT;
    }

  modecnf |= ((cfgset & GPIO_CNF_MASK) >> GPIO_CNF_SHIFT) << 2;
     
  /* Set the port configuration register */

  regval &= ~(GPIO_CR_MODECNF_MASK(pos));
  regval |= (modecnf << GPIO_CR_MODECNF_SHIFT(pos));
  putreg32(regval, cr);

  /* Set or reset the corresponding BRR/BSRR bit */

  if (!input)
    {
      /* It is an output or an alternate function.  We have to look at the CNF
       * bits to know which.
       */

      unsigned int cnf = (cfgset & GPIO_CNF_MASK);
      if (cnf != GPIO_CNF_OUTPP && cnf != GPIO_CNF_OUTOD)
        {
          /* Its an alternate function pin... we can return early */

          return OK;
        }
    }
  else
    {
      /* It is an input pin... Should it configured as an EXTI interrupt? */

      if ((cfgset & GPIO_EXTI) != 0)
        {
          int shift;

          /* Yes.. Set the bits in the EXTI CR register */

          regaddr = STM32_AFIO_EXTICR(pin);
          regval  = getreg32(regaddr);
          shift   = AFIO_EXTICR_EXTI_SHIFT(pin);
          regval &= ~(AFIO_EXTICR_PORT_MASK << shift);
          regval |= (((uint32_t)port) << shift);
          
          putreg32(regval, regaddr);
        }

      if ((cfgset & GPIO_CNF_MASK) != GPIO_CNF_INPULLUD)
        {
          /* Neither... we can return early */

          return OK;
        }
    }
   
  /* If it is an output... set the pin to the correct initial state.
   * If it is pull-down or pull up, then we need to set the ODR
   * appropriately for that function.
   */

  if ((cfgset & GPIO_OUTPUT_SET) != 0)
    {
      /* Use the BSRR register to set the output */

      regaddr = base + STM32_GPIO_BSRR_OFFSET;
    }
  else
    {
      /* Use the BRR register to clear */

      regaddr = base + STM32_GPIO_BRR_OFFSET;
    }

  regval  = getreg32(regaddr);
  regval |= (1 << pin);
  putreg32(regval, regaddr);
  return OK;
}
#endif

/****************************************************************************
 * Name: stm32_gpio_configlock (for the STM32F40xxx family
 ****************************************************************************/

#if defined(CONFIG_STM32_STM32F40XX)
static int stm32_gpio_configlock(uint32_t cfgset, bool altlock)
{
# warning "Missing logic"
  return -ENOSYS;
}
#endif

/****************************************************************************
 * Function:  stm32_gpioremap
 *
 * Description:
 *
 *   Based on configuration within the .config file, this function will
 *   remaps positions of alternative functions. 
 * 
 ****************************************************************************/

static inline void stm32_gpioremap(void)
{
#if defined(CONFIG_STM32_STM32F10XX)

  /* Remap according to the configuration within .config file */

  uint32_t val = 0;

#ifdef CONFIG_STM32_JTAG_FULL_ENABLE
  /* The reset default */
#elif CONFIG_STM32_JTAG_NOJNTRST_ENABLE
  val |= AFIO_MAPR_SWJ;    /* enabled but without JNTRST */
#elif CONFIG_STM32_JTAG_SW_ENABLE
  val |= AFIO_MAPR_SWDP;      /* set JTAG-DP disabled and SW-DP enabled */
#else
  val |= AFIO_MAPR_DISAB;     /* set JTAG-DP and SW-DP Disabled */
#endif

#ifdef CONFIG_STM32_TIM1_FULL_REMAP
  val |= AFIO_MAPR_TIM1_FULLREMAP;
#endif
#ifdef CONFIG_STM32_TIM1_PARTIAL_REMAP
  val |= AFIO_MAPR_TIM1_PARTREMAP;
#endif
#ifdef CONFIG_STM32_TIM2_FULL_REMAP
  val |= AFIO_MAPR_TIM2_FULLREMAP;
#endif
#ifdef CONFIG_STM32_TIM2_PARTIAL_REMAP_1
  val |= AFIO_MAPR_TIM2_PARTREMAP1;
#endif
#ifdef CONFIG_STM32_TIM2_PARTIAL_REMAP_2
  val |= AFIO_MAPR_TIM2_PARTREMAP2;
#endif
#ifdef CONFIG_STM32_TIM3_FULL_REMAP
  val |= AFIO_MAPR_TIM3_FULLREMAP;
#endif
#ifdef CONFIG_STM32_TIM3_PARTIAL_REMAP
  val |= AFIO_MAPR_TIM3_PARTREMAP;
#endif
#ifdef CONFIG_STM32_TIM4_REMAP
  val |= AFIO_MAPR_TIM4_REMAP;
#endif

#ifdef CONFIG_STM32_USART1_REMAP
  val |= AFIO_MAPR_USART1_REMAP;
#endif
#ifdef CONFIG_STM32_USART2_REMAP
  val |= AFIO_MAPR_USART2_REMAP;
#endif
#ifdef CONFIG_STM32_USART3_FULL_REMAP
  val |= AFIO_MAPR_USART3_FULLREMAP;
#endif
#ifdef CONFIG_STM32_USART3_PARTIAL_REMAP
  val |= AFIO_MAPR_USART3_PARTREMAP;
#endif

#ifdef CONFIG_STM32_SPI1_REMAP
  val |= AFIO_MAPR_SPI1_REMAP;
#endif
#ifdef CONFIG_STM32_SPI3_REMAP
#endif

#ifdef CONFIG_STM32_I2C1_REMAP
  val |= AFIO_MAPR_I2C1_REMAP;
#endif

#ifdef CONFIG_STM32_CAN1_REMAP1
  val |= AFIO_MAPR_PB89;
#endif
#ifdef CONFIG_STM32_CAN1_REMAP2
  val |= AFIO_MAPR_PD01;
#endif

  putreg32(val, STM32_AFIO_MAPR);  
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  stm32_gpioinit
 *
 * Description:
 *   Based on configuration within the .config file, it does:
 *    - Remaps positions of alternative functions. 
 * 
 * Typically called from stm32_start().
 ****************************************************************************/

void stm32_gpioinit(void)
{
  /* Remap according to the configuration within .config file */

  stm32_gpioremap();
}

/****************************************************************************
 * Name: stm32_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *   Once it is configured as Alternative (GPIO_ALT|GPIO_CNF_AFPP|...) 
 *   function, it must be unconfigured with stm32_unconfiggpio() with 
 *   the same cfgset first before it can be set to non-alternative function.
 * 
 * Returns:
 *   OK on success
 *   A negated errono valu on invalid port, or when pin is locked as ALT
 *   function.
 * 
 * To-Do: Auto Power Enable
 ****************************************************************************/

int stm32_configgpio(uint32_t cfgset)
{
  return stm32_gpio_configlock(cfgset, true);
}

/****************************************************************************
 * Name: stm32_unconfiggpio
 *
 * Description:
 *   Unconfigure a GPIO pin based on bit-encoded description of the pin, set it
 *   into default HiZ state (and possibly mark it's unused) and unlock it whether
 *   it was previsouly selected as alternative function (GPIO_ALT|GPIO_CNF_AFPP|...).
 * 
 *   This is a safety function and prevents hardware from schocks, as unexpected
 *   write to the Timer Channel Output GPIO to fixed '1' or '0' while it should
 *   operate in PWM mode could produce excessive on-board currents and trigger 
 *   over-current/alarm function. 
 * 
 * Returns:
 *  OK on success
 *  A negated errno value on invalid port
 *
 * To-Do: Auto Power Disable
 ****************************************************************************/

int stm32_unconfiggpio(uint32_t cfgset)
{
  /* Reuse port and pin number and set it to default HiZ INPUT */
    
  cfgset &= GPIO_PORT_MASK | GPIO_PIN_MASK;
#if defined(CONFIG_STM32_STM32F10XX)
  cfgset |= GPIO_INPUT | GPIO_CNF_INFLOAT | GPIO_MODE_INPUT;
#elif defined(CONFIG_STM32_STM32F40XX)
  cfgset |= GPIO_INPUT | GPIO_FLOAT;
#else
# error "Unsupported STM32 chip"
#endif
    
  /* To-Do: Mark its unuse for automatic power saving options */

  return stm32_gpio_configlock(cfgset, false);
}

/****************************************************************************
 * Name: stm32_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void stm32_gpiowrite(uint32_t pinset, bool value)
{
  uint32_t base;
#if defined(CONFIG_STM32_STM32F10XX)
  uint32_t offset;
#elif defined(CONFIG_STM32_STM32F40XX)
  uint32_t bit;
#endif
  unsigned int port;
  unsigned int pin;

  port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  if (port < STM32_NGPIO_PORTS)
    {
      /* Get the port base address */

      base = g_gpiobase[port];

      /* Get the pin number  */

      pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

      /* Set or clear the output on the pin */

#if defined(CONFIG_STM32_STM32F10XX)

      if (value)
        {
          offset = STM32_GPIO_BSRR_OFFSET;
        }
      else
        {
          offset = STM32_GPIO_BRR_OFFSET;
        }

      putreg32((1 << pin), base + offset);

#elif defined(CONFIG_STM32_STM32F40XX)

      if (value)
        {
          bit = GPIO_BSRR_SET(pin);
        }
      else
        {
          bit = GPIO_BSRR_RESET(pin);
        }

      putreg32(bit, base + STM32_GPIO_BSRR_OFFSET);

#else
# error "Unsupported STM32 chip"
#endif
    }
}

/****************************************************************************
 * Name: stm32_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool stm32_gpioread(uint32_t pinset)
{
  uint32_t base;
  unsigned int port;
  unsigned int pin;

  port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  if (port < STM32_NGPIO_PORTS)
    {
      /* Get the port base address */

      base = g_gpiobase[port];

      /* Get the pin number and return the input state of that pin */

      pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
      return ((getreg32(base + STM32_GPIO_IDR_OFFSET) & (1 << pin)) != 0);
    }
  return 0;
}
