/****************************************************************************
 * arch/arm/src/stm32/stm32_rcc.c
 *
 *   Copyright (C) 2009, 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#include <stdint.h>
#include <debug.h>
#include <arch/board/board.h>
#include <stdio.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"
#include "stm32_rcc.h"
#include "stm32_flash.h"
#include "stm32_internal.h"
#include "stm32_waste.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define HSERDY_TIMEOUT 256

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Activity reference count, showing inactivity after start-up.
 * Device drivers increment this count using rcclock() and rccunlock()
 * 
 * If this value goes beyond the range [0, MAX_RCCs] indicates
 * reference count leakage (asymetric number of locks vs. unlocks) and
 * system enters permanent active state.
 */

#ifdef CONFIG_STM32_RCCLOCK
static int stm32_rcclock_count = 0; 
#endif
 
/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Put all RCC registers in reset state */

static inline void rcc_reset(void)
{
  uint32_t regval;

  putreg32(0, STM32_RCC_APB2RSTR);          /* Disable APB2 Peripheral Reset */
  putreg32(0, STM32_RCC_APB1RSTR);          /* Disable APB1 Peripheral Reset */
  putreg32(RCC_AHBENR_FLITFEN|RCC_AHBENR_SRAMEN, STM32_RCC_AHBENR); /* FLITF and SRAM Clock ON */
  putreg32(0, STM32_RCC_APB2ENR);           /* Disable APB2 Peripheral Clock */
  putreg32(0, STM32_RCC_APB1ENR);           /* Disable APB1 Peripheral Clock */

  regval  = getreg32(STM32_RCC_CR);         /* Set the HSION bit */
  regval |= RCC_CR_HSION;
  putreg32(regval, STM32_RCC_CR);

  regval  = getreg32(STM32_RCC_CFGR);       /* Reset SW, HPRE, PPRE1, PPRE2, ADCPRE and MCO bits */
  regval &= ~(RCC_CFGR_SW_MASK|RCC_CFGR_HPRE_MASK|RCC_CFGR_PPRE1_MASK|RCC_CFGR_PPRE2_MASK|RCC_CFGR_ADCPRE_MASK|RCC_CFGR_MCO_MASK);
  putreg32(regval, STM32_RCC_CFGR);

  regval  = getreg32(STM32_RCC_CR);         /* Reset HSEON, CSSON and PLLON bits */
  regval &= ~(RCC_CR_HSEON|RCC_CR_CSSON|RCC_CR_PLLON);
  putreg32(regval, STM32_RCC_CR);

  regval  = getreg32(STM32_RCC_CR);         /* Reset HSEBYP bit */
  regval &= ~RCC_CR_HSEBYP;
  putreg32(regval, STM32_RCC_CR);
 
  regval  = getreg32(STM32_RCC_CFGR);       /* Reset PLLSRC, PLLXTPRE, PLLMUL and USBPRE bits */
  regval &= ~(RCC_CFGR_PLLSRC|RCC_CFGR_PLLXTPRE|RCC_CFGR_PLLMUL_MASK|RCC_CFGR_USBPRE);
  putreg32(regval, STM32_RCC_CFGR);

  putreg32(0, STM32_RCC_CIR);               /* Disable all interrupts */
}

static inline void rcc_enableahb(void)
{
  uint32_t regval;

  /* Always enable FLITF clock and SRAM clock */

  regval = RCC_AHBENR_FLITFEN|RCC_AHBENR_SRAMEN;

#if CONFIG_STM32_DMA1
  /* DMA 1 clock enable */

  regval |= RCC_AHBENR_DMA1EN;
#endif

#if CONFIG_STM32_DMA2
  /* DMA 2 clock enable */

  regval |= RCC_AHBENR_DMA2EN;
#endif

#if CONFIG_STM32_CRC
  /* CRC clock enable */

  regval |= RCC_AHBENR_CRCEN;
#endif

#if CONFIG_STM32_FSMC
  /* FSMC clock enable */

  regval |=  RCC_AHBENR_FSMCEN;
#endif

#if CONFIG_STM32_SDIO
  /* SDIO clock enable */

  regval |=  RCC_AHBENR_SDIOEN;
#endif

  putreg32(regval, STM32_RCC_AHBENR);   /* Enable peripherals */
}

static inline void rcc_enableapb1(void)
{
  uint32_t regval;

#if CONFIG_STM32_USB
  /* USB clock divider. This bit must be valid before enabling the USB
   * clock in the RCC_APB1ENR register. This bit can’t be reset if the USB
   * clock is enabled.
   */

  regval  = getreg32(STM32_RCC_CFGR);
  regval &= ~RCC_CFGR_USBPRE;
  regval |= STM32_CFGR_USBPRE;
  putreg32(regval, STM32_RCC_CFGR);
#endif

  /* Set the appropriate bits in the APB1ENR register to enabled the
   * selected APB1 peripherals.
   */

  regval  = getreg32(STM32_RCC_APB1ENR);
#if CONFIG_STM32_TIM2
  /* Timer 2 clock enable */
#ifdef CONFIG_STM32_FORCEPOWER
  regval |= RCC_APB1ENR_TIM2EN;
#endif
#endif

#if CONFIG_STM32_TIM3
  /* Timer 3 clock enable */
#ifdef CONFIG_STM32_FORCEPOWER
  regval |= RCC_APB1ENR_TIM3EN;
#endif
#endif

#if CONFIG_STM32_TIM4
  /* Timer 4 clock enable */
#ifdef CONFIG_STM32_FORCEPOWER
  regval |= RCC_APB1ENR_TIM4EN;
#endif
#endif

#if CONFIG_STM32_TIM5
  /* Timer 5 clock enable */
#ifdef CONFIG_STM32_FORCEPOWER
  regval |= RCC_APB1ENR_TIM5EN;
#endif
#endif

#if CONFIG_STM32_TIM6
  /* Timer 6 clock enable */
#ifdef CONFIG_STM32_FORCEPOWER
  regval |= RCC_APB1ENR_TIM6EN;
#endif
#endif

#if CONFIG_STM32_TIM7
  /* Timer 7 clock enable */
#ifdef CONFIG_STM32_FORCEPOWER
  regval |= RCC_APB1ENR_TIM7EN;
#endif
#endif

#if CONFIG_STM32_WWDG
  /* Window Watchdog clock enable */

  regval |= RCC_APB1ENR_WWDGEN;
#endif

#if CONFIG_STM32_SPI2
  /* SPI 2 clock enable */

  regval |= RCC_APB1ENR_SPI2EN;
#endif
  
#if CONFIG_STM32_SPI3
  /* SPI 3 clock enable */

  regval |= RCC_APB1ENR_SPI3EN;
#endif

#if CONFIG_STM32_USART2
  /* USART 2 clock enable */

  regval |= RCC_APB1ENR_USART2EN;
#endif

#if CONFIG_STM32_USART3
  /* USART 3 clock enable */

  regval |= RCC_APB1ENR_USART3EN;
#endif

#if CONFIG_STM32_UART4
  /* UART 4 clock enable */

  regval |= RCC_APB1ENR_UART4EN;
#endif

#if CONFIG_STM32_UART5
  /* UART 5 clock enable */

  regval |= RCC_APB1ENR_UART5EN;
#endif

#if CONFIG_STM32_I2C1
  /* I2C 1 clock enable */
#ifdef CONFIG_STM32_FORCEPOWER
  regval |= RCC_APB1ENR_I2C1EN;
#endif
#endif

#if CONFIG_STM32_I2C2
  /* I2C 2 clock enable */
#ifdef CONFIG_STM32_FORCEPOWER
  regval |= RCC_APB1ENR_I2C2EN;
#endif
#endif

#if CONFIG_STM32_USB
  /* USB clock enable */

  regval |= RCC_APB1ENR_USBEN;
#endif

#if CONFIG_STM32_CAN
  /* CAN clock enable */

  regval |= RCC_APB1ENR_CANEN;
#endif

#if CONFIG_STM32_BKP
  /* Backup interface clock enable */

  regval |= RCC_APB1ENR_BKPEN;
#endif

#if CONFIG_STM32_PWR
  /*  Power interface clock enable */

  regval |= RCC_APB1ENR_PWREN;
#endif

#if CONFIG_STM32_DAC
  /* DAC interface clock enable */

  regval |= RCC_APB1ENR_DACEN;
#endif
  putreg32(regval, STM32_RCC_APB1ENR);
}

static inline void rcc_enableapb2(void)
{
  uint32_t regval;

  /* Set the appropriate bits in the APB2ENR register to enabled the
   * selected APB2 peripherals.
   */

  /* Enable GPIOA, GPIOB, ... and AFIO clocks */

  regval = getreg32(STM32_RCC_APB2ENR);
  regval |= (RCC_APB2ENR_AFIOEN
#if STM32_NGPIO > 0
             |RCC_APB2ENR_IOPAEN
#endif
#if STM32_NGPIO > 16
             |RCC_APB2ENR_IOPBEN
#endif
#if STM32_NGPIO > 32
             |RCC_APB2ENR_IOPCEN
#endif
#if STM32_NGPIO > 48
             |RCC_APB2ENR_IOPDEN
#endif
#if STM32_NGPIO > 64
             |RCC_APB2ENR_IOPEEN
#endif
#if STM32_NGPIO > 80
             |RCC_APB2ENR_IOPFEN
#endif
#if STM32_NGPIO > 96
             |RCC_APB2ENR_IOPGEN
#endif
             );

#if CONFIG_STM32_ADC1
  /* ADC 1 interface clock enable */

  regval |= RCC_APB2ENR_ADC1EN;
#endif

#if CONFIG_STM32_ADC2
  /* ADC 2 interface clock enable */

  regval |= RCC_APB2ENR_ADC2EN;
#endif

#if CONFIG_STM32_TIM1
  /* TIM1 Timer clock enable */
#ifdef CONFIG_STM32_FORCEPOWER
  regval |= RCC_APB2ENR_TIM1EN;
#endif
#endif

#if CONFIG_STM32_SPI1
  /* SPI 1 clock enable */

  regval |= RCC_APB2ENR_SPI1EN;
#endif

#if CONFIG_STM32_TIM8
  /* TIM8 Timer clock enable */
#ifdef CONFIG_STM32_FORCEPOWER
  regval |= RCC_APB2ENR_TIM8EN;
#endif
#endif

#if CONFIG_STM32_USART1
  /* USART1 clock enable */

  regval |= RCC_APB2ENR_USART1EN;
#endif

#if CONFIG_STM32_ADC3
  /*ADC3 interface clock enable */

  regval |= RCC_APB2ENR_ADC3EN;
#endif
  putreg32(regval, STM32_RCC_APB2ENR);
}

#if !defined(CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG)

/* Called to change to new clock based on settings in board.h
 * 
 *   NOTE:  This logic would need to be extended if you need to select low-
 *   power clocking modes!
 */
 
static inline void stm32_stdclockconfig(void)
{
  uint32_t regval;
  volatile int32_t timeout;

  /* Enable External High-Speed Clock (HSE) */
 
  regval  = getreg32(STM32_RCC_CR);
  regval &= ~RCC_CR_HSEBYP;         /* Disable HSE clock bypass */
  regval |= RCC_CR_HSEON;           /* Enable HSE */
  putreg32(regval, STM32_RCC_CR);
   
  /* Wait until the HSE is ready (or until a timeout elapsed) */

  for (timeout = HSERDY_TIMEOUT; timeout > 0; timeout--)
  {
    /* Check if the HSERDY flag is the set in the CR */

    if ((getreg32(STM32_RCC_CR) & RCC_CR_HSERDY) != 0)
      {
        /* If so, then break-out with timeout > 0 */

        break;
      }
  }

  if( timeout > 0)
    {
    /* Enable FLASH prefetch buffer and 2 wait states */

    regval  = getreg32(STM32_FLASH_ACR);
    regval &= ~FLASH_ACR_LATENCY_MASK;
    regval |= (FLASH_ACR_LATENCY_2|FLASH_ACR_PRTFBE);
    putreg32(regval, STM32_FLASH_ACR);

    /* Set the HCLK source/divider */
 
    regval = getreg32(STM32_RCC_CFGR);
    regval &= ~RCC_CFGR_HPRE_MASK;
    regval |= STM32_RCC_CFGR_HPRE;
    putreg32(regval, STM32_RCC_CFGR);

    /* Set the PCLK2 divider */

    regval = getreg32(STM32_RCC_CFGR);
    regval &= ~RCC_CFGR_PPRE2_MASK;
    regval |= STM32_RCC_CFGR_PPRE2;
    putreg32(regval, STM32_RCC_CFGR);
  
    /* Set the PCLK1 divider */

    regval = getreg32(STM32_RCC_CFGR);
    regval &= ~RCC_CFGR_PPRE1_MASK;
    regval |= STM32_RCC_CFGR_PPRE1;
    putreg32(regval, STM32_RCC_CFGR);
 
    /* Set the PLL divider and multipler */

    regval = getreg32(STM32_RCC_CFGR);
    regval &= ~(RCC_CFGR_PLLSRC|RCC_CFGR_PLLXTPRE|RCC_CFGR_PLLMUL_MASK);
    regval |= (STM32_CFGR_PLLSRC|STM32_CFGR_PLLXTPRE|STM32_CFGR_PLLMUL);
    putreg32(regval, STM32_RCC_CFGR);
 
    /* Enable the PLL */

    regval = getreg32(STM32_RCC_CR);
    regval |= RCC_CR_PLLON;
    putreg32(regval, STM32_RCC_CR);
 
    /* Wait until the PLL is ready */
  
    while ((getreg32(STM32_RCC_CR) & RCC_CR_PLLRDY) == 0);
 
    /* Select the system clock source (probably the PLL) */
 
    regval  = getreg32(STM32_RCC_CFGR);
    regval &= ~RCC_CFGR_SW_MASK;
    regval |= STM32_SYSCLK_SW;
    putreg32(regval, STM32_RCC_CFGR);

    /* Wait until the selected source is used as the system clock source */
  
    while ((getreg32(STM32_RCC_CFGR) & RCC_CFGR_SWS_MASK) != STM32_SYSCLK_SWS);
  }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void stm32_clockconfig(void)
{
  /* Make sure that we are starting in the reset state */

  rcc_reset();
  
#if defined(CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG)

  /* Invoke Board Custom Clock Configuration */

  stm32_board_clockconfig();
  
#else

  /* Invoke standard, fixed clock configuration based on definitions in board.h */

  stm32_stdclockconfig();

#endif

  /* Enable peripheral clocking */
  
  rcc_enableahb();
  rcc_enableapb2();
  rcc_enableapb1();
}

/*
 * \todo Check for LSE good timeout and return with -1,
 *   possible ISR optimization? or at least ISR should be cough in case of failure
 */

void stm32_rcc_enablelse(void)
{
  /* Enable LSE */

  modifyreg16(STM32_RCC_BDCR, 0, RCC_BDCR_LSEON);

  /* We could wait for ISR here ... */

  while( !(getreg16(STM32_RCC_BDCR) & RCC_BDCR_LSERDY) ) up_waste();
    
  /* Select LSE as RTC Clock Source */

  modifyreg16(STM32_RCC_BDCR, RCC_BDCR_RTCSEL_MASK, RCC_BDCR_RTCSEL_LSE);
    
  /* Enable Clock */

  modifyreg16(STM32_RCC_BDCR, 0, RCC_BDCR_RTCEN);    
}

#ifdef CONFIG_STM32_RCCLOCK
uint32_t stm32_rcclock(uint8_t domain_id)
{
  // THINK:
  // maybe just shift domain_id into 32-bit or 64-bit register 
  // and if there value of this var != 0, we are active...
  // increment some variable, so it is possible to test leakage
  // multiple locks or multiple unlocks
    
  if (stm32_rcclock_count >= 0)
    {
      stm32_rcclock_count++;
      if (stm32_rcclock_count > 64)
        {
          stm32_rcclock_count = -1; /* capture error */
        }
    }
    
  return 0;
}

uint32_t stm32_rccunlock(uint8_t domain_id)
{
    if (stm32_rcclock_count > -1)
      {
        stm32_rcclock_count--;
      }
    return 0;
}

uint32_t stm32_setrccoptions(uint8_t domain_id, uint32_t options)
{
}

int stm32_getrccactivity(void)
{
    return stm32_rcclock_count;
}
#endif
