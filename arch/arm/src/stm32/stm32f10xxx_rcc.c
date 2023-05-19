/****************************************************************************
 * arch/arm/src/stm32/stm32f10xxx_rcc.c
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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Allow up to 100 milliseconds for the high speed clock to become ready.
 * that is a very long delay, but if the clock does not become ready we are
 * hosed anyway.  Normally this is very fast, but I have seen at least one
 * board that required this long, long timeout for the HSE to be ready.
 */

#define HSERDY_TIMEOUT (100 * CONFIG_BOARD_LOOPSPERMSEC)

/* The FLASH latency depends on the system clock.
 *
 * Calculate the wait cycles, based on STM32_SYSCLK_FREQUENCY:
 * 0WS from 0-24MHz
 * 1WS from 24-48MHz
 * 2WS from 48-72MHz
 */

#if (STM32_SYSCLK_FREQUENCY <= 24000000)
#  define FLASH_ACR_LATENCY_SETTING  FLASH_ACR_LATENCY_0
#elif (STM32_SYSCLK_FREQUENCY <= 48000000)
#  define FLASH_ACR_LATENCY_SETTING  FLASH_ACR_LATENCY_1
#elif (STM32_SYSCLK_FREQUENCY <= 78000000)
#  define FLASH_ACR_LATENCY_SETTING  FLASH_ACR_LATENCY_2
#else
#  error "STM32_SYSCLK_FREQUENCY is out of range!"
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rcc_reset
 *
 * Description:
 *   Put all RCC registers in reset state
 *
 ****************************************************************************/

static inline void rcc_reset(void)
{
  uint32_t regval;

  putreg32(0, STM32_RCC_APB2RSTR);          /* Disable APB2 Peripheral Reset */
  putreg32(0, STM32_RCC_APB1RSTR);          /* Disable APB1 Peripheral Reset */
  putreg32(RCC_AHBENR_FLITFEN | RCC_AHBENR_SRAMEN,
           STM32_RCC_AHBENR);               /* FLITF and SRAM Clock ON */
  putreg32(0, STM32_RCC_APB2ENR);           /* Disable APB2 Peripheral Clock */
  putreg32(0, STM32_RCC_APB1ENR);           /* Disable APB1 Peripheral Clock */

  regval  = getreg32(STM32_RCC_CR);         /* Set the HSION bit */
  regval |= RCC_CR_HSION;
  putreg32(regval, STM32_RCC_CR);

  regval  = getreg32(STM32_RCC_CFGR);       /* Reset SW, HPRE, PPRE1, PPRE2, ADCPRE and MCO bits */
  regval &= ~(RCC_CFGR_SW_MASK | RCC_CFGR_HPRE_MASK | RCC_CFGR_PPRE1_MASK
              | RCC_CFGR_PPRE2_MASK | RCC_CFGR_ADCPRE_MASK
              | RCC_CFGR_MCO_MASK);
  putreg32(regval, STM32_RCC_CFGR);

  regval  = getreg32(STM32_RCC_CR);         /* Reset HSEON, CSSON and PLLON bits */
  regval &= ~(RCC_CR_HSEON | RCC_CR_CSSON | RCC_CR_PLLON);
  putreg32(regval, STM32_RCC_CR);

  regval  = getreg32(STM32_RCC_CR);         /* Reset HSEBYP bit */
  regval &= ~RCC_CR_HSEBYP;
  putreg32(regval, STM32_RCC_CR);

  regval  = getreg32(STM32_RCC_CFGR);       /* Reset PLLSRC, PLLXTPRE, PLLMUL and USBPRE bits */
  regval &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMUL_MASK
#ifndef CONFIG_STM32_VALUELINE
              | RCC_CFGR_USBPRE
#endif
              );
  putreg32(regval, STM32_RCC_CFGR);

  putreg32(0, STM32_RCC_CIR);               /* Disable all interrupts */
}

/****************************************************************************
 * Name: rcc_enableahb
 *
 * Description:
 *   Enable selected AHB peripherals
 *
 ****************************************************************************/

static inline void rcc_enableahb(void)
{
  uint32_t regval;

#if defined(CONFIG_STM32_CONNECTIVITYLINE) && defined(CONFIG_STM32_OTGFS)
  /* USB clock divider for USB OTG FS.  This bit must be valid before
   * enabling the USB clock in the RCC_AHBENR register.  This bit can't be
   * reset if the USB clock is enabled.
   */

  regval  = getreg32(STM32_RCC_CFGR);
  regval &= ~RCC_CFGR_OTGFSPRE;
  regval |= STM32_CFGR_OTGFSPRE;
  putreg32(regval, STM32_RCC_CFGR);
#endif

  /* Always enable FLITF clock and SRAM clock */

  regval = RCC_AHBENR_FLITFEN | RCC_AHBENR_SRAMEN;

#ifdef CONFIG_STM32_DMA1
  /* DMA 1 clock enable */

  regval |= RCC_AHBENR_DMA1EN;
#endif

#ifdef CONFIG_STM32_DMA2
  /* DMA 2 clock enable */

  regval |= RCC_AHBENR_DMA2EN;
#endif

#ifdef CONFIG_STM32_CRC
  /* CRC clock enable */

  regval |= RCC_AHBENR_CRCEN;
#endif

#ifdef CONFIG_STM32_FSMC
  /* FSMC clock enable */

  regval |=  RCC_AHBENR_FSMCEN;
#endif

#ifdef CONFIG_STM32_SDIO
  /* SDIO clock enable */

  regval |=  RCC_AHBENR_SDIOEN;
#endif

#ifdef CONFIG_STM32_CONNECTIVITYLINE
#ifdef CONFIG_STM32_OTGFS
  /* USB OTG FS clock enable */

  regval |= RCC_AHBENR_OTGFSEN;
#endif

#ifdef CONFIG_STM32_ETHMAC
  /* Ethernet clock enable */

  regval |= (RCC_AHBENR_ETHMACEN | RCC_AHBENR_ETHMACTXEN
             | RCC_AHBENR_ETHMACRXEN);
#endif
#endif

  putreg32(regval, STM32_RCC_AHBENR);   /* Enable peripherals */
}

/****************************************************************************
 * Name: rcc_enableapb1
 *
 * Description:
 *   Enable selected APB1 peripherals
 *
 ****************************************************************************/

static inline void rcc_enableapb1(void)
{
  uint32_t regval;

#ifdef CONFIG_STM32_USB
  /* USB clock divider for USB FS device.  This bit must be valid before
   * enabling the USB clock in the RCC_APB1ENR register. This bit can't be
   * reset if the USB clock is enabled.
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
#ifdef CONFIG_STM32_TIM2
  /* Timer 2 clock enable */

#ifdef CONFIG_STM32_FORCEPOWER
  regval |= RCC_APB1ENR_TIM2EN;
#endif
#endif

#ifdef CONFIG_STM32_TIM3
  /* Timer 3 clock enable */

#ifdef CONFIG_STM32_FORCEPOWER
  regval |= RCC_APB1ENR_TIM3EN;
#endif
#endif

#ifdef CONFIG_STM32_TIM4
  /* Timer 4 clock enable */

#ifdef CONFIG_STM32_FORCEPOWER
  regval |= RCC_APB1ENR_TIM4EN;
#endif
#endif

#ifdef CONFIG_STM32_TIM5
  /* Timer 5 clock enable */

#ifdef CONFIG_STM32_FORCEPOWER
  regval |= RCC_APB1ENR_TIM5EN;
#endif
#endif

#ifdef CONFIG_STM32_TIM6
  /* Timer 6 clock enable */

#ifdef CONFIG_STM32_FORCEPOWER
  regval |= RCC_APB1ENR_TIM6EN;
#endif
#endif

#ifdef CONFIG_STM32_TIM7
  /* Timer 7 clock enable */

#ifdef CONFIG_STM32_FORCEPOWER
  regval |= RCC_APB1ENR_TIM7EN;
#endif
#endif

#ifdef CONFIG_STM32_TIM12
  /* Timer 12 clock enable */

#ifdef CONFIG_STM32_FORCEPOWER
  regval |= RCC_APB1ENR_TIM12EN;
#endif
#endif

#ifdef CONFIG_STM32_TIM13
  /* Timer 13 clock enable */

#ifdef CONFIG_STM32_FORCEPOWER
  regval |= RCC_APB1ENR_TIM13EN;
#endif
#endif

#ifdef CONFIG_STM32_TIM14
  /* Timer 14 clock enable */

#ifdef CONFIG_STM32_FORCEPOWER
  regval |= RCC_APB1ENR_TIM14EN;
#endif
#endif

#ifdef CONFIG_STM32_WWDG
  /* Window Watchdog clock enable */

  regval |= RCC_APB1ENR_WWDGEN;
#endif

#ifdef CONFIG_STM32_SPI2
  /* SPI 2 clock enable */

  regval |= RCC_APB1ENR_SPI2EN;
#endif

#ifdef CONFIG_STM32_SPI3
  /* SPI 3 clock enable */

  regval |= RCC_APB1ENR_SPI3EN;
#endif

#ifdef CONFIG_STM32_USART2
  /* USART 2 clock enable */

  regval |= RCC_APB1ENR_USART2EN;
#endif

#ifdef CONFIG_STM32_USART3
  /* USART 3 clock enable */

  regval |= RCC_APB1ENR_USART3EN;
#endif

#ifdef CONFIG_STM32_UART4
  /* UART 4 clock enable */

  regval |= RCC_APB1ENR_UART4EN;
#endif

#ifdef CONFIG_STM32_UART5
  /* UART 5 clock enable */

  regval |= RCC_APB1ENR_UART5EN;
#endif

#ifdef CONFIG_STM32_I2C1
  /* I2C 1 clock enable */

#ifdef CONFIG_STM32_FORCEPOWER
  regval |= RCC_APB1ENR_I2C1EN;
#endif
#endif

#ifdef CONFIG_STM32_I2C2
  /* I2C 2 clock enable */

#ifdef CONFIG_STM32_FORCEPOWER
  regval |= RCC_APB1ENR_I2C2EN;
#endif
#endif

#ifdef CONFIG_STM32_USB
  /* USB clock enable */

  regval |= RCC_APB1ENR_USBEN;
#endif

#ifdef CONFIG_STM32_CAN1
  /* CAN1 clock enable */

  regval |= RCC_APB1ENR_CAN1EN;
#endif

#ifdef CONFIG_STM32_CAN2
  /* CAN2 clock enable.  NOTE: CAN2 needs CAN1 clock as well. */

  regval |= (RCC_APB1ENR_CAN1EN | RCC_APB1ENR_CAN2EN);
#endif

#ifdef CONFIG_STM32_BKP
  /* Backup interface clock enable */

  regval |= RCC_APB1ENR_BKPEN;
#endif

#ifdef CONFIG_STM32_PWR
  /* Power interface clock enable */

  regval |= RCC_APB1ENR_PWREN;
#endif

#if defined(CONFIG_STM32_DAC1)
  /* DAC1 interface clock enable */

  regval |= RCC_APB1ENR_DAC1EN;
#endif

#ifdef CONFIG_STM32_CEC
  /* CEC clock enable */

  regval |= RCC_APB1ENR_CECEN;
#endif

  putreg32(regval, STM32_RCC_APB1ENR);
}

/****************************************************************************
 * Name: rcc_enableapb2
 *
 * Description:
 *   Enable selected APB2 peripherals
 *
 ****************************************************************************/

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
             | RCC_APB2ENR_IOPAEN
#endif
#if STM32_NGPIO > 16
             | RCC_APB2ENR_IOPBEN
#endif
#if STM32_NGPIO > 32
             | RCC_APB2ENR_IOPCEN
#endif
#if STM32_NGPIO > 48
             | RCC_APB2ENR_IOPDEN
#endif
#if STM32_NGPIO > 64
             | RCC_APB2ENR_IOPEEN
#endif
#if STM32_NGPIO > 80
             | RCC_APB2ENR_IOPFEN
#endif
#if STM32_NGPIO > 96
             | RCC_APB2ENR_IOPGEN
#endif
             );

#ifdef CONFIG_STM32_ADC1
  /* ADC 1 interface clock enable */

  regval |= RCC_APB2ENR_ADC1EN;
#endif

#ifdef CONFIG_STM32_ADC2
  /* ADC 2 interface clock enable */

  regval |= RCC_APB2ENR_ADC2EN;
#endif

#ifdef CONFIG_STM32_TIM1
  /* TIM1 Timer clock enable */

#ifdef CONFIG_STM32_FORCEPOWER
  regval |= RCC_APB2ENR_TIM1EN;
#endif
#endif

#ifdef CONFIG_STM32_SPI1
  /* SPI 1 clock enable */

  regval |= RCC_APB2ENR_SPI1EN;
#endif

#ifdef CONFIG_STM32_TIM8
  /* TIM8 Timer clock enable */

#ifdef CONFIG_STM32_FORCEPOWER
  regval |= RCC_APB2ENR_TIM8EN;
#endif
#endif

#ifdef CONFIG_STM32_USART1
  /* USART1 clock enable */

  regval |= RCC_APB2ENR_USART1EN;
#endif

#ifdef CONFIG_STM32_ADC3
  /* ADC3 interface clock enable */

  regval |= RCC_APB2ENR_ADC3EN;
#endif

#ifdef CONFIG_STM32_TIM15
  /* TIM15 Timer clock enable */

#ifdef CONFIG_STM32_FORCEPOWER
  regval |= RCC_APB2ENR_TIM15EN;
#endif
#endif

#ifdef CONFIG_STM32_TIM16
  /* TIM16 Timer clock enable */

#ifdef CONFIG_STM32_FORCEPOWER
  regval |= RCC_APB2ENR_TIM16EN;
#endif
#endif

#ifdef CONFIG_STM32_TIM17
  /* TIM17 Timer clock enable */

#ifdef CONFIG_STM32_FORCEPOWER
  regval |= RCC_APB2ENR_TIM17EN;
#endif
#endif

  putreg32(regval, STM32_RCC_APB2ENR);
}

/****************************************************************************
 * Name: stm32_stdclockconfig
 *
 * Description:
 *   Called to change to new clock based on settings in board.h.  This
 *   version is for the Connectivity Line parts.
 *
 *   NOTE:  This logic would need to be extended if you need to select low-
 *   power clocking modes!
 ****************************************************************************/

#if !defined(CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG) && \
    defined(CONFIG_STM32_CONNECTIVITYLINE)
static void stm32_stdclockconfig(void)
{
  uint32_t regval;

  /* Enable HSE */

  regval  = getreg32(STM32_RCC_CR);
  regval &= ~RCC_CR_HSEBYP;         /* Disable HSE clock bypass */
  regval |= RCC_CR_HSEON;           /* Enable HSE */
  putreg32(regval, STM32_RCC_CR);

  /* Enable prefetch buffer and set FLASH wait states */

  regval  = getreg32(STM32_FLASH_ACR);
  regval &= ~FLASH_ACR_LATENCY_MASK;
  regval |= (FLASH_ACR_LATENCY_SETTING | FLASH_ACR_PRTFBE);
  putreg32(regval, STM32_FLASH_ACR);

  /* Set up PLL input scaling (with source = PLL2) */

  regval = getreg32(STM32_RCC_CFGR2);
  regval &= ~(RCC_CFGR2_PREDIV2_MASK    | RCC_CFGR2_PLL2MUL_MASK |
              RCC_CFGR2_PREDIV1SRC_MASK | RCC_CFGR2_PREDIV1_MASK);
  regval |=  (STM32_PLL_PREDIV2         | STM32_PLL_PLL2MUL |
              RCC_CFGR2_PREDIV1SRC_PLL2 | STM32_PLL_PREDIV1);
  putreg32(regval, STM32_RCC_CFGR2);

  /* Set the PCLK2 divider */

  regval = getreg32(STM32_RCC_CFGR);
  regval &= ~(RCC_CFGR_PPRE2_MASK | RCC_CFGR_HPRE_MASK);
  regval |= STM32_RCC_CFGR_PPRE2;
  regval |= RCC_CFGR_HPRE_SYSCLK;
  putreg32(regval, STM32_RCC_CFGR);

  /* Set the PCLK1 divider */

  regval = getreg32(STM32_RCC_CFGR);
  regval &= ~RCC_CFGR_PPRE1_MASK;
  regval |= STM32_RCC_CFGR_PPRE1;
  putreg32(regval, STM32_RCC_CFGR);

  /* Enable PLL2 */

  regval = getreg32(STM32_RCC_CR);
  regval |= RCC_CR_PLL2ON;
  putreg32(regval, STM32_RCC_CR);

  /* Wait for PLL2 ready */

  while ((getreg32(STM32_RCC_CR) & RCC_CR_PLL2RDY) == 0);

#if defined(CONFIG_STM32_MII_MCO) || defined(CONFIG_STM32_RMII_MCO)
  /* Setup PLL3 for MII/RMII clock on MCO */

  regval = getreg32(STM32_RCC_CFGR2);
  regval &= ~(RCC_CFGR2_PLL3MUL_MASK);
  regval |= STM32_PLL_PLL3MUL;
  putreg32(regval, STM32_RCC_CFGR2);

  /* Switch PLL3 on */

  regval = getreg32(STM32_RCC_CR);
  regval |= RCC_CR_PLL3ON;
  putreg32(regval, STM32_RCC_CR);

  while ((getreg32(STM32_RCC_CR) & RCC_CR_PLL3RDY) == 0);
#endif

  /* Set main PLL source and multiplier */

  regval = getreg32(STM32_RCC_CFGR);
  regval &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLMUL_MASK);
  regval |= (RCC_CFGR_PLLSRC | STM32_PLL_PLLMUL);
  putreg32(regval, STM32_RCC_CFGR);

  /* Switch main PLL on */

  regval = getreg32(STM32_RCC_CR);
  regval |= RCC_CR_PLLON;
  putreg32(regval, STM32_RCC_CR);

  while ((getreg32(STM32_RCC_CR) & RCC_CR_PLLRDY) == 0);

  /* Select PLL as system clock source */

  regval  = getreg32(STM32_RCC_CFGR);
  regval &= ~RCC_CFGR_SW_MASK;
  regval |= RCC_CFGR_SW_PLL;
  putreg32(regval, STM32_RCC_CFGR);

  /* Wait until PLL is used as the system clock source */

  while ((getreg32(STM32_RCC_CFGR) & RCC_CFGR_SWS_PLL) == 0);
}
#endif

/****************************************************************************
 * Name: stm32_stdclockconfig
 *
 * Description:
 *   Called to change to new clock based on settings in board.h.  This
 *   version is for the non-Connectivity Line parts.
 *
 *   NOTE:  This logic would need to be extended if you need to select low-
 *   power clocking modes!
 ****************************************************************************/

#if !defined(CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG) && \
    !defined(CONFIG_STM32_CONNECTIVITYLINE)
static void stm32_stdclockconfig(void)
{
  uint32_t regval;

  /* If the PLL is using the HSE, or the HSE is the system clock */

#if (STM32_CFGR_PLLSRC == RCC_CFGR_PLLSRC) || (STM32_SYSCLK_SW == RCC_CFGR_SW_HSE)
    {
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

      if (timeout == 0)
        {
          /* In the case of a timeout starting the HSE, we really don't have
           * a strategy.  This is almost always a hardware failure or
           * misconfiguration.
           */

          return;
        }
    }

  /* If this is a value-line part and we are using the HSE as the PLL */

#  if defined(CONFIG_STM32_VALUELINE) && (STM32_CFGR_PLLSRC == RCC_CFGR_PLLSRC)

#    if (STM32_CFGR_PLLXTPRE >> 17) != (STM32_CFGR2_PREDIV1 & 1)
#      error STM32_CFGR_PLLXTPRE must match the LSB of STM32_CFGR2_PREDIV1
#    endif

  /* Set the HSE prescaler */

  regval = STM32_CFGR2_PREDIV1;
  putreg32(regval, STM32_RCC_CFGR2);

#  endif
#endif

  /* Value-line devices don't implement flash prefetch/waitstates */

#ifndef CONFIG_STM32_VALUELINE

  /* Enable FLASH prefetch buffer and set FLASH wait states */

  regval  = getreg32(STM32_FLASH_ACR);
  regval &= ~FLASH_ACR_LATENCY_MASK;
  regval |= (FLASH_ACR_LATENCY_SETTING | FLASH_ACR_PRTFBE);
  putreg32(regval, STM32_FLASH_ACR);

#endif

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

  /* If we are using the PLL, configure and start it */

#if STM32_SYSCLK_SW == RCC_CFGR_SW_PLL

  /* Set the PLL divider and multiplier */

  regval = getreg32(STM32_RCC_CFGR);
  regval &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMUL_MASK);
  regval |= (STM32_CFGR_PLLSRC | STM32_CFGR_PLLXTPRE | STM32_CFGR_PLLMUL);
  putreg32(regval, STM32_RCC_CFGR);

  /* Enable the PLL */

  regval = getreg32(STM32_RCC_CR);
  regval |= RCC_CR_PLLON;
  putreg32(regval, STM32_RCC_CR);

  /* Wait until the PLL is ready */

  while ((getreg32(STM32_RCC_CR) & RCC_CR_PLLRDY) == 0);

#endif

  /* Select the system clock source (probably the PLL) */

  regval  = getreg32(STM32_RCC_CFGR);
  regval &= ~RCC_CFGR_SW_MASK;
  regval |= STM32_SYSCLK_SW;
  putreg32(regval, STM32_RCC_CFGR);

  /* Wait until the selected source is used as the system clock source */

  while ((getreg32(STM32_RCC_CFGR) & RCC_CFGR_SWS_MASK) != STM32_SYSCLK_SWS);

#if defined(CONFIG_STM32_IWDG) || defined(CONFIG_STM32_RTC_LSICLOCK)
  /* Low speed internal clock source LSI */

  stm32_rcc_enablelsi();
#endif
}
#endif

/****************************************************************************
 * Name: rcc_enableperiphals
 ****************************************************************************/

static inline void rcc_enableperipherals(void)
{
  rcc_enableahb();
  rcc_enableapb2();
  rcc_enableapb1();

#if defined(CONFIG_STM32_RTC_LSECLOCK)
  /* Low speed external clock source LSE
   * For F1 it requires PWR and BKP from APB1
   */

  stm32_rcc_enablelse();
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
