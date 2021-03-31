/****************************************************************************
 * arch/arm/src/stm32/stm32f33xxx_rcc.c
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
#elif (STM32_SYSCLK_FREQUENCY <= 72000000)
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

  regval  = getreg32(STM32_RCC_CFGR);       /* Reset SW, HPRE, PPRE1, PPRE2, and MCO bits */
  regval &= ~(RCC_CFGR_SW_MASK | RCC_CFGR_HPRE_MASK | RCC_CFGR_PPRE1_MASK |
              RCC_CFGR_PPRE2_MASK | RCC_CFGR_MCO_MASK);

  putreg32(regval, STM32_RCC_CFGR);

  regval  = getreg32(STM32_RCC_CFGR2);       /* Reset PREDIV, and ADC12PRE bits */
  regval &= ~(RCC_CFGR2_PREDIV_MASK | RCC_CFGR2_ADC12PRES_MASK);
  putreg32(regval, STM32_RCC_CFGR2);

  regval  = getreg32(STM32_RCC_CFGR3);       /* Reset all U[S]ARTs, I2C1, TIM1 and HRTIM1 bits */
  regval &= ~(RCC_CFGR3_USART1SW_MASK | RCC_CFGR3_USART2SW_MASK | \
              RCC_CFGR3_USART3SW_MASK | RCC_CFGR3_I2C1SW | \
              RCC_CFGR3_TIM1SW | RCC_CFGR3_HRTIM1SW);
  putreg32(regval, STM32_RCC_CFGR3);

  regval  = getreg32(STM32_RCC_CR);         /* Reset HSEON, CSSON and PLLON bits */
  regval &= ~(RCC_CR_HSEON | RCC_CR_CSSON | RCC_CR_PLLON);
  putreg32(regval, STM32_RCC_CR);

  regval  = getreg32(STM32_RCC_CR);         /* Reset HSEBYP bit */
  regval &= ~RCC_CR_HSEBYP;
  putreg32(regval, STM32_RCC_CR);

  regval  = getreg32(STM32_RCC_CFGR);       /* Reset PLLSRC, PLLXTPRE, PLLMUL and USBPRE bits */
  regval &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMUL_MASK);
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

  /* Always enable FLITF clock and SRAM clock */

  regval = RCC_AHBENR_FLITFEN | RCC_AHBENR_SRAMEN;

  /* Enable GPIO PORTA, PORTB, ... PORTF */

  regval |= (RCC_AHBENR_IOPAEN | RCC_AHBENR_IOPBEN | RCC_AHBENR_IOPCEN |
             RCC_AHBENR_IOPDEN | RCC_AHBENR_IOPFEN);

#ifdef CONFIG_STM32_DMA1
  /* DMA 1 clock enable */

  regval |= RCC_AHBENR_DMA1EN;
#endif

#ifdef CONFIG_STM32_CRC
  /* CRC clock enable */

  regval |= RCC_AHBENR_CRCEN;
#endif

#ifdef CONFIG_STM32_TSC
  /* TSC clock enable */

  regval |= RCC_AHBENR_TSCEN;
#endif

#if defined(CONFIG_STM32_ADC1) || defined(CONFIG_STM32_ADC2)
  /* ADC1/ADC2 interface clock enable */

  regval |= RCC_AHBENR_ADC12EN;
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

#ifdef CONFIG_STM32_WWDG
  /* Window Watchdog clock enable */

  regval |= RCC_APB1ENR_WWDGEN;
#endif

#ifdef CONFIG_STM32_USART2
  /* USART 2 clock enable */

  regval |= RCC_APB1ENR_USART2EN;
#endif

#ifdef CONFIG_STM32_USART3
  /* USART 3 clock enable */

  regval |= RCC_APB1ENR_USART3EN;
#endif

#ifdef CONFIG_STM32_I2C1
  /* I2C 1 clock enable */

#ifdef CONFIG_STM32_FORCEPOWER
  regval |= RCC_APB1ENR_I2C1EN;
#endif
#endif

#ifdef CONFIG_STM32_CAN1
  /* CAN1 clock enable */

  regval |= RCC_APB1ENR_CANEN;
#endif

#ifdef CONFIG_STM32_DAC2
  /* DAC2 interface clock enable */

  regval |= RCC_APB1ENR_DAC2EN;
#endif

#ifdef CONFIG_STM32_PWR
  /* Power interface clock enable */

  regval |= RCC_APB1ENR_PWREN;
#endif

#ifdef CONFIG_STM32_DAC1
  /* DAC1 interface clock enable */

  regval |= RCC_APB1ENR_DAC1EN;
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

  regval = getreg32(STM32_RCC_APB2ENR);

#ifdef CONFIG_STM32_SYSCFG
  /* SYSCFG clock */

  regval |= RCC_APB2ENR_SYSCFGEN;
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

#ifdef CONFIG_STM32_USART1
  /* USART1 clock enable */

  regval |= RCC_APB2ENR_USART1EN;
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

#ifdef CONFIG_STM32_HRTIM1
  /* HRTIM1 Timer clock enable */

  regval |= RCC_APB2ENR_HRTIM1EN;
#endif

  putreg32(regval, STM32_RCC_APB2ENR);
}

/****************************************************************************
 * Name: stm32_stdclockconfig
 *
 * Description:
 *   Called to change to new clock based on settings in board.h.
 *
 *   NOTE:  This logic would need to be extended if you need to select low-
 *   power clocking modes!
 ****************************************************************************/

#if !defined(CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG)
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

#if STM32_SYSCLK_SW == RCC_CFGR_SW_PLL
  /* If we are using the PLL, configure and start it */

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

  /* Enable FLASH prefetch and wait states */

  regval = getreg32(STM32_FLASH_ACR);
  regval &= ~(FLASH_ACR_LATENCY_MASK);
  regval |= (FLASH_ACR_LATENCY_SETTING | FLASH_ACR_PRTFBE);
  putreg32(regval, STM32_FLASH_ACR);

  /* Select the system clock source (probably the PLL) */

  regval  = getreg32(STM32_RCC_CFGR);
  regval &= ~RCC_CFGR_SW_MASK;
  regval |= STM32_SYSCLK_SW;
  putreg32(regval, STM32_RCC_CFGR);

  /* Wait until the selected source is used as the system clock source */

  while ((getreg32(STM32_RCC_CFGR) & RCC_CFGR_SWS_MASK) != STM32_SYSCLK_SWS);

#if defined(CONFIG_STM32_IWDG) || defined(CONFIG_STM32_RTC_LSICLOCK)
  /* Low speed internal clock source LSI
   *
   * TODO: There is another case where the LSI needs to
   * be enabled: if the MCO pin selects LSI as source.
   */

  stm32_rcc_enablelsi();
#endif

#if defined(CONFIG_STM32_RTC_LSECLOCK)
  /* Low speed external clock source LSE
   *
   * TODO: There is another case where the LSE needs to
   * be enabled: if the MCO pin selects LSE as source.
   *
   * TODO: There is another case where the LSE needs to
   * be enabled: if USARTx selects LSE as source.
   */

  stm32_rcc_enablelse();
#endif

#ifdef CONFIG_STM32_HRTIM_CLK_FROM_PLL
  regval  = getreg32(STM32_RCC_CFGR3);
  regval |= RCC_CFGR3_HRTIM1SW;
  putreg32(regval, STM32_RCC_CFGR3);
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
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
