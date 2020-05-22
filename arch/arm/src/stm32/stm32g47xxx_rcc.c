/****************************************************************************
 *  arch/arm/src/stm32/stm32g47xxx_rcc.c
 *
 *  Licensed to the Apache Software Foundation (ASF) under one or more
 *  contributor license agreements.  See the NOTICE file distributed with
 *  this work for additional information regarding copyright ownership.  The
 *  ASF licenses this file to you under the Apache License, Version 2.0 (the
 *  "License"); you may not use this file except in compliance with the
 *  License.  You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 *  License for the specific language governing permissions and limitations
 *  under the License.
 *
 ****************************************************************************/

/* Unless otherwise specified, when comments in this file refer to the
 * reference manual, that is the STM32G474 Reference Manual (RM0440 Rev 2).
 *
 * This file requires a clocking configuration, which is set in board.h,
 * consisting of some or all of the following defines:
 *
 * STM32_HSI_FREQUENCY should be defined to the frequency of the MCU's
 * high speed internal (HSI) oscillator in Hz.
 *
 * STM32_LSI_FREQUENCY should be defined to the frequency of the MCU's low
 * speed internal (LSI) oscillator in Hz.
 *
 * If the board has an external crystal or oscillator, STM32_BOARD_XTAL
 * should be defined its frequency in Hz.
 *
 * If the board has a high speed external (HSE) crystal/oscillator,
 * STM32_HSE_FREQUENCY should be defined to its frequency in Hz.
 *
 * If the board has a low speed external (LSE) crystal/oscillator,
 * STM32_LSE_FREQUENCY should be defined to its frequency in Hz.
 *
 * If the PLL is used, its source must be set by defining
 * STM32_PLLCFGR_PLLSRC to one of the following: RCC_PLLCFGR_PLLSRC_HSI or
 * RCC_PLLCFGR_PLLSRC_HSE, and each of its output clock(s) should be
 * enabled, as needed, by setting STM32_PLLCFGR_PLLCFG to the bitwise OR
 * of RCC_PLLCFGR_PLLPEN, RCC_PLLCFGR_PLLQEN, and/or RCC_PLLCFGR_PLLREN.
 * Its prescale division and VCO multiplication factors must be set by
 * defining STM32_PLLCFGR_PLLM and STM32_PLLCFGR_PLLN, using the
 * RCC_PLLCFGR_PLLM() and RCC_PLLCFGR_PLLN() macros, respectively. The
 * division factors for each enabled PLL output must be set by defining
 * STM32_PLLCFGR_PLLP, STM32_PLLCFGR_PLLQ, and STM32_PLLCFGR_PLLR, using
 * the RCC_PLLCFGR_PLLP(), RCC_PLLCFGR_PLLQ(), and RCC_PLLCFGR_PLLR()
 * macros, respectively. The resulting frequencies must be specified by
 * defining STM32_VCO_FREQUENCY, STM32_PLLP_FREQUENCY,
 * STM32_PLLQ_FREQUENCY, and STM32_PLLR_FREQUENCY to those frequency in
 * Hz, which can be calculated in terms of above-defined frequencies.
 *
 * The SYSCLK source must be given by defining STM32_SYSCLK_SW to one of
 * RCC_CFGR_SW_HSI, RCC_CFGR_SW_HSE, or RCC_CFGR_SW_PLL, defining
 * STM32_SYSCLK_SWS to one of RCC_CFGR_SWS_HSI, RCC_CFGR_SWS_HSE, or
 * RCC_CFGR_SWS_PLL, and defining STM32_SYSCLK_FREQUENCY to the resulting
 * SYSCLK frequency in Hz. For example, if SYSCLK is driven by the PLL "R"
 * clock, STM32_SYSCLK_FREQUENCY can be defined to STM32_PLLR_FREQUENCY,
 * which was defined earlier.
 *
 * The AHB clock (HCLK) must be setup by defining STM32_RCC_CFGR_HPRE to
 * one of RCC_CFGR_HPRE_SYSCLK, RCC_CFGR_HPRE_SYSCLKd2,
 * RCC_CFGR_HPRE_SYSCLKd4, RCC_CFGR_HPRE_SYSCLKd8,
 * RCC_CFGR_HPRE_SYSCLKd16, RCC_CFGR_HPRE_SYSCLKd64,
 * RCC_CFGR_HPRE_SYSCLKd128, RCC_CFGR_HPRE_SYSCLKd256, or
 * RCC_CFGR_HPRE_SYSCLKd512. Also, STM32_HCLK_FREQUENCY must be defined to
 * its resulting frequency in Hz. For example, if HCLK is driven by
 * SYSCLK, STM32_HCLK_FREQUENCY can be defined to STM32_SYSCLK_FREQUENCY.
 *
 * The APB1 clock (PCLK1) must be setup by defining STM32_RCC_CFGR_PPRE1
 * to one of RCC_CFGR_PPRE1_HCLK, RCC_CFGR_PPRE1_HCLKd2,
 * RCC_CFGR_PPRE1_HCLKd4, RCC_CFGR_PPRE1_HCLKd8, RCC_CFGR_PPRE1_HCLKd16
 * and defining STM32_PCLK1_FREQUENCY to its frequency in Hz.
 *
 * The APB2 clock (PCLK2) must be setup by defining STM32_RCC_CFGR_PPRE2
 * to one of RCC_CFGR_PPRE2_HCLK, RCC_CFGR_PPRE2_HCLKd2,
 * RCC_CFGR_PPRE2_HCLKd4, RCC_CFGR_PPRE2_HCLKd8, RCC_CFGR_PPRE2_HCLKd16
 * and defining STM32_PCLK2_FREQUENCY to its frequency in Hz.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/stm32g47xxx_pwr.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if (STM32_SYSCLK_SW == RCC_CFGR_SW_HSE)
#  define USE_HSE
#endif

#if (STM32_SYSCLK_SW == RCC_CFGR_SW_HSI)
#  define USE_HSI
#endif

#if (STM32_SYSCLK_SW == RCC_CFGR_SW_PLL)
#  define USE_PLL
#  define PLLRDY_TIMEOUT               (100 * CONFIG_BOARD_LOOPSPERMSEC)
#  if (STM32_PLLCFGR_PLLSRC == RCC_PLLCFGR_PLLSRC_HSE)
#    define USE_HSE
#  elif (STM32_PLLCFGR_PLLSRC == RCC_PLLCFGR_PLLSRC_HSI)
#    define USE_HSI
#  else
#    error "STM32_SYSCLK_SW is RCC_CFGR_SW_PLL but STM32_PLLCFGR_PLLSRC is not recognized!"
#  endif
#endif

#if defined(USE_HSI)
#  define HSIRDY_TIMEOUT               (100 * CONFIG_BOARD_LOOPSPERMSEC)
#endif

#if defined(USE_HSE)
#  define HSERDY_TIMEOUT               (100 * CONFIG_BOARD_LOOPSPERMSEC)
#endif

/* Per the reference manual:
 *
 * Choose PWR VOS range setting and R1MODE based on SYSCLK frequency (see
 * section 5.1.5, Dynamic voltage scaling management).
 *
 * Choose number of FLASH wait states according to CPU clock (HCLK)
 * frequency (see section 3.3.3, Read access latency, and Table 9 in that
 * section.
 *
 * This will define FLASH_ACR_LATENCY_SETTING, PWR_CR1_VOS_RANGE_SETTING,
 * and PWR_CR5_R1MODE_SETTING to the appropriate settings.
 */

#if (STM32_SYSCLK_FREQUENCY > 26000000)
#  define PWR_CR1_VOS_RANGE_SETTING    PWR_CR1_VOS_RANGE_1
#  if (STM32_SYSCLK_FREQUENCY > 150000000)
#    define PWR_CR5_R1MODE_SETTING     0
#  else
#    define PWR_CR5_R1MODE_SETTING     PWR_CR5_R1MODE
#  endif
#  if (STM32_HCLK_FREQUENCY <= 20000000)
#    define FLASH_ACR_LATENCY_SETTING  FLASH_ACR_LATENCY_0
#  elif (STM32_HCLK_FREQUENCY <= 40000000)
#    define FLASH_ACR_LATENCY_SETTING  FLASH_ACR_LATENCY_1
#  elif (STM32_HCLK_FREQUENCY <= 60000000)
#    define FLASH_ACR_LATENCY_SETTING  FLASH_ACR_LATENCY_2
#  elif (STM32_HCLK_FREQUENCY <= 80000000)
#    define FLASH_ACR_LATENCY_SETTING  FLASH_ACR_LATENCY_3
#  elif (STM32_HCLK_FREQUENCY <= 100000000)
#    define FLASH_ACR_LATENCY_SETTING  FLASH_ACR_LATENCY_4
#  elif (STM32_HCLK_FREQUENCY <= 120000000)
#    define FLASH_ACR_LATENCY_SETTING  FLASH_ACR_LATENCY_5
#  elif (STM32_HCLK_FREQUENCY <= 140000000)
#    define FLASH_ACR_LATENCY_SETTING  FLASH_ACR_LATENCY_6
#  elif (STM32_HCLK_FREQUENCY <= 160000000)
#    define FLASH_ACR_LATENCY_SETTING  FLASH_ACR_LATENCY_7
#  elif (STM32_HCLK_FREQUENCY <= 170000000)
#    define FLASH_ACR_LATENCY_SETTING  FLASH_ACR_LATENCY_8
#  else
#    error "Incorrect STM32_HCLK_FREQUENCY (VOS range 1)!"
#  endif
#else
#  define PWR_CR1_VOS_RANGE_SETTING    PWR_CR1_VOS_RANGE_2;
#  define PWR_CR5_R1MODE_SETTING       0
#  if (STM32_HCLK_FREQUENCY <= 8000000)
#    define FLASH_ACR_LATENCY_SETTING  FLASH_ACR_LATENCY_0
#  elif (STM32_HCLK_FREQUENCY <= 16000000)
#    define FLASH_ACR_LATENCY_SETTING  FLASH_ACR_LATENCY_1
#  elif (STM32_HCLK_FREQUENCY <= 26000000)
#    define FLASH_ACR_LATENCY_SETTING  FLASH_ACR_LATENCY_2
#  else
#    error "Incorrect STM32_HCLK_FREQUENCY! (VOS range 2)"
#  endif
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rcc_reset
 *
 * Description:
 *   Put all RCC registers in reset state.
 *
 ****************************************************************************/

static inline void rcc_reset(void)
{
  uint32_t regval;

  /* Set HSION bit to the reset value and wait until HSI ready */

  regval = getreg32(STM32_RCC_CR);
  regval |= RCC_CR_HSION;
  putreg32(regval, STM32_RCC_CR);

  while ((getreg32(STM32_RCC_CR) & RCC_CR_HSIRDY) != RCC_CR_HSIRDY)
    {
    }

  /* Reset HSI trimming */

  regval = getreg32(STM32_RCC_ICSCR);
  regval &= ~(RCC_ICSCR_HSITRIM_MASK);
  regval |= RCC_ICSCR_HSITRIM_RESET;
  putreg32(regval, STM32_RCC_ICSCR);

  /* Reset CFGR register */

  regval = getreg32(STM32_RCC_CFGR);
  regval &= RCC_CFGR_RESERVED_MASK;
  regval |= RCC_CFGR_RESET;
  putreg32(regval, STM32_RCC_CFGR);

  /* Wait until HSI is being used as the system clock, else we cannot clear
   * the HSEON, HSEBYP, and PLLON bits.
   */

  while ((getreg32(STM32_RCC_CFGR) & RCC_CFGR_SWS_MASK) != RCC_CFGR_SWS_HSI)
    {
    }

  /* Clear the HSEON, HSEBYP, and PLLON bits */

  regval = getreg32(STM32_RCC_CR);
  regval &= ~(RCC_CR_HSEON | RCC_CR_HSEBYP | RCC_CR_PLLON);
  putreg32(regval, STM32_RCC_CR);

  /* Wait until PLL is OFF, else we cannot change some of the bits in the
   * PLLCFGR register.
   */

  while ((getreg32(STM32_RCC_CR) & RCC_CR_PLLRDY) != 0)
    {
    }

  /* Reset PLLCFGR register */

  regval = getreg32(STM32_RCC_PLLCFGR);
  regval &= RCC_PLLCFGR_RESERVED_MASK;
  regval |= RCC_PLLCFGR_RESET;
  putreg32(RCC_PLLCFGR_RESET, STM32_RCC_PLLCFGR);

  /* Disable all RCC interrupts and clear any previously pended ones */

  putreg32(0, STM32_RCC_CIER);
  putreg32(0xffffffff, STM32_RCC_CICR);
}

/****************************************************************************
 * Name: rcc_enableahb1
 *
 * Description:
 *   Enable selected AHB1 peripherals.
 *
 ****************************************************************************/

static inline void rcc_enableahb1(void)
{
  uint32_t regval;

  regval = getreg32(STM32_RCC_AHB1ENR);
  regval |= RCC_AHB1ENR_FLASHEN;

#if defined(CONFIG_STM32_DMA1)
  regval |= RCC_AHB1ENR_DMA1EN;
#endif

#if defined(CONFIG_STM32_DMA2)
  regval |= RCC_AHB1ENR_DMA2EN;
#endif

#if defined(CONFIG_STM32_DMA1) || defined(CONFIG_STM32_DMA2)
  regval |= RCC_AHB1ENR_DMAMUX1EN;
#endif

#if defined(CONFIG_STM32_CORDIC)
  regval |= RCC_AHB1ENR_CORDICEN;
#endif

#if defined(CONFIG_STM32_FMAC)
  regval |= RCC_AHB1ENR_FMACEN;
#endif

#if defined(CONFIG_STM32_CRC)
  regval |= RCC_AHB1ENR_CRCEN;
#endif

  putreg32(regval, STM32_RCC_AHB1ENR);
}

/****************************************************************************
 * Name: rcc_enableahb2
 *
 * Description:
 *   Enable selected AHB2 peripherals.
 *
 ****************************************************************************/

static inline void rcc_enableahb2(void)
{
  uint32_t regval;

  regval = getreg32(STM32_RCC_AHB2ENR);

#if (STM32_NGPIO_PORTS > 0)
  regval |= (RCC_AHB2ENR_GPIOAEN
#  if (STM32_NGPIO_PORTS > 1)
             | RCC_AHB2ENR_GPIOBEN
#  endif
#  if (STM32_NGPIO_PORTS > 2)
             | RCC_AHB2ENR_GPIOCEN
#  endif
#  if (STM32_NGPIO_PORTS > 3)
             | RCC_AHB2ENR_GPIODEN
#  endif
#  if (STM32_NGPIO_PORTS > 4)
             | RCC_AHB2ENR_GPIOEEN
#  endif
#  if (STM32_NGPIO_PORTS > 5)
             | RCC_AHB2ENR_GPIOFEN
#  endif
#  if (STM32_NGPIO_PORTS > 6)
             | RCC_AHB2ENR_GPIOGEN
#  endif
            );
#endif

#if defined(CONFIG_STM32_ADC1) || defined(CONFIG_STM32_ADC2)
  regval |= RCC_AHB2ENR_ADC12EN;
#endif

#if defined(CONFIG_STM32_ADC3) || defined(CONFIG_STM32_ADC4) || \
    defined(CONFIG_STM32_ADC5)
  regval |= RCC_AHB2ENR_ADC345EN;
#endif

#if defined(CONFIG_STM32_DAC1)
  regval |= RCC_AHB2ENR_DAC1EN;
#endif

#if defined(CONFIG_STM32_DAC2)
  regval |= RCC_AHB2ENR_DAC2EN;
#endif

#if defined(CONFIG_STM32_DAC3)
  regval |= RCC_AHB2ENR_DAC3EN;
#endif

#if defined(CONFIG_STM32_DAC4)
  regval |= RCC_AHB2ENR_DAC4EN;
#endif

#if defined(CONFIG_STM32_RNG)
  regval |= RCC_AHB2ENR_RNGEN;
#endif

  putreg32(regval, STM32_RCC_AHB2ENR);
}

/****************************************************************************
 * Name: rcc_enableahb3
 *
 * Description:
 *   Enable selected AHB3 peripherals.
 *
 ****************************************************************************/

static inline void rcc_enableahb3(void)
{
  uint32_t regval;

  regval = getreg32(STM32_RCC_AHB3ENR);

#if defined(CONFIG_STM32_FMC)
  regval |= RCC_AHB3ENR_FMCEN;
#endif

#if defined(CONFIG_STM32_QSPI)
  regval |= RCC_AHB3ENR_QSPIEN;
#endif

  putreg32(regval, STM32_RCC_AHB3ENR);
}

/****************************************************************************
 * Name: rcc_enableapb1
 *
 * Description:
 *   Enable selected APB1 peripherals.
 *
 ****************************************************************************/

static inline void rcc_enableapb1(void)
{
  uint32_t regval;

  /* Careful: There are two separate registers for enabling APB1
   * peripherals. Configure the first register:
   */

  regval = getreg32(STM32_RCC_APB1ENR1);
  regval |= RCC_APB1ENR1_PWREN;

#if defined(CONFIG_STM32_TIM2)
  regval |= RCC_APB1ENR1_TIM2EN;
#endif

#if defined(CONFIG_STM32_TIM3)
  regval |= RCC_APB1ENR1_TIM3EN;
#endif

#if defined(CONFIG_STM32_TIM4)
  regval |= RCC_APB1ENR1_TIM4EN;
#endif

#if defined(CONFIG_STM32_TIM5)
  regval |= RCC_APB1ENR1_TIM5EN;
#endif

#if defined(CONFIG_STM32_TIM6)
  regval |= RCC_APB1ENR1_TIM6EN;
#endif

#if defined(CONFIG_STM32_TIM7)
  regval |= RCC_APB1ENR1_TIM7EN;
#endif

#if defined(CONFIG_STM32_CRS)
  regval |= RCC_APB1ENR1_CRSEN;
#endif

#if defined(CONFIG_STM32_RTC)
  regval |= RCC_APB1ENR1_RTCAPBEN;
#endif

#if defined(CONFIG_STM32_WWDG)
  regval |= RCC_APB1ENR1_WWDGEN;
#endif

#if defined(CONFIG_STM32_SPI2)
  regval |= RCC_APB1ENR1_SPI2EN;
#endif

#if defined(CONFIG_STM32_SPI3)
  regval |= RCC_APB1ENR1_SPI3EN;
#endif

#if defined(CONFIG_STM32_USART2)
  regval |= RCC_APB1ENR1_USART2EN;
#endif

#if defined(CONFIG_STM32_USART3)
  regval |= RCC_APB1ENR1_USART3EN;
#endif

#if defined(CONFIG_STM32_UART4)
  regval |= RCC_APB1ENR1_UART4EN;
#endif

#if defined(CONFIG_STM32_UART5)
  regval |= RCC_APB1ENR1_UART5EN;
#endif

#if defined(CONFIG_STM32_I2C1)
  regval |= RCC_APB1ENR1_I2C1EN;
#endif

#if defined(CONFIG_STM32_I2C2)
  regval |= RCC_APB1ENR1_I2C2EN;
#endif

#if defined(CONFIG_STM32_USB)
  regval |= RCC_APB1ENR1_USBEN;
#endif

#if defined(CONFIG_STM32_FDCAN)
  regval |= RCC_APB1ENR1_FDCANEN;
#endif

#if defined(CONFIG_STM32_I2C3)
  regval |= RCC_APB1ENR1_I2C3EN;
#endif

#if defined(CONFIG_STM32_LPTIM1)
  regval |= RCC_APB1ENR1_LPTIM1EN;
#endif

  putreg32(regval, STM32_RCC_APB1ENR1);

  /* Now configure the second register: */

  regval = getreg32(STM32_RCC_APB1ENR2);

#if defined(CONFIG_STM32_LPUART1)
  regval |= RCC_APB1ENR2_LPUART1EN;
#endif

#if defined(CONFIG_STM32_I2C4)
  regval |= RCC_APB1ENR2_I2C4EN;
#endif

#if defined(CONFIG_STM32_UCPD)
  regval |= RCC_APB1ENR2_UCPD1EN;
#endif

  putreg32(regval, STM32_RCC_APB1ENR2);
}

/****************************************************************************
 * Name: rcc_enableapb2
 *
 * Description:
 *   Enable selected APB2 peripherals.
 *
 ****************************************************************************/

static inline void rcc_enableapb2(void)
{
  uint32_t regval;

  regval = getreg32(STM32_RCC_APB2ENR);

#if defined(CONFIG_STM32_SYSCFG)
  regval |= RCC_APB2ENR_SYSCFGEN;
#endif

#if defined(CONFIG_STM32_TIM1)
  regval |= RCC_APB2ENR_TIM1EN;
#endif

#if defined(CONFIG_STM32_SPI1)
  regval |= RCC_APB2ENR_SPI1EN;
#endif

#if defined(CONFIG_STM32_TIM8)
  regval |= RCC_APB2ENR_TIM8EN;
#endif

#if defined(CONFIG_STM32_USART1)
  regval |= RCC_APB2ENR_USART1EN;
#endif

#if defined(CONFIG_STM32_SPI4)
  regval |= RCC_APB2ENR_SPI4EN;
#endif

#if defined(CONFIG_STM32_TIM15)
  regval |= RCC_APB2ENR_TIM15EN;
#endif

#if defined(CONFIG_STM32_TIM16)
  regval |= RCC_APB2ENR_TIM16EN;
#endif

#if defined(CONFIG_STM32_TIM17)
  regval |= RCC_APB2ENR_TIM17EN;
#endif

#if defined(CONFIG_STM32_TIM20)
  regval |= RCC_APB2ENR_TIM20EN;
#endif

#if defined(CONFIG_STM32_SAI1)
  regval |= RCC_APB2ENR_SAI1EN;
#endif

#if defined(CONFIG_STM32_HRTIM1)
  regval |= RCC_APB2ENR_HRTIM1EN;
#endif

  putreg32(regval, STM32_RCC_APB2ENR);
}

/****************************************************************************
 * Name: stm32_rcc_enablehse
 *
 * Description:
 *   Enable the High-Speed External (HSE) Oscillator.
 *
 ****************************************************************************/

#if defined (USE_HSE)
static inline bool stm32_rcc_enablehse(void)
{
  uint32_t regval;
  uint32_t timeout;

  /* Enable External High-Speed Clock (HSE) */

  regval  = getreg32(STM32_RCC_CR);
#if defined(STM32_HSEBYP_ENABLE)    /* May be defined in board.h header file */
  regval |= RCC_CR_HSEBYP;          /* Enable HSE clock bypass */
#else
  regval &= ~RCC_CR_HSEBYP;         /* Disable HSE clock bypass */
#endif
  regval |= RCC_CR_HSEON;           /* Enable HSE */
  putreg32(regval, STM32_RCC_CR);

  /* Wait until the HSE is ready (or until a timeout elapsed) */

  for (timeout = HSERDY_TIMEOUT; timeout > 0; timeout--)
    {
      if ((getreg32(STM32_RCC_CR) & RCC_CR_HSERDY) != 0)
        {
          /* HSE has been enabled successfully and is ready */

          return true;
        }
    }

  /* HSE was not enabled successfully or timed out; this could
   * mean that the external crystal or oscillator is missing or
   * not working.
   */

  return false;
}
#endif /* USE_HSE */

/****************************************************************************
 * Name: stm32_rcc_enablehsi
 *
 * Description:
 *   Enable the High-Speed Internal (HSI) Oscillator.
 *
 ****************************************************************************/

#if defined (USE_HSI)
static inline bool stm32_rcc_enablehsi(void)
{
  uint32_t regval;
  uint32_t timeout;

  /* Enable Internal High-Speed Clock (HSI) */

  regval  = getreg32(STM32_RCC_CR);
  regval |= RCC_CR_HSION;
  putreg32(regval, STM32_RCC_CR);

  /* Wait until the HSI is ready (or until a timeout elapsed) */

  for (timeout = HSIRDY_TIMEOUT; timeout > 0; timeout--)
    {
      if ((getreg32(STM32_RCC_CR) & RCC_CR_HSIRDY) != 0)
        {
          /* HSI has been enabled successfully and is ready */

          return true;
        }
    }

  /* HSI was not enabled successfully or timed out */

  return false;
}
#endif /* USE_HSI */

/****************************************************************************
 * Name: stm32_rcc_enablepll
 *
 * Description:
 *   Enable the Phase Locked Loop (PLL).
 *
 ****************************************************************************/

#if defined (USE_PLL)
static inline bool stm32_rcc_enablepll(void)
{
  uint32_t regval;
  uint32_t timeout;

  /* Preserve reserved bits when altering the PLLCFGR register */

  regval = getreg32(STM32_RCC_PLLCFGR);
  regval &= ~(RCC_PLLCFGR_RESERVED_MASK);

  /* Configure PLL source and enables */

  regval |= STM32_PLLCFGR_PLLSRC | STM32_PLLCFGR_PLLCFG;

  /* Configure PLL multiplication and division factors */

  regval |= STM32_PLLCFGR_PLLM | STM32_PLLCFGR_PLLN;

  /* Configure PLL clock outputs division factors */

  regval |= STM32_PLLCFGR_PLLP | STM32_PLLCFGR_PLLQ | STM32_PLLCFGR_PLLR;

  /* Write PLLCFG register */

  putreg32(regval, STM32_RCC_PLLCFGR);

  /* Enable PLL */

  regval  = getreg32(STM32_RCC_CR);
  regval |= RCC_CR_PLLON;
  putreg32(regval, STM32_RCC_CR);

  /* Wait until PLL ready (or timeout) */

  for (timeout = PLLRDY_TIMEOUT; timeout > 0; timeout--)
    {
      if ((getreg32(STM32_RCC_CR) & RCC_CR_PLLRDY) != 0)
        {
          /* PLL has been enabled successfully and is ready */

          return true;
        }
    }

  /* PLL was not enabled successfully or timed out */

  return false;
}
#endif /* USE_PLL */

/****************************************************************************
 * Name: stm32_stdclockconfig
 *
 * Description:
 *   Called to change to new clock based on settings in board.h.
 *
 * Pre-conditions:
 *   rcc_reset() and rcc_resetbkp() have been called and the HSI is
 *   the MCU's SYSCLK.
 ****************************************************************************/

static void stm32_stdclockconfig(void)
{
  uint32_t regval;

  /* REVISIT:
   *
   * (1) We only support SYSCLK from HSE, HSI, or PLL, with PLL
   *     clocked by HSE or HSI. There is no configuration here for
   *     LSI, LSE, or HSI48.
   *
   * (2) We do not yet explicitly disable any clocks to save power.
   *
   * (3) Do we need to enable clock(s) to any bus(es) before we can
   *     access the PWR registers? And if so, do we need to disable
   *     any such clock(s) before programming the clock tree?
   *
   * (4) Do we need to enable write access or unlock anything
   *     before we can program any of the following things?
   */

  /* Set up the power regulator per configured SYSCLK frequency.
   *
   * Before we begin, make sure voltage regulator is ready to receive
   * any changes by waiting until VOSF bit is cleared by hardware.
   *
   * REVISIT: This should use the implementation in stm32_pwr.c, but
   * it appears to be too different than this family and will need
   * to be refactored accordingly. It is implemented here directly as
   * a stopgap.
   */

  /* REVISIT: Do we need to activate RCC_APB1ENR1_PWREN bit in
   * RCC_APB1ENR1?
   */

  while ((getreg32(STM32_PWR_SR2) & PWR_SR2_VOSF) != 0)
    {
    }

  /* Choose the appropriate PWR VOS range and R1MODE based on the
   * SYSCLK we're going to configure:
   *
   * R1MODE: Enable range 1 boost mode for 150MHz < SYSCLK <= 170MHz,
   *         disable range 1 boost mode for 26MHz < SYSCLK <= 150MHz.
   *
   * VOS: Range 1 for SYSCLK > 26MHz; range 2 for SYSCLK <= 26MHz.
   */

  regval = getreg32(STM32_PWR_CR5);
  regval &= ~(PWR_CR5_R1MODE);
  regval |= PWR_CR5_R1MODE_SETTING;
  putreg32(regval, STM32_PWR_CR5);

  regval = getreg32(STM32_PWR_CR1);
  regval &= ~(PWR_CR1_VOS_MASK);
  regval |= PWR_CR1_VOS_RANGE_SETTING;
  putreg32(regval, STM32_PWR_CR1);

  /* Now we have to wait until VOSF bit is cleared by hardware
   * again
   */

  while ((getreg32(STM32_PWR_SR2) & PWR_SR2_VOSF) != 0)
    {
    }

  /* Now we can program the clock tree */

#if defined(USE_HSE)
  /* The HSE is being used, either as input to the PLL or as SYSCLK
   * itself. Enable the HSE.
   */

  if (stm32_rcc_enablehse() != true)
    {
      /* REVISIT: If we get here, timeout occurred waiting for HSE ready.
       * We should have some sort of mechanism by which the application
       * software can query whether the MCU has started up properly, so
       * that it could possibly report an error or at least not attempt
       * to work with wrong timing. Currently, as there is no mechanism
       * in place to do that, we do not configure the clock any further.
       */

      return;
    }
#endif

#if defined (USE_HSI)
  /* The HSI is being used, either as input to the PLL or as SYSCLK
   * itself. Enable the HSI.
   */

  if (stm32_rcc_enablehsi() != true)
    {
      /* REVISIT: If we get here, timeout occurred waiting for HSI ready.
       * We should have some sort of mechanism by which the application
       * software can query whether the MCU has started up properly, so
       * that it could possibly report an error or at least not attempt
       * to work with wrong timing. Currently, as there is no mechanism
       * in place to do that, we do not configure the clock any further.
       */

       return;
    }
#endif

#if defined(USE_PLL)

  if (stm32_rcc_enablepll() != true)
    {
      /* REVISIT: If we get here, timeout occurred waiting for HSI ready.
       * We should have some sort of mechanism by which the application
       * software can query whether the MCU has started up properly, so
       * that it could possibly report an error or at least not attempt
       * to work with wrong timing. Currently, as there is no mechanism
       * in place to do that, we do not configure the clock any further.
       */

       return;
    }

#endif

  /* Configure FLASH wait states per the SYSCLK frequency that is about
   * to go into effect and enable prefetch to reduce latency due to
   * these wait states (ART accelerator).
   *
   * REVISIT: Should we also enable I-Cache and D-Cache? Also, the
   * reference manual suggests that we must read the ACR register to
   * make sure the latency setting has taken effect. Are we doing that
   * correctly?
   */

  regval  = getreg32(STM32_FLASH_ACR);
  regval &= ~FLASH_ACR_LATENCY_MASK;
  regval |= FLASH_ACR_LATENCY_SETTING;
  regval |= FLASH_ACR_PRFTEN;
  putreg32(regval, STM32_FLASH_ACR);

  while ((getreg32(STM32_FLASH_ACR) & FLASH_ACR_LATENCY_MASK) !=
         FLASH_ACR_LATENCY_SETTING)
    {
    }

  /* Before selecting the SYSCLK source, set the HPRE, PPRE1, and PPRE2
   * dividers.
   */

#if (STM32_SYSCLK_FREQUENCY > 150000000) && (STM32_RCC_CFGR_HPRE == RCC_CFGR_HPRE_SYSCLK)

  /* If SYSCLK > 150MHz, temporarily set the HCLK prescaler (RCC_CFGR_HPRE)
   * to divide by 2 (RCC_CFGR_HPRE_SYSCLKd2) before changing SYSCLK source
   * to PLL. Afterwards, (after waiting at least 1us) change back to no
   * division (RCC_CFGR_HPRE_SYSCLK). See reference manual, section 5.1.5.
   */

  regval = getreg32(STM32_RCC_CFGR);
  regval &= ~(RCC_CFGR_HPRE_MASK);
  regval |= RCC_CFGR_HPRE_SYSCLKd2;
  putreg32(regval, STM32_RCC_CFGR);
#endif

  /* Select the system clock source as defined in board.h. This could
   * be the HSI, HSE, or PLL (most likely the PLL).
   */

  regval  = getreg32(STM32_RCC_CFGR);
  regval &= ~(RCC_CFGR_SW_MASK);
  regval |= STM32_SYSCLK_SW;
  putreg32(regval, STM32_RCC_CFGR);

  /* Wait until the selected source is used as the system clock source */

  while ((getreg32(STM32_RCC_CFGR) & RCC_CFGR_SWS_MASK) != STM32_SYSCLK_SWS)
    {
    }

  /* Before we set HCLK prescaler to the correct value, temporarily
   * divide APB1 and APB2 clocks by 16 to avoid problems.
   */

  regval = getreg32(STM32_RCC_CFGR);
  regval &= ~(RCC_CFGR_PPRE1_MASK | RCC_CFGR_PPRE2_MASK);
  regval |= (RCC_CFGR_PPRE1_HCLKd16 | RCC_CFGR_PPRE2_HCLKd16);
  putreg32(regval, STM32_RCC_CFGR);

  /* Now set HCLK prescaler to the correct value */

  regval = getreg32(STM32_RCC_CFGR);
  regval &= ~(RCC_CFGR_HPRE_MASK);
  regval |= STM32_RCC_CFGR_HPRE;
  putreg32(regval, STM32_RCC_CFGR);

  /* Now set APB1 and APB2 prescalers to the correct value */

  regval = getreg32(STM32_RCC_CFGR);
  regval &= ~(RCC_CFGR_PPRE1_MASK | RCC_CFGR_PPRE2_MASK);
  regval |= (STM32_RCC_CFGR_PPRE1 | STM32_RCC_CFGR_PPRE2);
  putreg32(regval, STM32_RCC_CFGR);
}

/****************************************************************************
 * Name: rcc_enableperipherals
 *
 * Description:
 *   Enable all peripheral buses and all configured peripherals.
 *
 ****************************************************************************/

static inline void rcc_enableperipherals(void)
{
  rcc_enableahb1();
  rcc_enableahb2();
  rcc_enableahb3();
  rcc_enableapb1();
  rcc_enableapb2();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

