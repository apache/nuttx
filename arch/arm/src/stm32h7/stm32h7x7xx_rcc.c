/****************************************************************************
 * arch/arm/src/stm32h7/stm32h7x3xx_rcc.c
 *
 *   Copyright (C) 2018, 2019 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david.sidrane@nscdg.com>
 *            Mateusz Szafoni <raiden00@railab.me>
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

#include "stm32_pwr.h"
#include "hardware/stm32_axi.h"
#include "hardware/stm32_syscfg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Allow up to 100 milliseconds for the high speed clock to become ready.
 * that is a very long delay, but if the clock does not become ready we are
 * hosed anyway.  Normally this is very fast, but I have seen at least one
 * board that required this long, long timeout for the HSE to be ready.
 */

#define HSERDY_TIMEOUT (100 * CONFIG_BOARD_LOOPSPERMSEC)

/* Same for HSI */

#define HSIRDY_TIMEOUT HSERDY_TIMEOUT

/* HSE divisor to yield ~1MHz RTC clock */

#define HSE_DIVISOR (STM32_HSE_FREQUENCY + 500000) / 1000000

/* FLASH wait states */

#if !defined(BOARD_FLASH_WAITSTATES)
#  error BOARD_FLASH_WAITSTATES not defined
#elif BOARD_FLASH_WAITSTATES < 0 || BOARD_FLASH_WAITSTATES > 15
#  error BOARD_FLASH_WAITSTATES is out of range
#endif

/* Voltage output scale (default to Scale 1 mode) */

#ifndef STM32_PWR_VOS_SCALE
#  define STM32_PWR_VOS_SCALE PWR_D3CR_VOS_SCALE_1
#endif

#if !defined(BOARD_FLASH_PROGDELAY)
#  if STM32_PWR_VOS_SCALE == PWR_D3CR_VOS_SCALE_1
#    if STM32_SYSCLK_FREQUENCY <= 70000000 && BOARD_FLASH_WAITSTATES == 0
#      define BOARD_FLASH_PROGDELAY  0
#    elif STM32_SYSCLK_FREQUENCY <= 140000000 && BOARD_FLASH_WAITSTATES == 1
#      define BOARD_FLASH_PROGDELAY  10
#    elif STM32_SYSCLK_FREQUENCY <= 185000000 && BOARD_FLASH_WAITSTATES == 2
#      define BOARD_FLASH_PROGDELAY  1
#    elif STM32_SYSCLK_FREQUENCY <= 210000000 && BOARD_FLASH_WAITSTATES == 2
#      define BOARD_FLASH_PROGDELAY  2
#    elif STM32_SYSCLK_FREQUENCY <= 225000000 && BOARD_FLASH_WAITSTATES == 3
#      define BOARD_FLASH_PROGDELAY  2
#    else
#      define BOARD_FLASH_PROGDELAY  2
#    endif
#  endif
#endif

/* PLL are only enabled if the P,Q or R outputs are enabled. */

#undef USE_PLL1
#if STM32_PLLCFG_PLL1CFG & (RCC_PLLCFGR_DIVP1EN | RCC_PLLCFGR_DIVQ1EN | \
                            RCC_PLLCFGR_DIVR1EN)
#  define USE_PLL1
#endif

#undef USE_PLL2
#if STM32_PLLCFG_PLL2CFG & (RCC_PLLCFGR_DIVP2EN | RCC_PLLCFGR_DIVQ2EN | \
                            RCC_PLLCFGR_DIVR2EN)
#  define USE_PLL2
#endif

#undef USE_PLL3
#if STM32_PLLCFG_PLL3CFG & (RCC_PLLCFGR_DIVP3EN | RCC_PLLCFGR_DIVQ3EN | \
                            RCC_PLLCFGR_DIVR3EN)
#  define USE_PLL3
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
 *   Reset the RCC clock configuration to the default reset state
 *
 ****************************************************************************/

static inline void rcc_reset(void)
{
  uint32_t regval;

  /* Enable the Internal High Speed clock (HSI) */

  regval = getreg32(STM32_RCC_CR);
  regval |= RCC_CR_HSION;
  putreg32(regval, STM32_RCC_CR);

#if defined(CONFIG_STM32H7_AXI_SRAM_CORRUPTION_WAR)
  /* Errata 2.2.9 Enable workaround for Reading from AXI SRAM may lead to
   * data read corruption. See ES0392 Rev 6.
   */

  putreg32(AXI_TARG_READ_ISS_OVERRIDE, STM32_AXI_TARG7_FN_MOD);
#endif

  /* Reset CFGR register */

  putreg32(0x00000000, STM32_RCC_CFGR);

  /* Reset HSION, HSEON, CSSON and PLLON bits */

  regval  = getreg32(STM32_RCC_CR);
  regval &= ~(RCC_CR_HSEON | RCC_CR_HSI48ON |
              RCC_CR_CSION | RCC_CR_PLL1ON |
              RCC_CR_PLL2ON | RCC_CR_PLL3ON |
              RCC_CR_HSIDIV_MASK);

  /* Set HSI predivider to default (4, 16MHz) */

  regval |= RCC_CR_HSIDIV_4;

  putreg32(regval, STM32_RCC_CR);

  /* Reset PLLCFGR register to reset default */

  putreg32(RCC_PLLCFGR_RESET, STM32_RCC_PLLCFGR);

  /* Reset HSEBYP bit */

  regval  = getreg32(STM32_RCC_CR);
  regval &= ~RCC_CR_HSEBYP;
  putreg32(regval, STM32_RCC_CR);

  /* Disable all interrupts */

  putreg32(0x00000000, STM32_RCC_CIER);
}

/****************************************************************************
 * Name: rcc_enableahb1
 *
 * Description:
 *   Enable selected AHB1 peripherals
 *
 ****************************************************************************/

static inline void rcc_enableahb1(void)
{
  uint32_t regval;

  /* Set the appropriate bits in the AHB1ENR register to enabled the
   * selected AHB1 peripherals.
   */

  regval = getreg32(STM32_RCC_AHB1ENR);
#if defined(CONFIG_STM32H7_ADC1) || defined(CONFIG_STM32H7_ADC2)
  /* ADC1 & 2 clock enable */

  regval |= RCC_AHB1ENR_ADC12EN;
#endif

#ifdef CONFIG_STM32H7_DMA1
  /* DMA 1 clock enable */

  regval |= RCC_AHB1ENR_DMA1EN;
#endif

#ifdef CONFIG_STM32H7_DMA2
  /* DMA 2 clock enable */

  regval |= RCC_AHB1ENR_DMA2EN;
#endif

#ifdef CONFIG_STM32H7_OTGFS
  /* USB OTG FS clock enable */

  regval |= RCC_AHB1ENR_OTGFSEN;
#endif

#ifdef CONFIG_STM32H7_OTGHS
#ifdef BOARD_ENABLE_USBOTG_HSULPI
  /* Enable clocking for USB OTG HS and external PHY */

  regval |= (RCC_AHB1ENR_OTGHSEN | RCC_AHB1ENR_OTGHSULPIEN);
#else
  /* Enable only clocking for USB OTG HS */

  regval |= RCC_AHB1ENR_OTGHSEN;
#endif
#endif

#ifdef CONFIG_STM32H7_ETHMAC
  /* Enable ethernet clocks */

  regval |= (RCC_AHB1ENR_ETH1MACEN | RCC_AHB1ENR_ETH1TXEN |
             RCC_AHB1ENR_ETH1RXEN);
#endif

  putreg32(regval, STM32_RCC_AHB1ENR);   /* Enable peripherals */
}

/****************************************************************************
 * Name: rcc_enableahb2
 *
 * Description:
 *   Enable selected AHB2 peripherals
 *
 ****************************************************************************/

static inline void rcc_enableahb2(void)
{
  uint32_t regval;

  /* Set the appropriate bits in the AHB2ENR register to enabled the
   * selected AHB2 peripherals.
   */

  regval = getreg32(STM32_RCC_AHB2ENR);

  /* TODO: ... */

  putreg32(regval, STM32_RCC_AHB2ENR);   /* Enable peripherals */
}

/****************************************************************************
 * Name: rcc_enableahb3
 *
 * Description:
 *   Enable selected AHB3 peripherals
 *
 ****************************************************************************/

static inline void rcc_enableahb3(void)
{
  uint32_t regval;

  /* Set the appropriate bits in the AHB3ENR register to enabled the
   * selected AHB3 peripherals.
   */

  regval = getreg32(STM32_RCC_AHB3ENR);

#ifdef CONFIG_STM32H7_MDMA
  /* MDMA clock enable */

  regval |= RCC_AHB3ENR_MDMAEN;
#endif

#ifdef CONFIG_STM32H7_SDMMC1
  /* SDMMC clock enable */

  regval |= RCC_AHB3ENR_SDMMC1EN;
#endif

#ifdef CONFIG_STM32H7_FMC
  /* Flexible static memory controller module clock enable */

  regval |= RCC_AHB3ENR_FMCEN;
#endif

  /* TODO: ... */

  putreg32(regval, STM32_RCC_AHB3ENR);   /* Enable peripherals */
}

/****************************************************************************
 * Name: rcc_enableahb4
 *
 * Description:
 *   Enable selected AHB4 peripherals
 *
 ****************************************************************************/

static inline void rcc_enableahb4(void)
{
  uint32_t regval;

  /* Set the appropriate bits in the AHB4ENR register to enabled the
   * selected AHB4 peripherals.
   */

  regval = getreg32(STM32_RCC_AHB4ENR);

#ifdef CONFIG_STM32H7_ADC3
  /* ADC3 clock enable */

  regval |= RCC_AHB4ENR_ADC3EN;
#endif

  /* Enable GPIO, GPIOB, ... GPIOK */

#if STM32H7_NGPIO > 0
  regval |= (RCC_AHB4ENR_GPIOAEN
#if STM32H7_NGPIO > 1
             | RCC_AHB4ENR_GPIOBEN
#endif
#if STM32H7_NGPIO > 2
             | RCC_AHB4ENR_GPIOCEN
#endif
#if STM32H7_NGPIO > 3
             | RCC_AHB4ENR_GPIODEN
#endif
#if STM32H7_NGPIO > 4
             | RCC_AHB4ENR_GPIOEEN
#endif
#if (STM32H7_NGPIO > 5) && (defined(CONFIG_STM32H7_HAVE_GPIOF))
             | RCC_AHB4ENR_GPIOFEN
#endif
#if (STM32H7_NGPIO > 6) && (defined(CONFIG_STM32H7_HAVE_GPIOG))
             | RCC_AHB4ENR_GPIOGEN
#endif
#if STM32H7_NGPIO > 7
             | RCC_AHB4ENR_GPIOHEN
#endif
#if STM32H7_NGPIO > 8
             | RCC_AHB4ENR_GPIOIEN
#endif
#if STM32H7_NGPIO > 9
             | RCC_AHB4ENR_GPIOJEN
#endif
#if STM32H7_NGPIO > 10
             | RCC_AHB4ENR_GPIOKEN
#endif
    );
#endif

#ifdef CONFIG_STM32H7_BDMA
  /* BDMA clock enable */

  regval |= RCC_AHB4ENR_BDMAEN;
#endif

#ifdef CONFIG_STM32H7_CRC
  /* CRC clock enable */

  regval |= RCC_AHB4ENR_CRCEN;
#endif

#ifdef CONFIG_STM32H7_BKPSRAM
  /* Backup SRAM clock enable */

  regval |= RCC_AHB4ENR_BKPSRAMEN;
#endif

  putreg32(regval, STM32_RCC_AHB4ENR);   /* Enable peripherals */
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

  /* Set the appropriate bits in the APB1L/HENR register to enabled the
   * selected APB1 peripherals.
   */

  regval = getreg32(STM32_RCC_APB1LENR);

#ifdef CONFIG_STM32H7_SPI2
  /* SPI2 clock enable */

  regval |= RCC_APB1LENR_SPI2EN;
#endif

#ifdef CONFIG_STM32H7_SPI3
  /* SPI3 clock enable */

  regval |= RCC_APB1LENR_SPI3EN;
#endif

#ifdef CONFIG_STM32H7_I2C1
  /* I2C1 clock enable */

  regval |= RCC_APB1LENR_I2C1EN;
#endif

#ifdef CONFIG_STM32H7_I2C2
  /* I2C2 clock enable */

  regval |= RCC_APB1LENR_I2C2EN;
#endif

#ifdef CONFIG_STM32H7_I2C3
  /* I2C3 clock enable */

  regval |= RCC_APB1LENR_I2C3EN;
#endif

  /* TODO: ... */

  putreg32(regval, STM32_RCC_APB1LENR);   /* Enable APB1L peripherals */

  regval = getreg32(STM32_RCC_APB1HENR);

  /* TODO: ... */

  putreg32(regval, STM32_RCC_APB1HENR);   /* Enable APB1H peripherals */
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

#ifdef CONFIG_STM32H7_SPI1
  /* SPI1 clock enable */

  regval |= RCC_APB2ENR_SPI1EN;
#endif

#ifdef CONFIG_STM32H7_SPI4
  /* SPI4 clock enable */

  regval |= RCC_APB2ENR_SPI4EN;
#endif

#ifdef CONFIG_STM32H7_SPI5
  /* SPI5 clock enable */

  regval |= RCC_APB2ENR_SPI5EN;
#endif

#ifdef CONFIG_STM32H7_SDMMC2
  /* SDMMC2 clock enable */

  regval |= RCC_APB2ENR_SDMMC2EN;
#endif

  putreg32(regval, STM32_RCC_APB2ENR);   /* Enable peripherals */
}

/****************************************************************************
 * Name: rcc_enableapb3
 *
 * Description:
 *   Enable selected APB3 peripherals
 *
 ****************************************************************************/

static inline void rcc_enableapb3(void)
{
  uint32_t regval;

  /* Set the appropriate bits in the APB3ENR register to enabled the
   * selected APB3 peripherals.
   */

  regval = getreg32(STM32_RCC_APB3ENR);

  /* TODO: ... */

  putreg32(regval, STM32_RCC_APB3ENR);   /* Enable peripherals */
}

/****************************************************************************
 * Name: rcc_enableapb4
 *
 * Description:
 *   Enable selected APB4 peripherals
 *
 ****************************************************************************/

static inline void rcc_enableapb4(void)
{
  uint32_t regval;

  /* Set the appropriate bits in the APB4ENR register to enabled the
   * selected APB4 peripherals.
   */

  regval = getreg32(STM32_RCC_APB4ENR);

#ifdef CONFIG_STM32H7_SYSCFG
  /* System configuration controller clock enable */

  regval |= RCC_APB4ENR_SYSCFGEN;
#endif

#ifdef CONFIG_STM32H7_I2C4
  /* I2C4 clock enable */

  regval |= RCC_APB4ENR_I2C4EN;
#endif

#ifdef CONFIG_STM32H7_SPI6
  /* SPI6 clock enable */

  regval |= RCC_APB4ENR_SPI6EN;
#endif

  /* TODO: ... */

  putreg32(regval, STM32_RCC_APB4ENR);   /* Enable peripherals */
}

/****************************************************************************
 * Name: stm32_stdclockconfig
 *
 * Description:
 *   Called to change to new clock based on settings in board.h
 *
 *   NOTE:  This logic would need to be extended if you need to select low-
 *   power clocking modes!
 ****************************************************************************/

#ifndef CONFIG_STM32H7_CUSTOM_CLOCKCONFIG
static void stm32_stdclockconfig(void)
{
  uint32_t regval;
  volatile int32_t timeout;

#ifdef STM32_BOARD_USEHSI
  /* Enable Internal High-Speed Clock (HSI) */

  regval  = getreg32(STM32_RCC_CR);
  regval |= RCC_CR_HSION;           /* Enable HSI */
  putreg32(regval, STM32_RCC_CR);

  /* Wait until the HSI is ready (or until a timeout elapsed) */

  for (timeout = HSIRDY_TIMEOUT; timeout > 0; timeout--)
    {
      /* Check if the HSIRDY flag is the set in the CR */

      if ((getreg32(STM32_RCC_CR) & RCC_CR_HSIRDY) != 0)
        {
          /* If so, then break-out with timeout > 0 */

          break;
        }
    }

#else /* if STM32_BOARD_USEHSE */
  /* Enable External High-Speed Clock (HSE) */

  regval  = getreg32(STM32_RCC_CR);
#ifdef STM32_HSEBYP_ENABLE          /* May be defined in board.h header file */
  regval |= RCC_CR_HSEBYP;          /* Enable HSE clock bypass */
#else
  regval &= ~RCC_CR_HSEBYP;         /* Disable HSE clock bypass */
#endif
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
#endif

#define CONFIG_STM32H7_HSI48
#ifdef CONFIG_STM32H7_HSI48
  /* Enable HSI48 */

  regval  = getreg32(STM32_RCC_CR);
  regval |= RCC_CR_HSI48ON;
  putreg32(regval, STM32_RCC_CR);

  /* Wait until the HSI48 is ready */

  while ((getreg32(STM32_RCC_CR) & RCC_CR_HSI48RDY) == 0)
    {
    }
#endif

  /* Check for a timeout.  If this timeout occurs, then we are hosed.  We
   * have no real back-up plan, although the following logic makes it look
   * as though we do.
   */

  if (timeout > 0)
    {
      /* Set the D1 domain Core prescaler and the HCLK source/divider */

      regval = getreg32(STM32_RCC_D1CFGR);
      regval &= ~(RCC_D1CFGR_HPRE_MASK | RCC_D1CFGR_D1CPRE_MASK);
      regval |= (STM32_RCC_D1CFGR_HPRE | STM32_RCC_D1CFGR_D1CPRE);
      putreg32(regval, STM32_RCC_D1CFGR);

      /* Set PCLK1 */

      regval = getreg32(STM32_RCC_D2CFGR);
      regval &= ~RCC_D2CFGR_D2PPRE2_MASK;
      regval |= STM32_RCC_D2CFGR_D2PPRE1;
      putreg32(regval, STM32_RCC_D2CFGR);

      /* Set PCLK2 */

      regval = getreg32(STM32_RCC_D2CFGR);
      regval &= ~RCC_D2CFGR_D2PPRE2_MASK;
      regval |= STM32_RCC_D2CFGR_D2PPRE2;
      putreg32(regval, STM32_RCC_D2CFGR);

      /* Set PCLK3 */

      regval = getreg32(STM32_RCC_D1CFGR);
      regval &= ~RCC_D1CFGR_D1PPRE_MASK;
      regval |= STM32_RCC_D1CFGR_D1PPRE;
      putreg32(regval, STM32_RCC_D1CFGR);

      /* Set PCLK4 */

      regval = getreg32(STM32_RCC_D3CFGR);
      regval &= ~RCC_D3CFGR_D3PPRE_MASK;
      regval |= STM32_RCC_D3CFGR_D3PPRE;
      putreg32(regval, STM32_RCC_D3CFGR);

#ifdef CONFIG_STM32H7_RTC_HSECLOCK
      /* Set the RTC clock divisor */

      regval = getreg32(STM32_RCC_CFGR);
      regval &= ~RCC_CFGR_RTCPRE_MASK;
      regval |= RCC_CFGR_RTCPRE(HSE_DIVISOR);
      putreg32(regval, STM32_RCC_CFGR);
#endif

      /* Configure PLL123 clock source and multipiers */

#ifdef STM32_BOARD_USEHSI
      regval = (RCC_PLLCKSELR_PLLSRC_HSI |
                STM32_PLLCFG_PLL1M |
                STM32_PLLCFG_PLL2M |
                STM32_PLLCFG_PLL3M);
#else /* if STM32_BOARD_USEHSE */
      regval = (RCC_PLLCKSELR_PLLSRC_HSE |
                STM32_PLLCFG_PLL1M |
                STM32_PLLCFG_PLL2M |
                STM32_PLLCFG_PLL3M);
#endif
      putreg32(regval, STM32_RCC_PLLCKSELR);

      /* Each PLL offers 3 outputs with post-dividers (PLLxP/PLLxQ/PLLxR) */

      /* Configure PLL1 dividers */

      regval = (STM32_PLLCFG_PLL1N |
                STM32_PLLCFG_PLL1P |
                STM32_PLLCFG_PLL1Q |
                STM32_PLLCFG_PLL1R);
      putreg32(regval, STM32_RCC_PLL1DIVR);

      /* Configure PLL2 dividers */

      regval = (STM32_PLLCFG_PLL2N |
                STM32_PLLCFG_PLL2P |
                STM32_PLLCFG_PLL2Q |
                STM32_PLLCFG_PLL2R);
      putreg32(regval, STM32_RCC_PLL2DIVR);

      /* Configure PLL3 dividers */

      regval = (STM32_PLLCFG_PLL3N |
                STM32_PLLCFG_PLL3P |
                STM32_PLLCFG_PLL3Q |
                STM32_PLLCFG_PLL3R);
      putreg32(regval, STM32_RCC_PLL3DIVR);

      /* Configure PLLs */

      regval = (STM32_PLLCFG_PLL1CFG |
                STM32_PLLCFG_PLL2CFG |
                STM32_PLLCFG_PLL3CFG);
      putreg32(regval, STM32_RCC_PLLCFGR);

      regval = getreg32(STM32_RCC_CR);
#if defined(USE_PLL1)
      /* Enable the PLL1 */

      regval |= RCC_CR_PLL1ON;
#endif

#if defined(USE_PLL2)
      /* Enable the PLL2 */

      regval |= RCC_CR_PLL2ON;
#endif

#if defined(USE_PLL3)
      /* Enable the PLL3 */

      regval |= RCC_CR_PLL3ON;
#endif
      putreg32(regval, STM32_RCC_CR);

#if defined(USE_PLL1)
      /* Wait until the PLL1 is ready */

      while ((getreg32(STM32_RCC_CR) & RCC_CR_PLL1RDY) == 0)
        {
        }
#endif

#if defined(USE_PLL2)
      /* Wait until the PLL2 is ready */

      while ((getreg32(STM32_RCC_CR) & RCC_CR_PLL2RDY) == 0)
        {
        }
#endif

#if defined(USE_PLL3)
      /* Wait until the PLL3 is ready */

      while ((getreg32(STM32_RCC_CR) & RCC_CR_PLL3RDY) == 0)
        {
        }
#endif

      /* We must write the lower byte of the PWR_CR3 register is written once
       * after POR and it shall be written before changing VOS level or
       * ck_sys clock frequency. No limitation applies to the upper bytes.
       *
       * Programming data corresponding to an invalid combination of
       * LDOEN and BYPASS bits will be ignored: data will not be written,
       * the written-once mechanism will lock the register and any further
       * write access will be ignored. The default supply configuration will
       * be kept and the ACTVOSRDY bit in PWR control status register 1
       * (PWR_CSR1) will go on indicating invalid voltage levels.
       *
       * N.B. The system shall be power cycled before writing a new value.
       */

      regval = getreg32(STM32_PWR_CR3);
      regval |= STM32_PWR_CR3_LDOEN | STM32_PWR_CR3_LDOESCUEN;
      putreg32(regval, STM32_PWR_CR3);

#if 0

      /* Set the voltage output scale */

      regval = getreg32(STM32_PWR_D3CR);
      regval &= ~STM32_PWR_D3CR_VOS_MASK;
      regval |= STM32_PWR_VOS_SCALE;
      putreg32(regval, STM32_PWR_D3CR);

      while ((getreg32(STM32_PWR_D3CR) & STM32_PWR_D3CR_VOSRDY) == 0)
        {
        }

      /* Over-drive is needed if
       *  - Voltage output scale 1 mode is selected and SYSCLK frequency is
       *    over 400 MHz.
       */

      if ((STM32_PWR_VOS_SCALE == PWR_D3CR_VOS_SCALE_1) &&
           STM32_SYSCLK_FREQUENCY > 400000000)
        {
          /* Enable System configuration controller clock to Enable ODEN */

          regval = getreg32(STM32_RCC_APB4ENR);
          regval |= RCC_APB4ENR_SYSCFGEN;
          putreg32(regval, STM32_RCC_APB4ENR);

          /* Enable Overdrive to extend the clock frequency up to 480 MHz. */

          regval = getreg32(STM32_SYSCFG_PWRCR);
          regval |= SYSCFG_PWRCR_ODEN;
          putreg32(regval, STM32_SYSCFG_PWRCR);

          while ((getreg32(STM32_PWR_D3CR) & STM32_PWR_D3CR_VOSRDY) == 0)
            {
            }
        }
#endif

      /* Configure FLASH wait states */

      regval = FLASH_ACR_WRHIGHFREQ(BOARD_FLASH_PROGDELAY) |
               FLASH_ACR_LATENCY(BOARD_FLASH_WAITSTATES);

      putreg32(regval, STM32_FLASH_ACR);

      /* Select the PLL1P as system clock source */

      regval = getreg32(STM32_RCC_CFGR);
      regval &= ~RCC_CFGR_SW_MASK;
      regval |= RCC_CFGR_SW_PLL1;
      putreg32(regval, STM32_RCC_CFGR);

      /* Wait until the PLL source is used as the system clock source */

      while ((getreg32(STM32_RCC_CFGR) & RCC_CFGR_SWS_MASK) !=
             RCC_CFGR_SWS_PLL1)
        {
        }

      /* Configure I2C source clock */

#if defined(STM32_RCC_D2CCIP2R_I2C123SRC)
      regval = getreg32(STM32_RCC_D2CCIP2R);
      regval &= ~RCC_D2CCIP2R_I2C123SEL_MASK;
      regval |= STM32_RCC_D2CCIP2R_I2C123SRC;
      putreg32(regval, STM32_RCC_D2CCIP2R);
#endif

#if defined(STM32_RCC_D3CCIPR_I2C4SRC)
      regval = getreg32(STM32_RCC_D3CCIPR);
      regval &= ~RCC_D3CCIPR_I2C4SEL_MASK;
      regval |= STM32_RCC_D3CCIPR_I2C4SRC;
      putreg32(regval, STM32_RCC_D3CCIPR);
#endif

      /* Configure SPI source clock */

#if defined(STM32_RCC_D2CCIP1R_SPI123SRC)
      regval = getreg32(STM32_RCC_D2CCIP1R);
      regval &= ~RCC_D2CCIP1R_SPI123SEL_MASK;
      regval |= STM32_RCC_D2CCIP1R_SPI123SRC;
      putreg32(regval, STM32_RCC_D2CCIP1R);
#endif

#if defined(STM32_RCC_D2CCIP1R_SPI45SRC)
      regval = getreg32(STM32_RCC_D2CCIP1R);
      regval &= ~RCC_D2CCIP1R_SPI45SEL_MASK;
      regval |= STM32_RCC_D2CCIP1R_SPI45SRC;
      putreg32(regval, STM32_RCC_D2CCIP1R);
#endif

#if defined(STM32_RCC_D3CCIPR_SPI6SRC)
      regval = getreg32(STM32_RCC_D3CCIPR);
      regval &= ~RCC_D3CCIPR_SPI6SEL_MASK;
      regval |= STM32_RCC_D3CCIPR_SPI6SRC;
      putreg32(regval, STM32_RCC_D3CCIPR);
#endif

      /* Configure USB source clock */

#if defined(STM32_RCC_D2CCIP2R_USBSRC)
      regval = getreg32(STM32_RCC_D2CCIP2R);
      regval &= ~RCC_D2CCIP2R_USBSEL_MASK;
      regval |= STM32_RCC_D2CCIP2R_USBSRC;
      putreg32(regval, STM32_RCC_D2CCIP2R);
#endif

      /* Configure ADC source clock */

#if defined(STM32_RCC_D3CCIPR_ADCSEL)
      regval = getreg32(STM32_RCC_D3CCIPR);
      regval &= ~RCC_D3CCIPR_ADCSEL_MASK;
      regval |= STM32_RCC_D3CCIPR_ADCSEL;
      putreg32(regval, STM32_RCC_D3CCIPR);
#endif

#if defined(CONFIG_STM32H7_IWDG) || defined(CONFIG_STM32H7_RTC_LSICLOCK)
      /* Low speed internal clock source LSI */

      stm32_rcc_enablelsi();
#endif

#if defined(CONFIG_STM32H7_RTC_LSECLOCK)
      /* Low speed external clock source LSE
       *
       * TODO: There is another case where the LSE needs to
       * be enabled: if the MCO1 pin selects LSE as source.
       */

      stm32_rcc_enablelse();
#endif
    }
}
#endif

/****************************************************************************
 * Name: rcc_enableperiphals
 ****************************************************************************/

static inline void rcc_enableperipherals(void)
{
  rcc_enableahb1();
  rcc_enableahb2();
  rcc_enableahb3();
  rcc_enableahb4();
  rcc_enableapb1();
  rcc_enableapb2();
  rcc_enableapb3();
  rcc_enableapb4();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
