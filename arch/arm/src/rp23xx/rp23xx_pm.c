/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_pm.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#include <arch/board/board.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/power/pm.h>

#include "arm_internal.h"
#include "nvic.h"

#include "rp23xx_pm.h"
#include "rp23xx_gpio.h"
#include "rp23xx_pll.h"

#include "hardware/rp23xx_clocks.h"
#include "hardware/rp23xx_powman.h"
#include "hardware/rp23xx_xosc.h"
#include "hardware/rp23xx_pll.h"
#include "hardware/rp23xx_pads_bank0.h"
#include "hardware/rp23xx_io_bank0.h"
#include "hardware/rp23xx_memorymap.h"
#include "hardware/rp23xx_resets.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* No flash fetch works while the crystal oscillator is stopped, so the
 * dormant sequence runs from RAM (.time_critical).
 */

#define RP23XX_PM_RAMFUNC \
  __attribute__((section(".time_critical.rp23xx_pm"), noinline))

/* POWMAN ignores a write without this password in the top 16 bits */

#define POWMAN_PASSWORD   0x5afe0000

/* Atomic set/clear aliases of a bus register (RP2350 memory map). */

#define POWMAN_SET_ALIAS  0x2000
#define POWMAN_CLR_ALIAS  0x3000

/* Clocks that keep running through a PM_STANDBY WFI.  SLEEP_EN0/1 use the
 * bits of WAKE_EN0/1.  Only blocks with no driver in the configuration are
 * gated: the rp23xx drivers have no PM callbacks to say they are idle.
 */

#define RP23XX_PM_SLEEP_EN0_BASE \
  (RP23XX_CLOCKS_WAKE_EN0_CLK_SYS_CLOCKS     | \
   RP23XX_CLOCKS_WAKE_EN0_CLK_SYS_ACCESSCTRL | \
   RP23XX_CLOCKS_WAKE_EN0_CLK_SYS_BOOTRAM    | \
   RP23XX_CLOCKS_WAKE_EN0_CLK_SYS_BUSCTRL    | \
   RP23XX_CLOCKS_WAKE_EN0_CLK_SYS_BUSFABRIC  | \
   RP23XX_CLOCKS_WAKE_EN0_CLK_SYS_IO         | \
   RP23XX_CLOCKS_WAKE_EN0_CLK_SYS_PADS       | \
   RP23XX_CLOCKS_WAKE_EN0_CLK_SYS_PLL_SYS    | \
   RP23XX_CLOCKS_WAKE_EN0_CLK_REF_POWMAN     | \
   RP23XX_CLOCKS_WAKE_EN0_CLK_SYS_POWMAN     | \
   RP23XX_CLOCKS_WAKE_EN0_CLK_SYS_PSM        | \
   RP23XX_CLOCKS_WAKE_EN0_CLK_SYS_RESETS     | \
   RP23XX_CLOCKS_WAKE_EN0_CLK_SYS_ROM        | \
   RP23XX_CLOCKS_WAKE_EN0_CLK_SYS_ROSC       | \
   RP23XX_CLOCKS_WAKE_EN0_CLK_SYS_SIO)

#define RP23XX_PM_SLEEP_EN1_BASE \
  (RP23XX_CLOCKS_WAKE_EN1_CLK_SYS_SRAM0    | \
   RP23XX_CLOCKS_WAKE_EN1_CLK_SYS_SRAM1    | \
   RP23XX_CLOCKS_WAKE_EN1_CLK_SYS_SRAM2    | \
   RP23XX_CLOCKS_WAKE_EN1_CLK_SYS_SRAM3    | \
   RP23XX_CLOCKS_WAKE_EN1_CLK_SYS_SRAM4    | \
   RP23XX_CLOCKS_WAKE_EN1_CLK_SYS_SRAM5    | \
   RP23XX_CLOCKS_WAKE_EN1_CLK_SYS_SRAM6    | \
   RP23XX_CLOCKS_WAKE_EN1_CLK_SYS_SRAM7    | \
   RP23XX_CLOCKS_WAKE_EN1_CLK_SYS_SRAM8    | \
   RP23XX_CLOCKS_WAKE_EN1_CLK_SYS_SRAM9    | \
   RP23XX_CLOCKS_WAKE_EN1_CLK_SYS_SYSCFG   | \
   RP23XX_CLOCKS_WAKE_EN1_CLK_SYS_SYSINFO  | \
   RP23XX_CLOCKS_WAKE_EN1_CLK_SYS_TBMAN    | \
   RP23XX_CLOCKS_WAKE_EN1_CLK_REF_TICKS    | \
   RP23XX_CLOCKS_WAKE_EN1_CLK_SYS_TICKS    | \
   RP23XX_CLOCKS_WAKE_EN1_CLK_SYS_WATCHDOG | \
   RP23XX_CLOCKS_WAKE_EN1_CLK_SYS_XIP      | \
   RP23XX_CLOCKS_WAKE_EN1_CLK_SYS_XOSC)

/* Optional blocks, kept only when their driver is configured. */

#ifdef CONFIG_RP23XX_ADC
#  define RP23XX_PM_EN0_ADC  (RP23XX_CLOCKS_WAKE_EN0_CLK_SYS_ADC | \
                              RP23XX_CLOCKS_WAKE_EN0_CLK_ADC_ADC)
#else
#  define RP23XX_PM_EN0_ADC  0
#endif

#ifdef CONFIG_RP23XX_DMAC
#  define RP23XX_PM_EN0_DMA  RP23XX_CLOCKS_WAKE_EN0_CLK_SYS_DMA
#else
#  define RP23XX_PM_EN0_DMA  0
#endif

#ifdef CONFIG_RP23XX_I2C0
#  define RP23XX_PM_EN0_I2C0 RP23XX_CLOCKS_WAKE_EN0_CLK_SYS_I2C0
#else
#  define RP23XX_PM_EN0_I2C0 0
#endif

#ifdef CONFIG_RP23XX_I2C1
#  define RP23XX_PM_EN0_I2C1 RP23XX_CLOCKS_WAKE_EN0_CLK_SYS_I2C1
#else
#  define RP23XX_PM_EN0_I2C1 0
#endif

#ifdef CONFIG_RP23XX_PWM
#  define RP23XX_PM_EN0_PWM  RP23XX_CLOCKS_WAKE_EN0_CLK_SYS_PWM
#else
#  define RP23XX_PM_EN0_PWM  0
#endif

#ifdef CONFIG_RP23XX_OTP
#  define RP23XX_PM_EN0_OTP  (RP23XX_CLOCKS_WAKE_EN0_CLK_SYS_OTP | \
                              RP23XX_CLOCKS_WAKE_EN0_CLK_REF_OTP)
#else
#  define RP23XX_PM_EN0_OTP  0
#endif

#ifdef CONFIG_CRYPTO_CRYPTODEV_HARDWARE
#  define RP23XX_PM_EN0_SHA  RP23XX_CLOCKS_WAKE_EN0_CLK_SYS_SHA256
#else
#  define RP23XX_PM_EN0_SHA  0
#endif

/* The PIO blocks are shared by several drivers */

#if defined(CONFIG_RP23XX_I2S) || defined(CONFIG_WS2812) || \
    defined(CONFIG_IEEE80211_INFINEON_CYW43439)
#  define RP23XX_PM_EN0_PIO  (RP23XX_CLOCKS_WAKE_EN0_CLK_SYS_PIO0 | \
                              RP23XX_CLOCKS_WAKE_EN0_CLK_SYS_PIO1 | \
                              RP23XX_CLOCKS_WAKE_EN0_CLK_SYS_PIO2)
#else
#  define RP23XX_PM_EN0_PIO  0
#endif

#ifdef CONFIG_USBDEV
#  define RP23XX_PM_EN0_USB  RP23XX_CLOCKS_WAKE_EN0_CLK_SYS_PLL_USB
#  define RP23XX_PM_EN1_USB  (RP23XX_CLOCKS_WAKE_EN1_CLK_USB | \
                              RP23XX_CLOCKS_WAKE_EN1_CLK_SYS_USBCTRL)
#else
#  define RP23XX_PM_EN0_USB  0
#  define RP23XX_PM_EN1_USB  0
#endif

#ifdef CONFIG_RP23XX_UART0
#  define RP23XX_PM_EN1_UART0 (RP23XX_CLOCKS_WAKE_EN1_CLK_SYS_UART0 | \
                               RP23XX_CLOCKS_WAKE_EN1_CLK_PERI_UART0)
#else
#  define RP23XX_PM_EN1_UART0 0
#endif

#ifdef CONFIG_RP23XX_UART1
#  define RP23XX_PM_EN1_UART1 (RP23XX_CLOCKS_WAKE_EN1_CLK_SYS_UART1 | \
                               RP23XX_CLOCKS_WAKE_EN1_CLK_PERI_UART1)
#else
#  define RP23XX_PM_EN1_UART1 0
#endif

#ifdef CONFIG_RP23XX_SPI0
#  define RP23XX_PM_EN1_SPI0  (RP23XX_CLOCKS_WAKE_EN1_CLK_SYS_SPI0 | \
                               RP23XX_CLOCKS_WAKE_EN1_CLK_PERI_SPI0)
#else
#  define RP23XX_PM_EN1_SPI0  0
#endif

#ifdef CONFIG_RP23XX_SPI1
#  define RP23XX_PM_EN1_SPI1  (RP23XX_CLOCKS_WAKE_EN1_CLK_SYS_SPI1 | \
                               RP23XX_CLOCKS_WAKE_EN1_CLK_PERI_SPI1)
#else
#  define RP23XX_PM_EN1_SPI1  0
#endif

#ifdef CONFIG_RP23XX_RNG
#  define RP23XX_PM_EN1_TRNG  RP23XX_CLOCKS_WAKE_EN1_CLK_SYS_TRNG
#else
#  define RP23XX_PM_EN1_TRNG  0
#endif

#ifdef CONFIG_RP23XX_TIMER0
#  define RP23XX_PM_EN1_TIMER0 RP23XX_CLOCKS_WAKE_EN1_CLK_SYS_TIMER0
#else
#  define RP23XX_PM_EN1_TIMER0 0
#endif

#ifdef CONFIG_RP23XX_TIMER1
#  define RP23XX_PM_EN1_TIMER1 RP23XX_CLOCKS_WAKE_EN1_CLK_SYS_TIMER1
#else
#  define RP23XX_PM_EN1_TIMER1 0
#endif

/* The timer block that the tickless scheduler uses */

#if defined(CONFIG_RP23XX_SYSTIMER_TICKLESS_TIMER0)
#  define RP23XX_PM_EN1_SYSTIMER RP23XX_CLOCKS_WAKE_EN1_CLK_SYS_TIMER0
#elif defined(CONFIG_RP23XX_SYSTIMER_TICKLESS_TIMER1)
#  define RP23XX_PM_EN1_SYSTIMER RP23XX_CLOCKS_WAKE_EN1_CLK_SYS_TIMER1
#else
#  define RP23XX_PM_EN1_SYSTIMER 0
#endif

#define RP23XX_PM_SLEEP_EN0 \
  (RP23XX_PM_SLEEP_EN0_BASE | RP23XX_PM_EN0_ADC  | RP23XX_PM_EN0_DMA  | \
   RP23XX_PM_EN0_I2C0       | RP23XX_PM_EN0_I2C1 | RP23XX_PM_EN0_PWM  | \
   RP23XX_PM_EN0_OTP        | RP23XX_PM_EN0_SHA  | RP23XX_PM_EN0_PIO  | \
   RP23XX_PM_EN0_USB)

#define RP23XX_PM_SLEEP_EN1 \
  (RP23XX_PM_SLEEP_EN1_BASE  | RP23XX_PM_EN1_USB    | \
   RP23XX_PM_EN1_UART0       | RP23XX_PM_EN1_UART1  | \
   RP23XX_PM_EN1_SPI0        | RP23XX_PM_EN1_SPI1   | \
   RP23XX_PM_EN1_TRNG        | RP23XX_PM_EN1_TIMER0 | \
   RP23XX_PM_EN1_TIMER1      | RP23XX_PM_EN1_SYSTIMER)

#ifdef CONFIG_RP23XX_PM_QUIESCE_PADS
/* Blocks with no driver are held in reset.  A gated clock does not stop
 * the bias current of an analogue block such as the USB PHY.
 */

#ifndef CONFIG_USBDEV
#  define RP23XX_PM_RST_USB   RP23XX_RESETS_RESET_USBCTRL
#else
#  define RP23XX_PM_RST_USB   0
#endif

#ifndef CONFIG_RP23XX_ADC
#  define RP23XX_PM_RST_ADC   RP23XX_RESETS_RESET_ADC
#else
#  define RP23XX_PM_RST_ADC   0
#endif

#ifndef CONFIG_RP23XX_RNG
#  define RP23XX_PM_RST_TRNG  RP23XX_RESETS_RESET_TRNG
#else
#  define RP23XX_PM_RST_TRNG  0
#endif

#ifndef CONFIG_CRYPTO_CRYPTODEV_HARDWARE
#  define RP23XX_PM_RST_SHA   RP23XX_RESETS_RESET_SHA256
#else
#  define RP23XX_PM_RST_SHA   0
#endif

#ifndef CONFIG_RP23XX_PWM
#  define RP23XX_PM_RST_PWM   RP23XX_RESETS_RESET_PWM
#else
#  define RP23XX_PM_RST_PWM   0
#endif

#if !defined(CONFIG_RP23XX_I2S) && !defined(CONFIG_WS2812) && \
    !defined(CONFIG_IEEE80211_INFINEON_CYW43439)
#  define RP23XX_PM_RST_PIO   (RP23XX_RESETS_RESET_PIO0 | \
                               RP23XX_RESETS_RESET_PIO1 | \
                               RP23XX_RESETS_RESET_PIO2)
#else
#  define RP23XX_PM_RST_PIO   0
#endif

#ifndef CONFIG_RP23XX_SPI0
#  define RP23XX_PM_RST_SPI0  RP23XX_RESETS_RESET_SPI0
#else
#  define RP23XX_PM_RST_SPI0  0
#endif

#ifndef CONFIG_RP23XX_SPI1
#  define RP23XX_PM_RST_SPI1  RP23XX_RESETS_RESET_SPI1
#else
#  define RP23XX_PM_RST_SPI1  0
#endif

#ifndef CONFIG_RP23XX_I2C0
#  define RP23XX_PM_RST_I2C0  RP23XX_RESETS_RESET_I2C0
#else
#  define RP23XX_PM_RST_I2C0  0
#endif

#ifndef CONFIG_RP23XX_I2C1
#  define RP23XX_PM_RST_I2C1  RP23XX_RESETS_RESET_I2C1
#else
#  define RP23XX_PM_RST_I2C1  0
#endif

#ifndef CONFIG_RP23XX_UART1
#  define RP23XX_PM_RST_UART1 RP23XX_RESETS_RESET_UART1
#else
#  define RP23XX_PM_RST_UART1 0
#endif

/* NuttX has no HSTX driver */

#define RP23XX_PM_RST_HSTX    RP23XX_RESETS_RESET_HSTX

#define RP23XX_PM_RESET_UNUSED \
  (RP23XX_PM_RST_USB  | RP23XX_PM_RST_ADC  | RP23XX_PM_RST_TRNG | \
   RP23XX_PM_RST_SHA  | RP23XX_PM_RST_PWM  | RP23XX_PM_RST_PIO  | \
   RP23XX_PM_RST_SPI0 | RP23XX_PM_RST_SPI1 | RP23XX_PM_RST_I2C0 | \
   RP23XX_PM_RST_I2C1 | RP23XX_PM_RST_UART1 | RP23XX_PM_RST_HSTX)
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* GPIOs armed in the dormant-wake detector.  With none, no dormant state */

static uint64_t g_pm_wakeup_gpios;

/* Set by the wake interrupt */

static volatile bool g_pm_woken;

#ifdef CONFIG_RP23XX_PM_SUSPEND
/* Their trigger, to arm them again after a suspend to RAM */

static uint64_t g_pm_wakeup_edge;
static uint64_t g_pm_wakeup_high;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void powman_setbits(uint32_t reg, uint32_t bits)
{
  putreg32(POWMAN_PASSWORD | bits, reg + POWMAN_SET_ALIAS);
}

static inline void powman_clrbits(uint32_t reg, uint32_t bits)
{
  putreg32(POWMAN_PASSWORD | bits, reg + POWMAN_CLR_ALIAS);
}

/****************************************************************************
 * Name: rp23xx_pm_wake_irq
 *
 * Description:
 *   One-shot interrupt of a wake GPIO.  The dormant-wake edge stays in the
 *   INTR latch, so this interrupt ends the WFI of the dormant state as soon
 *   as the crystal oscillator runs again.
 *
 ****************************************************************************/

static int rp23xx_pm_wake_irq(int irq, FAR void *context, FAR void *arg)
{
  rp23xx_gpio_disable_irq(irq);
  g_pm_woken = true;
  return OK;
}

/****************************************************************************
 * Name: rp23xx_pm_stayed
 *
 * Description:
 *   True while a wakelock holds a state above PM_SLEEP.  A driver can take
 *   one from an interrupt after the governor chose PM_SLEEP.
 *
 ****************************************************************************/

static bool rp23xx_pm_stayed(void)
{
  int state;

  for (state = PM_NORMAL; state < PM_SLEEP; state++)
    {
      if (pm_staycount(PM_IDLE_DOMAIN, state) > 0)
        {
          return true;
        }
    }

  return false;
}

#ifdef CONFIG_RP23XX_PM_QUIESCE_PADS

/****************************************************************************
 * Name: rp23xx_pm_pads_quiesce
 *
 * Description:
 *   Isolate every unused pad and turn off its input buffer.  A floating
 *   bank 0 input settles near 2.2V (erratum RP2350-E9) and its buffer then
 *   draws a static current in every state.
 *
 ****************************************************************************/

void rp23xx_pm_pads_quiesce(void)
{
  int gpio;

  /* Reset the unused blocks first, so that they stop driving their pads */

  setbits_reg32(RP23XX_PM_RESET_UNUSED, RP23XX_RESETS_RESET);

  for (gpio = 0; gpio < RP23XX_GPIO_NUM; gpio++)
    {
      /* A wake source must keep watching its pad */

      if ((g_pm_wakeup_gpios & (1ull << gpio)) != 0)
        {
          continue;
        }

#ifdef CONFIG_RP23XX_UART0
      if (gpio == CONFIG_RP23XX_UART0_TX_GPIO ||
          gpio == CONFIG_RP23XX_UART0_RX_GPIO)
        {
          continue;
        }
#endif

#ifdef CONFIG_RP23XX_UART1
      if (gpio == CONFIG_RP23XX_UART1_TX_GPIO ||
          gpio == CONFIG_RP23XX_UART1_RX_GPIO)
        {
          continue;
        }
#endif

#ifdef CONFIG_RP23XX_PSRAM
      /* An isolated PSRAM chip select never asserts, and every PSRAM read
       * then returns the same pattern.
       */

      if (gpio == CONFIG_RP23XX_PSRAM_CS1_GPIO)
        {
          continue;
        }
#endif

      modbits_reg32(RP23XX_PADS_BANK0_GPIO_ISO,
                    RP23XX_PADS_BANK0_GPIO_IE |
                    RP23XX_PADS_BANK0_GPIO_ISO,
                    RP23XX_PADS_BANK0_GPIO(gpio));
    }
}
#endif /* CONFIG_RP23XX_PM_QUIESCE_PADS */

/****************************************************************************
 * Name: rp23xx_pm_dormant
 *
 * Description:
 *   Run from the crystal oscillator, stop it, and restore the clock tree
 *   after the wake.  As pico-extras sleep_goto_dormant_until_pin() does.
 *
 ****************************************************************************/

static void RP23XX_PM_RAMFUNC rp23xx_pm_dormant(void)
{
  uint32_t sys_ctrl;
  uint32_t sys_div;
  uint32_t ref_ctrl;
  uint32_t ref_div;

  /* Save the clk_sys and clk_ref setup, to restore it exactly */

  sys_ctrl = getreg32(RP23XX_CLOCKS_CLK_SYS_CTRL);
  sys_div  = getreg32(RP23XX_CLOCKS_CLK_SYS_DIV);
  ref_ctrl = getreg32(RP23XX_CLOCKS_CLK_REF_CTRL);
  ref_div  = getreg32(RP23XX_CLOCKS_CLK_REF_DIV);

  /* Move clk_sys from the system PLL to clk_ref (glitchless).  Leave the
   * divisors: they are 16.16, so writing 1 divides by 65536.
   */

  clrbits_reg32(RP23XX_CLOCKS_CLK_SYS_CTRL_SRC, RP23XX_CLOCKS_CLK_SYS_CTRL);
  while (getreg32(RP23XX_CLOCKS_CLK_SYS_SELECTED) != 1)
    {
    }

  /* Run clk_ref from the crystal oscillator, which the wake restarts */

  modbits_reg32(RP23XX_CLOCKS_CLK_REF_CTRL_SRC_XOSC_CLKSRC,
                RP23XX_CLOCKS_CLK_REF_CTRL_SRC_MASK,
                RP23XX_CLOCKS_CLK_REF_CTRL);
  while (!(getreg32(RP23XX_CLOCKS_CLK_REF_SELECTED) &
           (1u << RP23XX_CLOCKS_CLK_REF_CTRL_SRC_XOSC_CLKSRC)))
    {
    }

  /* Nothing uses the PLLs now, so power them down */

  putreg32(RP23XX_PLL_PWR_PD | RP23XX_PLL_PWR_VCOPD |
           RP23XX_PLL_PWR_POSTDIVPD,
           RP23XX_PLL_SYS_BASE + RP23XX_PLL_PWR_OFFSET);
  putreg32(RP23XX_PLL_PWR_PD | RP23XX_PLL_PWR_VCOPD |
           RP23XX_PLL_PWR_POSTDIVPD,
           RP23XX_PLL_USB_BASE + RP23XX_PLL_PWR_OFFSET);

  /* Arm the dormant request.  The oscillator stops only when the core
   * stops requesting a clock, so enter deep sleep.
   */

  putreg32(RP23XX_XOSC_DORMANT_DORMANT, RP23XX_XOSC_DORMANT);

  putreg32(getreg32(NVIC_SYSCON) | NVIC_SYSCON_SLEEPDEEP, NVIC_SYSCON);

  __asm__ __volatile__ ("dsb" ::: "memory");
  __asm__ __volatile__ ("wfi");
  __asm__ __volatile__ ("isb" ::: "memory");

  putreg32(getreg32(NVIC_SYSCON) & ~NVIC_SYSCON_SLEEPDEEP, NVIC_SYSCON);

  /* Woken: wait for the oscillator to be stable */

  while (!(getreg32(RP23XX_XOSC_STATUS) & RP23XX_XOSC_STATUS_STABLE))
    {
    }

  /* Restart the PLLs with the values of clocks_init() */

  rp23xx_pll_init(RP23XX_PLL_SYS_BASE, 1, 1500 * MHZ, 5, 2);
  rp23xx_pll_init(RP23XX_PLL_USB_BASE, 1, 1200 * MHZ, 5, 5);

  /* Restore clk_ref first, because clk_sys still runs from it */

  modbits_reg32(ref_ctrl, RP23XX_CLOCKS_CLK_REF_CTRL_SRC_MASK,
                RP23XX_CLOCKS_CLK_REF_CTRL);
  while (!(getreg32(RP23XX_CLOCKS_CLK_REF_SELECTED) &
           (1u << (ref_ctrl & RP23XX_CLOCKS_CLK_REF_CTRL_SRC_MASK))))
    {
    }

  putreg32(ref_div, RP23XX_CLOCKS_CLK_REF_DIV);

  putreg32(sys_ctrl, RP23XX_CLOCKS_CLK_SYS_CTRL);
  while (!(getreg32(RP23XX_CLOCKS_CLK_SYS_SELECTED) &
           (1u << (sys_ctrl & RP23XX_CLOCKS_CLK_SYS_CTRL_SRC))))
    {
    }

  putreg32(sys_div, RP23XX_CLOCKS_CLK_SYS_DIV);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp23xx_pm_standby
 ****************************************************************************/

void rp23xx_pm_standby(void)
{
  uint32_t saved_en0;

  uint32_t saved_en1;

  /* SLEEP_EN0/1 take effect only while the processors sleep */

  saved_en0 = getreg32(RP23XX_CLOCKS_SLEEP_EN0);
  saved_en1 = getreg32(RP23XX_CLOCKS_SLEEP_EN1);

  putreg32(RP23XX_PM_SLEEP_EN0, RP23XX_CLOCKS_SLEEP_EN0);
  putreg32(RP23XX_PM_SLEEP_EN1, RP23XX_CLOCKS_SLEEP_EN1);

  /* A plain WFI: any enabled interrupt wakes the core */

  putreg32(getreg32(NVIC_SYSCON) & ~NVIC_SYSCON_SLEEPDEEP, NVIC_SYSCON);

  __asm__ __volatile__ ("dsb" ::: "memory");
  __asm__ __volatile__ ("wfi");
  __asm__ __volatile__ ("isb" ::: "memory");

  putreg32(saved_en0, RP23XX_CLOCKS_SLEEP_EN0);
  putreg32(saved_en1, RP23XX_CLOCKS_SLEEP_EN1);
}

/****************************************************************************
 * Name: rp23xx_pm_sleep
 ****************************************************************************/

void rp23xx_pm_sleep(void)
{
  int gpio;

  if (rp23xx_pm_stayed())
    {
      rp23xx_pm_standby();
      return;
    }

  /* Only the GPIO dormant-wake detector can restart the oscillator; the
   * always-on timer alarm cannot.  With no GPIO armed, use standby.
   */

  if (g_pm_wakeup_gpios == 0)
    {
      rp23xx_pm_standby();
      return;
    }

  /* Clear the edges latched by the last wake, or it ends this one at once,
   * and enable the wake interrupts.
   */

  g_pm_woken = false;

  for (gpio = 0; gpio < RP23XX_GPIO_NUM; gpio++)
    {
      if ((g_pm_wakeup_gpios & (1ull << gpio)) != 0)
        {
          setbits_reg32(0xfu << ((gpio % 8) * 4),
                        RP23XX_IO_BANK0_INTR(gpio));
          rp23xx_gpio_enable_irq(gpio);
        }
    }

  /* An edge came in meanwhile: the interrupt is already gone */

  if (!g_pm_woken)
    {
      rp23xx_pm_dormant();
    }

  for (gpio = 0; gpio < RP23XX_GPIO_NUM; gpio++)
    {
      if ((g_pm_wakeup_gpios & (1ull << gpio)) != 0)
        {
          rp23xx_gpio_disable_irq(gpio);
        }
    }

  /* Let the input that woke the chip follow */

  pm_staytimeout(PM_IDLE_DOMAIN, PM_STANDBY, CONFIG_RP23XX_PM_WAKE_HOLD_MS);

#ifdef CONFIG_RTC
  /* The system tick stopped while dormant.  The always-on timer did not,
   * so take the time of day back from it.
   */

  clock_synchronize(NULL);
#endif
}

/****************************************************************************
 * Name: rp23xx_pm_gpio_wakeup
 ****************************************************************************/

int rp23xx_pm_gpio_wakeup(int gpio, bool edge, bool high)
{
  uint32_t bit;

  if (gpio < 0 || gpio >= RP23XX_GPIO_NUM)
    {
      return -EINVAL;
    }

  /* Enable the input and clear the isolation that pads have from reset */

  modbits_reg32(RP23XX_PADS_BANK0_GPIO_IE,
                RP23XX_PADS_BANK0_GPIO_ISO |
                RP23XX_PADS_BANK0_GPIO_IE |
                RP23XX_PADS_BANK0_GPIO_OD,
                RP23XX_PADS_BANK0_GPIO(gpio));

  /* Pull the pad away from the wake level.  A floating input settles
   * near 2.2V (erratum RP2350-E9) and would wake the chip at once.
   */

  rp23xx_gpio_set_pulls(gpio, !high, high);

  if (edge)
    {
      bit = high ? RP23XX_IO_BANK0_INTR_GPIO_EDGE_HIGH(gpio)
                 : RP23XX_IO_BANK0_INTR_GPIO_EDGE_LOW(gpio);
    }
  else
    {
      bit = high ? RP23XX_IO_BANK0_INTR_GPIO_LEVEL_HIGH(gpio)
                 : RP23XX_IO_BANK0_INTR_GPIO_LEVEL_LOW(gpio);
    }

  /* Clear old latched edges */

  setbits_reg32(0xfu << ((gpio % 8) * 4), RP23XX_IO_BANK0_INTR(gpio));

  setbits_reg32(bit, RP23XX_IO_BANK0_DORMANT_WAKE_INTE(gpio));

  /* The same trigger as an interrupt, enabled only around the dormant
   * state.  It replaces any other handler of this GPIO.
   */

  rp23xx_gpio_irq_attach(gpio, edge ? (high ? RP23XX_GPIO_INTR_EDGE_HIGH :
                                              RP23XX_GPIO_INTR_EDGE_LOW) :
                                      (high ? RP23XX_GPIO_INTR_LEVEL_HIGH :
                                              RP23XX_GPIO_INTR_LEVEL_LOW),
                         rp23xx_pm_wake_irq, NULL);

  g_pm_wakeup_gpios |= 1ull << gpio;

#ifdef CONFIG_RP23XX_PM_SUSPEND
  g_pm_wakeup_edge = edge ? g_pm_wakeup_edge | (1ull << gpio) :
                            g_pm_wakeup_edge & ~(1ull << gpio);
  g_pm_wakeup_high = high ? g_pm_wakeup_high | (1ull << gpio) :
                            g_pm_wakeup_high & ~(1ull << gpio);
#endif

  return OK;
}

/****************************************************************************
 * Name: rp23xx_pm_gpio_wakeup_disable
 ****************************************************************************/

int rp23xx_pm_gpio_wakeup_disable(int gpio)
{
  if (gpio < 0 || gpio >= RP23XX_GPIO_NUM)
    {
      return -EINVAL;
    }

  clrbits_reg32(0xfu << ((gpio % 8) * 4),
                RP23XX_IO_BANK0_DORMANT_WAKE_INTE(gpio));

  g_pm_wakeup_gpios &= ~(1ull << gpio);
  return OK;
}

#ifdef CONFIG_RP23XX_PM_SUSPEND
/****************************************************************************
 * Name: rp23xx_pm_gpio_wakeup_restore
 ****************************************************************************/

void rp23xx_pm_gpio_wakeup_restore(void)
{
  int gpio;

  for (gpio = 0; gpio < RP23XX_GPIO_NUM; gpio++)
    {
      if ((g_pm_wakeup_gpios & (1ull << gpio)) != 0)
        {
          rp23xx_pm_gpio_wakeup(gpio, (g_pm_wakeup_edge >> gpio) & 1,
                                (g_pm_wakeup_high >> gpio) & 1);
        }
    }
}
#endif
