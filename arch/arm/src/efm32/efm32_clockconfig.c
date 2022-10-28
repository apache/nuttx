/****************************************************************************
 * arch/arm/src/efm32/efm32_clockconfig.c
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

#include <arch/board/board.h>

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

#include "arm_internal.h"
#include "chip.h"
#include "itm_syslog.h"
#include "efm32_gpio.h"
#include "hardware/efm32_msc.h"
#include "hardware/efm32_cmu.h"
#include "hardware/efm32_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* BOARD Configuration ******************************************************/

/* Pre-scalers not currently implemented */

#if defined(CONFIG_EFM32_EFM32GG) && defined(BOARD_HFCLKDIV) && BOARD_HFCLKDIV != 0
#  error HFCLK divisor not yet supported
#endif

#if defined(BOARD_HFCORECLKDIV) && BOARD_HFCORECLKDIV != 0
#  error HFCORECLK divisor not yet supported
#endif

#if defined(BOARD_HFPERCLKDIV) && BOARD_HFPERCLKDIV != 0
#  error HFPERCLK divisor not yet supported
#endif

#ifdef BOARD_LFACLK_ULFRCO
#  define BOARD_LFA_ULFCO_ENABLE true
#else
#  define BOARD_LFA_ULFCO_ENABLE false
#endif

#ifdef BOARD_LFBCLK_ULFRCO
#  define BOARD_LFB_ULFCO_ENABLE true
#else
#  define BOARD_LFB_ULFCO_ENABLE false
#endif

#ifndef CONFIG_EFM32_LECLOCK
#   if ( ( BOARD_LFACLKSEL       != _CMU_LFCLKSEL_LFA_DISABLED   ) || \
         ( BOARD_LFBCLKSEL       != _CMU_LFCLKSEL_LFB_DISABLED   ) || \
         ( defined ( CONFIG_EFM32_RTC_BURTC )                    ) || \
         ( defined ( CONFIG_EFM32_LEUART0   )                    ) || \
         ( defined ( CONFIG_EFM32_LEUART1   )                    )    \
       )
#       define CONFIG_EFM32_LECLOCK
#   endif
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: efm32_statuswait
 *
 * Description:
 *   Wait for ongoing CMU status bit(s) to become set
 *
 * Input Parameters:
 *   bitset - Bitset corresponding to STATUS register defined bits,
 *            indicating events that we are waiting for.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void efm32_statuswait(uint32_t bitset)
{
  /* Wait for clock to stabilize if requested */

  while ((getreg32(EFM32_CMU_STATUS) & bitset) == 0);
}

/****************************************************************************
 * Name: efm32_enable_XXX
 *
 * Description:
 *   Enable specific oscillators
 *
 ****************************************************************************/

static void efm32_enable_lfrco(void)
{
  /* Enable the LFRCO */

  putreg32(CMU_OSCENCMD_LFRCOEN, EFM32_CMU_OSCENCMD);
  efm32_statuswait(CMU_STATUS_LFRCORDY);
}

static void efm32_enable_lfxo(void)
{
  /* Enable the LFXO */

  putreg32(CMU_OSCENCMD_LFXOEN, EFM32_CMU_OSCENCMD);
  efm32_statuswait(CMU_STATUS_LFXORDY);
}

static inline void efm32_enable_hfrco(void)
{
  /* Enable the HFRCO */

  putreg32(CMU_OSCENCMD_HFRCOEN, EFM32_CMU_OSCENCMD);
  efm32_statuswait(CMU_STATUS_HFRCORDY);
}

static void efm32_enable_hfxo(void)
{
  /* Enable the HFXO */

  putreg32(CMU_OSCENCMD_HFXOEN, EFM32_CMU_OSCENCMD);
  efm32_statuswait(CMU_STATUS_HFXORDY);
}

#ifdef CONFIG_ARMV7M_ITMSYSLOG
static inline void efm32_enable_auxhfrco(void)
{
  /* Enable the HFXO */

  putreg32(CMU_OSCENCMD_AUXHFRCOEN, EFM32_CMU_OSCENCMD);
  efm32_statuswait(CMU_STATUS_AUXHFRCORDY);
}
#endif

/****************************************************************************
 * Name: efm32_enable_leclocking
 *
 * Description:
 *   Enable HFCORE clocking to the LE
 *
 ****************************************************************************/

static void efm32_enable_leclocking(void)
{
  uint32_t regval;

  regval  = getreg32(EFM32_CMU_HFCORECLKEN0);
  regval |= CMU_HFCORECLKEN0_LE;
  putreg32(regval, EFM32_CMU_HFCORECLKEN0);
}

/****************************************************************************
 * Name: efm32_maxwaitstates
 *
 * Description:
 *   Configure flash access wait states to most maximum number of wait
 *   states, preserving the SCBTP setting.
 *
 ****************************************************************************/

static void efm32_maxwaitstates(void)
{
  uint32_t regval;
  uint32_t mode;

  /* Get the READCTRL register content and mask out the mode setting */

  regval  = getreg32(EFM32_MSC_READCTRL);
  mode    = regval & _MSC_READCTRL_MODE_MASK;
  regval &= ~_MSC_READCTRL_MODE_MASK;

  /* SCBTP mode? */

  if (mode == MSC_READCTRL_MODE_WS0SCBTP
     || mode == MSC_READCTRL_MODE_WS1SCBTP
#ifdef MSC_READCTRL_MODE_WS2SCBTP
     || mode == MSC_READCTRL_MODE_WS2SCBTP
#endif
     )
    {
      /* Yes.. select the mximum number of wait states with SCBTP */

      regval |= MSC_READCTRL_MODE_WSMAXSCBTP;
    }
  else
    {
      /* No.. select the mximum number of wait states without SCBTP */

      regval |= MSC_READCTRL_MODE_WSMAX;
    }

  /* And save the update READCTRL register */

  putreg32(regval, EFM32_MSC_READCTRL);
}

/****************************************************************************
 * Name: efm32_setwaitstates
 *
 * Description:
 *   Configure the optimal number of flash access wait states, preserving
 *   the SCBTP setting.
 *
 ****************************************************************************/

static void efm32_setwaitstates(uint32_t hfcoreclk)
{
  uint32_t regval;
  uint32_t mode;
  bool scbtp;

  /* SCBTP mode? */

  regval  = getreg32(EFM32_MSC_READCTRL);
  mode    = regval & _MSC_READCTRL_MODE_MASK;
  scbtp = (mode == MSC_READCTRL_MODE_WS0SCBTP
           || mode == MSC_READCTRL_MODE_WS1SCBTP
#ifdef MSC_READCTRL_MODE_WS2SCBTP
           || mode == MSC_READCTRL_MODE_WS2SCBTP
#endif
           );

  /* Select the number of wait states based on the HFCORECLK frequency */

  regval &= ~_MSC_READCTRL_MODE_MASK;

  /* We can't do more than 2 wait states in any configuration */

#ifdef MSC_READCTRL_MODE_WS2
  if (hfcoreclk > CMU_MAX_FREQ_2WS)
    {
      DEBUGPANIC();
    }
  else
#endif

  /* Check if we can use 2 wait states */

  if (hfcoreclk > CMU_MAX_FREQ_1WS)
    {
#ifdef MSC_READCTRL_MODE_WS2
      /* Yes.. select 2 wait states */

      regval |= (scbtp ? MSC_READCTRL_MODE_WS2SCBTP : MSC_READCTRL_MODE_WS2);
#else
      /* No.. this MCU does not support 2 wait states */

      DEBUGPANIC();
#endif
    }

  /* Check if we can use 1 wait states */

  else if (hfcoreclk > CMU_MAX_FREQ_0WS)
    {
      /* Yes.. select 1 wait state */

      regval |= (scbtp ? MSC_READCTRL_MODE_WS1SCBTP : MSC_READCTRL_MODE_WS1);
    }

  /* Check if we can use no wait states */

  else
    {
      /* Select no wait states */

      regval |= (scbtp ? MSC_READCTRL_MODE_WS0SCBTP : MSC_READCTRL_MODE_WS0);
    }

  /* And save the update READCTRL register */

  putreg32(regval, EFM32_MSC_READCTRL);
}

/****************************************************************************
 * Name: efm32_hfclk_config
 *
 * Description:
 *   Configure the High Frequency Clock, HFCLK.
 *
 *   HFCLK is the selected High Frequency Clock. This clock is used by the
 *   CMU and drives the two prescalers that generate HFCORECLK and HFPERCLK.
 *   The HFCLK can be driven by a high-frequency oscillator (HFRCO or HFXO)
 *   or one of the low-frequency oscillators (LFRCO or LFXO). By default the
 *   HFRCO is selected. To change the selected HFCLK write to HFCLKSEL in
 *   CMU_CMD. The HFCLK is running in EM0 and EM1.
 *
 *   HFCLK can optionally be divided down by setting HFCLKDIV in CMU_CTRL to
 *   a non-zero value. This divides down HFCLK to all high frequency
 *   components except the USB Core and is typically used to save energy in
 *   USB applications where the system is not required to run at 48 MHz.
 *   Combined with the HFCORECLK and HFPERCLK prescalers the HFCLK divider
 *   also allows for more flexible clock division.
 *
 ****************************************************************************/

static inline uint32_t efm32_hfclk_config(uint32_t hfclksel,
                                          uint32_t hfclkdiv)
{
  uint32_t frequency;
#ifdef CMU_CTRL_HFLE
  uint32_t regval;
#endif

  /* The HFRCO oscillator is selected by hardware as the clock source for
   * HFCLK when the device starts up . After reset, the HFRCO frequency is
   * 14 MHz.
   *
   * First enable the oscillator and wait for the oscillator to become ready
   * before switching the clock source. This way, the system continues to run
   * on the HFRCO until the oscillator has timed out and provides a reliable
   * clock.
   */

    switch (hfclksel)
    {
    case _CMU_CMD_HFCLKSEL_LFRCO:
      {
        frequency = BOARD_LFRCO_FREQUENCY;
        efm32_enable_lfrco();
      }
      break;

    case _CMU_CMD_HFCLKSEL_LFXO:
      {
        frequency = BOARD_LFXO_FREQUENCY;
        efm32_enable_lfxo();
      }
      break;

    case _CMU_CMD_HFCLKSEL_HFRCO:
      {
        frequency = BOARD_HFRCO_FREQUENCY;
        efm32_enable_hfrco();
      }
      break;

    case _CMU_CMD_HFCLKSEL_HFXO:
      {
        frequency = BOARD_HFXO_FREQUENCY;

#ifdef CMU_CTRL_HFLE
#if BOARD_HFXO_FREQUENCY > CMU_MAX_FREQ_HFLE
        /* Adjust HFXO buffer current for high crystal frequencies,
         * enable HFLE for frequencies above CMU_MAX_FREQ_HFLE.
         *
         * We must also have HFLE enabled to access some LE peripherals
         * >= 32MHz.
         */

        regval = getreg32(EFM32_CMU_CTRL);
        regval &= ~_CMU_CTRL_HFXOBUFCUR_MASK;
        regval |= CMU_CTRL_HFXOBUFCUR_BOOSTABOVE32MHZ | CMU_CTRL_HFLE;
        putreg32(regval, EFM32_CMU_CTRL);

        /* Set DIV4 factor for peripheral clock if HFCORE clock for LE is
         * enabled.
         */

        if ((getreg32(EFM32_CMU_HFCORECLKEN0) & CMU_HFCORECLKEN0_LE) != 0)
          {
            regval  = getreg32(EFM32_CMU_HFCORECLKDIV);
            regval |= CMU_HFCORECLKDIV_HFCORECLKLEDIV_DIV4;
            putreg32(regval, EFM32_CMU_HFCORECLKDIV);
          }
#else
        /* No boost... no HFLE */

        regval = getreg32(EFM32_CMU_CTRL);
        regval &= ~(_CMU_CTRL_HFXOBUFCUR_MASK | CMU_CTRL_HFLE);
        regval |= CMU_CTRL_HFXOBUFCUR_BOOSTUPTO32MHZ;
        putreg32(regval, EFM32_CMU_CTRL);
#endif
#endif
        /* Enable the HFXO */

        efm32_enable_hfxo();
      }
      break;

#ifdef CONFIG_DEBUG_FEATURES
      default:
        DEBUGPANIC();
#endif
    }

   /* Set the maximum  number of FLASH wait states before selecting the new
    * HFCLK source.
    */

  efm32_maxwaitstates();

  /* Switch to selected oscillator */

  putreg32(hfclksel << _CMU_CMD_HFCLKSEL_SHIFT, EFM32_CMU_CMD);

  /* Now select the optimal number of FLASH wait states */

  efm32_setwaitstates(frequency);
  return frequency;
}

/****************************************************************************
 * Name: efm32_hfcoreclk_config
 *
 * Description:
 *   Configure the High Frequency Core Clock, HFCORECLK.
 *
 *   HFCORECLK is a prescaled version of HFCLK. This clock drives the Core
 *   Modules, which consists of the CPU and modules that are tightly coupled
 *   to the CPU, e.g. MSC, DMA etc. This also includes the interface to the
 *   Low Energy Peripherals. Some of the modules that are driven by this
 *   clock can be clock gated completely when not in use. This is done by
 *   clearing the clock enable bit for the specific module in
 *   CMU_HFCORECLKEN0. The frequency of HFCORECLK is set using the
 *   CMU_HFCORECLKDIV register. The setting can be changed dynamically and
 *   the new setting takes effect immediately.
 *
 *   The USB Core clock (USBC) is always undivided regardless of the
 *   HFCLKDIV setting. When the USB Core is active this clock must be
 *   switched to a 32 kHz clock (LFRCO or LFXO) when entering EM2. The USB
 *   Core uses this clock for monitoring the USB bus. The switch is done by
 *   writing USBCCLKSEL in CMU_CMD. The currently active clock can be
 *   checked by reading CMU_STATUS.  The clock switch can take up to 1.5 32
 *   kHz cycle (45 us). To avoid polling the clock selection status when
 *   switching switching from 32 kHz to HFCLK when coming up from EM2 the
 *   USBCHFCLKSEL interrupt can be used. EM3 is not supported when the USB
 *   is active.
 *
 ****************************************************************************/

#ifdef CONFIG_EFM32_LECLOCK
uint32_t efm32_coreleclk_config(int frequency)
{
#ifdef CMU_CTRL_HFLE
  uint32_t regval;

  /* Check if the core frequency is higher than CMU_MAX_FREQ_HFLE */

  if (frequency > CMU_MAX_FREQ_HFLE)
    {
      /* Enable HFLE */

      regval = getreg32(EFM32_CMU_CTRL);
      regval |= CMU_CTRL_HFLE;
      putreg32(regval, EFM32_CMU_CTRL);

      /* Enable DIV4 factor for peripheral clock */

      regval  = getreg32(EFM32_CMU_HFCORECLKDIV);
      regval |= CMU_HFCORECLKDIV_HFCORECLKLEDIV_DIV4;
      putreg32(regval, EFM32_CMU_HFCORECLKDIV);
      frequency /= 4;
    }
#else
  frequency /= 2;
#endif

  /* Enable core clocking to the LE */

  efm32_enable_leclocking();

  return frequency;
}
#else
#   define efm32_coreleclk_config 0
#endif

/****************************************************************************
 * Name: efm32_hfcoreclk_config
 *
 * Description:
 *   Configure the High Frequency Core Clock, HFCORECLK.
 *
 *   HFCORECLK is a prescaled version of HFCLK. This clock drives the Core
 *   Modules, which consists of the CPU and modules that are tightly coupled
 *   to the CPU, e.g. MSC, DMA etc. This also includes the interface to the
 *   Low Energy Peripherals. Some of the modules that are driven by this
 *   clock can be clock gated completely when not in use. This is done by
 *   clearing the clock enable bit for the specific module in
 *   CMU_HFCORECLKEN0. The frequency of HFCORECLK is set using the
 *   CMU_HFCORECLKDIV register. The setting can be changed dynamically and
 *   the new setting takes effect immediately.
 *
 *   The USB Core clock (USBC) is always undivided regardless of the
 *   HFCLKDIV setting. When the USB Core is active this clock must be
 *   switched to a 32 kHz clock (LFRCO or LFXO) when entering EM2. The USB
 *   Core uses this clock for monitoring the USB bus. The switch is done by
 *   writing USBCCLKSEL in CMU_CMD. The currently active clock can be
 *   checked by reading CMU_STATUS.  The clock switch can take up to 1.5 32
 *   kHz cycle (45 us). To avoid polling the clock selection status when
 *   switching switching from 32 kHz to HFCLK when coming up from EM2 the
 *   USBCHFCLKSEL interrupt can be used. EM3 is not supported when the USB
 *   is active.
 *
 ****************************************************************************/

static inline uint32_t efm32_hfcoreclk_config(uint32_t hfcoreclkdiv,
                                              uint32_t hfclk)
{
  /* REVISIT:  Divider not currently used */

  return hfclk;
}

/****************************************************************************
 * Name: efm32_hfperclk_config
 *
 * Description:
 *   Configure the High Frequency Peripheral Clock, HFPERCLK.
 *
 *   Like HFCORECLK, HFPERCLK can also be a prescaled version of HFCLK. This
 *   clock drives the High-Frequency Peripherals. All the peripherals that
 *   are driven by this clock can be clock gated completely when not in use.
 *   This is done by clearing the clock enable bit for the specific
 *   peripheral in CMU_HFPERCLKEN0. The frequency of HFPERCLK is set using
 *   the CMU_HFPERCLKDIV register. The setting can be changed dynamically
 *   and the new setting takes effect immediately.
 *
 ****************************************************************************/

static inline uint32_t efm32_hfperclk_config(uint32_t hfperclkdiv,
                                             uint32_t hfclk)
{
  uint32_t regval;
  unsigned int divider;

  DEBUGASSERT(hfperclkdiv <= _CMU_HFPERCLKDIV_HFPERCLKDIV_HFCLK512);

  /* Set the divider and enable the HFPERCLK */

  regval = (hfperclkdiv << _CMU_HFPERCLKDIV_HFPERCLKDIV_SHIFT) |
           CMU_HFPERCLKDIV_HFPERCLKEN;
  putreg32(regval, EFM32_CMU_HFPERCLKDIV);

  /* The value of hfperclkdiv is log2 of the arithmetic divisor:
   * 0->1, 1->2, 2->4, 3->8, ... 9->512.
   */

  divider = 1 << hfperclkdiv;
  return hfclk / divider;
}

/****************************************************************************
 * Name: efm32_lfaclk_config
 *
 * Description:
 *   Configure the Low Frequency A Clock, LFACLK.
 *
 *   LFACLK is the selected clock for the Low Energy A Peripherals. There
 *   are four selectable sources for LFACLK: LFRCO, LFXO, HFCORECLK/2 and
 *   ULFRCO.  In addition, the LFACLK can be disabled. From reset, the
 *   LFACLK source is set to LFRCO. However, note that the LFRCO is disabled
 *   from reset. The selection is configured using the LFA field in
 *   CMU_LFCLKSEL. The HFCORECLK/2 setting allows the Low Energy A
 *   Peripherals to be used as high-frequency peripherals.
 *
 *   Each Low Energy Peripheral that is clocked by LFACLK has its own
 *   prescaler setting and enable bit. The prescaler settings are configured
 *   using CMU_LFAPRESC0 and the clock enable bits can be found in
 *   CMU_LFACLKEN0. Notice that the LCD has an additional high resolution
 *   prescaler for Frame Rate Control, configured by FDIV in CMU_LCDCTRL.
 *   When operating in oversampling mode, the pulse counters are clocked by
 *   LFACLK. This is configured for each pulse counter (n) individually by
 *   setting PCNTnCLKSEL in CMU_PCNTCTRL.
 *
 ****************************************************************************/

static inline uint32_t efm32_lfaclk_config(uint32_t lfaclksel, bool ulfrco,
                                           uint32_t hfcoreclk)
{
  uint32_t lfaclk;
  uint32_t regval;

  /* ULFRCO is a special case */

  if (ulfrco)
    {
      /* ULFRCO is always enabled */

      lfaclksel = _CMU_LFCLKSEL_LFA_DISABLED;
      lfaclk    = BOARD_ULFRCO_FREQUNCY;
    }
  else
    {
      /* Enable the oscillator source */

      switch (lfaclksel)
        {
          default:
          case _CMU_LFCLKSEL_LFA_DISABLED:
            {
              lfaclk = 0;
            }
            break;

          case CMU_LFCLKSEL_LFA_LFRCO:
            {
              efm32_enable_lfrco();
              lfaclk = BOARD_LFRCO_FREQUENCY;
            }
            break;

          case _CMU_LFCLKSEL_LFA_LFXO:
            {
              efm32_enable_lfxo();
              lfaclk = BOARD_LFXO_FREQUENCY;
            }
            break;

          case _CMU_LFCLKSEL_LFA_HFCORECLKLEDIV2:
            {
              lfaclk = hfcoreclk >> 1;
            }
            break;
        }
    }

  /* Enable the LFA clock in the LFCLKSEL register */

  regval  = getreg32(EFM32_CMU_LFCLKSEL);

#ifdef CMU_LFCLKSEL_LFAE
  regval &= ~_CMU_LFCLKSEL_LFAE_MASK;
#endif

  regval &= ~_CMU_LFCLKSEL_LFA_MASK;
  regval |= (lfaclksel << _CMU_LFCLKSEL_LFA_SHIFT);

#ifdef CMU_LFCLKSEL_LFAE_ULFRCO
  regval |= ((uint32_t)ulfrco << _CMU_LFCLKSEL_LFAE_SHIFT);
#endif

  putreg32(regval, EFM32_CMU_LFCLKSEL);

  return lfaclk;
}

/****************************************************************************
 * Name: efm32_lfbclk_config
 *
 * Description:
 *   Configure the Low Frequency B Clock, LFBCLK.
 *
 *   LFBCLK is the selected clock for the Low Energy B Peripherals. There
 *   are four selectable sources for LFBCLK: LFRCO, LFXO, HFCORECLK/2 and
 *   ULFRCO. In addition, the LFBCLK can be disabled. From reset, the LFBCLK
 *   source is set to LFRCO. However, note that the LFRCO is disabled from
 *   reset. The selection is configured using the LFB field in CMU_LFCLKSEL.
 *   The HFCORECLK/2 setting allows the Low Energy B Peripherals to be used
 *   as high-frequency peripherals.
 *
 *   Each Low Energy Peripheral that is clocked by LFBCLK has its own
 *   prescaler setting and enable bit. The prescaler settings are
 *   configured using CMU_LFBPRESC0 and the clock enable bits can be found
 *   in CMU_LFBCLKEN0.
 *
 ****************************************************************************/

static inline uint32_t efm32_lfbclk_config(uint32_t lfbclksel, bool ulfrco,
                                           uint32_t hfcoreclk)
{
  uint32_t lfbclk;
  uint32_t regval;

  /* ULFRCO is a special case */

  if (ulfrco)
    {
      /* ULFRCO is always enabled */

      lfbclksel = _CMU_LFCLKSEL_LFB_DISABLED;
      lfbclk    = BOARD_ULFRCO_FREQUNCY;
    }
  else
    {
      /* Enable the oscillator source */

      switch (lfbclksel)
        {
          default:
          case _CMU_LFCLKSEL_LFB_DISABLED:
            {
              lfbclk = 0;
            }
            break;

          case CMU_LFCLKSEL_LFB_LFRCO:
            {
              efm32_enable_lfrco();
              lfbclk = 0;
            }
            break;

          case _CMU_LFCLKSEL_LFB_LFXO:
            {
              efm32_enable_lfxo();
              lfbclk = BOARD_LFXO_FREQUENCY;
            }
            break;

          case _CMU_LFCLKSEL_LFB_HFCORECLKLEDIV2:
            {
              lfbclk = hfcoreclk >> 1;
            }
            break;
        }
    }

  /* Enable the LFB clock in the LFCLKSEL register */

  regval  = getreg32(EFM32_CMU_LFCLKSEL);

#ifdef CMU_LFCLKSEL_LFBE
  regval &= ~_CMU_LFCLKSEL_LFBE_MASK;
#endif

  regval &= ~_CMU_LFCLKSEL_LFB_MASK;
  regval |= (lfbclksel << _CMU_LFCLKSEL_LFB_SHIFT);

#ifdef CMU_LFCLKSEL_LFBE_ULFRCO
  regval |= ((uint32_t)ulfrco << _CMU_LFCLKSEL_LFBE_SHIFT);
#endif

  putreg32(regval, EFM32_CMU_LFCLKSEL);

  return lfbclk;
}

/****************************************************************************
 * Name: efm32_pcntclk_config
 *
 * Description:
 *  Configure the Pulse Counter n Clock, PCNTnCLK.
 *
 *  Each available pulse counter is driven by its own clock, PCNTnCLK where
 *  n is the pulse counter instance number. Each pulse counter can be
 *  configured to use an external pin (PCNTn_S0) or LFACLK as PCNTnCLK.
 *
 ****************************************************************************/

static inline void efm32_pcntclk_config(void)
{
  /* REVISIT: Not yet implemented */
}

/****************************************************************************
 * Name: efm32_wdogclk_config
 *
 * Description:
 *   Configure the Watchdog Timer Clock, WDOGCLK.
 *
 *   The Watchdog Timer (WDOG) can be configured to use one of three
 *   different clock sources: LFRCO, LFXO or ULFRCO. ULFRCO (Ultra Low
 *   Frequency RC Oscillator) is a separate 1 kHz RC oscillator that also
 *   runs in EM3.
 *
 ****************************************************************************/

static inline void efm32_wdogclk_config(void)
{
  /* REVISIT: Not yet implemented */
}

/****************************************************************************
 * Name: efm32_auxclk_config
 *
 * Description:
 *   Configure the Auxiliary Clock, AUXCLK.
 *
 *   AUXCLK is a 1-28 MHz clock driven by a separate RC oscillator, AUXHFRCO.
 *   This clock is used for flash programming, and Serial Wire Output (SWO),
 *   and LESENSE operation. During flash programming, or if needed by
 *   LESENSE, this clock will be active. If the AUXHFRCO has not been
 *   enabled explicitly by software, the MSC or LESENSE module will
 *   automatically start and stop it. The AUXHFRCO is enabled by writing a 1
 *   to AUXHFRCOEN in CMU_OSCENCMD. This explicit enabling is required when
 *   SWO is used.
 *
 ****************************************************************************/

static inline void efm32_auxclk_config(void)
{
  /* REVISIT: Not yet implemented */
}

/****************************************************************************
 * Name: efm32_gpioclock
 *
 * Description:
 *   Enable clocking to the GPIO
 *
 ****************************************************************************/

static inline void efm32_gpioclock(void)
{
  uint32_t regval;

  /* Enable clocking to the GPIO be setting the GPIO bit in the High
   * Frequency Peripheral Clock Enable.
   */

  regval = getreg32(EFM32_CMU_HFPERCLKEN0);
  regval |= CMU_HFPERCLKEN0_GPIO;
  putreg32(regval, EFM32_CMU_HFPERCLKEN0);
}

/****************************************************************************
 * Name: efm32_itm_syslog
 *
 * Description:
 *   Enable Serial wire output pin, configure debug clocking, and enable
 *   ITM syslog support.
 *
 ****************************************************************************/

#ifdef CONFIG_ARMV7M_ITMSYSLOG
static inline void efm32_itm_syslog(void)
{
  int regval;

  /* Enable Serial wire output pin
   *
   * Set location and enable output on the pin.  All pin configuration
   * information must be provided in the board.h header file.
   */

  regval  = getreg32(EFM32_GPIO_ROUTE);
  regval &= ~_GPIO_ROUTE_SWLOCATION_MASK;
  regval |= GPIO_ROUTE_SWOPEN;
  regval |= ((uint32_t)BOARD_SWOPORT_LOCATION <<
             _GPIO_ROUTE_SWLOCATION_SHIFT);
  putreg32(regval, EFM32_GPIO_ROUTE);

  /* Enable output on pin */

  efm32_configgpio(BOARD_GPIO_SWOPORT);

  /* Enable debug clock AUXHFRCO */

  efm32_enable_auxhfrco();
}
#else
#  define efm32_itm_syslog()
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: efm32_clockconfig
 *
 * Description:
 *   Called to initialize the EFM32 chip.  This does whatever setup is
 *   needed to put the  MCU in a usable state.  This includes the
 *   initialization of clocking using the settings in board.h.
 *
 ****************************************************************************/

void efm32_clockconfig(void)
{
  uint32_t hfclk;
  uint32_t hfcoreclk;
  uint32_t hfperclk;
  uint32_t coreleclk;
  uint32_t lfaclk;
  uint32_t lfbclk;

  /* Enable clocks and set dividers as determined by the board.h header
   * file
   */

  hfclk     = efm32_hfclk_config(BOARD_HFCLKSEL, BOARD_HFCLKDIV);
  hfcoreclk = efm32_hfcoreclk_config(BOARD_HFCORECLKDIV, hfclk);
  hfperclk  = efm32_hfperclk_config(BOARD_HFPERCLKDIV, hfclk);
  coreleclk = efm32_coreleclk_config(hfclk);
  lfaclk    = efm32_lfaclk_config(BOARD_LFACLKSEL,
                                  BOARD_LFA_ULFCO_ENABLE, hfcoreclk);
  lfbclk    = efm32_lfbclk_config(BOARD_LFBCLKSEL,
                                  BOARD_LFB_ULFCO_ENABLE, hfcoreclk);

  efm32_pcntclk_config();
  efm32_wdogclk_config();
  efm32_auxclk_config();

  UNUSED(hfperclk);
  UNUSED(coreleclk);
  UNUSED(lfaclk);
  UNUSED(lfbclk);

  /* Enable clocking of the GPIO ports */

  efm32_gpioclock();

  /* Enable Serial wire output pin, configure debug clocking, and enable ITM
   * syslog support.
   */

  efm32_itm_syslog();
}
