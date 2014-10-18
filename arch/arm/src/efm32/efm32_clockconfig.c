/****************************************************************************
 * arch/arm/src/efm32/efm32_clockconfig.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <arch/board/board.h>

#include "up_arch.h"

#include "chip.h"
#include "efm32_gpio.h"
#include "chip/efm32_cmu.h"

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
 * Name: emf32_synchronize
 *
 * Description:
 *   Wait for ongoing sync of register(s) to low frequency domain to
 *   complete.
 *
 * Input Parameters:
 *   bitset - Bitset corresponding to SYNCBUSY register defined bits,
 *            indicating registers that must complete any ongoing
 *            synchronization.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void emf32_synchronize(uint32_t bitset)
{
  /* Avoid deadlock if modifying a register again after freeze mode is
   * activated.
   */

  if ((getreg32(EFM32_CMU_FREEZE) & CMU_FREEZE_REGFREEZE) == 0)
    {
      /* Wait for any pending previous write operation to complete */

      while ((getreg32(EFM32_CMU_SYNCBUSY) & bitset) != 0);
    }
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

static inline void efm32_hfclk_config(void)
{
#warning Missing logic
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

static inline void efm32_hfcoreclk_config(void)
{
#warning Missing logic
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

static inline void efm32_hfperclk_config(void)
{
#warning Missing logic
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

static inline void efm32_lfaclk_config(void)
{
#warning Missing logic
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

static inline void efm32_lfbclk_config(void)
{
#warning Missing logic
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
#warning Missing logic
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
#warning Missing logic
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
#warning Missing logic
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
  /* Enable clocks and set dividers as determined by the board.h header file */

  efm32_hfclk_config();
  efm32_hfcoreclk_config();
  efm32_hfperclk_config();
  efm32_lfaclk_config();
  efm32_lfbclk_config();
  efm32_pcntclk_config();
  efm32_wdogclk_config();
  efm32_auxclk_config();

  /* Enable clocking of the GPIO ports */

  efm32_gpioclock();
}
