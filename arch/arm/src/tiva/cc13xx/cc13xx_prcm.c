/****************************************************************************
 * arch/arm/src/tiva/cc13xx/cc13xx_prcm.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This is a port of TI's prcm.c file (revision 49363) which has a fully
 * compatible BSD license:
 *
 *    Copyright (c) 2015-2017, Texas Instruments Incorporated
 *    All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are
 *  met:
 *
 *  1) Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *  2) Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *
 *  3) Neither the name NuttX nor the names of its contributors may be used
 *     to endorse or promote products derived from this software without
 *     specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 *  IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 *  TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 *  PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 *  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 *  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 *  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 *  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

#include "arm_internal.h"
#include "hardware/tiva_prcm.h"
#include "cc13xx/cc13xx_prcm.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Arrays that maps the "peripheral set" number (which is stored in
 * bits[11:8] of the PRCM_PERIPH_* defines) to the PRCM register that
 * contains the relevant bit for that peripheral.
 */

/* Run mode registers */

static const uintptr_t g_rcgcr_base[PRCM_NPERIPH] =
{
  TIVA_PRCM_GPTCLKGR,              /* Index 0 */
  TIVA_PRCM_SSICLKGR,              /* Index 1 */
  TIVA_PRCM_UARTCLKGR,             /* Index 2 */
  TIVA_PRCM_I2CCLKGR,              /* Index 3 */
  TIVA_PRCM_SECDMACLKGR,           /* Index 4 */
  TIVA_PRCM_GPIOCLKGR,             /* Index 5 */
  TIVA_PRCM_I2SCLKGR               /* Index 6 */
};
#define PRCM_PERIPH_GPIO        _PRCM_PERIPH(13, 1, 5, PRCM_GPIOCLKG_CLKEN_SHIFT)
#define PRCM_PERIPH_I2S         _PRCM_PERIPH(14, 1, 6, PRCM_I2SCLKG_CLKEN_SHIFT)
/* Sleep mode registers */

static const uintptr_t g_scgcr_base[PRCM_NPERIPH] =
{
  TIVA_PRCM_GPTCLKGS,              /* Index 0 */
  TIVA_PRCM_SSICLKGS,              /* Index 1 */
  TIVA_PRCM_UARTCLKGS,             /* Index 2 */
  TIVA_PRCM_I2CCLKGS,              /* Index 3 */
  TIVA_PRCM_SECDMACLKGS,           /* Index 4 */
  TIVA_PRCM_GPIOCLKGS,             /* Index 5 */
  TIVA_PRCM_I2SCLKGS               /* Index 6 */
};

/* Deep sleep mode registers */

static const uintptr_t g_dcgcr_base[PRCM_NPERIPH] =
{
  TIVA_PRCM_GPTCLKGDS,             /* Index 0 */
  TIVA_PRCM_SSICLKGDS,             /* Index 1 */
  TIVA_PRCM_UARTCLKGDS,            /* Index 2 */
  TIVA_PRCM_I2CCLKGDS,             /* Index 3 */
  TIVA_PRCM_SECDMACLKGDS,          /* Index 4 */
  TIVA_PRCM_GPIOCLKGDS,            /* Index 5 */
  TIVA_PRCM_I2SCLKGDS              /* Index 6 */
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: prcm_infclock_configure
 *
 * Description:
 *   Configure the infrastructure clock.
 *
 *   Each System CPU power mode has its own infrastructure clock division
 *   factor. This function can be used for setting up the division factor
 *   for the infrastructure clock in the available power modes for the
 *   System CPU. The infrastructure clock is used for all internal logic in
 *   the PRCM, and is always running as long as power is on in the MCU
 *   voltage domain.  This can be enabled and disabled from the AON Wake Up
 *   Controller.
 *
 *   NOTE:  If source clock is 48 MHz, minimum clock divider is 2.
 *
 * Input Parameters:
 *    clockdiv  - Determines the division ratio for the infrastructure clock
 *                when the device is in the specified mode.  Allowed
 *                division factors for all three System CPU power modes are:
 *                {1, 2, 8, or 32}
 *    powermode - Determines the System CPU operation mode for which to
 *                modify the clock division factor.  The three allowed power
 *                modes are:{PRCM_RUN_MODE, PRCM_SLEEP_MODE, or
 *                PRCM_DEEP_SLEEP_MODE}
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void prcm_infclock_configure(enum prcm_clkdivider_e clkdiv,
                             enum prcm_powermode_e powermode)
{
  uint32_t divisor;

  /* Find the correct division factor. */

  divisor = 0;
  if (clkdiv == PRCM_CLOCK_DIV_1)
    {
      divisor = PRCM_INFRCLKDIVR_RATIO_DIV1;
    }
  else if (clkdiv == PRCM_CLOCK_DIV_2)
    {
      divisor = PRCM_INFRCLKDIVR_RATIO_DIV2;
    }
  else if (clkdiv == PRCM_CLOCK_DIV_8)
    {
      divisor = PRCM_INFRCLKDIVR_RATIO_DIV8;
    }
  else if (clkdiv == PRCM_CLOCK_DIV_32)
    {
      divisor = PRCM_INFRCLKDIVR_RATIO_DIV32;
    }
  else
    {
      DEBUGPANIC();
    }

  /* Determine the correct power mode set the division factor accordingly. */

  if (powermode == PRCM_RUN_MODE)
    {
      putreg32(divisor, TIVA_PRCM_INFRCLKDIVR);
    }
  else if (powermode == PRCM_SLEEP_MODE)
    {
      putreg32(divisor, TIVA_PRCM_INFRCLKDIVS);
    }
  else if (powermode == PRCM_DEEP_SLEEP_MODE)
    {
      putreg32(divisor, TIVA_PRCM_INFRCLKDIVDS);
    }
  else
    {
      DEBUGPANIC();
    }
}

/****************************************************************************
 * Name: prcm_audioclock_manual
 *
 * Description:
 *   Configure the audio clock generation with manual setting of clock
 *   divider.
 *
 *   NOTE: See hardware documentation before setting audio clock dividers
 *   manually.
 *
 * Input Parameters:
 *   clkconfig - The audio clock configuration.  The parameter is a bitwise
 *               OR'ed value consisting of:
 *
 *               1) Phase: PRCM_I2SCLKCTL_WCLKPHASE_SINGLE or
 *                  PRCM_I2SCLKCTL_WCLKPHASE_DUAL and
 *               2) Clock polarity:  PRCM_I2SCLKCTL_POSEDGE or
 *                  PRCM_I2SCLKCTL_NEGEDGE
 *
 *   mstdiv    - The desired master clock divider.
 *   worddiv   - The desired word clock divider.
 *   bitdiv    - The desired bit clock divider.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_I2S
void prcm_audioclock_manual(uint32_t clkconfig, uint32_t mstdiv,
                            uint32_t bitdiv, uint32_t worddiv)
{
  uint32_t regval;

  DEBUGASSERT(clkconfig & ~(PRCM_I2SCLKCTL_WCLKPHASE_MASK |
                            PRCM_I2SCLKCTL_POSEDGE) == 0);

  /* Make sure the audio clock generation is disabled before reconfiguring. */

  prcm_audioclock_disable();

  /* Make sure to compensate the Frame clock division factor if using single
   * phase format.
   */

  if ((clkconfig & PRCM_I2SCLKCTL_WCLKPHASE_MASK) ==
      PRCM_I2SCLKCTL_WCLKPHASE_SINGLE)
    {
      worddiv -= 1;
    }

  /* Write the clock division factors. */

  putreg32(mstdiv, TIVA_PRCM_I2SMCLKDIV);
  putreg32(bitdiv, TIVA_PRCM_I2SBCLKDIV);
  putreg32(worddiv, TIVA_PRCM_I2SWCLKDIV);

  /* Configure the Word clock format and polarity. */

  regval  = getreg32(TIVA_PRCM_I2SCLKCTL);
  retval &= ~(PRCM_I2SCLKCTL_WCLKPHASE_MASK | PRCM_I2SCLKCTL_POSEDGE);
  putreg32(regval | clkconfig, TIVA_PRCM_I2SCLKCTL);
}
#endif

/****************************************************************************
 * Name: prcm_audioclock_configure
 *
 * Description:
 *   Configure the audio clock generation
 *
 *   Use this function to set the sample rate when using internal audio
 *   clock generation for the I2S module.
 *
 *   NOTE:  While other clocks are possible, the stability of the four
 *   sample rates defined here are only guaranteed if the clock input to the
 *   I2S module is 48MHz.
 *
 * Input Parameters:
 *   clkconfig - The audio clock configuration.  The parameter is a bitwise
 *               OR'ed value consisting of:
 *
 *               1) Phase: PRCM_I2SCLKCTL_WCLKPHASE_SINGLE or
 *                  PRCM_I2SCLKCTL_WCLKPHASE_DUAL and
 *               2) Clock polarity:  PRCM_I2SCLKCTL_POSEDGE or
 *                  PRCM_I2SCLKCTL_NEGEDGE
 *
 *   samplerate - The desired audio clock sample rate.  The supported sample
 *                rate configurations are: {I2S_SAMPLE_RATE_16K,
 *                I2S_SAMPLE_RATE_24K, I2S_SAMPLE_RATE_32K, or
 *                I2S_SAMPLE_RATE_48K}
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_I2S
void prcm_audioclock_configure(uint32_t clkconfig,
                               enum prcm_i2samplerate_e samplerate)
{
  uint32_t mstdiv;
  uint32_t bitdiv;
  uint32_t worddiv;

  DEBUGASSERT(samplerate == I2S_SAMPLE_RATE_16K ||
              samplerate == I2S_SAMPLE_RATE_24K ||
              samplerate == I2S_SAMPLE_RATE_32K ||
              samplerate == I2S_SAMPLE_RATE_48K);

  mstdiv  = 0;
  bitdiv  = 0;
  worddiv = 0;

  /* Define the clock division factors for the audio interface. */

  switch (samplerate)
    {
    case I2S_SAMPLE_RATE_16K:
      mstdiv  = 6;
      bitdiv  = 60;
      worddiv = 25;
      break;

    case I2S_SAMPLE_RATE_24K:
      mstdiv  = 4;
      bitdiv  = 40;
      worddiv = 25;
      break;

    case I2S_SAMPLE_RATE_32K:
      mstdiv  = 3;
      bitdiv  = 30;
      worddiv = 25;
      break;

    case I2S_SAMPLE_RATE_48K:
      mstdiv  = 2;
      bitdiv  = 20;
      worddiv = 25;
      break;
    }

  prcm_audioclock_manual(clkconfig, mstdiv, bitdiv, worddiv);
}
#endif

/****************************************************************************
 * Name: prcm_powerdomain_on
 *
 * Description:
 *   Turn power on in power domains in the MCU domain
 *   Use this function to turn on power domains inside the MCU voltage
 *   domain.
 *
 *   Power on and power off request has different implications for the
 *   different power domains.
 *   - RF Core power domain:
 *     - Power On  : Domain is on or in the process of turning on.
 *     - Power Off : Domain is powered down when System CPU is in deep
 *                   sleep. The third option for the RF Core is to power
 *                   down when the it is idle.  prcm_rfpowerdown_whenidle()
 *   - SERIAL power domain:
 *     - Power on  : Domain is powered on.
 *     - Power off : Domain is powered off.
 *   - PERIPHERAL power domain:
 *     - Power on  : Domain is powered on.
 *     - Power off : Domain is powered off.
 *   - VIMS power domain:
 *     - Power On  : Domain is powered if Bus domain is powered.
 *     - Power Off : Domain is only powered when CPU domain is on.
 *   - BUS power domain:
 *     - Power On  : Domain is on.
 *     - Power Off : Domain is on if requested by RF Core or if CPU domain
 *                   is on.
 *   - CPU power domain:
 *     - Power On  : Domain is on.
 *     - Power Off : Domain is powering down if System CPU is idle. This
 *                   will also initiate a power down of the SRAM and BUS
 *                   power domains, unless RF Core is requesting them to be
 *                   on.
 *
 *   NOTE:  After a call to this function the status of the power domain
 *   should be checked using either prcm_powerdomain_status(). Any write
 *   operation to a power domain which is still not operational can result
 *   in unexpected behavior.
 *
 * Input Parameters:
 *   domains - Determines which power domains to turn on.  The domains that
 *             can be turned on/off are:
 *             1) PRCM_DOMAIN_RFCORE : RF Core
 *             2) PRCM_DOMAIN_SERIAL : SSI0, UART0, I2C0
 *             3) PRCM_DOMAIN_PERIPH : GPT0, GPT1, GPT2, GPT3, GPIO, SSI1,
 *                                     I2S, DMA, UART1
 *             4) PRCM_DOMAIN_VIMS   : SRAM, FLASH, ROM
 *             5) PRCM_DOMAIN_SYSBUS
 *             6) PRCM_DOMAIN_CPU
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void prcm_powerdomain_on(uint32_t domains)
{
  /* Check the arguments. */

  DEBUGASSERT((domains & PRCM_DOMAIN_RFCORE) != 0 ||
              (domains & PRCM_DOMAIN_SERIAL) != 0 ||
              (domains & PRCM_DOMAIN_PERIPH) != 0 ||
              (domains & PRCM_DOMAIN_CPU)    != 0 ||
              (domains & PRCM_DOMAIN_VIMS)   != 0);

  /* Assert the request to power on the right domains. */

  if ((domains & PRCM_DOMAIN_RFCORE) != 0)
    {
      putreg32(PRCM_PDCTL0RFC_ON, TIVA_PRCM_PDCTL0RFC);

#ifdef CONFIG_ARCH_CHIP_CC13X0
      /* The PDCTL1RFC access is meant to "be used by RFC in autonomous
       * mode", but keeping it for compatibility on already ROM'ed products
       * (since this is a ROM function). RFC power domain is on if
       * (PRCM_O_PDCTL0RFC || PRCM_O_PDCTL1RFC).
       */

      putreg32(PRCM_PDCTL1RFC_ON, TIVA_PRCM_PDCTL1RFC);
#endif
    }

  if ((domains & PRCM_DOMAIN_SERIAL) != 0)
    {
      putreg32(PRCM_PDCTL0SERIAL_ON, TIVA_PRCM_PDCTL0SERIAL);
    }

  if ((domains & PRCM_DOMAIN_PERIPH) != 0)
    {
      putreg32(PRCM_PDCTL0PERIPH_ON, TIVA_PRCM_PDCTL0PERIPH);
    }

  if ((domains & PRCM_DOMAIN_VIMS) != 0)
    {
      putreg32(PRCM_PDCTL1VIMS_ON, TIVA_PRCM_PDCTL1VIMS);
    }

  if ((domains & PRCM_DOMAIN_CPU) != 0)
    {
      putreg32(PRCM_PDCTL1CPU_ON, TIVA_PRCM_PDCTL1CPU);
    }
}

/****************************************************************************
 * Name: prcm_powerdomain_off
 *
 * Description:
 *   Turn off a specific power domain
 *   Use this function to power down domains inside the MCU voltage domain.
 *
 *   NOTE:  See prcm_powerdomain_on() for specifics regarding on/off
 *   configuration.
 *
 * Input Parameters:
 *   domains - Determines which power domains to turn off.  The domains that
 *             can be turned on/off are:
 *             1) PRCM_DOMAIN_RFCORE : RF Core
 *             2) PRCM_DOMAIN_SERIAL : SSI0, UART0, I2C0
 *             3) PRCM_DOMAIN_PERIPH : GPT0, GPT1, GPT2, GPT3, GPIO, SSI1,
 *                                     I2S, DMA, UART1
 *             4) PRCM_DOMAIN_VIMS   : SRAM, FLASH, ROM
 *             5) PRCM_DOMAIN_SYSBUS
 *             6) PRCM_DOMAIN_CPU
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void prcm_powerdomain_off(uint32_t domains)
{
  /* Check the arguments. */

  DEBUGASSERT((domains & PRCM_DOMAIN_RFCORE) != 0 ||
              (domains & PRCM_DOMAIN_SERIAL) != 0 ||
              (domains & PRCM_DOMAIN_PERIPH) != 0 ||
              (domains & PRCM_DOMAIN_CPU)    != 0 ||
              (domains & PRCM_DOMAIN_VIMS)   != 0);

  /* Assert the request to power off the right domains. */

  if (domains & PRCM_DOMAIN_RFCORE)
    {
      putreg32(0, TIVA_PRCM_PDCTL0RFC);

#ifdef CONFIG_ARCH_CHIP_CC13X0
      /* The PDCTL1RFC access is meant to "be used by RFC in autonomous
       * mode", but keeping it for compatibility on already ROM'ed products
       * (since this is a ROM function). RFC power domain is on if
       * (PRCM_O_PDCTL0RFC || PRCM_O_PDCTL1RFC).
       */

      putreg32(0, TIVA_PRCM_PDCTL1RFC);
#endif
    }

  if (domains & PRCM_DOMAIN_SERIAL)
    {
      putreg32(0, TIVA_PRCM_PDCTL0SERIAL);
    }

  if (domains & PRCM_DOMAIN_PERIPH)
    {
      putreg32(0, TIVA_PRCM_PDCTL0PERIPH);
    }

  if (domains & PRCM_DOMAIN_VIMS)
    {
#ifdef CONFIG_ARCH_CHIP_CC13X0
      putreg32(0, TIVA_PRCM_PDCTL1VIMS);
#else
      /* Write bits domains[17:16] to the VIMS_MODE alias register.
       * PRCM_DOMAIN_VIMS sets VIMS_MODE=0b00, PRCM_DOMAIN_VIMS_OFF_NO_WAKEUP
       * sets VIMS_MODE=0b10.
       */

      DEBUGASSERT((domains & 0x00010000) == 0);
      putreg32((domains >> 16) & 3, TIVA_PRCM_PDCTL1VIMS);
#endif
    }

  if (domains & PRCM_DOMAIN_CPU)
    {
      putreg32(0, TIVA_PRCM_PDCTL1CPU);
    }
}

/****************************************************************************
 * Name: prcm_powerdomain_status
 *
 * Description:
 *   Use this function to retrieve the current power status of one or more
 *   power domains.
 *
 * Input Parameters:
 *    domains - Determines which domain to get the power status for.  The
 *              parameter must be an OR'ed combination of one or several of:
 *              1) PRCM_DOMAIN_RFCORE : RF Core.
 *              2) PRCM_DOMAIN_SERIAL : SSI0, UART0, I2C0
 *              3) PRCM_DOMAIN_PERIPH : GPT0, GPT1, GPT2, GPT3, GPIO, SSI1,
 *                 I2S, DMA, UART1
 *
 * Returned Value
 *    - True:  The specified domains are all powered up.
 *    - False: One or more of the domains is powered down.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

bool prcm_powerdomain_status(uint32_t domains)
{
  uint32_t pdstat0;
  uint32_t pdstat1;
  bool status;

  DEBUGASSERT((domains & (PRCM_DOMAIN_RFCORE | PRCM_DOMAIN_SERIAL |
                          PRCM_DOMAIN_PERIPH)) != 0);

  status  = true;
  pdstat0 = getreg32(TIVA_PRCM_PDSTAT0);
  pdstat1 = getreg32(TIVA_PRCM_PDSTAT1);

  /* Return the correct power status. */

  if (domains & PRCM_DOMAIN_RFCORE)
    {
       status = status && ((pdstat0 & PRCM_PDSTAT0_RFC_ON) != 0 ||
                           (pdstat1 & PRCM_PDSTAT1_RFC_ON) != 0);
    }

  if (domains & PRCM_DOMAIN_SERIAL)
    {
      status = status && (pdstat0 & PRCM_PDSTAT0_SERIAL_ON) != 0;
    }

  if (domains & PRCM_DOMAIN_PERIPH)
    {
      status = status && (pdstat0 & PRCM_DOMAIN_PERIPH) != 0;
    }

  /* Return the status. */

  return status;
}

/****************************************************************************
 * Name: prcm_periph_runenable
 *
 * Description:
 *   Enables a peripheral in Run mode
 *
 *   Peripherals are enabled with this function.  At power-up, some
 *   peripherals are disabled; they must be enabled in order to operate or
 *   respond to register reads/writes.
 *
 *   NOTE:  The actual enabling of the peripheral may be delayed until some
 *   time after this function returns. Care should be taken to ensure that
 *   the peripheral is not accessed until it is enabled.
 *
 *   When enabling Timers always make sure that the division factor for the
 *   PERBUSCPUCLK is set. This will guarantee that the timers run at a
 *   continuous rate even if the SYSBUSCLK is gated.
 *
 *   NOTE: A call to this function will only setup the shadow registers in
 *   the MCU domain for the PRCM module. For the changes to propagate to the
 *   system controller in the AON domain a call to this function should
 *   always be followed by a call to prcm_load_set().
 *
 * Input Parameters:
 *   peripheral - The peripheral to enable. This is an encoded value.  See
 *                the PRCRM_PERIPH_* definitions for available encodings.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void prcm_periph_runenable(uint32_t peripheral)
{
  unsigned int index;

  /* Extract the index */

  index = PRCM_PERIPH_INDEX(peripheral);
  DEBUGASSERT(index < PRCM_NPERIPH);

  /* Enable module in Run Mode. */

  modifyreg32(g_rcgcr_base[index], 0, PRCM_PERIPH_MASKBIT(peripheral));
}

/****************************************************************************
 * Name: prcm_periph_rundisable
 *
 * Description:
 *   Disables a peripheral in Run mode
 *
 *   Peripherals are disabled with this function. Once disabled, they will
 *   not operate or respond to register reads/writes.
 *
 *   NOTE: A call to this function will only setup the shadow registers in
 *   the MCU domain for the PRCM module. For the changes to propagate to the
 *   system controller in the AON domain a call to this function should
 *   always be followed by a call to prcm_load_set().
 *
 *   NOTE: The actual disabling of the peripheral may be delayed until some
 *   time after this function returns. Care should be taken by the user to
 *   ensure that the peripheral is not accessed in this interval as this
 *   might cause the system to hang.
 *
 * Input Parameters:
 *   peripheral - The peripheral to disable. This is an encoded value.  See
 *                the PRCRM_PERIPH_* definitions for available encodings.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void prcm_periph_rundisable(uint32_t peripheral)
{
  unsigned int index;

  /* Extract the index */

  index = PRCM_PERIPH_INDEX(peripheral);
  DEBUGASSERT(index < PRCM_NPERIPH);

  /* Disable module in Run Mode. */

  modifyreg32(g_rcgcr_base[index], PRCM_PERIPH_MASKBIT(peripheral), 0);
}

/****************************************************************************
 * Name: prcm_periph_sleepenable
 *
 * Description:
 *   Enables a peripheral in sleep mode
 *
 *   This function allows a peripheral to continue operating when the
 *   processor goes into sleep mode. Since the clocking configuration of the
 *   device does not change, any peripheral can safely continue operating
 *   while the processor is in sleep mode, and can therefore wake the
 *   processor from sleep mode.
 *
 *   NOTE: A call to this function will only setup the shadow registers in
 *   the MCU domain for the PRCM module. For the changes to propagate to the
 *   system controller in the AON domain a call to this function should
 *   always be followed by a call to prcm_load_set().
 *
 * Input Parameters:
 *   peripheral - The peripheral to enable in sleep mode. This is an encoded
 *                value.  See the PRCRM_PERIPH_* definitions for available
 *                encodings.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void prcm_periph_sleepenable(uint32_t peripheral)
{
  unsigned int index;

  /* Extract the index */

  index = PRCM_PERIPH_INDEX(peripheral);
  DEBUGASSERT(index < PRCM_NPERIPH);

  /* Enable this peripheral in sleep mode. */

  modifyreg32(g_scgcr_base[index], 0, PRCM_PERIPH_MASKBIT(peripheral));
}

/****************************************************************************
 * Name: prcm_periph_sleepdisable
 *
 * Description:
 *   Disables a peripheral in sleep mode
 *
 *   This function causes a peripheral to stop operating when the processor
 *   goes into sleep mode. Disabling peripherals while in sleep mode helps
 *   to lower the current draw of the device. If enabled (via
 *   prcm_periph_runenable()), the peripheral will automatically resume
 *   operation when the processor leaves sleep mode, maintaining its entire
 *   state from before sleep mode was entered.
 *
 *   NOTE: A call to this function will only setup the shadow registers in
 *   the MCU domain for the PRCM module. For the changes to propagate to the
 *   system controller in the AON domain a call to this function should
 *   always be followed by a call to prcm_load_set().
 *
 * Input Parameters:
 *   peripheral - The peripheral to disable in sleep mode. This is an
 *                encoded value.  See the PRCRM_PERIPH_* definitions for
 *                available encodings.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void prcm_periph_sleepdisable(uint32_t peripheral)
{
  unsigned int index;

  /* Extract the index */

  index = PRCM_PERIPH_INDEX(peripheral);
  DEBUGASSERT(index < PRCM_NPERIPH);

  /* Disable this peripheral in sleep mode */

  modifyreg32(g_scgcr_base[index], PRCM_PERIPH_MASKBIT(peripheral), 0);
}

/****************************************************************************
 * Name: prcm_periph_deepsleepenable
 *
 * Description:
 *   Enables a peripheral in deep-sleep mode
 *
 *   This function allows a peripheral to continue operating when the
 *   processor goes into deep-sleep mode.  Since the clocking configuration
 *   of the device may change, not all peripherals can safely continue
 *   operating while the processor is in sleep mode. This in turn depends on
 *   the chosen power mode. It is the responsibility of the caller to make
 *   sensible choices.
 *
 *   NOTE: A call to this function will only setup the shadow registers in
 *   the MCU domain for the PRCM module. For the changes to propagate to the
 *   system controller in the AON domain a call to this function should
 *   always be followed by a call to prcm_load_set().
 *
 * Input Parameters:
 *   peripheral - The peripheral to ensable in deep sleep mode. This is an
 *                encoded value.  See the PRCRM_PERIPH_* definitions for
 *                available encodings.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void prcm_periph_deepsleepenable(uint32_t peripheral)
{
  unsigned int index;

  /* Extract the index */

  index = PRCM_PERIPH_INDEX(peripheral);
  DEBUGASSERT(index < PRCM_NPERIPH);

  /* Enable this peripheral in sleep mode. */

  modifyreg32(g_dcgcr_base[index], 0, PRCM_PERIPH_MASKBIT(peripheral));
}

/****************************************************************************
 * Name: prcm_periph_deepsleepdisable
 *
 * Description:
 *   Disables a peripheral in deep-sleep mode
 *
 *   This function causes a peripheral to stop operating when the processor
 *   goes into deep-sleep mode.  Disabling peripherals while in deep-sleep
 *   mode helps to lower the current draw of the device, and can keep
 *   peripherals that require a particular clock frequency from operating
 *   when the clock changes as a result of entering deep-sleep mode.  If
 *   enabled (via prcm_periph_runenable()), the peripheral will
 *   automatically resume operation when the processor leaves deep-sleep
 *   mode, maintaining its entire state from before deep-sleep mode was
 *   entered.
 *
 *   NOTE: A call to this function will only setup the shadow registers in
 *   the MCU domain for the PRCM module. For the changes to propagate to the
 *   system controller in the AON domain a call to this function should
 *   always be followed by a call to prcm_load_set().
 *
 * Input Parameters:
 *   peripheral - The peripheral to disable in deep sleep mode. This is an
 *                encoded value.  See the PRCRM_PERIPH_* definitions for
 *                available encodings.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void prcm_periph_deepsleepdisable(uint32_t peripheral)
{
  unsigned int index;

  /* Extract the index */

  index = PRCM_PERIPH_INDEX(peripheral);
  DEBUGASSERT(index < PRCM_NPERIPH);

  /* Enable this peripheral in sleep mode. */

  modifyreg32(g_dcgcr_base[index], PRCM_PERIPH_MASKBIT(peripheral), 0);
}
