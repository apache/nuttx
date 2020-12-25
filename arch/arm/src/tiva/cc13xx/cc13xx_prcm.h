/****************************************************************************
 * arch/arm/src/tiva/cc13xx/.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Includes definitions from TI's prcm.c file which has a fully compatible
 * BSD license:
 *
 *    Copyright (c) 2015-2017, Texas Instruments Incorporated
 *    All rights reserved.
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

#ifndef __ARCH_ARM_SRC_TIVA_CC13XX_CC13XX_PRCM_H
#define __ARCH_ARM_SRC_TIVA_CC13XX_CC13XX_PRCM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

#include "arm_arch.h"
#include "hardware/tiva_prcm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Power Domains */

#define PRCM_DOMAIN_RFCORE      (1 << 0)  /* RF Core domain ID for
                                           * clock/power control. */
#define PRCM_DOMAIN_SERIAL      (1 << 1)  /* Serial domain ID for
                                           * clock/power control. */
#define PRCM_DOMAIN_PERIPH      (1 << 2)  /* Peripheral domain ID for
                                           * clock/power control. */
#define PRCM_DOMAIN_SYSBUS      (1 << 3)  /* Bus domain ID for clock/power
                                           * control. */
#define PRCM_DOMAIN_VIMS        (1 << 4)  /* VIMS domain ID for clock/power
                                           * control. */
#define PRCM_DOMAIN_CPU         (1 << 5)  /* CPU domain ID for clock/power
                                           * control. */
#define PRCM_DOMAIN_TIMER       (1 << 6)  /* GPT domain ID for clock
                                           * control. */
#define PRCM_DOMAIN_CLKCTRL     (1 << 7)  /* Clock Control domain ID for
                                           * clock/power control. */
#define PRCM_DOMAIN_MCU         (1 << 8)  /* Reset control for entire MCU
                                           * domain. */

#define PRCM_DOMAIN_VIMS_OFF_NO_WAKEUP \
                                0x00020010  /* For function rom_prcm_powerdomain_off()
                                             * it is an option to select that
                                             * VIMS power domain shall not
                                             * power up during the next wake
                                             * up from uLDO (VIMS_MODE=0b10).
                                             */

/* Encoded values used for enabling and disabling peripheral modules in the
 * MCU domain.  Encoding:
 *
 *   Bits 0-4:   Defines the bit position within the register.
 *   Bits 5-7:   Defines the index into the register offset constant tables.
 *   Bit  8:     Power domain.  0=SERIAL 1=PERIPH
 *   Bits 12-15: Unique peripheral identifier
 */

#define _PRCM_PERIPH(p,d,i,b)   (((p) << 12) | ((d) << 8) | ((i) << 5) | (b))
#define PRCM_PERIPH_TIMER0      _PRCM_PERIPH( 0, 1, 0, PRCM_GPTCLKG_CLKEN_GPT0_SHIFT)
#define PRCM_PERIPH_TIMER1      _PRCM_PERIPH( 1, 1, 0, PRCM_GPTCLKG_CLKEN_GPT1_SHIFT)
#define PRCM_PERIPH_TIMER2      _PRCM_PERIPH( 2, 1, 0, PRCM_GPTCLKG_CLKEN_GPT2_SHIFT)
#define PRCM_PERIPH_TIMER3      _PRCM_PERIPH( 3, 1, 0, PRCM_GPTCLKG_CLKEN_GPT3_SHIFT)
#define PRCM_PERIPH_SSI0        _PRCM_PERIPH( 4, 0, 1, PRCM_SSICLKG_CLKEN_SSI1_SHIFT)
#define PRCM_PERIPH_SSI1        _PRCM_PERIPH( 5, 1, 1, PRCM_SSICLKG_CLKEN_SSI1_SHIFT)
#define PRCM_PERIPH_UART0       _PRCM_PERIPH( 6, 0, 2, PRCM_UARTCLKG_CLKEN_UART0_SHIFT)
#ifdef CONFIG_ARCH_CHIP_CC13X2
#  define PRCM_PERIPH_UART1     _PRCM_PERIPH( 7, 1, 2, PRCM_UARTCLKG_CLKEN_UART1_SHIFT)
#endif
#define PRCM_PERIPH_I2C0        _PRCM_PERIPH( 8, 0, 3, PRCM_I2CCLKGR_CLKEN_SHIFT)
#define PRCM_PERIPH_CRYPTO      _PRCM_PERIPH( 9, 1, 4, PRCM_SECDMACLKG_CRYPTO_CLKEN_SHIFT)
#define PRCM_PERIPH_TRNG        _PRCM_PERIPH(10, 1, 4, PRCM_SECDMACLKG_TRNG_CLKEN_SHIFT)
#ifdef CONFIG_ARCH_CHIP_CC13X2
#  define PRCM_PERIPH_PKA       _PRCM_PERIPH(11, 1, 4, PRCM_SECDMACLKG_PKA_CLKEN_SHIFT)
#endif
#define PRCM_PERIPH_UDMA        _PRCM_PERIPH(12, 1, 4, PRCM_SECDMACLKG_DMA_CLKEN_SHIFT)
#define PRCM_PERIPH_GPIO        _PRCM_PERIPH(13, 1, 5, PRCM_GPIOCLKG_CLKEN_SHIFT)
#define PRCM_PERIPH_I2S         _PRCM_PERIPH(14, 1, 6, PRCM_I2SCLKG_CLKEN_SHIFT)

/* This macro extracts the power domain index out of the peripheral number */

#define PRCM_PERIPH_ID(a)       (((a) >> 12) & 15)

/* This macro extracts the power domain index out of the peripheral number */

#define PRCM_DOMAIN_INDEX(a)    (((a) >> 8) & 1)

/* This macro extracts the array index out of the peripheral number */

#define PRCM_PERIPH_INDEX(a)    (((a) >> 5) & 7)

/* This macro extracts the peripheral instance number and generates bit
 * mask
 */

#define PRCM_PERIPH_MASKBIT(a)  (1 << ((a) & 0x1f))

/* The size of a register look-up table */

#define PRCM_NPERIPH            7

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum prcm_powermode_e
{
  PRCM_RUN_MODE = 0,
  PRCM_SLEEP_MODE,
  PRCM_DEEP_SLEEP_MODE
};

enum prcm_clkdivider_e
{
  PRCM_CLOCK_DIV_1 = 0,
  PRCM_CLOCK_DIV_2,
  PRCM_CLOCK_DIV_8,
  PRCM_CLOCK_DIV_32
};

enum prcm_i2samplerate_e
{
  I2S_SAMPLE_RATE_16K = 1,
  I2S_SAMPLE_RATE_24K = 2,
  I2S_SAMPLE_RATE_32K = 4,
  I2S_SAMPLE_RATE_48K = 8
};

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: prcm_audioclock_enable
 *
 * Description:
 *   Use this function to enable the audio clock generation.
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_I2S
static inline void prcm_audioclock_enable(void)
{
  /* Enable the audio clock generation. */

  modifyreg32(TIVA_PRCM_I2SCLKCTL, PRCM_I2SCLKCTL_EN, 0);
}
#endif

/****************************************************************************
 * Name: prcm_audioclock_disable
 *
 * Description:
 *   Use this function to disable the audio clock generation.
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_I2S
static inline void prcm_audioclock_disable(void)
{
  /* Disable the audio clock generation */

  modifyreg32(TIVA_PRCM_I2SCLKCTL, 0, PRCM_I2SCLKCTL_EN);
}
#endif

/****************************************************************************
 * Name: prcm_mcuuldo_configure
 *
 * Description:
 *   Assert or de-assert a request for the uLDO.
 *
 *   Use this function to request to switch to the micro Low Voltage Dropout
 *   regulator (uLDO). The uLDO has a much lower capacity for supplying power
 *   to the system. It is therefore imperative and solely the programmers
 *   responsibility to ensure that a sufficient amount of peripheral modules
 *   have been turned of before requesting a switch to the uLDO.
 *
 *   NOTE: Asserting this bit has no effect until:
 *   1. FLASH has accepted to be powered down
 *   2. Deepsleep must be asserted
 *
 * Input Parameters:
 *   enable
 *   - 0 : Disable uLDO request
 *   - 1 : Enable uLDO request
 *
 ****************************************************************************/

static inline void prcm_mcuuldo_configure(uint32_t enable)
{
  /* Enable or disable the uLDO request signal. */

  putreg32(enable, TIVA_PRCM_VDCTL);
}

/****************************************************************************
 * Name: prcm_gptclock_set
 *
 * Description:
 *   Setup the clock division factor for the GP-Timer domain.
 *
 *   Use this function to set up the clock division factor on the GP-Timer.
 *
 *   The division rate will be constant and ungated for Run / Sleep /
 *   DeepSleep mode when it is slower than PRCM_GPTCLKDIV_RATIO setting.
 *   When set faster than PRCM_GPTCLKDIV_RATIO setting PRCM_GPTCLKDIV_RATIO
 *   will be used.
 *
 *   Note that the register will contain the written content even though the
 *   setting is faster than PRCM_GPTCLKDIV_RATIO setting.
 *
 *   NOTE: For change to take effect, prcm_load_set() needs to be called
 *
 * Input Parameters:
 *   clkdiv - The division factor to set.  The argument must be only one of
 *            the following values:
 *              PRCM_GPTCLKDIV_DIV1, PRCM_GPTCLKDIV_DIV2,
 *              PRCM_GPTCLKDIV_DIV4, PRCM_GPTCLKDIV_DIV8,
 *              PRCM_GPTCLKDIV_DIV16, PRCM_GPTCLKDIV_DIV32.
 *              PRCM_GPTCLKDIV_DIV64, PRCM_GPTCLKDIV_DIV128,
 *              PRCM_GPTCLKDIV_DIV256
 *
 ****************************************************************************/

static inline void prcm_gptclock_set(uint32_t clkdiv)
{
  putreg32(clkdiv, TIVA_PRCM_GPTCLKDIV);
}

/****************************************************************************
 * Name:
 *
 * Description:
 *   Get the clock division factor for the GP-Timer domain.
 *
 ****************************************************************************/

static inline uint32_t prcm_gptclock_get(void)
{
  return getreg32(TIVA_PRCM_GPTCLKDIV);
}

/****************************************************************************
 * Name: prcm_load_set
 *
 * Description:
 *   Use this function to synchronize the load settings.
 *
 *   Most of the clock settings in the PRCM module should be updated
 *   synchronously. This is ensured by the implementation of a load
 *   registers that, when written to, will let the previous written update
 *   values for all the relevant registers propagate through to hardware.
 *
 *   The functions that require a synchronization of the clock settings are:
 *   - prcm_audioclock_manual()
 *   - prcm_audioclock_configure()
 *   - prcm_audioclock_disable()
 *   - prcm_domain_enable()
 *   - prcm_domain_disable()
 *   - prcm_periph_runenable()
 *   - prcm_periph_rundisable()
 *   - prcm_periph_sleepenable()
 *   - prcm_periph_sleepdisable()
 *   - prcm_periph_deepsleepenable()
 *   - prcm_periph_deepsleepdisable()
 *
 ****************************************************************************/

static inline void prcm_load_set(void)
{
  /* Enable the update of all load related registers. */

  putreg32(PRCM_CLKLOADCTL_LOAD, TIVA_PRCM_CLKLOADCTL);
}

/****************************************************************************
 * Name: prcm_load_get
 *
 * Description:
 *   Check if any of the load sensitive register has been updated.
 *
 * Returned Value:
 *   Returns status of the load sensitive register:
 *   - true  : No registers have changed since the last load.
 *   - false : Any register has changed.
 *
 ****************************************************************************/

static inline bool prcm_load_get(void)
{
  /* Return the load status. */

  return ((getreg32(TIVA_PRCM_CLKLOADCTL) & PRCM_CLKLOADCTL_LOADDONE) != 0);
}

/****************************************************************************
 * Name: prcm_domain_enable
 *
 * Description:
 *   Enable clock domains in the MCU voltage domain.
 *
 *   NOTE: A call to this function will only setup the shadow registers in
 *   the MCU domain for the PRCM module. For the changes to propagate to the
 *   system controller in the AON domain a call to this function should
 *   always be followed by a call to prcm_load_set().
 *
 *   NOTE: Clocks will only be running if the domain is powered.
 *
 *   domains is a bit mask containing the clock domains to enable.
 *   The independent clock domains inside the MCU voltage domain which can be
 *   configured are:
 *   - PRCM_DOMAIN_RFCORE
 *   - PRCM_DOMAIN_VIMS
 *
 ****************************************************************************/

static inline void prcm_domain_enable(uint32_t domains)
{
  DEBUGASSERT((domains & PRCM_DOMAIN_RFCORE) != 0 ||
              (domains & PRCM_DOMAIN_VIMS) != 0);

  /* Enable the clock domain(s). */

  if ((domains & PRCM_DOMAIN_RFCORE) != 0)
    {
      putreg32(PRCM_RFCCLKG_CLKEN, TIVA_PRCM_RFCCLKG);
    }

  if ((domains & PRCM_DOMAIN_VIMS) != 0)
    {
      putreg32(PRCM_VIMSCLKG_CLKEN_ENA, TIVA_PRCM_VIMSCLKG);
    }
}

/****************************************************************************
 * Name: prcm_domain_disable
 *
 * Description:
 *   Disable clock domains in the MCU voltage domain.
 *
 *   NOTE: A call to this function will only setup the shadow registers in
 *   the MCU domain for the PRCM module. For the changes to propagate to the
 *   system controller in the AON domain a call to this function should
 *   always be followed by a call to prcm_load_set().
 *
 *   NOTE: Clocks will only be running if the domain is powered.
 *
 *   domains is a bit mask containing the clock domains to disable.
 *   The independent clock domains inside the MCU voltage domain are:
 *   - PRCM_DOMAIN_RFCORE
 *   - PRCM_DOMAIN_VIMS
 *
 ****************************************************************************/

static inline void prcm_domain_disable(uint32_t domains)
{
  DEBUGASSERT((domains & PRCM_DOMAIN_RFCORE) != 0 ||
              (domains & PRCM_DOMAIN_VIMS) != 0);

  /* Disable the power domains. */

  if ((domains & PRCM_DOMAIN_RFCORE) != 0)
    {
      putreg32(0, TIVA_PRCM_RFCCLKG);
    }

  if ((domains & PRCM_DOMAIN_VIMS) != 0)
    {
      putreg32(0, TIVA_PRCM_VIMSCLKG);
    }
}

/****************************************************************************
 * Name: prcm_rfpowerdown_whenidle
 *
 * Description:
 *   Configure RF core to power down when idle.
 *
 *   Use this function to configure the RF core to power down when Idle.
 *   This is handled automatically in hardware if the RF Core reports that
 *   it is idle.
 *
 ****************************************************************************/

static inline void prcm_rfpowerdown_whenidle(void)
{
  /* Configure the RF power domain. */

  modifyreg32(TIVA_PRCM_PDCTL0RFC, 0, PRCM_PDCTL0RFC_ON);
}

/****************************************************************************
 * Name: prcm_rfready
 *
 * Description:
 *   Return the access status of the RF Core.
 *
 *   Use this function to check if the RF Core is on and ready to be
 *   accessed. Accessing register or memories that are not powered and
 *   clocked will cause a bus fault.
 *
 *   Returns access status of the RF Core.
 *   - true  : RF Core can be accessed.
 *   - false : RF Core domain is not ready for access.
 *
 ****************************************************************************/

static inline bool prcm_rfready(void)
{
  /* Return the ready status of the RF Core. */

  return ((getreg32(TIVA_PRCM_PDSTAT1RFC) & PRCM_PDSTAT1RFC_ON) != 0);
}

/****************************************************************************
 * Name: prcm_cacheretention_enable
 *
 * Description:
 *   Enable CACHE RAM retention
 *
 *   Enables CACHE RAM retention on both VIMS_TRAM and VIMS_CRAM
 *
 ****************************************************************************/

static inline void prcm_cacheretention_enable(void)
{
  modifyreg32(TIVA_PRCM_RAMRETEN,
              PRCM_RAMRETEN_VIMS_TRAM | PRCM_RAMRETEN_VIMS_CRAM, 0);
}

/****************************************************************************
 * Name: prcm_cacheretention_disable
 *
 * Description:
 *   Disable CACHE RAM retention
 *
 *   Disables CACHE RAM retention on both VIMS_TRAM and VIMS_CRAM
 *
 ****************************************************************************/

static inline void prcm_cacheretention_disable(void)
{
  modifyreg32(TIVA_PRCM_RAMRETEN, 0, PRCM_RAMRETEN_VIMS_MASK);
}

/****************************************************************************
 * Public Function Prototypes
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
 *    clockdiv  - Determines the division ratio for the infrastructure
 *                clock when the device is in the specified mode.  Allowed
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
                             enum prcm_powermode_e powermode);

/****************************************************************************
 * Name: prcm_audioclock_manual
 *
 * Description:
 *   Configure the audio clock generation with manual setting of clock
 *   divider.
 *
 *   NOTE: See hardware documentation before setting audio clock dividers
 *         manually.
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
                            uint32_t bitdiv, uint32_t worddiv);
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
                               enum prcm_i2samplerate_e samplerate);
#endif

/****************************************************************************
 * Name: prcm_powerdomain_on
 *
 * Description:
 *   Turn power on in power domains in the MCU domain
 *
 *   Use this function to turn on power domains inside the MCU voltage
 *   domain.
 *
 *   Power on and power off request has different implications for the
 *   different power domains.
 *   - RF Core power domain:
 *     - Power On  : Domain is on or in the process of turning on.
 *     - Power Off : Domain is powered down when System CPU is in deep sleep.
 *                   The third option for the RF Core is to power down when
 *                   the it is idle.  prcm_rfpowerdown_whenidle()
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
 *                   power domains, unless RF Core is requesting them to
 *                   be on.
 *
 *   NOTE:  After a call to this function the status of the power domain
 *   should be checked using either prcm_powerdoamin_status().
 *   Any write operation to a power domain which is still not operational can
 *   result in unexpected behavior.
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

void prcm_powerdomain_on(uint32_t domains);

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

void prcm_powerdomain_off(uint32_t domains);

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

bool prcm_powerdomain_status(uint32_t domains);

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

void prcm_periph_runenable(uint32_t peripheral);

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
 *   peripheral - The peripheral to enable. This is an encoded value.  See
 *                the PRCRM_PERIPH_* definitions for available encodings.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void prcm_periph_rundisable(uint32_t peripheral);

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

void prcm_periph_sleepenable(uint32_t peripheral);

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
 *   peripheral - The peripheral to disable in sleep mode. This is an encoded
 *                value.  See the PRCRM_PERIPH_* definitions for available
 *                encodings.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void prcm_periph_sleepdisable(uint32_t peripheral);

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

void prcm_periph_deepsleepenable(uint32_t peripheral);

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

void prcm_periph_deepsleepdisable(uint32_t peripheral);

#endif /* __ARCH_ARM_SRC_TIVA_CC13XX_CC13XX_PRCM_H */
