/************************************************************************************
 * arch/arm/src/lpc54xx/lpc546x_power.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC54XX_LPC546X_PERIPHPOWER_H
#define __ARCH_ARM_SRC_LPC54XX_LPC546X_PERIPHPOWER_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "up_arch.h"
#include "chip/lpc54_syscon.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Setting the bit corresponding in the PDRUNCFGSET0/1 register powers down the
 * selected component;  clearling the bit enables it.
 */

#define lpc54_enable0(s)           putreg32((s), LPC54_SYSCON_PDRUNCFGCLR0)
#define lpc54_enable1(s)           putreg32((s), LPC54_SYSCON_PDRUNCFGCLR1)
#define lpc54_disable0(s)          putreg32((s), LPC54_SYSCON_PDRUNCFGSET0)
#define lpc54_disable1(s)          putreg32((s), LPC54_SYSCON_PDRUNCFGSET1)
#define lpc54_isenabled0(s)        ((getreg32(LPC54_SYSCON_PDRUNCFG0) & (s)) == 0)
#define lpc54_isenabled1(s)        ((getreg32(LPC54_SYSCON_PDRUNCFG1) & (s)) == 0)

/* Enable power */

#define lpc54_fro_enable()         lpc54_enable0(SYSCON_PDRUNCFG0_FRO)
#define lpc54_ts_enable()          lpc54_enable0(SYSCON_PDRUNCFG0_TS)
#define lpc54_bodrst_enable()      lpc54_enable0(SYSCON_PDRUNCFG0_BODRST)
#define lpc54_bodintr_enable()     lpc54_enable0(SYSCON_PDRUNCFG0_BODINTR)
#define lpc54_vd2ana_enable()      lpc54_enable0(SYSCON_PDRUNCFG0_VD2ANA)
#define lpc54_adc0_enable()        lpc54_enable0(SYSCON_PDRUNCFG0_ADC0)
#define lpc54_sramx_enable()       lpc54_enable0(SYSCON_PDRUNCFG0_SRAMX)
#define lpc54_sam0_enable()        lpc54_enable0(SYSCON_PDRUNCFG0_SRAM0)
#define lpc54_sram123_enable()     lpc54_enable0(SYSCON_PDRUNCFG0_SRAM123)
#define lpc54_usbram_enable()      lpc54_enable0(SYSCON_PDRUNCFG0_USBRAM)
#define lpc54_vdda_enable()        lpc54_enable0(SYSCON_PDRUNCFG0_VDDA)
#define lpc54_wdtosc_enable()      lpc54_enable0(SYSCON_PDRUNCFG0_WDTOSC)
#define lpc54_usb0phy_enable()     lpc54_enable0(SYSCON_PDRUNCFG0_USB0PHY)
#define lpc54_syspll_enable()      lpc54_enable0(SYSCON_PDRUNCFG0_SYSPLL)
#define lpc54_vrefp_enable()       lpc54_enable0(SYSCON_PDRUNCFG0_VREFP)
#define lpc54_vd3_enable()         lpc54_enable0(SYSCON_PDRUNCFG0_VD3)
#define lpc54_vd4_enable()         lpc54_enable0(SYSCON_PDRUNCFG0_VD4)
#define lpc54_vd5_enable()         lpc54_enable0(SYSCON_PDRUNCFG0_VD5)
#define lpc54_vd6_enable()         lpc54_enable0(SYSCON_PDRUNCFG0_VD6)

#define lpc54_usb1phy_enable()     lpc54_enable1(SYSCON_PDRUNCFG1_USB1PHY)
#define lpc54_usb1pll_enable()     lpc54_enable1(SYSCON_PDRUNCFG1_USB1PLL)
#define lpc54_audpll_enable()      lpc54_enable1(SYSCON_PDRUNCFG1_AUDPLL)
#define lpc54_sysosc_enable()      lpc54_enable1(SYSCON_PDRUNCFG1_SYSOSC)
#define lpc54_eeprom_enable()      lpc54_enable1(SYSCON_PDRUNCFG1_EEPROM)
#define lpc54_rng_enable()         lpc54_enable1(SYSCON_PDRUNCFG1_RNG)

/* Disable power */

#define lpc54_fro_disable()        lpc54_disable0(SYSCON_PDRUNCFG0_FRO)
#define lpc54_ts_disable()         lpc54_disable0(SYSCON_PDRUNCFG0_TS)
#define lpc54_bodrst_disable()     lpc54_disable0(SYSCON_PDRUNCFG0_BODRST)
#define lpc54_bodintr_disable()    lpc54_disable0(SYSCON_PDRUNCFG0_BODINTR)
#define lpc54_vd2ana_disable()     lpc54_disable0(SYSCON_PDRUNCFG0_VD2ANA)
#define lpc54_adc0_disable()       lpc54_disable0(SYSCON_PDRUNCFG0_ADC0)
#define lpc54_sramx_disable()      lpc54_disable0(SYSCON_PDRUNCFG0_SRAMX)
#define lpc54_sam0_disable()       lpc54_disable0(SYSCON_PDRUNCFG0_SRAM0)
#define lpc54_sram123_disable()    lpc54_disable0(SYSCON_PDRUNCFG0_SRAM123)
#define lpc54_usbram_disable()     lpc54_disable0(SYSCON_PDRUNCFG0_USBRAM)
#define lpc54_vdda_disable()       lpc54_disable0(SYSCON_PDRUNCFG0_VDDA)
#define lpc54_wdtosc_disable()     lpc54_disable0(SYSCON_PDRUNCFG0_WDTOSC)
#define lpc54_usb0phy_disable()    lpc54_disable0(SYSCON_PDRUNCFG0_USB0PHY)
#define lpc54_syspll_disable()     lpc54_disable0(SYSCON_PDRUNCFG0_SYSPLL)
#define lpc54_vrefp_disable()      lpc54_disable0(SYSCON_PDRUNCFG0_VREFP)
#define lpc54_vd3_disable()        lpc54_disable0(SYSCON_PDRUNCFG0_VD3)
#define lpc54_vd4_disable()        lpc54_disable0(SYSCON_PDRUNCFG0_VD4)
#define lpc54_vd5_disable()        lpc54_disable0(SYSCON_PDRUNCFG0_VD5)
#define lpc54_vd6_disable()        lpc54_disable0(SYSCON_PDRUNCFG0_VD6)

#define lpc54_usb1phy_disable()    lpc54_disable1(SYSCON_PDRUNCFG1_USB1PHY)
#define lpc54_usb1pll_disable()    lpc54_disable1(SYSCON_PDRUNCFG1_USB1PLL)
#define lpc54_audpll_disable()     lpc54_disable1(SYSCON_PDRUNCFG1_AUDPLL)
#define lpc54_sysosc_disable()     lpc54_disable1(SYSCON_PDRUNCFG1_SYSOSC)
#define lpc54_eeprom_disable()     lpc54_disable1(SYSCON_PDRUNCFG1_EEPROM)
#define lpc54_rng_disable()        lpc54_disable1(SYSCON_PDRUNCFG1_RNG)

/* Test if power is enabled */

#define lpc54_fro_isenabled()      lpc54_isenabled0(SYSCON_PDRUNCFG0_FRO)
#define lpc54_ts_isenabled()       lpc54_isenabled0(SYSCON_PDRUNCFG0_TS)
#define lpc54_bodrst_isenabled()   lpc54_isenabled0(SYSCON_PDRUNCFG0_BODRST)
#define lpc54_bodintr_isenabled()  lpc54_isenabled0(SYSCON_PDRUNCFG0_BODINTR)
#define lpc54_vd2ana_isenabled()   lpc54_isenabled0(SYSCON_PDRUNCFG0_VD2ANA)
#define lpc54_adc0_isenabled()     lpc54_isenabled0(SYSCON_PDRUNCFG0_ADC0)
#define lpc54_sramx_isenabled()    lpc54_isenabled0(SYSCON_PDRUNCFG0_SRAMX)
#define lpc54_sam0_isenabled()     lpc54_isenabled0(SYSCON_PDRUNCFG0_SRAM0)
#define lpc54_sram123_isenabled()  lpc54_isenabled0(SYSCON_PDRUNCFG0_SRAM123)
#define lpc54_usbram_isenabled()   lpc54_isenabled0(SYSCON_PDRUNCFG0_USBRAM)
#define lpc54_vdda_isenabled()     lpc54_isenabled0(SYSCON_PDRUNCFG0_VDDA)
#define lpc54_wdtosc_isenabled()   lpc54_isenabled0(SYSCON_PDRUNCFG0_WDTOSC)
#define lpc54_usb0phy_isenabled()  lpc54_isenabled0(SYSCON_PDRUNCFG0_USB0PHY)
#define lpc54_syspll_isenabled()   lpc54_isenabled0(SYSCON_PDRUNCFG0_SYSPLL)
#define lpc54_vrefp_isenabled()    lpc54_isenabled0(SYSCON_PDRUNCFG0_VREFP)
#define lpc54_vd3_isenabled()      lpc54_isenabled0(SYSCON_PDRUNCFG0_VD3)
#define lpc54_vd4_isenabled()      lpc54_isenabled0(SYSCON_PDRUNCFG0_VD4)
#define lpc54_vd5_isenabled()      lpc54_isenabled0(SYSCON_PDRUNCFG0_VD5)
#define lpc54_vd6_isenabled()      lpc54_isenabled0(SYSCON_PDRUNCFG0_VD6)

#define lpc54_usb1phy_isenabled()  lpc54_isenabled1(SYSCON_PDRUNCFG1_USB1PHY)
#define lpc54_usb1pll_isenabled()  lpc54_isenabled1(SYSCON_PDRUNCFG1_USB1PLL)
#define lpc54_audpll_isenabled()   lpc54_isenabled1(SYSCON_PDRUNCFG1_AUDPLL)
#define lpc54_sysosc_isenabled()   lpc54_isenabled1(SYSCON_PDRUNCFG1_SYSOSC)
#define lpc54_eeprom_isenabled()   lpc54_isenabled1(SYSCON_PDRUNCFG1_EEPROM)
#define lpc54_rng_isenabled()      lpc54_isenabled1(SYSCON_PDRUNCFG1_RNG)

#endif /* __ARCH_ARM_SRC_LPC54XX_LPC546X_PERIPHPOWER_H */
