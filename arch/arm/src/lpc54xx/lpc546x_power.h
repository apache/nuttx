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

#include "arm_arch.h"
#include "hardware/lpc54_syscon.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Setting the bit corresponding in the PDRUNCFGSET0/1 register powers down the
 * selected component;  clearing the bit enables it.
 */

#define lpc54_powerup0(s)          putreg32((s), LPC54_SYSCON_PDRUNCFGCLR0)
#define lpc54_powerup1(s)          putreg32((s), LPC54_SYSCON_PDRUNCFGCLR1)
#define lpc54_powerdown0(s)        putreg32((s), LPC54_SYSCON_PDRUNCFGSET0)
#define lpc54_powerdown1(s)        putreg32((s), LPC54_SYSCON_PDRUNCFGSET1)
#define lpc54_ispowered0(s)        ((getreg32(LPC54_SYSCON_PDRUNCFG0) & (s)) == 0)
#define lpc54_ispowered1(s)        ((getreg32(LPC54_SYSCON_PDRUNCFG1) & (s)) == 0)

/* Enable power */

#define lpc54_fro_powerup()        lpc54_powerup0(SYSCON_PDRUNCFG0_FRO)
#define lpc54_ts_powerup()         lpc54_powerup0(SYSCON_PDRUNCFG0_TS)
#define lpc54_bodrst_powerup()     lpc54_powerup0(SYSCON_PDRUNCFG0_BODRST)
#define lpc54_bodintr_powerup()    lpc54_powerup0(SYSCON_PDRUNCFG0_BODINTR)
#define lpc54_vd2ana_powerup()     lpc54_powerup0(SYSCON_PDRUNCFG0_VD2ANA)
#define lpc54_adc0_powerup()       lpc54_powerup0(SYSCON_PDRUNCFG0_ADC0)
#define lpc54_sramx_powerup()      lpc54_powerup0(SYSCON_PDRUNCFG0_SRAMX)
#define lpc54_sam0_powerup()       lpc54_powerup0(SYSCON_PDRUNCFG0_SRAM0)
#define lpc54_sram123_powerup()    lpc54_powerup0(SYSCON_PDRUNCFG0_SRAM123)
#define lpc54_usbram_powerup()     lpc54_powerup0(SYSCON_PDRUNCFG0_USBRAM)
#define lpc54_vdda_powerup()       lpc54_powerup0(SYSCON_PDRUNCFG0_VDDA)
#define lpc54_wdtosc_powerup()     lpc54_powerup0(SYSCON_PDRUNCFG0_WDTOSC)
#define lpc54_usb0phy_powerup()    lpc54_powerup0(SYSCON_PDRUNCFG0_USB0PHY)
#define lpc54_syspll_powerup()     lpc54_powerup0(SYSCON_PDRUNCFG0_SYSPLL)
#define lpc54_vrefp_powerup()      lpc54_powerup0(SYSCON_PDRUNCFG0_VREFP)
#define lpc54_vd3_powerup()        lpc54_powerup0(SYSCON_PDRUNCFG0_VD3)
#define lpc54_vd4_powerup()        lpc54_powerup0(SYSCON_PDRUNCFG0_VD4)
#define lpc54_vd5_powerup()        lpc54_powerup0(SYSCON_PDRUNCFG0_VD5)
#define lpc54_vd6_powerup()        lpc54_powerup0(SYSCON_PDRUNCFG0_VD6)

#define lpc54_usb1phy_powerup()    lpc54_powerup1(SYSCON_PDRUNCFG1_USB1PHY)
#define lpc54_usb1pll_powerup()    lpc54_powerup1(SYSCON_PDRUNCFG1_USB1PLL)
#define lpc54_audpll_powerup()     lpc54_powerup1(SYSCON_PDRUNCFG1_AUDPLL)
#define lpc54_sysosc_powerup()     lpc54_powerup1(SYSCON_PDRUNCFG1_SYSOSC)
#define lpc54_eeprom_powerup()     lpc54_powerup1(SYSCON_PDRUNCFG1_EEPROM)
#define lpc54_rng_powerup()        lpc54_powerup1(SYSCON_PDRUNCFG1_RNG)

/* Disable power */

#define lpc54_fro_powerdown()      lpc54_powerdown0(SYSCON_PDRUNCFG0_FRO)
#define lpc54_ts_powerdown()       lpc54_powerdown0(SYSCON_PDRUNCFG0_TS)
#define lpc54_bodrst_powerdown()   lpc54_powerdown0(SYSCON_PDRUNCFG0_BODRST)
#define lpc54_bodintr_powerdown()  lpc54_powerdown0(SYSCON_PDRUNCFG0_BODINTR)
#define lpc54_vd2ana_powerdown()   lpc54_powerdown0(SYSCON_PDRUNCFG0_VD2ANA)
#define lpc54_adc0_powerdown()     lpc54_powerdown0(SYSCON_PDRUNCFG0_ADC0)
#define lpc54_sramx_powerdown()    lpc54_powerdown0(SYSCON_PDRUNCFG0_SRAMX)
#define lpc54_sam0_powerdown()     lpc54_powerdown0(SYSCON_PDRUNCFG0_SRAM0)
#define lpc54_sram123_powerdown()  lpc54_powerdown0(SYSCON_PDRUNCFG0_SRAM123)
#define lpc54_usbram_powerdown()   lpc54_powerdown0(SYSCON_PDRUNCFG0_USBRAM)
#define lpc54_vdda_powerdown()     lpc54_powerdown0(SYSCON_PDRUNCFG0_VDDA)
#define lpc54_wdtosc_powerdown()   lpc54_powerdown0(SYSCON_PDRUNCFG0_WDTOSC)
#define lpc54_usb0phy_powerdown()  lpc54_powerdown0(SYSCON_PDRUNCFG0_USB0PHY)
#define lpc54_syspll_powerdown()   lpc54_powerdown0(SYSCON_PDRUNCFG0_SYSPLL)
#define lpc54_vrefp_powerdown()    lpc54_powerdown0(SYSCON_PDRUNCFG0_VREFP)
#define lpc54_vd3_powerdown()      lpc54_powerdown0(SYSCON_PDRUNCFG0_VD3)
#define lpc54_vd4_powerdown()      lpc54_powerdown0(SYSCON_PDRUNCFG0_VD4)
#define lpc54_vd5_powerdown()      lpc54_powerdown0(SYSCON_PDRUNCFG0_VD5)
#define lpc54_vd6_powerdown()      lpc54_powerdown0(SYSCON_PDRUNCFG0_VD6)

#define lpc54_usb1phy_powerdown()  lpc54_powerdown1(SYSCON_PDRUNCFG1_USB1PHY)
#define lpc54_usb1pll_powerdown()  lpc54_powerdown1(SYSCON_PDRUNCFG1_USB1PLL)
#define lpc54_audpll_powerdown()   lpc54_powerdown1(SYSCON_PDRUNCFG1_AUDPLL)
#define lpc54_sysosc_powerdown()   lpc54_powerdown1(SYSCON_PDRUNCFG1_SYSOSC)
#define lpc54_eeprom_powerdown()   lpc54_powerdown1(SYSCON_PDRUNCFG1_EEPROM)
#define lpc54_rng_powerdown()      lpc54_powerdown1(SYSCON_PDRUNCFG1_RNG)

/* Test if power is enabled */

#define lpc54_fro_ispowered()      lpc54_ispowered0(SYSCON_PDRUNCFG0_FRO)
#define lpc54_ts_ispowered()       lpc54_ispowered0(SYSCON_PDRUNCFG0_TS)
#define lpc54_bodrst_ispowered()   lpc54_ispowered0(SYSCON_PDRUNCFG0_BODRST)
#define lpc54_bodintr_ispowered()  lpc54_ispowered0(SYSCON_PDRUNCFG0_BODINTR)
#define lpc54_vd2ana_ispowered()   lpc54_ispowered0(SYSCON_PDRUNCFG0_VD2ANA)
#define lpc54_adc0_ispowered()     lpc54_ispowered0(SYSCON_PDRUNCFG0_ADC0)
#define lpc54_sramx_ispowered()    lpc54_ispowered0(SYSCON_PDRUNCFG0_SRAMX)
#define lpc54_sam0_ispowered()     lpc54_ispowered0(SYSCON_PDRUNCFG0_SRAM0)
#define lpc54_sram123_ispowered()  lpc54_ispowered0(SYSCON_PDRUNCFG0_SRAM123)
#define lpc54_usbram_ispowered()   lpc54_ispowered0(SYSCON_PDRUNCFG0_USBRAM)
#define lpc54_vdda_ispowered()     lpc54_ispowered0(SYSCON_PDRUNCFG0_VDDA)
#define lpc54_wdtosc_ispowered()   lpc54_ispowered0(SYSCON_PDRUNCFG0_WDTOSC)
#define lpc54_usb0phy_ispowered()  lpc54_ispowered0(SYSCON_PDRUNCFG0_USB0PHY)
#define lpc54_syspll_ispowered()   lpc54_ispowered0(SYSCON_PDRUNCFG0_SYSPLL)
#define lpc54_vrefp_ispowered()    lpc54_ispowered0(SYSCON_PDRUNCFG0_VREFP)
#define lpc54_vd3_ispowered()      lpc54_ispowered0(SYSCON_PDRUNCFG0_VD3)
#define lpc54_vd4_ispowered()      lpc54_ispowered0(SYSCON_PDRUNCFG0_VD4)
#define lpc54_vd5_ispowered()      lpc54_ispowered0(SYSCON_PDRUNCFG0_VD5)
#define lpc54_vd6_ispowered()      lpc54_ispowered0(SYSCON_PDRUNCFG0_VD6)

#define lpc54_usb1phy_ispowered()  lpc54_ispowered1(SYSCON_PDRUNCFG1_USB1PHY)
#define lpc54_usb1pll_ispowered()  lpc54_ispowered1(SYSCON_PDRUNCFG1_USB1PLL)
#define lpc54_audpll_ispowered()   lpc54_ispowered1(SYSCON_PDRUNCFG1_AUDPLL)
#define lpc54_sysosc_ispowered()   lpc54_ispowered1(SYSCON_PDRUNCFG1_SYSOSC)
#define lpc54_eeprom_ispowered()   lpc54_ispowered1(SYSCON_PDRUNCFG1_EEPROM)
#define lpc54_rng_ispowered()      lpc54_ispowered1(SYSCON_PDRUNCFG1_RNG)

#endif /* __ARCH_ARM_SRC_LPC54XX_LPC546X_PERIPHPOWER_H */
