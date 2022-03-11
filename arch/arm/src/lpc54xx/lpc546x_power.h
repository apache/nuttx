/****************************************************************************
 * arch/arm/src/lpc54xx/lpc546x_power.h
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

#ifndef __ARCH_ARM_SRC_LPC54XX_LPC546X_POWER_H
#define __ARCH_ARM_SRC_LPC54XX_LPC546X_POWER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "arm_internal.h"
#include "hardware/lpc54_syscon.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Setting the bit corresponding in the PDRUNCFGSET0/1 register powers down
 * the selected component;  clearing the bit enables it.
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

#endif /* __ARCH_ARM_SRC_LPC54XX_LPC546X_POWER_H */
