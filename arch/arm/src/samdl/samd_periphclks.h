/****************************************************************************
 * arch/arm/src/samdl/samd_periphclks.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_SAMDL_SAMD_PERIPHCLKS_H
#define __ARCH_ARM_SRC_SAMDL_SAMD_PERIPHCLKS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip/samd_pm.h"

#if defined(CONFIG_ARCH_FAMILY_SAMD20) || defined(CONFIG_ARCH_FAMILY_SAMD21)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define sam_apba_enableperiph(s)      modifyreg32(SAM_PM_APBAMASK,0,s)

#define sam_pac0_enableperiph()       sam_apba_enableperiph(PM_APBAMASK_PAC0)
#define sam_pm_enableperiph()         sam_apba_enableperiph(PM_APBAMASK_PM)
#define sam_sysctrl_enableperiph()    sam_apba_enableperiph(PM_APBAMASK_SYSCTRL)
#define sam_gclk_enableperiph()       sam_apba_enableperiph(PM_APBAMASK_GCLK)
#define sam_wdt_enableperiph()        sam_apba_enableperiph(PM_APBAMASK_WDT)
#define sam_rtc_enableperiph()        sam_apba_enableperiph(PM_APBAMASK_RTC)
#define sam_eic_enableperiph()        sam_apba_enableperiph(PM_APBAMASK_EIC)

#define sam_apbb_enableperiph(s)      modifyreg32(SAM_PM_APBBMASK,0,s)

#define sam_pac1_enableperiph()       sam_apbb_enableperiph(PM_APBBMASK_PAC1)
#define sam_dsu_enableperiph()        sam_apbb_enableperiph(PM_APBBMASK_DSU)
#define sam_nvmctrl_enableperiph()    sam_apbb_enableperiph(PM_APBBMASK_NVMCTRL)
#define sam_port_enableperiph()       sam_apbb_enableperiph(PM_APBBMASK_PORT)

#ifdef CONFIG_ARCH_FAMILY_SAMD21
#  define sam_dmac_enableperiph()     sam_apbb_enableperiph(PM_APBBMASK_DMAC)
#  define sam_usb_enableperiph()      sam_apbb_enableperiph(PM_APBBMASK_USB)
#endif

#define sam_apbc_enableperiph(s)      modifyreg32(SAM_PM_APBCMASK,0,s)

#define sam_pac2_enableperiph()       sam_apbc_enableperiph(PM_APBCMASK_PAC2)
#define sam_devsys_enableperiph()     sam_apbc_enableperiph(PM_APBCMASK_EVSYS)
#define sam_sercom_enableperiph(n)    sam_apbc_enableperiph(PM_APBCMASK_SERCOM(n))
#define sam_sercom0_enableperiph()    sam_apbc_enableperiph(PM_APBCMASK_SERCOM0)
#define sam_sercom1_enableperiph()    sam_apbc_enableperiph(PM_APBCMASK_SERCOM1)
#define sam_sercom2_enableperiph()    sam_apbc_enableperiph(PM_APBCMASK_SERCOM2)
#define sam_sercom3_enableperiph()    sam_apbc_enableperiph(PM_APBCMASK_SERCOM3)
#define sam_sercom4_enableperiph()    sam_apbc_enableperiph(PM_APBCMASK_SERCOM4)
#define sam_sercom5_enableperiph()    sam_apbc_enableperiph(PM_APBCMASK_SERCOM5)

#ifdef CONFIG_ARCH_FAMILY_SAMD20
#  define sam_tc0_enableperiph()      sam_apbc_enableperiph(PM_APBCMASK_TC0)
#  define sam_tc1_enableperiph()      sam_apbc_enableperiph(PM_APBCMASK_TC1)
#  define sam_tc2_enableperiph()      sam_apbc_enableperiph(PM_APBCMASK_TC2)
#endif

#ifdef CONFIG_ARCH_FAMILY_SAMD21
#  define sam_tcc0_enableperiph()     sam_apbc_enableperiph(PM_APBCMASK_TCC0)
#  define sam_tcc1_enableperiph()     sam_apbc_enableperiph(PM_APBCMASK_TCC1)
#  define sam_tcc2_enableperiph()     sam_apbc_enableperiph(PM_APBCMASK_TCC2)
#endif

#define sam_tc3_enableperiph()        sam_apbc_enableperiph(PM_APBCMASK_TC3)
#define sam_tc4_enableperiph()        sam_apbc_enableperiph(PM_APBCMASK_TC4)
#define sam_tc5_enableperiph()        sam_apbc_enableperiph(PM_APBCMASK_TC5)
#define sam_tc6_enableperiph()        sam_apbc_enableperiph(PM_APBCMASK_TC6)
#define sam_tc7_enableperiph()        sam_apbc_enableperiph(PM_APBCMASK_TC7)
#define sam_adc_enableperiph()        sam_apbc_enableperiph(PM_APBCMASK_ADC)
#define sam_ac_enableperiph()         sam_apbc_enableperiph(PM_APBCMASK_AC)
#define sam_dac_enableperiph()        sam_apbc_enableperiph(PM_APBCMASK_DAC)
#define sam_ptc_enableperiph()        sam_apbc_enableperiph(PM_APBCMASK_PTC)

#ifdef CONFIG_ARCH_FAMILY_SAMD21
#  define sam_i2s_enableperiph()      sam_apbc_enableperiph(PM_APBBMASK_I2S)
#endif

#define sam_apba_disableperiph(s)     modifyreg32(SAM_PM_APBAMASK,s,0)

#define sam_pac0_disableperiph()      sam_apba_disableperiph(PM_APBAMASK_PAC0)
#define sam_pm_disableperiph()        sam_apba_disableperiph(PM_APBAMASK_PM)
#define sam_sysctrl_disableperiph()   sam_apba_disableperiph(PM_APBAMASK_SYSCTRL)
#define sam_gclk_disableperiph()      sam_apba_disableperiph(PM_APBAMASK_GCLK)
#define sam_wdt_disableperiph()       sam_apba_disableperiph(PM_APBAMASK_WDT)
#define sam_rtc_disableperiph()       sam_apba_disableperiph(PM_APBAMASK_RTC)
#define sam_eic_disableperiph()       sam_apba_disableperiph(PM_APBAMASK_EIC)

#define sam_apbb_disableperiph(s)     modifyreg32(SAM_PM_APBBMASK,s,0)

#define sam_pac1_disableperiph()      sam_apbb_disableperiph(PM_APBBMASK_PAC1)
#define sam_dsu_disableperiph()       sam_apbb_disableperiph(PM_APBBMASK_DSU)
#define sam_nvmctrl_disableperiph()   sam_apbb_disableperiph(PM_APBBMASK_NVMCTRL)
#define sam_port_disableperiph()      sam_apbb_disableperiph(PM_APBBMASK_PORT)

#ifdef CONFIG_ARCH_FAMILY_SAMD21
#  define sam_dmac_disableperiph()    sam_apbb_disableperiph(PM_APBBMASK_DMAC)
#  define sam_usb_disableperiph()     sam_apbb_disableperiph(PM_APBBMASK_USB)
#endif

#define sam_apbc_disableperiph(s)     modifyreg32(SAM_PM_APBCMASK,s,0)

#define sam_pac2_disableperiph()      sam_apbc_disableperiph(PM_APBCMASK_PAC2)
#define sam_devsys_disableperiph()    sam_apbc_disableperiph(PM_APBCMASK_EVSYS)
#define sam_sercom_disableperiph(n)   sam_apbc_disableperiph(PM_APBCMASK_SERCOM(n))
#define sam_sercom0_disableperiph()   sam_apbc_disableperiph(PM_APBCMASK_SERCOM0)
#define sam_sercom1_disableperiph()   sam_apbc_disableperiph(PM_APBCMASK_SERCOM1)
#define sam_sercom2_disableperiph()   sam_apbc_disableperiph(PM_APBCMASK_SERCOM2)
#define sam_sercom3_disableperiph()   sam_apbc_disableperiph(PM_APBCMASK_SERCOM3)
#define sam_sercom4_disableperiph()   sam_apbc_disableperiph(PM_APBCMASK_SERCOM4)
#define sam_sercom5_disableperiph()   sam_apbc_disableperiph(PM_APBCMASK_SERCOM5)

#ifdef CONFIG_ARCH_FAMILY_SAMD20
#  define sam_tc0_disableperiph()     sam_apbc_disableperiph(PM_APBCMASK_TC0)
#  define sam_tc1_disableperiph()     sam_apbc_disableperiph(PM_APBCMASK_TC1)
#  define sam_tc2_disableperiph()     sam_apbc_disableperiph(PM_APBCMASK_TC2)
#endif

#ifdef CONFIG_ARCH_FAMILY_SAMD21
#  define sam_tcc0_disableperiph()    sam_apbc_disableperiph(PM_APBCMASK_TCC0)
#  define sam_tcc1_disableperiph()    sam_apbc_disableperiph(PM_APBCMASK_TCC1)
#  define sam_tcc2_disableperiph()    sam_apbc_disableperiph(PM_APBCMASK_TCC2)
#endif

#define sam_tc3_disableperiph()       sam_apbc_disableperiph(PM_APBCMASK_TC3)
#define sam_tc4_disableperiph()       sam_apbc_disableperiph(PM_APBCMASK_TC4)
#define sam_tc5_disableperiph()       sam_apbc_disableperiph(PM_APBCMASK_TC5)
#define sam_tc6_disableperiph()       sam_apbc_disableperiph(PM_APBCMASK_TC6)
#define sam_tc7_disableperiph()       sam_apbc_disableperiph(PM_APBCMASK_TC7)
#define sam_adc_disableperiph()       sam_apbc_disableperiph(PM_APBCMASK_ADC)
#define sam_ac_disableperiph()        sam_apbc_disableperiph(PM_APBCMASK_AC)
#define sam_dac_disableperiph()       sam_apbc_disableperiph(PM_APBCMASK_DAC)
#define sam_ptc_disableperiph()       sam_apbc_disableperiph(PM_APBCMASK_PTC)

#ifdef CONFIG_ARCH_FAMILY_SAMD21
#  define sam_i2s_disableperiph()     sam_apbc_disableperiph(PM_APBBMASK_I2S)
#endif

#define sam_apba_isenabled(s)         (getreg32(SAM_PM_APBAMASK) & (s)) != 0)

#define sam_pac0_isenabled()          sam_apba_isenabled(PM_APBAMASK_PAC0)
#define sam_pm_isenabled()            sam_apba_isenabled(PM_APBAMASK_PM)
#define sam_sysctrl_isenabled()       sam_apba_isenabled(PM_APBAMASK_SYSCTRL)
#define sam_gclk_isenabled()          sam_apba_isenabled(PM_APBAMASK_GCLK)
#define sam_wdt_isenabled()           sam_apba_isenabled(PM_APBAMASK_WDT)
#define sam_rtc_isenabled()           sam_apba_isenabled(PM_APBAMASK_RTC)
#define sam_eic_isenabled()           sam_apba_isenabled(PM_APBAMASK_EIC)

#define sam_apbb_isenabled(s)         (getreg32(SAM_PM_APBBMASK) & (s)) != 0)

#define sam_pac1_isenabled()          sam_apbb_isenabled(PM_APBBMASK_PAC1)
#define sam_dsu_isenabled()           sam_apbb_isenabled(PM_APBBMASK_DSU)
#define sam_nvmctrl_isenabled()       sam_apbb_isenabled(PM_APBBMASK_NVMCTRL)
#define sam_port_isenabled()          sam_apbb_isenabled(PM_APBBMASK_PORT)

#ifdef CONFIG_ARCH_FAMILY_SAMD21
#  define sam_dmac_isenabled()        sam_apbb_isenabled(PM_APBBMASK_DMAC)
#  define sam_usb_isenabled()         sam_apbb_isenabled(PM_APBBMASK_USB)
#endif

#define sam_apbc_isenabled(s)         (getreg32(SAM_PM_APBCMASK) & (s)) != 0)

#define sam_pac2_isenabled()          sam_apbc_isenabled(PM_APBCMASK_PAC2)
#define sam_devsys_isenabled()        sam_apbc_isenabled(PM_APBCMASK_EVSYS)
#define sam_sercom_isenabled(n)       sam_apbc_isenabled(PM_APBCMASK_SERCOM(n))
#define sam_sercom0_isenabled()       sam_apbc_isenabled(PM_APBCMASK_SERCOM0)
#define sam_sercom1_isenabled()       sam_apbc_isenabled(PM_APBCMASK_SERCOM1)
#define sam_sercom2_isenabled()       sam_apbc_isenabled(PM_APBCMASK_SERCOM2)
#define sam_sercom3_isenabled()       sam_apbc_isenabled(PM_APBCMASK_SERCOM3)
#define sam_sercom4_isenabled()       sam_apbc_isenabled(PM_APBCMASK_SERCOM4)
#define sam_sercom5_isenabled()       sam_apbc_isenabled(PM_APBCMASK_SERCOM5)

#ifdef CONFIG_ARCH_FAMILY_SAMD20
#  define sam_tc0_isenabled()         sam_apbc_isenabled(PM_APBCMASK_TC0)
#  define sam_tc1_isenabled()         sam_apbc_isenabled(PM_APBCMASK_TC1)
#  define sam_tc2_isenabled()         sam_apbc_isenabled(PM_APBCMASK_TC2)
#endif

#ifdef CONFIG_ARCH_FAMILY_SAMD21
#  define sam_tcc0_isenabled()        sam_apbc_isenabled(PM_APBCMASK_TCC0)
#  define sam_tcc1_isenabled()        sam_apbc_isenabled(PM_APBCMASK_TCC1)
#  define sam_tcc2_isenabled()        sam_apbc_isenabled(PM_APBCMASK_TCC2)
#endif

#define sam_tc3_isenabled()           sam_apbc_isenabled(PM_APBCMASK_TC3)
#define sam_tc4_isenabled()           sam_apbc_isenabled(PM_APBCMASK_TC4)
#define sam_tc5_isenabled()           sam_apbc_isenabled(PM_APBCMASK_TC5)
#define sam_tc6_isenabled()           sam_apbc_isenabled(PM_APBCMASK_TC6)
#define sam_tc7_isenabled()           sam_apbc_isenabled(PM_APBCMASK_TC7)
#define sam_adc_isenabled()           sam_apbc_isenabled(PM_APBCMASK_ADC)
#define sam_ac_isenabled()            sam_apbc_isenabled(PM_APBCMASK_AC)
#define sam_dac_isenabled()           sam_apbc_isenabled(PM_APBCMASK_DAC)
#define sam_ptc_isenabled()           sam_apbc_isenabled(PM_APBCMASK_PTC)

#ifdef CONFIG_ARCH_FAMILY_SAMD21
#  define sam_i2s_isenabled()         sam_apbc_isenabled(PM_APBBMASK_I2S)
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* CONFIG_ARCH_FAMILY_SAMD20 || CONFIG_ARCH_FAMILY_SAMD21 */
#endif /* __ARCH_ARM_SRC_SAMDL_SAMD_PERIPHCLKS_H */
