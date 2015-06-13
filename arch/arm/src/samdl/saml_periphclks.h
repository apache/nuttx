/****************************************************************************
 * arch/arm/src/samdl/saml_periphclks.h
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

#ifndef __ARCH_ARM_SRC_SAMDL_SAML_PERIPHCLKS_H
#define __ARCH_ARM_SRC_SAMDL_SAML_PERIPHCLKS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip/saml_mclk.h"

#ifdef CONFIG_ARCH_FAMILY_SAML21

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define sam_ahb_enableperiph(s)        modifyreg32(SAM_MCLK_AHBMASK,0,s)

#if 0 /* Not used, conflicting names */
#define sam_apba_enableperiph()        sam_ahb_enableperiph(MCLK_AHBMASK_APBA)
#define sam_apbb_enableperiph()        sam_ahb_enableperiph(MCLK_AHBMASK_APBB)
#define sam_apbc_enableperiph()        sam_ahb_enableperiph(MCLK_AHBMASK_APBC)
#define sam_apbd_enableperiph()        sam_ahb_enableperiph(MCLK_AHBMASK_APBD)
#define sam_apbe_enableperiph()        sam_ahb_enableperiph(MCLK_AHBMASK_APBE)
#define sam_dsu_enableperiph()         sam_ahb_enableperiph(MCLK_AHBMASK_DSU)
#define sam_nvmctrl_enableperiph()     sam_ahb_enableperiph(MCLK_AHBMASK_NVMCTRL)
#endif
#define sam_dmac_enableperiph()        sam_ahb_enableperiph(MCLK_AHBMASK_DMAC)
#if 0 /* Not used, conflicting names */
#define sam_usb_enableperiph()         sam_ahb_enableperiph(MCLK_AHBMASK_USB)
#define sam_pac_enableperiph()         sam_ahb_enableperiph(MCLK_AHBMASK_PAC)
#endif

#define sam_apba_enableperiph(s)       modifyreg32(SAM_MCLK_APBAMASK,0,s)

#define sam_pm_enableperiph()          sam_apba_enableperiph(MCLK_APBAMASK_PM)
#define sam_mclk_enableperiph()        sam_apba_enableperiph(MCLK_APBAMASK_MCLK)
#define sam_rstc_enableperiph()        sam_apba_enableperiph(MCLK_APBAMASK_RSTC)
#define sam_oscctrl_enableperiph()     sam_apba_enableperiph(MCLK_APBAMASK_OSCCTRL)
#define sam_osc32kctrl_enableperiph()  sam_apba_enableperiph(MCLK_APBAMASK_OSC32KCTRL)
#define sam_supc_enableperiph()        sam_apba_enableperiph(MCLK_APBAMASK_SUPC)
#define sam_gclk_enableperiph()        sam_apba_enableperiph(MCLK_APBAMASK_GCLK)
#define sam_wdt_enableperiph()         sam_apba_enableperiph(MCLK_APBAMASK_WDT)
#define sam_rtc_enableperiph()         sam_apba_enableperiph(MCLK_APBAMASK_RTC)
#define sam_eic_enableperiph()         sam_apba_enableperiph(MCLK_APBAMASK_EIC)
#define sam_port_enableperiph()        sam_apba_enableperiph(MCLK_APBAMASK_PORT)

#define sam_apbb_enableperiph(s)       modifyreg32(SAM_MCLK_APBBMASK,0,s)

#define sam_usb_enableperiph()         sam_apbb_enableperiph(MCLK_APBBMASK_USB)
#define sam_dsu_enableperiph()         sam_apbb_enableperiph(MCLK_APBBMASK_DSU)
#define sam_nvmctrl_enableperiph()     sam_apbb_enableperiph(MCLK_APBBMASK_NVMCTRL)

#define sam_apbc_enableperiph(s)       modifyreg32(SAM_MCLK_APBCMASK,0,s)

#define sam_sercom_enableperiph(n)     sam_apbc_enableperiph(MCLK_APBCMASK_SERCOM(n))
#define sam_sercom0_enableperiph()     sam_apbc_enableperiph(MCLK_APBCMASK_SERCOM0)
#define sam_sercom1_enableperiph()     sam_apbc_enableperiph(MCLK_APBCMASK_SERCOM1)
#define sam_sercom2_enableperiph()     sam_apbc_enableperiph(MCLK_APBCMASK_SERCOM2)
#define sam_sercom3_enableperiph()     sam_apbc_enableperiph(MCLK_APBCMASK_SERCOM3)
#define sam_sercom4_enableperiph()     sam_apbc_enableperiph(MCLK_APBCMASK_SERCOM4)
#define sam_tcc0_enableperiph()        sam_apbc_enableperiph(MCLK_APBCMASK_TCC0)
#define sam_tcc1_enableperiph()        sam_apbc_enableperiph(MCLK_APBCMASK_TCC1)
#define sam_tcc2_enableperiph()        sam_apbc_enableperiph(MCLK_APBCMASK_TCC2)
#define sam_tc0_enableperiph()         sam_apbc_enableperiph(MCLK_APBCMASK_TC0)
#define sam_tc1_enableperiph()         sam_apbc_enableperiph(MCLK_APBCMASK_TC1)
#define sam_tc2_enableperiph()         sam_apbc_enableperiph(MCLK_APBCMASK_TC2)
#define sam_tc3_enableperiph()         sam_apbc_enableperiph(MCLK_APBCMASK_TC3)
#define sam_dac_enableperiph()         sam_apbc_enableperiph(MCLK_APBCMASK_DAC)
#define sam_aes_enableperiph()         sam_apbc_enableperiph(MCLK_APBCMASK_AES)
#define sam_trng_enableperiph()        sam_apbc_enableperiph(MCLK_APBCMASK_TRNG)

#define sam_apbd_enableperiph(s)       modifyreg32(SAM_MCLK_APBDMASK,0,s)

#define sam_evsys_enableperiph()       sam_apbd_enableperiph(MCLK_APBDMASK_EVSYS)
#define sam_sercom5_enableperiph()     sam_apbd_enableperiph(MCLK_APBDMASK_SERCOM5)
#define sam_tc4_enableperiph()         sam_apbd_enableperiph(MCLK_APBDMASK_TC4)
#define sam_adc_enableperiph()         sam_apbd_enableperiph(MCLK_APBDMASK_ADC)
#define sam_ac_enableperiph()          sam_apbd_enableperiph(MCLK_APBDMASK_AC)
#define sam_ptc_enableperiph()         sam_apbd_enableperiph(MCLK_APBDMASK_PTC)
#define sam_opamp_enableperiph()       sam_apbd_enableperiph(MCLK_APBDMASK_OPAMP)
#define sam_ccl_enableperiph()         sam_apbd_enableperiph(MCLK_APBDMASK_CCL)

#define sam_apbe_enableperiph(s)       modifyreg32(SAM_MCLK_APBEMASK,0,s)

#define sam_pac_enableperiph()         sam_apbe_enableperiph(MCLK_APBEMASK_PAC)

#define sam_ahb_disableperiph(s)       modifyreg32(SAM_MCLK_AHBMASK,s,0)

#if 0 /* Not used, conflicting names */
#define sam_apba_disableperiph()       sam_ahb_disableperiph(MCLK_AHBMASK_APBA)
#define sam_apbb_disableperiph()       sam_ahb_disableperiph(MCLK_AHBMASK_APBB)
#define sam_apbc_disableperiph()       sam_ahb_disableperiph(MCLK_AHBMASK_APBC)
#define sam_apbd_disableperiph()       sam_ahb_disableperiph(MCLK_AHBMASK_APBD)
#define sam_apbe_disableperiph()       sam_ahb_disableperiph(MCLK_AHBMASK_APBE)
#define sam_dsu_disableperiph()        sam_ahb_disableperiph(MCLK_AHBMASK_DSU)
#define sam_nvmctrl_disableperiph()    sam_ahb_disableperiph(MCLK_AHBMASK_NVMCTRL)
#endif
#define sam_dmac_disableperiph()       sam_ahb_disableperiph(MCLK_AHBMASK_DMAC)
#if 0 /* Not used, conflicting names */
#define sam_usb_disableperiph()        sam_ahb_disableperiph(MCLK_AHBMASK_USB)
#define sam_pac_disableperiph()        sam_ahb_disableperiph(MCLK_AHBMASK_PAC)
#endif

#define sam_apba_disableperiph(s)      modifyreg32(SAM_MCLK_APBAMASK,s,0)

#define sam_pm_disableperiph()         sam_apba_disableperiph(MCLK_APBAMASK_PM)
#define sam_mclk_disableperiph()       sam_apba_disableperiph(MCLK_APBAMASK_MCLK)
#define sam_rstc_disableperiph()       sam_apba_disableperiph(MCLK_APBAMASK_RSTC)
#define sam_oscctrl_disableperiph()    sam_apba_disableperiph(MCLK_APBAMASK_OSCCTRL)
#define sam_osc32kctrl_disableperiph() sam_apba_disableperiph(MCLK_APBAMASK_OSC32KCTRL)
#define sam_supc_disableperiph()       sam_apba_disableperiph(MCLK_APBAMASK_SUPC)
#define sam_gclk_disableperiph()       sam_apba_disableperiph(MCLK_APBAMASK_GCLK)
#define sam_wdt_disableperiph()        sam_apba_disableperiph(MCLK_APBAMASK_WDT)
#define sam_rtc_disableperiph()        sam_apba_disableperiph(MCLK_APBAMASK_RTC)
#define sam_eic_disableperiph()        sam_apba_disableperiph(MCLK_APBAMASK_EIC)
#define sam_port_disableperiph()       sam_apba_disableperiph(MCLK_APBAMASK_PORT)

#define sam_apbb_disableperiph(s)      modifyreg32(SAM_MCLK_APBBMASK,s,0)

#define sam_usb_disableperiph()        sam_apbb_disableperiph(MCLK_APBBMASK_USB)
#define sam_dsu_disableperiph()        sam_apbb_disableperiph(MCLK_APBBMASK_DSU)
#define sam_nvmctrl_disableperiph()    sam_apbb_disableperiph(MCLK_APBBMASK_NVMCTRL)

#define sam_apbc_disableperiph(s)      modifyreg32(SAM_MCLK_APBCMASK,s,0)

#define sam_sercom_disableperiph(n)    sam_apbc_disableperiph(MCLK_APBCMASK_SERCOM(n))
#define sam_sercom0_disableperiph()    sam_apbc_disableperiph(MCLK_APBCMASK_SERCOM0)
#define sam_sercom1_disableperiph()    sam_apbc_disableperiph(MCLK_APBCMASK_SERCOM1)
#define sam_sercom2_disableperiph()    sam_apbc_disableperiph(MCLK_APBCMASK_SERCOM2)
#define sam_sercom3_disableperiph()    sam_apbc_disableperiph(MCLK_APBCMASK_SERCOM3)
#define sam_sercom4_disableperiph()    sam_apbc_disableperiph(MCLK_APBCMASK_SERCOM4)
#define sam_tcc0_disableperiph()       sam_apbc_disableperiph(MCLK_APBCMASK_TCC0)
#define sam_tcc1_disableperiph()       sam_apbc_disableperiph(MCLK_APBCMASK_TCC1)
#define sam_tcc2_disableperiph()       sam_apbc_disableperiph(MCLK_APBCMASK_TCC2)
#define sam_tc0_disableperiph()        sam_apbc_disableperiph(MCLK_APBCMASK_TC0)
#define sam_tc1_disableperiph()        sam_apbc_disableperiph(MCLK_APBCMASK_TC1)
#define sam_tc2_disableperiph()        sam_apbc_disableperiph(MCLK_APBCMASK_TC2)
#define sam_tc3_disableperiph()        sam_apbc_disableperiph(MCLK_APBCMASK_TC3)
#define sam_dac_disableperiph()        sam_apbc_disableperiph(MCLK_APBCMASK_DAC)
#define sam_aes_disableperiph()        sam_apbc_disableperiph(MCLK_APBCMASK_AES)
#define sam_trng_disableperiph()       sam_apbc_disableperiph(MCLK_APBCMASK_TRNG)

#define sam_apbd_disableperiph(s)      modifyreg32(SAM_MCLK_APBDMASK,s,0)

#define sam_evsys_disableperiph()      sam_apbd_disableperiph(MCLK_APBDMASK_EVSYS)
#define sam_sercom5_disableperiph()    sam_apbd_disableperiph(MCLK_APBDMASK_SERCOM5)
#define sam_tc4_disableperiph()        sam_apbd_disableperiph(MCLK_APBDMASK_TC4)
#define sam_adc_disableperiph()        sam_apbd_disableperiph(MCLK_APBDMASK_ADC)
#define sam_ac_disableperiph()         sam_apbd_disableperiph(MCLK_APBDMASK_AC)
#define sam_ptc_disableperiph()        sam_apbd_disableperiph(MCLK_APBDMASK_PTC)
#define sam_opamp_disableperiph()      sam_apbd_disableperiph(MCLK_APBDMASK_OPAMP)
#define sam_ccl_disableperiph()        sam_apbd_disableperiph(MCLK_APBDMASK_CCL)

#define sam_apbe_disableperiph(s)      modifyreg32(SAM_MCLK_APBEMASK,s,0)

#define sam_pac_disableperiph()        sam_apbe_disableperiph(MCLK_APBEMASK_PAC)

#define sam_ahb_isenabled(s)           (getreg32(SAM_MCLK_AHBMASK) & (s)) != 0)

#if 0 /* Not used, conflicting names */
#define sam_apba_isenabled()           sam_ahb_isenabled(MCLK_AHBMASK_APBA)
#define sam_apbb_isenabled()           sam_ahb_isenabled(MCLK_AHBMASK_APBB)
#define sam_apbc_isenabled()           sam_ahb_isenabled(MCLK_AHBMASK_APBC)
#define sam_apbd_isenabled()           sam_ahb_isenabled(MCLK_AHBMASK_APBD)
#define sam_apbe_isenabled()           sam_ahb_isenabled(MCLK_AHBMASK_APBE)
#define sam_dsu_isenabled()            sam_ahb_isenabled(MCLK_AHBMASK_DSU)
#define sam_nvmctrl_isenabled()        sam_ahb_isenabled(MCLK_AHBMASK_NVMCTRL)
#endif
#define sam_dmac_isenabled()           sam_ahb_isenabled(MCLK_AHBMASK_DMAC)
#if 0 /* Not used, conflicting names */
#define sam_usb_isenabled()            sam_ahb_isenabled(MCLK_AHBMASK_USB)
#define sam_pac_isenabled()            sam_ahb_isenabled(MCLK_AHBMASK_PAC)
#endif

#define sam_apba_isenabled(s)          (getreg32(SAM_MCLK_APBAMASK) & (s)) != 0)

#define sam_pm_isenabled()             sam_apba_isenabled(MCLK_APBAMASK_PM)
#define sam_mclk_isenabled()           sam_apba_isenabled(MCLK_APBAMASK_MCLK)
#define sam_rstc_isenabled()           sam_apba_isenabled(MCLK_APBAMASK_RSTC)
#define sam_oscctrl_isenabled()        sam_apba_isenabled(MCLK_APBAMASK_OSCCTRL)
#define sam_osc32kctrl_isenabled()     sam_apba_isenabled(MCLK_APBAMASK_OSC32KCTRL)
#define sam_supc_isenabled()           sam_apba_isenabled(MCLK_APBAMASK_SUPC)
#define sam_gclk_isenabled()           sam_apba_isenabled(MCLK_APBAMASK_GCLK)
#define sam_wdt_isenabled()            sam_apba_isenabled(MCLK_APBAMASK_WDT)
#define sam_rtc_isenabled()            sam_apba_isenabled(MCLK_APBAMASK_RTC)
#define sam_eic_isenabled()            sam_apba_isenabled(MCLK_APBAMASK_EIC)
#define sam_port_isenabled()           sam_apba_isenabled(MCLK_APBAMASK_PORT)

#define sam_apbb_isenabled(s)          (getreg32(SAM_MCLK_APBBMASK) & (s)) != 0)

#define sam_usb_isenabled()            sam_apbb_isenabled(MCLK_APBBMASK_USB)
#define sam_dsu_isenabled()            sam_apbb_isenabled(MCLK_APBBMASK_DSU)
#define sam_nvmctrl_isenabled()        sam_apbb_isenabled(MCLK_APBBMASK_NVMCTRL)

#define sam_apbc_isenabled(s)          (getreg32(SAM_MCLK_APBCMASK) & (s)) != 0)

#define sam_sercom_isenabled(n)        sam_apbc_isenabled(MCLK_APBCMASK_SERCOM(n))
#define sam_sercom0_isenabled()        sam_apbc_isenabled(MCLK_APBCMASK_SERCOM0)
#define sam_sercom1_isenabled()        sam_apbc_isenabled(MCLK_APBCMASK_SERCOM1)
#define sam_sercom2_isenabled()        sam_apbc_isenabled(MCLK_APBCMASK_SERCOM2)
#define sam_sercom3_isenabled()        sam_apbc_isenabled(MCLK_APBCMASK_SERCOM3)
#define sam_sercom4_isenabled()        sam_apbc_isenabled(MCLK_APBCMASK_SERCOM4)
#define sam_tcc0_isenabled()           sam_apbc_isenabled(MCLK_APBCMASK_TCC0)
#define sam_tcc1_isenabled()           sam_apbc_isenabled(MCLK_APBCMASK_TCC1)
#define sam_tcc2_isenabled()           sam_apbc_isenabled(MCLK_APBCMASK_TCC2)
#define sam_tc0_isenabled()            sam_apbc_isenabled(MCLK_APBCMASK_TC0)
#define sam_tc1_isenabled()            sam_apbc_isenabled(MCLK_APBCMASK_TC1)
#define sam_tc2_isenabled()            sam_apbc_isenabled(MCLK_APBCMASK_TC2)
#define sam_tc3_isenabled()            sam_apbc_isenabled(MCLK_APBCMASK_TC3)
#define sam_dac_isenabled()            sam_apbc_isenabled(MCLK_APBCMASK_DAC)
#define sam_aes_isenabled()            sam_apbc_isenabled(MCLK_APBCMASK_AES)
#define sam_trng_isenabled()           sam_apbc_isenabled(MCLK_APBCMASK_TRNG)

#define sam_apbd_isenabled(s)          (getreg32(SAM_MCLK_APBDMASK) & (s)) != 0)

#define sam_evsys_isenabled()          sam_apbd_isenabled(MCLK_APBDMASK_EVSYS)
#define sam_sercom5_isenabled()        sam_apbd_isenabled(MCLK_APBDMASK_SERCOM5)
#define sam_tc4_isenabled()            sam_apbd_isenabled(MCLK_APBDMASK_TC4)
#define sam_adc_isenabled()            sam_apbd_isenabled(MCLK_APBDMASK_ADC)
#define sam_ac_isenabled()             sam_apbd_isenabled(MCLK_APBDMASK_AC)
#define sam_ptc_isenabled()            sam_apbd_isenabled(MCLK_APBDMASK_PTC)
#define sam_opamp_isenabled()          sam_apbd_isenabled(MCLK_APBDMASK_OPAMP)
#define sam_ccl_isenabled()            sam_apbd_isenabled(MCLK_APBDMASK_CCL)

#define sam_apbe_isenabled(s)          (getreg32(SAM_MCLK_APBEMASK) & (s)) != 0)

#define sam_pac_isenabled()            sam_apbe_isenabled(MCLK_APBEMASK_PAC)

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
#endif /* CONFIG_ARCH_FAMILY_SAML21 */
#endif /* __ARCH_ARM_SRC_SAMDL_SAML_PERIPHCLKS_H */
