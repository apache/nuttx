/****************************************************************************
 * arch/arm/src/samd5e5/sam_periphclks.h
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

#ifndef __ARCH_ARM_SRC_SAMD5E5_SAM_PERIPHCLKS_H
#define __ARCH_ARM_SRC_SAMD5E5_SAM_PERIPHCLKS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/sam_mclk.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define sam_ahb_enableperiph(s)            modifyreg32(SAM_MCLK_AHBMASK,0,s)

#define sam_ahb_hpb0_enableperiph()        sam_ahb_enableperiph(MCLK_AHBMASK_HPB0)
#define sam_ahb_hpb1_enableperiph()        sam_ahb_enableperiph(MCLK_AHBMASK_HPB1)
#define sam_ahb_hpb2_enableperiph()        sam_ahb_enableperiph(MCLK_AHBMASK_HPB2)
#define sam_ahb_hpb3_enableperiph()        sam_ahb_enableperiph(MCLK_AHBMASK_HPB3)
#define sam_ahb_dsu_enableperiph()         sam_ahb_enableperiph(MCLK_AHBMASK_DSU)
#define sam_ahb_nvmctrl_enableperiph()     sam_ahb_enableperiph(MCLK_AHBMASK_NVMCTRL)
#define sam_ahb_cmcc_enableperiph()        sam_ahb_enableperiph(MCLK_AHBMASK_CMCC)
#define sam_ahb_dmac_enableperiph()        sam_ahb_enableperiph(MCLK_AHBMASK_DMAC)
#define sam_ahb_usb_enableperiph()         sam_ahb_enableperiph(MCLK_AHBMASK_USB)
#define sam_ahb_pac_enableperiph()         sam_ahb_enableperiph(MCLK_AHBMASK_PAC)
#define sam_ahb_qspi_enableperiph()        sam_ahb_enableperiph(MCLK_AHBMASK_QSPI)
#define sam_ahb_gmac_enableperiph()        sam_ahb_enableperiph(MCLK_AHBMASK_GMAC)
#define sam_ahb_sdhc0_enableperiph()       sam_ahb_enableperiph(MCLK_AHBMASK_SDHC0)
#define sam_ahb_sdhc1_enableperiph()       sam_ahb_enableperiph(MCLK_AHBMASK_SDHC1)
#define sam_ahb_can0_enableperiph()        sam_ahb_enableperiph(MCLK_AHBMASK_CAN0)
#define sam_ahb_can1_enableperiph()        sam_ahb_enableperiph(MCLK_AHBMASK_CAN1)
#define sam_ahb_icm_enableperiph()         sam_ahb_enableperiph(MCLK_AHBMASK_ICM)
#define sam_ahb_pukcc_enableperiph()       sam_ahb_enableperiph(MCLK_AHBMASK_PUKCC)
#define sam_ahb_qspi2x_enableperiph()      sam_ahb_enableperiph(MCLK_AHBMASK_QSPI2X)
#define sam_ahb_nvmctrl_smeeprom_enableperiph() sam_ahb_enableperiph(MCLK_AHBMASK_NVMCTRL_SMEEPROM)
#define sam_ahb_nvmctrl_cache_enableperiph()    sam_ahb_enableperiph(MCLK_AHBMASK_NVMCTRL_CACHE)

#define sam_apba_enableperiph(s)           modifyreg32(SAM_MCLK_APBAMASK,0,s)

#define sam_apb_pac_enableperiph()         sam_apba_enableperiph(MCLK_APBAMASK_PAC)
#define sam_apb_pm_enableperiph()          sam_apba_enableperiph(MCLK_APBAMASK_PM)
#define sam_apb_mclk_enableperiph()        sam_apba_enableperiph(MCLK_APBAMASK_MCLK)
#define sam_apb_rstc_enableperiph()        sam_apba_enableperiph(MCLK_APBAMASK_RSTC)
#define sam_apb_oscctrl_enableperiph()     sam_apba_enableperiph(MCLK_APBAMASK_OSCCTRL)
#define sam_apb_osc32kctrl_enableperiph()  sam_apba_enableperiph(MCLK_APBAMASK_OSC32KCTRL)
#define sam_apb_supc_enableperiph()        sam_apba_enableperiph(MCLK_APBAMASK_SUPC)
#define sam_apb_gclk_enableperiph()        sam_apba_enableperiph(MCLK_APBAMASK_GCLK)
#define sam_apb_wdt_enableperiph()         sam_apba_enableperiph(MCLK_APBAMASK_WDT)
#define sam_apb_rtc_enableperiph()         sam_apba_enableperiph(MCLK_APBAMASK_RTC)
#define sam_apb_eic_enableperiph()         sam_apba_enableperiph(MCLK_APBAMASK_EIC)
#define sam_apb_freqm_enableperiph()       sam_apba_enableperiph(MCLK_APBAMASK_FREQM)
#define sam_apb_sercom0_enableperiph()     sam_apba_enableperiph(MCLK_APBAMASK_SERCOM0)
#define sam_apb_sercom1_enableperiph()     sam_apba_enableperiph(MCLK_APBAMASK_SERCOM1)
#define sam_apb_tc0_enableperiph()         sam_apba_enableperiph(MCLK_APBAMASK_TC0)
#define sam_apb_tc1_enableperiph()         sam_apba_enableperiph(MCLK_APBAMASK_TC1)

#define sam_apbb_enableperiph(s)           modifyreg32(SAM_MCLK_APBBMASK,0,s)

#define sam_apb_usb_enableperiph()         sam_apbb_enableperiph(MCLK_APBBMASK_USB)
#define sam_apb_dsu_enableperiph()         sam_apbb_enableperiph(MCLK_APBBMASK_DSU)
#define sam_apb_nvmctrl_enableperiph()     sam_apbb_enableperiph(MCLK_APBBMASK_NVMCTRL)
#define sam_apb_port_enableperiph()        sam_apbb_enableperiph(MCLK_APBBMASK_PORT)
#define sam_apb_evsys_enableperiph()       sam_apbb_enableperiph(MCLK_APBBMASK_EVSYS)
#define sam_apb_sercom2_enableperiph()     sam_apbb_enableperiph(MCLK_APBBMASK_SERCOM2)
#define sam_apb_sercom3_enableperiph()     sam_apbb_enableperiph(MCLK_APBBMASK_SERCOM3)
#define sam_apb_tcc0_enableperiph()        sam_apbb_enableperiph(MCLK_APBBMASK_TCC0)
#define sam_apb_tcc1_enableperiph()        sam_apbb_enableperiph(MCLK_APBBMASK_TCC1)
#define sam_apb_tc2_enableperiph()         sam_apbb_enableperiph(MCLK_APBBMASK_TC2)
#define sam_apb_tc3_enableperiph()         sam_apbb_enableperiph(MCLK_APBBMASK_TC3)
#define sam_apb_ramecc_enableperiph()      sam_apbb_enableperiph(MCLK_APBBMASK_RAMECC)

#define sam_apbc_enableperiph(s)           modifyreg32(SAM_MCLK_APBCMASK,0,s)

#define sam_apb_gmac_enableperiph(n)       sam_apbc_enableperiph(MCLK_APBCMASK_GMAC)
#define sam_apb_tcc2_enableperiph(n)       sam_apbc_enableperiph(MCLK_APBCMASK_TCC2)
#define sam_apb_tcc3_enableperiph(n)       sam_apbc_enableperiph(MCLK_APBCMASK_TCC3)
#define sam_apb_tc4_enableperiph(n)        sam_apbc_enableperiph(MCLK_APBCMASK_TC4)
#define sam_apb_tc5_enableperiph(n)        sam_apbc_enableperiph(MCLK_APBCMASK_TC5)
#define sam_apb_pdec_enableperiph(n)       sam_apbc_enableperiph(MCLK_APBCMASK_PDEC)
#define sam_apb_ac_enableperiph(n)         sam_apbc_enableperiph(MCLK_APBCMASK_AC)
#define sam_apb_aes_enableperiph(n)        sam_apbc_enableperiph(MCLK_APBCMASK_AES)
#define sam_apb_trng_enableperiph(n)       sam_apbc_enableperiph(MCLK_APBCMASK_TRNG)
#define sam_apb_icm_enableperiph(n)        sam_apbc_enableperiph(MCLK_APBCMASK_ICM)
#define sam_apb_qspi_enableperiph(n)       sam_apbc_enableperiph(MCLK_APBCMASK_QSPI)
#define sam_apb_ccl_enableperiph(n)        sam_apbc_enableperiph(MCLK_APBCMASK_CCL)

#define sam_apbd_enableperiph(s)           modifyreg32(SAM_MCLK_APBDMASK,0,s)

#define sam_apb_sercom4_enableperiph()     sam_apbd_enableperiph(MCLK_APBDMASK_SERCOM4)
#define sam_apb_sercom5_enableperiph()     sam_apbd_enableperiph(MCLK_APBDMASK_SERCOM5)
#define sam_apb_sercom6_enableperiph()     sam_apbd_enableperiph(MCLK_APBDMASK_SERCOM6)
#define sam_apb_sercom7_enableperiph()     sam_apbd_enableperiph(MCLK_APBDMASK_SERCOM7)
#define sam_apb_tcc4_enableperiph()        sam_apbd_enableperiph(MCLK_APBDMASK_TCC4)
#define sam_apb_tc6_enableperiph()         sam_apbd_enableperiph(MCLK_APBDMASK_TC6)
#define sam_apb_tc7_enableperiph()         sam_apbd_enableperiph(MCLK_APBDMASK_TC7)
#define sam_apb_adc0_enableperiph()        sam_apbd_enableperiph(MCLK_APBDMASK_ADC0)
#define sam_apb_adc1_enableperiph()        sam_apbd_enableperiph(MCLK_APBDMASK_ADC1)
#define sam_apb_dac_enableperiph()         sam_apbd_enableperiph(MCLK_APBDMASK_DAC)
#define sam_apb_i2c_enableperiph()         sam_apbd_enableperiph(MCLK_APBDMASK_I2C)
#define sam_apb_pcc_enableperiph()         sam_apbd_enableperiph(MCLK_APBDMASK_PCC)

#define sam_ahb_disableperiph(s)           modifyreg32(SAM_MCLK_AHBMASK,s,0)

#define sam_ahb_hpb0_disableperiph()       sam_ahb_disableperiph(MCLK_AHBMASK_HPB0)
#define sam_ahb_hpb1_disableperiph()       sam_ahb_disableperiph(MCLK_AHBMASK_HPB1)
#define sam_ahb_hpb2_disableperiph()       sam_ahb_disableperiph(MCLK_AHBMASK_HPB2)
#define sam_ahb_hpb3_disableperiph()       sam_ahb_disableperiph(MCLK_AHBMASK_HPB3)
#define sam_ahb_dsu_disableperiph()        sam_ahb_disableperiph(MCLK_AHBMASK_DSU)
#define sam_ahb_nvmctrl_disableperiph()    sam_ahb_disableperiph(MCLK_AHBMASK_NVMCTRL)
#define sam_ahb_cmcc_disableperiph()       sam_ahb_disableperiph(MCLK_AHBMASK_CMCC)
#define sam_ahb_dmac_disableperiph()       sam_ahb_disableperiph(MCLK_AHBMASK_DMAC)
#define sam_ahb_usb_disableperiph()        sam_ahb_disableperiph(MCLK_AHBMASK_USB)
#define sam_ahb_pac_disableperiph()        sam_ahb_disableperiph(MCLK_AHBMASK_PAC)
#define sam_ahb_qspi_disableperiph()       sam_ahb_disableperiph(MCLK_AHBMASK_QSPI)
#define sam_ahb_gmac_disableperiph()       sam_ahb_disableperiph(MCLK_AHBMASK_GMAC)
#define sam_ahb_sdhc0_disableperiph()      sam_ahb_disableperiph(MCLK_AHBMASK_SDHC0)
#define sam_ahb_sdhc1_disableperiph()      sam_ahb_disableperiph(MCLK_AHBMASK_SDHC1)
#define sam_ahb_can0_disableperiph()       sam_ahb_disableperiph(MCLK_AHBMASK_CAN0)
#define sam_ahb_can1_disableperiph()       sam_ahb_disableperiph(MCLK_AHBMASK_CAN1)
#define sam_ahb_icm_disableperiph()        sam_ahb_disableperiph(MCLK_AHBMASK_ICM)
#define sam_ahb_pukcc_disableperiph()      sam_ahb_disableperiph(MCLK_AHBMASK_PUKCC)
#define sam_ahb_qspi2x_disableperiph()     sam_ahb_disableperiph(MCLK_AHBMASK_QSPI2X)
#define sam_ahb_nvmctrl_smeeprom_disableperiph() sam_ahb_disableperiph(MCLK_AHBMASK_NVMCTRL_SMEEPROM)
#define sam_ahb_nvmctrl_cache_disableperiph()    sam_ahb_disableperiph(MCLK_AHBMASK_NVMCTRL_CACHE)

#define sam_apba_disableperiph(s)          modifyreg32(SAM_MCLK_APBAMASK,s,0)

#define sam_apb_pac_disableperiph()        sam_apba_disableperiph(MCLK_APBAMASK_PAC)
#define sam_apb_pm_disableperiph()         sam_apba_disableperiph(MCLK_APBAMASK_PM)
#define sam_apb_mclk_disableperiph()       sam_apba_disableperiph(MCLK_APBAMASK_MCLK)
#define sam_apb_rstc_disableperiph()       sam_apba_disableperiph(MCLK_APBAMASK_RSTC)
#define sam_apb_oscctrl_disableperiph()    sam_apba_disableperiph(MCLK_APBAMASK_OSCCTRL)
#define sam_apb_osc32kctrl_disableperiph() sam_apba_disableperiph(MCLK_APBAMASK_OSC32KCTRL)
#define sam_apb_supc_disableperiph()       sam_apba_disableperiph(MCLK_APBAMASK_SUPC)
#define sam_apb_gclk_disableperiph()       sam_apba_disableperiph(MCLK_APBAMASK_GCLK)
#define sam_apb_wdt_disableperiph()        sam_apba_disableperiph(MCLK_APBAMASK_WDT)
#define sam_apb_rtc_disableperiph()        sam_apba_disableperiph(MCLK_APBAMASK_RTC)
#define sam_apb_eic_disableperiph()        sam_apba_disableperiph(MCLK_APBAMASK_EIC)
#define sam_apb_freqm_disableperiph()      sam_apba_disableperiph(MCLK_APBAMASK_FREQM)
#define sam_apb_sercom0_disableperiph()    sam_apba_disableperiph(MCLK_APBAMASK_SERCOM0)
#define sam_apb_sercom1_disableperiph()    sam_apba_disableperiph(MCLK_APBAMASK_SERCOM1)
#define sam_apb_tc0_disableperiph()        sam_apba_disableperiph(MCLK_APBAMASK_TC0)
#define sam_apb_tc1_disableperiph()        sam_apba_disableperiph(MCLK_APBAMASK_TC1)

#define sam_apbb_disableperiph(s)          modifyreg32(SAM_MCLK_APBBMASK,s,0)

#define sam_apb_usb_disableperiph()        sam_apbb_disableperiph(MCLK_APBBMASK_USB)
#define sam_apb_dsu_disableperiph()        sam_apbb_disableperiph(MCLK_APBBMASK_DSU)
#define sam_apb_nvmctrl_disableperiph()    sam_apbb_disableperiph(MCLK_APBBMASK_NVMCTRL)
#define sam_apb_port_disableperiph()       sam_apbb_disableperiph(MCLK_APBBMASK_PORT)
#define sam_apb_evsys_disableperiph()      sam_apbb_disableperiph(MCLK_APBBMASK_EVSYS)
#define sam_apb_sercom2_disableperiph()    sam_apbb_disableperiph(MCLK_APBBMASK_SERCOM2)
#define sam_apb_sercom3_disableperiph()    sam_apbb_disableperiph(MCLK_APBBMASK_SERCOM3)
#define sam_apb_tcc0_disableperiph()       sam_apbb_disableperiph(MCLK_APBBMASK_TCC0)
#define sam_apb_tcc1_disableperiph()       sam_apbb_disableperiph(MCLK_APBBMASK_TCC1)
#define sam_apb_tc2_disableperiph()        sam_apbb_disableperiph(MCLK_APBBMASK_TC2)
#define sam_apb_tc3_disableperiph()        sam_apbb_disableperiph(MCLK_APBBMASK_TC3)
#define sam_apb_ramecc_disableperiph()     sam_apbb_disableperiph(MCLK_APBBMASK_RAMECC)

#define sam_apbc_disableperiph(s)          modifyreg32(SAM_MCLK_APBCMASK,s,0)

#define sam_apb_gmac_disableperiph(n)      sam_apbc_disableperiph(MCLK_APBCMASK_GMAC)
#define sam_apb_tcc2_disableperiph(n)      sam_apbc_disableperiph(MCLK_APBCMASK_TCC2)
#define sam_apb_tcc3_disableperiph(n)      sam_apbc_disableperiph(MCLK_APBCMASK_TCC3)
#define sam_apb_tc4_disableperiph(n)       sam_apbc_disableperiph(MCLK_APBCMASK_TC4)
#define sam_apb_tc5_disableperiph(n)       sam_apbc_disableperiph(MCLK_APBCMASK_TC5)
#define sam_apb_pdec_disableperiph(n)      sam_apbc_disableperiph(MCLK_APBCMASK_PDEC)
#define sam_apb_ac_disableperiph(n)        sam_apbc_disableperiph(MCLK_APBCMASK_AC)
#define sam_apb_aes_disableperiph(n)       sam_apbc_disableperiph(MCLK_APBCMASK_AES)
#define sam_apb_trng_disableperiph(n)      sam_apbc_disableperiph(MCLK_APBCMASK_TRNG)
#define sam_apb_icm_disableperiph(n)       sam_apbc_disableperiph(MCLK_APBCMASK_ICM)
#define sam_apb_qspi_disableperiph(n)      sam_apbc_disableperiph(MCLK_APBCMASK_QSPI)
#define sam_apb_ccl_disableperiph(n)       sam_apbc_disableperiph(MCLK_APBCMASK_CCL)

#define sam_apbd_disableperiph(s)          modifyreg32(SAM_MCLK_APBDMASK,s,0)

#define sam_apb_sercom4_disableperiph()    sam_apbd_disableperiph(MCLK_APBDMASK_SERCOM4)
#define sam_apb_sercom5_disableperiph()    sam_apbd_disableperiph(MCLK_APBDMASK_SERCOM5)
#define sam_apb_sercom6_disableperiph()    sam_apbd_disableperiph(MCLK_APBDMASK_SERCOM6)
#define sam_apb_sercom7_disableperiph()    sam_apbd_disableperiph(MCLK_APBDMASK_SERCOM7)
#define sam_apb_tcc4_disableperiph()       sam_apbd_disableperiph(MCLK_APBDMASK_TCC4)
#define sam_apb_tc6_disableperiph()        sam_apbd_disableperiph(MCLK_APBDMASK_TC6)
#define sam_apb_tc7_disableperiph()        sam_apbd_disableperiph(MCLK_APBDMASK_TC7)
#define sam_apb_adc0_disableperiph()       sam_apbd_disableperiph(MCLK_APBDMASK_ADC0)
#define sam_apb_adc1_disableperiph()       sam_apbd_disableperiph(MCLK_APBDMASK_ADC1)
#define sam_apb_dac_disableperiph()        sam_apbd_disableperiph(MCLK_APBDMASK_DAC)
#define sam_apb_i2c_disableperiph()        sam_apbd_disableperiph(MCLK_APBDMASK_I2C)
#define sam_apb_pcc_disableperiph()        sam_apbd_disableperiph(MCLK_APBDMASK_PCC)

#define sam_ahb_isenabled(s)               (getreg32(SAM_MCLK_AHBMASK) & (s)) != 0)

#define sam_ahb_hpb0_isenabled()           sam_ahb_isenabled(MCLK_AHBMASK_HPB0)
#define sam_ahb_hpb1_isenabled()           sam_ahb_isenabled(MCLK_AHBMASK_HPB1)
#define sam_ahb_hpb2_isenabled()           sam_ahb_isenabled(MCLK_AHBMASK_HPB2)
#define sam_ahb_hpb3_isenabled()           sam_ahb_isenabled(MCLK_AHBMASK_HPB3)
#define sam_ahb_dsu_isenabled()            sam_ahb_isenabled(MCLK_AHBMASK_DSU)
#define sam_ahb_nvmctrl_isenabled()        sam_ahb_isenabled(MCLK_AHBMASK_NVMCTRL)
#define sam_ahb_cmcc_isenabled()           sam_ahb_isenabled(MCLK_AHBMASK_CMCC)
#define sam_ahb_dmac_isenabled()           sam_ahb_isenabled(MCLK_AHBMASK_DMAC)
#define sam_ahb_usb_isenabled()            sam_ahb_isenabled(MCLK_AHBMASK_USB)
#define sam_ahb_pac_isenabled()            sam_ahb_isenabled(MCLK_AHBMASK_PAC)
#define sam_ahb_qspi_isenabled()           sam_ahb_isenabled(MCLK_AHBMASK_QSPI)
#define sam_ahb_gmac_isenabled()           sam_ahb_isenabled(MCLK_AHBMASK_GMAC)
#define sam_ahb_sdhc0_isenabled()          sam_ahb_isenabled(MCLK_AHBMASK_SDHC0)
#define sam_ahb_sdhc1_isenabled()          sam_ahb_isenabled(MCLK_AHBMASK_SDHC1)
#define sam_ahb_can0_isenabled()           sam_ahb_isenabled(MCLK_AHBMASK_CAN0)
#define sam_ahb_can1_isenabled()           sam_ahb_isenabled(MCLK_AHBMASK_CAN1)
#define sam_ahb_icm_isenabled()            sam_ahb_isenabled(MCLK_AHBMASK_ICM)
#define sam_ahb_pukcc_isenabled()          sam_ahb_isenabled(MCLK_AHBMASK_PUKCC)
#define sam_ahb_qspi2x_isenabled()         sam_ahb_isenabled(MCLK_AHBMASK_QSPI2X)
#define sam_ahb_nvmctrl_smeeprom_isenabled() sam_ahb_isenabled(MCLK_AHBMASK_NVMCTRL_SMEEPROM)
#define sam_ahb_nvmctrl_cache_isenabled()    sam_ahb_isenabled(MCLK_AHBMASK_NVMCTRL_CACHE)

#define sam_apba_isenabled(s)              (getreg32(SAM_MCLK_APBAMASK) & (s)) != 0)

#define sam_apb_pac_isenabled()            sam_apba_isenabled(MCLK_APBAMASK_PAC)
#define sam_apb_pm_isenabled()             sam_apba_isenabled(MCLK_APBAMASK_PM)
#define sam_apb_mclk_isenabled()           sam_apba_isenabled(MCLK_APBAMASK_MCLK)
#define sam_apb_rstc_isenabled()           sam_apba_isenabled(MCLK_APBAMASK_RSTC)
#define sam_apb_oscctrl_isenabled()        sam_apba_isenabled(MCLK_APBAMASK_OSCCTRL)
#define sam_apb_osc32kctrl_isenabled()     sam_apba_isenabled(MCLK_APBAMASK_OSC32KCTRL)
#define sam_apb_supc_isenabled()           sam_apba_isenabled(MCLK_APBAMASK_SUPC)
#define sam_apb_gclk_isenabled()           sam_apba_isenabled(MCLK_APBAMASK_GCLK)
#define sam_apb_wdt_isenabled()            sam_apba_isenabled(MCLK_APBAMASK_WDT)
#define sam_apb_rtc_isenabled()            sam_apba_isenabled(MCLK_APBAMASK_RTC)
#define sam_apb_eic_isenabled()            sam_apba_isenabled(MCLK_APBAMASK_EIC)
#define sam_apb_freqm_isenabled()          sam_apba_isenabled(MCLK_APBAMASK_FREQM)
#define sam_apb_sercom0_isenabled()        sam_apba_isenabled(MCLK_APBAMASK_SERCOM0)
#define sam_apb_sercom1_isenabled()        sam_apba_isenabled(MCLK_APBAMASK_SERCOM1)
#define sam_apb_tc0_isenabled()            sam_apba_isenabled(MCLK_APBAMASK_TC0)
#define sam_apb_tc1_isenabled()            sam_apba_isenabled(MCLK_APBAMASK_TC1)

#define sam_apbb_isenabled(s)              (getreg32(SAM_MCLK_APBBMASK) & (s)) != 0)

#define sam_apb_usb_isenabled()            sam_apbb_isenabled(MCLK_APBBMASK_USB)
#define sam_apb_dsu_isenabled()            sam_apbb_isenabled(MCLK_APBBMASK_DSU)
#define sam_apb_nvmctrl_isenabled()        sam_apbb_isenabled(MCLK_APBBMASK_NVMCTRL)
#define sam_apb_port_isenabled()           sam_apbb_isenabled(MCLK_APBBMASK_PORT)
#define sam_apb_evsys_isenabled()          sam_apbb_isenabled(MCLK_APBBMASK_EVSYS)
#define sam_apb_sercom2_isenabled()        sam_apbb_isenabled(MCLK_APBBMASK_SERCOM2)
#define sam_apb_sercom3_isenabled()        sam_apbb_isenabled(MCLK_APBBMASK_SERCOM3)
#define sam_apb_tcc0_isenabled()           sam_apbb_isenabled(MCLK_APBBMASK_TCC0)
#define sam_apb_tcc1_isenabled()           sam_apbb_isenabled(MCLK_APBBMASK_TCC1)
#define sam_apb_tc2_isenabled()            sam_apbb_isenabled(MCLK_APBBMASK_TC2)
#define sam_apb_tc3_isenabled()            sam_apbb_isenabled(MCLK_APBBMASK_TC3)
#define sam_apb_ramecc_isenabled()         sam_apbb_isenabled(MCLK_APBBMASK_RAMECC)

#define sam_apbc_isenabled(s)              (getreg32(SAM_MCLK_APBCMASK) & (s)) != 0)

#define sam_apb_gmac_isenabled(n)          sam_apbc_isenabled(MCLK_APBCMASK_GMAC)
#define sam_apb_tcc2_isenabled(n)          sam_apbc_isenabled(MCLK_APBCMASK_TCC2)
#define sam_apb_tcc3_isenabled(n)          sam_apbc_isenabled(MCLK_APBCMASK_TCC3)
#define sam_apb_tc4_isenabled(n)           sam_apbc_isenabled(MCLK_APBCMASK_TC4)
#define sam_apb_tc5_isenabled(n)           sam_apbc_isenabled(MCLK_APBCMASK_TC5)
#define sam_apb_pdec_isenabled(n)          sam_apbc_isenabled(MCLK_APBCMASK_PDEC)
#define sam_apb_ac_isenabled(n)            sam_apbc_isenabled(MCLK_APBCMASK_AC)
#define sam_apb_aes_isenabled(n)           sam_apbc_isenabled(MCLK_APBCMASK_AES)
#define sam_apb_trng_isenabled(n)          sam_apbc_isenabled(MCLK_APBCMASK_TRNG)
#define sam_apb_icm_isenabled(n)           sam_apbc_isenabled(MCLK_APBCMASK_ICM)
#define sam_apb_qspi_isenabled(n)          sam_apbc_isenabled(MCLK_APBCMASK_QSPI)
#define sam_apb_ccl_isenabled(n)           sam_apbc_isenabled(MCLK_APBCMASK_CCL)

#define sam_apbd_isenabled(s)              (getreg32(SAM_MCLK_APBDMASK) & (s)) != 0)

#define sam_apb_sercom4_isenabled()        sam_apbd_isenabled(MCLK_APBDMASK_SERCOM4)
#define sam_apb_sercom5_isenabled()        sam_apbd_isenabled(MCLK_APBDMASK_SERCOM5)
#define sam_apb_sercom6_isenabled()        sam_apbd_isenabled(MCLK_APBDMASK_SERCOM6)
#define sam_apb_sercom7_isenabled()        sam_apbd_isenabled(MCLK_APBDMASK_SERCOM7)
#define sam_apb_tcc4_isenabled()           sam_apbd_isenabled(MCLK_APBDMASK_TCC4)
#define sam_apb_tc6_isenabled()            sam_apbd_isenabled(MCLK_APBDMASK_TC6)
#define sam_apb_tc7_isenabled()            sam_apbd_isenabled(MCLK_APBDMASK_TC7)
#define sam_apb_adc0_isenabled()           sam_apbd_isenabled(MCLK_APBDMASK_ADC0)
#define sam_apb_adc1_isenabled()           sam_apbd_isenabled(MCLK_APBDMASK_ADC1)
#define sam_apb_dac_isenabled()            sam_apbd_isenabled(MCLK_APBDMASK_DAC)
#define sam_apb_i2c_isenabled()            sam_apbd_isenabled(MCLK_APBDMASK_I2C)
#define sam_apb_pcc_isenabled()            sam_apbd_isenabled(MCLK_APBDMASK_PCC)

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
#endif /* __ARCH_ARM_SRC_SAMD5E5_SAM_PERIPHCLKS_H */
