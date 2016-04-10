/************************************************************************************
 * arch/arm/src/sam34/sam4l_periphclks.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_SAM34_SAM4L_PERIPHCLKS_H
#define __ARCH_ARM_SRC_SAM34_SAM4L_PERIPHCLKS_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip/sam4l_pm.h"

#ifdef CONFIG_ARCH_CHIP_SAM4L

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* SAM4L helper macros */

#define sam_enableperipheral(a,s)    sam_modifyperipheral(a,0,s)
#define sam_disableperipheral(a,s)   sam_modifyperipheral(a,s,0)

#define sam_cpu_enableperipheral(s)  sam_enableperipheral(SAM_PM_CPUMASK,s)
#define sam_hsb_enableperipheral(s)  sam_enableperipheral(SAM_PM_HSBMASK,s)
#define sam_pbc_enableperipheral(s)  sam_enableperipheral(SAM_PM_PBCMASK,s)
#define sam_pbd_enableperipheral(s)  sam_enableperipheral(SAM_PM_PBDMASK,s)

#define sam_cpu_disableperipheral(s) sam_disableperipheral(SAM_PM_CPUMASK,s)
#define sam_hsb_disableperipheral(s) sam_disableperipheral(SAM_PM_HSBMASK,s)
#define sam_pbc_enableperipheral(s)  sam_enableperipheral(SAM_PM_PBCMASK,s)
#define sam_pbd_enableperipheral(s)  sam_enableperipheral(SAM_PM_PBDMASK,s)

#define sam_pba_enabledivmask(s)     sam_pba_modifydivmask(0,s)
#define sam_pba_disabledivmask(s)    sam_pba_modifydivmask(s,0)

/* Macros to enable clocking to individual peripherals */

#define sam_aesa_enableclk()         sam_hsb_enableperipheral(PM_HSBMASK_AESA)
#define sam_iisc_enableclk()         sam_pba_enableperipheral(PM_PBAMASK_IISC)
#define sam_spi0_enableclk()         sam_pba_enableperipheral(PM_PBAMASK_SPI)

#define sam_tc0_enableclk() \
  do { \
    sam_pba_enableperipheral(PM_PBAMASK_TC0); \
    sam_pba_enabledivmask(PM_PBADIVMASK_TIMER_CLOCKS); \
  } while (0)

#define sam_tc1_enableclk() \
  do { \
    sam_pba_enableperipheral(PM_PBAMASK_TC1); \
    sam_pba_enabledivmask(PM_PBADIVMASK_TIMER_CLOCKS); \
  } while (0)

#define sam_twim0_enableclk()         sam_pba_enableperipheral(PM_PBAMASK_TWIM0)
#define sam_twis0_enableclk()         sam_pba_enableperipheral(PM_PBAMASK_TWIS0)
#define sam_twim1_enableclk()         sam_pba_enableperipheral(PM_PBAMASK_TWIM1)
#define sam_twis1_enableclk()         sam_pba_enableperipheral(PM_PBAMASK_TWIS1)

#define sam_usart0_enableclk() \
  do { \
    sam_pba_enableperipheral(PM_PBAMASK_USART0); \
    sam_pba_enabledivmask(PM_PBADIVMASK_CLK_USART); \
  } while (0)

#define sam_usart1_enableclk() \
  do { \
    sam_pba_enableperipheral(PM_PBAMASK_USART1); \
    sam_pba_enabledivmask(PM_PBADIVMASK_CLK_USART); \
  } while (0)

#define sam_usart2_enableclk() \
  do { \
    sam_pba_enableperipheral(PM_PBAMASK_USART2); \
    sam_pba_enabledivmask(PBA_DIVMASK_CLK_USART); \
  } while (0)

#define sam_usart3_enableclk() \
  do { \
    sam_pba_enableperipheral(PM_PBAMASK_USART3); \
    sam_pba_enabledivmask(PBA_DIVMASK_CLK_USART); \
  } while (0)

#define sam_adcife_enableclk()        sam_pba_enableperipheral(PM_PBAMASK_ADCIFE)
#define sam_dacc_enableclk()          sam_pba_enableperipheral(PM_PBAMASK_DACC)
#define sam_acifc_enableclk()         sam_pba_enableperipheral(PM_PBAMASK_ACIFC)
#define sam_gloc_enableclk()          sam_pba_enableperipheral(PM_PBAMASK_GLOC)
#define sam_abdacb_enableclk()        sam_pba_enableperipheral(PM_PBAMASK_ABDACB)
#define sam_trng_enableclk()          sam_pba_enableperipheral(PM_PBAMASK_TRNG)
#define sam_parc_enableclk()          sam_pba_enableperipheral(PM_PBAMASK_PARC)
#define sam_catb_enableclk()          sam_pba_enableperipheral(PM_PBAMASK_CATB)
#define sam_twim2_enableclk()         sam_pba_enableperipheral(PM_PBAMASK_TWIM2)
#define sam_twim3_enableclk()         sam_pba_enableperipheral(PM_PBAMASK_TWIM3)
#define sam_lcdca_enableclk()         sam_pba_enableperipheral(PM_PBAMASK_LCDCA)

#define sam_flashcalw_enableclk() \
  do { \
    sam_hsb_enableperipheral(PM_HSBMASK_FLASHCALW); \
    sam_pbb_enableperipheral(PM_PBBMASK_FLASHCALW); \
  } while (0)

#define sam_picocache_enableclk() \
  do { \
    sam_hsb_enableperipheral(PM_HSBMASK_HRAMC1); \
    sam_pbb_enableperipheral(PM_PBBMASK_HRAMC1); \
  } while (0)

#define sam_hmatrix_enableclk()       sam_pbb_enableperipheral(PM_PBBMASK_HMATRIX)

#define sam_pdca_enableclk() \
  do { \
    sam_hsb_enableperipheral(PM_HSBMASK_PDCA); \
    sam_pbb_enableperipheral(PM_PBBMASK_PDCA); \
  } while (0)

#define sam_crccu_enableclk() \
  do { \
    sam_hsb_enableperipheral(PM_HSBMASK_CRCCU); \
    sam_pbb_enableperipheral(PM_PBBMASK_CRCCU); \
  } while (0)

#define sam_pevc_enableclk()          sam_pbb_enableperipheral(PM_PBBMASK_PEVC)
#define sam_pm_enableclk()            sam_pbc_enableperipheral(PM_PBCMASK_PM)
#define sam_chipid_enableclk()        sam_pbc_enableperipheral(PM_PBCMASK_CHIPID)
#define sam_scif_enableclk()          sam_pbc_enableperipheral(PM_PBCMASK_SCIF)
#define sam_freqm_enableclk()         sam_pbc_enableperipheral(PM_PBCMASK_FREQM)
#define sam_gpio_enableclk()          sam_pbc_enableperipheral(PM_PBCMASK_GPIO)
#define sam_bpm_enableclk()           sam_pbd_enableperipheral(PM_PBDMASK_BPM)
#define sam_bscif_enableclk()         sam_pbd_enableperipheral(PM_PBDMASK_BSCIF)
#define sam_ast_enableclk()           sam_pbd_enableperipheral(PM_PBDMASK_AST)
#define sam_wdt_enableclk()           sam_pbd_enableperipheral(PM_PBDMASK_WDT)
#define sam_eic_enableclk()           sam_pbd_enableperipheral(PM_PBDMASK_EIC)
#define sam_picouart_enableclk()      sam_pbd_enableperipheral(PM_PBDMASK_PICOUART)

/* Macros to disable clocking to individual peripherals */

#define sam_aesa_disableclk()         sam_hsb_disableperipheral(PM_HSBMASK_AESA)
#define sam_iisc_disableclk()         sam_pba_disableperipheral(PM_PBAMASK_IISC)
#define sam_spi0_disableclk()         sam_pba_disableperipheral(PM_PBAMASK_SPI)
#define sam_tc0_disableclk()          sam_pba_disableperipheral(PM_PBAMASK_TC0)
#define sam_tc1_disableclk()          sam_pba_disableperipheral(PM_PBAMASK_TC1)
#define sam_twim0_disableclk()        sam_pba_disableperipheral(PM_PBAMASK_TWIM0)
#define sam_twis0_disableclk()        sam_pba_disableperipheral(PM_PBAMASK_TWIS0)
#define sam_twim1_disableclk()        sam_pba_disableperipheral(PM_PBAMASK_TWIM1)
#define sam_twis1_disableclk()        sam_pba_disableperipheral(PM_PBAMASK_TWIS1)
#define sam_usart0_disableclk()       sam_pba_disableperipheral(PM_PBAMASK_USART0)
#define sam_usart1_disableclk()       sam_pba_disableperipheral(PM_PBAMASK_USART1)
#define sam_usart2_disableclk()       sam_pba_disableperipheral(PM_PBAMASK_USART2)
#define sam_usart3_disableclk()       sam_pba_disableperipheral(PM_PBAMASK_USART3)
#define sam_adcife_disableclk()       sam_pba_disableperipheral(PM_PBAMASK_ADCIFE)
#define sam_dacc_disableclk()         sam_pba_disableperipheral(PM_PBAMASK_DACC)
#define sam_acifc_disableclk()        sam_pba_disableperipheral(PM_PBAMASK_ACIFC)
#define sam_gloc_disableclk()         sam_pba_disableperipheral(PM_PBAMASK_GLOC)
#define sam_abdacb_disableclk()       sam_pba_disableperipheral(PM_PBAMASK_ABDACB)
#define sam_trng_disableclk()         sam_pba_disableperipheral(PM_PBAMASK_TRNG)
#define sam_parc_disableclk()         sam_pba_disableperipheral(PM_PBAMASK_PARC)
#define sam_catb_disableclk()         sam_pba_disableperipheral(PM_PBAMASK_CATB)
#define sam_twim2_disableclk()        sam_pba_disableperipheral(PM_PBAMASK_TWIM2)
#define sam_twim3_disableclk()        sam_pba_disableperipheral(PM_PBAMASK_TWIM3)
#define sam_lcdca_disableclk()        sam_pba_disableperipheral(PM_PBAMASK_LCDCA)
#define sam_flashcalw_disableclk()    sam_pba_disableperipheral(PM_HSBMASK_FLASHCALW)

#define sam_picocache_disableclk() \
  do { \
    sam_hsb_disableperipheral(PM_HSBMASK_HRAMC1); \
    sam_pbb_disableperipheral(PM_PBBMASK_HRAMC1); \
  } while (0)

#define sam_hmatrix_disableclk()      sam_pbb_disableperipheral(PM_PBBMASK_HMATRIX)

#define sam_pdca_disableclk() \
  do { \
    sam_hsb_disableperipheral(PM_HSBMASK_PDCA); \
    sam_pbb_disableperipheral(PM_PBBMASK_PDCA); \
  } while (0)

#define sam_crccu_disableclk() \
  do { \
    sam_hsb_disableperipheral(PM_HSBMASK_CRCCU); \
    sam_pbb_disableperipheral(PM_PBBMASK_CRCCU); \
  } while (0)

#define sam_pevc_disableclk()         sam_pbb_disableperipheral(PM_PBBMASK_PEVC)
#define sam_pm_disableclk()           sam_pbc_disableperipheral(PM_PBCMASK_PM)
#define sam_chipid_disableclk()       sam_pbc_disableperipheral(PM_PBCMASK_CHIPID)
#define sam_scif_disableclk()         sam_pbc_disableperipheral(PM_PBCMASK_SCIF)
#define sam_freqm_disableclk()        sam_pbc_disableperipheral(PM_PBCMASK_FREQM)
#define sam_gpio_disableclk()         sam_pbc_disableperipheral(PM_PBCMASK_GPIO)
#define sam_bpm_disableclk()          sam_pbd_disableperipheral(PM_PBDMASK_BPM)
#define sam_bscif_disableclk()        sam_pbd_disableperipheral(PM_PBDMASK_BSCIF)
#define sam_ast_disableclk()          sam_pbd_disableperipheral(PM_PBDMASK_AST)
#define sam_wdt_disableclk()          sam_pbd_disableperipheral(PM_PBDMASK_WDT)
#define sam_eic_disableclk()          sam_pbd_disableperipheral(PM_PBDMASK_EIC)
#define sam_picouart_disableclk()     sam_pbd_disableperipheral(PM_PBDMASK_PICOUART)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Data
 ************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

/************************************************************************************
 * Name: sam_init_periphclks
 *
 * Description:
 *   Called during boot to enable clocking on all selected peripherals.
 *
 ************************************************************************************/

void sam_init_periphclks(void);

/************************************************************************************
 * Name: sam_modifyperipheral
 *
 * Description:
 *   This is a convenience function that is intended to be used to enable or disable
 *   module clocking.
 *
 ************************************************************************************/

void sam_modifyperipheral(uintptr_t regaddr, uint32_t clrbits, uint32_t setbits);

/************************************************************************************
 * Name: sam_pba_modifydivmask
 *
 * Description:
 *   This is a convenience function that is intended to be used to modify bits in
 *   the PBA divided clock (DIVMASK) register.
 *
 ************************************************************************************/

void sam_pba_modifydivmask(uint32_t clrbits, uint32_t setbits);

/************************************************************************************
 * Name: sam_pba_enableperipheral
 *
 * Description:
 *   This is a convenience function to enable a peripheral on the APBA bridge.
 *
 ************************************************************************************/

void sam_pba_enableperipheral(uint32_t bitset);

/************************************************************************************
 * Name: sam_pba_disableperipheral
 *
 * Description:
 *   This is a convenience function to disable a peripheral on the APBA bridge.
 *
 ************************************************************************************/

void sam_pba_disableperipheral(uint32_t bitset);

/************************************************************************************
 * Name: sam_pbb_enableperipheral
 *
 * Description:
 *   This is a convenience function to enable a peripheral on the APBB bridge.
 *
 ************************************************************************************/

void sam_pbb_enableperipheral(uint32_t bitset);

/************************************************************************************
 * Name: sam_pbb_disableperipheral
 *
 * Description:
 *   This is a convenience function to disable a peripheral on the APBA bridge.
 *
 ************************************************************************************/

void sam_pbb_disableperipheral(uint32_t bitset);

/************************************************************************************
 * Name: sam_usbc_enableclk
 *
 * Description:
 *   Enable clocking for the USBC using settings from the board.h header files.
 *
 *  "The USBC has two bus clocks connected: One High Speed Bus clock
 *   (CLK_USBC_AHB) and one Peripheral Bus clock (CLK_USBC_APB). These clocks
 *   are generated by the Power Manager.  Both clocks are enabled at reset
 *   and can be disabled by the Power Manager. It is recommended to disable
 *   the USBC before disabling the clocks, to avoid freezing the USBC in
 *   an undefined state.
 *
 *  "To follow the usb data rate at 12Mbit/s in full-speed mode, the
 *   CLK_USBC_AHB clock should be at minimum 12MHz.
 *
 *  "The 48MHz USB clock is generated by a dedicated generic clock from
 *   the SCIF module. Before using the USB, the user must ensure that the
 *   USB generic clock (GCLK_USBC) is enabled at 48MHz in the SCIF module."
 *
 ************************************************************************************/

#ifdef CONFIG_SAM34_USBC
void sam_usbc_enableclk(void);
#endif

/************************************************************************************
 * Name: sam_usbc_disableclk
 *
 * Description:
 *   Disable clocking to the USBC.
 *
 ************************************************************************************/

#ifdef CONFIG_SAM34_USBC
void sam_usbc_disableclk(void);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_ARCH_CHIP_SAM4L */
#endif /* __ARCH_ARM_SRC_SAM34_SAM4L_PERIPHCLKS_H */
