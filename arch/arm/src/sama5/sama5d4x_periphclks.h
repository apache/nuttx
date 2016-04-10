/************************************************************************************
 * arch/arm/src/sama5/sama5d4x_periphclks.h
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMA5_SAMAD54X_PERIPHCLKS_H
#define __ARCH_ARM_SRC_SAMA5_SAMAD54X_PERIPHCLKS_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <arch/irq.h>
#include "chip/sam_pmc.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Helper macros */

#define sam_enableperiph0(s)       putreg32((1 << (s)), SAM_PMC_PCER0)
#define sam_enableperiph1(s)       putreg32((1 << ((s) - 32)), SAM_PMC_PCER1)
#define sam_disableperiph0(s)      putreg32((1 << (s)), SAM_PMC_PCDR0)
#define sam_disableperiph1(s)      putreg32((1 << ((s) - 32)), SAM_PMC_PCDR1)
#define sam_isenabled0(s)          (getreg32(SAM_PMC_PCER0) & (1 << (s)) != 0)
#define sam_isenabled1(s)          (getreg32(SAM_PMC_PCER1) & (1 << ((s) - 32)) != 0)

/* Enable peripheral clocking */

#define sam_arm_enableclk()        sam_enableperiph0(SAM_PID_ARM)
#define sam_pit_enableclk()        sam_enableperiph0(SAM_PID_PIT)
#define sam_wdt_enableclk()        sam_enableperiph0(SAM_PID_WDT)
#define sam_piod_enableclk()       sam_enableperiph0(SAM_PID_PIOD)
#define sam_usart0_enableclk()     sam_enableperiph0(SAM_PID_USART0)
#define sam_usart1_enableclk()     sam_enableperiph0(SAM_PID_USART1)
#define sam_xdmac0_enableclk()     sam_enableperiph0(SAM_PID_XDMAC0)
#define sam_icm_enableclk()        sam_enableperiph0(SAM_PID_ICM)
#define sam_cpkcc_enableclk()      sam_enableperiph0(SAM_PID_CPKCC)
#define sam_aes_enableclk()        sam_enableperiph0(SAM_PID_AES)
#define sam_aesb_enableclk()       sam_enableperiph0(SAM_PID_AESB)
#define sam_tdes_enableclk()       sam_enableperiph0(SAM_PID_TDES)
#define sam_sha_enableclk()        sam_enableperiph0(SAM_PID_SHA)
#define sam_mpddrc_enableclk()     sam_enableperiph0(SAM_PID_MPDDRC)
#define sam_matrix1_enableclk()    sam_enableperiph0(SAM_PID_MATRIX1)
#define sam_matrix0_enableclk()    sam_enableperiph0(SAM_PID_MATRIX0)
#define sam_vdec_enableclk()       sam_enableperiph0(SAM_PID_VDEC)
#define sam_sbm_enableclk()        sam_enableperiph0(SAM_PID_SBM)
#define sam_hsmc_enableclk()       sam_enableperiph0(SAM_PID_HSMC)
#define sam_pioa_enableclk()       sam_enableperiph0(SAM_PID_PIOA)
#define sam_piob_enableclk()       sam_enableperiph0(SAM_PID_PIOB)
#define sam_pioc_enableclk()       sam_enableperiph0(SAM_PID_PIOC)
#define sam_pioe_enableclk()       sam_enableperiph0(SAM_PID_PIOE)
#define sam_uart0_enableclk()      sam_enableperiph0(SAM_PID_UART0)
#define sam_uart1_enableclk()      sam_enableperiph0(SAM_PID_UART1)
#define sam_usart2_enableclk()     sam_enableperiph0(SAM_PID_USART2)
#define sam_usart3_enableclk()     sam_enableperiph0(SAM_PID_USART3)
#define sam_usart4_enableclk()     sam_enableperiph0(SAM_PID_USART4)

#define sam_twi0_enableclk()       sam_enableperiph1(SAM_PID_TWI0)
#define sam_twi1_enableclk()       sam_enableperiph1(SAM_PID_TWI1)
#define sam_twi2_enableclk()       sam_enableperiph1(SAM_PID_TWI2)
#define sam_hsmci0_enableclk()     sam_enableperiph1(SAM_PID_HSMCI0)
#define sam_hsmci1_enableclk()     sam_enableperiph1(SAM_PID_HSMCI1)
#define sam_spi0_enableclk()       sam_enableperiph1(SAM_PID_SPI0)
#define sam_spi1_enableclk()       sam_enableperiph1(SAM_PID_SPI1)
#define sam_spi2_enableclk()       sam_enableperiph1(SAM_PID_SPI2)
#define sam_tc0_enableclk()        sam_enableperiph1(SAM_PID_TC0)
#define sam_tc1_enableclk()        sam_enableperiph1(SAM_PID_TC1)
#define sam_tc2_enableclk()        sam_enableperiph1(SAM_PID_TC2)
#define sam_pwm_enableclk()        sam_enableperiph1(SAM_PID_PWM)
#define sam_adc_enableclk()        sam_enableperiph1(SAM_PID_ADC)
#define sam_dbgu_enableclk()       sam_enableperiph1(SAM_PID_DBGU)
#define sam_uhphs_enableclk()      sam_enableperiph1(SAM_PID_UHPHS)
#define sam_udphs_enableclk()      sam_enableperiph1(SAM_PID_UDPHS)
#define sam_ssc0_enableclk()       sam_enableperiph1(SAM_PID_SSC0)
#define sam_ssc1_enableclk()       sam_enableperiph1(SAM_PID_SSC1)
#define sam_xdmac1_enableclk()     sam_enableperiph1(SAM_PID_XDMAC1)
#define sam_lcdc_enableclk()       sam_enableperiph1(SAM_PID_LCDC)
#define sam_isi_enableclk()        sam_enableperiph1(SAM_PID_ISI)
#define sam_trng_enableclk()       sam_enableperiph1(SAM_PID_TRNG)
#define sam_emac0_enableclk()      sam_enableperiph1(SAM_PID_EMAC0)
#define sam_emac1_enableclk()      sam_enableperiph1(SAM_PID_EMAC1)
#define sam_aicid_enableclk()      sam_enableperiph1(SAM_PID_AICID)
#define sam_sfc_enableclk()        sam_enableperiph1(SAM_PID_SFC)
#define sam_secureram_enableclk()  sam_enableperiph1(SAM_PID_SECURAM)
#define sam_smd_enableclk()        sam_enableperiph1(SAM_PID_SMD)
#define sam_twi3_enableclk()       sam_enableperiph1(SAM_PID_TWI3)
#define sam_catb_enableclk()       sam_enableperiph1(SAM_PID_CATB)

/* The Advanced Interrupt Controller and L2CC cache controllers are
 * continuously clocked. The Power Management Controller has no effect on
 * their behavior.
 *
 * (I presume that this is true of the SFR as well since it has no PMC
 *  bits to control its clocking).
 */

#define sam_sfr_enableclk()
#define sam_aic_enableclk()
#define sam_saic_enableclk()
#define sam_l2cc_enableclk()

/* Disable peripheral clocking */

#define sam_arm_disableclk()       sam_disableperiph0(SAM_PID_ARM)
#define sam_pit_disableclk()       sam_disableperiph0(SAM_PID_PIT)
#define sam_wdt_disableclk()       sam_disableperiph0(SAM_PID_WDT)
#define sam_piod_disableclk()      sam_disableperiph0(SAM_PID_PIOD)
#define sam_usart0_disableclk()    sam_disableperiph0(SAM_PID_USART0)
#define sam_usart1_disableclk()    sam_disableperiph0(SAM_PID_USART1)
#define sam_xdmac0_disableclk()    sam_disableperiph0(SAM_PID_XDMAC0)
#define sam_icm_disableclk()       sam_disableperiph0(SAM_PID_ICM)
#define sam_cpkcc_disableclk()     sam_disableperiph0(SAM_PID_CPKCC)
#define sam_aes_disableclk()       sam_disableperiph0(SAM_PID_AES)
#define sam_aesb_disableclk()      sam_disableperiph0(SAM_PID_AESB)
#define sam_tdes_disableclk()      sam_disableperiph0(SAM_PID_TDES)
#define sam_sha_disableclk()       sam_disableperiph0(SAM_PID_SHA)
#define sam_mpddrc_disableclk()    sam_disableperiph0(SAM_PID_MPDDRC)
#define sam_matrix1_disableclk()   sam_disableperiph0(SAM_PID_MATRIX1)
#define sam_matrix0_disableclk()   sam_disableperiph0(SAM_PID_MATRIX0)
#define sam_vdec_disableclk()      sam_disableperiph0(SAM_PID_VDEC)
#define sam_sbm_disableclk()       sam_disableperiph0(SAM_PID_SBM)
#define sam_hsmc_disableclk()      sam_disableperiph0(SAM_PID_HSMC)
#define sam_pioa_disableclk()      sam_disableperiph0(SAM_PID_PIOA)
#define sam_piob_disableclk()      sam_disableperiph0(SAM_PID_PIOB)
#define sam_pioc_disableclk()      sam_disableperiph0(SAM_PID_PIOC)
#define sam_pioe_disableclk()      sam_disableperiph0(SAM_PID_PIOE)
#define sam_uart0_disableclk()     sam_disableperiph0(SAM_PID_UART0)
#define sam_uart1_disableclk()     sam_disableperiph0(SAM_PID_UART1)
#define sam_usart2_disableclk()    sam_disableperiph0(SAM_PID_USART2)
#define sam_usart3_disableclk()    sam_disableperiph0(SAM_PID_USART3)
#define sam_usart4_disableclk()    sam_disableperiph0(SAM_PID_USART4)

#define sam_twi0_disableclk()      sam_disableperiph1(SAM_PID_TWI0)
#define sam_twi1_disableclk()      sam_disableperiph1(SAM_PID_TWI1)
#define sam_twi2_disableclk()      sam_disableperiph1(SAM_PID_TWI2)
#define sam_hsmci0_disableclk()    sam_disableperiph1(SAM_PID_HSMCI0)
#define sam_hsmci1_disableclk()    sam_disableperiph1(SAM_PID_HSMCI1)
#define sam_spi0_disableclk()      sam_disableperiph1(SAM_PID_SPI0)
#define sam_spi1_disableclk()      sam_disableperiph1(SAM_PID_SPI1)
#define sam_spi2_disableclk()      sam_disableperiph1(SAM_PID_SPI2)
#define sam_tc0_disableclk()       sam_disableperiph1(SAM_PID_TC0)
#define sam_tc1_disableclk()       sam_disableperiph1(SAM_PID_TC1)
#define sam_tc2_disableclk()       sam_disableperiph1(SAM_PID_TC2)
#define sam_pwm_disableclk()       sam_disableperiph1(SAM_PID_PWM)
#define sam_adc_disableclk()       sam_disableperiph1(SAM_PID_ADC)
#define sam_dbgu_disableclk()      sam_disableperiph1(SAM_PID_DBGU)
#define sam_uhphs_disableclk()     sam_disableperiph1(SAM_PID_UHPHS)
#define sam_udphs_disableclk()     sam_disableperiph1(SAM_PID_UDPHS)
#define sam_ssc0_disableclk()      sam_disableperiph1(SAM_PID_SSC0)
#define sam_ssc1_disableclk()      sam_disableperiph1(SAM_PID_SSC1)
#define sam_xdmac1_disableclk()    sam_disableperiph1(SAM_PID_XDMAC1)
#define sam_lcdc_disableclk()      sam_disableperiph1(SAM_PID_LCDC)
#define sam_isi_disableclk()       sam_disableperiph1(SAM_PID_ISI)
#define sam_trng_disableclk()      sam_disableperiph1(SAM_PID_TRNG)
#define sam_emac0_disableclk()     sam_disableperiph1(SAM_PID_EMAC0)
#define sam_emac1_disableclk()     sam_disableperiph1(SAM_PID_EMAC1)
#define sam_aicid_disableclk()     sam_disableperiph1(SAM_PID_AICID)
#define sam_sfc_disableclk()       sam_disableperiph1(SAM_PID_SFC)
#define sam_secureram_disableclk() sam_disableperiph1(SAM_PID_SECURAM)
#define sam_smd_disableclk()       sam_disableperiph1(SAM_PID_SMD)
#define sam_twi3_disableclk()      sam_disableperiph1(SAM_PID_TWI3)
#define sam_catb_disableclk()      sam_disableperiph1(SAM_PID_CATB)

/* The Advanced Interrupt Controller and L2CC cache controllers are
 * continuously clocked. The Power Management Controller has no effect on
 * their behavior.
 *
 * (I presume that this is true of the SFR as well since it has no PMC
 *  bits to control its clocking).
 */

#define sam_sfr_disableclk()
#define sam_aic_disableclk()
#define sam_saic_disableclk()
#define sam_l2cc_disableclk()

/* Test if peripheral clocking is enabled */

#define sam_arm_isenabled()         sam_isenabled0(SAM_PID_ARM)
#define sam_pit_isenabled()         sam_isenabled0(SAM_PID_PIT)
#define sam_wdt_isenabled()         sam_isenabled0(SAM_PID_WDT)
#define sam_piod_isenabled()        sam_isenabled0(SAM_PID_PIOD)
#define sam_usart0_isenabled()      sam_isenabled0(SAM_PID_USART0)
#define sam_usart1_isenabled()      sam_isenabled0(SAM_PID_USART1)
#define sam_xdmac0_isenabled()      sam_isenabled0(SAM_PID_XDMAC0)
#define sam_icm_isenabled()         sam_isenabled0(SAM_PID_ICM)
#define sam_cpkcc_isenabled()       sam_isenabled0(SAM_PID_CPKCC)
#define sam_aes_isenabled()         sam_isenabled0(SAM_PID_AES)
#define sam_aesb_isenabled()        sam_isenabled0(SAM_PID_AESB)
#define sam_tdes_isenabled()        sam_isenabled0(SAM_PID_TDES)
#define sam_sha_isenabled()         sam_isenabled0(SAM_PID_SHA)
#define sam_mpddrc_isenabled()      sam_isenabled0(SAM_PID_MPDDRC)
#define sam_matrix1_isenabled()     sam_isenabled0(SAM_PID_MATRIX1)
#define sam_matrix0_isenabled()     sam_isenabled0(SAM_PID_MATRIX0)
#define sam_vdec_isenabled()        sam_isenabled0(SAM_PID_VDEC)
#define sam_sbm_isenabled()         sam_isenabled0(SAM_PID_SBM)
#define sam_hsmc_isenabled()        sam_isenabled0(SAM_PID_HSMC)
#define sam_pioa_isenabled()        sam_isenabled0(SAM_PID_PIOA)
#define sam_piob_isenabled()        sam_isenabled0(SAM_PID_PIOB)
#define sam_pioc_isenabled()        sam_isenabled0(SAM_PID_PIOC)
#define sam_pioe_isenabled()        sam_isenabled0(SAM_PID_PIOE)
#define sam_uart0_isenabled()       sam_isenabled0(SAM_PID_UART0)
#define sam_uart1_isenabled()       sam_isenabled0(SAM_PID_UART1)
#define sam_usart2_isenabled()      sam_isenabled0(SAM_PID_USART2)
#define sam_usart3_isenabled()      sam_isenabled0(SAM_PID_USART3)
#define sam_usart4_isenabled()      sam_isenabled0(SAM_PID_USART4)

#define sam_twi0_isenabled()        sam_isenabled1(SAM_PID_TWI0)
#define sam_twi1_isenabled()        sam_isenabled1(SAM_PID_TWI1)
#define sam_twi2_isenabled()        sam_isenabled1(SAM_PID_TWI2)
#define sam_hsmci0_isenabled()      sam_isenabled1(SAM_PID_HSMCI0)
#define sam_hsmci1_isenabled()      sam_isenabled1(SAM_PID_HSMCI1)
#define sam_spi0_isenabled()        sam_isenabled1(SAM_PID_SPI0)
#define sam_spi1_isenabled()        sam_isenabled1(SAM_PID_SPI1)
#define sam_spi2_isenabled()        sam_isenabled1(SAM_PID_SPI2)
#define sam_tc0_isenabled()         sam_isenabled1(SAM_PID_TC0)
#define sam_tc1_isenabled()         sam_isenabled1(SAM_PID_TC1)
#define sam_tc2_isenabled()         sam_isenabled1(SAM_PID_TC2)
#define sam_pwm_isenabled()         sam_isenabled1(SAM_PID_PWM)
#define sam_adc_isenabled()         sam_isenabled1(SAM_PID_ADC)
#define sam_dbgu_isenabled()        sam_isenabled1(SAM_PID_DBGU)
#define sam_uhphs_isenabled()       sam_isenabled1(SAM_PID_UHPHS)
#define sam_udphs_isenabled()       sam_isenabled1(SAM_PID_UDPHS)
#define sam_ssc0_isenabled()        sam_isenabled1(SAM_PID_SSC0)
#define sam_ssc1_isenabled()        sam_isenabled1(SAM_PID_SSC1)
#define sam_xdmac1_isenabled()      sam_isenabled1(SAM_PID_XDMAC1)
#define sam_lcdc_isenabled()        sam_isenabled1(SAM_PID_LCDC)
#define sam_isi_isenabled()         sam_isenabled1(SAM_PID_ISI)
#define sam_trng_isenabled()        sam_isenabled1(SAM_PID_TRNG)
#define sam_emac0_isenabled()       sam_isenabled1(SAM_PID_EMAC0)
#define sam_emac1_isenabled()       sam_isenabled1(SAM_PID_EMAC1)
#define sam_aicid_isenabled()       sam_isenabled1(SAM_PID_AICID)
#define sam_sfc_isenabled()         sam_isenabled1(SAM_PID_SFC)
#define sam_secureram_isenabled()   sam_isenabled1(SAM_PID_SECURAM)
#define sam_smd_isenabled()         sam_isenabled1(SAM_PID_SMD)
#define sam_twi3_isenabled()        sam_isenabled1(SAM_PID_TWI3)
#define sam_catb_isenabled()        sam_isenabled1(SAM_PID_CATB)

/* The Advanced Interrupt Controller and L2CC cache controllers are
 * continuously clocked. The Power Management Controller has no effect on
 * their behavior.
 *
 * (I presume that this is true of the SFR as well since it has no PMC
 *  bits to control its clocking).
 */

#define sam_sfr_isenabled()         (true)
#define sam_aic_isenabled()         (true)
#define sam_saic_isenabled()        (true)
#define sam_l2cc_isenabled()        (true)

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

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_SAMA5_SAMAD54X_PERIPHCLKS_H */
