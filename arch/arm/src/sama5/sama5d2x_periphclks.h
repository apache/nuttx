/************************************************************************************
 * arch/arm/src/sama5/sama5d2x_periphclks.h
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMA5_SAMAD52X_PERIPHCLKS_H
#define __ARCH_ARM_SRC_SAMA5_SAMAD52X_PERIPHCLKS_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <arch/irq.h>
#include "hardware/sam_pmc.h"

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

#define sam_fiq_enableclk()        /* No peripheral clock */
#define sam_arm_enableclk()        /* No peripheral clock */
#define sam_pit_enableclk()        sam_enableperiph0(SAM_PID_PIT)
#define sam_wdt_enableclk()        sam_enableperiph0(SAM_PID_WDT)
#define sam_emac0_enableclk()      sam_enableperiph0(SAM_PID_EMAC0)
#define sam_xdmac0_enableclk()     sam_enableperiph0(SAM_PID_XDMAC0)
#define sam_xdmac1_enableclk()     sam_enableperiph0(SAM_PID_XDMAC1)
#define sam_icm_enableclk()        sam_enableperiph0(SAM_PID_ICM)
#define sam_aes_enableclk()        sam_enableperiph0(SAM_PID_AES)
#define sam_aesb_enableclk()       sam_enableperiph0(SAM_PID_AESB)
#define sam_tdes_enableclk()       sam_enableperiph0(SAM_PID_TDES)
#define sam_sha_enableclk()        sam_enableperiph0(SAM_PID_SHA)
#define sam_mpddrc_enableclk()     sam_enableperiph0(SAM_PID_MPDDRC)
#define sam_matrix1_enableclk()    sam_enableperiph0(SAM_PID_MATRIX1)
#define sam_matrix0_enableclk()    sam_enableperiph0(SAM_PID_MATRIX0)
#define sam_secumod_enableclk()    sam_enableperiph0(SAM_PID_SECUMOD)
#define sam_hsmc_enableclk()       sam_enableperiph0(SAM_PID_HSMC)
#define sam_pioa_enableclk()       sam_enableperiph0(SAM_PID_PIOA)
#define sam_flexcom0_enableclk()   sam_enableperiph0(SAM_PID_FLEXCOM0)
#define sam_flexcom1_enableclk()   sam_enableperiph0(SAM_PID_FLEXCOM1)
#define sam_flexcom2_enableclk()   sam_enableperiph0(SAM_PID_FLEXCOM2)
#define sam_flexcom3_enableclk()   sam_enableperiph0(SAM_PID_FLEXCOM3)
#define sam_flexcom4_enableclk()   sam_enableperiph0(SAM_PID_FLEXCOM4)
#define sam_uart0_enableclk()      sam_enableperiph0(SAM_PID_UART0)
#define sam_uart1_enableclk()      sam_enableperiph0(SAM_PID_UART1)
#define sam_uart2_enableclk()      sam_enableperiph0(SAM_PID_UART2)
#define sam_uart3_enableclk()      sam_enableperiph0(SAM_PID_UART3)
#define sam_uart4_enableclk()      sam_enableperiph0(SAM_PID_UART4)
#define sam_twi0_enableclk()       sam_enableperiph0(SAM_PID_TWI0)
#define sam_twi1_enableclk()       sam_enableperiph0(SAM_PID_TWI1)
#define sam_sdmmc0_enableclk()     sam_enableperiph0(SAM_PID_SDMMC0)

#define sam_sdmmc1_enableclk()     sam_enableperiph1(SAM_PID_SDMMC1)
#define sam_spi0_enableclk()       sam_enableperiph1(SAM_PID_SPI0)
#define sam_spi1_enableclk()       sam_enableperiph1(SAM_PID_SPI1)
#define sam_tc0_enableclk()        sam_enableperiph1(SAM_PID_TC0)
#define sam_tc1_enableclk()        sam_enableperiph1(SAM_PID_TC1)
#define sam_pwm_enableclk()        sam_enableperiph1(SAM_PID_PWM)
#define sam_adc_enableclk()        sam_enableperiph1(SAM_PID_ADC)
#define sam_uhphs_enableclk()      sam_enableperiph1(SAM_PID_UHPHS)
#define sam_udphs_enableclk()      sam_enableperiph1(SAM_PID_UDPHS)
#define sam_ssc0_enableclk()       sam_enableperiph1(SAM_PID_SSC0)
#define sam_ssc1_enableclk()       sam_enableperiph1(SAM_PID_SSC1)
#define sam_lcdc_enableclk()       sam_enableperiph1(SAM_PID_LCDC)
#define sam_isc_enableclk()        sam_enableperiph1(SAM_PID_ISC)
#define sam_trng_enableclk()       sam_enableperiph1(SAM_PID_TRNG)
#define sam_pdmic_enableclk()      sam_enableperiph1(SAM_PID_PDMIC)
#define sam_irqid_enableclk()      sam_enableperiph1(SAM_PID_IRQID)
#define sam_sfc_enableclk()        sam_enableperiph1(SAM_PID_SFC)
#define sam_securam_enableclk()    sam_enableperiph1(SAM_PID_SECURAM)
#define sam_qspi0_enableclk()      sam_enableperiph1(SAM_PID_QSPI0)
#define sam_qspi1_enableclk()      sam_enableperiph1(SAM_PID_QSPI1)
#define sam_i2sc0_enableclk()      sam_enableperiph1(SAM_PID_I2SC0)
#define sam_i2sc1_enableclk()      sam_enableperiph1(SAM_PID_I2SC1)
#define sam_mcan0_enableclk()      sam_enableperiph1(SAM_PID_MCAN00)
#define sam_mcan1_enableclk()      sam_enableperiph1(SAM_PID_MCAN10)
#define sam_classd_enableclk()     sam_enableperiph1(SAM_PID_CLASSD)
#define sam_sfr_enableclk()        sam_enableperiph1(SAM_PID_SFR)
#define sam_saic_enableclk()       sam_enableperiph1(SAM_PID_SAIC)
#define sam_aic_enableclk()        sam_enableperiph1(SAM_PID_AIC)

#define sam_piob_enableclk()       /* No peripheral clock */
#define sam_pioc_enableclk()       /* No peripheral clock */
#define sam_piod_enableclk()       /* No peripheral clock */
#define sam_sys_enableclk()        /* No peripheral clock */
#define sam_acc_enableclk()        /* No peripheral clock */
#define sam_rxlp_enableclk()       /* No peripheral clock */
#define sam_sfrbu_enableclk()      /* No peripheral clock */
#define sam_chipid_enableclk()     /* No peripheral clock */

#define sam_fiq_disableclk()       /* No peripheral clock */
#define sam_arm_disableclk()       /* No peripheral clock */
#define sam_pit_disableclk()       sam_disableperiph0(SAM_PID_PIT)
#define sam_wdt_disableclk()       sam_disableperiph0(SAM_PID_WDT)
#define sam_emac0_disableclk()     sam_disableperiph0(SAM_PID_EMAC0)
#define sam_xdmac0_disableclk()    sam_disableperiph0(SAM_PID_XDMAC0)
#define sam_xdmac1_disableclk()    sam_disableperiph0(SAM_PID_XDMAC1)
#define sam_icm_disableclk()       sam_disableperiph0(SAM_PID_ICM)
#define sam_aes_disableclk()       sam_disableperiph0(SAM_PID_AES)
#define sam_aesb_disableclk()      sam_disableperiph0(SAM_PID_AESB)
#define sam_tdes_disableclk()      sam_disableperiph0(SAM_PID_TDES)
#define sam_sha_disableclk()       sam_disableperiph0(SAM_PID_SHA)
#define sam_mpddrc_disableclk()    sam_disableperiph0(SAM_PID_MPDDRC)
#define sam_matrix1_disableclk()   sam_disableperiph0(SAM_PID_MATRIX1)
#define sam_matrix0_disableclk()   sam_disableperiph0(SAM_PID_MATRIX0)
#define sam_secumod_disableclk()   sam_disableperiph0(SAM_PID_SECUMOD)
#define sam_hsmc_disableclk()      sam_disableperiph0(SAM_PID_HSMC)
#define sam_pio_disableclk()       sam_disableperiph0(SAM_PID_PIOA)
#define sam_flexcom0_disableclk()  sam_disableperiph0(SAM_PID_FLEXCOM0)
#define sam_flexcom1_disableclk()  sam_disableperiph0(SAM_PID_FLEXCOM1)
#define sam_flexcom2_disableclk()  sam_disableperiph0(SAM_PID_FLEXCOM2)
#define sam_flexcom3_disableclk()  sam_disableperiph0(SAM_PID_FLEXCOM3)
#define sam_flexcom4_disableclk()  sam_disableperiph0(SAM_PID_FLEXCOM4)
#define sam_uart0_disableclk()     sam_disableperiph0(SAM_PID_UART0)
#define sam_uart1_disableclk()     sam_disableperiph0(SAM_PID_UART1)
#define sam_uart2_disableclk()     sam_disableperiph0(SAM_PID_UART2)
#define sam_uart3_disableclk()     sam_disableperiph0(SAM_PID_UART3)
#define sam_uart4_disableclk()     sam_disableperiph0(SAM_PID_UART4)
#define sam_twi0_disableclk()      sam_disableperiph0(SAM_PID_TWI0)
#define sam_twi1_disableclk()      sam_disableperiph0(SAM_PID_TWI1)
#define sam_sdmmc0_disableclk()    sam_disableperiph0(SAM_PID_SDMMC0)

#define sam_sdmmc1_disableclk()    sam_disableperiph1(SAM_PID_SDMMC1)
#define sam_spi0_disableclk()      sam_disableperiph1(SAM_PID_SPI0)
#define sam_spi1_disableclk()      sam_disableperiph1(SAM_PID_SPI1)
#define sam_tc0_disableclk()       sam_disableperiph1(SAM_PID_TC0)
#define sam_tc1_disableclk()       sam_disableperiph1(SAM_PID_TC1)
#define sam_pwm_disableclk()       sam_disableperiph1(SAM_PID_PWM)
#define sam_adc_disableclk()       sam_disableperiph1(SAM_PID_ADC)
#define sam_uhphs_disableclk()     sam_disableperiph1(SAM_PID_UHPHS)
#define sam_udphs_disableclk()     sam_disableperiph1(SAM_PID_UDPHS)
#define sam_ssc0_disableclk()      sam_disableperiph1(SAM_PID_SSC0)
#define sam_ssc1_disableclk()      sam_disableperiph1(SAM_PID_SSC1)
#define sam_lcdc_disableclk()      sam_disableperiph1(SAM_PID_LCDC)
#define sam_isc_disableclk()       sam_disableperiph1(SAM_PID_ISC)
#define sam_trng_disableclk()      sam_disableperiph1(SAM_PID_TRNG)
#define sam_pdmic_disableclk()     sam_disableperiph1(SAM_PID_PDMIC)
#define sam_irqid_disableclk()     sam_disableperiph1(SAM_PID_IRQID)
#define sam_sfc_disableclk()       sam_disableperiph1(SAM_PID_SFC)
#define sam_securam_disableclk()   sam_disableperiph1(SAM_PID_SECURAM)
#define sam_qspi0_disableclk()     sam_disableperiph1(SAM_PID_QSPI0)
#define sam_qspi1_disableclk()     sam_disableperiph1(SAM_PID_QSPI1)
#define sam_i2sc0_disableclk()     sam_disableperiph1(SAM_PID_I2SC0)
#define sam_i2sc1_disableclk()     sam_disableperiph1(SAM_PID_I2SC1)
#define sam_mcan0_disableclk()     sam_disableperiph1(SAM_PID_MCAN00)
#define sam_mcan1_disableclk()     sam_disableperiph1(SAM_PID_MCAN10)
#define sam_classd_disableclk()    sam_disableperiph1(SAM_PID_CLASSD)
#define sam_sfr_disableclk()       sam_disableperiph1(SAM_PID_SFR)
#define sam_saic_disableclk()      sam_disableperiph1(SAM_PID_SAIC)
#define sam_aic_disableclk()       sam_disableperiph1(SAM_PID_AIC)

#define sam_piob_disableclk()      /* No peripheral clock */
#define sam_pioc_disableclk()      /* No peripheral clock */
#define sam_piod_disableclk()      /* No peripheral clock */
#define sam_sys_disableclk()       /* No peripheral clock */
#define sam_acc_disableclk()       /* No peripheral clock */
#define sam_rxlp_disableclk()      /* No peripheral clock */
#define sam_sfrbu_disableclk()     /* No peripheral clock */
#define sam_chipid_disableclk()    /* No peripheral clock */

#define sam_fiq_isenabled()        (false) /* No peripheral clock */
#define sam_arm_isenabled()        (false) /* No peripheral clock */
#define sam_pit_isenabled()        sam_isenabled0(SAM_PID_PIT)
#define sam_wdt_isenabled()        sam_isenabled0(SAM_PID_WDT)
#define sam_emac0_isenabled()      sam_isenabled0(SAM_PID_EMAC0)
#define sam_xdmac0_isenabled()     sam_isenabled0(SAM_PID_XDMAC0)
#define sam_xdmac1_isenabled()     sam_isenabled0(SAM_PID_XDMAC1)
#define sam_icm_isenabled()        sam_isenabled0(SAM_PID_ICM)
#define sam_aes_isenabled()        sam_isenabled0(SAM_PID_AES)
#define sam_aesb_isenabled()       sam_isenabled0(SAM_PID_AESB)
#define sam_tdes_isenabled()       sam_isenabled0(SAM_PID_TDES)
#define sam_sha_isenabled()        sam_isenabled0(SAM_PID_SHA)
#define sam_mpddrc_isenabled()     sam_isenabled0(SAM_PID_MPDDRC)
#define sam_matrix1_isenabled()    sam_isenabled0(SAM_PID_MATRIX1)
#define sam_matrix0_isenabled()    sam_isenabled0(SAM_PID_MATRIX0)
#define sam_secumod_isenabled()    sam_isenabled0(SAM_PID_SECUMOD)
#define sam_hsmc_isenabled()       sam_isenabled0(SAM_PID_HSMC)
#define sam_pio_isenabled()        sam_isenabled0(SAM_PID_PIOA)
#define sam_flexcom0_isenabled()   sam_isenabled0(SAM_PID_FLEXCOM0)
#define sam_flexcom1_isenabled()   sam_isenabled0(SAM_PID_FLEXCOM1)
#define sam_flexcom2_isenabled()   sam_isenabled0(SAM_PID_FLEXCOM2)
#define sam_flexcom3_isenabled()   sam_isenabled0(SAM_PID_FLEXCOM3)
#define sam_flexcom4_isenabled()   sam_isenabled0(SAM_PID_FLEXCOM4)
#define sam_uart0_isenabled()      sam_isenabled0(SAM_PID_UART0)
#define sam_uart1_isenabled()      sam_isenabled0(SAM_PID_UART1)
#define sam_uart2_isenabled()      sam_isenabled0(SAM_PID_UART2)
#define sam_uart3_isenabled()      sam_isenabled0(SAM_PID_UART3)
#define sam_uart4_isenabled()      sam_isenabled0(SAM_PID_UART4)
#define sam_twi0_isenabled()       sam_isenabled0(SAM_PID_TWI0)
#define sam_twi1_isenabled()       sam_isenabled0(SAM_PID_TWI1)
#define sam_sdmmc0_isenabled()     sam_isenabled0(SAM_PID_SDMMC0)

#define sam_sdmmc1_isenabled()     sam_isenabled1(SAM_PID_SDMMC1)
#define sam_spi0_isenabled()       sam_isenabled1(SAM_PID_SPI0)
#define sam_spi1_isenabled()       sam_isenabled1(SAM_PID_SPI1)
#define sam_tc0_isenabled()        sam_isenabled1(SAM_PID_TC0)
#define sam_tc1_isenabled()        sam_isenabled1(SAM_PID_TC1)
#define sam_pwm_isenabled()        sam_isenabled1(SAM_PID_PWM)
#define sam_adc_isenabled()        sam_isenabled1(SAM_PID_ADC)
#define sam_uhphs_isenabled()      sam_isenabled1(SAM_PID_UHPHS)
#define sam_udphs_isenabled()      sam_isenabled1(SAM_PID_UDPHS)
#define sam_ssc0_isenabled()       sam_isenabled1(SAM_PID_SSC0)
#define sam_ssc1_isenabled()       sam_isenabled1(SAM_PID_SSC1)
#define sam_lcdc_isenabled()       sam_isenabled1(SAM_PID_LCDC)
#define sam_isc_isenabled()        sam_isenabled1(SAM_PID_ISC)
#define sam_trng_isenabled()       sam_isenabled1(SAM_PID_TRNG)
#define sam_pdmic_isenabled()      sam_isenabled1(SAM_PID_PDMIC)
#define sam_irqid_isenabled()      sam_isenabled1(SAM_PID_IRQID)
#define sam_sfc_isenabled()        sam_isenabled1(SAM_PID_SFC)
#define sam_securam_isenabled()    sam_isenabled1(SAM_PID_SECURAM)
#define sam_qspi0_isenabled()      sam_isenabled1(SAM_PID_QSPI0)
#define sam_qspi1_isenabled()      sam_isenabled1(SAM_PID_QSPI1)
#define sam_i2sc0_isenabled()      sam_isenabled1(SAM_PID_I2SC0)
#define sam_i2sc1_isenabled()      sam_isenabled1(SAM_PID_I2SC1)
#define sam_mcan0_isenabled()      sam_isenabled1(SAM_PID_MCAN00)
#define sam_mcan1_isenabled()      sam_isenabled1(SAM_PID_MCAN10)
#define sam_classd_isenabled()     sam_isenabled1(SAM_PID_CLASSD)
#define sam_sfr_isenabled()        sam_isenabled1(SAM_PID_SFR)
#define sam_saic_isenabled()       sam_isenabled1(SAM_PID_SAIC)
#define sam_aic_isenabled()        sam_isenabled1(SAM_PID_AIC)

#define sam_piob_isenabled()       (false) /* No peripheral clock */
#define sam_pioc_isenabled()       (false) /* No peripheral clock */
#define sam_piod_isenabled()       (false) /* No peripheral clock */
#define sam_sys_isenabled()        (false) /* No peripheral clock */
#define sam_acc_isenabled()        (false) /* No peripheral clock */
#define sam_rxlp_isenabled()       (false) /* No peripheral clock */
#define sam_sfrbu_isenabled()      (false) /* No peripheral clock */
#define sam_chipid_isenabled()     (false) /* No peripheral clock */

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
#endif /* __ARCH_ARM_SRC_SAMA5_SAMAD52X_PERIPHCLKS_H */
