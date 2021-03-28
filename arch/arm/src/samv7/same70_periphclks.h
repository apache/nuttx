/****************************************************************************
 * arch/arm/src/samv7/same70_periphclks.h
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

#ifndef __ARCH_ARM_SRC_SAMV7_SAME70_PERIPHCLKS_H
#define __ARCH_ARM_SRC_SAMV7_SAME70_PERIPHCLKS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <arch/irq.h>
#include "hardware/sam_pmc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Helper macros */

#define sam_enableperiph0(s)       putreg32((1 << (s)), SAM_PMC_PCER0)
#define sam_enableperiph1(s)       putreg32((1 << ((s) - 32)), SAM_PMC_PCER1)
#define sam_disableperiph0(s)      putreg32((1 << (s)), SAM_PMC_PCDR0)
#define sam_disableperiph1(s)      putreg32((1 << ((s) - 32)), SAM_PMC_PCDR1)

#define sam_supc_enableclk()
#define sam_rstc_enableclk()
#define sam_rtc_enableclk()
#define sam_rtt_enableclk()
#define sam_wdt0_enableclk()
#define sam_pmc_enableclk()
#define sam_efc_enableclk()

#define sam_uart0_enableclk()      sam_enableperiph0(SAM_PID_UART0)
#define sam_uart1_enableclk()      sam_enableperiph0(SAM_PID_UART1)
#define sam_smc_enableclk()        sam_enableperiph0(SAM_PID_SMC)
#define sam_pioa_enableclk()       sam_enableperiph0(SAM_PID_PIOA)
#define sam_piob_enableclk()       sam_enableperiph0(SAM_PID_PIOB)
#define sam_pioc_enableclk()       sam_enableperiph0(SAM_PID_PIOC)
#define sam_usart0_enableclk()     sam_enableperiph0(SAM_PID_USART0)
#define sam_usart1_enableclk()     sam_enableperiph0(SAM_PID_USART1)
#define sam_usart2_enableclk()     sam_enableperiph0(SAM_PID_USART2)
#define sam_piod_enableclk()       sam_enableperiph0(SAM_PID_PIOD)
#define sam_pioe_enableclk()       sam_enableperiph0(SAM_PID_PIOE)
#define sam_hsmci0_enableclk()     sam_enableperiph0(SAM_PID_HSMCI0)
#define sam_twihs0_enableclk()     sam_enableperiph0(SAM_PID_TWIHS0)
#define sam_twihs1_enableclk()     sam_enableperiph0(SAM_PID_TWIHS1)
#define sam_spi0_enableclk()       sam_enableperiph0(SAM_PID_SPI0)
#define sam_ssc_enableclk()        sam_enableperiph0(SAM_PID_SSC0)
#define sam_tc0_enableclk()        sam_enableperiph0(SAM_PID_TC0)
#define sam_tc1_enableclk()        sam_enableperiph0(SAM_PID_TC1)
#define sam_tc2_enableclk()        sam_enableperiph0(SAM_PID_TC2)
#define sam_tc3_enableclk()        sam_enableperiph0(SAM_PID_TC3)
#define sam_tc4_enableclk()        sam_enableperiph0(SAM_PID_TC4)
#define sam_tc5_enableclk()        sam_enableperiph0(SAM_PID_TC5)
#define sam_afec0_enableclk()      sam_enableperiph0(SAM_PID_AFEC0)
#define sam_dacc_enableclk()       sam_enableperiph0(SAM_PID_DACC)
#define sam_pwm0_enableclk()       sam_enableperiph0(SAM_PID_PWM0)

#define sam_icm_enableclk()        sam_enableperiph1(SAM_PID_ICM)
#define sam_acc_enableclk()        sam_enableperiph1(SAM_PID_ACC)
#define sam_usbhs_enableclk()      sam_enableperiph1(SAM_PID_USBHS)
#define sam_mcan00_enableclk()     sam_enableperiph1(SAM_PID_MCAN00)
#define sam_mcan01_enableclk()
#define sam_mcan10_enableclk()     sam_enableperiph1(SAM_PID_MCAN10)
#define sam_mcan11_enableclk()
#define sam_emac0_enableclk()      sam_enableperiph1(SAM_PID_EMAC0)
#define sam_afec1_enableclk()      sam_enableperiph1(SAM_PID_AFEC1)
#define sam_twihs2_enableclk()     sam_enableperiph1(SAM_PID_TWIHS2)
#define sam_spi1_enableclk()       sam_enableperiph1(SAM_PID_SPI1)
#define sam_qspi_enableclk()       sam_enableperiph1(SAM_PID_QSPI)
#define sam_uart2_enableclk()      sam_enableperiph1(SAM_PID_UART2)
#define sam_uart3_enableclk()      sam_enableperiph1(SAM_PID_UART3)
#define sam_uart4_enableclk()      sam_enableperiph1(SAM_PID_UART4)
#define sam_tc6_enableclk()        sam_enableperiph1(SAM_PID_TC6)
#define sam_tc7_enableclk()        sam_enableperiph1(SAM_PID_TC7)
#define sam_tc8_enableclk()        sam_enableperiph1(SAM_PID_TC8)
#define sam_tc9_enableclk()        sam_enableperiph1(SAM_PID_TC9)
#define sam_tc10_enableclk()       sam_enableperiph1(SAM_PID_TC10)
#define sam_tc11_enableclk()       sam_enableperiph1(SAM_PID_TC11)
#define sam_aes_enableclk()        sam_enableperiph1(SAM_PID_AES)
#define sam_trng_enableclk()       sam_enableperiph1(SAM_PID_TRNG)
#define sam_xdmac_enableclk()      sam_enableperiph1(SAM_PID_XDMAC)
#define sam_isi_enableclk()        sam_enableperiph1(SAM_PID_ISI)
#define sam_pwm1_enableclk()       sam_enableperiph1(SAM_PID_PWM1)
#define sam_fpu_enableclk()
#define sam_sdramc_enableclk()     sam_enableperiph1(SAM_PID_SDRAMC) /* REVISIT: Does this exist? */
#define sam_wdt1_enableclk()

#define sam_ccw_enableclk()

#define sam_supc_disableclk()
#define sam_rstc_disableclk()
#define sam_rtc_disableclk()
#define sam_rtt_disableclk()
#define sam_wdt0_disableclk()
#define sam_pmc_disableclk()
#define sam_efc_disableclk()

#define sam_uart0_disableclk()     sam_disableperiph0(SAM_PID_UART0)
#define sam_uart1_disableclk()     sam_disableperiph0(SAM_PID_UART1)
#define sam_smc_disableclk()       sam_disableperiph0(SAM_PID_SMC)
#define sam_pioa_disableclk()      sam_disableperiph0(SAM_PID_PIOA)
#define sam_piob_disableclk()      sam_disableperiph0(SAM_PID_PIOB)
#define sam_pioc_disableclk()      sam_disableperiph0(SAM_PID_PIOC)
#define sam_usart0_disableclk()    sam_disableperiph0(SAM_PID_USART0)
#define sam_usart1_disableclk()    sam_disableperiph0(SAM_PID_USART1)
#define sam_usart2_disableclk()    sam_disableperiph0(SAM_PID_USART2)
#define sam_piod_disableclk()      sam_disableperiph0(SAM_PID_PIOD)
#define sam_pioe_disableclk()      sam_disableperiph0(SAM_PID_PIOE)
#define sam_hsmci0_disableclk()    sam_disableperiph0(SAM_PID_HSMCI0)
#define sam_twihs0_disableclk()    sam_disableperiph0(SAM_PID_TWIHS0)
#define sam_twihs1_disableclk()    sam_disableperiph0(SAM_PID_TWIHS1)
#define sam_spi0_disableclk()      sam_disableperiph0(SAM_PID_SPI0)
#define sam_ssc_disableclk()       sam_disableperiph0(SAM_PID_SSC0)
#define sam_tc0_disableclk()       sam_disableperiph0(SAM_PID_TC0)
#define sam_tc1_disableclk()       sam_disableperiph0(SAM_PID_TC1)
#define sam_tc2_disableclk()       sam_disableperiph0(SAM_PID_TC2)
#define sam_tc3_disableclk()       sam_disableperiph0(SAM_PID_TC3)
#define sam_tc4_disableclk()       sam_disableperiph0(SAM_PID_TC4)
#define sam_tc5_disableclk()       sam_disableperiph0(SAM_PID_TC5)
#define sam_afec0_disableclk()     sam_disableperiph0(SAM_PID_AFEC0)
#define sam_dacc_disableclk()      sam_disableperiph0(SAM_PID_DACC)
#define sam_pwm0_disableclk()      sam_disableperiph0(SAM_PID_PWM0)

#define sam_icm_disableclk()       sam_disableperiph1(SAM_PID_ICM)
#define sam_acc_disableclk()       sam_disableperiph1(SAM_PID_ACC)
#define sam_usbhs_disableclk()     sam_disableperiph1(SAM_PID_USBHS)
#define sam_mcan00_disableclk()    sam_disableperiph1(SAM_PID_MCAN00)
#define sam_mcan01_disableclk()
#define sam_mcan10_disableclk()    sam_disableperiph1(SAM_PID_MCAN10)
#define sam_mcan11_disableclk()
#define sam_emac0_disableclk()     sam_disableperiph1(SAM_PID_EMAC0)
#define sam_afec1_disableclk()     sam_disableperiph1(SAM_PID_AFEC1)
#define sam_twihs2_disableclk()    sam_disableperiph1(SAM_PID_TWIHS2)
#define sam_spi1_disableclk()      sam_disableperiph1(SAM_PID_SPI1)
#define sam_qspi_disableclk()      sam_disableperiph1(SAM_PID_QSPI)
#define sam_uart2_disableclk()     sam_disableperiph1(SAM_PID_UART2)
#define sam_uart3_disableclk()     sam_disableperiph1(SAM_PID_UART3)
#define sam_uart4_disableclk()     sam_disableperiph1(SAM_PID_UART4)
#define sam_tc6_disableclk()       sam_disableperiph1(SAM_PID_TC6)
#define sam_tc7_disableclk()       sam_disableperiph1(SAM_PID_TC7)
#define sam_tc8_disableclk()       sam_disableperiph1(SAM_PID_TC8)
#define sam_tc9_disableclk()       sam_disableperiph1(SAM_PID_TC9)
#define sam_tc10_disableclk()      sam_disableperiph1(SAM_PID_TC10)
#define sam_tc11_disableclk()      sam_disableperiph1(SAM_PID_TC11)
#define sam_aes_disableclk()       sam_disableperiph1(SAM_PID_AES)
#define sam_trng_disableclk()      sam_disableperiph1(SAM_PID_TRNG)
#define sam_xdmac_disableclk()     sam_disableperiph1(SAM_PID_XDMAC)
#define sam_isi_disableclk()       sam_disableperiph1(SAM_PID_ISI)
#define sam_pwm1_disableclk()      sam_disableperiph1(SAM_PID_PWM1)
#define sam_fpu_disableclk()
#define sam_sdramc_disableclk()    sam_disableperiph1(SAM_PID_SDRAMC) /* REVISIT: Does this exist? */
#define sam_wdt1_disableclk()

#define sam_ccw_disableclk()

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
#endif /* __ARCH_ARM_SRC_SAMV7_SAME70_PERIPHCLKS_H */
