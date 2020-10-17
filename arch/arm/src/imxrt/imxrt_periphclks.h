/************************************************************************************
 * arch/arm/src/imxrt/imxrt_periphclks.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_IMXRT_IMXRT_PERIPHCLKS_H
#define __ARCH_ARM_SRC_IMXRT_IMXRT_PERIPHCLKS_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include "hardware/imxrt_ccm.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Clock is off during all modes. Stop enter hardware handshake is disabled. */

#define imxrt_clockoff_acmp1()            imxrt_periphclk_configure(CCM_CCGR_ACMP1, CCM_CG_OFF)
#define imxrt_clockoff_acmp2()            imxrt_periphclk_configure(CCM_CCGR_ACMP2, CCM_CG_OFF)
#define imxrt_clockoff_acmp3()            imxrt_periphclk_configure(CCM_CCGR_ACMP3, CCM_CG_OFF)
#define imxrt_clockoff_acmp4()            imxrt_periphclk_configure(CCM_CCGR_ACMP4, CCM_CG_OFF)
#define imxrt_clockoff_adc1()             imxrt_periphclk_configure(CCM_CCGR_ADC1, CCM_CG_OFF)
#define imxrt_clockoff_adc2()             imxrt_periphclk_configure(CCM_CCGR_ADC2, CCM_CG_OFF)
#define imxrt_clockoff_aips_tz1()         imxrt_periphclk_configure(CCM_CCGR_AIPS_TZ1, CCM_CG_OFF)
#define imxrt_clockoff_aips_tz2()         imxrt_periphclk_configure(CCM_CCGR_AIPS_TZ2, CCM_CG_OFF)
#define imxrt_clockoff_aips_tz3()         imxrt_periphclk_configure(CCM_CCGR_AIPS_TZ3, CCM_CG_OFF)
#define imxrt_clockoff_aipstz4()          imxrt_periphclk_configure(CCM_CCGR_AIPSTZ4, CCM_CG_OFF)
#define imxrt_clockoff_anadig()           imxrt_periphclk_configure(CCM_CCGR_ANADIG, CCM_CG_OFF)
#define imxrt_clockoff_aoi1()             imxrt_periphclk_configure(CCM_CCGR_AOI1, CCM_CG_OFF)
#define imxrt_clockoff_aoi2()             imxrt_periphclk_configure(CCM_CCGR_AOI2, CCM_CG_OFF)
#define imxrt_clockoff_bee()              imxrt_periphclk_configure(CCM_CCGR_BEE, CCM_CG_OFF)
#define imxrt_clockoff_can1()             imxrt_periphclk_configure(CCM_CCGR_CAN1, CCM_CG_OFF)
#define imxrt_clockoff_can1_serial()      imxrt_periphclk_configure(CCM_CCGR_CAN1_SERIAL, CCM_CG_OFF)
#define imxrt_clockoff_can2()             imxrt_periphclk_configure(CCM_CCGR_CAN2, CCM_CG_OFF)
#define imxrt_clockoff_can2_serial()      imxrt_periphclk_configure(CCM_CCGR_CAN2_SERIAL, CCM_CG_OFF)
#define imxrt_clockoff_can3()             imxrt_periphclk_configure(CCM_CCGR_CAN3, CCM_CG_OFF)
#define imxrt_clockoff_can3_serial()      imxrt_periphclk_configure(CCM_CCGR_CAN3_SERIAL, CCM_CG_OFF)
#define imxrt_clockoff_csi()              imxrt_periphclk_configure(CCM_CCGR_CSI, CCM_CG_OFF)
#define imxrt_clockoff_csu()              imxrt_periphclk_configure(CCM_CCGR_CSU, CCM_CG_OFF)
#define imxrt_clockoff_dcdc()             imxrt_periphclk_configure(CCM_CCGR_DCDC, CCM_CG_OFF)
#define imxrt_clockoff_dcp()              imxrt_periphclk_configure(CCM_CCGR_DCP, CCM_CG_OFF)
#define imxrt_clockoff_dma()              imxrt_periphclk_configure(CCM_CCGR_DMA, CCM_CG_OFF)
#define imxrt_clockoff_enc1()             imxrt_periphclk_configure(CCM_CCGR_ENC1, CCM_CG_OFF)
#define imxrt_clockoff_enc2()             imxrt_periphclk_configure(CCM_CCGR_ENC2, CCM_CG_OFF)
#define imxrt_clockoff_enc3()             imxrt_periphclk_configure(CCM_CCGR_ENC3, CCM_CG_OFF)
#define imxrt_clockoff_enc4()             imxrt_periphclk_configure(CCM_CCGR_ENC4, CCM_CG_OFF)
#define imxrt_clockoff_enet()             imxrt_periphclk_configure(CCM_CCGR_ENET, CCM_CG_OFF)
#define imxrt_clockoff_ewm()              imxrt_periphclk_configure(CCM_CCGR_EWM, CCM_CG_OFF)
#define imxrt_clockoff_flexio1()          imxrt_periphclk_configure(CCM_CCGR_FLEXIO1, CCM_CG_OFF)
#define imxrt_clockoff_flexio2()          imxrt_periphclk_configure(CCM_CCGR_FLEXIO2, CCM_CG_OFF)
#define imxrt_clockoff_flexram()          imxrt_periphclk_configure(CCM_CCGR_FLEXRAM, CCM_CG_OFF)
#define imxrt_clockoff_flexspi()          imxrt_periphclk_configure(CCM_CCGR_FLEXSPI, CCM_CG_OFF)
#define imxrt_clockoff_gpio1()            imxrt_periphclk_configure(CCM_CCGR_GPIO1, CCM_CG_OFF)
#define imxrt_clockoff_gpio2()            imxrt_periphclk_configure(CCM_CCGR_GPIO2, CCM_CG_OFF)
#define imxrt_clockoff_gpio3()            imxrt_periphclk_configure(CCM_CCGR_GPIO3, CCM_CG_OFF)
#define imxrt_clockoff_gpio4()            imxrt_periphclk_configure(CCM_CCGR_GPIO4, CCM_CG_OFF)
#define imxrt_clockoff_gpt_bus()          imxrt_periphclk_configure(CCM_CCGR_GPT_BUS, CCM_CG_OFF)
#define imxrt_clockoff_gpt_serial()       imxrt_periphclk_configure(CCM_CCGR_GPT_SERIAL, CCM_CG_OFF)
#define imxrt_clockoff_gpt2_bus()         imxrt_periphclk_configure(CCM_CCGR_GPT2_BUS, CCM_CG_OFF)
#define imxrt_clockoff_gpt2_serial()      imxrt_periphclk_configure(CCM_CCGR_GPT2_SERIAL, CCM_CG_OFF)
#define imxrt_clockoff_iomuxc()           imxrt_periphclk_configure(CCM_CCGR_IOMUXC, CCM_CG_OFF)
#define imxrt_clockoff_iomuxc_gpr()       imxrt_periphclk_configure(CCM_CCGR_IOMUXC_GPR, CCM_CG_OFF)
#define imxrt_clockoff_iomuxc_snvs()      imxrt_periphclk_configure(CCM_CCGR_IOMUXC_SNVS, CCM_CG_OFF)
#define imxrt_clockoff_iomuxc_snvs_gpr()  imxrt_periphclk_configure(CCM_CCGR_IOMUXC_SNVS_GPR, CCM_CG_OFF)
#define imxrt_clockoff_ipmux1()           imxrt_periphclk_configure(CCM_CCGR_IPMUX1, CCM_CG_OFF)
#define imxrt_clockoff_ipmux2()           imxrt_periphclk_configure(CCM_CCGR_IPMUX2, CCM_CG_OFF)
#define imxrt_clockoff_ipmux3()           imxrt_periphclk_configure(CCM_CCGR_IPMUX3, CCM_CG_OFF)
#define imxrt_clockoff_ipmux4()           imxrt_periphclk_configure(CCM_CCGR_IPMUX4, CCM_CG_OFF)
#define imxrt_clockoff_kpp()              imxrt_periphclk_configure(CCM_CCGR_KPP, CCM_CG_OFF)
#define imxrt_clockoff_lcd()              imxrt_periphclk_configure(CCM_CCGR_LCD, CCM_CG_OFF)
#define imxrt_clockoff_lcdif_pix()        imxrt_periphclk_configure(CCM_CCGR_LCDIF_PIX, CCM_CG_OFF)
#define imxrt_clockoff_lpi2c1()           imxrt_periphclk_configure(CCM_CCGR_LPI2C1, CCM_CG_OFF)
#define imxrt_clockoff_lpi2c2()           imxrt_periphclk_configure(CCM_CCGR_LPI2C2, CCM_CG_OFF)
#define imxrt_clockoff_lpi2c3()           imxrt_periphclk_configure(CCM_CCGR_LPI2C3, CCM_CG_OFF)
#define imxrt_clockoff_lpi2c4_serial()    imxrt_periphclk_configure(CCM_CCGR_LPI2C4_SERIAL, CCM_CG_OFF)
#define imxrt_clockoff_lpspi1()           imxrt_periphclk_configure(CCM_CCGR_LPSPI1, CCM_CG_OFF)
#define imxrt_clockoff_lpspi2()           imxrt_periphclk_configure(CCM_CCGR_LPSPI2, CCM_CG_OFF)
#define imxrt_clockoff_lpspi3()           imxrt_periphclk_configure(CCM_CCGR_LPSPI3, CCM_CG_OFF)
#define imxrt_clockoff_lpspi4()           imxrt_periphclk_configure(CCM_CCGR_LPSPI4, CCM_CG_OFF)
#define imxrt_clockoff_lpuart1()          imxrt_periphclk_configure(CCM_CCGR_LPUART1, CCM_CG_OFF)
#define imxrt_clockoff_lpuart2()          imxrt_periphclk_configure(CCM_CCGR_LPUART2, CCM_CG_OFF)
#define imxrt_clockoff_lpuart3()          imxrt_periphclk_configure(CCM_CCGR_LPUART3, CCM_CG_OFF)
#define imxrt_clockoff_lpuart4()          imxrt_periphclk_configure(CCM_CCGR_LPUART4, CCM_CG_OFF)
#define imxrt_clockoff_lpuart5()          imxrt_periphclk_configure(CCM_CCGR_LPUART5, CCM_CG_OFF)
#define imxrt_clockoff_lpuart6()          imxrt_periphclk_configure(CCM_CCGR_LPUART6, CCM_CG_OFF)
#define imxrt_clockoff_lpuart7()          imxrt_periphclk_configure(CCM_CCGR_LPUART7, CCM_CG_OFF)
#define imxrt_clockoff_lpuart8()          imxrt_periphclk_configure(CCM_CCGR_LPUART8, CCM_CG_OFF)
#define imxrt_clockoff_mqs()              imxrt_periphclk_configure(CCM_CCGR_MQS, CCM_CG_OFF)
#define imxrt_clockoff_ocotp_ctrl()       imxrt_periphclk_configure(CCM_CCGR_OCOTP_CTRL, CCM_CG_OFF)
#define imxrt_clockoff_ocram()            imxrt_periphclk_configure(CCM_CCGR_OCRAM, CCM_CG_OFF)
#define imxrt_clockoff_pit()              imxrt_periphclk_configure(CCM_CCGR_PIT, CCM_CG_OFF)
#define imxrt_clockoff_pwm1()             imxrt_periphclk_configure(CCM_CCGR_PWM1, CCM_CG_OFF)
#define imxrt_clockoff_pwm2()             imxrt_periphclk_configure(CCM_CCGR_PWM2, CCM_CG_OFF)
#define imxrt_clockoff_pwm3()             imxrt_periphclk_configure(CCM_CCGR_PWM3, CCM_CG_OFF)
#define imxrt_clockoff_pwm4()             imxrt_periphclk_configure(CCM_CCGR_PWM4, CCM_CG_OFF)
#define imxrt_clockoff_pxp()              imxrt_periphclk_configure(CCM_CCGR_PXP, CCM_CG_OFF)
#define imxrt_clockoff_rom()              imxrt_periphclk_configure(CCM_CCGR_ROM, CCM_CG_OFF)
#define imxrt_clockoff_sai1()             imxrt_periphclk_configure(CCM_CCGR_SAI1, CCM_CG_OFF)
#define imxrt_clockoff_sai2()             imxrt_periphclk_configure(CCM_CCGR_SAI2, CCM_CG_OFF)
#define imxrt_clockoff_sai3()             imxrt_periphclk_configure(CCM_CCGR_SAI3, CCM_CG_OFF)
#define imxrt_clockoff_semc()             imxrt_periphclk_configure(CCM_CCGR_SEMC, CCM_CG_OFF)
#define imxrt_clockoff_sim_ems()          imxrt_periphclk_configure(CCM_CCGR_SIM_EMS, CCM_CG_OFF)
#define imxrt_clockoff_sim_m()            imxrt_periphclk_configure(CCM_CCGR_SIM_M, CCM_CG_OFF)
#define imxrt_clockoff_sim_m7()           imxrt_periphclk_configure(CCM_CCGR_SIM_M7, CCM_CG_OFF)
#define imxrt_clockoff_sim_main()         imxrt_periphclk_configure(CCM_CCGR_SIM_MAIN, CCM_CG_OFF)
#define imxrt_clockoff_sim_per()          imxrt_periphclk_configure(CCM_CCGR_SIM_PER, CCM_CG_OFF)
#define imxrt_clockoff_snvs_hp()          imxrt_periphclk_configure(CCM_CCGR_SNVS_HP, CCM_CG_OFF)
#define imxrt_clockoff_snvs_lp()          imxrt_periphclk_configure(CCM_CCGR_SNVS_LP, CCM_CG_OFF)
#define imxrt_clockoff_spdif()            imxrt_periphclk_configure(CCM_CCGR_SPDIF, CCM_CG_OFF)
#define imxrt_clockoff_timer1()           imxrt_periphclk_configure(CCM_CCGR_TIMER1, CCM_CG_OFF)
#define imxrt_clockoff_timer2()           imxrt_periphclk_configure(CCM_CCGR_TIMER2, CCM_CG_OFF)
#define imxrt_clockoff_timer3()           imxrt_periphclk_configure(CCM_CCGR_TIMER3, CCM_CG_OFF)
#define imxrt_clockoff_timer4()           imxrt_periphclk_configure(CCM_CCGR_TIMER4, CCM_CG_OFF)
#define imxrt_clockoff_trace()            imxrt_periphclk_configure(CCM_CCGR_TRACE, CCM_CG_OFF)
#define imxrt_clockoff_trng()             imxrt_periphclk_configure(CCM_CCGR_TRNG, CCM_CG_OFF)
#define imxrt_clockoff_tsc_dig()          imxrt_periphclk_configure(CCM_CCGR_TSC_DIG, CCM_CG_OFF)
#define imxrt_clockoff_usboh3()           imxrt_periphclk_configure(CCM_CCGR_USBOH3, CCM_CG_OFF)
#define imxrt_clockoff_usdhc1()           imxrt_periphclk_configure(CCM_CCGR_USDHC1, CCM_CG_OFF)
#define imxrt_clockoff_usdhc2()           imxrt_periphclk_configure(CCM_CCGR_USDHC2, CCM_CG_OFF)
#define imxrt_clockoff_wdog1()            imxrt_periphclk_configure(CCM_CCGR_WDOG1, CCM_CG_OFF)
#define imxrt_clockoff_wdog2()            imxrt_periphclk_configure(CCM_CCGR_WDOG2, CCM_CG_OFF)
#define imxrt_clockoff_wdog3()            imxrt_periphclk_configure(CCM_CCGR_WDOG3, CCM_CG_OFF)
#define imxrt_clockoff_xbar1()            imxrt_periphclk_configure(CCM_CCGR_XBAR1, CCM_CG_OFF)
#define imxrt_clockoff_xbar2()            imxrt_periphclk_configure(CCM_CCGR_XBAR2, CCM_CG_OFF)
#define imxrt_clockoff_xbar3()            imxrt_periphclk_configure(CCM_CCGR_XBAR3, CCM_CG_OFF)

/* Clock is on in run mode, but off in WAIT and STOP modes. */

#define imxrt_clockrun_acmp1()            imxrt_periphclk_configure(CCM_CCGR_ACMP1, CCM_CG_RUN)
#define imxrt_clockrun_acmp2()            imxrt_periphclk_configure(CCM_CCGR_ACMP2, CCM_CG_RUN)
#define imxrt_clockrun_acmp3()            imxrt_periphclk_configure(CCM_CCGR_ACMP3, CCM_CG_RUN)
#define imxrt_clockrun_acmp4()            imxrt_periphclk_configure(CCM_CCGR_ACMP4, CCM_CG_RUN)
#define imxrt_clockrun_adc1()             imxrt_periphclk_configure(CCM_CCGR_ADC1, CCM_CG_RUN)
#define imxrt_clockrun_adc2()             imxrt_periphclk_configure(CCM_CCGR_ADC2, CCM_CG_RUN)
#define imxrt_clockrun_aips_tz1()         imxrt_periphclk_configure(CCM_CCGR_AIPS_TZ1, CCM_CG_RUN)
#define imxrt_clockrun_aips_tz2()         imxrt_periphclk_configure(CCM_CCGR_AIPS_TZ2, CCM_CG_RUN)
#define imxrt_clockrun_aips_tz3()         imxrt_periphclk_configure(CCM_CCGR_AIPS_TZ3, CCM_CG_RUN)
#define imxrt_clockrun_aipstz4()          imxrt_periphclk_configure(CCM_CCGR_AIPSTZ4, CCM_CG_RUN)
#define imxrt_clockrun_anadig()           imxrt_periphclk_configure(CCM_CCGR_ANADIG, CCM_CG_RUN)
#define imxrt_clockrun_aoi1()             imxrt_periphclk_configure(CCM_CCGR_AOI1, CCM_CG_RUN)
#define imxrt_clockrun_aoi2()             imxrt_periphclk_configure(CCM_CCGR_AOI2, CCM_CG_RUN)
#define imxrt_clockrun_bee()              imxrt_periphclk_configure(CCM_CCGR_BEE, CCM_CG_RUN)
#define imxrt_clockrun_can1()             imxrt_periphclk_configure(CCM_CCGR_CAN1, CCM_CG_RUN)
#define imxrt_clockrun_can1_serial()      imxrt_periphclk_configure(CCM_CCGR_CAN1_SERIAL, CCM_CG_RUN)
#define imxrt_clockrun_can2()             imxrt_periphclk_configure(CCM_CCGR_CAN2, CCM_CG_RUN)
#define imxrt_clockrun_can2_serial()      imxrt_periphclk_configure(CCM_CCGR_CAN2_SERIAL, CCM_CG_RUN)
#define imxrt_clockrun_can3()             imxrt_periphclk_configure(CCM_CCGR_CAN3, CCM_CG_RUN)
#define imxrt_clockrun_can3_serial()      imxrt_periphclk_configure(CCM_CCGR_CAN3_SERIAL, CCM_CG_RUN)
#define imxrt_clockrun_csi()              imxrt_periphclk_configure(CCM_CCGR_CSI, CCM_CG_RUN)
#define imxrt_clockrun_csu()              imxrt_periphclk_configure(CCM_CCGR_CSU, CCM_CG_RUN)
#define imxrt_clockrun_dcdc()             imxrt_periphclk_configure(CCM_CCGR_DCDC, CCM_CG_RUN)
#define imxrt_clockrun_dcp()              imxrt_periphclk_configure(CCM_CCGR_DCP, CCM_CG_RUN)
#define imxrt_clockrun_dma()              imxrt_periphclk_configure(CCM_CCGR_DMA, CCM_CG_RUN)
#define imxrt_clockrun_enc1()             imxrt_periphclk_configure(CCM_CCGR_ENC1, CCM_CG_RUN)
#define imxrt_clockrun_enc2()             imxrt_periphclk_configure(CCM_CCGR_ENC2, CCM_CG_RUN)
#define imxrt_clockrun_enc3()             imxrt_periphclk_configure(CCM_CCGR_ENC3, CCM_CG_RUN)
#define imxrt_clockrun_enc4()             imxrt_periphclk_configure(CCM_CCGR_ENC4, CCM_CG_RUN)
#define imxrt_clockrun_enet()             imxrt_periphclk_configure(CCM_CCGR_ENET, CCM_CG_RUN)
#define imxrt_clockrun_ewm()              imxrt_periphclk_configure(CCM_CCGR_EWM, CCM_CG_RUN)
#define imxrt_clockrun_flexio1()          imxrt_periphclk_configure(CCM_CCGR_FLEXIO1, CCM_CG_RUN)
#define imxrt_clockrun_flexio2()          imxrt_periphclk_configure(CCM_CCGR_FLEXIO2, CCM_CG_RUN)
#define imxrt_clockrun_flexram()          imxrt_periphclk_configure(CCM_CCGR_FLEXRAM, CCM_CG_RUN)
#define imxrt_clockrun_flexspi()          imxrt_periphclk_configure(CCM_CCGR_FLEXSPI, CCM_CG_RUN)
#define imxrt_clockrun_gpio1()            imxrt_periphclk_configure(CCM_CCGR_GPIO1, CCM_CG_RUN)
#define imxrt_clockrun_gpio2()            imxrt_periphclk_configure(CCM_CCGR_GPIO2, CCM_CG_RUN)
#define imxrt_clockrun_gpio3()            imxrt_periphclk_configure(CCM_CCGR_GPIO3, CCM_CG_RUN)
#define imxrt_clockrun_gpio4()            imxrt_periphclk_configure(CCM_CCGR_GPIO4, CCM_CG_RUN)
#define imxrt_clockrun_gpt_bus()          imxrt_periphclk_configure(CCM_CCGR_GPT_BUS, CCM_CG_RUN)
#define imxrt_clockrun_gpt_serial()       imxrt_periphclk_configure(CCM_CCGR_GPT_SERIAL, CCM_CG_RUN)
#define imxrt_clockrun_gpt2_bus()         imxrt_periphclk_configure(CCM_CCGR_GPT2_BUS, CCM_CG_RUN)
#define imxrt_clockrun_gpt2_serial()      imxrt_periphclk_configure(CCM_CCGR_GPT2_SERIAL, CCM_CG_RUN)
#define imxrt_clockrun_iomuxc()           imxrt_periphclk_configure(CCM_CCGR_IOMUXC, CCM_CG_RUN)
#define imxrt_clockrun_iomuxc_gpr()       imxrt_periphclk_configure(CCM_CCGR_IOMUXC_GPR, CCM_CG_RUN)
#define imxrt_clockrun_iomuxc_snvs()      imxrt_periphclk_configure(CCM_CCGR_IOMUXC_SNVS, CCM_CG_RUN)
#define imxrt_clockrun_iomuxc_snvs_gpr()  imxrt_periphclk_configure(CCM_CCGR_IOMUXC_SNVS_GPR, CCM_CG_RUN)
#define imxrt_clockrun_ipmux1()           imxrt_periphclk_configure(CCM_CCGR_IPMUX1, CCM_CG_RUN)
#define imxrt_clockrun_ipmux2()           imxrt_periphclk_configure(CCM_CCGR_IPMUX2, CCM_CG_RUN)
#define imxrt_clockrun_ipmux3()           imxrt_periphclk_configure(CCM_CCGR_IPMUX3, CCM_CG_RUN)
#define imxrt_clockrun_ipmux4()           imxrt_periphclk_configure(CCM_CCGR_IPMUX4, CCM_CG_RUN)
#define imxrt_clockrun_kpp()              imxrt_periphclk_configure(CCM_CCGR_KPP, CCM_CG_RUN)
#define imxrt_clockrun_lcd()              imxrt_periphclk_configure(CCM_CCGR_LCD, CCM_CG_RUN)
#define imxrt_clockrun_lcdif_pix()        imxrt_periphclk_configure(CCM_CCGR_LCDIF_PIX, CCM_CG_RUN)
#define imxrt_clockrun_lpi2c1()           imxrt_periphclk_configure(CCM_CCGR_LPI2C1, CCM_CG_RUN)
#define imxrt_clockrun_lpi2c2()           imxrt_periphclk_configure(CCM_CCGR_LPI2C2, CCM_CG_RUN)
#define imxrt_clockrun_lpi2c3()           imxrt_periphclk_configure(CCM_CCGR_LPI2C3, CCM_CG_RUN)
#define imxrt_clockrun_lpi2c4_serial()    imxrt_periphclk_configure(CCM_CCGR_LPI2C4_SERIAL, CCM_CG_RUN)
#define imxrt_clockrun_lpspi1()           imxrt_periphclk_configure(CCM_CCGR_LPSPI1, CCM_CG_RUN)
#define imxrt_clockrun_lpspi2()           imxrt_periphclk_configure(CCM_CCGR_LPSPI2, CCM_CG_RUN)
#define imxrt_clockrun_lpspi3()           imxrt_periphclk_configure(CCM_CCGR_LPSPI3, CCM_CG_RUN)
#define imxrt_clockrun_lpspi4()           imxrt_periphclk_configure(CCM_CCGR_LPSPI4, CCM_CG_RUN)
#define imxrt_clockrun_lpuart1()          imxrt_periphclk_configure(CCM_CCGR_LPUART1, CCM_CG_RUN)
#define imxrt_clockrun_lpuart2()          imxrt_periphclk_configure(CCM_CCGR_LPUART2, CCM_CG_RUN)
#define imxrt_clockrun_lpuart3()          imxrt_periphclk_configure(CCM_CCGR_LPUART3, CCM_CG_RUN)
#define imxrt_clockrun_lpuart4()          imxrt_periphclk_configure(CCM_CCGR_LPUART4, CCM_CG_RUN)
#define imxrt_clockrun_lpuart5()          imxrt_periphclk_configure(CCM_CCGR_LPUART5, CCM_CG_RUN)
#define imxrt_clockrun_lpuart6()          imxrt_periphclk_configure(CCM_CCGR_LPUART6, CCM_CG_RUN)
#define imxrt_clockrun_lpuart7()          imxrt_periphclk_configure(CCM_CCGR_LPUART7, CCM_CG_RUN)
#define imxrt_clockrun_lpuart8()          imxrt_periphclk_configure(CCM_CCGR_LPUART8, CCM_CG_RUN)
#define imxrt_clockrun_mqs()              imxrt_periphclk_configure(CCM_CCGR_MQS, CCM_CG_RUN)
#define imxrt_clockrun_ocotp_ctrl()       imxrt_periphclk_configure(CCM_CCGR_OCOTP_CTRL, CCM_CG_RUN)
#define imxrt_clockrun_ocram()            imxrt_periphclk_configure(CCM_CCGR_OCRAM, CCM_CG_RUN)
#define imxrt_clockrun_pit()              imxrt_periphclk_configure(CCM_CCGR_PIT, CCM_CG_RUN)
#define imxrt_clockrun_pwm1()             imxrt_periphclk_configure(CCM_CCGR_PWM1, CCM_CG_RUN)
#define imxrt_clockrun_pwm2()             imxrt_periphclk_configure(CCM_CCGR_PWM2, CCM_CG_RUN)
#define imxrt_clockrun_pwm3()             imxrt_periphclk_configure(CCM_CCGR_PWM3, CCM_CG_RUN)
#define imxrt_clockrun_pwm4()             imxrt_periphclk_configure(CCM_CCGR_PWM4, CCM_CG_RUN)
#define imxrt_clockrun_pxp()              imxrt_periphclk_configure(CCM_CCGR_PXP, CCM_CG_RUN)
#define imxrt_clockrun_rom()              imxrt_periphclk_configure(CCM_CCGR_ROM, CCM_CG_RUN)
#define imxrt_clockrun_sai1()             imxrt_periphclk_configure(CCM_CCGR_SAI1, CCM_CG_RUN)
#define imxrt_clockrun_sai2()             imxrt_periphclk_configure(CCM_CCGR_SAI2, CCM_CG_RUN)
#define imxrt_clockrun_sai3()             imxrt_periphclk_configure(CCM_CCGR_SAI3, CCM_CG_RUN)
#define imxrt_clockrun_semc()             imxrt_periphclk_configure(CCM_CCGR_SEMC, CCM_CG_RUN)
#define imxrt_clockrun_sim_ems()          imxrt_periphclk_configure(CCM_CCGR_SIM_EMS, CCM_CG_RUN)
#define imxrt_clockrun_sim_m()            imxrt_periphclk_configure(CCM_CCGR_SIM_M, CCM_CG_RUN)
#define imxrt_clockrun_sim_m7()           imxrt_periphclk_configure(CCM_CCGR_SIM_M7, CCM_CG_RUN)
#define imxrt_clockrun_sim_main()         imxrt_periphclk_configure(CCM_CCGR_SIM_MAIN, CCM_CG_RUN)
#define imxrt_clockrun_sim_per()          imxrt_periphclk_configure(CCM_CCGR_SIM_PER, CCM_CG_RUN)
#define imxrt_clockrun_snvs_hp()          imxrt_periphclk_configure(CCM_CCGR_SNVS_HP, CCM_CG_RUN)
#define imxrt_clockrun_snvs_lp()          imxrt_periphclk_configure(CCM_CCGR_SNVS_LP, CCM_CG_RUN)
#define imxrt_clockrun_spdif()            imxrt_periphclk_configure(CCM_CCGR_SPDIF, CCM_CG_RUN)
#define imxrt_clockrun_timer1()           imxrt_periphclk_configure(CCM_CCGR_TIMER1, CCM_CG_RUN)
#define imxrt_clockrun_timer2()           imxrt_periphclk_configure(CCM_CCGR_TIMER2, CCM_CG_RUN)
#define imxrt_clockrun_timer3()           imxrt_periphclk_configure(CCM_CCGR_TIMER3, CCM_CG_RUN)
#define imxrt_clockrun_timer4()           imxrt_periphclk_configure(CCM_CCGR_TIMER4, CCM_CG_RUN)
#define imxrt_clockrun_trace()            imxrt_periphclk_configure(CCM_CCGR_TRACE, CCM_CG_RUN)
#define imxrt_clockrun_trng()             imxrt_periphclk_configure(CCM_CCGR_TRNG, CCM_CG_RUN)
#define imxrt_clockrun_tsc_dig()          imxrt_periphclk_configure(CCM_CCGR_TSC_DIG, CCM_CG_RUN)
#define imxrt_clockrun_usboh3()           imxrt_periphclk_configure(CCM_CCGR_USBOH3, CCM_CG_RUN)
#define imxrt_clockrun_usdhc1()           imxrt_periphclk_configure(CCM_CCGR_USDHC1, CCM_CG_RUN)
#define imxrt_clockrun_usdhc2()           imxrt_periphclk_configure(CCM_CCGR_USDHC2, CCM_CG_RUN)
#define imxrt_clockrun_wdog1()            imxrt_periphclk_configure(CCM_CCGR_WDOG1, CCM_CG_RUN)
#define imxrt_clockrun_wdog2()            imxrt_periphclk_configure(CCM_CCGR_WDOG2, CCM_CG_RUN)
#define imxrt_clockrun_wdog3()            imxrt_periphclk_configure(CCM_CCGR_WDOG3, CCM_CG_RUN)
#define imxrt_clockrun_xbar1()            imxrt_periphclk_configure(CCM_CCGR_XBAR1, CCM_CG_RUN)
#define imxrt_clockrun_xbar2()            imxrt_periphclk_configure(CCM_CCGR_XBAR2, CCM_CG_RUN)
#define imxrt_clockrun_xbar3()            imxrt_periphclk_configure(CCM_CCGR_XBAR3, CCM_CG_RUN)

/* Clock is on during all modes, except STOP mode. */

#define imxrt_clockall_acmp1()            imxrt_periphclk_configure(CCM_CCGR_ACMP1, CCM_CG_ALL)
#define imxrt_clockall_acmp2()            imxrt_periphclk_configure(CCM_CCGR_ACMP2, CCM_CG_ALL)
#define imxrt_clockall_acmp3()            imxrt_periphclk_configure(CCM_CCGR_ACMP3, CCM_CG_ALL)
#define imxrt_clockall_acmp4()            imxrt_periphclk_configure(CCM_CCGR_ACMP4, CCM_CG_ALL)
#define imxrt_clockall_adc1()             imxrt_periphclk_configure(CCM_CCGR_ADC1, CCM_CG_ALL)
#define imxrt_clockall_adc2()             imxrt_periphclk_configure(CCM_CCGR_ADC2, CCM_CG_ALL)
#define imxrt_clockall_aips_tz1()         imxrt_periphclk_configure(CCM_CCGR_AIPS_TZ1, CCM_CG_ALL)
#define imxrt_clockall_aips_tz2()         imxrt_periphclk_configure(CCM_CCGR_AIPS_TZ2, CCM_CG_ALL)
#define imxrt_clockall_aips_tz3()         imxrt_periphclk_configure(CCM_CCGR_AIPS_TZ3, CCM_CG_ALL)
#define imxrt_clockall_aipstz4()          imxrt_periphclk_configure(CCM_CCGR_AIPSTZ4, CCM_CG_ALL)
#define imxrt_clockall_anadig()           imxrt_periphclk_configure(CCM_CCGR_ANADIG, CCM_CG_ALL)
#define imxrt_clockall_aoi1()             imxrt_periphclk_configure(CCM_CCGR_AOI1, CCM_CG_ALL)
#define imxrt_clockall_aoi2()             imxrt_periphclk_configure(CCM_CCGR_AOI2, CCM_CG_ALL)
#define imxrt_clockall_bee()              imxrt_periphclk_configure(CCM_CCGR_BEE, CCM_CG_ALL)
#define imxrt_clockall_can1()             imxrt_periphclk_configure(CCM_CCGR_CAN1, CCM_CG_ALL)
#define imxrt_clockall_can1_serial()      imxrt_periphclk_configure(CCM_CCGR_CAN1_SERIAL, CCM_CG_ALL)
#define imxrt_clockall_can2()             imxrt_periphclk_configure(CCM_CCGR_CAN2, CCM_CG_ALL)
#define imxrt_clockall_can2_serial()      imxrt_periphclk_configure(CCM_CCGR_CAN2_SERIAL, CCM_CG_ALL)
#define imxrt_clockall_can3()             imxrt_periphclk_configure(CCM_CCGR_CAN3, CCM_CG_ALL)
#define imxrt_clockall_can3_serial()      imxrt_periphclk_configure(CCM_CCGR_CAN3_SERIAL, CCM_CG_ALL)
#define imxrt_clockall_csi()              imxrt_periphclk_configure(CCM_CCGR_CSI, CCM_CG_ALL)
#define imxrt_clockall_csu()              imxrt_periphclk_configure(CCM_CCGR_CSU, CCM_CG_ALL)
#define imxrt_clockall_dcdc()             imxrt_periphclk_configure(CCM_CCGR_DCDC, CCM_CG_ALL)
#define imxrt_clockall_dcp()              imxrt_periphclk_configure(CCM_CCGR_DCP, CCM_CG_ALL)
#define imxrt_clockall_dma()              imxrt_periphclk_configure(CCM_CCGR_DMA, CCM_CG_ALL)
#define imxrt_clockall_enc1()             imxrt_periphclk_configure(CCM_CCGR_ENC1, CCM_CG_ALL)
#define imxrt_clockall_enc2()             imxrt_periphclk_configure(CCM_CCGR_ENC2, CCM_CG_ALL)
#define imxrt_clockall_enc3()             imxrt_periphclk_configure(CCM_CCGR_ENC3, CCM_CG_ALL)
#define imxrt_clockall_enc4()             imxrt_periphclk_configure(CCM_CCGR_ENC4, CCM_CG_ALL)
#define imxrt_clockall_enet()             imxrt_periphclk_configure(CCM_CCGR_ENET, CCM_CG_ALL)
#define imxrt_clockall_ewm()              imxrt_periphclk_configure(CCM_CCGR_EWM, CCM_CG_ALL)
#define imxrt_clockall_flexio1()          imxrt_periphclk_configure(CCM_CCGR_FLEXIO1, CCM_CG_ALL)
#define imxrt_clockall_flexio2()          imxrt_periphclk_configure(CCM_CCGR_FLEXIO2, CCM_CG_ALL)
#define imxrt_clockall_flexram()          imxrt_periphclk_configure(CCM_CCGR_FLEXRAM, CCM_CG_ALL)
#define imxrt_clockall_flexspi()          imxrt_periphclk_configure(CCM_CCGR_FLEXSPI, CCM_CG_ALL)
#define imxrt_clockall_gpio1()            imxrt_periphclk_configure(CCM_CCGR_GPIO1, CCM_CG_ALL)
#define imxrt_clockall_gpio2()            imxrt_periphclk_configure(CCM_CCGR_GPIO2, CCM_CG_ALL)
#define imxrt_clockall_gpio3()            imxrt_periphclk_configure(CCM_CCGR_GPIO3, CCM_CG_ALL)
#define imxrt_clockall_gpio4()            imxrt_periphclk_configure(CCM_CCGR_GPIO4, CCM_CG_ALL)
#define imxrt_clockall_gpt_bus()          imxrt_periphclk_configure(CCM_CCGR_GPT_BUS, CCM_CG_ALL)
#define imxrt_clockall_gpt_serial()       imxrt_periphclk_configure(CCM_CCGR_GPT_SERIAL, CCM_CG_ALL)
#define imxrt_clockall_gpt2_bus()         imxrt_periphclk_configure(CCM_CCGR_GPT2_BUS, CCM_CG_ALL)
#define imxrt_clockall_gpt2_serial()      imxrt_periphclk_configure(CCM_CCGR_GPT2_SERIAL, CCM_CG_ALL)
#define imxrt_clockall_iomuxc()           imxrt_periphclk_configure(CCM_CCGR_IOMUXC, CCM_CG_ALL)
#define imxrt_clockall_iomuxc_gpr()       imxrt_periphclk_configure(CCM_CCGR_IOMUXC_GPR, CCM_CG_ALL)
#define imxrt_clockall_iomuxc_snvs()      imxrt_periphclk_configure(CCM_CCGR_IOMUXC_SNVS, CCM_CG_ALL)
#define imxrt_clockall_iomuxc_snvs_gpr()  imxrt_periphclk_configure(CCM_CCGR_IOMUXC_SNVS_GPR, CCM_CG_ALL)
#define imxrt_clockall_ipmux1()           imxrt_periphclk_configure(CCM_CCGR_IPMUX1, CCM_CG_ALL)
#define imxrt_clockall_ipmux2()           imxrt_periphclk_configure(CCM_CCGR_IPMUX2, CCM_CG_ALL)
#define imxrt_clockall_ipmux3()           imxrt_periphclk_configure(CCM_CCGR_IPMUX3, CCM_CG_ALL)
#define imxrt_clockall_ipmux4()           imxrt_periphclk_configure(CCM_CCGR_IPMUX4, CCM_CG_ALL)
#define imxrt_clockall_kpp()              imxrt_periphclk_configure(CCM_CCGR_KPP, CCM_CG_ALL)
#define imxrt_clockall_lcd()              imxrt_periphclk_configure(CCM_CCGR_LCD, CCM_CG_ALL)
#define imxrt_clockall_lcdif_pix()        imxrt_periphclk_configure(CCM_CCGR_LCDIF_PIX, CCM_CG_ALL)
#define imxrt_clockall_lpi2c1()           imxrt_periphclk_configure(CCM_CCGR_LPI2C1, CCM_CG_ALL)
#define imxrt_clockall_lpi2c2()           imxrt_periphclk_configure(CCM_CCGR_LPI2C2, CCM_CG_ALL)
#define imxrt_clockall_lpi2c3()           imxrt_periphclk_configure(CCM_CCGR_LPI2C3, CCM_CG_ALL)
#define imxrt_clockall_lpi2c4_serial()    imxrt_periphclk_configure(CCM_CCGR_LPI2C4_SERIAL, CCM_CG_ALL)
#define imxrt_clockall_lpspi1()           imxrt_periphclk_configure(CCM_CCGR_LPSPI1, CCM_CG_ALL)
#define imxrt_clockall_lpspi2()           imxrt_periphclk_configure(CCM_CCGR_LPSPI2, CCM_CG_ALL)
#define imxrt_clockall_lpspi3()           imxrt_periphclk_configure(CCM_CCGR_LPSPI3, CCM_CG_ALL)
#define imxrt_clockall_lpspi4()           imxrt_periphclk_configure(CCM_CCGR_LPSPI4, CCM_CG_ALL)
#define imxrt_clockall_lpuart1()          imxrt_periphclk_configure(CCM_CCGR_LPUART1, CCM_CG_ALL)
#define imxrt_clockall_lpuart2()          imxrt_periphclk_configure(CCM_CCGR_LPUART2, CCM_CG_ALL)
#define imxrt_clockall_lpuart3()          imxrt_periphclk_configure(CCM_CCGR_LPUART3, CCM_CG_ALL)
#define imxrt_clockall_lpuart4()          imxrt_periphclk_configure(CCM_CCGR_LPUART4, CCM_CG_ALL)
#define imxrt_clockall_lpuart5()          imxrt_periphclk_configure(CCM_CCGR_LPUART5, CCM_CG_ALL)
#define imxrt_clockall_lpuart6()          imxrt_periphclk_configure(CCM_CCGR_LPUART6, CCM_CG_ALL)
#define imxrt_clockall_lpuart7()          imxrt_periphclk_configure(CCM_CCGR_LPUART7, CCM_CG_ALL)
#define imxrt_clockall_lpuart8()          imxrt_periphclk_configure(CCM_CCGR_LPUART8, CCM_CG_ALL)
#define imxrt_clockall_mqs()              imxrt_periphclk_configure(CCM_CCGR_MQS, CCM_CG_ALL)
#define imxrt_clockall_ocotp_ctrl()       imxrt_periphclk_configure(CCM_CCGR_OCOTP_CTRL, CCM_CG_ALL)
#define imxrt_clockall_ocram()            imxrt_periphclk_configure(CCM_CCGR_OCRAM, CCM_CG_ALL)
#define imxrt_clockall_pit()              imxrt_periphclk_configure(CCM_CCGR_PIT, CCM_CG_ALL)
#define imxrt_clockall_pwm1()             imxrt_periphclk_configure(CCM_CCGR_PWM1, CCM_CG_ALL)
#define imxrt_clockall_pwm2()             imxrt_periphclk_configure(CCM_CCGR_PWM2, CCM_CG_ALL)
#define imxrt_clockall_pwm3()             imxrt_periphclk_configure(CCM_CCGR_PWM3, CCM_CG_ALL)
#define imxrt_clockall_pwm4()             imxrt_periphclk_configure(CCM_CCGR_PWM4, CCM_CG_ALL)
#define imxrt_clockall_pxp()              imxrt_periphclk_configure(CCM_CCGR_PXP, CCM_CG_ALL)
#define imxrt_clockall_rom()              imxrt_periphclk_configure(CCM_CCGR_ROM, CCM_CG_ALL)
#define imxrt_clockall_sai1()             imxrt_periphclk_configure(CCM_CCGR_SAI1, CCM_CG_ALL)
#define imxrt_clockall_sai2()             imxrt_periphclk_configure(CCM_CCGR_SAI2, CCM_CG_ALL)
#define imxrt_clockall_sai3()             imxrt_periphclk_configure(CCM_CCGR_SAI3, CCM_CG_ALL)
#define imxrt_clockall_semc()             imxrt_periphclk_configure(CCM_CCGR_SEMC, CCM_CG_ALL)
#define imxrt_clockall_sim_ems()          imxrt_periphclk_configure(CCM_CCGR_SIM_EMS, CCM_CG_ALL)
#define imxrt_clockall_sim_m()            imxrt_periphclk_configure(CCM_CCGR_SIM_M, CCM_CG_ALL)
#define imxrt_clockall_sim_m7()           imxrt_periphclk_configure(CCM_CCGR_SIM_M7, CCM_CG_ALL)
#define imxrt_clockall_sim_main()         imxrt_periphclk_configure(CCM_CCGR_SIM_MAIN, CCM_CG_ALL)
#define imxrt_clockall_sim_per()          imxrt_periphclk_configure(CCM_CCGR_SIM_PER, CCM_CG_ALL)
#define imxrt_clockall_snvs_hp()          imxrt_periphclk_configure(CCM_CCGR_SNVS_HP, CCM_CG_ALL)
#define imxrt_clockall_snvs_lp()          imxrt_periphclk_configure(CCM_CCGR_SNVS_LP, CCM_CG_ALL)
#define imxrt_clockall_spdif()            imxrt_periphclk_configure(CCM_CCGR_SPDIF, CCM_CG_ALL)
#define imxrt_clockall_timer1()           imxrt_periphclk_configure(CCM_CCGR_TIMER1, CCM_CG_ALL)
#define imxrt_clockall_timer2()           imxrt_periphclk_configure(CCM_CCGR_TIMER2, CCM_CG_ALL)
#define imxrt_clockall_timer3()           imxrt_periphclk_configure(CCM_CCGR_TIMER3, CCM_CG_ALL)
#define imxrt_clockall_timer4()           imxrt_periphclk_configure(CCM_CCGR_TIMER4, CCM_CG_ALL)
#define imxrt_clockall_trace()            imxrt_periphclk_configure(CCM_CCGR_TRACE, CCM_CG_ALL)
#define imxrt_clockall_trng()             imxrt_periphclk_configure(CCM_CCGR_TRNG, CCM_CG_ALL)
#define imxrt_clockall_tsc_dig()          imxrt_periphclk_configure(CCM_CCGR_TSC_DIG, CCM_CG_ALL)
#define imxrt_clockall_usboh3()           imxrt_periphclk_configure(CCM_CCGR_USBOH3, CCM_CG_ALL)
#define imxrt_clockall_usdhc1()           imxrt_periphclk_configure(CCM_CCGR_USDHC1, CCM_CG_ALL)
#define imxrt_clockall_usdhc2()           imxrt_periphclk_configure(CCM_CCGR_USDHC2, CCM_CG_ALL)
#define imxrt_clockall_wdog1()            imxrt_periphclk_configure(CCM_CCGR_WDOG1, CCM_CG_ALL)
#define imxrt_clockall_wdog2()            imxrt_periphclk_configure(CCM_CCGR_WDOG2, CCM_CG_ALL)
#define imxrt_clockall_wdog3()            imxrt_periphclk_configure(CCM_CCGR_WDOG3, CCM_CG_ALL)
#define imxrt_clockall_xbar1()            imxrt_periphclk_configure(CCM_CCGR_XBAR1, CCM_CG_ALL)
#define imxrt_clockall_xbar2()            imxrt_periphclk_configure(CCM_CCGR_XBAR2, CCM_CG_ALL)
#define imxrt_clockall_xbar3()            imxrt_periphclk_configure(CCM_CCGR_XBAR3, CCM_CG_ALL)

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Name: imxrt_periphclk_configure
 *
 * Description:
 *   Configure a peripheral clock by modifying the appropriate field in the
 *   appropriate CCRGR register.
 *
 * Input Parameters:
 *   regaddr - The CCMD CCGR register to be modified
 *   index   - The index of the field to be modified
 *   value   - The new value of the field
 *
 * Returned Value:
 *  None
 *
 ************************************************************************************/

void imxrt_periphclk_configure(uintptr_t regaddr, unsigned int index,
                               unsigned int value);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ARCH_ARM_SRC_IMXRT_IMXRT_PERIPHCLKS_H */
