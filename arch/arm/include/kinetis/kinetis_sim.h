/************************************************************************************
 * arch/arm/include/kinetis/kinetis_sim.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
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

#ifndef __ARCH_ARM_INCLUDE_KINETIS_KINETIS_SIM_H
#define __ARCH_ARM_INCLUDE_KINETIS_KINETIS_SIM_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Note: It is envisioned that in the long term as a chip is added. The author of
 * the new chip definitions will either find the exact configuration in an existing
 * chip define and add the new chip to it Or add the SIM feature configuration
 * #defines to the chip ifdef list below. In either case the author should mark
 * it as "Verified to Document Number:" taken from the reference manual.
 *
 * To maintain backward compatibility to the version of NuttX prior to
 * 2/16/2017, the catch all KINETIS_SIM_VERSION_UKN configuration is assigned
 * to all the chips that did not have any conditional compilation based on
 * KINETIS_K64 or KINETIS_K66. This is  a "No worse" than the original code solution.
 * N.B. Each original chip "if"definitions have been left intact so that the
 * complete legacy definitions prior to 2/16/2017 may be filled in completely when
 * vetted.
 */

/* SIM Register Configuration
 *
 * KINETIS_SIM_HAS_SOPT1                   -  SoC has SOPT1 Register
 * KINETIS_SIM_HAS_SOPT1_OSC32KOUT         -  SoC has SOPT1[OSC32KOUT]
 * KINETIS_SIM_HAS_SOPT1_OSC32KSEL         -  SoC has SOPT1[OSC32KSEL]
 * KINETIS_SIM_HAS_SOPT1_OSC32KSEL_BITS    -  SoC has n bits SOPT1[OSC32KSEL]
 * KINETIS_SIM_HAS_SOPT1_RAMSIZE           -  SoC has SOPT1[RAMSIZE]
 * KINETIS_SIM_HAS_SOPT1_USBREGEN          -  SoC has SOPT1[USBREGEN]
 * KINETIS_SIM_HAS_SOPT1_USBSSTBY          -  SoC has SOPT1[USBSSTBY]
 * KINETIS_SIM_HAS_SOPT1_USBVSTBY          -  SoC has SOPT1[USBVSTBY]
 * KINETIS_SIM_HAS_SOPT1CFG                -  SoC has SOPT1CFG Register
 * KINETIS_SIM_HAS_SOPT1CFG_URWE           -  SoC has SOPT1CFG[URWE]
 * KINETIS_SIM_HAS_SOPT1CFG_USSWE          -  SoC has SOPT1CFG[USSWE]
 * KINETIS_SIM_HAS_SOPT1CFG_UVSWE          -  SoC has SOPT1CFG[UVSWE]
 * KINETIS_SIM_HAS_USBPHYCTL               -  SoC has USBPHYCTL Register
 * KINETIS_SIM_HAS_USBPHYCTL_USB3VOUTTRG   -  SoC has USBPHYCTL[USB3VOUTTRG]
 * KINETIS_SIM_HAS_USBPHYCTL_USBDISILIM    -  SoC has USBPHYCTL[USBDISILIM]
 * KINETIS_SIM_HAS_USBPHYCTL_USBVREGPD     -  SoC has USBPHYCTL[USBVREGPD]
 * KINETIS_SIM_HAS_USBPHYCTL_USBVREGSEL    -  SoC has USBPHYCTL[USBVREGSEL]
 * KINETIS_SIM_HAS_SOPT2                   -  SoC has SOPT2 Register
 * KINETIS_SIM_HAS_SOPT2_CMTUARTPAD        -  SoC has SOPT2[CMTUARTPAD]
 * KINETIS_SIM_HAS_SOPT2_FBSL              -  SoC has SOPT2[FBSL]
 * KINETIS_SIM_HAS_SOPT2_FLEXIOSRC         -  SoC has SOPT2[FLEXIOSRC]
 * KINETIS_SIM_HAS_SOPT2_LPUARTSRC         -  SoC has SOPT2[LPUARTSRC]
 * KINETIS_SIM_HAS_SOPT2_PLLFLLSEL         -  SoC has SOPT2[PLLFLLSEL]
 * KINETIS_SIM_HAS_SOPT2_PLLFLLSEL_BITS    -  SoC has n bits SOPT2[PLLFLLSEL]
 * KINETIS_SIM_HAS_SOPT2_PTD7PAD           -  SoC has SOPT2[PTD7PAD]
 * KINETIS_SIM_HAS_SOPT2_RMIISRC           -  SoC has SOPT2[RMIISRC]
 * KINETIS_SIM_HAS_SOPT2_RTCCLKOUTSEL      -  SoC has SOPT2[RTCCLKOUTSEL]
 * KINETIS_SIM_HAS_SOPT2_CLKOUTSEL         -  SoC has SOPT2[CLKOUTSEL]
 * KINETIS_SIM_HAS_SOPT2_SDHCSRC           -  SoC has SOPT2[SDHCSRC]
 * KINETIS_SIM_HAS_SOPT2_NFCSRC            -  SoC has SOPT2[NFCSRC]
 * KINETIS_SIM_HAS_SOPT2_I2SSRC            -  SoC has SOPT2[I2SSRC]
 * KINETIS_SIM_HAS_SOPT2_TIMESRC           -  SoC has SOPT2[TIMESRC]
 * KINETIS_SIM_HAS_SOPT2_TPMSRC            -  SoC has SOPT2[TPMSRC]
 * KINETIS_SIM_HAS_SOPT2_USBFSRC           -  SoC has SOPT2[USBFSRC]
 * KINETIS_SIM_HAS_SOPT2_TRACECLKSEL       -  SoC has SOPT2[TRACECLKSEL]
 * KINETIS_SIM_HAS_SOPT2_USBREGEN          -  SoC has SOPT2[USBREGEN]
 * KINETIS_SIM_HAS_SOPT2_USBSLSRC          -  SoC has SOPT2[USBSLSRC]
 * KINETIS_SIM_HAS_SOPT2_USBHSRC           -  SoC has SOPT2[USBHSRC]
 * KINETIS_SIM_HAS_SOPT2_USBSRC            -  SoC has SOPT2[USBSRC]
 * KINETIS_SIM_HAS_SOPT2_MCGCLKSEL         -  SoC has SOPT2[MCGCLKSEL]
 * KINETIS_SIM_HAS_SOPT4                   -  SoC has SOPT4 Register
 * KINETIS_SIM_HAS_SOPT4_FTM0FLT0          -  SoC has SOPT4[FTM0FLT0]
 * KINETIS_SIM_HAS_SOPT4_FTM0FLT1          -  SoC has SOPT4[FTM0FLT1]
 * KINETIS_SIM_HAS_SOPT4_FTM0FLT2          -  SoC has SOPT4[FTM0FLT2]
 * KINETIS_SIM_HAS_SOPT4_FTM0FLT3          -  SoC has SOPT4[FTM0FLT3]
 * KINETIS_SIM_HAS_SOPT4_FTM0TRG0SRC       -  SoC has SOPT4[FTM0TRG0SRC]
 * KINETIS_SIM_HAS_SOPT4_FTM0TRG1SRC       -  SoC has SOPT4[FTM0TRG1SRC]
 * KINETIS_SIM_HAS_SOPT4_FTM1CH0SRC        -  SoC has SOPT4[FTM1CH0SRC] 1, 3 if SOF
 * KINETIS_SIM_HAS_SOPT4_FTM1FLT0          -  SoC has SOPT4[FTM1FLT0]
 * KINETIS_SIM_HAS_SOPT4_FTM1FLT1          -  SoC has SOPT4[FTM1FLT1]
 * KINETIS_SIM_HAS_SOPT4_FTM1FLT2          -  SoC has SOPT4[FTM1FLT2]
 * KINETIS_SIM_HAS_SOPT4_FTM1FLT3          -  SoC has SOPT4[FTM1FLT3]
 * KINETIS_SIM_HAS_SOPT4_FTM2CH0SRC        -  SoC has SOPT4[FTM2CH0SRC]
 * KINETIS_SIM_HAS_SOPT4_FTM2CH1SRC        -  SoC has SOPT4[FTM2CH1SRC]
 * KINETIS_SIM_HAS_SOPT4_FTM2FLT0          -  SoC has SOPT4[FTM2FLT0]
 * KINETIS_SIM_HAS_SOPT4_FTM2FLT1          -  SoC has SOPT4[FTM2FLT1]
 * KINETIS_SIM_HAS_SOPT4_FTM2FLT2          -  SoC has SOPT4[FTM2FLT2]
 * KINETIS_SIM_HAS_SOPT4_FTM2FLT3          -  SoC has SOPT4[FTM2FLT3]
 * KINETIS_SIM_HAS_SOPT4_FTM3CH0SRC        -  SoC has SOPT4[FTM3CH0SRC]
 * KINETIS_SIM_HAS_SOPT4_FTM3FLT0          -  SoC has SOPT4[FTM3FLT0]
 * KINETIS_SIM_HAS_SOPT4_FTM3FLT1          -  SoC has SOPT4[FTM3FLT1]
 * KINETIS_SIM_HAS_SOPT4_FTM3FLT2          -  SoC has SOPT4[FTM3FLT2]
 * KINETIS_SIM_HAS_SOPT4_FTM3FLT3          -  SoC has SOPT4[FTM3FLT3]
 * KINETIS_SIM_HAS_SOPT4_FTM3TRG0SRC       -  SoC has SOPT4[FTM3TRG0SRC]
 * KINETIS_SIM_HAS_SOPT4_FTM3TRG1SRC       -  SoC has SOPT4[FTM3TRG1SRC]
 * KINETIS_SIM_HAS_SOPT4_TPM0CLKSEL        -  SoC has SOPT4[TPM0CLKSEL]
 * KINETIS_SIM_HAS_SOPT4_TPM1CH0SRC        -  SoC has SOPT4[TPM1CH0SRC]
 * KINETIS_SIM_HAS_SOPT4_TPM1CLKSEL        -  SoC has SOPT4[TPM1CLKSEL]
 * KINETIS_SIM_HAS_SOPT4_TPM2CH0SRC        -  SoC has SOPT4[TPM2CH0SRC]
 * KINETIS_SIM_HAS_SOPT4_TPM2CLKSEL        -  SoC has SOPT4[TPM2CLKSEL]
 * KINETIS_SIM_HAS_SOPT5                   -  SoC has SOPT5 Register
 * KINETIS_SIM_HAS_SOPT5_LPUART0RXSRC      -  SoC has SOPT5[LPUART0RXSRC]
 * KINETIS_SIM_HAS_SOPT5_LPUART0TXSRC      -  SoC has SOPT5[LPUART0TXSRC]
 * KINETIS_SIM_HAS_SOPT6                   -  SoC has SOPT6 Register
 * KINETIS_SIM_HAS_SOPT6_MCC               -  SoC has SOPT6[MCC]
 * KINETIS_SIM_HAS_SOPT6_PCR               -  SoC has SOPT6[PCR]
 * KINETIS_SIM_HAS_SOPT6_RSTFLTSEL         -  SoC has SOPT6[RSTFLTSEL]
 * KINETIS_SIM_HAS_SOPT6_RSTFLTEN          -  SoC has SOPT6[RSTFLTEN]
 * KINETIS_SIM_HAS_SOPT7                   -  SoC has SOPT7 Register
 * KINETIS_SIM_HAS_SOPT7_ADC0ALTTRGSEL     -  SoC has SOPT7[ADC0ALTTRGSEL]
 * KINETIS_SIM_HAS_SOPT7_ADC1ALTTRGSEL     -  SoC has SOPT7[ADC1ALTTRGSEL]
 * KINETIS_SIM_HAS_SOPT7_ADC0PRETRGSEL     -  SoC has SOPT7[ADC0PRETRGSEL]
 * KINETIS_SIM_HAS_SOPT7_ADC1PRETRGSEL     -  SoC has SOPT7[ADC1PRETRGSEL]
 * KINETIS_SIM_HAS_SOPT7_ADC2PRETRGSEL     -  SoC has SOPT7[ADC2PRETRGSEL]
 * KINETIS_SIM_HAS_SOPT7_ADC3PRETRGSEL     -  SoC has SOPT7[ADC3PRETRGSEL]
 * KINETIS_SIM_HAS_SOPT7_ADC0TRGSEL        -  SoC has n SOPT7[ADC0TRGSEL]
 * KINETIS_SIM_HAS_SOPT7_ADC1TRGSEL        -  SoC has n SOPT7[ADC1TRGSEL]
 * KINETIS_SIM_HAS_SOPT7_ADC2TRGSEL        -  SoC has n SOPT7[ADC2TRGSEL]
 * KINETIS_SIM_HAS_SOPT7_ADC3TRGSEL        -  SoC has n SOPT7[ADC3TRGSEL]
 * KINETIS_SIM_SOPT7_ADC0ALTTRGEN          -  SoC has ADC0 alternate trigger enable
 * KINETIS_SIM_SOPT7_ADC1ALTTRGEN          -  SoC has ADC1 alternate trigger enable
 * KINETIS_SIM_SOPT7_ADC2ALTTRGEN          -  SoC has ADC2 alternate trigger enable
 * KINETIS_SIM_SOPT7_ADC3ALTTRGEN          -  SoC has ADC3 alternate trigger enable
 * KINETIS_SIM_HAS_SOPT8                   -  SoC has SOPT8 Register
 * KINETIS_SIM_HAS_SOPT8_FTM0SYNCBIT       -  SoC has SOPT8[FTM0SYNCBIT]
 * KINETIS_SIM_HAS_SOPT8_FTM1SYNCBIT       -  SoC has SOPT8[FTM1SYNCBIT]
 * KINETIS_SIM_HAS_SOPT8_FTM2SYNCBIT       -  SoC has SOPT8[FTM2SYNCBIT]
 * KINETIS_SIM_HAS_SOPT8_FTM3SYNCBIT       -  SoC has SOPT8[FTM3SYNCBIT]
 * KINETIS_SIM_HAS_SOPT8_FTM0OCH0SRC       -  SoC has SOPT8[FTM0OCH0SRC]
 * KINETIS_SIM_HAS_SOPT8_FTM0OCH1SRC       -  SoC has SOPT8[FTM0OCH1SRC]
 * KINETIS_SIM_HAS_SOPT8_FTM0OCH2SRC       -  SoC has SOPT8[FTM0OCH2SRC]
 * KINETIS_SIM_HAS_SOPT8_FTM0OCH3SRC       -  SoC has SOPT8[FTM0OCH3SRC]
 * KINETIS_SIM_HAS_SOPT8_FTM0OCH4SRC       -  SoC has SOPT8[FTM0OCH4SRC]
 * KINETIS_SIM_HAS_SOPT8_FTM0OCH5SRC       -  SoC has SOPT8[FTM0OCH5SRC]
 * KINETIS_SIM_HAS_SOPT8_FTM0OCH6SRC       -  SoC has SOPT8[FTM0OCH6SRC]
 * KINETIS_SIM_HAS_SOPT8_FTM0OCH7SRC       -  SoC has SOPT8[FTM0OCH7SRC]
 * KINETIS_SIM_HAS_SOPT8_FTM3OCH0SRC       -  SoC has SOPT8[FTM3OCH0SRC]
 * KINETIS_SIM_HAS_SOPT8_FTM3OCH1SRC       -  SoC has SOPT8[FTM3OCH1SRC]
 * KINETIS_SIM_HAS_SOPT8_FTM3OCH2SRC       -  SoC has SOPT8[FTM3OCH2SRC]
 * KINETIS_SIM_HAS_SOPT8_FTM3OCH3SRC       -  SoC has SOPT8[FTM3OCH3SRC]
 * KINETIS_SIM_HAS_SOPT8_FTM3OCH4SRC       -  SoC has SOPT8[FTM3OCH4SRC]
 * KINETIS_SIM_HAS_SOPT8_FTM3OCH5SRC       -  SoC has SOPT8[FTM3OCH5SRC]
 * KINETIS_SIM_HAS_SOPT8_FTM3OCH6SRC       -  SoC has SOPT8[FTM3OCH6SRC]
 * KINETIS_SIM_HAS_SOPT8_FTM3OCH7SRC       -  SoC has SOPT8[FTM3OCH7SRC]
 * KINETIS_SIM_HAS_SOPT9                   -  SoC has SOPT9 Register
 * KINETIS_SIM_HAS_SOPT9_TPM1CH0SRC        -  SoC has SOPT9[TPM1CH0SRC]
 * KINETIS_SIM_HAS_SOPT9_TPM2CH0SRC        -  SoC has SOPT9[TPM2CH0SRC]
 * KINETIS_SIM_HAS_SOPT9_TPM1CLKSEL        -  SoC has SOPT9[TPM1CLKSEL]
 * KINETIS_SIM_HAS_SOPT9_TPM2CLKSEL        -  SoC has SOPT9[TPM2CLKSEL]
 * KINETIS_SIM_HAS_SDID                    -  SoC has SDID Register
 * KINETIS_SIM_HAS_SDID_DIEID              -  SoC has SDID[DIEID]
 * KINETIS_SIM_HAS_SDID_FAMID              -  SoC has SDID[FAMID]
 * KINETIS_SIM_HAS_SDID_FAMILYID           -  SoC has SDID[FAMILYID]
 * KINETIS_SIM_HAS_SDID_SERIESID           -  SoC has SDID[SERIESID]
 * KINETIS_SIM_HAS_SDID_SRAMSIZE           -  SoC has SDID[SRAMSIZE]
 * KINETIS_SIM_HAS_SDID_SUBFAMID           -  SoC has SDID[SUBFAMID]
 * KINETIS_SIM_HAS_SCGC1                   -  SoC has _SCGC1 Register
 * KINETIS_SIM_HAS_SCGC1_UART5             -  SoC has SCGC1[UART5]
 * KINETIS_SIM_HAS_SCGC1_UART4             -  SoC has SCGC1[UART4]
 * KINETIS_SIM_HAS_SCGC1_I2C3              -  SoC has SCGC1[I2C3]
 * KINETIS_SIM_HAS_SCGC1_I2C2              -  SoC has SCGC1[I2C2]
 * KINETIS_SIM_HAS_SCGC1_OSC1              -  SoC has SCGC1[OSC1]
 * KINETIS_SIM_HAS_SCGC2                   -  SoC has SCGC2 Register
 * KINETIS_SIM_HAS_SCGC2_ENET              -  SoC has SCGC2[ENET]
 * KINETIS_SIM_HAS_SCGC2_LPUART0           -  SoC has SCGC2[LPUART0]
 * KINETIS_SIM_HAS_SCGC2_TPM1              -  SoC has SCGC2[TPM1]
 * KINETIS_SIM_HAS_SCGC2_TPM2              -  SoC has SCGC2[TPM2]
 * KINETIS_SIM_HAS_SCGC3                   -  SoC has SCGC3 Register
 * KINETIS_SIM_HAS_SCGC3                   -  SoC has SCGC3 Register
 * KINETIS_SIM_HAS_SCGC3_RNGA              -  SoC has SCGC3[RNGA]
 * KINETIS_SIM_HAS_SCGC3_USBHS             -  SoC has SCGC3[USBHS]
 * KINETIS_SIM_HAS_SCGC3_USBHSPHY          -  SoC has SCGC3[USBHSPHY]
 * KINETIS_SIM_HAS_SCGC3_USBHSDCD          -  SoC has SCGC3[USBHSDCD]
 * KINETIS_SIM_HAS_SCGC3_FLEXCAN1          -  SoC has SCGC3[FLEXCAN1]
 * KINETIS_SIM_HAS_SCGC3_NFC               -  SoC has SCGC3[NFC]
 * KINETIS_SIM_HAS_SCGC3_SPI2              -  SoC has SCGC3[SPI2]
 * KINETIS_SIM_HAS_SCGC3_SAI1              -  SoC has SCGC3[SAI1]
 * KINETIS_SIM_HAS_SCGC3_SDHC              -  SoC has SCGC3[SDHC]
 * KINETIS_SIM_HAS_SCGC3_FTM2              -  SoC has SCGC3[FTM2]
 * KINETIS_SIM_HAS_SCGC3_FTM3              -  SoC has SCGC3[FTM3]
 * KINETIS_SIM_HAS_SCGC3_ADC1              -  SoC has SCGC3[ADC1]
 * KINETIS_SIM_HAS_SCGC3_ADC3              -  SoC has SCGC3[ADC3]
 * KINETIS_SIM_HAS_SCGC3_SLCD              -  SoC has SCGC3[SLCD]
 * KINETIS_SIM_HAS_SCGC4                   -  SoC has SCGC4 Register
 * KINETIS_SIM_HAS_SCGC4_LLWU              -  SoC has SCGC4[LLWU] clock gate
 * KINETIS_SIM_HAS_SCGC4_UART0             -  SoC has SCGC4[UART0]
 * KINETIS_SIM_HAS_SCGC4_UART1             -  SoC has SCGC4[UART1]
 * KINETIS_SIM_HAS_SCGC4_UART2             -  SoC has SCGC4[UART2]
 * KINETIS_SIM_HAS_SCGC4_UART3             -  SoC has SCGC4[UART3]
 * KINETIS_SIM_HAS_SCGC5                   -  SoC has _SCGC5 Register
 * KINETIS_SIM_HAS_SCGC5_REGFILE           -  SoC has SCGC5[REGFILE]
 * KINETIS_SIM_HAS_SCGC5_TSI               -  SoC has SCGC5[TSI]
 * KINETIS_SIM_HAS_SCGC5_PORTF             -  SoC has SCGC5[PORTf]
 * KINETIS_SIM_HAS_SCGC6                   -  SoC has SCGC6 Register
 * KINETIS_SIM_HAS_SCGC6_FTFL              -  SoC has SCGC6[FTFL]
 * KINETIS_SIM_HAS_SCGC6_DMAMUX1           -  SoC has SCGC6[DEMUX1]
 * KINETIS_SIM_HAS_SCGC6_USBHS             -  SoC has SCGC6[USBHS]
 * KINETIS_SIM_HAS_SCGC6_RNGA              -  SoC has SCGC6[RNGA]
 * KINETIS_SIM_HAS_SCGC6_FTM2              -  SoC has SCGC6[FTM2]
 * KINETIS_SIM_HAS_SCGC6_ADC2              -  SoC has SCGC6[ADC2]
 * KINETIS_SIM_HAS_SCGC6_DAC0              -  SoC has SCGC6[DAC0]
 * KINETIS_SIM_HAS_SCGC7                   -  SoC has SCGC7 Register
 * KINETIS_SIM_HAS_SCGC7_FLEXBUS           -  SoC has SCGC7[FLEXBUS]
 * KINETIS_SIM_HAS_SCGC7_DMA               -  SoC has SCGC7[DMS]
 * KINETIS_SIM_HAS_SCGC7_MPU               -  SoC has SCGC7[MPU]
 * KINETIS_SIM_HAS_SCGC7_SDRAMC            -  SoC has SCGC7[SDRAMC]
 * KINETIS_SIM_HAS_CLKDIV1                 -  SoC has CLKDIV1 Register
 * KINETIS_SIM_HAS_CLKDIV1_OUTDIV2         -  SoC has CLKDIV1[OUTDIV2]
 * KINETIS_SIM_HAS_CLKDIV1_OUTDIV3         -  SoC has CLKDIV1[OUTDIV3]
 * KINETIS_SIM_HAS_CLKDIV1_OUTDIV4         -  SoC has CLKDIV1[OUTDIV4]
 * KINETIS_SIM_HAS_CLKDIV1_OUTDIV5         -  SoC has CLKDIV1[OUTDIV5]
 * KINETIS_SIM_HAS_CLKDIV2                 -  SoC has CLKDIV2 Register
 * KINETIS_SIM_HAS_CLKDIV2_USBDIV          -  SoC has CLKDIV2[USBDIV]
 * KINETIS_SIM_HAS_CLKDIV2_USBFRAC         -  SoC has CLKDIV2[USBFRAC]
 * KINETIS_SIM_HAS_CLKDIV2_I2SDIV          -  SoC has CLKDIV2[I2SDIV]
 * KINETIS_SIM_HAS_CLKDIV2_I2SFRAC         -  SoC has CLKDIV2[I2SFRAC]
 * KINETIS_SIM_HAS_CLKDIV2_USBHSDIV        -  SoC has CLKDIV2[USBHSDIV]
 * KINETIS_SIM_HAS_CLKDIV2_USBHSFRAC       -  SoC has CLKDIV2[USBHSFRAC]
 * KINETIS_SIM_HAS_FCFG1                   -  SoC has FCFG1 Register
 * KINETIS_SIM_HAS_FCFG1_DEPART            -  SoC has FCFG1[DEPART]
 * KINETIS_SIM_HAS_FCFG1_EESIZE            -  SoC has FCFG1[EESIZE]
 * KINETIS_SIM_HAS_FCFG1_FLASHDIS          -  SoC has FCFG1[FLASHDIS]
 * KINETIS_SIM_HAS_FCFG1_FLASHDOZE         -  SoC has FCFG1[FLASHDOZE]
 * KINETIS_SIM_HAS_FCFG1_FTFDIS            -  SoC has FCFG1[FTFDIS]
 * KINETIS_SIM_HAS_FCFG1_NVMSIZE           -  SoC has FCFG1[NVMSIZE]
 * KINETIS_SIM_HAS_FCFG2                   -  SoC has FCFG2 Register
 * KINETIS_SIM_HAS_FCFG2_MAXADDR0          -  SoC has n bit of FCFG2[MAXADDR0]
 * KINETIS_SIM_HAS_FCFG2_MAXADDR1          -  SoC has n bit of FCFG2[MAXADDR1]
 * KINETIS_SIM_HAS_FCFG2_PFLSH             -  SoC has FCFG2[PFLSH]
 * KINETIS_SIM_HAS_FCFG2_SWAPPFLSH         -  SoC has FCFG2[SWAPPFLSH]
 * KINETIS_SIM_HAS_UIDH                    -  SoC has UIDH Register
 * KINETIS_SIM_HAS_UIDMH                   -  SoC has UIDMH Register
 * KINETIS_SIM_HAS_UIDML                   -  SoC has UIDML Register
 * KINETIS_SIM_HAS_UIDL                    -  SoC has UIDL Register
 * KINETIS_SIM_HAS_CLKDIV3                 -  SoC has CLKDIV3 Register
 * KINETIS_SIM_HAS_CLKDIV3_PLLFLLDIV       -  SoC has CLKDIV3[PLLFLLDIV]
 * KINETIS_SIM_HAS_CLKDIV3_PLLFLLFRAC      -  SoC has CLKDIV3[PLLFLLFRAC]
 * KINETIS_SIM_HAS_CLKDIV4                 -  SoC has CLKDIV4 Register
 * KINETIS_SIM_HAS_CLKDIV4_TRACEDIV        -  SoC has CLKDIV4[TRACEDIV]
 * KINETIS_SIM_HAS_CLKDIV4_TRACEFRAC       -  SoC has CLKDIV4[TRACEFRAC]
 * KINETIS_SIM_HAS_CLKDIV4_NFCEDIV         -  SoC has CLKDIV4[NFCDIV]
 * KINETIS_SIM_HAS_CLKDIV4_NFCFRAC         -  SoC has CLKDIV4[NFCFRAC]
 * KINETIS_SIM_HAS_MCR                     -  SoC has MCR Register
 */

/* Describe the version of the SIM
 *
 * These defines are not related to any NXP reference but are merely
 * a way to label the versions we are using
 */

#define KINETIS_SIM_VERSION_UKN   -1  /* What was in nuttx prior to 2/16/2017 */
#define KINETIS_SIM_VERSION_01     1  /* Verified Document Number: K60P144M150SF3RM Rev. 3, November 2014 */
#define KINETIS_SIM_VERSION_04     4  /* Verified to Document Number: K64P144M120SF5RM Rev. 2, January 2014 */
#define KINETIS_SIM_VERSION_06     6  /* Verified to Document Number: K66P144M180SF5RMV2 Rev. 2, May 2015 */

/* MK20DX/DN---VLH5
 *
 *  ------------- ------ --- ------- ------ ------- ------ ----- ----
 *  PART NUMBER   CPU    PIN PACKAGE TOTAL  PROGRAM EEPROM SRAM  GPIO
 *                FREQ   CNT         FLASH  FLASH
 *  ------------- ------ --- ------- ------ ------- ------ ----- ----
 *  MK20DN32VLH5  50 MHz 64  LQFP     32 KB 32 KB   —       8 KB 40
 *  MK20DX32VLH5  50 MHz 64  LQFP     64 KB 32 KB   2 KB    8 KB 40
 *  MK20DN64VLH5  50 MHz 64  LQFP     64 KB 64 KB   —      16 KB 40
 *  MK20DX64VLH5  50 MHz 64  LQFP     96 KB 64 KB   2 KB   16 KB 40
 *  MK20DN128VLH5 50 MHz 64  LQFP    128 KB 128 KB  —      16 KB 40
 *  MK20DX128VLH5 50 MHz 64  LQFP    160 KB 128 KB  2 KB   16 KB 40
 */

#if defined(CONFIG_ARCH_CHIP_MK20DN32VLH5) || \
    defined(CONFIG_ARCH_CHIP_MK20DX32VLH5) || \
    defined(CONFIG_ARCH_CHIP_MK20DN64VLH5) || \
    defined(CONFIG_ARCH_CHIP_MK20DX64VLH5) || \
    defined(CONFIG_ARCH_CHIP_MK20DN128VLH5) || \
    defined(CONFIG_ARCH_CHIP_MK20DX128VLH5)

#  define KINETIS_SIM_VERSION KINETIS_SIM_VERSION_UKN

/* MK20DX---VLH7
 *
 *  ------------- ------ --- ------- ------ ------- ------ ----- ----
 *  PART NUMBER   CPU    PIN PACKAGE TOTAL  PROGRAM EEPROM SRAM  GPIO
 *                FREQ   CNT         FLASH  FLASH
 *  ------------- ------ --- ------- ------ ------- ------ ----- ----
 *  MK20DX64VLH7  72 MHz 64  LQFP     96 KB  64 KB  2 KB   16 KB 40
 *  MK20DX128VLH7 72 MHz 64  LQFP    160 KB 128 KB  2 KB   32 KB 40
 *  MK20DX256VLH7 72 MHz 64  LQFP    288 KB 256 KB  2 KB   64 KB 40
 *  ------------- ------ --- ------- ------ ------- ------ ----- ----
 */

#elif defined(CONFIG_ARCH_CHIP_MK20DX64VLH7) || defined(CONFIG_ARCH_CHIP_MK20DX128VLH7) || \
      defined(CONFIG_ARCH_CHIP_MK20DX256VLH7)

#  define KINETIS_SIM_VERSION KINETIS_SIM_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK40X64VFX50) || defined(CONFIG_ARCH_CHIP_MK40X64VLH50) || \
      defined(CONFIG_ARCH_CHIP_MK40X64VLK50) || defined(CONFIG_ARCH_CHIP_MK40X64VMB50)

#  define KINETIS_SIM_VERSION KINETIS_SIM_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK40X128VFX50) || defined(CONFIG_ARCH_CHIP_MK40X128VLH50) || \
      defined(CONFIG_ARCH_CHIP_MK40X128VLK50) || defined(CONFIG_ARCH_CHIP_MK40X128VMB50) || \
      defined(CONFIG_ARCH_CHIP_MK40X128VLL50) || defined(CONFIG_ARCH_CHIP_MK40X128VML50) || \
      defined(CONFIG_ARCH_CHIP_MK40X128VFX72) || defined(CONFIG_ARCH_CHIP_MK40X128VLH72) || \
      defined(CONFIG_ARCH_CHIP_MK40X128VLK72) || defined(CONFIG_ARCH_CHIP_MK40X128VMB72) || \
      defined(CONFIG_ARCH_CHIP_MK40X128VLL72) || defined(CONFIG_ARCH_CHIP_MK40X128VML72)

#  define KINETIS_SIM_VERSION KINETIS_SIM_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK40X256VLK72) || defined(CONFIG_ARCH_CHIP_MK40X256VMB72) || \
      defined(CONFIG_ARCH_CHIP_MK40X256VLL72) || defined(CONFIG_ARCH_CHIP_MK40X256VML72)

#  define KINETIS_SIM_VERSION KINETIS_SIM_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK40X128VLQ100) || defined(CONFIG_ARCH_CHIP_MK40X128VMD100)

#  define KINETIS_SIM_VERSION KINETIS_SIM_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK40X256VLQ100) || defined(CONFIG_ARCH_CHIP_MK40X256VMD100)

#  define KINETIS_SIM_VERSION KINETIS_SIM_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK40N512VLK100) || defined(CONFIG_ARCH_CHIP_MK40N512VMB100) || \
      defined(CONFIG_ARCH_CHIP_MK40N512VLL100) || defined(CONFIG_ARCH_CHIP_MK40N512VML100) || \
      defined(CONFIG_ARCH_CHIP_MK40N512VLQ100) || defined(CONFIG_ARCH_CHIP_MK40N512VMD100)

#  define KINETIS_SIM_VERSION KINETIS_SIM_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK60N256VLL100)

#  define KINETIS_SIM_VERSION KINETIS_SIM_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK60X256VLL100)

#  define KINETIS_SIM_VERSION KINETIS_SIM_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK60N512VLL100)

#  define KINETIS_SIM_VERSION KINETIS_SIM_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK60N256VML100)

#  define KINETIS_SIM_VERSION KINETIS_SIM_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK60X256VML100)

#  define KINETIS_SIM_VERSION KINETIS_SIM_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK60N512VML100)

#  define KINETIS_SIM_VERSION KINETIS_SIM_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK60N256VLQ100)

#  define KINETIS_SIM_VERSION KINETIS_SIM_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK60X256VLQ100)

#  define KINETIS_SIM_VERSION KINETIS_SIM_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK60N512VLQ100)

#  define KINETIS_SIM_VERSION KINETIS_SIM_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK60N256VMD100)

#  define KINETIS_SIM_VERSION KINETIS_SIM_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK60X256VMD100)

#  define KINETIS_SIM_VERSION KINETIS_SIM_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK60N512VMD100)

#  define KINETIS_SIM_VERSION KINETIS_SIM_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK60FN1M0VLQ12)

/* Verified to Document Number: K60P144M100SF2V2RM Rev. 2 Jun 2012 */

#  define KINETIS_SIM_VERSION KINETIS_SIM_VERSION_01

/* SIM Register Configuration */

#  define KINETIS_SIM_HAS_SOPT1                       1   /* SoC has SOPT1 Register */
#  undef  KINETIS_SIM_HAS_SOPT1_OSC32KOUT                 /* SoC has SOPT1[OSC32KOUT] */
#  define KINETIS_SIM_HAS_SOPT1_OSC32KSEL             1   /* SoC has SOPT1[OSC32KSEL] */
#  define KINETIS_SIM_HAS_SOPT1_OSC32KSEL_BITS        1   /* SoC has 1 bit SOPT1[OSC32KSEL] */
#  define KINETIS_SIM_HAS_SOPT1_RAMSIZE               1   /* SoC has SOPT1[RAMSIZE] */
#  define KINETIS_SIM_HAS_SOPT1_USBREGEN              1   /* SoC has SOPT1[USBREGEN] */
#  define KINETIS_SIM_HAS_SOPT1_USBSSTBY              1   /* SoC has SOPT1[USBSSTBY] */
#  define KINETIS_SIM_HAS_SOPT1_USBVSTBY              1   /* SoC has SOPT1[USBVSTBY] */
#  define KINETIS_SIM_HAS_SOPT1CFG                        /* SoC has SOPT1CFG Register */
#  define KINETIS_SIM_HAS_SOPT1CFG_URWE               1   /* SoC has SOPT1CFG[URWE] */
#  define KINETIS_SIM_HAS_SOPT1CFG_USSWE              1   /* SoC has SOPT1CFG[USSWE] */
#  define KINETIS_SIM_HAS_SOPT1CFG_UVSWE              1   /* SoC has SOPT1CFG[UVSWE] */
#  undef  KINETIS_SIM_HAS_USBPHYCTL                       /* SoC has USBPHYCTL Register */
#  undef  KINETIS_SIM_HAS_USBPHYCTL_USB3VOUTTRG           /* SoC has USBPHYCTL[USB3VOUTTRG] */
#  undef  KINETIS_SIM_HAS_USBPHYCTL_USBDISILIM            /* SoC has USBPHYCTL[USBDISILIM] */
#  undef  KINETIS_SIM_HAS_USBPHYCTL_USBVREGPD             /* SoC has USBPHYCTL[USBVREGPD] */
#  undef  KINETIS_SIM_HAS_USBPHYCTL_USBVREGSEL            /* SoC has USBPHYCTL[USBVREGSEL] */
#  define KINETIS_SIM_HAS_SOPT2                       1   /* SoC has SOPT2 Register */
#  define KINETIS_SIM_HAS_SOPT2_FBSL                  1   /* SoC has SOPT2[FBSL] */
#  define KINETIS_SIM_HAS_SOPT2_CMTUARTPAD            1   /* SoC has SOPT2[CMTUARTPAD] */
#  define KINETIS_SIM_HAS_SOPT2_FLEXIOSRC             1   /* SoC has SOPT2[FLEXIOSRC] */
#  undef  KINETIS_SIM_HAS_SOPT2_LPUARTSRC                 /* SoC has SOPT2[LPUARTSRC] */
#  define KINETIS_SIM_HAS_SOPT2_PLLFLLSEL             1   /* SoC has SOPT2[PLLFLLSEL] */
#  define KINETIS_SIM_HAS_SOPT2_PLLFLLSEL_BITS        2   /* SoC has 2 bits of SOPT2[PLLFLLSEL] */
#  undef  KINETIS_SIM_HAS_SOPT2_PTD7PAD                   /* SoC has SOPT2[PTD7PAD] */
#  undef  KINETIS_SIM_HAS_SOPT2_RMIISRC                   /* SoC has SOPT2[RMIISRC] */
#  define KINETIS_SIM_HAS_SOPT2_RTCCLKOUTSEL          1   /* SoC has SOPT2[RTCCLKOUTSEL] */
#  define KINETIS_SIM_HAS_SOPT2_CLKOUTSEL             1   /* SoC has SOPT2[CLKOUTSEL] */
#  define KINETIS_SIM_HAS_SOPT2_SDHCSRC               1   /* SoC has SOPT2[SDHCSRC] */
#  define KINETIS_SIM_HAS_SOPT2_NFCSRC                1   /* SoC has SOPT2[NFCSRC] */
#  undef  KINETIS_SIM_HAS_SOPT2_I2SSRC                    /* SoC has SOPT2[I2SSRC] */
#  define KINETIS_SIM_HAS_SOPT2_TIMESRC               1   /* SoC has SOPT2[TIMESRC] */
#  undef  KINETIS_SIM_HAS_SOPT2_TPMSRC                    /* SoC has SOPT2[TPMSRC] */
#  define KINETIS_SIM_HAS_SOPT2_USBFSRC               1   /* SoC has SOPT2[USBFSRC] */
#  define KINETIS_SIM_HAS_SOPT2_TRACECLKSEL           1   /* SoC has SOPT2[TRACECLKSEL] */
#  undef  KINETIS_SIM_HAS_SOPT2_USBREGEN                  /* SoC has SOPT2[USBREGEN] */
#  undef  KINETIS_SIM_HAS_SOPT2_USBSLSRC                  /* SoC has SOPT2[USBSLSRC] */
#  define KINETIS_SIM_HAS_SOPT2_USBHSRC               1   /* SoC has SOPT2[USBHSRC] */
#  define KINETIS_SIM_HAS_SOPT2_USBSRC                1   /* SoC has SOPT2[USBSRC] */
#  undef  KINETIS_SIM_HAS_SOPT2_MCGCLKSEL                 /* SoC has SOPT2[MCGCLKSEL] */
#  define KINETIS_SIM_HAS_SOPT4                       1   /* SoC has SOPT4 Register */
#  define KINETIS_SIM_HAS_SOPT4_FTM0FLT0              1   /* SoC has SOPT4[FTM0FLT0] */
#  define KINETIS_SIM_HAS_SOPT4_FTM0FLT1              1   /* SoC has SOPT4[FTM0FLT1] */
#  define KINETIS_SIM_HAS_SOPT4_FTM0FLT2              1   /* SoC has SOPT4[FTM0FLT2] */
#  define KINETIS_SIM_HAS_SOPT4_FTM0FLT3              1   /* SoC has SOPT4[FTM0FLT3] */
#  define KINETIS_SIM_HAS_SOPT4_FTM0TRG0SRC           1   /* SoC has SOPT4[FTM0TRG0SRC] */
#  define KINETIS_SIM_HAS_SOPT4_FTM0TRG1SRC           1   /* SoC has SOPT4[FTM0TRG1SRC] */
#  define KINETIS_SIM_HAS_SOPT4_FTM1CH0SRC            3   /* SoC has SOPT4[FTM1CH0SRC] 1, 3 if SOF */
#  define KINETIS_SIM_HAS_SOPT4_FTM1FLT0              1   /* SoC has SOPT4[FTM1FLT0] */
#  define KINETIS_SIM_HAS_SOPT4_FTM1FLT1              1   /* SoC has SOPT4[FTM1FLT1] */
#  define KINETIS_SIM_HAS_SOPT4_FTM1FLT2              1   /* SoC has SOPT4[FTM1FLT2] */
#  define KINETIS_SIM_HAS_SOPT4_FTM1FLT3              1   /* SoC has SOPT4[FTM1FLT3] */
#  define KINETIS_SIM_HAS_SOPT4_FTM2CH0SRC            1   /* SoC has SOPT4[FTM2CH0SRC] */
#  undef  KINETIS_SIM_HAS_SOPT4_FTM2CH1SRC                /* SoC has SOPT4[FTM2CH1SRC] */
#  define KINETIS_SIM_HAS_SOPT4_FTM2FLT0              1   /* SoC has SOPT4[FTM2FLT0] */
#  define KINETIS_SIM_HAS_SOPT4_FTM2FLT1              1   /* SoC has SOPT4[FTM2FLT1] */
#  define KINETIS_SIM_HAS_SOPT4_FTM2FLT2              1   /* SoC has SOPT4[FTM2FLT2] */
#  define KINETIS_SIM_HAS_SOPT4_FTM2FLT3              1   /* SoC has SOPT4[FTM2FLT3] */
#  define KINETIS_SIM_HAS_SOPT4_FTM3CH0SRC            1   /* SoC has SOPT4[FTM3CH0SRC] */
#  define KINETIS_SIM_HAS_SOPT4_FTM3FLT0              1   /* SoC has SOPT4[FTM3FLT0] */
#  define KINETIS_SIM_HAS_SOPT4_FTM3FLT1              1   /* SoC has SOPT4[FTM3FLT1] */
#  define KINETIS_SIM_HAS_SOPT4_FTM3FLT2              1   /* SoC has SOPT4[FTM3FLT2] */
#  define KINETIS_SIM_HAS_SOPT4_FTM3FLT3              1   /* SoC has SOPT4[FTM3FLT3] */
#  define KINETIS_SIM_HAS_SOPT4_FTM3TRG0SRC           1   /* SoC has SOPT4[FTM3TRG0SRC] */
#  define KINETIS_SIM_HAS_SOPT4_FTM3TRG1SRC           1   /* SoC has SOPT4[FTM3TRG1SRC] */
#  define KINETIS_SIM_HAS_SOPT4_TPM0CLKSEL            1   /* SoC has SOPT4[TPM0CLKSEL] */
#  define KINETIS_SIM_HAS_SOPT4_TPM1CH0SRC            1   /* SoC has SOPT4[TPM1CH0SRC] */
#  define KINETIS_SIM_HAS_SOPT4_TPM1CLKSEL            1   /* SoC has SOPT4[TPM1CLKSEL] */
#  define KINETIS_SIM_HAS_SOPT4_TPM2CH0SRC            1   /* SoC has SOPT4[TPM2CH0SRC] */
#  define KINETIS_SIM_HAS_SOPT4_TPM2CLKSEL            1   /* SoC has SOPT4[TPM2CLKSEL] */
#  define KINETIS_SIM_HAS_SOPT5                       1   /* SoC has SOPT5 Register */
#  undef  KINETIS_SIM_HAS_SOPT5_LPUART0RXSRC              /* SoC has SOPT5[LPUART0RXSRC] */
#  undef  KINETIS_SIM_HAS_SOPT5_LPUART0TXSRC              /* SoC has SOPT5[LPUART0TXSRC] */
#  define KINETIS_SIM_HAS_SOPT6                       1   /* SoC has SOPT6 Register */
#  define KINETIS_SIM_HAS_SOPT6_MCC                   1   /* SoC has SOPT6[MCC] */
#  define KINETIS_SIM_HAS_SOPT6_PCR                   1   /* SoC has SOPT6[PCR] */
#  undef  KINETIS_SIM_HAS_SOPT6_RSTFLTSEL                 /* SoC has SOPT6[RSTFLTSEL] */
#  undef  KINETIS_SIM_HAS_SOPT6_RSTFLTEN                  /* SoC has SOPT6[RSTFLTEN] */
#  define KINETIS_SIM_HAS_SOPT7                       1   /* SoC has SOPT7 Register */
#  define KINETIS_SIM_HAS_SOPT7_ADC0ALTTRGSEL         1   /* SoC has SOPT7[ADC0ALTTRGSEL] */
#  define KINETIS_SIM_HAS_SOPT7_ADC1ALTTRGSEL         1   /* SoC has SOPT7[ADC1ALTTRGSEL] */
#  define KINETIS_SIM_HAS_SOPT7_ADC0PRETRGSEL         1   /* SoC has SOPT7[ADC0PRETRGSEL] */
#  define KINETIS_SIM_HAS_SOPT7_ADC1PRETRGSEL         1   /* SoC has SOPT7[ADC1PRETRGSEL] */
#  define KINETIS_SIM_HAS_SOPT7_ADC2PRETRGSEL         1   /* SoC has SOPT7[ADC2PRETRGSEL] */
#  define KINETIS_SIM_HAS_SOPT7_ADC3PRETRGSEL         1   /* SoC has SOPT7[ADC3PRETRGSEL] */
#  define KINETIS_SIM_HAS_SOPT7_ADC0TRGSEL            15  /* SoC has 15 SOPT7[ADC0TRGSEL] */
#  define KINETIS_SIM_HAS_SOPT7_ADC1TRGSEL            15  /* SoC has 15 SOPT7[ADC1TRGSEL] */
#  define KINETIS_SIM_HAS_SOPT7_ADC2TRGSEL            15  /* SoC has 15 SOPT7[ADC2TRGSEL] */
#  define KINETIS_SIM_HAS_SOPT7_ADC3TRGSEL            15  /* SoC has 15 SOPT7[ADC3TRGSEL] */
#  define KINETIS_SIM_SOPT7_ADC0ALTTRGEN              1   /* ADC0 alternate trigger enable */
#  define KINETIS_SIM_SOPT7_ADC1ALTTRGEN              1   /* ADC1 alternate trigger enable */
#  define KINETIS_SIM_SOPT7_ADC2ALTTRGEN              1   /* ADC2 alternate trigger enable */
#  define KINETIS_SIM_SOPT7_ADC3ALTTRGEN              1   /* ADC3 alternate trigger enable */
#  undef  KINETIS_SIM_HAS_SOPT8                           /* SoC has SOPT8 Register */
#  undef  KINETIS_SIM_HAS_SOPT8_FTM0SYNCBIT               /* SoC has SOPT8[FTM0SYNCBIT] */
#  undef  KINETIS_SIM_HAS_SOPT8_FTM1SYNCBIT               /* SoC has SOPT8[FTM1SYNCBIT] */
#  undef  KINETIS_SIM_HAS_SOPT8_FTM2SYNCBIT               /* SoC has SOPT8[FTM2SYNCBIT] */
#  undef  KINETIS_SIM_HAS_SOPT8_FTM3SYNCBIT               /* SoC has SOPT8[FTM3SYNCBIT] */
#  undef  KINETIS_SIM_HAS_SOPT8_FTM0OCH0SRC               /* SoC has SOPT8[FTM0OCH0SRC] */
#  undef  KINETIS_SIM_HAS_SOPT8_FTM0OCH1SRC               /* SoC has SOPT8[FTM0OCH1SRC] */
#  undef  KINETIS_SIM_HAS_SOPT8_FTM0OCH2SRC               /* SoC has SOPT8[FTM0OCH2SRC] */
#  undef  KINETIS_SIM_HAS_SOPT8_FTM0OCH3SRC               /* SoC has SOPT8[FTM0OCH3SRC] */
#  undef  KINETIS_SIM_HAS_SOPT8_FTM0OCH4SRC               /* SoC has SOPT8[FTM0OCH4SRC] */
#  undef  KINETIS_SIM_HAS_SOPT8_FTM0OCH5SRC               /* SoC has SOPT8[FTM0OCH5SRC] */
#  undef  KINETIS_SIM_HAS_SOPT8_FTM0OCH6SRC               /* SoC has SOPT8[FTM0OCH6SRC] */
#  undef  KINETIS_SIM_HAS_SOPT8_FTM0OCH7SRC               /* SoC has SOPT8[FTM0OCH7SRC] */
#  undef  KINETIS_SIM_HAS_SOPT8_FTM3OCH0SRC               /* SoC has SOPT8[FTM3OCH0SRC] */
#  undef  KINETIS_SIM_HAS_SOPT8_FTM3OCH1SRC               /* SoC has SOPT8[FTM3OCH1SRC] */
#  undef  KINETIS_SIM_HAS_SOPT8_FTM3OCH2SRC               /* SoC has SOPT8[FTM3OCH2SRC] */
#  undef  KINETIS_SIM_HAS_SOPT8_FTM3OCH3SRC               /* SoC has SOPT8[FTM3OCH3SRC] */
#  undef  KINETIS_SIM_HAS_SOPT8_FTM3OCH4SRC               /* SoC has SOPT8[FTM3OCH4SRC] */
#  undef  KINETIS_SIM_HAS_SOPT8_FTM3OCH5SRC               /* SoC has SOPT8[FTM3OCH5SRC] */
#  undef  KINETIS_SIM_HAS_SOPT8_FTM3OCH6SRC               /* SoC has SOPT8[FTM3OCH6SRC] */
#  undef  KINETIS_SIM_HAS_SOPT8_FTM3OCH7SRC               /* SoC has SOPT8[FTM3OCH7SRC] */
#  undef  KINETIS_SIM_HAS_SOPT9                           /* SoC has SOPT9 Register */
#  undef  KINETIS_SIM_HAS_SOPT9_TPM1CH0SRC                /* SoC has SOPT9[TPM1CH0SRC] */
#  undef  KINETIS_SIM_HAS_SOPT9_TPM2CH0SRC                /* SoC has SOPT9[TPM2CH0SRC] */
#  undef  KINETIS_SIM_HAS_SOPT9_TPM1CLKSEL                /* SoC has SOPT9[TPM1CLKSEL] */
#  undef  KINETIS_SIM_HAS_SOPT9_TPM2CLKSEL                /* SoC has SOPT9[TPM2CLKSEL] */
#  define KINETIS_SIM_HAS_SDID                        1   /* SoC has SDID Register */
#  undef  KINETIS_SIM_HAS_SDID_DIEID                      /* SoC has SDID[DIEID] */
#  define KINETIS_SIM_HAS_SDID_FAMID                  1   /* SoC has SDID[FAMID] */
#  undef  KINETIS_SIM_HAS_SDID_FAMILYID                   /* SoC has SDID[FAMILYID] */
#  undef  KINETIS_SIM_HAS_SDID_SERIESID                   /* SoC has SDID[SERIESID] */
#  undef  KINETIS_SIM_HAS_SDID_SRAMSIZE                   /* SoC has SDID[SRAMSIZE] */
#  undef  KINETIS_SIM_HAS_SDID_SUBFAMID                   /* SoC has SDID[SUBFAMID] */
#  define KINETIS_SIM_HAS_SCGC1                       1   /* SoC has _SCGC1 Register */
#  define KINETIS_SIM_HAS_SCGC1_UART5                 1   /* SoC has SCGC1[UART5] */
#  define KINETIS_SIM_HAS_SCGC1_UART4                 1   /* SoC has SCGC1[UART4] */
#  undef  KINETIS_SIM_HAS_SCGC1_I2C3                      /* SoC has SCGC1[I2C3] */
#  undef  KINETIS_SIM_HAS_SCGC1_I2C2                      /* SoC has SCGC1[I2C2] */
#  define KINETIS_SIM_HAS_SCGC1_OSC1                  1    /* SoC has SCGC1[OSC1] */
#  define KINETIS_SIM_HAS_SCGC2                       1   /* SoC has _SCGC2 Register */
#  define KINETIS_SIM_HAS_SCGC2                       1   /* SoC has SCGC2 Register */
#  define KINETIS_SIM_HAS_SCGC2_ENET                  1   /* SoC has SCGC2[ENET] */
#  undef  KINETIS_SIM_HAS_SCGC2_LPUART0                   /* SoC has SCGC2[LPUART0] */
#  undef  KINETIS_SIM_HAS_SCGC2_TPM1                      /* SoC has SCGC2[TPM1] */
#  undef  KINETIS_SIM_HAS_SCGC2_TPM2                      /* SoC has SCGC2[TPM2] */
#  define KINETIS_SIM_HAS_SCGC3                       1   /* SoC has SCGC3 Register */
#  define KINETIS_SIM_HAS_SCGC3_RNGA                  1   /* SoC has SCGC3[RNGA] */
#  undef  KINETIS_SIM_HAS_SCGC3_USBHS                     /* SoC has SCGC3[USBHS] */
#  undef  KINETIS_SIM_HAS_SCGC3_USBHSPHY                  /* SoC has SCGC3[USBHSPHY] */
#  undef  KINETIS_SIM_HAS_SCGC3_USBHSDCD                  /* SoC has SCGC3[USBHSDCD] */
#  define KINETIS_SIM_HAS_SCGC3_FLEXCAN1              1   /* SoC has SCGC3[FLEXCAN1] */
#  define KINETIS_SIM_HAS_SCGC3_NFC                   1   /* SoC has SCGC3[NFC] */
#  define KINETIS_SIM_HAS_SCGC3_SPI2                  1   /* SoC has SCGC3[SPI2] */
#  define KINETIS_SIM_HAS_SCGC3_SAI1                  1    /* SoC has SCGC3[SAI1] */
#  define KINETIS_SIM_HAS_SCGC3_SDHC                  1   /* SoC has SCGC3[SDHC] */
#  define KINETIS_SIM_HAS_SCGC3_FTM2                  1   /* SoC has SCGC3[FTM2] */
#  define KINETIS_SIM_HAS_SCGC3_FTM3                  1   /* SoC has SCGC3[FTM3] */
#  define KINETIS_SIM_HAS_SCGC3_ADC1                  1   /* SoC has SCGC3[ADC1] */
#  define KINETIS_SIM_HAS_SCGC3_ADC3                  1   /* SoC has SCGC3[ADC3] */
#  undef  KINETIS_SIM_HAS_SCGC3_SLCD                      /* SoC has SCGC3[SLCD] */
#  define KINETIS_SIM_HAS_SCGC4                       1   /* SoC has SCGC4 Register */
#  define KINETIS_SIM_HAS_SCGC4_LLWU                  1   /* SoC has SCGC4[LLWU] clock gate */
#  define KINETIS_SIM_HAS_SCGC4_UART0                 1   /* SoC has SCGC4[UART0] */
#  define KINETIS_SIM_HAS_SCGC4_UART1                 1   /* SoC has SCGC4[UART1] */
#  define KINETIS_SIM_HAS_SCGC4_UART2                 1   /* SoC has SCGC4[UART2] */
#  define KINETIS_SIM_HAS_SCGC4_UART3                 1   /* SoC has SCGC4[UART3] */
#  define KINETIS_SIM_HAS_SCGC5                       1   /* SoC has _SCGC5 Register */
#  undef  KINETIS_SIM_HAS_SCGC5_REGFILE                   /* SoC has SCGC5[REGFILE] */
#  define KINETIS_SIM_HAS_SCGC5_TSI                   1   /* SoC has SCGC5[TSI] */
#  define KINETIS_SIM_HAS_SCGC5_PORTF                 1   /* SoC has SCGC5[PORTF] */
#  define KINETIS_SIM_HAS_SCGC6                       1   /* SoC has SCGC6 Register */
#  undef  KINETIS_SIM_HAS_SCGC6_FTFL                      /* SoC has SCGC6[FTFL] */
#  define KINETIS_SIM_HAS_SCGC6_DMAMUX1               1   /* SoC has SCGC6[DEMUX1] */
#  define KINETIS_SIM_HAS_SCGC6_USBHS                 1   /* SoC has SCGC6[USBHS] */
#  define KINETIS_SIM_HAS_SCGC6_RNGA                  1   /* SoC has SCGC6[RNGA] */
#  undef  KINETIS_SIM_HAS_SCGC6_FTM2                      /* SoC has SCGC6[FTM2] */
#  define KINETIS_SIM_HAS_SCGC6_ADC2                  1   /* SoC has SCGC6[ADC2] */
#  undef  KINETIS_SIM_HAS_SCGC6_DAC0                      /* SoC has SCGC6[DAC0] */
#  define KINETIS_SIM_HAS_SCGC7                       1   /* SoC has SCGC7 Register */
#  define KINETIS_SIM_HAS_SCGC7_FLEXBUS               1   /* SoC has SCGC7[FLEXBUS] */
#  define KINETIS_SIM_HAS_SCGC7_DMA                   1   /* SoC has SCGC7[DMS] */
#  define KINETIS_SIM_HAS_SCGC7_MPU                   1   /* SoC has SCGC7[MPU] */
#  undef  KINETIS_SIM_HAS_SCGC7_SDRAMC                    /* SoC has SCGC7[SDRAMC] */
#  define KINETIS_SIM_HAS_CLKDIV1                     1   /* SoC has CLKDIV1 Register */
#  define KINETIS_SIM_HAS_CLKDIV1_OUTDIV2             1   /* SoC has CLKDIV1[OUTDIV2] */
#  define KINETIS_SIM_HAS_CLKDIV1_OUTDIV3             1   /* SoC has CLKDIV1[OUTDIV3] */
#  define KINETIS_SIM_HAS_CLKDIV1_OUTDIV4             1   /* SoC has CLKDIV1[OUTDIV4] */
#  undef  KINETIS_SIM_HAS_CLKDIV1_OUTDIV5                 /* SoC has CLKDIV1[OUTDIV5] */
#  define KINETIS_SIM_HAS_CLKDIV2                     1   /* SoC has CLKDIV2 Register */
#  define KINETIS_SIM_HAS_CLKDIV2_USBDIV              1   /* SoC has CLKDIV2[USBDIV] */
#  define KINETIS_SIM_HAS_CLKDIV2_USBFRAC             1   /* SoC has CLKDIV2[USBFRAC] */
#  undef  KINETIS_SIM_HAS_CLKDIV2_USBFSDIV                /* SoC has CLKDIV2[USBFSDIV] */
#  undef  KINETIS_SIM_HAS_CLKDIV2_USBFSFRAC               /* SoC has CLKDIV2[USBFSFRAC] */
#  define KINETIS_SIM_HAS_CLKDIV2_USBHSDIV            1   /* SoC has CLKDIV2[USBHSDIV] */
#  define KINETIS_SIM_HAS_CLKDIV2_USBHSFRAC           1   /* SoC has CLKDIV2[USBHSFRAC] */
#  undef  KINETIS_SIM_HAS_CLKDIV2_I2SDIV                  /* SoC has CLKDIV2[I2SDIV] */
#  undef  KINETIS_SIM_HAS_CLKDIV2_I2SFRAC                 /* SoC has CLKDIV2[I2SFRAC] */
#  define KINETIS_SIM_HAS_FCFG1                       1   /* SoC has FCFG1 Register */
#  define KINETIS_SIM_HAS_FCFG1_DEPART                1   /* SoC has FCFG1[DEPART] */
#  define KINETIS_SIM_HAS_FCFG1_EESIZE                1   /* SoC has FCFG1[EESIZE] */
#  undef  KINETIS_SIM_HAS_FCFG1_FLASHDIS                  /* SoC has FCFG1[FLASHDIS] */
#  undef  KINETIS_SIM_HAS_FCFG1_FLASHDOZE                 /* SoC has FCFG1[FLASHDOZE] */
#  define KINETIS_SIM_HAS_FCFG1_FTFDIS                1   /* SoC has FCFG1[FTFDIS] */
#  define KINETIS_SIM_HAS_FCFG1_NVMSIZE               1   /* SoC has FCFG1[NVMSIZE] */
#  define KINETIS_SIM_HAS_FCFG2                       1   /* SoC has FCFG2 Register */
#  define KINETIS_SIM_HAS_FCFG2_MAXADDR0              6   /* SoC has n bit of FCFG2[MAXADDR0] */
#  define KINETIS_SIM_HAS_FCFG2_MAXADDR1              6   /* SoC has n bit of FCFG2[MAXADDR1] */
#  define KINETIS_SIM_HAS_FCFG2_PFLSH                 1   /* SoC has FCFG2[PFLSH] */
#  define KINETIS_SIM_HAS_FCFG2_SWAPPFLSH             1   /* SoC has FCFG2[SWAPPFLSH] */
#  define KINETIS_SIM_HAS_UIDH                        1   /* SoC has UIDH Register */
#  define KINETIS_SIM_HAS_UIDMH                       1   /* SoC has UIDMH Register */
#  define KINETIS_SIM_HAS_UIDML                       1   /* SoC has UIDML Register */
#  define KINETIS_SIM_HAS_UIDL                        1   /* SoC has UIDL Register */
#  undef  KINETIS_SIM_HAS_CLKDIV3                         /* SoC has CLKDIV3 Register */
#  undef  KINETIS_SIM_HAS_CLKDIV3_PLLFLLDIV               /* SoC has CLKDIV3[PLLFLLDIV] */
#  undef  KINETIS_SIM_HAS_CLKDIV3_PLLFLLFRAC              /* SoC has CLKDIV3[PLLFLLFRAC] */
#  define KINETIS_SIM_HAS_CLKDIV4                     1   /* SoC has CLKDIV4 Register */
#  define KINETIS_SIM_HAS_CLKDIV4_TRACEDIV            1   /* SoC has CLKDIV4[TRACEDIV] */
#  define KINETIS_SIM_HAS_CLKDIV4_TRACEFRAC           1   /* SoC has CLKDIV4[TRACEFRAC] */
#  define KINETIS_SIM_HAS_CLKDIV4_NFCDIV              1   /* SoC has CLKDIV4[NFCDIV] */
#  define KINETIS_SIM_HAS_CLKDIV4_NFCFRAC             1   /* SoC has CLKDIV4[NFCFRAC] */
#  define KINETIS_SIM_HAS_MCR                         1   /* SoC has MCR Register */

#elif defined(CONFIG_ARCH_CHIP_MK64FN1M0VLL12) || defined(CONFIG_ARCH_CHIP_MK64FX512VLL12) || \
      defined(CONFIG_ARCH_CHIP_MK64FX512VDC12) || defined(CONFIG_ARCH_CHIP_MK64FN1M0VDC12) || \
      defined(CONFIG_ARCH_CHIP_MK64FX512VLQ12) || defined(CONFIG_ARCH_CHIP_MK64FN1M0VLQ12) || \
      defined(CONFIG_ARCH_CHIP_MK64FX512VMD12) || defined(CONFIG_ARCH_CHIP_MK64FN1M0VMD12)

/* Verified to Document Number: K64P144M120SF5RM Rev. 2, January 2014 */

#  define KINETIS_SIM_VERSION KINETIS_SIM_VERSION_04

/* SIM Register Configuration */

#  define KINETIS_SIM_HAS_SOPT1                       1   /* SoC has SOPT1 Register */
#  undef  KINETIS_SIM_HAS_SOPT1_OSC32KOUT                 /* SoC has SOPT1[OSC32KOUT] */
#  define KINETIS_SIM_HAS_SOPT1_OSC32KSEL             1   /* SoC has SOPT1[OSC32KSEL] */
#  define KINETIS_SIM_HAS_SOPT1_OSC32KSEL_BITS        2   /* SoC has 2 bits of SOPT1[OSC32KSEL] */
#  define KINETIS_SIM_HAS_SOPT1_RAMSIZE               1   /* SoC has SOPT1[RAMSIZE] */
#  define KINETIS_SIM_HAS_SOPT1_USBREGEN              1   /* SoC has SOPT1[USBREGEN] */
#  define KINETIS_SIM_HAS_SOPT1_USBSSTBY              1   /* SoC has SOPT1[USBSSTBY] */
#  define KINETIS_SIM_HAS_SOPT1_USBVSTBY              1   /* SoC has SOPT1[USBVSTBY] */
#  define KINETIS_SIM_HAS_SOPT1CFG                        /* SoC has SOPT1CFG Register */
#  define KINETIS_SIM_HAS_SOPT1CFG_URWE               1   /* SoC has SOPT1CFG[URWE] */
#  define KINETIS_SIM_HAS_SOPT1CFG_USSWE              1   /* SoC has SOPT1CFG[USSWE] */
#  define KINETIS_SIM_HAS_SOPT1CFG_UVSWE              1   /* SoC has SOPT1CFG[UVSWE] */
#  undef  KINETIS_SIM_HAS_USBPHYCTL                       /* SoC has USBPHYCTL Register */
#  undef  KINETIS_SIM_HAS_USBPHYCTL_USB3VOUTTRG           /* SoC has USBPHYCTL[USB3VOUTTRG] */
#  undef  KINETIS_SIM_HAS_USBPHYCTL_USBDISILIM            /* SoC has USBPHYCTL[USBDISILIM] */
#  undef  KINETIS_SIM_HAS_USBPHYCTL_USBVREGPD             /* SoC has USBPHYCTL[USBVREGPD] */
#  undef  KINETIS_SIM_HAS_USBPHYCTL_USBVREGSEL            /* SoC has USBPHYCTL[USBVREGSEL] */
#  define KINETIS_SIM_HAS_SOPT2                       1   /* SoC has SOPT2 Register */
#  define KINETIS_SIM_HAS_SOPT2_FBSL                  1   /* SoC has SOPT2[FBSL] */
#  undef  KINETIS_SIM_HAS_SOPT2_CMTUARTPAD                /* SoC has SOPT2[CMTUARTPAD] */
#  define KINETIS_SIM_HAS_SOPT2_FLEXIOSRC             1   /* SoC has SOPT2[FLEXIOSRC] */
#  undef  KINETIS_SIM_HAS_SOPT2_LPUARTSRC                 /* SoC has SOPT2[LPUARTSRC] */
#  define KINETIS_SIM_HAS_SOPT2_PLLFLLSEL             1   /* SoC has SOPT2[PLLFLLSEL] */
#  define KINETIS_SIM_HAS_SOPT2_PLLFLLSEL_BITS        2   /* SoC has 2 bits of SOPT2[PLLFLLSEL] */
#  define KINETIS_SIM_HAS_SOPT2_PTD7PAD               1   /* SoC has SOPT2[PTD7PAD] */
#  define KINETIS_SIM_HAS_SOPT2_RMIISRC               1   /* SoC has SOPT2[RMIISRC] */
#  define KINETIS_SIM_HAS_SOPT2_RTCCLKOUTSEL          1   /* SoC has SOPT2[RTCCLKOUTSEL] */
#  define KINETIS_SIM_HAS_SOPT2_CLKOUTSEL             1   /* SoC has SOPT2[CLKOUTSEL] */
#  define KINETIS_SIM_HAS_SOPT2_SDHCSRC               1   /* SoC has SOPT2[SDHCSRC] */
#  undef  KINETIS_SIM_HAS_SOPT2_NFCSRC                    /* SoC has SOPT2[NFCSRC] */
#  undef  KINETIS_SIM_HAS_SOPT2_I2SSRC                    /* SoC has SOPT2[I2SSRC] */
#  define KINETIS_SIM_HAS_SOPT2_TIMESRC               1   /* SoC has SOPT2[TIMESRC] */
#  undef  KINETIS_SIM_HAS_SOPT2_TPMSRC                    /* SoC has SOPT2[TPMSRC] */
#  undef  KINETIS_SIM_HAS_SOPT2_USBFSRC                   /* SoC has SOPT2[USBFSRC] */
#  define KINETIS_SIM_HAS_SOPT2_TRACECLKSEL           1   /* SoC has SOPT2[TRACECLKSEL] */
#  undef  KINETIS_SIM_HAS_SOPT2_USBREGEN                  /* SoC has SOPT2[USBREGEN] */
#  undef  KINETIS_SIM_HAS_SOPT2_USBSLSRC                  /* SoC has SOPT2[USBSLSRC] */
#  undef  KINETIS_SIM_HAS_SOPT2_USBHSRC                   /* SoC has SOPT2[USBHSRC] */
#  define KINETIS_SIM_HAS_SOPT2_USBSRC                1   /* SoC has SOPT2[USBSRC] */
#  undef  KINETIS_SIM_HAS_SOPT2_MCGCLKSEL                 /* SoC has SOPT2[MCGCLKSEL] */
#  define KINETIS_SIM_HAS_SOPT4                       1   /* SoC has SOPT4 Register */
#  define KINETIS_SIM_HAS_SOPT4_FTM0FLT0              1   /* SoC has SOPT4[FTM0FLT0] */
#  define KINETIS_SIM_HAS_SOPT4_FTM0FLT1              1   /* SoC has SOPT4[FTM0FLT1] */
#  define KINETIS_SIM_HAS_SOPT4_FTM0FLT2              1   /* SoC has SOPT4[FTM0FLT2] */
#  undef  KINETIS_SIM_HAS_SOPT4_FTM0FLT3                  /* SoC has SOPT4[FTM0FLT3] */
#  define KINETIS_SIM_HAS_SOPT4_FTM0TRG0SRC           1   /* SoC has SOPT4[FTM0TRG0SRC] */
#  define KINETIS_SIM_HAS_SOPT4_FTM0TRG1SRC           1   /* SoC has SOPT4[FTM0TRG1SRC] */
#  define KINETIS_SIM_HAS_SOPT4_FTM1CH0SRC            3   /* SoC has SOPT4[FTM1CH0SRC] 1, 3 if SOF */
#  define KINETIS_SIM_HAS_SOPT4_FTM1FLT0              1   /* SoC has SOPT4[FTM1FLT0] */
#  define KINETIS_SIM_HAS_SOPT4_FTM1FLT1              1   /* SoC has SOPT4[FTM1FLT1] */
#  define KINETIS_SIM_HAS_SOPT4_FTM1FLT2              1   /* SoC has SOPT4[FTM1FLT2] */
#  define KINETIS_SIM_HAS_SOPT4_FTM1FLT3              1   /* SoC has SOPT4[FTM1FLT3] */
#  define KINETIS_SIM_HAS_SOPT4_FTM2CH0SRC            1   /* SoC has SOPT4[FTM2CH0SRC] */
#  undef  KINETIS_SIM_HAS_SOPT4_FTM2CH1SRC                /* SoC has SOPT4[FTM2CH1SRC] */
#  define KINETIS_SIM_HAS_SOPT4_FTM2FLT0              1   /* SoC has SOPT4[FTM2FLT0] */
#  define KINETIS_SIM_HAS_SOPT4_FTM2FLT1              1   /* SoC has SOPT4[FTM2FLT1] */
#  define KINETIS_SIM_HAS_SOPT4_FTM2FLT2              1   /* SoC has SOPT4[FTM2FLT2] */
#  define KINETIS_SIM_HAS_SOPT4_FTM2FLT3              1   /* SoC has SOPT4[FTM2FLT3] */
#  define KINETIS_SIM_HAS_SOPT4_FTM3CH0SRC            1   /* SoC has SOPT4[FTM3CH0SRC] */
#  define KINETIS_SIM_HAS_SOPT4_FTM3FLT0              1   /* SoC has SOPT4[FTM3FLT0] */
#  define KINETIS_SIM_HAS_SOPT4_FTM3FLT1              1   /* SoC has SOPT4[FTM3FLT1] */
#  define KINETIS_SIM_HAS_SOPT4_FTM3FLT2              1   /* SoC has SOPT4[FTM3FLT2] */
#  define KINETIS_SIM_HAS_SOPT4_FTM3FLT3              1   /* SoC has SOPT4[FTM3FLT3] */
#  define KINETIS_SIM_HAS_SOPT4_FTM3TRG0SRC           1   /* SoC has SOPT4[FTM3TRG0SRC] */
#  define KINETIS_SIM_HAS_SOPT4_FTM3TRG1SRC           1   /* SoC has SOPT4[FTM3TRG1SRC] */
#  define KINETIS_SIM_HAS_SOPT4_TPM0CLKSEL            1   /* SoC has SOPT4[TPM0CLKSEL] */
#  define KINETIS_SIM_HAS_SOPT4_TPM1CH0SRC            1   /* SoC has SOPT4[TPM1CH0SRC] */
#  define KINETIS_SIM_HAS_SOPT4_TPM1CLKSEL            1   /* SoC has SOPT4[TPM1CLKSEL] */
#  define KINETIS_SIM_HAS_SOPT4_TPM2CH0SRC            1   /* SoC has SOPT4[TPM2CH0SRC] */
#  define KINETIS_SIM_HAS_SOPT4_TPM2CLKSEL            1   /* SoC has SOPT4[TPM2CLKSEL] */
#  define KINETIS_SIM_HAS_SOPT5                       1   /* SoC has SOPT5 Register */
#  undef  KINETIS_SIM_HAS_SOPT5_LPUART0RXSRC              /* SoC has SOPT5[LPUART0RXSRC] */
#  undef  KINETIS_SIM_HAS_SOPT5_LPUART0TXSRC              /* SoC has SOPT5[LPUART0TXSRC] */
#  undef  KINETIS_SIM_HAS_SOPT6                           /* SoC has SOPT6 Register */
#  undef  KINETIS_SIM_HAS_SOPT6_MCC                       /* SoC has SOPT6[MCC] */
#  undef  KINETIS_SIM_HAS_SOPT6_PCR                       /* SoC has SOPT6[PCR] */
#  undef  KINETIS_SIM_HAS_SOPT6_RSTFLTSEL                 /* SoC has SOPT6[RSTFLTSEL] */
#  undef  KINETIS_SIM_HAS_SOPT6_RSTFLTEN                  /* SoC has SOPT6[RSTFLTEN] */
#  define KINETIS_SIM_HAS_SOPT7                       1   /* SoC has SOPT7 Register */
#  define KINETIS_SIM_HAS_SOPT7_ADC0ALTTRGSEL         1   /* SoC has SOPT7[ADC0ALTTRGSEL] */
#  define KINETIS_SIM_HAS_SOPT7_ADC1ALTTRGSEL         1   /* SoC has SOPT7[ADC1ALTTRGSEL] */
#  define KINETIS_SIM_HAS_SOPT7_ADC0PRETRGSEL         1   /* SoC has SOPT7[ADC0PRETRGSEL] */
#  define KINETIS_SIM_HAS_SOPT7_ADC1PRETRGSEL         1   /* SoC has SOPT7[ADC1PRETRGSEL] */
#  undef  KINETIS_SIM_HAS_SOPT7_ADC2PRETRGSEL             /* SoC has SOPT7[ADC2PRETRGSEL] */
#  undef  KINETIS_SIM_HAS_SOPT7_ADC3PRETRGSEL             /* SoC has SOPT7[ADC3PRETRGSEL] */
#  define KINETIS_SIM_HAS_SOPT7_ADC0TRGSEL            14  /* SoC has 10 SOPT7[ADC0TRGSEL] */
#  define KINETIS_SIM_HAS_SOPT7_ADC1TRGSEL            14  /* SoC has 10 SOPT7[ADC1TRGSEL] */
#  undef  KINETIS_SIM_HAS_SOPT7_ADC2TRGSEL                /* SoC has 10 SOPT7[ADC2TRGSEL] */
#  undef  KINETIS_SIM_HAS_SOPT7_ADC3TRGSEL                /* SoC has 10 SOPT7[ADC3TRGSEL] */
#  define KINETIS_SIM_SOPT7_ADC0ALTTRGEN              1   /* ADC0 alternate trigger enable */
#  define KINETIS_SIM_SOPT7_ADC1ALTTRGEN              1   /* ADC1 alternate trigger enable */
#  undef  KINETIS_SIM_SOPT7_ADC2ALTTRGEN                  /* ADC2 alternate trigger enable */
#  undef  KINETIS_SIM_SOPT7_ADC3ALTTRGEN                  /* ADC3 alternate trigger enable */
#  undef  KINETIS_SIM_HAS_SOPT8                           /* SoC has SOPT8 Register */
#  undef  KINETIS_SIM_HAS_SOPT8_FTM0SYNCBIT               /* SoC has SOPT8[FTM0SYNCBIT] */
#  undef  KINETIS_SIM_HAS_SOPT8_FTM1SYNCBIT               /* SoC has SOPT8[FTM1SYNCBIT] */
#  undef  KINETIS_SIM_HAS_SOPT8_FTM2SYNCBIT               /* SoC has SOPT8[FTM2SYNCBIT] */
#  undef  KINETIS_SIM_HAS_SOPT8_FTM3SYNCBIT               /* SoC has SOPT8[FTM3SYNCBIT] */
#  undef  KINETIS_SIM_HAS_SOPT8_FTM0OCH0SRC               /* SoC has SOPT8[FTM0OCH0SRC] */
#  undef  KINETIS_SIM_HAS_SOPT8_FTM0OCH1SRC               /* SoC has SOPT8[FTM0OCH1SRC] */
#  undef  KINETIS_SIM_HAS_SOPT8_FTM0OCH2SRC               /* SoC has SOPT8[FTM0OCH2SRC] */
#  undef  KINETIS_SIM_HAS_SOPT8_FTM0OCH3SRC               /* SoC has SOPT8[FTM0OCH3SRC] */
#  undef  KINETIS_SIM_HAS_SOPT8_FTM0OCH4SRC               /* SoC has SOPT8[FTM0OCH4SRC] */
#  undef  KINETIS_SIM_HAS_SOPT8_FTM0OCH5SRC               /* SoC has SOPT8[FTM0OCH5SRC] */
#  undef  KINETIS_SIM_HAS_SOPT8_FTM0OCH6SRC               /* SoC has SOPT8[FTM0OCH6SRC] */
#  undef  KINETIS_SIM_HAS_SOPT8_FTM0OCH7SRC               /* SoC has SOPT8[FTM0OCH7SRC] */
#  undef  KINETIS_SIM_HAS_SOPT8_FTM3OCH0SRC               /* SoC has SOPT8[FTM3OCH0SRC] */
#  undef  KINETIS_SIM_HAS_SOPT8_FTM3OCH1SRC               /* SoC has SOPT8[FTM3OCH1SRC] */
#  undef  KINETIS_SIM_HAS_SOPT8_FTM3OCH2SRC               /* SoC has SOPT8[FTM3OCH2SRC] */
#  undef  KINETIS_SIM_HAS_SOPT8_FTM3OCH3SRC               /* SoC has SOPT8[FTM3OCH3SRC] */
#  undef  KINETIS_SIM_HAS_SOPT8_FTM3OCH4SRC               /* SoC has SOPT8[FTM3OCH4SRC] */
#  undef  KINETIS_SIM_HAS_SOPT8_FTM3OCH5SRC               /* SoC has SOPT8[FTM3OCH5SRC] */
#  undef  KINETIS_SIM_HAS_SOPT8_FTM3OCH6SRC               /* SoC has SOPT8[FTM3OCH6SRC] */
#  undef  KINETIS_SIM_HAS_SOPT8_FTM3OCH7SRC               /* SoC has SOPT8[FTM3OCH7SRC] */
#  undef  KINETIS_SIM_HAS_SOPT9                           /* SoC has SOPT9 Register */
#  undef  KINETIS_SIM_HAS_SOPT9_TPM1CH0SRC                /* SoC has SOPT9[TPM1CH0SRC] */
#  undef  KINETIS_SIM_HAS_SOPT9_TPM2CH0SRC                /* SoC has SOPT9[TPM2CH0SRC] */
#  undef  KINETIS_SIM_HAS_SOPT9_TPM1CLKSEL                /* SoC has SOPT9[TPM1CLKSEL] */
#  undef  KINETIS_SIM_HAS_SOPT9_TPM2CLKSEL                /* SoC has SOPT9[TPM2CLKSEL] */
#  define KINETIS_SIM_HAS_SDID                        1   /* SoC has SDID Register */
#  define KINETIS_SIM_HAS_SDID_DIEID                  1   /* SoC has SDID[DIEID] */
#  define KINETIS_SIM_HAS_SDID_FAMID                  1   /* SoC has SDID[FAMID] */
#  define KINETIS_SIM_HAS_SDID_FAMILYID               1   /* SoC has SDID[FAMILYID] */
#  define KINETIS_SIM_HAS_SDID_SERIESID               1   /* SoC has SDID[SERIESID] */
#  undef  KINETIS_SIM_HAS_SDID_SRAMSIZE                   /* SoC has SDID[SRAMSIZE] */
#  define KINETIS_SIM_HAS_SDID_SUBFAMID               1   /* SoC has SDID[SUBFAMID] */
#  define KINETIS_SIM_HAS_SCGC1                       1   /* SoC has _SCGC1 Register */
#  define KINETIS_SIM_HAS_SCGC1_UART5                 1   /* SoC has SCGC1[UART5] */
#  define KINETIS_SIM_HAS_SCGC1_UART4                 1   /* SoC has SCGC1[UART4] */
#  undef  KINETIS_SIM_HAS_SCGC1_I2C3                      /* SoC has SCGC1[I2C3] */
#  define KINETIS_SIM_HAS_SCGC1_I2C2                  1   /* SoC has SCGC1[I2C2] */
#  undef  KINETIS_SIM_HAS_SCGC1_OSC1                      /* SoC has SCGC1[OSC1] */
#  define KINETIS_SIM_HAS_SCGC2                       1   /* SoC has _SCGC2 Register */
#  define KINETIS_SIM_HAS_SCGC2                       1   /* SoC has SCGC2 Register */
#  define KINETIS_SIM_HAS_SCGC2_ENET                  1   /* SoC has SCGC2[ENET] */
#  undef  KINETIS_SIM_HAS_SCGC2_LPUART0                   /* SoC has SCGC2[LPUART0] */
#  undef  KINETIS_SIM_HAS_SCGC2_TPM1                      /* SoC has SCGC2[TPM1] */
#  undef  KINETIS_SIM_HAS_SCGC2_TPM2                      /* SoC has SCGC2[TPM2] */
#  define KINETIS_SIM_HAS_SCGC3                       1   /* SoC has SCGC3 Register */
#  define KINETIS_SIM_HAS_SCGC3_RNGA                  1   /* SoC has SCGC3[RNGA] */
#  undef  KINETIS_SIM_HAS_SCGC3_USBHS                     /* SoC has SCGC3[USBHS] */
#  undef  KINETIS_SIM_HAS_SCGC3_USBHSPHY                  /* SoC has SCGC3[USBHSPHY] */
#  undef  KINETIS_SIM_HAS_SCGC3_USBHSDCD                  /* SoC has SCGC3[USBHSDCD] */
#  undef  KINETIS_SIM_HAS_SCGC3_FLEXCAN1                  /* SoC has SCGC3[FLEXCAN1] */
#  undef  KINETIS_SIM_HAS_SCGC3_NFC                       /* SoC has SCGC3[NFC] */
#  define KINETIS_SIM_HAS_SCGC3_SPI2                  1   /* SoC has SCGC3[SPI2] */
#  undef  KINETIS_SIM_HAS_SCGC3_SAI1                      /* SoC has SCGC3[SAI1] */
#  define KINETIS_SIM_HAS_SCGC3_SDHC                  1   /* SoC has SCGC3[SDHC] */
#  define KINETIS_SIM_HAS_SCGC3_FTM2                  1   /* SoC has SCGC3[FTM2] */
#  define KINETIS_SIM_HAS_SCGC3_FTM3                  1   /* SoC has SCGC3[FTM3] */
#  define KINETIS_SIM_HAS_SCGC3_ADC1                  1   /* SoC has SCGC3[ADC1] */
#  undef  KINETIS_SIM_HAS_SCGC3_ADC3                      /* SoC has SCGC3[ADC3] */
#  undef  KINETIS_SIM_HAS_SCGC3_SLCD                      /* SoC has SCGC3[SLCD] */
#  define KINETIS_SIM_HAS_SCGC4                       1   /* SoC has SCGC4 Register */
#  undef  KINETIS_SIM_HAS_SCGC4_LLWU                      /* SoC has SCGC4[LLWU] clock gate */
#  define KINETIS_SIM_HAS_SCGC4_UART0                 1   /* SoC has SCGC4[UART0] */
#  define KINETIS_SIM_HAS_SCGC4_UART1                 1   /* SoC has SCGC4[UART1] */
#  define KINETIS_SIM_HAS_SCGC4_UART2                 1   /* SoC has SCGC4[UART2] */
#  define KINETIS_SIM_HAS_SCGC4_UART3                 1   /* SoC has SCGC4[UART3] */
#  define KINETIS_SIM_HAS_SCGC5                       1   /* SoC has _SCGC5 Register */
#  undef  KINETIS_SIM_HAS_SCGC5_REGFILE                   /* SoC has SCGC5[REGFILE] */
#  undef  KINETIS_SIM_HAS_SCGC5_TSI                       /* SoC has SCGC5[TSI] */
#  undef  KINETIS_SIM_HAS_SCGC5_PORTF                     /* SoC has SCGC5[PORTF] */
#  define KINETIS_SIM_HAS_SCGC6                       1   /* SoC has SCGC6 Register */
#  define KINETIS_SIM_HAS_SCGC6_FTFL                  1   /* SoC has SCGC6[FTFL] */
#  undef  KINETIS_SIM_HAS_SCGC6_DMAMUX1                   /* SoC has SCGC6[DEMUX1] */
#  undef  KINETIS_SIM_HAS_SCGC6_USBHS                     /* SoC has SCGC6[USBHS] */
#  define KINETIS_SIM_HAS_SCGC6_RNGA                  1   /* SoC has SCGC6[RNGA] */
#  define KINETIS_SIM_HAS_SCGC6_FTM2                  1   /* SoC has SCGC6[FTM2] */
#  undef  KINETIS_SIM_HAS_SCGC6_ADC2                      /* SoC has SCGC6[ADC2] */
#  define KINETIS_SIM_HAS_SCGC6_DAC0                  1   /* SoC has SCGC6[DAC0] */
#  define KINETIS_SIM_HAS_SCGC7                       1   /* SoC has SCGC7 Register */
#  define KINETIS_SIM_HAS_SCGC7_FLEXBUS               1   /* SoC has SCGC7[FLEXBUS] */
#  define KINETIS_SIM_HAS_SCGC7_DMA                   1   /* SoC has SCGC7[DMS] */
#  define KINETIS_SIM_HAS_SCGC7_MPU                   1   /* SoC has SCGC7[MPU] */
#  undef  KINETIS_SIM_HAS_SCGC7_SDRAMC                    /* SoC has SCGC7[SDRAMC] */
#  define KINETIS_SIM_HAS_CLKDIV1                     1   /* SoC has CLKDIV1 Register */
#  define KINETIS_SIM_HAS_CLKDIV1_OUTDIV2             1   /* SoC has CLKDIV1[OUTDIV2] */
#  define KINETIS_SIM_HAS_CLKDIV1_OUTDIV3             1   /* SoC has CLKDIV1[OUTDIV3] */
#  define KINETIS_SIM_HAS_CLKDIV1_OUTDIV4             1   /* SoC has CLKDIV1[OUTDIV4] */
#  undef  KINETIS_SIM_HAS_CLKDIV1_OUTDIV5                 /* SoC has CLKDIV1[OUTDIV5] */
#  define KINETIS_SIM_HAS_CLKDIV2                     1   /* SoC has CLKDIV2 Register */
#  define KINETIS_SIM_HAS_CLKDIV2_USBDIV              1   /* SoC has CLKDIV2[USBDIV] */
#  define KINETIS_SIM_HAS_CLKDIV2_USBFRAC             1   /* SoC has CLKDIV2[USBFRAC] */
#  undef  KINETIS_SIM_HAS_CLKDIV2_USBFSDIV                /* SoC has CLKDIV2[USBFSDIV] */
#  undef  KINETIS_SIM_HAS_CLKDIV2_USBFSFRAC               /* SoC has CLKDIV2[USBFSFRAC] */
#  undef  KINETIS_SIM_HAS_CLKDIV2_USBHSDIV                /* SoC has CLKDIV2[USBHSDIV] */
#  undef  KINETIS_SIM_HAS_CLKDIV2_USBHSFRAC               /* SoC has CLKDIV2[USBHSFRAC] */
#  undef  KINETIS_SIM_HAS_CLKDIV2_I2SDIV                  /* SoC has CLKDIV2[I2SDIV] */
#  undef  KINETIS_SIM_HAS_CLKDIV2_I2SFRAC                 /* SoC has CLKDIV2[I2SFRAC] */
#  define KINETIS_SIM_HAS_FCFG1                       1   /* SoC has FCFG1 Register */
#  define KINETIS_SIM_HAS_FCFG1_DEPART                1   /* SoC has FCFG1[DEPART] */
#  define KINETIS_SIM_HAS_FCFG1_EESIZE                1   /* SoC has FCFG1[EESIZE] */
#  define KINETIS_SIM_HAS_FCFG1_FLASHDIS              1   /* SoC has FCFG1[FLASHDIS] */
#  define KINETIS_SIM_HAS_FCFG1_FLASHDOZE             1   /* SoC has FCFG1[FLASHDOZE] */
#  undef  KINETIS_SIM_HAS_FCFG1_FTFDIS                    /* SoC has FCFG1[FTFDIS] */
#  define KINETIS_SIM_HAS_FCFG1_NVMSIZE               1   /* SoC has FCFG1[NVMSIZE] */
#  define KINETIS_SIM_HAS_FCFG2                       1   /* SoC has FCFG2 Register */
#  define KINETIS_SIM_HAS_FCFG2_MAXADDR0              7   /* SoC has n bit of FCFG2[MAXADDR0] */
#  define KINETIS_SIM_HAS_FCFG2_MAXADDR1              7   /* SoC has n bit of FCFG2[MAXADDR1] */
#  define KINETIS_SIM_HAS_FCFG2_PFLSH                 1   /* SoC has FCFG2[PFLSH] */
#  undef  KINETIS_SIM_HAS_FCFG2_SWAPPFLSH                 /* SoC has FCFG2[SWAPPFLSH] */
#  define KINETIS_SIM_HAS_UIDH                        1   /* SoC has UIDH Register */
#  define KINETIS_SIM_HAS_UIDMH                       1   /* SoC has UIDMH Register */
#  define KINETIS_SIM_HAS_UIDML                       1   /* SoC has UIDML Register */
#  define KINETIS_SIM_HAS_UIDL                        1   /* SoC has UIDL Register */
#  undef  KINETIS_SIM_HAS_CLKDIV3                         /* SoC has CLKDIV3 Register */
#  undef  KINETIS_SIM_HAS_CLKDIV3_PLLFLLDIV               /* SoC has CLKDIV3[PLLFLLDIV] */
#  undef  KINETIS_SIM_HAS_CLKDIV3_PLLFLLFRAC              /* SoC has CLKDIV3[PLLFLLFRAC] */
#  undef  KINETIS_SIM_HAS_CLKDIV4                         /* SoC has CLKDIV4 Register */
#  undef  KINETIS_SIM_HAS_CLKDIV4_TRACEDIV                /* SoC has CLKDIV4[TRACEDIV] */
#  undef  KINETIS_SIM_HAS_CLKDIV4_TRACEFRAC               /* SoC has CLKDIV4[TRACEFRAC] */
#  undef  KINETIS_SIM_HAS_CLKDIV4_NFCDIV                  /* SoC has CLKDIV4[NFCDIV] */
#  undef  KINETIS_SIM_HAS_CLKDIV4_NFCFRAC                 /* SoC has CLKDIV4[NFCFRAC] */
#  undef  KINETIS_SIM_HAS_MCR                             /* SoC has MCR Register */

/* MK66F N/X 1M0/2M0 V MD/LQ 18
 *
 *  --------------- ------- --- ------- ------- ------ ------ ------ -----
 *  PART NUMBER     CPU     PIN PACKAGE  TOTAL  PROGRAM EEPROM SRAM  GPIO
 *                  FREQ    CNT          FLASH  FLASH
 *  --------------- ------- --- ------- ------- ------ ------ ------ -----
 *  MK66FN2M0VMD18  180 MHz 144 MAPBGA   2   MB    —    — KB  260 KB 100
 *  MK66FX1M0VMD18  180 MHz 144 MAPBGA  1.25 MB  1 MB   4 KB  256 KB 100
 *  MK66FN2M0VLQ18  180 MHz 144 LQFP     2   MB    —    — KB  260 KB 100
 *  MK66FX1M0VLQ18  180 MHz 144 LQFP    1.25 MB  1 MB   4 KB  256 KB 100
 */

#elif defined(CONFIG_ARCH_CHIP_MK66FN2M0VMD18) || defined(CONFIG_ARCH_CHIP_MK66FX1M0VMD18) || \
      defined(CONFIG_ARCH_CHIP_MK66FN2M0VLQ18) || defined(CONFIG_ARCH_CHIP_MK66FX1M0VLQ18)

/* Verified to Document Number: Document Number: K66P144M180SF5RMV2 Rev. 2, May 2015 */

#  define KINETIS_SIM_VERSION KINETIS_SIM_VERSION_06

/* SIM Register Configuration */

#  define KINETIS_SIM_HAS_SOPT1                       1   /* SoC has SOPT1 Register */
#  undef  KINETIS_SIM_HAS_SOPT1_OSC32KOUT                 /* SoC has SOPT1[OSC32KOUT] */
#  define KINETIS_SIM_HAS_SOPT1_OSC32KSEL             1   /* SoC has SOPT1[OSC32KSEL] */
#  define KINETIS_SIM_HAS_SOPT1_OSC32KSEL_BITS        2   /* SoC has 1 bit SOPT1[OSC32KSEL] */
#  define KINETIS_SIM_HAS_SOPT1_RAMSIZE               1   /* SoC has SOPT1[RAMSIZE] */
#  define KINETIS_SIM_HAS_SOPT1_USBREGEN              1   /* SoC has SOPT1[USBREGEN] */
#  define KINETIS_SIM_HAS_SOPT1_USBSSTBY              1   /* SoC has SOPT1[USBSSTBY] */
#  define KINETIS_SIM_HAS_SOPT1_USBVSTBY              1   /* SoC has SOPT1[USBVSTBY] */
#  define KINETIS_SIM_HAS_SOPT1CFG                        /* SoC has SOPT1CFG Register */
#  define KINETIS_SIM_HAS_SOPT1CFG_URWE               1   /* SoC has SOPT1CFG[URWE] */
#  define KINETIS_SIM_HAS_SOPT1CFG_USSWE              1   /* SoC has SOPT1CFG[USSWE] */
#  define KINETIS_SIM_HAS_SOPT1CFG_UVSWE              1   /* SoC has SOPT1CFG[UVSWE] */
#  define KINETIS_SIM_HAS_USBPHYCTL                   1   /* SoC has USBPHYCTL Register */
#  define KINETIS_SIM_HAS_USBPHYCTL_USB3VOUTTRG       1   /* SoC has USBPHYCTL[USB3VOUTTRG] */
#  define KINETIS_SIM_HAS_USBPHYCTL_USBDISILIM        1   /* SoC has USBPHYCTL[USBDISILIM] */
#  define KINETIS_SIM_HAS_USBPHYCTL_USBVREGPD         1   /* SoC has USBPHYCTL[USBVREGPD] */
#  define KINETIS_SIM_HAS_USBPHYCTL_USBVREGSEL        1   /* SoC has USBPHYCTL[USBVREGSEL] */
#  define KINETIS_SIM_HAS_SOPT2                       1   /* SoC has SOPT2 Register */
#  define KINETIS_SIM_HAS_SOPT2_FBSL                  1   /* SoC has SOPT2[FBSL] */
#  undef  KINETIS_SIM_HAS_SOPT2_CMTUARTPAD                /* SoC has SOPT2[CMTUARTPAD] */
#  define KINETIS_SIM_HAS_SOPT2_FLEXIOSRC             1   /* SoC has SOPT2[FLEXIOSRC] */
#  define KINETIS_SIM_HAS_SOPT2_LPUARTSRC             1   /* SoC has SOPT2[LPUARTSRC] */
#  define KINETIS_SIM_HAS_SOPT2_PLLFLLSEL             1   /* SoC has SOPT2[PLLFLLSEL] */
#  define KINETIS_SIM_HAS_SOPT2_PLLFLLSEL_BITS        2   /* SoC has 2 bits of SOPT2[PLLFLLSEL] */
#  undef  KINETIS_SIM_HAS_SOPT2_PTD7PAD                   /* SoC has SOPT2[PTD7PAD] */
#  define KINETIS_SIM_HAS_SOPT2_RMIISRC               1   /* SoC has SOPT2[RMIISRC] */
#  define KINETIS_SIM_HAS_SOPT2_RTCCLKOUTSEL          1   /* SoC has SOPT2[RTCCLKOUTSEL] */
#  define KINETIS_SIM_HAS_SOPT2_CLKOUTSEL             1   /* SoC has SOPT2[CLKOUTSEL] */
#  define KINETIS_SIM_HAS_SOPT2_SDHCSRC               1   /* SoC has SOPT2[SDHCSRC] */
#  undef  KINETIS_SIM_HAS_SOPT2_NFCSRC                    /* SoC has SOPT2[NFCSRC] */
#  undef  KINETIS_SIM_HAS_SOPT2_I2SSRC                    /* SoC has SOPT2[I2SSRC] */
#  define KINETIS_SIM_HAS_SOPT2_TIMESRC               1   /* SoC has SOPT2[TIMESRC] */
#  define KINETIS_SIM_HAS_SOPT2_TPMSRC                1   /* SoC has SOPT2[TPMSRC] */
#  undef  KINETIS_SIM_HAS_SOPT2_USBFSRC                   /* SoC has SOPT2[USBFSRC] */
#  define KINETIS_SIM_HAS_SOPT2_TRACECLKSEL           1   /* SoC has SOPT2[TRACECLKSEL] */
#  define KINETIS_SIM_HAS_SOPT2_USBREGEN              1   /* SoC has SOPT2[USBREGEN] */
#  define KINETIS_SIM_HAS_SOPT2_USBSLSRC              1   /* SoC has SOPT2[USBSLSRC] */
#  undef  KINETIS_SIM_HAS_SOPT2_USBHSRC                   /* SoC has SOPT2[USBHSRC] */
#  define KINETIS_SIM_HAS_SOPT2_USBSRC                1   /* SoC has SOPT2[USBSRC] */
#  undef  KINETIS_SIM_HAS_SOPT2_MCGCLKSEL                 /* SoC has SOPT2[MCGCLKSEL] */
#  define KINETIS_SIM_HAS_SOPT4                       1   /* SoC has SOPT4 Register */
#  define KINETIS_SIM_HAS_SOPT4_FTM0FLT0              1   /* SoC has SOPT4[FTM0FLT0] */
#  define KINETIS_SIM_HAS_SOPT4_FTM0FLT1              1   /* SoC has SOPT4[FTM0FLT1] */
#  define KINETIS_SIM_HAS_SOPT4_FTM0FLT2              1   /* SoC has SOPT4[FTM0FLT2] */
#  define KINETIS_SIM_HAS_SOPT4_FTM0FLT3              1   /* SoC has SOPT4[FTM0FLT3] */
#  define KINETIS_SIM_HAS_SOPT4_FTM0TRG0SRC           1   /* SoC has SOPT4[FTM0TRG0SRC] */
#  define KINETIS_SIM_HAS_SOPT4_FTM0TRG1SRC           1   /* SoC has SOPT4[FTM0TRG1SRC] */
#  define KINETIS_SIM_HAS_SOPT4_FTM1CH0SRC            3   /* SoC has SOPT4[FTM1CH0SRC] 1, 3 if SOF */
#  define KINETIS_SIM_HAS_SOPT4_FTM1FLT0              1   /* SoC has SOPT4[FTM1FLT0] */
#  define KINETIS_SIM_HAS_SOPT4_FTM1FLT1              1   /* SoC has SOPT4[FTM1FLT1] */
#  define KINETIS_SIM_HAS_SOPT4_FTM1FLT2              1   /* SoC has SOPT4[FTM1FLT2] */
#  define KINETIS_SIM_HAS_SOPT4_FTM1FLT3              1   /* SoC has SOPT4[FTM1FLT3] */
#  define KINETIS_SIM_HAS_SOPT4_FTM2CH0SRC            1   /* SoC has SOPT4[FTM2CH0SRC] */
#  define KINETIS_SIM_HAS_SOPT4_FTM2CH1SRC            1   /* SoC has SOPT4[FTM2CH1SRC] */
#  define KINETIS_SIM_HAS_SOPT4_FTM2FLT0              1   /* SoC has SOPT4[FTM2FLT0] */
#  define KINETIS_SIM_HAS_SOPT4_FTM2FLT1              1   /* SoC has SOPT4[FTM2FLT1] */
#  define KINETIS_SIM_HAS_SOPT4_FTM2FLT2              1   /* SoC has SOPT4[FTM2FLT2] */
#  define KINETIS_SIM_HAS_SOPT4_FTM2FLT3              1   /* SoC has SOPT4[FTM2FLT3] */
#  define KINETIS_SIM_HAS_SOPT4_FTM3CH0SRC            1   /* SoC has SOPT4[FTM3CH0SRC] */
#  define KINETIS_SIM_HAS_SOPT4_FTM3FLT0              1   /* SoC has SOPT4[FTM3FLT0] */
#  define KINETIS_SIM_HAS_SOPT4_FTM3FLT1              1   /* SoC has SOPT4[FTM3FLT1] */
#  define KINETIS_SIM_HAS_SOPT4_FTM3FLT2              1   /* SoC has SOPT4[FTM3FLT2] */
#  define KINETIS_SIM_HAS_SOPT4_FTM3FLT3              1   /* SoC has SOPT4[FTM3FLT3] */
#  define KINETIS_SIM_HAS_SOPT4_FTM3TRG0SRC           1   /* SoC has SOPT4[FTM3TRG0SRC] */
#  define KINETIS_SIM_HAS_SOPT4_FTM3TRG1SRC           1   /* SoC has SOPT4[FTM3TRG1SRC] */
#  define KINETIS_SIM_HAS_SOPT4_TPM0CLKSEL            1   /* SoC has SOPT4[TPM0CLKSEL] */
#  define KINETIS_SIM_HAS_SOPT4_TPM1CH0SRC            1   /* SoC has SOPT4[TPM1CH0SRC] */
#  define KINETIS_SIM_HAS_SOPT4_TPM1CLKSEL            1   /* SoC has SOPT4[TPM1CLKSEL] */
#  define KINETIS_SIM_HAS_SOPT4_TPM2CH0SRC            1   /* SoC has SOPT4[TPM2CH0SRC] */
#  define KINETIS_SIM_HAS_SOPT4_TPM2CLKSEL            1   /* SoC has SOPT4[TPM2CLKSEL] */
#  define KINETIS_SIM_HAS_SOPT5                       1   /* SoC has SOPT5 Register */
#  define KINETIS_SIM_HAS_SOPT5_LPUART0RXSRC          1   /* SoC has SOPT5[LPUART0RXSRC] */
#  define KINETIS_SIM_HAS_SOPT5_LPUART0TXSRC          1   /* SoC has SOPT5[LPUART0TXSRC] */
#  undef  KINETIS_SIM_HAS_SOPT6                           /* SoC has SOPT6 Register */
#  undef  KINETIS_SIM_HAS_SOPT6_MCC                       /* SoC has SOPT6[MCC] */
#  undef  KINETIS_SIM_HAS_SOPT6_PCR                       /* SoC has SOPT6[PCR] */
#  undef  KINETIS_SIM_HAS_SOPT6_RSTFLTSEL                 /* SoC has SOPT6[RSTFLTSEL] */
#  undef  KINETIS_SIM_HAS_SOPT6_RSTFLTEN                  /* SoC has SOPT6[RSTFLTEN] */
#  define KINETIS_SIM_HAS_SOPT7                       1   /* SoC has SOPT7 Register */
#  define KINETIS_SIM_HAS_SOPT7_ADC0ALTTRGSEL         1   /* SoC has SOPT7[ADC0ALTTRGSEL] */
#  define KINETIS_SIM_HAS_SOPT7_ADC1ALTTRGSEL         1   /* SoC has SOPT7[ADC1ALTTRGSEL] */
#  define KINETIS_SIM_HAS_SOPT7_ADC0PRETRGSEL         1   /* SoC has SOPT7[ADC0PRETRGSEL] */
#  define KINETIS_SIM_HAS_SOPT7_ADC1PRETRGSEL         1   /* SoC has SOPT7[ADC1PRETRGSEL] */
#  undef  KINETIS_SIM_HAS_SOPT7_ADC2PRETRGSEL             /* SoC has SOPT7[ADC2PRETRGSEL] */
#  undef  KINETIS_SIM_HAS_SOPT7_ADC3PRETRGSEL             /* SoC has SOPT7[ADC3PRETRGSEL] */
#  define KINETIS_SIM_HAS_SOPT7_ADC0TRGSEL            15  /* SoC has 10 SOPT7[ADC0TRGSEL] */
#  define KINETIS_SIM_HAS_SOPT7_ADC1TRGSEL            15  /* SoC has 10 SOPT7[ADC1TRGSEL] */
#  undef  KINETIS_SIM_HAS_SOPT7_ADC2TRGSEL                /* SoC has 10 SOPT7[ADC2TRGSEL] */
#  undef  KINETIS_SIM_HAS_SOPT7_ADC3TRGSEL                /* SoC has 10 SOPT7[ADC3TRGSEL] */
#  define KINETIS_SIM_SOPT7_ADC0ALTTRGEN              1   /* ADC0 alternate trigger enable */
#  define KINETIS_SIM_SOPT7_ADC1ALTTRGEN              1   /* ADC1 alternate trigger enable */
#  undef  KINETIS_SIM_SOPT7_ADC2ALTTRGEN                  /* ADC2 alternate trigger enable */
#  undef  KINETIS_SIM_SOPT7_ADC3ALTTRGEN                  /* ADC3 alternate trigger enable */
#  define KINETIS_SIM_HAS_SOPT8                       1   /* SoC has SOPT8 Register */
#  define KINETIS_SIM_HAS_SOPT8_FTM0SYNCBIT           1   /* SoC has SOPT8[FTM0SYNCBIT] */
#  define KINETIS_SIM_HAS_SOPT8_FTM1SYNCBIT           1   /* SoC has SOPT8[FTM1SYNCBIT] */
#  define KINETIS_SIM_HAS_SOPT8_FTM2SYNCBIT           1   /* SoC has SOPT8[FTM2SYNCBIT] */
#  define KINETIS_SIM_HAS_SOPT8_FTM3SYNCBIT           1   /* SoC has SOPT8[FTM3SYNCBIT] */
#  define KINETIS_SIM_HAS_SOPT8_FTM0OCH0SRC           1   /* SoC has SOPT8[FTM0OCH0SRC] */
#  define KINETIS_SIM_HAS_SOPT8_FTM0OCH1SRC           1   /* SoC has SOPT8[FTM0OCH1SRC] */
#  define KINETIS_SIM_HAS_SOPT8_FTM0OCH2SRC           1   /* SoC has SOPT8[FTM0OCH2SRC] */
#  define KINETIS_SIM_HAS_SOPT8_FTM0OCH3SRC           1   /* SoC has SOPT8[FTM0OCH3SRC] */
#  define KINETIS_SIM_HAS_SOPT8_FTM0OCH4SRC           1   /* SoC has SOPT8[FTM0OCH4SRC] */
#  define KINETIS_SIM_HAS_SOPT8_FTM0OCH5SRC           1   /* SoC has SOPT8[FTM0OCH5SRC] */
#  define KINETIS_SIM_HAS_SOPT8_FTM0OCH6SRC           1   /* SoC has SOPT8[FTM0OCH6SRC] */
#  define KINETIS_SIM_HAS_SOPT8_FTM0OCH7SRC           1   /* SoC has SOPT8[FTM0OCH7SRC] */
#  define KINETIS_SIM_HAS_SOPT8_FTM3OCH0SRC           1   /* SoC has SOPT8[FTM3OCH0SRC] */
#  define KINETIS_SIM_HAS_SOPT8_FTM3OCH1SRC           1   /* SoC has SOPT8[FTM3OCH1SRC] */
#  define KINETIS_SIM_HAS_SOPT8_FTM3OCH2SRC           1   /* SoC has SOPT8[FTM3OCH2SRC] */
#  define KINETIS_SIM_HAS_SOPT8_FTM3OCH3SRC           1   /* SoC has SOPT8[FTM3OCH3SRC] */
#  define KINETIS_SIM_HAS_SOPT8_FTM3OCH4SRC           1   /* SoC has SOPT8[FTM3OCH4SRC] */
#  define KINETIS_SIM_HAS_SOPT8_FTM3OCH5SRC           1   /* SoC has SOPT8[FTM3OCH5SRC] */
#  define KINETIS_SIM_HAS_SOPT8_FTM3OCH6SRC           1   /* SoC has SOPT8[FTM3OCH6SRC] */
#  define KINETIS_SIM_HAS_SOPT8_FTM3OCH7SRC           1   /* SoC has SOPT8[FTM3OCH7SRC] */
#  define KINETIS_SIM_HAS_SOPT9                       1   /* SoC has SOPT9 Register */
#  define KINETIS_SIM_HAS_SOPT9_TPM1CH0SRC            1   /* SoC has SOPT9[TPM1CH0SRC] */
#  define KINETIS_SIM_HAS_SOPT9_TPM2CH0SRC            1   /* SoC has SOPT9[TPM2CH0SRC] */
#  define KINETIS_SIM_HAS_SOPT9_TPM1CLKSEL            1   /* SoC has SOPT9[TPM1CLKSEL] */
#  define KINETIS_SIM_HAS_SOPT9_TPM2CLKSEL            1   /* SoC has SOPT9[TPM2CLKSEL] */
#  define KINETIS_SIM_HAS_SDID                        1   /* SoC has SDID Register */
#  define KINETIS_SIM_HAS_SDID_DIEID                  1   /* SoC has SDID[DIEID] */
#  define KINETIS_SIM_HAS_SDID_FAMID                  1   /* SoC has SDID[FAMID] */
#  define KINETIS_SIM_HAS_SDID_FAMILYID               1   /* SoC has SDID[FAMILYID] */
#  define KINETIS_SIM_HAS_SDID_SERIESID               1   /* SoC has SDID[SERIESID] */
#  undef  KINETIS_SIM_HAS_SDID_SRAMSIZE                   /* SoC has SDID[SRAMSIZE] */
#  define KINETIS_SIM_HAS_SDID_SUBFAMID               1   /* SoC has SDID[SUBFAMID] */
#  define KINETIS_SIM_HAS_SCGC1                       1   /* SoC has _SCGC1 Register */
#  undef  KINETIS_SIM_HAS_SCGC1_UART5                     /* SoC has SCGC1[UART5] */
#  define KINETIS_SIM_HAS_SCGC1_UART4                 1   /* SoC has SCGC1[UART4] */
#  define KINETIS_SIM_HAS_SCGC1_I2C3                  1   /* SoC has SCGC1[I2C3] */
#  define KINETIS_SIM_HAS_SCGC1_I2C2                  1   /* SoC has SCGC1[I2C2] */
#  undef  KINETIS_SIM_HAS_SCGC1_OSC1                      /* SoC has SCGC1[OSC1] */
#  define KINETIS_SIM_HAS_SCGC2                       1   /* SoC has _SCGC2 Register */
#  define KINETIS_SIM_HAS_SCGC2                       1   /* SoC has SCGC2 Register */
#  define KINETIS_SIM_HAS_SCGC2_ENET                  1   /* SoC has SCGC2[ENET] */
#  define KINETIS_SIM_HAS_SCGC2_LPUART0               1   /* SoC has SCGC2[LPUART0] */
#  define KINETIS_SIM_HAS_SCGC2_TPM1                  1   /* SoC has SCGC2[TPM1] */
#  define KINETIS_SIM_HAS_SCGC2_TPM2                  1   /* SoC has SCGC2[TPM2] */
#  define KINETIS_SIM_HAS_SCGC3                       1   /* SoC has SCGC3 Register */
#  define KINETIS_SIM_HAS_SCGC3_RNGA                  1   /* SoC has SCGC3[RNGA] */
#  define KINETIS_SIM_HAS_SCGC3_USBHS                 1   /* SoC has SCGC3[USBHS] */
#  define KINETIS_SIM_HAS_SCGC3_USBHSPHY              1   /* SoC has SCGC3[USBHSPHY] */
#  define KINETIS_SIM_HAS_SCGC3_USBHSDCD              1   /* SoC has SCGC3[USBHSDCD] */
#  define KINETIS_SIM_HAS_SCGC3_FLEXCAN1              1   /* SoC has SCGC3[FLEXCAN1] */
#  undef  KINETIS_SIM_HAS_SCGC3_NFC                       /* SoC has SCGC3[NFC] */
#  define KINETIS_SIM_HAS_SCGC3_SPI2                  1   /* SoC has SCGC3[SPI2] */
#  undef  KINETIS_SIM_HAS_SCGC3_SAI1                      /* SoC has SCGC3[SAI1] */
#  define KINETIS_SIM_HAS_SCGC3_SDHC                  1   /* SoC has SCGC3[SDHC] */
#  define KINETIS_SIM_HAS_SCGC3_FTM2                  1   /* SoC has SCGC3[FTM2] */
#  define KINETIS_SIM_HAS_SCGC3_FTM3                  1   /* SoC has SCGC3[FTM3] */
#  define KINETIS_SIM_HAS_SCGC3_ADC1                  1   /* SoC has SCGC3[ADC1] */
#  undef  KINETIS_SIM_HAS_SCGC3_ADC3                      /* SoC has SCGC3[ADC3] */
#  undef  KINETIS_SIM_HAS_SCGC3_SLCD                      /* SoC has SCGC3[SLCD] */
#  define KINETIS_SIM_HAS_SCGC4                       1   /* SoC has SCGC4 Register */
#  undef  KINETIS_SIM_HAS_SCGC4_LLWU                      /* SoC has SCGC4[LLWU] clock gate */
#  define KINETIS_SIM_HAS_SCGC4_UART0                 1   /* SoC has SCGC4[UART0] */
#  define KINETIS_SIM_HAS_SCGC4_UART1                 1   /* SoC has SCGC4[UART1] */
#  define KINETIS_SIM_HAS_SCGC4_UART2                 1   /* SoC has SCGC4[UART2] */
#  define KINETIS_SIM_HAS_SCGC4_UART3                 1   /* SoC has SCGC4[UART3] */
#  define KINETIS_SIM_HAS_SCGC5                       1   /* SoC has _SCGC5 Register */
#  undef  KINETIS_SIM_HAS_SCGC5_REGFILE                   /* SoC has SCGC5[REGFILE] */
#  define KINETIS_SIM_HAS_SCGC5_TSI                   1   /* SoC has SCGC5[TSI] */
#  undef  KINETIS_SIM_HAS_SCGC5_PORTF                     /* SoC has SCGC5[PORTF] */
#  define KINETIS_SIM_HAS_SCGC6                       1   /* SoC has SCGC6 Register */
#  define KINETIS_SIM_HAS_SCGC6_FTFL                  1   /* SoC has SCGC6[FTFL] */
#  undef  KINETIS_SIM_HAS_SCGC6_DMAMUX1                   /* SoC has SCGC6[DEMUX1] */
#  undef  KINETIS_SIM_HAS_SCGC6_USBHS                     /* SoC has SCGC6[USBHS] */
#  define KINETIS_SIM_HAS_SCGC6_RNGA                  1   /* SoC has SCGC6[RNGA] */
#  define KINETIS_SIM_HAS_SCGC6_FTM2                  1   /* SoC has SCGC6[FTM2] */
#  undef  KINETIS_SIM_HAS_SCGC6_ADC2                      /* SoC has SCGC6[ADC2] */
#  define KINETIS_SIM_HAS_SCGC6_DAC0                  1   /* SoC has SCGC6[DAC0] */
#  define KINETIS_SIM_HAS_SCGC7                       1   /* SoC has SCGC7 Register */
#  define KINETIS_SIM_HAS_SCGC7_FLEXBUS               1   /* SoC has SCGC7[FLEXBUS] */
#  define KINETIS_SIM_HAS_SCGC7_DMA                   1   /* SoC has SCGC7[DMS] */
#  define KINETIS_SIM_HAS_SCGC7_MPU                   1   /* SoC has SCGC7[MPU] */
#  define KINETIS_SIM_HAS_SCGC7_SDRAMC                1   /* SoC has SCGC7[SDRAMC] */
#  define KINETIS_SIM_HAS_CLKDIV1                     1   /* SoC has CLKDIV1 Register */
#  define KINETIS_SIM_HAS_CLKDIV1_OUTDIV2             1   /* SoC has CLKDIV1[OUTDIV2] */
#  define KINETIS_SIM_HAS_CLKDIV1_OUTDIV3             1   /* SoC has CLKDIV1[OUTDIV3] */
#  define KINETIS_SIM_HAS_CLKDIV1_OUTDIV4             1   /* SoC has CLKDIV1[OUTDIV4] */
#  undef  KINETIS_SIM_HAS_CLKDIV1_OUTDIV5                 /* SoC has CLKDIV1[OUTDIV5] */
#  define KINETIS_SIM_HAS_CLKDIV2                     1   /* SoC has CLKDIV2 Register */
#  define KINETIS_SIM_HAS_CLKDIV2_USBDIV              1   /* SoC has CLKDIV2[USBDIV] */
#  define KINETIS_SIM_HAS_CLKDIV2_USBFRAC             1   /* SoC has CLKDIV2[USBFRAC] */
#  undef  KINETIS_SIM_HAS_CLKDIV2_USBFSDIV                /* SoC has CLKDIV2[USBFSDIV] */
#  undef  KINETIS_SIM_HAS_CLKDIV2_USBFSFRAC               /* SoC has CLKDIV2[USBFSFRAC] */
#  undef  KINETIS_SIM_HAS_CLKDIV2_USBHSDIV                /* SoC has CLKDIV2[USBHSDIV] */
#  undef  KINETIS_SIM_HAS_CLKDIV2_USBHSFRAC               /* SoC has CLKDIV2[USBHSFRAC] */
#  undef  KINETIS_SIM_HAS_CLKDIV2_I2SDIV                  /* SoC has CLKDIV2[I2SDIV] */
#  undef  KINETIS_SIM_HAS_CLKDIV2_I2SFRAC                 /* SoC has CLKDIV2[I2SFRAC] */
#  define KINETIS_SIM_HAS_FCFG1                       1   /* SoC has FCFG1 Register */
#  define KINETIS_SIM_HAS_FCFG1_DEPART                1   /* SoC has FCFG1[DEPART] */
#  define KINETIS_SIM_HAS_FCFG1_EESIZE                1   /* SoC has FCFG1[EESIZE] */
#  define KINETIS_SIM_HAS_FCFG1_FLASHDIS              1   /* SoC has FCFG1[FLASHDIS] */
#  define KINETIS_SIM_HAS_FCFG1_FLASHDOZE             1   /* SoC has FCFG1[FLASHDOZE] */
#  undef  KINETIS_SIM_HAS_FCFG1_FTFDIS                    /* SoC has FCFG1[FTFDIS] */
#  define KINETIS_SIM_HAS_FCFG1_NVMSIZE               1   /* SoC has FCFG1[NVMSIZE] */
#  define KINETIS_SIM_HAS_FCFG2                       1   /* SoC has FCFG2 Register */
#  define KINETIS_SIM_HAS_FCFG2_MAXADDR0              7   /* SoC has n bit of FCFG2[MAXADDR0] */
#  define KINETIS_SIM_HAS_FCFG2_MAXADDR1              7   /* SoC has n bit of FCFG2[MAXADDR1] */
#  define KINETIS_SIM_HAS_FCFG2_PFLSH                 1   /* SoC has FCFG2[PFLSH] */
#  define KINETIS_SIM_HAS_FCFG2_SWAPPFLSH             1   /* SoC has FCFG2[SWAPPFLSH] */
#  define KINETIS_SIM_HAS_UIDH                        1   /* SoC has UIDH Register */
#  define KINETIS_SIM_HAS_UIDMH                       1   /* SoC has UIDMH Register */
#  define KINETIS_SIM_HAS_UIDML                       1   /* SoC has UIDML Register */
#  define KINETIS_SIM_HAS_UIDL                        1   /* SoC has UIDL Register */
#  define KINETIS_SIM_HAS_CLKDIV3                     1   /* SoC has CLKDIV3 Register */
#  define KINETIS_SIM_HAS_CLKDIV3_PLLFLLDIV           1   /* SoC has CLKDIV3[PLLFLLDIV] */
#  define KINETIS_SIM_HAS_CLKDIV3_PLLFLLFRAC          1   /* SoC has CLKDIV3[PLLFLLFRAC] */
#  define KINETIS_SIM_HAS_CLKDIV4                     1   /* SoC has CLKDIV4 Register */
#  define KINETIS_SIM_HAS_CLKDIV4_TRACEDIV            1   /* SoC has CLKDIV4[TRACEDIV] */
#  define KINETIS_SIM_HAS_CLKDIV4_TRACEFRAC           1   /* SoC has CLKDIV4[TRACEFRAC] */
#  undef  KINETIS_SIM_HAS_CLKDIV4_NFCDIV                  /* SoC has CLKDIV4[NFCDIV] */
#  undef  KINETIS_SIM_HAS_CLKDIV4_NFCFRAC                 /* SoC has CLKDIV4[NFCFRAC] */
#  undef  KINETIS_SIM_HAS_MCR                             /* SoC has MCR Register */
#else
#  error "Unsupported Kinetis chip"
#endif

/* Use the catch all configuration for the SIM based on the implementations in nuttx prior 2/16/2017 */

#if KINETIS_SIM_VERSION == KINETIS_SIM_VERSION_UKN

/* SIM Register Configuration */

#  define KINETIS_SIM_HAS_SOPT1                       1   /* SoC has SOPT1 Register */
#  undef  KINETIS_SIM_HAS_SOPT1_OSC32KOUT                 /* SoC has SOPT1[OSC32KOUT] */
#  define KINETIS_SIM_HAS_SOPT1_OSC32KSEL             1   /* SoC has SOPT1[OSC32KSEL] */
#  define KINETIS_SIM_HAS_SOPT1_OSC32KSEL_BITS        1   /* SoC has 1 bit SOPT1[OSC32KSEL] */
#  define KINETIS_SIM_HAS_SOPT1_RAMSIZE               1   /* SoC has SOPT1[RAMSIZE] */
#  define KINETIS_SIM_HAS_SOPT1_USBREGEN              1   /* SoC has SOPT1[USBREGEN] */
#  define KINETIS_SIM_HAS_SOPT1_USBSSTBY              1   /* SoC has SOPT1[USBSSTBY] */
#  undef  KINETIS_SIM_HAS_SOPT1_USBVSTBY                  /* SoC has SOPT1[USBVSTBY] */
#  undef  KINETIS_SIM_HAS_SOPT1CFG                        /* SoC has SOPT1CFG Register */
#  undef  KINETIS_SIM_HAS_SOPT1CFG_URWE                   /* SoC has SOPT1CFG[URWE] */
#  undef  KINETIS_SIM_HAS_SOPT1CFG_USSWE                  /* SoC has SOPT1CFG[USSWE] */
#  undef  KINETIS_SIM_HAS_SOPT1CFG_UVSWE                  /* SoC has SOPT1CFG[UVSWE] */
#  undef  KINETIS_SIM_HAS_USBPHYCTL                       /* SoC has USBPHYCTL Register */
#  undef  KINETIS_SIM_HAS_USBPHYCTL_USB3VOUTTRG           /* SoC has USBPHYCTL[USB3VOUTTRG] */
#  undef  KINETIS_SIM_HAS_USBPHYCTL_USBDISILIM            /* SoC has USBPHYCTL[USBDISILIM] */
#  undef  KINETIS_SIM_HAS_USBPHYCTL_USBVREGPD             /* SoC has USBPHYCTL[USBVREGPD] */
#  undef  KINETIS_SIM_HAS_USBPHYCTL_USBVREGSEL            /* SoC has USBPHYCTL[USBVREGSEL] */
#  define KINETIS_SIM_HAS_SOPT2                       1   /* SoC has SOPT2 Register */
#  define KINETIS_SIM_HAS_SOPT2_CMTUARTPAD            1   /* SoC has SOPT2[CMTUARTPAD] */
#  define KINETIS_SIM_HAS_SOPT2_FBSL                  1   /* SoC has SOPT2[FBSL] */
#  undef  KINETIS_SIM_HAS_SOPT2_FLEXIOSRC                 /* SoC has SOPT2[FLEXIOSRC] */
#  undef  KINETIS_SIM_HAS_SOPT2_LPUARTSRC                 /* SoC has SOPT2[LPUARTSRC] */
#  define KINETIS_SIM_HAS_SOPT2_PLLFLLSEL             1   /* SoC has SOPT2[PLLFLLSEL] */
#  define KINETIS_SIM_HAS_SOPT2_PLLFLLSEL_BITS        1   /* SoC has 1 bit of SOPT2[PLLFLLSEL] */
#  undef  KINETIS_SIM_HAS_SOPT2_PTD7PAD                   /* SoC has SOPT2[PTD7PAD] */
#  undef  KINETIS_SIM_HAS_SOPT2_RMIISRC                   /* SoC has SOPT2[RMIISRC] */
#  undef  KINETIS_SIM_HAS_SOPT2_RTCCLKOUTSEL              /* SoC has SOPT2[RTCCLKOUTSEL] */
#  undef  KINETIS_SIM_HAS_SOPT2_CLKOUTSEL                 /* SoC has SOPT2[CLKOUTSEL] */
#  define KINETIS_SIM_HAS_SOPT2_SDHCSRC               1   /* SoC has SOPT2[SDHCSRC] */
#  define KINETIS_SIM_HAS_SOPT2_TIMESRC               1   /* SoC has SOPT2[TIMESRC] */
#  undef  KINETIS_SIM_HAS_SOPT2_TPMSRC                    /* SoC has SOPT2[TPMSRC] */
#  undef  KINETIS_SIM_HAS_SOPT2_USBFSRC                   /* SoC has SOPT2[USBFSRC] */
#  define KINETIS_SIM_HAS_SOPT2_I2SSRC                1   /* SoC has SOPT2[I2SSRC] */
#  define KINETIS_SIM_HAS_SOPT2_TRACECLKSEL           1   /* SoC has SOPT2[TRACECLKSEL] */
#  undef  KINETIS_SIM_HAS_SOPT2_USBREGEN                  /* SoC has SOPT2[USBREGEN] */
#  undef  KINETIS_SIM_HAS_SOPT2_USBSLSRC                  /* SoC has SOPT2[USBSLSRC] */
#  undef  KINETIS_SIM_HAS_SOPT2_USBHSRC                   /* SoC has SOPT2[USBHSRC] */
#  define KINETIS_SIM_HAS_SOPT2_USBSRC                1   /* SoC has SOPT2[USBSRC] */
#  define KINETIS_SIM_HAS_SOPT2_MCGCLKSEL             1   /* SoC has SOPT2[MCGCLKSEL] */
#  define KINETIS_SIM_HAS_SOPT4                       1   /* SoC has SOPT4 Register */
#  define KINETIS_SIM_HAS_SOPT4_FTM0FLT0              1   /* SoC has SOPT4[FTM0FLT0] */
#  define KINETIS_SIM_HAS_SOPT4_FTM0FLT1              1   /* SoC has SOPT4[FTM0FLT1] */
#  define KINETIS_SIM_HAS_SOPT4_FTM0FLT2              1   /* SoC has SOPT4[FTM0FLT2] */
#  undef  KINETIS_SIM_HAS_SOPT4_FTM0FLT3                  /* SoC has SOPT4[FTM0FLT3] */
#  undef  KINETIS_SIM_HAS_SOPT4_FTM0TRG0SRC               /* SoC has SOPT4[FTM0TRG0SRC] */
#  undef  KINETIS_SIM_HAS_SOPT4_FTM0TRG1SRC               /* SoC has SOPT4[FTM0TRG1SRC] */
#  define KINETIS_SIM_HAS_SOPT4_FTM1CH0SRC            1   /* SoC has SOPT4[FTM1CH0SRC] No OF */
#  undef  KINETIS_SIM_HAS_SOPT4_FTM1FLT0                  /* SoC has SOPT4[FTM1FLT0] */
#  undef  KINETIS_SIM_HAS_SOPT4_FTM1FLT1                  /* SoC has SOPT4[FTM1FLT1] */
#  undef  KINETIS_SIM_HAS_SOPT4_FTM1FLT2                  /* SoC has SOPT4[FTM1FLT2] */
#  undef  KINETIS_SIM_HAS_SOPT4_FTM1FLT3                  /* SoC has SOPT4[FTM1FLT3] */
#  define KINETIS_SIM_HAS_SOPT4_FTM2CH0SRC            1   /* SoC has SOPT4[FTM2CH0SRC] */
#  undef  KINETIS_SIM_HAS_SOPT4_FTM2CH1SRC                /* SoC has SOPT4[FTM2CH1SRC] */
#  undef  KINETIS_SIM_HAS_SOPT4_FTM2FLT0                  /* SoC has SOPT4[FTM2FLT0] */
#  undef  KINETIS_SIM_HAS_SOPT4_FTM2FLT1                  /* SoC has SOPT4[FTM2FLT1] */
#  undef  KINETIS_SIM_HAS_SOPT4_FTM2FLT2                  /* SoC has SOPT4[FTM2FLT2] */
#  undef  KINETIS_SIM_HAS_SOPT4_FTM2FLT3                  /* SoC has SOPT4[FTM2FLT3] */
#  undef  KINETIS_SIM_HAS_SOPT4_FTM3CH0SRC                /* SoC has SOPT4[FTM3CH0SRC] */
#  undef  KINETIS_SIM_HAS_SOPT4_FTM3FLT0                  /* SoC has SOPT4[FTM3FLT0] */
#  undef  KINETIS_SIM_HAS_SOPT4_FTM3FLT1                  /* SoC has SOPT4[FTM3FLT1] */
#  undef  KINETIS_SIM_HAS_SOPT4_FTM3FLT2                  /* SoC has SOPT4[FTM3FLT2] */
#  undef  KINETIS_SIM_HAS_SOPT4_FTM3FLT3                  /* SoC has SOPT4[FTM3FLT3] */
#  undef  KINETIS_SIM_HAS_SOPT4_FTM3TRG0SRC               /* SoC has SOPT4[FTM3TRG0SRC] */
#  undef  KINETIS_SIM_HAS_SOPT4_FTM3TRG1SRC               /* SoC has SOPT4[FTM3TRG1SRC] */
#  undef  KINETIS_SIM_HAS_SOPT4_TPM0CLKSEL                /* SoC has SOPT4[TPM0CLKSEL] */
#  undef  KINETIS_SIM_HAS_SOPT4_TPM1CH0SRC                /* SoC has SOPT4[TPM1CH0SRC] */
#  undef  KINETIS_SIM_HAS_SOPT4_TPM1CLKSEL                /* SoC has SOPT4[TPM1CLKSEL] */
#  undef  KINETIS_SIM_HAS_SOPT4_TPM2CH0SRC                /* SoC has SOPT4[TPM2CH0SRC] */
#  undef  KINETIS_SIM_HAS_SOPT4_TPM2CLKSEL                /* SoC has SOPT4[TPM2CLKSEL] */
#  define KINETIS_SIM_HAS_SOPT5                           /* SoC has SOPT5 Register */
#  undef  KINETIS_SIM_HAS_SOPT5_LPUART0RXSRC              /* SoC has SOPT5[LPUART0RXSRC] */
#  undef  KINETIS_SIM_HAS_SOPT5_LPUART0TXSRC              /* SoC has SOPT5[LPUART0TXSRC] */
#  define KINETIS_SIM_HAS_SOPT6                       1   /* SoC has SOPT6 Register */
#  undef  KINETIS_SIM_HAS_SOPT6_MCC                       /* SoC has SOPT6[MCC] */
#  undef  KINETIS_SIM_HAS_SOPT6_PCR                       /* SoC has SOPT6[PCR] */
#  define KINETIS_SIM_HAS_SOPT6_RSTFLTSEL             1   /* SoC has SOPT6[RSTFLTSEL] */
#  define KINETIS_SIM_HAS_SOPT6_RSTFLTEN              1   /* SoC has SOPT6[RSTFLTEN] */
#  define KINETIS_SIM_HAS_SOPT7                       1   /* SoC has SOPT7 Register */
#  define KINETIS_SIM_HAS_SOPT7_ADC0ALTTRGSEL         1   /* SoC has SOPT7[ADC0ALTTRGSEL] */
#  define KINETIS_SIM_HAS_SOPT7_ADC1ALTTRGSEL         1   /* SoC has SOPT7[ADC1ALTTRGSEL] */
#  define KINETIS_SIM_HAS_SOPT7_ADC0PRETRGSEL         1   /* SoC has SOPT7[ADC0PRETRGSEL] */
#  define KINETIS_SIM_HAS_SOPT7_ADC1PRETRGSEL         1   /* SoC has SOPT7[ADC1PRETRGSEL] */
#  undef  KINETIS_SIM_HAS_SOPT7_ADC2PRETRGSEL             /* SoC has SOPT7[ADC2PRETRGSEL] */
#  undef  KINETIS_SIM_HAS_SOPT7_ADC3PRETRGSEL             /* SoC has SOPT7[ADC3PRETRGSEL] */
#  define KINETIS_SIM_HAS_SOPT7_ADC0TRGSEL            14  /* SoC has 10 SOPT7[ADC0TRGSEL] */
#  define KINETIS_SIM_HAS_SOPT7_ADC1TRGSEL            14  /* SoC has 10 SOPT7[ADC1TRGSEL] */
#  undef  KINETIS_SIM_HAS_SOPT7_ADC2TRGSEL                /* SoC has 10 SOPT7[ADC2TRGSEL] */
#  undef  KINETIS_SIM_HAS_SOPT7_ADC3TRGSEL                /* SoC has 10 SOPT7[ADC3TRGSEL] */
#  define KINETIS_SIM_SOPT7_ADC0ALTTRGEN              1   /* ADC0 alternate trigger enable */
#  define KINETIS_SIM_SOPT7_ADC1ALTTRGEN              1   /* ADC1 alternate trigger enable */
#  undef  KINETIS_SIM_SOPT7_ADC2ALTTRGEN                  /* ADC2 alternate trigger enable */
#  undef  KINETIS_SIM_SOPT7_ADC3ALTTRGEN                  /* ADC3 alternate trigger enable */
#  undef  KINETIS_SIM_HAS_SOPT8                           /* SoC has SOPT8 Register */
#  undef  KINETIS_SIM_HAS_SOPT9                           /* SoC has SOPT9 Register */
#  define KINETIS_SIM_HAS_SDID                        1   /* SoC has SDID Register */
#  undef  KINETIS_SIM_HAS_SDID_DIEID                      /* SoC has SDID[DIEID] */
#  define KINETIS_SIM_HAS_SDID_FAMID                  1   /* SoC has SDID[FAMID] */
#  undef  KINETIS_SIM_HAS_SDID_FAMILYID                   /* SoC has SDID[FAMILYID] */
#  undef  KINETIS_SIM_HAS_SDID_SERIESID                   /* SoC has SDID[SERIESID] */
#  undef  KINETIS_SIM_HAS_SDID_SRAMSIZE                   /* SoC has SDID[SRAMSIZE] */
#  undef  KINETIS_SIM_HAS_SDID_SUBFAMID                   /* SoC has SDID[SUBFAMID] */
#  define KINETIS_SIM_HAS_SCGC1                       1   /* SoC has SCGC1 Register */
#  define KINETIS_SIM_HAS_SCGC1_UART5                 1   /* SoC has SCGC1[UART5] */
#  define KINETIS_SIM_HAS_SCGC1_UART4                 1   /* SoC has SCGC1[UART4] */
#  undef  KINETIS_SIM_HAS_SCGC1_I2C3                      /* SoC has SCGC1[I2C3] */
#  undef  KINETIS_SIM_HAS_SCGC1_I2C2                      /* SoC has SCGC1[I2C2] */
#  undef  KINETIS_SIM_HAS_SCGC1_OSC1                      /* SoC has SCGC1[OSC1] */
#  define KINETIS_SIM_HAS_SCGC2                       1   /* SoC has SCGC2 Register */
#  define KINETIS_SIM_HAS_SCGC2_ENET                  1   /* SoC has SCGC2[ENET] */
#  undef  KINETIS_SIM_HAS_SCGC2_LPUART0                   /* SoC has SCGC2[LPUART0] */
#  undef  KINETIS_SIM_HAS_SCGC2_TPM1                      /* SoC has SCGC2[TPM1] */
#  undef  KINETIS_SIM_HAS_SCGC2_TPM2                      /* SoC has SCGC2[TPM2] */
#  define KINETIS_SIM_HAS_SCGC3                       1   /* SoC has SCGC3 Register */
#  define KINETIS_SIM_HAS_SCGC3_RNGA                  1   /* SoC has SCGC3[RNGA] */
#  undef  KINETIS_SIM_HAS_SCGC3_USBHS                     /* SoC has SCGC3[USBHS] */
#  undef  KINETIS_SIM_HAS_SCGC3_USBHSPHY                  /* SoC has SCGC3[USBHSPHY] */
#  undef  KINETIS_SIM_HAS_SCGC3_USBHSDCD                  /* SoC has SCGC3[USBHSDCD] */
#  define KINETIS_SIM_HAS_SCGC3_FLEXCAN1              1   /* SoC has SCGC3[FLEXCAN1] */
#  undef  KINETIS_SIM_HAS_SCGC3_NFC                       /* SoC has SCGC3[NFC] */
#  define KINETIS_SIM_HAS_SCGC3_SPI2                  1   /* SoC has SCGC3[SPI2] */
#  undef  KINETIS_SIM_HAS_SCGC3_SAI1                      /* SoC has SCGC3[SAI1] */
#  define KINETIS_SIM_HAS_SCGC3_SDHC                  1   /* SoC has SCGC3[SDHC] */
#  define KINETIS_SIM_HAS_SCGC3_FTM2                  1   /* SoC has SCGC3[FTM2] */
#  define KINETIS_SIM_HAS_SCGC3_FTM3                  1   /* SoC has SCGC3[FTM3] */
#  define KINETIS_SIM_HAS_SCGC3_ADC1                  1   /* SoC has SCGC3[ADC1] */
#  undef  KINETIS_SIM_HAS_SCGC3_ADC3                      /* SoC has SCGC3[ADC3] */
#  define KINETIS_SIM_HAS_SCGC3_SLCD                  1   /* SoC has SCGC3[SLCD] */
#  define KINETIS_SIM_HAS_SCGC4                       1   /* SoC has SCGC4 Register */
#  define KINETIS_SIM_HAS_SCGC4_LLWU                  1   /* SoC has SCGC4[LLWU] clock gate */
#  define KINETIS_SIM_HAS_SCGC4_UART0                 1   /* SoC has SCGC4[UART0] */
#  define KINETIS_SIM_HAS_SCGC4_UART1                 1   /* SoC has SCGC4[UART1] */
#  define KINETIS_SIM_HAS_SCGC4_UART2                 1   /* SoC has SCGC4[UART2] */
#  define KINETIS_SIM_HAS_SCGC4_UART3                 1   /* SoC has SCGC4[UART3] */
#  define KINETIS_SIM_HAS_SCGC5                       1   /* SoC has SCGC5 Register */
#  define KINETIS_SIM_HAS_SCGC5_REGFILE               1   /* SoC has SCGC5[REGFILE] */
#  define KINETIS_SIM_HAS_SCGC5_TSI                   1   /* SoC has SCGC5[TSI] */
#  undef  KINETIS_SIM_HAS_SCGC5_PORTF                     /* SoC has SCGC5[PORTF] */
#  define KINETIS_SIM_HAS_SCGC6                       1   /* SoC has SCGC6 Register */
#  define KINETIS_SIM_HAS_SCGC6_FTFL                  1   /* SoC has SCGC6[FTFL] */
#  undef  KINETIS_SIM_HAS_SCGC6_DMAMUX1                   /* SoC has SCGC6[DEMUX1] */
#  undef  KINETIS_SIM_HAS_SCGC6_USBHS                     /* SoC has SCGC6[USBHS] */
#  undef  KINETIS_SIM_HAS_SCGC6_RNGA                      /* SoC has SCGC6[RNGA] */
#  undef  KINETIS_SIM_HAS_SCGC6_FTM2                      /* SoC has SCGC6[FTM2] */
#  undef  KINETIS_SIM_HAS_SCGC6_ADC2                      /* SoC has SCGC6[ADC2] */
#  undef  KINETIS_SIM_HAS_SCGC6_DAC0                      /* SoC has SCGC6[DAC0] */
#  define KINETIS_SIM_HAS_SCGC7                       1   /* SoC has SCGC7 Register */
#  define KINETIS_SIM_HAS_SCGC7_FLEXBUS               1   /* SoC has SCGC7[FLEXBUS] */
#  define KINETIS_SIM_HAS_SCGC7_DMA                   1   /* SoC has SCGC7[DMS] */
#  define KINETIS_SIM_HAS_SCGC7_MPU                   1   /* SoC has SCGC7[MPU] */
#  undef  KINETIS_SIM_HAS_SCGC7_SDRAMC                    /* SoC has SCGC7[SDRAMC] */
#  define KINETIS_SIM_HAS_CLKDIV1                     1   /* SoC has CLKDIV1 Register */
#  define KINETIS_SIM_HAS_CLKDIV1_OUTDIV2             1   /* SoC has CLKDIV1[OUTDIV2] */
#  define KINETIS_SIM_HAS_CLKDIV1_OUTDIV3             1   /* SoC has CLKDIV1[OUTDIV3] */
#  define KINETIS_SIM_HAS_CLKDIV1_OUTDIV4             1   /* SoC has CLKDIV1[OUTDIV4] */
#  undef  KINETIS_SIM_HAS_CLKDIV1_OUTDIV5                 /* SoC has CLKDIV1[OUTDIV5] */
#  define KINETIS_SIM_HAS_CLKDIV2                     1   /* SoC has CLKDIV2 Register */
#  define KINETIS_SIM_HAS_CLKDIV2_USBDIV              1   /* SoC has CLKDIV2[USBDIV] */
#  define KINETIS_SIM_HAS_CLKDIV2_USBFRAC             1   /* SoC has CLKDIV2[USBFRAC] */
#  undef  KINETIS_SIM_HAS_CLKDIV2_USBFSDIV                /* SoC has CLKDIV2[USBFSDIV] */
#  undef  KINETIS_SIM_HAS_CLKDIV2_USBFSFRAC               /* SoC has CLKDIV2[USBFSFRAC] */
#  undef  KINETIS_SIM_HAS_CLKDIV2_USBHSDIV                /* SoC has CLKDIV2[USBHSDIV] */
#  undef  KINETIS_SIM_HAS_CLKDIV2_USBHSFRAC               /* SoC has CLKDIV2[USBHSFRAC] */
#  define KINETIS_SIM_HAS_CLKDIV2_I2SDIV              1   /* SoC has CLKDIV2[I2SDIV] */
#  define KINETIS_SIM_HAS_CLKDIV2_I2SFRAC             1   /* SoC has CLKDIV2[I2SFRAC] */
#  define KINETIS_SIM_HAS_FCFG1                       1   /* SoC has FCFG1 Register */
#  define KINETIS_SIM_HAS_FCFG1_DEPART                1   /* SoC has FCFG1[DEPART] */
#  define KINETIS_SIM_HAS_FCFG1_EESIZE                1   /* SoC has FCFG1[EESIZE] */
#  undef  KINETIS_SIM_HAS_FCFG1_FLASHDIS                  /* SoC has FCFG1[FLASHDIS] */
#  undef  KINETIS_SIM_HAS_FCFG1_FLASHDOZE                 /* SoC has FCFG1[FLASHDOZE] */
#  undef  KINETIS_SIM_HAS_FCFG1_FTFDIS                    /* SoC has FCFG1[FTFDIS] */
#  define KINETIS_SIM_HAS_FCFG1_NVMSIZE               1   /* SoC has FCFG1[NVMSIZE] */
#  define KINETIS_SIM_HAS_FCFG2                       1   /* SoC has FCFG2 Register */
#  define KINETIS_SIM_HAS_FCFG2_MAXADDR0              6   /* SoC has n bit of FCFG2[MAXADDR0] */
#  define KINETIS_SIM_HAS_FCFG2_MAXADDR1              6   /* SoC has n bit of FCFG2[MAXADDR1] */
#  define KINETIS_SIM_HAS_FCFG2_PFLSH                 1   /* SoC has FCFG2[PFLSH] */
#  define KINETIS_SIM_HAS_FCFG2_SWAPPFLSH             1   /* SoC has FCFG2[SWAPPFLSH] */
#  define KINETIS_SIM_HAS_UIDH                        1   /* SoC has UIDH Register */
#  define KINETIS_SIM_HAS_UIDMH                       1   /* SoC has UIDMH Register */
#  define KINETIS_SIM_HAS_UIDML                       1   /* SoC has UIDML Register */
#  define KINETIS_SIM_HAS_UIDL                        1   /* SoC has UIDL Register */
#  undef  KINETIS_SIM_HAS_CLKDIV3                         /* SoC has CLKDIV3 Register */
#  undef  KINETIS_SIM_HAS_CLKDIV3_PLLFLLDIV               /* SoC has CLKDIV3[PLLFLLDIV] */
#  undef  KINETIS_SIM_HAS_CLKDIV3_PLLFLLFRAC              /* SoC has CLKDIV3[PLLFLLFRAC] */
#  undef  KINETIS_SIM_HAS_CLKDIV4                         /* SoC has CLKDIV4 Register */
#  undef  KINETIS_SIM_HAS_CLKDIV4_TRACEDIV                /* SoC has CLKDIV4[TRACEDIV] */
#  undef  KINETIS_SIM_HAS_CLKDIV4_TRACEFRAC               /* SoC has CLKDIV4[TRACEFRAC] */
#  undef  KINETIS_SIM_HAS_CLKDIV4_NFCDIV                  /* SoC has CLKDIV4[NFCDIV] */
#  undef  KINETIS_SIM_HAS_CLKDIV4_NFCFRAC                 /* SoC has CLKDIV4[NFCFRAC] */
#  undef  KINETIS_SIM_HAS_MCR                             /* SoC has MCR Register */
#endif

#if !defined(KINETIS_SIM_VERSION)
#  error "No KINETIS_SIM_VERSION defined!"
#endif

#if defined(KINETIS_SIM_HAS_SOPT1_OSC32KSEL)
#  define KINETIS_SIM_SOPT1_OSC32KSEL_MASK  ((1 << (KINETIS_SIM_HAS_SOPT1_OSC32KSEL_BITS))-1)
#endif

#if defined(KINETIS_SIM_HAS_SOPT2_PLLFLLSEL)
#  define KINETIS_SIM_SOPT2_PLLFLLSEL_MASK  ((1 << (KINETIS_SIM_HAS_SOPT2_PLLFLLSEL_BITS))-1)
#endif

#if defined(KINETIS_SIM_HAS_FCFG2_MAXADDR0)
#  define KINETIS_SIM_FCFG2_MAXADDR0_MASK  ((1 << (KINETIS_SIM_HAS_FCFG2_MAXADDR0))-1)
#endif

#if defined(KINETIS_SIM_HAS_FCFG2_MAXADDR1)
#  define KINETIS_SIM_FCFG2_MAXADDR1_MASK  ((1 << (KINETIS_SIM_HAS_FCFG2_MAXADDR1))-1)
#endif

#endif /* __ARCH_ARM_INCLUDE_KINETIS_KINETIS_SIM_H */
