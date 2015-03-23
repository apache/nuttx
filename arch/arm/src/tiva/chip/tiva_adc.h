/************************************************************************************
 * arch/arm/src/tiva/chip/tiva_adc.h
 *
 *   Copyright (C) 2015 Calvin Maguranis. All rights reserved.
 *   Author: Calvin Maguranis <c.maguranis@gmail.com>
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

#ifndef __ARCH_ARM_SRC_TIVA_CHIP_TIVA_ADC_H
#define __ARCH_ARM_SRC_TIVA_CHIP_TIVA_ADC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* ADC register offsets *************************************************************/

#define TIVA_ADC_ACTSS_OFFSET    0x000   /* ADC Active Sample Sequencer */
#define TIVA_ADC_RIS_OFFSET      0x004   /* ADC Raw Interrupt Status */
#define TIVA_ADC_IM_OFFSET       0x008   /* ADC Interrupt Mask */
#define TIVA_ADC_ISC_OFFSET      0x00c   /* ADC Interrupt Status and Clear */
#define TIVA_ADC_OSTAT_OFFSET    0x010   /* ADC Overflow Status */
#define TIVA_ADC_EMUX_OFFSET     0x014   /* ADC Event Multiplexer Select */
#define TIVA_ADC_USTAT_OFFSET    0x018   /* ADC Underflow Status */
#define TIVA_ADC_TSSEL_OFFSET    0x01c   /* ADC Trigger Source Select */
#define TIVA_ADC_SSPRI_OFFSET    0x020   /* ADC Sample Sequencer Priority */
#define TIVA_ADC_SPC_OFFSET      0x024   /* ADC Sample Phase Control */
#define TIVA_ADC_PSSI_OFFSET     0x028   /* ADC Processor Sample Sequence Initiate */
#define TIVA_ADC_SAC_OFFSET      0x030   /* ADC Sample Averaging Control */
#define TIVA_ADC_DCISC_OFFSET    0x034   /* ADC Digital Comparator Interrupt Status and Clear */
#define TIVA_ADC_CTL_OFFSET      0x038   /* ADC Control */

#define TIVA_ADC_SS_BASE         0x040   /* ADC Sample Sequence base address */
#define TIVA_ADC_SSMUX_OFFSET    0x020   /* ADC Sample Sequence Input Multiplexer Select */
#define TIVA_ADC_SSCTL_OFFSET    0x004   /* ADC Sample Sequence Control */
#define TIVA_ADC_SSFIFO_OFFSET   0x008   /* ADC Sample Sequence Result FIFO */
#define TIVA_ADC_SSFSTAT_OFFSET  0x00c   /* ADC Sample Sequence FIFO Status */
#define TIVA_ADC_SSOP_OFFSET     0x010   /* ADC Sample Sequence Operation */
#define TIVA_ADC_SSDC_OFFSET     0x014   /* ADC Sample Sequence Digital Comparator Select */
#define TIVA_ADC_SSEMUX_OFFSET   0x018   /* ADC Sample Sequence Extended Input Multiplexer Select */
#define TIVA_ADC_SSTSH_OFFSET    0x01c   /* ADC Sample Sequence Sample and Hold Time */

#define TIVA_ADC_DCRIC_OFFSET    0xd00   /* ADC Digital Comparator Reset Initial Conditions */
#define TIVA_ADC_DCCTL0_OFFSET   0xe00   /* ADC Digital Comparator Control 0 */
#define TIVA_ADC_DCCTL1_OFFSET   0xe04   /* ADC Digital Comparator Control 1 */
#define TIVA_ADC_DCCTL2_OFFSET   0xe08   /* ADC Digital Comparator Control 2 */
#define TIVA_ADC_DCCTL3_OFFSET   0xe0c   /* ADC Digital Comparator Control 3 */
#define TIVA_ADC_DCCTL4_OFFSET   0xe10   /* ADC Digital Comparator Control 4 */
#define TIVA_ADC_DCCTL5_OFFSET   0xe14   /* ADC Digital Comparator Control 5 */
#define TIVA_ADC_DCCTL6_OFFSET   0xe18   /* ADC Digital Comparator Control 6 */
#define TIVA_ADC_DCCTL7_OFFSET   0xe1c   /* ADC Digital Comparator Control 7 */

#define TIVA_ADC_DCCMP0_OFFSET   0xe40   /* ADC Digital Comparator Range 0 */
#define TIVA_ADC_DCCMP1_OFFSET   0xe44   /* ADC Digital Comparator Range 1 */
#define TIVA_ADC_DCCMP2_OFFSET   0xe48   /* ADC Digital Comparator Range 2 */
#define TIVA_ADC_DCCMP3_OFFSET   0xe4c   /* ADC Digital Comparator Range 3 */
#define TIVA_ADC_DCCMP4_OFFSET   0xe50   /* ADC Digital Comparator Range 4 */
#define TIVA_ADC_DCCMP5_OFFSET   0xe54   /* ADC Digital Comparator Range 5 */
#define TIVA_ADC_DCCMP6_OFFSET   0xe58   /* ADC Digital Comparator Range 6 */
#define TIVA_ADC_DCCMP7_OFFSET   0xe5c   /* ADC Digital Comparator Range 7 */

#define TIVA_ADC_PP_OFFSET       0xfc0   /* ADC Peripheral Properties */
#define TIVA_ADC_PC_OFFSET       0xfc4   /* ADC Peripheral Configuration */
#define TIVA_ADC_CC_OFFSET       0xfc8   /* ADC Clock Configuration */

/* ADC register addresses ***********************************************************/

#define TIVA_ADC_BASE(n)         (TIVA_ADC0_BASE + (n)*0x01000)
#define TIVA_ADC_SS(n)           (TIVA_ADC_SS_BASE + ((n)*TIVA_ADC_SSMUX_OFFSET))

#define TIVA_ADC_ACTSS(n)        (TIVA_ADC_BASE(n)+TIVA_ADC_ACTSS_OFFSET) /* ADC Active Sample Sequencer */
#define TIVA_ADC_RIS(n)          (TIVA_ADC_BASE(n)+TIVA_ADC_RIS_OFFSET) /* ADC Raw Interrupt Status */
#define TIVA_ADC_IM(n)           (TIVA_ADC_BASE(n)+TIVA_ADC_IM_OFFSET) /* ADC Interrupt Mask */
#define TIVA_ADC_ISC(n)          (TIVA_ADC_BASE(n)+TIVA_ADC_ISC_OFFSET) /* ADC Interrupt Status and Clear */
#define TIVA_ADC_OSTAT(n)        (TIVA_ADC_BASE(n)+TIVA_ADC_OSTAT_OFFSET) /* ADC Overflow Status */
#define TIVA_ADC_EMUX(n)         (TIVA_ADC_BASE(n)+TIVA_ADC_EMUX_OFFSET) /* ADC Event Multiplexer Select */
#define TIVA_ADC_USTAT(n)        (TIVA_ADC_BASE(n)+TIVA_ADC_USTAT_OFFSET) /* ADC Underflow Status */
#define TIVA_ADC_TSSEL(n)        (TIVA_ADC_BASE(n)+TIVA_ADC_TSSEL_OFFSET) /* ADC Trigger Source Select */
#define TIVA_ADC_SSPRI(n)        (TIVA_ADC_BASE(n)+TIVA_ADC_SSPRI_OFFSET) /* ADC Sample Sequencer Priority */
#define TIVA_ADC_SPC(n)          (TIVA_ADC_BASE(n)+TIVA_ADC_SPC_OFFSET) /* ADC Sample Phase Control */
#define TIVA_ADC_PSSI(n)         (TIVA_ADC_BASE(n)+TIVA_ADC_PSSI_OFFSET) /* ADC Processor Sample Sequence Initiate */
#define TIVA_ADC_SAC(n)          (TIVA_ADC_BASE(n)+TIVA_ADC_SAC_OFFSET) /* ADC Sample Averaging Control */
#define TIVA_ADC_DCISC(n)        (TIVA_ADC_BASE(n)+TIVA_ADC_DCISC_OFFSET) /* ADC Digital Comparator Interrupt Status and Clear */
#define TIVA_ADC_CTL(n)          (TIVA_ADC_BASE(n)+TIVA_ADC_CTL_OFFSET) /* ADC Control */

#define TIVA_ADC_SSMUX(n)        TIVA_ADC_SS(n) /* ADC Sample Sequence Input Multiplexer Select */
#define TIVA_ADC_SSCTL(n)        (TIVA_ADC_SS(n)+TIVA_ADC_SSCTL_OFFSET) /* ADC Sample Sequence Control */
#define TIVA_ADC_SSFIFO(n)       (TIVA_ADC_SS(n)+TIVA_ADC_SSFIFO_OFFSET) /* ADC Sample Sequence Result FIFO */
#define TIVA_ADC_SSFSTAT(n)      (TIVA_ADC_SS(n)+TIVA_ADC_SSFSTAT_OFFSET) /* ADC Sample Sequence FIFO Status */
#define TIVA_ADC_SSOP(n)         (TIVA_ADC_SS(n)+TIVA_ADC_SSOP_OFFSET) /* ADC Sample Sequence Operation */
#define TIVA_ADC_SSDC(n)         (TIVA_ADC_SS(n)+TIVA_ADC_SSDC_OFFSET) /* ADC Sample Sequence Digital Comparator Select */
#define TIVA_ADC_SSEMUX(n)       (TIVA_ADC_SS(n)+TIVA_ADC_SSEMUX_OFFSET) /* ADC Sample Sequence Extended Input Multiplexer Select */
#define TIVA_ADC_SSTSH(n)        (TIVA_ADC_SS(n)+TIVA_ADC_SSTSH_OFFSET) /* ADC Sample Sequence Sample and Hold Time */

#define TIVA_ADC_DCRIC(n)        (TIVA_ADC_BASE(n)+TIVA_ADC_DCRIC_OFFSET) /* ADC Digital Comparator Reset Initial Conditions */
#define TIVA_ADC_DCCTL0(n)       (TIVA_ADC_BASE(n)+TIVA_ADC_DCCTL0_OFFSET) /* ADC Digital Comparator Control 0 */
#define TIVA_ADC_DCCTL1(n)       (TIVA_ADC_BASE(n)+TIVA_ADC_DCCTL1_OFFSET) /* ADC Digital Comparator Control 1 */
#define TIVA_ADC_DCCTL2(n)       (TIVA_ADC_BASE(n)+TIVA_ADC_DCCTL2_OFFSET) /* ADC Digital Comparator Control 2 */
#define TIVA_ADC_DCCTL3(n)       (TIVA_ADC_BASE(n)+TIVA_ADC_DCCTL3_OFFSET) /* ADC Digital Comparator Control 3 */
#define TIVA_ADC_DCCTL4(n)       (TIVA_ADC_BASE(n)+TIVA_ADC_DCCTL4_OFFSET) /* ADC Digital Comparator Control 4 */
#define TIVA_ADC_DCCTL5(n)       (TIVA_ADC_BASE(n)+TIVA_ADC_DCCTL5_OFFSET) /* ADC Digital Comparator Control 5 */
#define TIVA_ADC_DCCTL6(n)       (TIVA_ADC_BASE(n)+TIVA_ADC_DCCTL6_OFFSET) /* ADC Digital Comparator Control 6 */
#define TIVA_ADC_DCCTL7(n)       (TIVA_ADC_BASE(n)+TIVA_ADC_DCCTL7_OFFSET) /* ADC Digital Comparator Control 7 */

#define TIVA_ADC_DCCMP0(n)       (TIVA_ADC_BASE(n)+TIVA_ADC_DCCMP0_OFFSET) /* ADC Digital Comparator Range 0 */
#define TIVA_ADC_DCCMP1(n)       (TIVA_ADC_BASE(n)+TIVA_ADC_DCCMP1_OFFSET) /* ADC Digital Comparator Range 1 */
#define TIVA_ADC_DCCMP2(n)       (TIVA_ADC_BASE(n)+TIVA_ADC_DCCMP2_OFFSET) /* ADC Digital Comparator Range 2 */
#define TIVA_ADC_DCCMP3(n)       (TIVA_ADC_BASE(n)+TIVA_ADC_DCCMP3_OFFSET) /* ADC Digital Comparator Range 3 */
#define TIVA_ADC_DCCMP4(n)       (TIVA_ADC_BASE(n)+TIVA_ADC_DCCMP4_OFFSET) /* ADC Digital Comparator Range 4 */
#define TIVA_ADC_DCCMP5(n)       (TIVA_ADC_BASE(n)+TIVA_ADC_DCCMP5_OFFSET) /* ADC Digital Comparator Range 5 */
#define TIVA_ADC_DCCMP6(n)       (TIVA_ADC_BASE(n)+TIVA_ADC_DCCMP6_OFFSET) /* ADC Digital Comparator Range 6 */
#define TIVA_ADC_DCCMP7(n)       (TIVA_ADC_BASE(n)+TIVA_ADC_DCCMP7_OFFSET) /* ADC Digital Comparator Range 7 */

/* ADC register address expansion ***************************************************/

#define TIVA_ADC0_ACTSS          (TIVA_ADC_BASE(0) + TIVA_ADC_ACTSS_OFFSET) /* ADC Active Sample Sequencer */
#define TIVA_ADC0_RIS            (TIVA_ADC_BASE(0) + TIVA_ADC_RIS_OFFSET) /* ADC Raw Interrupt Status */
#define TIVA_ADC0_IM             (TIVA_ADC_BASE(0) + TIVA_ADC_IM_OFFSET) /* ADC Interrupt Mask */
#define TIVA_ADC0_ISC            (TIVA_ADC_BASE(0) + TIVA_ADC_ISC_OFFSET) /* ADC Interrupt Status and Clear */
#define TIVA_ADC0_OSTAT          (TIVA_ADC_BASE(0) + TIVA_ADC_OSTAT_OFFSET) /* ADC Overflow Status */
#define TIVA_ADC0_EMUX           (TIVA_ADC_BASE(0) + TIVA_ADC_EMUX_OFFSET) /* ADC Event Multiplexer Select */
#define TIVA_ADC0_USTAT          (TIVA_ADC_BASE(0) + TIVA_ADC_USTAT_OFFSET) /* ADC Underflow Status */
#define TIVA_ADC0_TSSEL          (TIVA_ADC_BASE(0) + TIVA_ADC_TSSEL_OFFSET) /* ADC Trigger Source Select */
#define TIVA_ADC0_SSPRI          (TIVA_ADC_BASE(0) + TIVA_ADC_SSPRI_OFFSET) /* ADC Sample Sequencer Priority */
#define TIVA_ADC0_SPC            (TIVA_ADC_BASE(0) + TIVA_ADC_SPC_OFFSET) /* ADC Sample Phase Control */
#define TIVA_ADC0_PSSI           (TIVA_ADC_BASE(0) + TIVA_ADC_PSSI_OFFSET) /* ADC Processor Sample Sequence Initiate */
#define TIVA_ADC0_SAC            (TIVA_ADC_BASE(0) + TIVA_ADC_SAC_OFFSET) /* ADC Sample Averaging Control */
#define TIVA_ADC0_DCISC          (TIVA_ADC_BASE(0) + TIVA_ADC_DCISC_OFFSET) /* ADC Digital Comparator Interrupt Status and Clear */
#define TIVA_ADC0_CTL            (TIVA_ADC_BASE(0) + TIVA_ADC_CTL_OFFSET) /* ADC Control */

#define TIVA_ADC0_SSMUX0         (TIVA_ADC_BASE(0) + TIVA_ADC_SSMUX(0)) /* ADC Sample Sequence Input Multiplexer Select 0 */
#define TIVA_ADC0_SSCTL0         (TIVA_ADC_BASE(0) + TIVA_ADC_SSCTL(0)) /* ADC Sample Sequence Control 0 */
#define TIVA_ADC0_SSFIFO0        (TIVA_ADC_BASE(0) + TIVA_ADC_SSFIFO(0)) /* ADC Sample Sequence Result FIFO 0 */
#define TIVA_ADC0_SSFSTAT0       (TIVA_ADC_BASE(0) + TIVA_ADC_SSFSTAT(0)) /* ADC Sample Sequence FIFO 0 Status */
#define TIVA_ADC0_SSOP0          (TIVA_ADC_BASE(0) + TIVA_ADC_SSOP(0)) /* ADC Sample Sequence 0 Operation */
#define TIVA_ADC0_SSDC0          (TIVA_ADC_BASE(0) + TIVA_ADC_SSDC(0)) /* ADC Sample Sequence 0 Digital Comparator Select */
#define TIVA_ADC0_SSEMUX0        (TIVA_ADC_BASE(0) + TIVA_ADC_SSEMUX(0)) /* ADC Sample Sequence Extended Input Multiplexer Select 0 */
#define TIVA_ADC0_SSTSH0         (TIVA_ADC_BASE(0) + TIVA_ADC_SSTSH(0)) /* ADC Sample Sequence 0 Sample and Hold Time */

#define TIVA_ADC0_SSMUX1         (TIVA_ADC_BASE(0) + TIVA_ADC_SSMUX(1)) /* ADC Sample Sequence Input Multiplexer Select 1 */
#define TIVA_ADC0_SSCTL1         (TIVA_ADC_BASE(0) + TIVA_ADC_SSCTL(1)) /* ADC Sample Sequence Control 1 */
#define TIVA_ADC0_SSFIFO1        (TIVA_ADC_BASE(0) + TIVA_ADC_SSFIFO(1)) /* ADC Sample Sequence Result FIFO 1 */
#define TIVA_ADC0_SSFSTAT1       (TIVA_ADC_BASE(0) + TIVA_ADC_SSFSTAT(1)) /* ADC Sample Sequence FIFO 1 Status */
#define TIVA_ADC0_SSOP1          (TIVA_ADC_BASE(0) + TIVA_ADC_SSOP(1)) /* ADC Sample Sequence 1 Operation */
#define TIVA_ADC0_SSDC1          (TIVA_ADC_BASE(0) + TIVA_ADC_SSDC(1)) /* ADC Sample Sequence 1 Digital Comparator Select */
#define TIVA_ADC0_SSEMUX1        (TIVA_ADC_BASE(0) + TIVA_ADC_SSEMUX(1)) /* ADC Sample Sequence Extended Input Multiplexer Select 1 */
#define TIVA_ADC0_SSTSH1         (TIVA_ADC_BASE(0) + TIVA_ADC_SSTSH(1)) /* ADC Sample Sequence 1 Sample and Hold Time */

#define TIVA_ADC0_SSMUX2         (TIVA_ADC_BASE(0) + TIVA_ADC_SSMUX(2)) /* ADC Sample Sequence Input Multiplexer Select 2 */
#define TIVA_ADC0_SSCTL2         (TIVA_ADC_BASE(0) + TIVA_ADC_SSCTL(2)) /* ADC Sample Sequence Control 2 */
#define TIVA_ADC0_SSFIFO2        (TIVA_ADC_BASE(0) + TIVA_ADC_SSFIFO(2)) /* ADC Sample Sequence Result FIFO 2 */
#define TIVA_ADC0_SSFSTAT2       (TIVA_ADC_BASE(0) + TIVA_ADC_SSFSTAT(2)) /* ADC Sample Sequence FIFO 2 Status */
#define TIVA_ADC0_SSOP2          (TIVA_ADC_BASE(0) + TIVA_ADC_SSOP(2)) /* ADC Sample Sequence 2 Operation */
#define TIVA_ADC0_SSDC2          (TIVA_ADC_BASE(0) + TIVA_ADC_SSDC(2)) /* ADC Sample Sequence 2 Digital Comparator Select */
#define TIVA_ADC0_SSEMUX2        (TIVA_ADC_BASE(0) + TIVA_ADC_SSEMUX(2)) /* ADC Sample Sequence Extended Input Multiplexer Select 2 */
#define TIVA_ADC0_SSTSH2         (TIVA_ADC_BASE(0) + TIVA_ADC_SSTSH(2)) /* ADC Sample Sequence 2 Sample and Hold Time */

#define TIVA_ADC0_SSMUX3         (TIVA_ADC_BASE(0) + TIVA_ADC_SSMUX(3)) /* ADC Sample Sequence Input Multiplexer Select 3 */
#define TIVA_ADC0_SSCTL3         (TIVA_ADC_BASE(0) + TIVA_ADC_SSCTL(3)) /* ADC Sample Sequence Control 3 */
#define TIVA_ADC0_SSFIFO3        (TIVA_ADC_BASE(0) + TIVA_ADC_SSFIFO(3)) /* ADC Sample Sequence Result FIFO 3 */
#define TIVA_ADC0_SSFSTAT3       (TIVA_ADC_BASE(0) + TIVA_ADC_SSFSTAT(3)) /* ADC Sample Sequence FIFO 3 Status */
#define TIVA_ADC0_SSOP3          (TIVA_ADC_BASE(0) + TIVA_ADC_SSOP(3)) /* ADC Sample Sequence 3 Operation */
#define TIVA_ADC0_SSDC3          (TIVA_ADC_BASE(0) + TIVA_ADC_SSDC(3)) /* ADC Sample Sequence 3 Digital Comparator Select */
#define TIVA_ADC0_SSEMUX3        (TIVA_ADC_BASE(0) + TIVA_ADC_SSEMUX(3)) /* ADC Sample Sequence Extended Input Multiplexer Select 3 */
#define TIVA_ADC0_SSTSH3         (TIVA_ADC_BASE(0) + TIVA_ADC_SSTSH(3)) /* ADC Sample Sequence 3 Sample and Hold Time */

#define TIVA_ADC0_DCRIC          (TIVA_ADC_BASE(0) + TIVA_ADC_DCRIC_OFFSET) /* ADC Digital Comparator Reset Initial Conditions */
#define TIVA_ADC0_DCCTL0         (TIVA_ADC_BASE(0) + TIVA_ADC_DCCTL0_OFFSET) /* ADC Digital Comparator Control 0 */
#define TIVA_ADC0_DCCTL1         (TIVA_ADC_BASE(0) + TIVA_ADC_DCCTL1_OFFSET) /* ADC Digital Comparator Control 1 */
#define TIVA_ADC0_DCCTL2         (TIVA_ADC_BASE(0) + TIVA_ADC_DCCTL2_OFFSET) /* ADC Digital Comparator Control 2 */
#define TIVA_ADC0_DCCTL3         (TIVA_ADC_BASE(0) + TIVA_ADC_DCCTL3_OFFSET) /* ADC Digital Comparator Control 3 */
#define TIVA_ADC0_DCCTL4         (TIVA_ADC_BASE(0) + TIVA_ADC_DCCTL4_OFFSET) /* ADC Digital Comparator Control 4 */
#define TIVA_ADC0_DCCTL5         (TIVA_ADC_BASE(0) + TIVA_ADC_DCCTL5_OFFSET) /* ADC Digital Comparator Control 5 */
#define TIVA_ADC0_DCCTL6         (TIVA_ADC_BASE(0) + TIVA_ADC_DCCTL6_OFFSET) /* ADC Digital Comparator Control 6 */
#define TIVA_ADC0_DCCTL7         (TIVA_ADC_BASE(0) + TIVA_ADC_DCCTL7_OFFSET) /* ADC Digital Comparator Control 7 */

#define TIVA_ADC0_DCCMP0         (TIVA_ADC_BASE(0) + TIVA_ADC_DCCMP0_OFFSET) /* ADC Digital Comparator Range 0 */
#define TIVA_ADC0_DCCMP1         (TIVA_ADC_BASE(0) + TIVA_ADC_DCCMP1_OFFSET) /* ADC Digital Comparator Range 1 */
#define TIVA_ADC0_DCCMP2         (TIVA_ADC_BASE(0) + TIVA_ADC_DCCMP2_OFFSET) /* ADC Digital Comparator Range 2 */
#define TIVA_ADC0_DCCMP3         (TIVA_ADC_BASE(0) + TIVA_ADC_DCCMP3_OFFSET) /* ADC Digital Comparator Range 3 */
#define TIVA_ADC0_DCCMP4         (TIVA_ADC_BASE(0) + TIVA_ADC_DCCMP4_OFFSET) /* ADC Digital Comparator Range 4 */
#define TIVA_ADC0_DCCMP5         (TIVA_ADC_BASE(0) + TIVA_ADC_DCCMP5_OFFSET) /* ADC Digital Comparator Range 5 */
#define TIVA_ADC0_DCCMP6         (TIVA_ADC_BASE(0) + TIVA_ADC_DCCMP6_OFFSET) /* ADC Digital Comparator Range 6 */
#define TIVA_ADC0_DCCMP7         (TIVA_ADC_BASE(0) + TIVA_ADC_DCCMP7_OFFSET) /* ADC Digital Comparator Range 7 */

#define TIVA_ADC1_ACTSS          (TIVA_ADC_BASE(1) + TIVA_ADC_ACTSS_OFFSET) /* ADC Active Sample Sequencer */
#define TIVA_ADC1_RIS            (TIVA_ADC_BASE(1) + TIVA_ADC_RIS_OFFSET) /* ADC Raw Interrupt Status */
#define TIVA_ADC1_IM             (TIVA_ADC_BASE(1) + TIVA_ADC_IM_OFFSET) /* ADC Interrupt Mask */
#define TIVA_ADC1_ISC            (TIVA_ADC_BASE(1) + TIVA_ADC_ISC_OFFSET) /* ADC Interrupt Status and Clear */
#define TIVA_ADC1_OSTAT          (TIVA_ADC_BASE(1) + TIVA_ADC_OSTAT_OFFSET) /* ADC Overflow Status */
#define TIVA_ADC1_EMUX           (TIVA_ADC_BASE(1) + TIVA_ADC_EMUX_OFFSET) /* ADC Event Multiplexer Select */
#define TIVA_ADC1_USTAT          (TIVA_ADC_BASE(1) + TIVA_ADC_USTAT_OFFSET) /* ADC Underflow Status */
#define TIVA_ADC1_TSSEL          (TIVA_ADC_BASE(1) + TIVA_ADC_TSSEL_OFFSET) /* ADC Trigger Source Select */
#define TIVA_ADC1_SSPRI          (TIVA_ADC_BASE(1) + TIVA_ADC_SSPRI_OFFSET) /* ADC Sample Sequencer Priority */
#define TIVA_ADC1_SPC            (TIVA_ADC_BASE(1) + TIVA_ADC_SPC_OFFSET) /* ADC Sample Phase Control */
#define TIVA_ADC1_PSSI           (TIVA_ADC_BASE(1) + TIVA_ADC_PSSI_OFFSET) /* ADC Processor Sample Sequence Initiate */
#define TIVA_ADC1_SAC            (TIVA_ADC_BASE(1) + TIVA_ADC_SAC_OFFSET) /* ADC Sample Averaging Control */
#define TIVA_ADC1_DCISC          (TIVA_ADC_BASE(1) + TIVA_ADC_DCISC_OFFSET) /* ADC Digital Comparator Interrupt Status and Clear */
#define TIVA_ADC1_CTL            (TIVA_ADC_BASE(1) + TIVA_ADC_CTL_OFFSET) /* ADC Control */

#define TIVA_ADC1_SSMUX0         (TIVA_ADC_BASE(1) + TIVA_ADC_SSMUX(0)) /* ADC Sample Sequence Input Multiplexer Select 0 */
#define TIVA_ADC1_SSCTL0         (TIVA_ADC_BASE(1) + TIVA_ADC_SSCTL(0)) /* ADC Sample Sequence Control 0 */
#define TIVA_ADC1_SSFIFO0        (TIVA_ADC_BASE(1) + TIVA_ADC_SSFIFO(0)) /* ADC Sample Sequence Result FIFO 0 */
#define TIVA_ADC1_SSFSTAT0       (TIVA_ADC_BASE(1) + TIVA_ADC_SSFSTAT(0)) /* ADC Sample Sequence FIFO 0 Status */
#define TIVA_ADC1_SSOP0          (TIVA_ADC_BASE(1) + TIVA_ADC_SSOP(0)) /* ADC Sample Sequence 0 Operation */
#define TIVA_ADC1_SSDC0          (TIVA_ADC_BASE(1) + TIVA_ADC_SSDC(0)) /* ADC Sample Sequence 0 Digital Comparator Select */
#define TIVA_ADC1_SSEMUX0        (TIVA_ADC_BASE(1) + TIVA_ADC_SSEMUX(0)) /* ADC Sample Sequence Extended Input Multiplexer Select 0 */
#define TIVA_ADC1_SSTSH0         (TIVA_ADC_BASE(1) + TIVA_ADC_SSTSH(0)) /* ADC Sample Sequence 0 Sample and Hold Time */

#define TIVA_ADC1_SSMUX1         (TIVA_ADC_BASE(1) + TIVA_ADC_SSMUX(1)) /* ADC Sample Sequence Input Multiplexer Select 1 */
#define TIVA_ADC1_SSCTL1         (TIVA_ADC_BASE(1) + TIVA_ADC_SSCTL(1)) /* ADC Sample Sequence Control 1 */
#define TIVA_ADC1_SSFIFO1        (TIVA_ADC_BASE(1) + TIVA_ADC_SSFIFO(1)) /* ADC Sample Sequence Result FIFO 1 */
#define TIVA_ADC1_SSFSTAT1       (TIVA_ADC_BASE(1) + TIVA_ADC_SSFSTAT(1)) /* ADC Sample Sequence FIFO 1 Status */
#define TIVA_ADC1_SSOP1          (TIVA_ADC_BASE(1) + TIVA_ADC_SSOP(1)) /* ADC Sample Sequence 1 Operation */
#define TIVA_ADC1_SSDC1          (TIVA_ADC_BASE(1) + TIVA_ADC_SSDC(1)) /* ADC Sample Sequence 1 Digital Comparator Select */
#define TIVA_ADC1_SSEMUX1        (TIVA_ADC_BASE(1) + TIVA_ADC_SSEMUX(1)) /* ADC Sample Sequence Extended Input Multiplexer Select 1 */
#define TIVA_ADC1_SSTSH1         (TIVA_ADC_BASE(1) + TIVA_ADC_SSTSH(1)) /* ADC Sample Sequence 1 Sample and Hold Time */

#define TIVA_ADC1_SSMUX2         (TIVA_ADC_BASE(1) + TIVA_ADC_SSMUX(2)) /* ADC Sample Sequence Input Multiplexer Select 2 */
#define TIVA_ADC1_SSCTL2         (TIVA_ADC_BASE(1) + TIVA_ADC_SSCTL(2)) /* ADC Sample Sequence Control 2 */
#define TIVA_ADC1_SSFIFO2        (TIVA_ADC_BASE(1) + TIVA_ADC_SSFIFO(2)) /* ADC Sample Sequence Result FIFO 2 */
#define TIVA_ADC1_SSFSTAT2       (TIVA_ADC_BASE(1) + TIVA_ADC_SSFSTAT(2)) /* ADC Sample Sequence FIFO 2 Status */
#define TIVA_ADC1_SSOP2          (TIVA_ADC_BASE(1) + TIVA_ADC_SSOP(2)) /* ADC Sample Sequence 2 Operation */
#define TIVA_ADC1_SSDC2          (TIVA_ADC_BASE(1) + TIVA_ADC_SSDC(2)) /* ADC Sample Sequence 2 Digital Comparator Select */
#define TIVA_ADC1_SSEMUX2        (TIVA_ADC_BASE(1) + TIVA_ADC_SSEMUX(2)) /* ADC Sample Sequence Extended Input Multiplexer Select 2 */
#define TIVA_ADC1_SSTSH2         (TIVA_ADC_BASE(1) + TIVA_ADC_SSTSH(2)) /* ADC Sample Sequence 2 Sample and Hold Time */

#define TIVA_ADC1_SSMUX3         (TIVA_ADC_BASE(1) + TIVA_ADC_SSMUX(3)) /* ADC Sample Sequence Input Multiplexer Select 3 */
#define TIVA_ADC1_SSCTL3         (TIVA_ADC_BASE(1) + TIVA_ADC_SSCTL(3)) /* ADC Sample Sequence Control 3 */
#define TIVA_ADC1_SSFIFO3        (TIVA_ADC_BASE(1) + TIVA_ADC_SSFIFO(3)) /* ADC Sample Sequence Result FIFO 3 */
#define TIVA_ADC1_SSFSTAT3       (TIVA_ADC_BASE(1) + TIVA_ADC_SSFSTAT(3)) /* ADC Sample Sequence FIFO 3 Status */
#define TIVA_ADC1_SSOP3          (TIVA_ADC_BASE(1) + TIVA_ADC_SSOP(3)) /* ADC Sample Sequence 3 Operation */
#define TIVA_ADC1_SSDC3          (TIVA_ADC_BASE(1) + TIVA_ADC_SSDC(3)) /* ADC Sample Sequence 3 Digital Comparator Select */
#define TIVA_ADC1_SSEMUX3        (TIVA_ADC_BASE(1) + TIVA_ADC_SSEMUX(3)) /* ADC Sample Sequence Extended Input Multiplexer Select 3 */
#define TIVA_ADC1_SSTSH3         (TIVA_ADC_BASE(1) + TIVA_ADC_SSTSH(3)) /* ADC Sample Sequence 3 Sample and Hold Time */

#define TIVA_ADC1_DCRIC          (TIVA_ADC_BASE(1) + TIVA_ADC_DCRIC_OFFSET) /* ADC Digital Comparator Reset Initial Conditions */
#define TIVA_ADC1_DCCTL0         (TIVA_ADC_BASE(1) + TIVA_ADC_DCCTL0_OFFSET) /* ADC Digital Comparator Control 0 */
#define TIVA_ADC1_DCCTL1         (TIVA_ADC_BASE(1) + TIVA_ADC_DCCTL1_OFFSET) /* ADC Digital Comparator Control 1 */
#define TIVA_ADC1_DCCTL2         (TIVA_ADC_BASE(1) + TIVA_ADC_DCCTL2_OFFSET) /* ADC Digital Comparator Control 2 */
#define TIVA_ADC1_DCCTL3         (TIVA_ADC_BASE(1) + TIVA_ADC_DCCTL3_OFFSET) /* ADC Digital Comparator Control 3 */
#define TIVA_ADC1_DCCTL4         (TIVA_ADC_BASE(1) + TIVA_ADC_DCCTL4_OFFSET) /* ADC Digital Comparator Control 4 */
#define TIVA_ADC1_DCCTL5         (TIVA_ADC_BASE(1) + TIVA_ADC_DCCTL5_OFFSET) /* ADC Digital Comparator Control 5 */
#define TIVA_ADC1_DCCTL6         (TIVA_ADC_BASE(1) + TIVA_ADC_DCCTL6_OFFSET) /* ADC Digital Comparator Control 6 */
#define TIVA_ADC1_DCCTL7         (TIVA_ADC_BASE(1) + TIVA_ADC_DCCTL7_OFFSET) /* ADC Digital Comparator Control 7 */

#define TIVA_ADC1_DCCMP0         (TIVA_ADC_BASE(1) + TIVA_ADC_DCCMP0_OFFSET) /* ADC Digital Comparator Range 0 */
#define TIVA_ADC1_DCCMP1         (TIVA_ADC_BASE(1) + TIVA_ADC_DCCMP1_OFFSET) /* ADC Digital Comparator Range 1 */
#define TIVA_ADC1_DCCMP2         (TIVA_ADC_BASE(1) + TIVA_ADC_DCCMP2_OFFSET) /* ADC Digital Comparator Range 2 */
#define TIVA_ADC1_DCCMP3         (TIVA_ADC_BASE(1) + TIVA_ADC_DCCMP3_OFFSET) /* ADC Digital Comparator Range 3 */
#define TIVA_ADC1_DCCMP4         (TIVA_ADC_BASE(1) + TIVA_ADC_DCCMP4_OFFSET) /* ADC Digital Comparator Range 4 */
#define TIVA_ADC1_DCCMP5         (TIVA_ADC_BASE(1) + TIVA_ADC_DCCMP5_OFFSET) /* ADC Digital Comparator Range 5 */
#define TIVA_ADC1_DCCMP6         (TIVA_ADC_BASE(1) + TIVA_ADC_DCCMP6_OFFSET) /* ADC Digital Comparator Range 6 */
#define TIVA_ADC1_DCCMP7         (TIVA_ADC_BASE(1) + TIVA_ADC_DCCMP7_OFFSET) /* ADC Digital Comparator Range 7 */

/* ADC register bit settings ********************************************************/

/* Bit fields in the TIVA_ADC_ACTSS register. */

#define ADC_ACTSS_BUSY           0x00010000               /* ADC Busy */
#define ADC_ACTSS_ADEN3          0x00000800               /* ADC SS3 DMA Enable */
#define ADC_ACTSS_ADEN2          0x00000400               /* ADC SS2 DMA Enable */
#define ADC_ACTSS_ADEN1          0x00000200               /* ADC SS1 DMA Enable */
#define ADC_ACTSS_ADEN0          0x00000100               /* ADC SS1 DMA Enable */
#define ADC_ACTSS_ASEN3          0x00000008               /* ADC SS3 Enable */
#define ADC_ACTSS_ASEN2          0x00000004               /* ADC SS2 Enable */
#define ADC_ACTSS_ASEN1          0x00000002               /* ADC SS1 Enable */
#define ADC_ACTSS_ASEN0          0x00000001               /* ADC SS0 Enable */

/* Bit fields in the TIVA_ADC_RIS register. */

#define ADC_RIS_INRDC            0x00010000               /* Digital Comparator Raw Interrupt Status */
#define ADC_RIS_DMAINR3          0x00000800               /* SS3 DMA Raw Interrupt Status */
#define ADC_RIS_DMAINR2          0x00000400               /* SS2 DMA Raw Interrupt Status */
#define ADC_RIS_DMAINR1          0x00000200               /* SS1 DMA Raw Interrupt Status */
#define ADC_RIS_DMAINR0          0x00000100               /* SS0 DMA Raw Interrupt Status */
#define ADC_RIS_INR3             0x00000008               /* SS3 Raw Interrupt Status */
#define ADC_RIS_INR2             0x00000004               /* SS2 Raw Interrupt Status */
#define ADC_RIS_INR1             0x00000002               /* SS1 Raw Interrupt Status */
#define ADC_RIS_INR0             0x00000001               /* SS0 Raw Interrupt Status */

/* Bit fields in the TIVA_ADC_IM register. */

#define ADC_IM_DCONSS3           0x00080000               /* Digital Comparator Interrupt on SS3 */
#define ADC_IM_DCONSS2           0x00040000               /* Digital Comparator Interrupt on SS2 */
#define ADC_IM_DCONSS1           0x00020000               /* Digital Comparator Interrupt on SS1 */
#define ADC_IM_DCONSS0           0x00010000               /* Digital Comparator Interrupt on SS0 */
#define ADC_IM_DMAMASK3          0x00000800               /* SS3 DMA Interrupt Mask */
#define ADC_IM_DMAMASK2          0x00000400               /* SS2 DMA Interrupt Mask */
#define ADC_IM_DMAMASK1          0x00000200               /* SS1 DMA Interrupt Mask */
#define ADC_IM_DMAMASK0          0x00000100               /* SS0 DMA Interrupt Mask */
#define ADC_IM_MASK3             0x00000008               /* SS3 Interrupt Mask */
#define ADC_IM_MASK2             0x00000004               /* SS2 Interrupt Mask */
#define ADC_IM_MASK1             0x00000002               /* SS1 Interrupt Mask */
#define ADC_IM_MASK0             0x00000001               /* SS0 Interrupt Mask */

/* Bit fields in the TIVA_ADC_ISC register. */

#define ADC_ISC_SSE(n)           (1 << ((n)*4))
#define ADC_ISC_DCIN_SHIFT       20
#  define ADC_ISC_DCINSS3        (0x8)                    /* Digital Comparator Interrupt Status on SS3 */
#  define ADC_ISC_DCINSS2        (0x4)                    /* Digital Comparator Interrupt Status on SS2 */
#  define ADC_ISC_DCINSS1        (0x2)                    /* Digital Comparator Interrupt Status on SS1 */
#  define ADC_ISC_DCINSS0        (0x1)                    /* Digital Comparator Interrupt Status on SS0 */
#define ADC_ISC_DMAIN_SHIFT      8
#  define ADC_ISC_DMAIN3         (0x8)                    /* SS3 DMA Interrupt Status and Clear */
#  define ADC_ISC_DMAIN2         (0x4)                    /* SS2 DMA Interrupt Status and Clear */
#  define ADC_ISC_DMAIN1         (0x2)                    /* SS1 DMA Interrupt Status and Clear */
#  define ADC_ISC_DMAIN0         (0x1)                    /* SS0 DMA Interrupt Status and Clear */
#define ADC_ISC_IN_SHIFT         0
#  define ADC_ISC_IN3            (0x8)                    /* SS3 Interrupt Status and Clear */
#  define ADC_ISC_IN2            (0x4)                    /* SS2 Interrupt Status and Clear */
#  define ADC_ISC_IN1            (0x2)                    /* SS1 Interrupt Status and Clear */
#  define ADC_ISC_IN0            (0x1)                    /* SS0 Interrupt Status and Clear */

/* Bit fields in the TIVA_ADC_OSTAT register. */

#define ADC_OSTAT_OV3            0x00000008               /* SS3 FIFO Overflow */
#define ADC_OSTAT_OV2            0x00000004               /* SS2 FIFO Overflow */
#define ADC_OSTAT_OV1            0x00000002               /* SS1 FIFO Overflow */
#define ADC_OSTAT_OV0            0x00000001               /* SS0 FIFO Overflow */

/* Bit fields in the TIVA_ADC_EMUX register. */

#define ADC_EMUX_SHIFT(n)        (4 * (n))                  /* SS EMUX Shift */
#define ADC_EMUX_MASK(n)         (0xF << ADC_EMUX_SHIFT(n)) /* SS EMUX Mask */
#  define ADC_EMUX_PROC          (0x0)                    /* Processor (default) */
#  define ADC_EMUX_COMP0         (0x1)                    /* Analog Comparator 0 */
#  define ADC_EMUX_COMP1         (0x2)                    /* Analog Comparator 1 */
#  define ADC_EMUX_COMP2         (0x3)                    /* Analog Comparator 2 */
#  define ADC_EMUX_EXTERNAL      (0x4)                    /* External (GPIO Pins) */
#  define ADC_EMUX_TIMER         (0x5)                    /* Timer */
#  define ADC_EMUX_PWM0          (0x6)                    /* PWM generator 0 */
#  define ADC_EMUX_PWM1          (0x7)                    /* PWM generator 1 */
#  define ADC_EMUX_PWM2          (0x8)                    /* PWM generator 2 */
#  define ADC_EMUX_PWM3          (0x9)                    /* PWM generator 3 */
#  define ADC_EMUX_NEVER         (0xe)                    /* Never Trigger */
#  define ADC_EMUX_ALWAYS        (0xf)                    /* Always (continuously sample) */

/* Bit fields in the TIVA_ADC_USTAT register. */

#define ADC_USTAT_UV3            0x00000008               /* SS3 FIFO Underflow */
#define ADC_USTAT_UV2            0x00000004               /* SS2 FIFO Underflow */
#define ADC_USTAT_UV1            0x00000002               /* SS1 FIFO Underflow */
#define ADC_USTAT_UV0            0x00000001               /* SS0 FIFO Underflow */

/* Bit fields in the TIVA_ADC_TSSEL register. */

#define ADC_TSSEL_PS_SHIFT(n)    (((n)+((n)+1))*4)
#define ADC_TSSEL_PS_MASK(n)     (0x3 << ADC_TSSEL_PS_SHIFT((n)))
#  define ADC_TSSEL_PS_M         (0x3)                    /* PWM module trigger select */
#  define ADC_TSSEL_PS_0         (0x0)                    /* Use PWM module 0 */
#  define ADC_TSSEL_PS_1         (0x1)                    /* Use PWM module 1 */

/* Bit fields in the TIVA_ADC_SSPRI register. */

#define ADC_SSPRI_SHIFT(n)       ((n) * 4)                   /* SSE priority mask */
#define ADC_SSPRI_MASK(n)        (0x3 << ADC_SSPRI_SHIFT(n)) /* SSE priority mask */
#  define ADC_SSPRI_0            (0x0)                    /* SSE priority value 0 (highest) */
#  define ADC_SSPRI_1            (0x1)                    /* SSE priority value 1 (high) */
#  define ADC_SSPRI_2            (0x2)                    /* SSE priority value 2 (low) */
#  define ADC_SSPRI_3            (0x3)                    /* SSE priority value 3 (lowest) */

/* Bit fields in the TIVA_ADC_SPC register. */

#define ADC_SPC_PHASE_MASK       0x0000000F               /* Phase Difference */
#define ADC_SPC_PHASE_0          0x00000000               /* ADC sample lags by 0.0 */
#define ADC_SPC_PHASE_22_5       0x00000001               /* ADC sample lags by 22.5 */
#define ADC_SPC_PHASE_45         0x00000002               /* ADC sample lags by 45.0 */
#define ADC_SPC_PHASE_67_5       0x00000003               /* ADC sample lags by 67.5 */
#define ADC_SPC_PHASE_90         0x00000004               /* ADC sample lags by 90.0 */
#define ADC_SPC_PHASE_112_5      0x00000005               /* ADC sample lags by 112.5 */
#define ADC_SPC_PHASE_135        0x00000006               /* ADC sample lags by 135.0 */
#define ADC_SPC_PHASE_157_5      0x00000007               /* ADC sample lags by 157.5 */
#define ADC_SPC_PHASE_180        0x00000008               /* ADC sample lags by 180.0 */
#define ADC_SPC_PHASE_202_5      0x00000009               /* ADC sample lags by 202.5 */
#define ADC_SPC_PHASE_225        0x0000000A               /* ADC sample lags by 225.0 */
#define ADC_SPC_PHASE_247_5      0x0000000B               /* ADC sample lags by 247.5 */
#define ADC_SPC_PHASE_270        0x0000000C               /* ADC sample lags by 270.0 */
#define ADC_SPC_PHASE_292_5      0x0000000D               /* ADC sample lags by 292.5 */
#define ADC_SPC_PHASE_315        0x0000000E               /* ADC sample lags by 315.0 */
#define ADC_SPC_PHASE_337_5      0x0000000F               /* ADC sample lags by 337.5 */

/* Bit fields in the TIVA_ADC_PSSI register. */

#define ADC_PSSI_GSYNC           0x80000000               /* Global Synchronize */
#define ADC_PSSI_SYNCWAIT        0x08000000               /* Synchronize Wait */

#define ADC_PSSI_TRIG_MASK       0xf                      /* Enable triggering mask */
#  define ADC_PSSI_SS3           0x8                      /* SS3 Initiate */
#  define ADC_PSSI_SS2           0x4                      /* SS2 Initiate */
#  define ADC_PSSI_SS1           0x2                      /* SS1 Initiate */
#  define ADC_PSSI_SS0           0x1                      /* SS0 Initiate */

/* Bit fields in the TIVA_ADC_SAC register. */

#define ADC_SAC_AVG_MASK         0x00000007               /* Hardware Averaging Control */
#define ADC_SAC_AVG_OFF          0x00000000               /* No hardware oversampling */
#define ADC_SAC_AVG_2X           0x00000001               /* 2x hardware oversampling */
#define ADC_SAC_AVG_4X           0x00000002               /* 4x hardware oversampling */
#define ADC_SAC_AVG_8X           0x00000003               /* 8x hardware oversampling */
#define ADC_SAC_AVG_16X          0x00000004               /* 16x hardware oversampling */
#define ADC_SAC_AVG_32X          0x00000005               /* 32x hardware oversampling */
#define ADC_SAC_AVG_64X          0x00000006               /* 64x hardware oversampling */

/* Bit fields in the TIVA_ADC_DCISC register. */

#define ADC_DCISC_DCINT7         0x00000080               /* Digital Comparator 7 Interrupt Status and Clear */
#define ADC_DCISC_DCINT6         0x00000040               /* Digital Comparator 6 Interrupt Status and Clear */
#define ADC_DCISC_DCINT5         0x00000020               /* Digital Comparator 5 Interrupt Status and Clear */
#define ADC_DCISC_DCINT4         0x00000010               /* Digital Comparator 4 Interrupt Status and Clear */
#define ADC_DCISC_DCINT3         0x00000008               /* Digital Comparator 3 Interrupt Status and Clear */
#define ADC_DCISC_DCINT2         0x00000004               /* Digital Comparator 2 Interrupt Status and Clear */
#define ADC_DCISC_DCINT1         0x00000002               /* Digital Comparator 1 Interrupt Status and Clear */
#define ADC_DCISC_DCINT0         0x00000001               /* Digital Comparator 0 Interrupt Status and Clear */

/* Bit fields in the TIVA_ADC_CTL register. */

#define ADC_CTL_VREF_MASK        0x00000003               /* Voltage Reference Select */
#define ADC_CTL_DITHER           0x00000040               /* Dither Mode Enable */
#define ADC_CTL_VREF_INTERNAL    0x00000000               /* VDDA and GNDA are the voltage references */
#define ADC_CTL_VREF_EXT_3V      0x00000001               /* The external VREFA+ and VREFA- inputs are the voltage references */

/* Bit fields in the TIVA_ADC_SSMUX register. */

#define ADC_SSMUX_MUX_SHIFT(n)   ((n)*4)                  /* nth Sample Input Select */
#define ADC_SSMUX_MUX_MASK(n)    (0xF << ADC_SSMUX_MUX_SHIFT(n))

/* Bit fields in the TIVA_ADC_SSCTL register. */

#define ADC_SSCTL_SHIFT(n)       ((n)*4)
#define ADC_SSCTL_MASK(n)        (0xF << ADC_SSCTL_SHIFT((n)))
#  define ADC_SSCTL_TS           (0x8)                    /* Sample Temp Sensor Select */
#  define ADC_SSCTL_IE           (0x4)                    /* Sample Interrupt Enable */
#  define ADC_SSCTL_END          (0x2)                    /* Sample is End of Sequence */
#  define ADC_SSCTL_D            (0x1)                    /* Sample Differential Input Select */

/* Bit fields in the TIVA_ADC_SSFIFO0 register. */

#define ADC_SSFIFO0_DATA_MASK    0x00000FFF               /* Conversion Result Data */
#  define ADC_SSFIFO0_DATA_SHIFT 0

/* Bit fields in the TIVA_ADC_SSFSTAT0 register. */

#define ADC_SSFSTAT0_HPTR_MASK   0x000000F0               /* FIFO Head Pointer */
#define ADC_SSFSTAT_TPTR_MASK    0x0000000F               /* FIFO Tail Pointer */
#  define ADC_SSFSTAT_HPTR_SHIFT 4
#  define ADC_SSFSTAT_TPTR_SHIFT 0
#define ADC_SSFSTAT_FULL         0x00001000               /* FIFO Full */
#define ADC_SSFSTAT_EMPTY        0x00000100               /* FIFO Empty */

/* Bit fields in the TIVA_ADC_SSOP0 register. */

#define ADC_SSOP0_S7DCOP         0x10000000               /* Sample 7 Digital Comparator Operation */
#define ADC_SSOP0_S6DCOP         0x01000000               /* Sample 6 Digital Comparator Operation */
#define ADC_SSOP0_S5DCOP         0x00100000               /* Sample 5 Digital Comparator Operation */
#define ADC_SSOP0_S4DCOP         0x00010000               /* Sample 4 Digital Comparator Operation */
#define ADC_SSOP0_S3DCOP         0x00001000               /* Sample 3 Digital Comparator Operation */
#define ADC_SSOP0_S2DCOP         0x00000100               /* Sample 2 Digital Comparator Operation */
#define ADC_SSOP0_S1DCOP         0x00000010               /* Sample 1 Digital Comparator Operation */
#define ADC_SSOP0_S0DCOP         0x00000001               /* Sample 0 Digital Comparator Operation */

/* Bit fields in the TIVA_ADC_SSDC0 register. */

#define ADC_SSDC0_S7DCSEL_MASK   0xF0000000               /* Sample 7 Digital Comparator Select */
#define ADC_SSDC0_S6DCSEL_MASK   0x0F000000               /* Sample 6 Digital Comparator Select */
#define ADC_SSDC0_S5DCSEL_MASK   0x00F00000               /* Sample 5 Digital Comparator Select */
#define ADC_SSDC0_S4DCSEL_MASK   0x000F0000               /* Sample 4 Digital Comparator Select */
#define ADC_SSDC0_S3DCSEL_MASK   0x0000F000               /* Sample 3 Digital Comparator Select */
#define ADC_SSDC0_S2DCSEL_MASK   0x00000F00               /* Sample 2 Digital Comparator Select */
#define ADC_SSDC0_S1DCSEL_MASK   0x000000F0               /* Sample 1 Digital Comparator Select */
#define ADC_SSDC0_S0DCSEL_MASK   0x0000000F               /* Sample 0 Digital Comparator Select */
#define ADC_SSDC0_S6DCSEL_SHIFT  24
#define ADC_SSDC0_S5DCSEL_SHIFT  20
#define ADC_SSDC0_S4DCSEL_SHIFT  16
#define ADC_SSDC0_S3DCSEL_SHIFT  12
#define ADC_SSDC0_S2DCSEL_SHIFT  8
#define ADC_SSDC0_S1DCSEL_SHIFT  4
#define ADC_SSDC0_S0DCSEL_SHIFT  0

/* Bit fields in the TIVA_ADC_SSEMUX register. */

#define ADC_SSEMUX_MUX_SHIFT(n)      ((n) * 4)                        /* Sample sequence extended mux n */
#define ADC_SSEMUX_MUX_MASK(n)       (0xF << ADC_SSEMUX_MUX_SHIFT(n)) /* Sample sequence extended mux n mask */
#  define ADC_SSEMUX_MUX_CHN_0_15    (0x0)                            /*  SSE uses channels 0 to 15 */
#  define ADC_SSEMUX_MUX_CHN_16_23   (0x1)                            /*  SSE uses channels 16 to 23 */

/* Bit fields in the TIVA_ADC_SSTSH register. */

#define ADC_SSTSH_SHIFT(n)       ((n) * 4)
#define ADC_SSTSH_MASK(n)        (0xf << (ADC_SSTSH_SHIFT(n))) /* nth Sample and Hold Period Select */
#  define ADC_SSTH_SHOLD_4       (0x0)                    /* Sample and hold 4 ADC clocks */
#  define ADC_SSTH_SHOLD_8       (0x2)                    /* Sample and hold 8 ADC clocks */
#  define ADC_SSTH_SHOLD_16      (0x4)                    /* Sample and hold 16 ADC clocks */
#  define ADC_SSTH_SHOLD_32      (0x6)                    /* Sample and hold 32 ADC clocks */
#  define ADC_SSTH_SHOLD_64      (0x8)                    /* Sample and hold 64 ADC clocks */
#  define ADC_SSTH_SHOLD_128     (0xa)                    /* Sample and hold 128 ADC clocks */
#  define ADC_SSTH_SHOLD_256     (0xc)                    /* Sample and hold 256 ADC clocks */
#  define SSTSH_TSH_TS           ADC_SSTH_SHOLD_4         /* Same and hold time for the temp sensor should be at least 16 ADC ticks */

/* Bit fields in the TIVA_ADC_SSFIFO1 register. */

#define ADC_SSFIFO1_DATA_MASK    0x00000FFF               /* Conversion Result Data */
#  define ADC_SSFIFO1_DATA_SHIFT 0

/* Bit fields in the TIVA_ADC_SSFSTAT1 register. */

#define ADC_SSFSTAT1_HPTR_MASK   0x000000F0               /* FIFO Head Pointer */
#define ADC_SSFSTAT1_TPTR_MASK   0x0000000F               /* FIFO Tail Pointer */
#  define ADC_SSFSTAT1_HPTR_SHIFT  4
#  define ADC_SSFSTAT1_TPTR_SHIFT  0
#define ADC_SSFSTAT1_FULL        0x00001000               /* FIFO Full */
#define ADC_SSFSTAT1_EMPTY       0x00000100               /* FIFO Empty */

/* Bit fields in the TIVA_ADC_SSOP1 register. */

#define ADC_SSOP1_S3DCOP         0x00001000               /* Sample 3 Digital Comparator Operation */
#define ADC_SSOP1_S2DCOP         0x00000100               /* Sample 2 Digital Comparator Operation */
#define ADC_SSOP1_S1DCOP         0x00000010               /* Sample 1 Digital Comparator Operation */
#define ADC_SSOP1_S0DCOP         0x00000001               /* Sample 0 Digital Comparator Operation */

/* Bit fields in the TIVA_ADC_SSDC1 register. */

#define ADC_SSDC1_S3DCSEL_MASK   0x0000F000               /* Sample 3 Digital Comparator Select */
#define ADC_SSDC1_S2DCSEL_MASK   0x00000F00               /* Sample 2 Digital Comparator Select */
#define ADC_SSDC1_S1DCSEL_MASK   0x000000F0               /* Sample 1 Digital Comparator Select */
#define ADC_SSDC1_S0DCSEL_MASK   0x0000000F               /* Sample 0 Digital Comparator Select */
#  define ADC_SSDC1_S2DCSEL_SHIFT  8
#  define ADC_SSDC1_S1DCSEL_SHIFT  4
#  define ADC_SSDC1_S0DCSEL_SHIFT  0

/* Bit fields in the TIVA_ADC_SSTSH1 register. */

#define ADC_SSTSH1_TSH3_MASK     0x0000F000               /* 4th Sample and Hold Period Select */
#define ADC_SSTSH1_TSH2_MASK     0x00000F00               /* 3rd Sample and Hold Period Select */
#define ADC_SSTSH1_TSH1_MASK     0x000000F0               /* 2nd Sample and Hold Period Select */
#define ADC_SSTSH1_TSH0_MASK     0x0000000F               /* 1st Sample and Hold Period Select */
#  define ADC_SSTSH1_TSH3_SHIFT    12
#  define ADC_SSTSH1_TSH2_SHIFT    8
#  define ADC_SSTSH1_TSH1_SHIFT    4
#  define ADC_SSTSH1_TSH0_SHIFT    0

/* Bit fields in the TIVA_ADC_SSFIFO2 register. */

#define ADC_SSFIFO2_DATA_MASK    0x00000FFF               /* Conversion Result Data */
#  define ADC_SSFIFO2_DATA_SHIFT   0

/* Bit fields in the TIVA_ADC_SSFSTAT2 register. */

#define ADC_SSFSTAT2_HPTR_MASK   0x000000F0               /* FIFO Head Pointer */
#define ADC_SSFSTAT2_TPTR_MASK   0x0000000F               /* FIFO Tail Pointer */
#  define ADC_SSFSTAT2_HPTR_SHIFT  4
#  define ADC_SSFSTAT2_TPTR_SHIFT  0
#define ADC_SSFSTAT2_FULL        0x00001000               /* FIFO Full */
#define ADC_SSFSTAT2_EMPTY       0x00000100               /* FIFO Empty */

/* Bit fields in the TIVA_ADC_SSOP2 register. */

#define ADC_SSOP2_S3DCOP         0x00001000               /* Sample 3 Digital Comparator Operation */
#define ADC_SSOP2_S2DCOP         0x00000100               /* Sample 2 Digital Comparator Operation */
#define ADC_SSOP2_S1DCOP         0x00000010               /* Sample 1 Digital Comparator Operation */
#define ADC_SSOP2_S0DCOP         0x00000001               /* Sample 0 Digital Comparator Operation */

/* Bit fields in the TIVA_ADC_SSDC2 register. */

#define ADC_SSDC2_S3DCSEL_MASK   0x0000F000               /* Sample 3 Digital Comparator Select */
#define ADC_SSDC2_S2DCSEL_MASK   0x00000F00               /* Sample 2 Digital Comparator Select */
#define ADC_SSDC2_S1DCSEL_MASK   0x000000F0               /* Sample 1 Digital Comparator Select */
#define ADC_SSDC2_S0DCSEL_MASK   0x0000000F               /* Sample 0 Digital Comparator Select */
#  define ADC_SSDC2_S2DCSEL_SHIFT  8
#  define ADC_SSDC2_S1DCSEL_SHIFT  4
#  define ADC_SSDC2_S0DCSEL_SHIFT  0

/* Bit fields in the TIVA_ADC_SSTSH2 register. */

#define ADC_SSTSH2_TSH3_MASK     0x0000F000               /* 4th Sample and Hold Period Select */
#define ADC_SSTSH2_TSH2_MASK     0x00000F00               /* 3rd Sample and Hold Period Select */
#define ADC_SSTSH2_TSH1_MASK     0x000000F0               /* 2nd Sample and Hold Period Select */
#define ADC_SSTSH2_TSH0_MASK     0x0000000F               /* 1st Sample and Hold Period Select */
#  define ADC_SSTSH2_TSH3_SHIFT  12
#  define ADC_SSTSH2_TSH2_SHIFT  8
#  define ADC_SSTSH2_TSH1_SHIFT  4
#  define ADC_SSTSH2_TSH0_SHIFT  0

/* Bit fields in the TIVA_ADC_SSFIFO3 register. */

#define ADC_SSFIFO3_DATA_MASK    0x00000FFF               /* Conversion Result Data */
#  define ADC_SSFIFO3_DATA_SHIFT 0

/* Bit fields in the TIVA_ADC_SSFSTAT3 register. */

#define ADC_SSFSTAT3_HPTR_MASK   0x000000F0               /* FIFO Head Pointer */
#define ADC_SSFSTAT3_TPTR_MASK   0x0000000F               /* FIFO Tail Pointer */
#  define ADC_SSFSTAT3_HPTR_SHIFT  4
#  define ADC_SSFSTAT3_TPTR_SHIFT  0
#define ADC_SSFSTAT3_FULL        0x00001000               /* FIFO Full */
#define ADC_SSFSTAT3_EMPTY       0x00000100               /* FIFO Empty */

/* Bit fields in the TIVA_ADC_SSOP3 register. */

#define ADC_SSOP3_S0DCOP         0x00000001               /* Sample 0 Digital Comparator Operation */

/* Bit fields in the TIVA_ADC_SSDC3 register. */

#define ADC_SSDC3_S0DCSEL_MASK   0x0000000F               /* Sample 0 Digital Comparator Select */

/* Bit fields in the TIVA_ADC_SSTSH3 register. */

#define ADC_SSTSH3_TSH0_MASK     0x0000000F               /* 1st Sample and Hold Period Select */
#  define ADC_SSTSH3_TSH0_SHIFT  0

/* Bit fields in the TIVA_ADC_DCRIC register. */

#define ADC_DCRIC_DCTRIG7        0x00800000               /* Digital Comparator Trigger 7 */
#define ADC_DCRIC_DCTRIG6        0x00400000               /* Digital Comparator Trigger 6 */
#define ADC_DCRIC_DCTRIG5        0x00200000               /* Digital Comparator Trigger 5 */
#define ADC_DCRIC_DCTRIG4        0x00100000               /* Digital Comparator Trigger 4 */
#define ADC_DCRIC_DCTRIG3        0x00080000               /* Digital Comparator Trigger 3 */
#define ADC_DCRIC_DCTRIG2        0x00040000               /* Digital Comparator Trigger 2 */
#define ADC_DCRIC_DCTRIG1        0x00020000               /* Digital Comparator Trigger 1 */
#define ADC_DCRIC_DCTRIG0        0x00010000               /* Digital Comparator Trigger 0 */
#define ADC_DCRIC_DCINT7         0x00000080               /* Digital Comparator Interrupt 7 */
#define ADC_DCRIC_DCINT6         0x00000040               /* Digital Comparator Interrupt 6 */
#define ADC_DCRIC_DCINT5         0x00000020               /* Digital Comparator Interrupt 5 */
#define ADC_DCRIC_DCINT4         0x00000010               /* Digital Comparator Interrupt 4 */
#define ADC_DCRIC_DCINT3         0x00000008               /* Digital Comparator Interrupt 3 */
#define ADC_DCRIC_DCINT2         0x00000004               /* Digital Comparator Interrupt 2 */
#define ADC_DCRIC_DCINT1         0x00000002               /* Digital Comparator Interrupt 1 */
#define ADC_DCRIC_DCINT0         0x00000001               /* Digital Comparator Interrupt 0 */

/* Bit fields in the TIVA_ADC_DCCTL0 register. */

#define ADC_DCCTL0_CTC_MASK      0x00000C00               /* Comparison Trigger Condition */
#define ADC_DCCTL0_CTM_MASK      0x00000300               /* Comparison Trigger Mode */
#define ADC_DCCTL0_CIC_MASK      0x0000000C               /* Comparison Interrupt Condition */
#define ADC_DCCTL0_CIM_MASK      0x00000003               /* Comparison Interrupt Mode */
#define ADC_DCCTL0_CTE           0x00001000               /* Comparison Trigger Enable */
#define ADC_DCCTL0_CTC_LOW       0x00000000               /* Low Band */
#define ADC_DCCTL0_CTC_MID       0x00000400               /* Mid Band */
#define ADC_DCCTL0_CTC_HIGH      0x00000C00               /* High Band */
#define ADC_DCCTL0_CTM_ALWAYS    0x00000000               /* Always */
#define ADC_DCCTL0_CTM_ONCE      0x00000100               /* Once */
#define ADC_DCCTL0_CTM_HALWAYS   0x00000200               /* Hysteresis Always */
#define ADC_DCCTL0_CTM_HONCE     0x00000300               /* Hysteresis Once */
#define ADC_DCCTL0_CIE           0x00000010               /* Comparison Interrupt Enable */
#define ADC_DCCTL0_CIC_LOW       0x00000000               /* Low Band */
#define ADC_DCCTL0_CIC_MID       0x00000004               /* Mid Band */
#define ADC_DCCTL0_CIC_HIGH      0x0000000C               /* High Band */
#define ADC_DCCTL0_CIM_ALWAYS    0x00000000               /* Always */
#define ADC_DCCTL0_CIM_ONCE      0x00000001               /* Once */
#define ADC_DCCTL0_CIM_HALWAYS   0x00000002               /* Hysteresis Always */
#define ADC_DCCTL0_CIM_HONCE     0x00000003               /* Hysteresis Once */

/* Bit fields in the TIVA_ADC_DCCTL1 register. */

#define ADC_DCCTL1_CTC_MASK      0x00000C00               /* Comparison Trigger Condition */
#define ADC_DCCTL1_CTM_MASK      0x00000300               /* Comparison Trigger Mode */
#define ADC_DCCTL1_CIC_MASK      0x0000000C               /* Comparison Interrupt Condition */
#define ADC_DCCTL1_CIM_MASK      0x00000003               /* Comparison Interrupt Mode */
#define ADC_DCCTL1_CTE           0x00001000               /* Comparison Trigger Enable */
#define ADC_DCCTL1_CTC_LOW       0x00000000               /* Low Band */
#define ADC_DCCTL1_CTC_MID       0x00000400               /* Mid Band */
#define ADC_DCCTL1_CTC_HIGH      0x00000C00               /* High Band */
#define ADC_DCCTL1_CTM_ALWAYS    0x00000000               /* Always */
#define ADC_DCCTL1_CTM_ONCE      0x00000100               /* Once */
#define ADC_DCCTL1_CTM_HALWAYS   0x00000200               /* Hysteresis Always */
#define ADC_DCCTL1_CTM_HONCE     0x00000300               /* Hysteresis Once */
#define ADC_DCCTL1_CIE           0x00000010               /* Comparison Interrupt Enable */
#define ADC_DCCTL1_CIC_LOW       0x00000000               /* Low Band */
#define ADC_DCCTL1_CIC_MID       0x00000004               /* Mid Band */
#define ADC_DCCTL1_CIC_HIGH      0x0000000C               /* High Band */
#define ADC_DCCTL1_CIM_ALWAYS    0x00000000               /* Always */
#define ADC_DCCTL1_CIM_ONCE      0x00000001               /* Once */
#define ADC_DCCTL1_CIM_HALWAYS   0x00000002               /* Hysteresis Always */
#define ADC_DCCTL1_CIM_HONCE     0x00000003               /* Hysteresis Once */

/* Bit fields in the TIVA_ADC_DCCTL2 register. */

#define ADC_DCCTL2_CTC_MASK      0x00000C00               /* Comparison Trigger Condition */
#define ADC_DCCTL2_CTM_MASK      0x00000300               /* Comparison Trigger Mode */
#define ADC_DCCTL2_CIC_MASK      0x0000000C               /* Comparison Interrupt Condition */
#define ADC_DCCTL2_CIM_MASK      0x00000003               /* Comparison Interrupt Mode */
#define ADC_DCCTL2_CTE           0x00001000               /* Comparison Trigger Enable */
#define ADC_DCCTL2_CTC_LOW       0x00000000               /* Low Band */
#define ADC_DCCTL2_CTC_MID       0x00000400               /* Mid Band */
#define ADC_DCCTL2_CTC_HIGH      0x00000C00               /* High Band */
#define ADC_DCCTL2_CTM_ALWAYS    0x00000000               /* Always */
#define ADC_DCCTL2_CTM_ONCE      0x00000100               /* Once */
#define ADC_DCCTL2_CTM_HALWAYS   0x00000200               /* Hysteresis Always */
#define ADC_DCCTL2_CTM_HONCE     0x00000300               /* Hysteresis Once */
#define ADC_DCCTL2_CIE           0x00000010               /* Comparison Interrupt Enable */
#define ADC_DCCTL2_CIC_LOW       0x00000000               /* Low Band */
#define ADC_DCCTL2_CIC_MID       0x00000004               /* Mid Band */
#define ADC_DCCTL2_CIC_HIGH      0x0000000C               /* High Band */
#define ADC_DCCTL2_CIM_ALWAYS    0x00000000               /* Always */
#define ADC_DCCTL2_CIM_ONCE      0x00000001               /* Once */
#define ADC_DCCTL2_CIM_HALWAYS   0x00000002               /* Hysteresis Always */
#define ADC_DCCTL2_CIM_HONCE     0x00000003               /* Hysteresis Once */

/* Bit fields in the TIVA_ADC_DCCTL3 register. */

#define ADC_DCCTL3_CTC_MASK      0x00000C00               /* Comparison Trigger Condition */
#define ADC_DCCTL3_CTM_MASK      0x00000300               /* Comparison Trigger Mode */
#define ADC_DCCTL3_CIC_MASK      0x0000000C               /* Comparison Interrupt Condition */
#define ADC_DCCTL3_CIM_MASK      0x00000003               /* Comparison Interrupt Mode */
#define ADC_DCCTL3_CTE           0x00001000               /* Comparison Trigger Enable */
#define ADC_DCCTL3_CTC_LOW       0x00000000               /* Low Band */
#define ADC_DCCTL3_CTC_MID       0x00000400               /* Mid Band */
#define ADC_DCCTL3_CTC_HIGH      0x00000C00               /* High Band */
#define ADC_DCCTL3_CTM_ALWAYS    0x00000000               /* Always */
#define ADC_DCCTL3_CTM_ONCE      0x00000100               /* Once */
#define ADC_DCCTL3_CTM_HALWAYS   0x00000200               /* Hysteresis Always */
#define ADC_DCCTL3_CTM_HONCE     0x00000300               /* Hysteresis Once */
#define ADC_DCCTL3_CIE           0x00000010               /* Comparison Interrupt Enable */
#define ADC_DCCTL3_CIC_LOW       0x00000000               /* Low Band */
#define ADC_DCCTL3_CIC_MID       0x00000004               /* Mid Band */
#define ADC_DCCTL3_CIC_HIGH      0x0000000C               /* High Band */
#define ADC_DCCTL3_CIM_ALWAYS    0x00000000               /* Always */
#define ADC_DCCTL3_CIM_ONCE      0x00000001               /* Once */
#define ADC_DCCTL3_CIM_HALWAYS   0x00000002               /* Hysteresis Always */
#define ADC_DCCTL3_CIM_HONCE     0x00000003               /* Hysteresis Once */

/* Bit fields in the TIVA_ADC_DCCTL4 register. */

#define ADC_DCCTL4_CTC_MASK      0x00000C00               /* Comparison Trigger Condition */
#define ADC_DCCTL4_CTM_MASK      0x00000300               /* Comparison Trigger Mode */
#define ADC_DCCTL4_CIC_MASK      0x0000000C               /* Comparison Interrupt Condition */
#define ADC_DCCTL4_CIM_MASK      0x00000003               /* Comparison Interrupt Mode */
#define ADC_DCCTL4_CTE           0x00001000               /* Comparison Trigger Enable */
#define ADC_DCCTL4_CTC_LOW       0x00000000               /* Low Band */
#define ADC_DCCTL4_CTC_MID       0x00000400               /* Mid Band */
#define ADC_DCCTL4_CTC_HIGH      0x00000C00               /* High Band */
#define ADC_DCCTL4_CTM_ALWAYS    0x00000000               /* Always */
#define ADC_DCCTL4_CTM_ONCE      0x00000100               /* Once */
#define ADC_DCCTL4_CTM_HALWAYS   0x00000200               /* Hysteresis Always */
#define ADC_DCCTL4_CTM_HONCE     0x00000300               /* Hysteresis Once */
#define ADC_DCCTL4_CIE           0x00000010               /* Comparison Interrupt Enable */
#define ADC_DCCTL4_CIC_LOW       0x00000000               /* Low Band */
#define ADC_DCCTL4_CIC_MID       0x00000004               /* Mid Band */
#define ADC_DCCTL4_CIC_HIGH      0x0000000C               /* High Band */
#define ADC_DCCTL4_CIM_ALWAYS    0x00000000               /* Always */
#define ADC_DCCTL4_CIM_ONCE      0x00000001               /* Once */
#define ADC_DCCTL4_CIM_HALWAYS   0x00000002               /* Hysteresis Always */
#define ADC_DCCTL4_CIM_HONCE     0x00000003               /* Hysteresis Once */

/* Bit fields in the TIVA_ADC_DCCTL5 register. */

#define ADC_DCCTL5_CTC_MASK      0x00000C00               /* Comparison Trigger Condition */
#define ADC_DCCTL5_CTM_MASK      0x00000300               /* Comparison Trigger Mode */
#define ADC_DCCTL5_CIC_MASK      0x0000000C               /* Comparison Interrupt Condition */
#define ADC_DCCTL5_CIM_MASK      0x00000003               /* Comparison Interrupt Mode */
#define ADC_DCCTL5_CTE           0x00001000               /* Comparison Trigger Enable */
#define ADC_DCCTL5_CTC_LOW       0x00000000               /* Low Band */
#define ADC_DCCTL5_CTC_MID       0x00000400               /* Mid Band */
#define ADC_DCCTL5_CTC_HIGH      0x00000C00               /* High Band */
#define ADC_DCCTL5_CTM_ALWAYS    0x00000000               /* Always */
#define ADC_DCCTL5_CTM_ONCE      0x00000100               /* Once */
#define ADC_DCCTL5_CTM_HALWAYS   0x00000200               /* Hysteresis Always */
#define ADC_DCCTL5_CTM_HONCE     0x00000300               /* Hysteresis Once */
#define ADC_DCCTL5_CIE           0x00000010               /* Comparison Interrupt Enable */
#define ADC_DCCTL5_CIC_LOW       0x00000000               /* Low Band */
#define ADC_DCCTL5_CIC_MID       0x00000004               /* Mid Band */
#define ADC_DCCTL5_CIC_HIGH      0x0000000C               /* High Band */
#define ADC_DCCTL5_CIM_ALWAYS    0x00000000               /* Always */
#define ADC_DCCTL5_CIM_ONCE      0x00000001               /* Once */
#define ADC_DCCTL5_CIM_HALWAYS   0x00000002               /* Hysteresis Always */
#define ADC_DCCTL5_CIM_HONCE     0x00000003               /* Hysteresis Once */

/* Bit fields in the TIVA_ADC_DCCTL6 register. */

#define ADC_DCCTL6_CTC_MASK      0x00000C00               /* Comparison Trigger Condition */
#define ADC_DCCTL6_CTM_MASK      0x00000300               /* Comparison Trigger Mode */
#define ADC_DCCTL6_CIC_MASK      0x0000000C               /* Comparison Interrupt Condition */
#define ADC_DCCTL6_CIM_MASK      0x00000003               /* Comparison Interrupt Mode */
#define ADC_DCCTL6_CTE           0x00001000               /* Comparison Trigger Enable */
#define ADC_DCCTL6_CTC_LOW       0x00000000               /* Low Band */
#define ADC_DCCTL6_CTC_MID       0x00000400               /* Mid Band */
#define ADC_DCCTL6_CTC_HIGH      0x00000C00               /* High Band */
#define ADC_DCCTL6_CTM_ALWAYS    0x00000000               /* Always */
#define ADC_DCCTL6_CTM_ONCE      0x00000100               /* Once */
#define ADC_DCCTL6_CTM_HALWAYS   0x00000200               /* Hysteresis Always */
#define ADC_DCCTL6_CTM_HONCE     0x00000300               /* Hysteresis Once */
#define ADC_DCCTL6_CIE           0x00000010               /* Comparison Interrupt Enable */
#define ADC_DCCTL6_CIC_LOW       0x00000000               /* Low Band */
#define ADC_DCCTL6_CIC_MID       0x00000004               /* Mid Band */
#define ADC_DCCTL6_CIC_HIGH      0x0000000C               /* High Band */
#define ADC_DCCTL6_CIM_ALWAYS    0x00000000               /* Always */
#define ADC_DCCTL6_CIM_ONCE      0x00000001               /* Once */
#define ADC_DCCTL6_CIM_HALWAYS   0x00000002               /* Hysteresis Always */
#define ADC_DCCTL6_CIM_HONCE     0x00000003               /* Hysteresis Once */

/* Bit fields in the TIVA_ADC_DCCTL7 register. */

#define ADC_DCCTL7_CTC_MASK      0x00000C00               /* Comparison Trigger Condition */
#define ADC_DCCTL7_CTM_MASK      0x00000300               /* Comparison Trigger Mode */
#define ADC_DCCTL7_CIC_MASK      0x0000000C               /* Comparison Interrupt Condition */
#define ADC_DCCTL7_CIM_MASK      0x00000003               /* Comparison Interrupt Mode */
#define ADC_DCCTL7_CTE           0x00001000               /* Comparison Trigger Enable */
#define ADC_DCCTL7_CTC_LOW       0x00000000               /* Low Band */
#define ADC_DCCTL7_CTC_MID       0x00000400               /* Mid Band */
#define ADC_DCCTL7_CTC_HIGH      0x00000C00               /* High Band */
#define ADC_DCCTL7_CTM_ALWAYS    0x00000000               /* Always */
#define ADC_DCCTL7_CTM_ONCE      0x00000100               /* Once */
#define ADC_DCCTL7_CTM_HALWAYS   0x00000200               /* Hysteresis Always */
#define ADC_DCCTL7_CTM_HONCE     0x00000300               /* Hysteresis Once */
#define ADC_DCCTL7_CIE           0x00000010               /* Comparison Interrupt Enable */
#define ADC_DCCTL7_CIC_LOW       0x00000000               /* Low Band */
#define ADC_DCCTL7_CIC_MID       0x00000004               /* Mid Band */
#define ADC_DCCTL7_CIC_HIGH      0x0000000C               /* High Band */
#define ADC_DCCTL7_CIM_ALWAYS    0x00000000               /* Always */
#define ADC_DCCTL7_CIM_ONCE      0x00000001               /* Once */
#define ADC_DCCTL7_CIM_HALWAYS   0x00000002               /* Hysteresis Always */
#define ADC_DCCTL7_CIM_HONCE     0x00000003               /* Hysteresis Once */

/* Bit fields in the TIVA_ADC_DCCMP0 register. */

#define ADC_DCCMP0_COMP1_MASK    0x0FFF0000               /* Compare 1 */
#define ADC_DCCMP0_COMP0_MASK    0x00000FFF               /* Compare 0 */
#  define ADC_DCCMP0_COMP1_SHIFT 16
#  define ADC_DCCMP0_COMP0_SHIFT 0

/* Bit fields in the TIVA_ADC_DCCMP1 register. */

#define ADC_DCCMP1_COMP1_MASK    0x0FFF0000               /* Compare 1 */
#define ADC_DCCMP1_COMP0_MASK    0x00000FFF               /* Compare 0 */
#  define ADC_DCCMP1_COMP1_SHIFT 16
#  define ADC_DCCMP1_COMP0_SHIFT 0

/* Bit fields in the TIVA_ADC_DCCMP2 register. */

#define ADC_DCCMP2_COMP1_MASK    0x0FFF0000               /* Compare 1 */
#define ADC_DCCMP2_COMP0_MASK    0x00000FFF               /* Compare 0 */
#  define ADC_DCCMP2_COMP1_SHIFT 16
#  define ADC_DCCMP2_COMP0_SHIFT 0

/* Bit fields in the TIVA_ADC_DCCMP3 register. */

#define ADC_DCCMP3_COMP1_MASK    0x0FFF0000               /* Compare 1 */
#define ADC_DCCMP3_COMP0_MASK    0x00000FFF               /* Compare 0 */
#  define ADC_DCCMP3_COMP1_SHIFT 16
#  define ADC_DCCMP3_COMP0_SHIFT 0

/* Bit fields in the TIVA_ADC_DCCMP4 register. */

#define ADC_DCCMP4_COMP1_MASK    0x0FFF0000               /* Compare 1 */
#define ADC_DCCMP4_COMP0_MASK    0x00000FFF               /* Compare 0 */
#  define ADC_DCCMP4_COMP1_SHIFT 16
#  define ADC_DCCMP4_COMP0_SHIFT 0

/* Bit fields in the TIVA_ADC_DCCMP5 register. */

#define ADC_DCCMP5_COMP1_MASK    0x0FFF0000               /* Compare 1 */
#define ADC_DCCMP5_COMP0_MASK    0x00000FFF               /* Compare 0 */
#  define ADC_DCCMP5_COMP1_SHIFT 16
#  define ADC_DCCMP5_COMP0_SHIFT 0

/* Bit fields in the TIVA_ADC_DCCMP6 register. */

#define ADC_DCCMP6_COMP1_MASK    0x0FFF0000               /* Compare 1 */
#define ADC_DCCMP6_COMP0_MASK    0x00000FFF               /* Compare 0 */
#  define ADC_DCCMP6_COMP1_SHIFT 16
#  define ADC_DCCMP6_COMP0_SHIFT 0

/* Bit fields in the TIVA_ADC_DCCMP7 register. */

#define ADC_DCCMP7_COMP1_MASK    0x0FFF0000               /* Compare 1 */
#define ADC_DCCMP7_COMP0_MASK    0x00000FFF               /* Compare 0 */
#  define ADC_DCCMP7_COMP1_SHIFT 16
#  define ADC_DCCMP7_COMP0_SHIFT 0

/* Bit fields in the TIVA_ADC_PP register. */

#define ADC_PP_RSL_MASK          0x007C0000               /* Resolution */
#define ADC_PP_TYPE_MASK         0x00030000               /* ADC Architecture */
#define ADC_PP_DC_MASK           0x0000FC00               /* Digital Comparator Count */
#define ADC_PP_CH_MASK           0x000003F0               /* ADC Channel Count */
#define ADC_PP_MCR_MASK          0x0000000F               /* Maximum Conversion Rate */
#define ADC_PP_MSR_MASK          0x0000000F               /* Maximum ADC Sample Rate */
#  define ADC_PP_RSL_SHIFT       18
#  define ADC_PP_DC_SHIFT        10
#  define ADC_PP_CH_SHIFT        4
#define ADC_PP_APSHT             0x01000000               /* Application-Programmable Sample-and-Hold Time */
#define ADC_PP_TS                0x00800000               /* Temperature Sensor */
#define ADC_PP_TYPE_SAR          0x00000000               /* SAR */
#define ADC_PP_MCR_FULL          0x00000007               /* Full conversion rate (FCONV) as defined by TADC and NSH */
#define ADC_PP_MSR_125K          0x00000001               /* 125 ksps */
#define ADC_PP_MSR_250K          0x00000003               /* 250 ksps */
#define ADC_PP_MSR_500K          0x00000005               /* 500 ksps */
#define ADC_PP_MSR_1M            0x00000007               /* 1 Msps */

/* Bit fields in the TIVA_ADC_PC register. */

#define ADC_PC_SR_MASK           0x0000000F               /* ADC Sample Rate */
#define ADC_PC_MCR_MASK          0x0000000F               /* Conversion Rate */
#define ADC_PC_SR_125K           0x00000001               /* 125 ksps */
#define ADC_PC_SR_250K           0x00000003               /* 250 ksps */
#define ADC_PC_SR_500K           0x00000005               /* 500 ksps */
#define ADC_PC_SR_1M             0x00000007               /* 1 Msps */
#define ADC_PC_MCR_1_8           0x00000001               /* Eighth conversion rate. After a conversion completes, the logic pauses for 112 TADC periods before starting the next conversion */
#define ADC_PC_MCR_1_4           0x00000003               /* Quarter conversion rate. After a conversion completes, the logic pauses for 48 TADC periods before starting the next conversion */
#define ADC_PC_MCR_1_2           0x00000005               /* Half conversion rate. After a conversion completes, the logic pauses for 16 TADC periods before starting the next conversion */
#define ADC_PC_MCR_FULL          0x00000007               /* Full conversion rate (FCONV) as defined by TADC and NSH */

/* Bit fields in the TIVA_ADC_CC register. */

#define ADC_CC_CLKDIV_MASK       (0x3F0)                  /* PLL VCO Clock Divisor */
#define ADC_CC_CS_MASK           (0x00F)                  /* ADC Clock Source */
#  define ADC_CC_CLKDIV_SHIFT    4
#define ADC_CC_CS_SYSPLL         (0x000)                  /* PLL VCO divided by CLKDIV */
#define ADC_CC_CS_PIOSC          (0x001)                  /* PIOSC */
#define ADC_CC_CS_MOSC           (0x002)                  /* MOSC */

#endif // __ARCH_ARM_SRC_TIVA_CHIP_TIVA_ADC_H
