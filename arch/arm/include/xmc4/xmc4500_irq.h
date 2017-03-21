/*****************************************************************************
 * arch/arm/include/xmc4/xmc4500_.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

/* This file should never be included directed but, rather, only indirectly
 * through nuttx/irq.h
 */

#ifndef xmc4__ARCH_ARM_INCLUDE_XMC4_XM4500_IRQ_H
#define xmc4__ARCH_ARM_INCLUDE_XMC4_XM4500_IRQ_H

/*****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/*****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IRQ numbers.  The IRQ number corresponds vector number and hence map
 * directly to bits in the NVIC.  This does, however, waste several words of
 * memory in the IRQ to handle mapping tables.
 *
 * Processor Exceptions (vectors 0-15).  These common definitions can be found
 * in the file nuttx/arch/arm/include/kinets/irq.h which includes this file
 *
 * External interrupts (vectors >= 16)
 *
 * Acronyms:
 *  ADC   - Analog to Digital Converter
 *  CCU   - Capture Compare Unit
 *  DAC   - Digital to Analog Converter
 *  DSD   - Delta Sigmoid Demodulator
 *  ERU   - External Request Unit
 *  FCE   - Flexible CRC Engine
 *  GPDMA - General Purpose DMA
 *  LEDTS - LED and Touch Sense Control Unit
 *  PMU   - Program Management Unit
 *  POSIF - Position Interface
 *  SDMMC - Multi Media Card Interface
 *  USB   - Universal Serial Bus
 *  USCI  - Universal Serial Interface
 */

#define XMC4_IRQ_SCU        (XMC4_IRQ_FIRST+0)   /*   0: System Control */
#define XMC4_IRQ_ERU0_SR0   (XMC4_IRQ_FIRST+1)   /*   1: ERU0, SR0 */
#define XMC4_IRQ_ERU0_SR1   (XMC4_IRQ_FIRST+2)   /*   2: ERU0, SR1 */
#define XMC4_IRQ_ERU0_SR2   (XMC4_IRQ_FIRST+3)   /*   3: ERU0, SR2 */
#define XMC4_IRQ_ERU0_SR3   (XMC4_IRQ_FIRST+4)   /*   4: ERU0, SR3 */
#define XMC4_IRQ_ERU1_SR0   (XMC4_IRQ_FIRST+5)   /*   5: ERU1, SR0 */
#define XMC4_IRQ_ERU1_SR1   (XMC4_IRQ_FIRST+6)   /*   6: ERU1, SR1 */
#define XMC4_IRQ_ERU1_SR2   (XMC4_IRQ_FIRST+7)   /*   7: ERU1, SR2 */
#define XMC4_IRQ_ERU1_SR3   (XMC4_IRQ_FIRST+8)   /*   8: ERU1, SR3 */
#define XMC4_IRQ_RESVD009   (XMC4_IRQ_FIRST+9)   /*   9: Reserved */
#define XMC4_IRQ_RESVD010   (XMC4_IRQ_FIRST+10)  /*  10: Reserved */
#define XMC4_IRQ_RESVD011   (XMC4_IRQ_FIRST+11)  /*  11: Reserved */
#define XMC4_IRQ_PMU1_SR0   (XMC4_IRQ_FIRST+12)  /*  12: PMU, SR0 */
#define XMC4_IRQ_RESVD011   (XMC4_IRQ_FIRST+13)  /*  13: Reserved */
#define XMC4_IRQ_VADC_COSR0 (XMC4_IRQ_FIRST+14)  /*  14: ADC Common Block 0 */
#define XMC4_IRQ_VADC_COSR1 (XMC4_IRQ_FIRST+15)  /*  15: ADC Common Block 1 */
#define XMC4_IRQ_VADC_COSR2 (XMC4_IRQ_FIRST+16)  /*  16: ADC Common Block 2 */
#define XMC4_IRQ_VADC_COSR3 (XMC4_IRQ_FIRST+17)  /*  17: ADC Common Block 3 */
#define XMC4_IRQ_VADC_GOSR0 (XMC4_IRQ_FIRST+18)  /*  18: ADC Group 0, SR0 */
#define XMC4_IRQ_VADC_GOSR1 (XMC4_IRQ_FIRST+19)  /*  19: ADC Group 0, SR1 */
#define XMC4_IRQ_VADC_GOSR2 (XMC4_IRQ_FIRST+20)  /*  20: ADC Group 0, SR2 */
#define XMC4_IRQ_VADC_GOSR3 (XMC4_IRQ_FIRST+21)  /*  21: ADC Group 0, SR3 */
#define XMC4_IRQ_VADC_G1SR0 (XMC4_IRQ_FIRST+22)  /*  22: ADC Group 1, SR0 */
#define XMC4_IRQ_VADC_G1SR1 (XMC4_IRQ_FIRST+23)  /*  23: ADC Group 1, SR1 */
#define XMC4_IRQ_VADC_G1SR2 (XMC4_IRQ_FIRST+24)  /*  24: ADC Group 1, SR2 */
#define XMC4_IRQ_VADC_G1SR3 (XMC4_IRQ_FIRST+25)  /*  25: ADC Group 1, SR3 */
#define XMC4_IRQ_VADC_G2SR0 (XMC4_IRQ_FIRST+26)  /*  26: ADC Group 2, SR0 */
#define XMC4_IRQ_VADC_G2SR1 (XMC4_IRQ_FIRST+27)  /*  27: ADC Group 2, SR1 */
#define XMC4_IRQ_VADC_G2SR2 (XMC4_IRQ_FIRST+28)  /*  28: ADC Group 2, SR2 */
#define XMC4_IRQ_VADC_G2SR3 (XMC4_IRQ_FIRST+29)  /*  29: ADC Group 2, SR3 */
#define XMC4_IRQ_VADC_G3SR0 (XMC4_IRQ_FIRST+30)  /*  30: ADC Group 3, SR0 */
#define XMC4_IRQ_VADC_G3SR1 (XMC4_IRQ_FIRST+31)  /*  31: ADC Group 3, SR1 */
#define XMC4_IRQ_VADC_G3SR2 (XMC4_IRQ_FIRST+32)  /*  32: ADC Group 3, SR2 */
#define XMC4_IRQ_VADC_G3SR3 (XMC4_IRQ_FIRST+33)  /*  33: ADC Group 3, SR3 */
#define XMC4_IRQ_DSD_SRM0   (XMC4_IRQ_FIRST+34)  /*  34: DSD Main, SRM0 */
#define XMC4_IRQ_DSD_SRM1   (XMC4_IRQ_FIRST+35)  /*  35: DSD Main, SRM1 */
#define XMC4_IRQ_DSD_SRM2   (XMC4_IRQ_FIRST+36)  /*  36: DSD Main, SRM2 */
#define XMC4_IRQ_DSD_SRM3   (XMC4_IRQ_FIRST+37)  /*  37: DSD Main, SRM3 */
#define XMC4_IRQ_DSD_SRA0   (XMC4_IRQ_FIRST+38)  /*  38: DSD Auxiliary, SRA0 */
#define XMC4_IRQ_DSD_SRA1   (XMC4_IRQ_FIRST+39)  /*  39: DSD Auxiliary, SRA1 */
#define XMC4_IRQ_DSD_SRA2   (XMC4_IRQ_FIRST+40)  /*  40: DSD Auxiliary, SRA2 */
#define XMC4_IRQ_DSD_SRA3   (XMC4_IRQ_FIRST+41)  /*  41: DSD Auxiliary, SRA3 */
#define XMC4_IRQ_DAC_SR0    (XMC4_IRQ_FIRST+42)  /*  42: DAC, SR0 */
#define XMC4_IRQ_DAC_SR1    (XMC4_IRQ_FIRST+43)  /*  43: DAC, SR1 */
#define XMC4_IRQ_CCU40_SR0  (XMC4_IRQ_FIRST+44)  /*  44: CCU4 Module 0, SR0 */
#define XMC4_IRQ_CCU40_SR1  (XMC4_IRQ_FIRST+45)  /*  45: CCU4 Module 0, SR1 */
#define XMC4_IRQ_CCU40_SR2  (XMC4_IRQ_FIRST+46)  /*  46: CCU4 Module 0, SR2 */
#define XMC4_IRQ_CCU40_SR3  (XMC4_IRQ_FIRST+47)  /*  47: CCU4 Module 0, SR3 */
#define XMC4_IRQ_CCU41_SR0  (XMC4_IRQ_FIRST+48)  /*  48: CCU4 Module 1, SR0 */
#define XMC4_IRQ_CCU41_SR1  (XMC4_IRQ_FIRST+49)  /*  49: CCU4 Module 1, SR1 */
#define XMC4_IRQ_CCU41_SR2  (XMC4_IRQ_FIRST+50)  /*  50: CCU4 Module 1, SR2 */
#define XMC4_IRQ_CCU41_SR3  (XMC4_IRQ_FIRST+51)  /*  51: CCU4 Module 1, SR3 */
#define XMC4_IRQ_CCU42_SR0  (XMC4_IRQ_FIRST+52)  /*  52: CCU4 Module 2, SR0 */
#define XMC4_IRQ_CCU42_SR1  (XMC4_IRQ_FIRST+53)  /*  53: CCU4 Module 2, SR1 */
#define XMC4_IRQ_CCU42_SR2  (XMC4_IRQ_FIRST+54)  /*  54: CCU4 Module 2, SR2 */
#define XMC4_IRQ_CCU42_SR3  (XMC4_IRQ_FIRST+55)  /*  55: CCU4 Module 2, SR3 */
#define XMC4_IRQ_CCU43_SR0  (XMC4_IRQ_FIRST+56)  /*  56: CCU4 Module 3, SR0 */
#define XMC4_IRQ_CCU43_SR1  (XMC4_IRQ_FIRST+57)  /*  57: CCU4 Module 3, SR1 */
#define XMC4_IRQ_CCU43_SR2  (XMC4_IRQ_FIRST+58)  /*  58: CCU4 Module 3, SR2 */
#define XMC4_IRQ_CCU43_SR3  (XMC4_IRQ_FIRST+59)  /*  59: CCU4 Module 3, SR3 */
#define XMC4_IRQ_CCU80_SR0  (XMC4_IRQ_FIRST+60)  /*  60: CCU8 Module 0, SR0 */
#define XMC4_IRQ_CCU80_SR1  (XMC4_IRQ_FIRST+61)  /*  61: CCU8 Module 0, SR1 */
#define XMC4_IRQ_CCU80_SR2  (XMC4_IRQ_FIRST+62)  /*  62: CCU8 Module 0, SR2 */
#define XMC4_IRQ_CCU80_SR3  (XMC4_IRQ_FIRST+63)  /*  63: CCU8 Module 0, SR3 */
#define XMC4_IRQ_CCU81_SR0  (XMC4_IRQ_FIRST+64)  /*  64: CCU8 Module 1, SR0 */
#define XMC4_IRQ_CCU81_SR1  (XMC4_IRQ_FIRST+65)  /*  65: CCU8 Module 1, SR1 */
#define XMC4_IRQ_CCU81_SR2  (XMC4_IRQ_FIRST+66)  /*  66: CCU8 Module 1, SR2 */
#define XMC4_IRQ_CCU81_SR3  (XMC4_IRQ_FIRST+67)  /*  67: CCU8 Module 1, SR3 */
#define XMC4_IRQ_POSIF0_SR0 (XMC4_IRQ_FIRST+68)  /*  68: POSIF Module 0, SR0 */
#define XMC4_IRQ_POSIF0_SR1 (XMC4_IRQ_FIRST+69)  /*  69: POSIF Module 0, SR1 */
#define XMC4_IRQ_POSIF1_SR0 (XMC4_IRQ_FIRST+70)  /*  70: POSIF Module 1, SR0 */
#define XMC4_IRQ_POSIF1_SR1 (XMC4_IRQ_FIRST+71)  /*  71: POSIF Module 1, SR1 */
#define XMC4_IRQ_RESVD072   (XMC4_IRQ_FIRST+72)  /*  72: Reserved */
#define XMC4_IRQ_RESVD073   (XMC4_IRQ_FIRST+73)  /*  73: Reserved */
#define XMC4_IRQ_RESVD074   (XMC4_IRQ_FIRST+74)  /*  74: Reserved */
#define XMC4_IRQ_RESVD075   (XMC4_IRQ_FIRST+75)  /*  75: Reserved */
#define XMC4_IRQ_CAN_SR0    (XMC4_IRQ_FIRST+76)  /*  76: MultiCAN, SR0 */
#define XMC4_IRQ_CAN_SR1    (XMC4_IRQ_FIRST+77)  /*  77: MultiCAN, SR1 */
#define XMC4_IRQ_CAN_SR2    (XMC4_IRQ_FIRST+78)  /*  78: MultiCAN, SR2 */
#define XMC4_IRQ_CAN_SR3    (XMC4_IRQ_FIRST+79)  /*  79: MultiCAN, SR3 */
#define XMC4_IRQ_CAN_SR4    (XMC4_IRQ_FIRST+80)  /*  80: MultiCAN, SR4 */
#define XMC4_IRQ_CAN_SR5    (XMC4_IRQ_FIRST+81)  /*  81: MultiCAN, SR5 */
#define XMC4_IRQ_CAN_SR6    (XMC4_IRQ_FIRST+82)  /*  82: MultiCAN, SR6 */
#define XMC4_IRQ_CAN_SR7    (XMC4_IRQ_FIRST+83)  /*  83: MultiCAN, SR7 */
#define XMC4_IRQ_USIC0_SR0  (XMC4_IRQ_FIRST+84)  /*  84: USIC0 Channel, SR0 */
#define XMC4_IRQ_USIC0_SR1  (XMC4_IRQ_FIRST+85)  /*  85: USIC0 Channel, SR1 */
#define XMC4_IRQ_USIC0_SR2  (XMC4_IRQ_FIRST+86)  /*  86: USIC0 Channel, SR2 */
#define XMC4_IRQ_USIC0_SR3  (XMC4_IRQ_FIRST+87)  /*  87: USIC0 Channel, SR3 */
#define XMC4_IRQ_USIC0_SR4  (XMC4_IRQ_FIRST+88)  /*  88: USIC0 Channel, SR4 */
#define XMC4_IRQ_USIC0_SR5  (XMC4_IRQ_FIRST+89)  /*  89: USIC0 Channel, SR5 */
#define XMC4_IRQ_USIC1_SR0  (XMC4_IRQ_FIRST+90)  /*  90: USIC1 Channel, SR0 */
#define XMC4_IRQ_USIC1_SR1  (XMC4_IRQ_FIRST+91)  /*  91: USIC1 Channel, SR1 */
#define XMC4_IRQ_USIC1_SR2  (XMC4_IRQ_FIRST+92)  /*  92: USIC1 Channel, SR2 */
#define XMC4_IRQ_USIC1_SR3  (XMC4_IRQ_FIRST+93)  /*  93: USIC1 Channel, SR3 */
#define XMC4_IRQ_USIC1_SR4  (XMC4_IRQ_FIRST+94)  /*  94: USIC1 Channel, SR4 */
#define XMC4_IRQ_USIC1_SR5  (XMC4_IRQ_FIRST+95)  /*  95: USIC1 Channel, SR5 */
#define XMC4_IRQ_USIC2_SR0  (XMC4_IRQ_FIRST+96)  /*  96: USIC1 Channel, SR0 */
#define XMC4_IRQ_USIC2_SR1  (XMC4_IRQ_FIRST+97)  /*  97: USIC1 Channel, SR1 */
#define XMC4_IRQ_USIC2_SR2  (XMC4_IRQ_FIRST+98)  /*  98: USIC1 Channel, SR2 */
#define XMC4_IRQ_USIC2_SR3  (XMC4_IRQ_FIRST+99)  /*  99: USIC1 Channel, SR3 */
#define XMC4_IRQ_USIC2_SR4  (XMC4_IRQ_FIRST+100) /* 100: USIC1 Channel, SR4 */
#define XMC4_IRQ_USIC2_SR5  (XMC4_IRQ_FIRST+101) /* 101: USIC1 Channel, SR5 */
#define XMC4_IRQ_LEDTS0_SR0 (XMC4_IRQ_FIRST+102) /* 102: LEDTS0, SR0 */
#define XMC4_IRQ_RESVD103   (XMC4_IRQ_FIRST+103) /* 103: Reserved */
#define XMC4_IRQ_FCR_SR0    (XMC4_IRQ_FIRST+104) /* 102: FCE, SR0 */
#define XMC4_IRQ_GPCMA0_SR0 (XMC4_IRQ_FIRST+105) /* 105: GPDMA0, SR0 */
#define XMC4_IRQ_SDMMC_SR0  (XMC4_IRQ_FIRST+106) /* 106: SDMMC, SR0 */
#define XMC4_IRQ_USB0_SR0   (XMC4_IRQ_FIRST+107) /* 107: USB, SR0 */
#define XMC4_IRQ_ETH0_SR0   (XMC4_IRQ_FIRST+108) /* 108: Ethernet, module 0, SR0 */
#define XMC4_IRQ_ECAT0_SR0  (XMC4_IRQ_FIRST+109) /* 109: EtherCAT, module 0, SR0 */
#define XMC4_IRQ_GPCMA1_SR0 (XMC4_IRQ_FIRST+110) /* 110: GPDMA1, SR0 */
#define XMC4_IRQ_RESVD111   (XMC4_IRQ_FIRST+111) /* 111: Reserved */

#define NR_INTERRUPTS       112                  /* 112 Non core IRQs*/
#define NR_VECTORS          (XMC4_IRQ_FIRST+NR_INTERRUPTS) /* 118 vectors */

/* GPIO IRQ interrupts -- To be provided */

#define NR_IRQS             NR_VECTORS

/*****************************************************************************
 * Public Types
 ****************************************************************************/

/*****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/*****************************************************************************
 * Public Functions
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* xmc4__ARCH_ARM_INCLUDE_XMC4_XM4500_IRQ_H */
