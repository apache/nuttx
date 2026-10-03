/****************************************************************************
 * arch/mips/include/pic32mz/irq_pic32mzw1.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

/* This file should never be included directly but, rather, only indirectly
 * through nuttx/irq.h
 *
 * PIC32MZ-W1's interrupt vector assignment is NOT a renumbered/truncated
 * version of EC/EF's - it is a different, non-contiguous table (plenty of
 * reserved/unimplemented vector numbers interspersed), transcribed here
 * verbatim from the Microchip PIC32MZ-W_DFP (Apache-2.0),
 * atdf/WFI32E01.atdf <interrupts> list. Do not assume any EC/EF vector
 * number carries over.
 */

#ifndef __ARCH_MIPS_INCLUDE_PIC32MZ_IRQ_PIC32MZW1_H
#define __ARCH_MIPS_INCLUDE_PIC32MZ_IRQ_PIC32MZW1_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Interrupt vector numbers.  These should be used to attach to interrupts
 * and to change interrupt priorities.
 */

#define PIC32MZ_IRQ_CT           0 /* Vector: 0,   Core Timer Interrupt */
#define PIC32MZ_IRQ_CS0          1 /* Vector: 1,   Core Software Interrupt 0 */
#define PIC32MZ_IRQ_CS1          2 /* Vector: 2,   Core Software Interrupt 1 */
#define PIC32MZ_IRQ_INT0         3 /* Vector: 3,   External Interrupt 0 */
#define PIC32MZ_IRQ_T1           4 /* Vector: 4,   Timer 1 */
#define PIC32MZ_IRQ_ICE1         5 /* Vector: 5,   Input Capture 1 Error */
#define PIC32MZ_IRQ_IC1          6 /* Vector: 6,   Input Capture 1 */
#define PIC32MZ_IRQ_OC1          7 /* Vector: 7,   Output Compare 1 */
#define PIC32MZ_IRQ_INT1         8 /* Vector: 8,   External Interrupt 1 */
#define PIC32MZ_IRQ_T2           9 /* Vector: 9,   Timer 2 */

#define PIC32MZ_IRQ_ICE2        10 /* Vector: 10,  Input Capture 2 Error */
#define PIC32MZ_IRQ_IC2         11 /* Vector: 11,  Input Capture 2 */
#define PIC32MZ_IRQ_OC2         12 /* Vector: 12,  Output Compare 2 */
#define PIC32MZ_IRQ_INT2        13 /* Vector: 13,  External Interrupt 2 */
#define PIC32MZ_IRQ_T3          14 /* Vector: 14,  Timer 3 */
#define PIC32MZ_IRQ_ICE3        15 /* Vector: 15,  Input Capture 3 Error */
#define PIC32MZ_IRQ_IC3         16 /* Vector: 16,  Input Capture 3 */
#define PIC32MZ_IRQ_OC3         17 /* Vector: 17,  Output Compare 3 */
#define PIC32MZ_IRQ_INT3        18 /* Vector: 18,  External Interrupt 3 */
#define PIC32MZ_IRQ_T4          19 /* Vector: 19,  Timer 4 */

#define PIC32MZ_IRQ_ICE4        20 /* Vector: 20,  Input Capture 4 Error */
#define PIC32MZ_IRQ_IC4         21 /* Vector: 21,  Input Capture 4 */
#define PIC32MZ_IRQ_OC4         22 /* Vector: 22,  Output Compare 4 */
#define PIC32MZ_IRQ_INT4        23 /* Vector: 23,  External Interrupt 4 */
#define PIC32MZ_IRQ_T5          24 /* Vector: 24,  Timer 5 */

/* Vectors 25-29 reserved/unimplemented */

#define PIC32MZ_IRQ_FCE         30 /* Vector: 30,  Flash Control Event */
#define PIC32MZ_IRQ_PREF        31 /* Vector: 31,  Prefetch Module */
#define PIC32MZ_IRQ_PFWCRC      32 /* Vector: 32,  Prefetch Module CRC */
#define PIC32MZ_IRQ_RTCC        33 /* Vector: 33,  Real-Time Clock and Calendar */
#define PIC32MZ_IRQ_USB         34 /* Vector: 34,  USB */

#define PIC32MZ_IRQ_SPI1F       35 /* Vector: 35,  SPI1 Fault */
#define PIC32MZ_IRQ_SPI1RX      36 /* Vector: 36,  SPI1 Receive Done */
#define PIC32MZ_IRQ_SPI1TX      37 /* Vector: 37,  SPI1 Transfer Done */
#define PIC32MZ_IRQ_U1E         38 /* Vector: 38,  UART1 Fault */
#define PIC32MZ_IRQ_U1RX        39 /* Vector: 39,  UART1 Receive Done */
#define PIC32MZ_IRQ_U1TX        40 /* Vector: 40,  UART1 Transfer Done */
#define PIC32MZ_IRQ_I2C1COL     41 /* Vector: 41,  I2C1 Bus Collision Event */
#define PIC32MZ_IRQ_I2C1S       42 /* Vector: 42,  I2C1 Slave Event */
#define PIC32MZ_IRQ_I2C1M       43 /* Vector: 43,  I2C1 Master Event */
#define PIC32MZ_IRQ_CNA         44 /* Vector: 44,  PORTA Change Notice */
#define PIC32MZ_IRQ_CNB         45 /* Vector: 45,  PORTB Change Notice */
#define PIC32MZ_IRQ_CNC         46 /* Vector: 46,  PORTC Change Notice */
#define PIC32MZ_IRQ_CNK         47 /* Vector: 47,  PORTK Change Notice */

/* Vectors 48-52 reserved/unimplemented */

#define PIC32MZ_IRQ_SPI2F       53 /* Vector: 53,  SPI2 Fault */
#define PIC32MZ_IRQ_SPI2RX      54 /* Vector: 54,  SPI2 Receive Done */
#define PIC32MZ_IRQ_SPI2TX      55 /* Vector: 55,  SPI2 Transfer Done */
#define PIC32MZ_IRQ_U2E         56 /* Vector: 56,  UART2 Fault */
#define PIC32MZ_IRQ_U2RX        57 /* Vector: 57,  UART2 Receive Done */
#define PIC32MZ_IRQ_U2TX        58 /* Vector: 58,  UART2 Transfer Done */
#define PIC32MZ_IRQ_I2C2COL     59 /* Vector: 59,  I2C2 Bus Collision Event */
#define PIC32MZ_IRQ_I2C2S       60 /* Vector: 60,  I2C2 Slave Event */
#define PIC32MZ_IRQ_I2C2M       61 /* Vector: 61,  I2C2 Master Event */
#define PIC32MZ_IRQ_U3E         62 /* Vector: 62,  UART3 Fault (not yet supported, see pic32mzw1_memorymap.h) */
#define PIC32MZ_IRQ_U3RX        63 /* Vector: 63,  UART3 Receive Done (not yet supported) */
#define PIC32MZ_IRQ_U3TX        64 /* Vector: 64,  UART3 Transfer Done (not yet supported) */

/* Vectors 65-67 reserved/unimplemented */

#define PIC32MZ_IRQ_DMA0        68 /* Vector: 68,  DMA Channel 0 */
#define PIC32MZ_IRQ_DMA1        69 /* Vector: 69,  DMA Channel 1 */
#define PIC32MZ_IRQ_DMA2        70 /* Vector: 70,  DMA Channel 2 */
#define PIC32MZ_IRQ_DMA3        71 /* Vector: 71,  DMA Channel 3 */
#define PIC32MZ_IRQ_DMA4        72 /* Vector: 72,  DMA Channel 4 */
#define PIC32MZ_IRQ_DMA5        73 /* Vector: 73,  DMA Channel 5 */
#define PIC32MZ_IRQ_DMA6        74 /* Vector: 74,  DMA Channel 6 */
#define PIC32MZ_IRQ_DMA7        75 /* Vector: 75,  DMA Channel 7 */
#define PIC32MZ_IRQ_T6          76 /* Vector: 76,  Timer 6 */

/* Vectors 77-79 reserved/unimplemented */

#define PIC32MZ_IRQ_T7          80 /* Vector: 80,  Timer 7 */

/* Vectors 81-82 reserved/unimplemented */

#define PIC32MZ_IRQ_RFSMC       83  /* Vector: 83,  Wi-Fi SMC */
#define PIC32MZ_IRQ_RFMAC       84  /* Vector: 84,  Wi-Fi MAC */
#define PIC32MZ_IRQ_CTR1EVENT   85  /* Vector: 85,  CTR1 Event */
#define PIC32MZ_IRQ_RFTM0       86  /* Vector: 86,  Wi-Fi Timer 0 */
#define PIC32MZ_IRQ_RFTM1       87  /* Vector: 87,  Wi-Fi Timer 1 */
#define PIC32MZ_IRQ_RFTM2       88  /* Vector: 88,  Wi-Fi Timer 2 */
#define PIC32MZ_IRQ_RFTM3       89  /* Vector: 89,  Wi-Fi Timer 3 */
#define PIC32MZ_IRQ_CTR1TRG     90  /* Vector: 90,  CTR1 Trigger */
#define PIC32MZ_IRQ_RFWCOE      91  /* Vector: 91,  Wi-Fi Wake-up/Coexistence */
#define PIC32MZ_IRQ_ADC         92  /* Vector: 92,  ADC */

/* Vector 93 reserved/unimplemented */

#define PIC32MZ_IRQ_ADCDC1      94  /* Vector: 94,  ADC Digital Comparator 1 */
#define PIC32MZ_IRQ_ADCDC2      95  /* Vector: 95,  ADC Digital Comparator 2 */
#define PIC32MZ_IRQ_ADCDF1      96  /* Vector: 96,  ADC Digital Filter 1 */
#define PIC32MZ_IRQ_ADCDF2      97  /* Vector: 97,  ADC Digital Filter 2 */

/* Vectors 98-99 reserved/unimplemented */

#define PIC32MZ_IRQ_ADCFAULT    100 /* Vector: 100, ADC Fault */
#define PIC32MZ_IRQ_ADCEOS      101 /* Vector: 101, ADC End of Scan */
#define PIC32MZ_IRQ_ADCARDY     102 /* Vector: 102, ADC Analog-to-Digital Ready */
#define PIC32MZ_IRQ_ADCURDY     103 /* Vector: 103, ADC Digital Filter Ready */
#define PIC32MZ_IRQ_ADCDMA      104 /* Vector: 104, ADC DMA */

/* Vector 105 reserved/unimplemented */

#define PIC32MZ_IRQ_ADCDATA0    106 /* Vector: 106, ADC Data 0 */
#define PIC32MZ_IRQ_ADCDATA1    107 /* Vector: 107, ADC Data 1 */
#define PIC32MZ_IRQ_ADCDATA2    108 /* Vector: 108, ADC Data 2 */
#define PIC32MZ_IRQ_ADCDATA3    109 /* Vector: 109, ADC Data 3 */
#define PIC32MZ_IRQ_ADCDATA4    110 /* Vector: 110, ADC Data 4 */
#define PIC32MZ_IRQ_ADCDATA5    111 /* Vector: 111, ADC Data 5 */
#define PIC32MZ_IRQ_ADCDATA6    112 /* Vector: 112, ADC Data 6 */
#define PIC32MZ_IRQ_ADCDATA7    113 /* Vector: 113, ADC Data 7 */
#define PIC32MZ_IRQ_ADCDATA8    114 /* Vector: 114, ADC Data 8 */
#define PIC32MZ_IRQ_ADCDATA9    115 /* Vector: 115, ADC Data 9 */

#define PIC32MZ_IRQ_ADCDATA10   116 /* Vector: 116, ADC Data 10 */
#define PIC32MZ_IRQ_ADCDATA11   117 /* Vector: 117, ADC Data 11 */
#define PIC32MZ_IRQ_ADCDATA12   118 /* Vector: 118, ADC Data 12 */
#define PIC32MZ_IRQ_ADCDATA13   119 /* Vector: 119, ADC Data 13 */
#define PIC32MZ_IRQ_ADCDATA14   120 /* Vector: 120, ADC Data 14 */
#define PIC32MZ_IRQ_ADCDATA15   121 /* Vector: 121, ADC Data 15 */
#define PIC32MZ_IRQ_ADCDATA16   122 /* Vector: 122, ADC Data 16 */
#define PIC32MZ_IRQ_ADCDATA17   123 /* Vector: 123, ADC Data 17 */
#define PIC32MZ_IRQ_ADCDATA18   124 /* Vector: 124, ADC Data 18 */
#define PIC32MZ_IRQ_ADCDATA19   125 /* Vector: 125, ADC Data 19 */

#define PIC32MZ_IRQ_ADCDATA20   126 /* Vector: 126, ADC Data 20 */
#define PIC32MZ_IRQ_ADCDATA21   127 /* Vector: 127, ADC Data 21 */
#define PIC32MZ_IRQ_ADCDATA22   128 /* Vector: 128, ADC Data 22 */
#define PIC32MZ_IRQ_ADCDATA23   129 /* Vector: 129, ADC Data 23 */

/* Vectors 130-141 reserved/unimplemented */

#define PIC32MZ_IRQ_CAN1        142 /* Vector: 142, CAN1 */
#define PIC32MZ_IRQ_CAN2RX      143 /* Vector: 143, CAN2 RX (CAN2 not present on WFI32E01) */
#define PIC32MZ_IRQ_CAN2TX      144 /* Vector: 144, CAN2 TX (CAN2 not present on WFI32E01) */
#define PIC32MZ_IRQ_CAN2MISC    145 /* Vector: 145, CAN2 Misc (CAN2 not present on WFI32E01) */

/* Vectors 146-149 reserved/unimplemented */

#define PIC32MZ_IRQ_SQI1        150 /* Vector: 150, SQI1 */

/* Vector 151 reserved/unimplemented */

#define PIC32MZ_IRQ_PTG0STEP    152 /* Vector: 152, PTG0 Step */
#define PIC32MZ_IRQ_PTG0WDT     153 /* Vector: 153, PTG0 Watchdog */
#define PIC32MZ_IRQ_PTG0TRG0    154 /* Vector: 154, PTG0 Trigger 0 */
#define PIC32MZ_IRQ_PTG0TRG1    155 /* Vector: 155, PTG0 Trigger 1 */
#define PIC32MZ_IRQ_PTG0TRG2    156 /* Vector: 156, PTG0 Trigger 2 */
#define PIC32MZ_IRQ_PTG0TRG3    157 /* Vector: 157, PTG0 Trigger 3 */

/* Vectors 158-161 reserved/unimplemented */

#define PIC32MZ_IRQ_CORECOUNT   162 /* Vector: 162, Core Performance Counter */
#define PIC32MZ_IRQ_COREFDC     163 /* Vector: 163, Core Fast Debug Channel */
#define PIC32MZ_IRQ_CRYPTO      164 /* Vector: 164, Crypto Engine */
#define PIC32MZ_IRQ_ETH         165 /* Vector: 165, Ethernet MAC */
#define PIC32MZ_IRQ_CRYPTO1     166 /* Vector: 166, Crypto Engine 1 */
#define PIC32MZ_IRQ_CRYPTO1F    167 /* Vector: 167, Crypto Engine 1 Fault */
#define PIC32MZ_IRQ_CVDEVENT    168 /* Vector: 168, Capacitive Voltage Divider Event */

#define NR_IRQS                 169

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_MIPS_INCLUDE_PIC32MZ_IRQ_PIC32MZW1_H */
