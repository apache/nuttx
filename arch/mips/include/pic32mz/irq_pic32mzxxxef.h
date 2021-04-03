/****************************************************************************
 * arch/mips/include/pic32mz/irq_pic32mzxxxef.h
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
 */

#ifndef __ARCH_MIPS_INCLUDE_PIC32MZ_IRQ_PIC32MZXXXEF_H
#define __ARCH_MIPS_INCLUDE_PIC32MZ_IRQ_PIC32MZXXXEF_H

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
#define PIC32MZ_IRQ_ICE5        25 /* Vector: 25,  Input Capture 5 Error */
#define PIC32MZ_IRQ_IC5         26 /* Vector: 26,  Input Capture 5 */
#define PIC32MZ_IRQ_OC5         27 /* Vector: 27,  Output Compare 5 */
#define PIC32MZ_IRQ_T6          28 /* Vector: 28,  Timer 6 */
#define PIC32MZ_IRQ_ICE6        29 /* Vector: 29,  Input Capture 6 Error */

#define PIC32MZ_IRQ_IC6         30 /* Vector: 30,  Input Capture 6 */
#define PIC32MZ_IRQ_OC6         31 /* Vector: 31,  Output Compare 6 */
#define PIC32MZ_IRQ_T7          32 /* Vector: 32,  Timer 7 */
#define PIC32MZ_IRQ_ICE7        33 /* Vector: 33,  Input Capture 7 Error */
#define PIC32MZ_IRQ_IC7         34 /* Vector: 34,  Input Capture 7 */
#define PIC32MZ_IRQ_OC7         35 /* Vector: 35,  Output Compare 7 */
#define PIC32MZ_IRQ_T8          36 /* Vector: 36,  Timer 8 */
#define PIC32MZ_IRQ_ICE8        37 /* Vector: 37,  Input Capture 8 Error */
#define PIC32MZ_IRQ_IC8         38 /* Vector: 38,  Input Capture 8 */
#define PIC32MZ_IRQ_OC8         39 /* Vector: 39,  Output Compare 8 */

#define PIC32MZ_IRQ_T9          40 /* Vector: 40,  Timer 9 */
#define PIC32MZ_IRQ_ICE9        41 /* Vector: 41,  Input Capture 9 Error */
#define PIC32MZ_IRQ_IC9         42 /* Vector: 42,  Input Capture 9 */
#define PIC32MZ_IRQ_OC9         43 /* Vector: 43,  Output Compare 9 */
#define PIC32MZ_IRQ_AD1         44 /* Vector: 44,  ADC1 Global Interrupt */
#define PIC32MZ_IRQ_AD1FIFO     45 /* Vector: 45,  ADC1 FIFO Data Ready Interrupt */
#define PIC32MZ_IRQ_AD1CMP1     46 /* Vector: 46,  ADC1 Digital Comparator 1 */
#define PIC32MZ_IRQ_AD1CMP2     47 /* Vector: 47,  ADC1 Digital Comparator 2 */
#define PIC32MZ_IRQ_AD1CMP3     48 /* Vector: 48,  ADC1 Digital Comparator 3 */
#define PIC32MZ_IRQ_AD1CMP4     49 /* Vector: 49,  ADC1 Digital Comparator 4 */

#define PIC32MZ_IRQ_AD1CMP5     50 /* Vector: 50,  ADC1 Digital Comparator 5 */
#define PIC32MZ_IRQ_AD1CMP6     51 /* Vector: 51,  ADC1 Digital Comparator 6 */
#define PIC32MZ_IRQ_AD1FLT1     52 /* Vector: 52,  ADC1 Digital Filter 1 */
#define PIC32MZ_IRQ_AD1FLT2     53 /* Vector: 53,  ADC1 Digital Filter 2 */
#define PIC32MZ_IRQ_AD1FLT3     54 /* Vector: 54,  ADC1 Digital Filter 3 */
#define PIC32MZ_IRQ_AD1FLT4     55 /* Vector: 55,  ADC1 Digital Filter 4 */
#define PIC32MZ_IRQ_AD1FLT5     56 /* Vector: 56,  ADC1 Digital Filter 5 */
#define PIC32MZ_IRQ_AD1FLT6     57 /* Vector: 57,  ADC1 Digital Filter 6 */
#define PIC32MZ_IRQ_AD1FAULT    59 /* Vector: 58,  ADC1 Fault */
#define PIC32MZ_IRQ_AD1DAT0     59 /* Vector: 59,  ADC1 Data 0 */

#define PIC32MZ_IRQ_AD1DAT1     60 /* Vector: 60,  ADC1 Data 1 */
#define PIC32MZ_IRQ_AD1DAT2     61 /* Vector: 61,  ADC1 Data 2 */
#define PIC32MZ_IRQ_AD1DAT3     62 /* Vector: 62,  ADC1 Data 3 */
#define PIC32MZ_IRQ_AD1DAT4     63 /* Vector: 63,  ADC1 Data 4 */
#define PIC32MZ_IRQ_AD1DAT5     64 /* Vector: 64,  ADC1 Data 5 */
#define PIC32MZ_IRQ_AD1DAT6     65 /* Vector: 65,  ADC1 Data 6 */
#define PIC32MZ_IRQ_AD1DAT7     66 /* Vector: 66,  ADC1 Data 7 */
#define PIC32MZ_IRQ_AD1DAT8     67 /* Vector: 67,  ADC1 Data 8 */
#define PIC32MZ_IRQ_AD1DAT9     68 /* Vector: 68,  ADC1 Data 9 */
#define PIC32MZ_IRQ_AD1DAT10    69 /* Vector: 69,  ADC1 Data 10 */

#define PIC32MZ_IRQ_AD1DAT11    70 /* Vector: 70,  ADC1 Data 11 */
#define PIC32MZ_IRQ_AD1DAT12    71 /* Vector: 71,  ADC1 Data 12 */
#define PIC32MZ_IRQ_AD1DAT13    72 /* Vector: 72,  ADC1 Data 13 */
#define PIC32MZ_IRQ_AD1DAT14    73 /* Vector: 73,  ADC1 Data 14 */
#define PIC32MZ_IRQ_AD1DAT15    74 /* Vector: 74,  ADC1 Data 15 */
#define PIC32MZ_IRQ_AD1DAT16    75 /* Vector: 75,  ADC1 Data 16 */
#define PIC32MZ_IRQ_AD1DAT17    76 /* Vector: 76,  ADC1 Data 17 */
#define PIC32MZ_IRQ_AD1DAT18    77 /* Vector: 77,  ADC1 Data 18 */
#define PIC32MZ_IRQ_AD1DAT19    78 /* Vector: 78,  ADC1 Data 19 */
#define PIC32MZ_IRQ_AD1DAT20    79 /* Vector: 79,  ADC1 Data 20 */

#define PIC32MZ_IRQ_AD1DAT21    80 /* Vector: 80,  ADC1 Data 21 */
#define PIC32MZ_IRQ_AD1DAT22    81 /* Vector: 81,  ADC1 Data 22 */
#define PIC32MZ_IRQ_AD1DAT23    82 /* Vector: 82,  ADC1 Data 23 */
#define PIC32MZ_IRQ_AD1DAT24    83 /* Vector: 83,  ADC1 Data 24 */
#define PIC32MZ_IRQ_AD1DAT25    84 /* Vector: 84,  ADC1 Data 25 */
#define PIC32MZ_IRQ_AD1DAT26    85 /* Vector: 85,  ADC1 Data 26 */
#define PIC32MZ_IRQ_AD1DAT27    86 /* Vector: 86,  ADC1 Data 27 */
#define PIC32MZ_IRQ_AD1DAT28    87 /* Vector: 87,  ADC1 Data 28 */
#define PIC32MZ_IRQ_AD1DAT29    88 /* Vector: 88,  ADC1 Data 29 */
#define PIC32MZ_IRQ_AD1DAT30    89 /* Vector: 89,  ADC1 Data 30 */

#define PIC32MZ_IRQ_AD1DAT31    90 /* Vector: 90,  ADC1 Data 31 */
#define PIC32MZ_IRQ_AD1DAT32    91 /* Vector: 91,  ADC1 Data 32 */
#define PIC32MZ_IRQ_AD1DAT33    92 /* Vector: 92,  ADC1 Data 33 */
#define PIC32MZ_IRQ_AD1DAT34    93 /* Vector: 93,  ADC1 Data 34 */
#define PIC32MZ_IRQ_AD1DAT35    94 /* Vector: 94,  ADC1 Data 35 */
#define PIC32MZ_IRQ_AD1DAT36    95 /* Vector: 95,  ADC1 Data 36 */
#define PIC32MZ_IRQ_AD1DAT37    96 /* Vector: 96,  ADC1 Data 37 */
#define PIC32MZ_IRQ_AD1DAT38    97 /* Vector: 97,  ADC1 Data 38 */
#define PIC32MZ_IRQ_AD1DAT39    98 /* Vector: 98,  ADC1 Data 39 */
#define PIC32MZ_IRQ_AD1DAT40    99 /* Vector: 99,  ADC1 Data 40 */

#define PIC32MZ_IRQ_AD1DAT41   100 /* Vector: 100, ADC1 Data 41 */
#define PIC32MZ_IRQ_AD1DAT42   101 /* Vector: 101, ADC1 Data 42 */
#define PIC32MZ_IRQ_AD1DAT43   102 /* Vector: 102, ADC1 Data 43 */
#define PIC32MZ_IRQ_AD1DAT44   103 /* Vector: 103, ADC1 Data 44 */
#define PIC32MZ_IRQ_COREPERF   104 /* Vector: 104, Core Performance Counter Interrupt */
#define PIC32MZ_IRQ_COREFDBG   105 /* Vector: 105, Core Fast Debug Channel Interrupt */
#define PIC32MZ_IRQ_BUSPROT    106 /* Vector: 106, System Bus Protection Violation */
#define PIC32MZ_IRQ_CTYPTO     107 /* Vector: 107, Crypto Engine Event */
                                   /* Vector: 108, Reserved */
#define PIC32MZ_IRQ_SPI1F      109 /* Vector: 109, SPI1 Fault */

#define PIC32MZ_IRQ_SPI1RX     110 /* Vector: 110, SPI1 Receive Done */
#define PIC32MZ_IRQ_SPI1TX     111 /* Vector: 111, SPI1 Transfer Done */
#define PIC32MZ_IRQ_U1E        112 /* Vector: 112, UART1 Fault */
#define PIC32MZ_IRQ_U1RX       113 /* Vector: 113, UART1 Receive Done */
#define PIC32MZ_IRQ_U1TX       114 /* Vector: 114, UART1 Transfer Done */
#define PIC32MZ_IRQ_I2C1COL    115 /* Vector: 115, I2C1 Bus Collision Event */
#define PIC32MZ_IRQ_I2C1S      116 /* Vector: 116, I2C1 Slave Event */
#define PIC32MZ_IRQ_I2C1M      117 /* Vector: 117, I2C1 Master Event */
#define PIC32MZ_IRQ_PORTA      118 /* Vector: 118, PORTA Input Change Interrupt */
#define PIC32MZ_IRQ_PORTB      119 /* Vector: 119, PORTB Input Change Interrupt */

#define PIC32MZ_IRQ_PORTC      120 /* Vector: 120, PORTC Input Change Interrupt */
#define PIC32MZ_IRQ_PORTD      121 /* Vector: 121, PORTD Input Change Interrupt */
#define PIC32MZ_IRQ_PORTE      122 /* Vector: 122, PORTE Input Change Interrupt */
#define PIC32MZ_IRQ_PORTF      123 /* Vector: 123, PORTF Input Change Interrupt */
#define PIC32MZ_IRQ_PORTG      124 /* Vector: 124, PORTG Input Change Interrupt */
#define PIC32MZ_IRQ_PORTH      125 /* Vector: 125, PORTH Input Change Interrupt */
#define PIC32MZ_IRQ_PORTJ      126 /* Vector: 126, PORTJ Input Change Interrupt */
#define PIC32MZ_IRQ_PORTK      127 /* Vector: 127, PORTK Input Change Interrupt */
#define PIC32MZ_IRQ_PMP        128 /* Vector: 128, Parallel Master Port */
#define PIC32MZ_IRQ_PMPE       129 /* Vector: 129, Parallel Master Port Error */

#define PIC32MZ_IRQ_CMP1       130 /* Vector: 130, Comparator 1 Interrupt */
#define PIC32MZ_IRQ_CMP2       131 /* Vector: 131, Comparator 2 Interrupt */
#define PIC32MZ_IRQ_USBGEN     132 /* Vector: 132, USB General Event */
#define PIC32MZ_IRQ_USBDMA     133 /* Vector: 133, USB DMA Event */
#define PIC32MZ_IRQ_DMA0       134 /* Vector: 134, DMA Channel 0 */
#define PIC32MZ_IRQ_DMA1       135 /* Vector: 135, DMA Channel 1 */
#define PIC32MZ_IRQ_DMA2       136 /* Vector: 136, DMA Channel 2 */
#define PIC32MZ_IRQ_DMA3       137 /* Vector: 137, DMA Channel 3 */
#define PIC32MZ_IRQ_DMA4       138 /* Vector: 138, DMA Channel 4 */
#define PIC32MZ_IRQ_DMA5       139 /* Vector: 139, DMA Channel 5 */

#define PIC32MZ_IRQ_DMA6       140 /* Vector: 140, DMA Channel 6 */
#define PIC32MZ_IRQ_DMA7       141 /* Vector: 141, DMA Channel 7 */
#define PIC32MZ_IRQ_SPI2F      142 /* Vector: 142, SPI2 Fault */
#define PIC32MZ_IRQ_SPI2RX     143 /* Vector: 143, SPI2 Receive Done */
#define PIC32MZ_IRQ_SPI2TX     144 /* Vector: 144, SPI2 Transfer Done */
#define PIC32MZ_IRQ_U2E        145 /* Vector: 145, UART2 Fault */
#define PIC32MZ_IRQ_U2RX       146 /* Vector: 146, UART2 Receive Done */
#define PIC32MZ_IRQ_U2TX       147 /* Vector: 147, UART2 Transfer Done */
#define PIC32MZ_IRQ_I2C2COL    148 /* Vector: 148, I2C2 Bus Collision Event */
#define PIC32MZ_IRQ_I2C2S      149 /* Vector: 149, I2C2 Slave Event */

#define PIC32MZ_IRQ_I2C2M      150 /* Vector: 150, I2C2 Master Event */
#define PIC32MZ_IRQ_CAN1       151 /* Vector: 151, Control Area Network 1 */
#define PIC32MZ_IRQ_CAN2       152 /* Vector: 152, Control Area Network 2 */
#define PIC32MZ_IRQ_ETH        153 /* Vector: 153, Ethernet interrupt */
#define PIC32MZ_IRQ_SPI3F      154 /* Vector: 154, SPI3 Fault */
#define PIC32MZ_IRQ_SPI3RX     155 /* Vector: 155, SPI3 Receive Done */
#define PIC32MZ_IRQ_SPI3TX     156 /* Vector: 156, SPI3 Transfer Done */
#define PIC32MZ_IRQ_U3E        157 /* Vector: 157, UART3 Fault */
#define PIC32MZ_IRQ_U3RX       158 /* Vector: 158, UART3 Receive Done */
#define PIC32MZ_IRQ_U3TX       159 /* Vector: 159, UART3 Transfer Done */

#define PIC32MZ_IRQ_I2C3COL    160 /* Vector: 160, I2C3 Bus Collision Event */
#define PIC32MZ_IRQ_I2C3S      161 /* Vector: 161, I2C3 Slave Event */
#define PIC32MZ_IRQ_I2C3M      162 /* Vector: 162, I2C3 Master Event */
#define PIC32MZ_IRQ_SPI4F      163 /* Vector: 163, SPI4 Fault */
#define PIC32MZ_IRQ_SPI4RX     164 /* Vector: 164, SPI4 Receive Done */
#define PIC32MZ_IRQ_SPI4TX     165 /* Vector: 165, SPI4 Transfer Done */
#define PIC32MZ_IRQ_RTCC       166 /* Vector: 166, Real-Time Clock and Calendar */
#define PIC32MZ_IRQ_FCE        167 /* Vector: 167, Flash Control Event */
#define PIC32MZ_IRQ_PMSEC      168 /* Vector: 168, Prefetch Module SEC Event */
#define PIC32MZ_IRQ_SQI1       169 /* Vector: 169, SQI1 Event */

#define PIC32MZ_IRQ_U4E        170 /* Vector: 170, UART4 Fault */
#define PIC32MZ_IRQ_U4RX       171 /* Vector: 171, UART4 Receive Done */
#define PIC32MZ_IRQ_U4TX       172 /* Vector: 172, UART4 Transfer Done */
#define PIC32MZ_IRQ_I2C4COL    173 /* Vector: 173, I2C4 Bus Collision Event */
#define PIC32MZ_IRQ_I2C4S      174 /* Vector: 174, I2C4 Slave Event */
#define PIC32MZ_IRQ_I2C4M      175 /* Vector: 175, I2C4 Master Event */
#define PIC32MZ_IRQ_SPI5F      176 /* Vector: 176, SPI5 Fault */
#define PIC32MZ_IRQ_SPI5RX     177 /* Vector: 177, SPI5 Receive Done */
#define PIC32MZ_IRQ_SPI5TX     178 /* Vector: 178, SPI5 Transfer Done */
#define PIC32MZ_IRQ_U5E        179 /* Vector: 179, UART5 Fault */

#define PIC32MZ_IRQ_U5RX       180 /* Vector: 180, UART5 Receive Done */
#define PIC32MZ_IRQ_U5TX       181 /* Vector: 181, UART5 Transfer Done */
#define PIC32MZ_IRQ_I2C5COL    182 /* Vector: 182, I2C5 Bus Collision Event */
#define PIC32MZ_IRQ_I2C5S      183 /* Vector: 183, I2C5 Slave Event */
#define PIC32MZ_IRQ_I2C5M      184 /* Vector: 184, I2C5 Master Event */
#define PIC32MZ_IRQ_SPI6F      185 /* Vector: 185, SPI6 Fault */
#define PIC32MZ_IRQ_SPI6RX     186 /* Vector: 186, SPI6 Receive Done */
#define PIC32MZ_IRQ_SPI6TX     187 /* Vector: 187, SPI6 Transfer Done */
#define PIC32MZ_IRQ_U6E        188 /* Vector: 188, UART6 Fault */
#define PIC32MZ_IRQ_U6RX       189 /* Vector: 189, UART6 Receive Done */

#define PIC32MZ_IRQ_U6TX       190 /* Vector: 190, UART6 Transfer Done */
                                   /* Vector: 191, Reserved */
#define PIC32MZ_IRQ_ADCESR     192 /* Vector: ADC End of Scan Ready */
#define PIC32MZ_IRQ_ADCACR     193 /* Vector: ADC Analog Circuits Ready */
#define PIC32MZ_IRQ_ADCUR      194 /* Vector: ADC Update Ready */
                                   /* Vector: 195, Reserved */
#define PIC32MZ_IRQ_ADCGEIR    196 /* Vector: ADC Group Early Interrupt Request */
                                   /* Vector: 197, Reserved */
#define PIC32MZ_IRQ_ADC0ER     198 /* Vector: ADC0 Early Interrupt */
#define PIC32MZ_IRQ_ADC1ER     199 /* Vector: ADC1 Early Interrupt */
#define PIC32MZ_IRQ_ADC2ER     200 /* Vector: ADC2 Early Interrupt */
#define PIC32MZ_IRQ_ADC3ER     201 /* Vector: ADC3 Early Interrupt */
#define PIC32MZ_IRQ_ADC4ER     202 /* Vector: ADC4 Early Interrupt */
                                   /* Vector: 203, Reserved */
                                   /* Vector: 204, Reserved */
#define PIC32MZ_IRQ_ADC7ER     205 /* Vector: ADC7 Early Interrupt */
#define PIC32MZ_IRQ_ADC0WI     206 /* Vector: ADC0 Warm Interrupt */
#define PIC32MZ_IRQ_ADC1WI     207 /* Vector: ADC1 Warm Interrupt */
#define PIC32MZ_IRQ_ADC2WI     208 /* Vector: ADC2 Warm Interrupt */
#define PIC32MZ_IRQ_ADC3WI     209 /* Vector: ADC3 Warm Interrupt */
#define PIC32MZ_IRQ_ADC4WI     210 /* Vector: ADC4 Warm Interrupt */
                                   /* Vector: 211, Reserved */
                                   /* Vector: 212, Reserved */
#define PIC32MZ_IRQ_ADC7WI     213 /* Vector: ADC7 Warm Interrupt */

#define PIC32MZ_IRQ_BAD        214 /* Not a real IRQ number */
#define NR_IRQS                214

/****************************************************************************
 * Public Types
 ****************************************************************************/

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
#endif /* __ARCH_MIPS_INCLUDE_PIC32MZ_IRQ_PIC32MZXXXEF_H */
