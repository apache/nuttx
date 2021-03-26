/****************************************************************************
 * arch/arm/src/s32k1xx/hardware/s32k1xx_pcc.h
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

#ifndef __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_PCC_H
#define __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_PCC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <hardware/s32k1xx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PCC Register Offsets *****************************************************/

#define S32K1XX_PCC_FTFC_OFFSET      0x0080  /* PCC FTFC Register */
#define S32K1XX_PCC_DMAMUX_OFFSET    0x0084  /* PCC DMAMUX Register */
#define S32K1XX_PCC_FLEXCAN0_OFFSET  0x0090  /* PCC FlexCAN0 Register */
#define S32K1XX_PCC_FLEXCAN1_OFFSET  0x0094  /* PCC FlexCAN1 Register */
#define S32K1XX_PCC_FTM3_OFFSET      0x0098  /* PCC FTM3 Register */
#define S32K1XX_PCC_ADC1_OFFSET      0x009c  /* PCC ADC1 Register */
#define S32K1XX_PCC_FLEXCAN2_OFFSET  0x00ac  /* PCC FlexCAN2 Register */
#define S32K1XX_PCC_LPSPI0_OFFSET    0x00b0  /* PCC LPSPI0 Register */
#define S32K1XX_PCC_LPSPI1_OFFSET    0x00b4  /* PCC LPSPI1 Register */
#define S32K1XX_PCC_LPSPI2_OFFSET    0x00b8  /* PCC LPSPI2 Register */
#define S32K1XX_PCC_PDB1_OFFSET      0x00c4  /* PCC PDB1 Register */
#define S32K1XX_PCC_CRC_OFFSET       0x00c8  /* PCC CRC Register */
#define S32K1XX_PCC_PDB0_OFFSET      0x00d8  /* PCC PDB0 Register */
#define S32K1XX_PCC_LPIT_OFFSET      0x00dc  /* PCC LPIT Register */
#define S32K1XX_PCC_FTM0_OFFSET      0x00e0  /* PCC FTM0 Register */
#define S32K1XX_PCC_FTM1_OFFSET      0x00e4  /* PCC FTM1 Register */
#define S32K1XX_PCC_FTM2_OFFSET      0x00e8  /* PCC FTM2 Register */
#define S32K1XX_PCC_ADC0_OFFSET      0x00ec  /* PCC ADC0 Register */
#define S32K1XX_PCC_RTC_OFFSET       0x00f4  /* PCC RTC Register */
#define S32K1XX_PCC_CMU0_OFFSET      0x00f8  /* PCC CMU0 Register */
#define S32K1XX_PCC_CMU1_OFFSET      0x00fc  /* PCC CMU1 Register */
#define S32K1XX_PCC_LPTMR0_OFFSET    0x0100  /* PCC LPTMR0 Register */
#define S32K1XX_PCC_PORTA_OFFSET     0x0124  /* PCC PORTA Register */
#define S32K1XX_PCC_PORTB_OFFSET     0x0128  /* PCC PORTB Register */
#define S32K1XX_PCC_PORTC_OFFSET     0x012c  /* PCC PORTC Register */
#define S32K1XX_PCC_PORTD_OFFSET     0x0130  /* PCC PORTD Register */
#define S32K1XX_PCC_PORTE_OFFSET     0x0134  /* PCC PORTE Register */
#define S32K1XX_PCC_SAI0_OFFSET      0x0150  /* PCC SAI0 Register */
#define S32K1XX_PCC_SAI1_OFFSET      0x0154  /* PCC SAI1 Register */
#define S32K1XX_PCC_FLEXIO_OFFSET    0x0168  /* PCC FlexIO Register */
#define S32K1XX_PCC_EWM_OFFSET       0x0184  /* PCC EWM Register */
#define S32K1XX_PCC_LPI2C0_OFFSET    0x0198  /* PCC LPI2C0 Register */
#define S32K1XX_PCC_LPI2C1_OFFSET    0x019c  /* PCC LPI2C1 Register */
#define S32K1XX_PCC_LPUART0_OFFSET   0x01a8  /* PCC LPUART0 Register */
#define S32K1XX_PCC_LPUART1_OFFSET   0x01ac  /* PCC LPUART1 Register */
#define S32K1XX_PCC_LPUART2_OFFSET   0x01b0  /* PCC LPUART2 Register */
#define S32K1XX_PCC_FTM4_OFFSET      0x01b8  /* PCC FTM4 Register */
#define S32K1XX_PCC_FTM5_OFFSET      0x01bc  /* PCC FTM5 Register */
#define S32K1XX_PCC_FTM6_OFFSET      0x01c0  /* PCC FTM6 Register */
#define S32K1XX_PCC_FTM7_OFFSET      0x01c4  /* PCC FTM7 Register */
#define S32K1XX_PCC_CMP0_OFFSET      0x01cc  /* PCC CMP0 Register */
#define S32K1XX_PCC_QSPI_OFFSET      0x01d8  /* PCC QSPI Register */
#define S32K1XX_PCC_ENET_OFFSET      0x01e4  /* PCC ENET Register */

/* PCC Register Addresses ***************************************************/

#define S32K1XX_PCC_FTFC             (S32K1XX_PCC_BASE + S32K1XX_PCC_FTFC_OFFSET)
#define S32K1XX_PCC_DMAMUX           (S32K1XX_PCC_BASE + S32K1XX_PCC_DMAMUX_OFFSET)
#define S32K1XX_PCC_FLEXCAN0         (S32K1XX_PCC_BASE + S32K1XX_PCC_FLEXCAN0_OFFSET)
#define S32K1XX_PCC_FLEXCAN1         (S32K1XX_PCC_BASE + S32K1XX_PCC_FLEXCAN1_OFFSET)
#define S32K1XX_PCC_FTM3             (S32K1XX_PCC_BASE + S32K1XX_PCC_FTM3_OFFSET)
#define S32K1XX_PCC_ADC1             (S32K1XX_PCC_BASE + S32K1XX_PCC_ADC1_OFFSET)
#define S32K1XX_PCC_FLEXCAN2         (S32K1XX_PCC_BASE + S32K1XX_PCC_FLEXCAN2_OFFSET)
#define S32K1XX_PCC_LPSPI0           (S32K1XX_PCC_BASE + S32K1XX_PCC_LPSPI0_OFFSET)
#define S32K1XX_PCC_LPSPI1           (S32K1XX_PCC_BASE + S32K1XX_PCC_LPSPI1_OFFSET)
#define S32K1XX_PCC_LPSPI2           (S32K1XX_PCC_BASE + S32K1XX_PCC_LPSPI2_OFFSET)
#define S32K1XX_PCC_PDB1             (S32K1XX_PCC_BASE + S32K1XX_PCC_PDB1_OFFSET)
#define S32K1XX_PCC_CRC              (S32K1XX_PCC_BASE + S32K1XX_PCC_CRC_OFFSET)
#define S32K1XX_PCC_PDB0             (S32K1XX_PCC_BASE + S32K1XX_PCC_PDB0_OFFSET)
#define S32K1XX_PCC_LPIT             (S32K1XX_PCC_BASE + S32K1XX_PCC_LPIT_OFFSET)
#define S32K1XX_PCC_FTM0             (S32K1XX_PCC_BASE + S32K1XX_PCC_FTM0_OFFSET)
#define S32K1XX_PCC_FTM1             (S32K1XX_PCC_BASE + S32K1XX_PCC_FTM1_OFFSET)
#define S32K1XX_PCC_FTM2             (S32K1XX_PCC_BASE + S32K1XX_PCC_FTM2_OFFSET)
#define S32K1XX_PCC_ADC0             (S32K1XX_PCC_BASE + S32K1XX_PCC_ADC0_OFFSET)
#define S32K1XX_PCC_RTC              (S32K1XX_PCC_BASE + S32K1XX_PCC_RTC_OFFSET)
#define S32K1XX_PCC_CMU0             (S32K1XX_PCC_BASE + S32K1XX_PCC_CMU0_OFFSET)
#define S32K1XX_PCC_CMU1             (S32K1XX_PCC_BASE + S32K1XX_PCC_CMU1_OFFSET)
#define S32K1XX_PCC_LPTMR0           (S32K1XX_PCC_BASE + S32K1XX_PCC_LPTMR0_OFFSET)
#define S32K1XX_PCC_PORTA            (S32K1XX_PCC_BASE + S32K1XX_PCC_PORTA_OFFSET)
#define S32K1XX_PCC_PORTB            (S32K1XX_PCC_BASE + S32K1XX_PCC_PORTB_OFFSET)
#define S32K1XX_PCC_PORTC            (S32K1XX_PCC_BASE + S32K1XX_PCC_PORTC_OFFSET)
#define S32K1XX_PCC_PORTD            (S32K1XX_PCC_BASE + S32K1XX_PCC_PORTD_OFFSET)
#define S32K1XX_PCC_PORTE            (S32K1XX_PCC_BASE + S32K1XX_PCC_PORTE_OFFSET)
#define S32K1XX_PCC_SAI0             (S32K1XX_PCC_BASE + S32K1XX_PCC_SAI0_OFFSET)
#define S32K1XX_PCC_SAI1             (S32K1XX_PCC_BASE + S32K1XX_PCC_SAI1_OFFSET)
#define S32K1XX_PCC_FLEXIO           (S32K1XX_PCC_BASE + S32K1XX_PCC_FLEXIO_OFFSET)
#define S32K1XX_PCC_EWM              (S32K1XX_PCC_BASE + S32K1XX_PCC_EWM_OFFSET)
#define S32K1XX_PCC_LPI2C0           (S32K1XX_PCC_BASE + S32K1XX_PCC_LPI2C0_OFFSET)
#define S32K1XX_PCC_LPI2C1           (S32K1XX_PCC_BASE + S32K1XX_PCC_LPI2C1_OFFSET)
#define S32K1XX_PCC_LPUART0          (S32K1XX_PCC_BASE + S32K1XX_PCC_LPUART0_OFFSET)
#define S32K1XX_PCC_LPUART1          (S32K1XX_PCC_BASE + S32K1XX_PCC_LPUART1_OFFSET)
#define S32K1XX_PCC_LPUART2          (S32K1XX_PCC_BASE + S32K1XX_PCC_LPUART2_OFFSET)
#define S32K1XX_PCC_FTM4             (S32K1XX_PCC_BASE + S32K1XX_PCC_FTM4_OFFSET)
#define S32K1XX_PCC_FTM5             (S32K1XX_PCC_BASE + S32K1XX_PCC_FTM5_OFFSET)
#define S32K1XX_PCC_FTM6             (S32K1XX_PCC_BASE + S32K1XX_PCC_FTM6_OFFSET)
#define S32K1XX_PCC_FTM7             (S32K1XX_PCC_BASE + S32K1XX_PCC_FTM7_OFFSET)
#define S32K1XX_PCC_CMP0             (S32K1XX_PCC_BASE + S32K1XX_PCC_CMP0_OFFSET)
#define S32K1XX_PCC_QSPI             (S32K1XX_PCC_BASE + S32K1XX_PCC_QSPI_OFFSET)
#define S32K1XX_PCC_ENET             (S32K1XX_PCC_BASE + S32K1XX_PCC_ENET_OFFSET)

/* PCC Register Bitfield Definitions ****************************************/

/* The form of each PCC register is the same as follows.
 *  Some register, however, do not support all of the fields:
 *
 * PCD  - ENET
 * FRAC - ENET
 * PCS  - FTM3, ADC0, ADC1, LPSPI0, LPSPI1, LPSPI2, LPIT,
 *        FTM0, FTM2, FTM4, FTM6, FTM7, LPTMR0, FLEXIO, LPI2C0, LPCI2C1,
 *        UART0, UART1, UART2, ENET
 * CGC  - All PCC registers
 * PR   - All PCC registers
 */

#define PCC_PCD_SHIFT      (0)       /* Bits 0-2:  Peripheral Clock Divider Select */
#define PCC_PCD_MASK       (7 << PCC_PCD_SHIFT)
#  define PCC_PCD(n)       ((uint32_t)((n) - 1) << PCC_PCD_SHIFT) /* n=1..8 */

#define PCC_FRAC           (1 << 3)  /* Bits 3:  Peripheral Clock Divider Fraction */
#define PCC_PCS_SHIFT      (24)      /* Bits 24-26:  Peripheral Clock Source Select */
#define PCC_PCS_MASK       (7 << PCC_PCS_SHIFT)
#  define PCC_PCS(n)       ((uint32_t)(n) << PCC_PCS_SHIFT)
#  define PCC_PCS_OFF      (0 << PCC_PCS_SHIFT)  /* Clock is off */
#  define PCC_PCS_OPTION1  (1 << PCC_PCS_SHIFT)  /* Clock option 1 */
#  define PCC_PCS_OPTION2  (2 << PCC_PCS_SHIFT)  /* Clock option 2 */
#  define PCC_PCS_OPTION3  (3 << PCC_PCS_SHIFT)  /* Clock option 3 */
#  define PCC_PCS_OPTION4  (4 << PCC_PCS_SHIFT)  /* Clock option 4 */
#  define PCC_PCS_OPTION5  (5 << PCC_PCS_SHIFT)  /* Clock option 5 */
#  define PCC_PCS_OPTION6  (6 << PCC_PCS_SHIFT)  /* Clock option 6 */
#  define PCC_PCS_OPTION7  (7 << PCC_PCS_SHIFT)  /* Clock option 7 */

#define PCC_CGC            (1 << 30) /* Clock Gate Control */
#define PCC_PR             (1 << 31) /* Present */

#endif /* __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_PCC_H */
