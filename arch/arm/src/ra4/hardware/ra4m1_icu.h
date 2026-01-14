/****************************************************************************
 * arch/arm/src/ra4/hardware/ra4m1_icu.h
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

#ifndef __ARCH_ARM_SRC_RA4M1_HARDWARE_RA4M1_ICU_H
#define __ARCH_ARM_SRC_RA4M1_HARDWARE_RA4M1_ICU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/ra_memorymap.h"
#if defined(CONFIG_RA4M1_FAMILY)
#  include "hardware/ra4m1_icu.h"
#else
#  error "Unsupported RA memory map"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

 #define  EVENT_NONE                       (0x00)  /* Link disabled */
 #define  EVENT_ICU_IRQ0                   (0x01)  /* External pin interrupt 0 */
 #define  EVENT_ICU_IRQ1                   (0x02)  /* External pin interrupt 1 */
 #define  EVENT_ICU_IRQ2                   (0x03)  /* External pin interrupt 2 */
 #define  EVENT_ICU_IRQ3                   (0x04)  /* External pin interrupt 3 */
 #define  EVENT_ICU_IRQ4                   (0x05)  /* External pin interrupt 4 */
 #define  EVENT_ICU_IRQ5                   (0x06)  /* External pin interrupt 5 */
 #define  EVENT_ICU_IRQ6                   (0x07)  /* External pin interrupt 6 */
 #define  EVENT_ICU_IRQ7                   (0x08)  /* External pin interrupt 7 */
 #define  EVENT_ICU_IRQ8                   (0x09)  /* External pin interrupt 8 */
 #define  EVENT_ICU_IRQ9                   (0x0A)  /* External pin interrupt 9 */
 #define  EVENT_ICU_IRQ10                  (0x0B)  /* External pin interrupt 10 */
 #define  EVENT_ICU_IRQ11                  (0x0C)  /* External pin interrupt 11 */
 #define  EVENT_ICU_IRQ12                  (0x0D)  /* External pin interrupt 12 */
 #define  EVENT_ICU_IRQ14                  (0x0F)  /* External pin interrupt 14 */
 #define  EVENT_ICU_IRQ15                  (0x10)  /* External pin interrupt 15 */
 #define  EVENT_DMAC0_INT                  (0x11)  /* DMAC0 transfer end */
 #define  EVENT_DMAC1_INT                  (0x12)  /* DMAC1 transfer end */
 #define  EVENT_DMAC2_INT                  (0x13)  /* DMAC2 transfer end */
 #define  EVENT_DMAC3_INT                  (0x14)  /* DMAC3 transfer end */
 #define  EVENT_DTC_COMPLETE               (0x15)  /* DTC transfer complete */
 #define  EVENT_DTC_END                    (0x16)  /* DTC transfer end */
 #define  EVENT_ICU_SNOOZE_CANCEL          (0x17)  /* Canceling from Snooze mode */
 #define  EVENT_FCU_FRDYI                  (0x18)  /* Flash ready interrupt */
 #define  EVENT_LVD_LVD1                   (0x19)  /* Voltage monitor 1 interrupt */
 #define  EVENT_LVD_LVD2                   (0x1A)  /* Voltage monitor 2 interrupt */
 #define  EVENT_LVD_VBATT                  (0x1B)  /* VBATT low voltage detect */
 #define  EVENT_CGC_MOSC_STOP              (0x1C)  /* Main Clock oscillation stop */
 #define  EVENT_LPM_SNOOZE_REQUEST         (0x1D)  /* Snooze entry */
 #define  EVENT_AGT0_INT                   (0x1E)  /* AGT interrupt */
 #define  EVENT_AGT0_COMPARE_A             (0x1F)  /* Compare match A */
 #define  EVENT_AGT0_COMPARE_B             (0x20)  /* Compare match B */
 #define  EVENT_AGT1_INT                   (0x21)  /* AGT interrupt */
 #define  EVENT_AGT1_COMPARE_A             (0x22)  /* Compare match A */
 #define  EVENT_AGT1_COMPARE_B             (0x23)  /* Compare match B */
 #define  EVENT_IWDT_UNDERFLOW             (0x24)  /* IWDT underflow */
 #define  EVENT_WDT_UNDERFLOW              (0x25)  /* WDT underflow */
 #define  EVENT_RTC_ALARM                  (0x26)  /* Alarm interrupt */
 #define  EVENT_RTC_PERIOD                 (0x27)  /* Periodic interrupt */
 #define  EVENT_RTC_CARRY                  (0x28)  /* Carry interrupt */
 #define  EVENT_ADC0_SCAN_END              (0x29)  /* End of A/D scanning operation */
 #define  EVENT_ADC0_SCAN_END_B            (0x2A)  /* A/D scan end interrupt for group B */
 #define  EVENT_ADC0_WINDOW_A              (0x2B)  /* Window A Compare match interrupt */
 #define  EVENT_ADC0_WINDOW_B              (0x2C)  /* Window B Compare match interrupt */
 #define  EVENT_ADC0_COMPARE_MATCH         (0x2D)  /* Compare match */
 #define  EVENT_ADC0_COMPARE_MISMATCH      (0x2E)  /* Compare mismatch */
 #define  EVENT_ACMPLP0_INT                (0x2F)  /* Low Power Comparator channel 0 interrupt */
 #define  EVENT_ACMPLP1_INT                (0x30)  /* Low Power Comparator channel 1 interrupt */
 #define  EVENT_USBFS_FIFO_0               (0x31)  /* DMA transfer request 0 */
 #define  EVENT_USBFS_FIFO_1               (0x32)  /* DMA transfer request 1 */
 #define  EVENT_USBFS_INT                  (0x33)  /* USBFS interrupt */
 #define  EVENT_USBFS_RESUME               (0x34)  /* USBFS resume interrupt */
 #define  EVENT_IIC0_RXI                   (0x35)  /* Receive data full */
 #define  EVENT_IIC0_TXI                   (0x36)  /* Transmit data empty */
 #define  EVENT_IIC0_TEI                   (0x37)  /* Transmit end */
 #define  EVENT_IIC0_ERI                   (0x38)  /* Transfer error */
 #define  EVENT_IIC0_WUI                   (0x39)  /* Wakeup interrupt */
 #define  EVENT_IIC1_RXI                   (0x3A)  /* Receive data full */
 #define  EVENT_IIC1_TXI                   (0x3B)  /* Transmit data empty */
 #define  EVENT_IIC1_TEI                   (0x3C)  /* Transmit end */
 #define  EVENT_IIC1_ERI                   (0x3D)  /* Transfer error */
 #define  EVENT_SSI0_TXI                   (0x3E)  /* Transmit data empty */
 #define  EVENT_SSI0_RXI                   (0x3F)  /* Receive data full */
 #define  EVENT_SSI0_INT                   (0x41)  /* Error interrupt */
 #define  EVENT_CTSU_WRITE                 (0x42)  /* Write request interrupt */
 #define  EVENT_CTSU_READ                  (0x43)  /* Measurement data transfer request interrupt */
 #define  EVENT_CTSU_END                   (0x44)  /* Measurement end interrupt */
 #define  EVENT_KEY_INT                    (0x45)  /* Key interrupt */
 #define  EVENT_DOC_INT                    (0x46)  /* Data operation circuit interrupt */
 #define  EVENT_CAC_FREQUENCY_ERROR        (0x47)  /* Frequency error interrupt */
 #define  EVENT_CAC_MEASUREMENT_END        (0x48)  /* Measurement end interrupt */
 #define  EVENT_CAC_OVERFLOW               (0x49)  /* Overflow interrupt */
 #define  EVENT_CAN0_ERROR                 (0x4A)  /* Error interrupt */
 #define  EVENT_CAN0_FIFO_RX               (0x4B)  /* Receive FIFO interrupt */
 #define  EVENT_CAN0_FIFO_TX               (0x4C)  /* Transmit FIFO interrupt */
 #define  EVENT_CAN0_MAILBOX_RX            (0x4D)  /* Reception complete interrupt */
 #define  EVENT_CAN0_MAILBOX_TX            (0x4E)  /* Transmission complete interrupt */
 #define  EVENT_IOPORT_EVENT_1             (0x4F)  /* Port 1 event */
 #define  EVENT_IOPORT_EVENT_2             (0x50)  /* Port 2 event */
 #define  EVENT_IOPORT_EVENT_3             (0x51)  /* Port 3 event */
 #define  EVENT_IOPORT_EVENT_4             (0x52)  /* Port 4 event */
 #define  EVENT_SOFTWARE_EVENT_0           (0x53)  /* Software event 0 */
 #define  EVENT_SOFTWARE_EVENT_1           (0x54)  /* Software event 1 */
 #define  EVENT_POEG0_EVENT                (0x55)  /* Port Output disable 0 interrupt */
 #define  EVENT_POEG1_EVENT                (0x56)  /* Port Output disable 1 interrupt */
 #define  EVENT_GPT0_CAPTURE_COMPARE_A     (0x57)  /* Capture/Compare match A */
 #define  EVENT_GPT0_CAPTURE_COMPARE_B     (0x58)  /* Capture/Compare match B */
 #define  EVENT_GPT0_COMPARE_C             (0x59)  /* Compare match C */
 #define  EVENT_GPT0_COMPARE_D             (0x5A)  /* Compare match D */
 #define  EVENT_GPT0_COMPARE_E             (0x5B)  /* Compare match E */
 #define  EVENT_GPT0_COMPARE_F             (0x5C)  /* Compare match F */
 #define  EVENT_GPT0_COUNTER_OVERFLOW      (0x5D)  /* Overflow */
 #define  EVENT_GPT0_COUNTER_UNDERFLOW     (0x5E)  /* Underflow */
 #define  EVENT_GPT1_CAPTURE_COMPARE_A     (0x5F)  /* Capture/Compare match A */
 #define  EVENT_GPT1_CAPTURE_COMPARE_B     (0x60)  /* Capture/Compare match B */
 #define  EVENT_GPT1_COMPARE_C             (0x61)  /* Compare match C */
 #define  EVENT_GPT1_COMPARE_D             (0x62)  /* Compare match D */
 #define  EVENT_GPT1_COMPARE_E             (0x63)  /* Compare match E */
 #define  EVENT_GPT1_COMPARE_F             (0x64)  /* Compare match F */
 #define  EVENT_GPT1_COUNTER_OVERFLOW      (0x65)  /* Overflow */
 #define  EVENT_GPT1_COUNTER_UNDERFLOW     (0x66)  /* Underflow */
 #define  EVENT_GPT2_CAPTURE_COMPARE_A     (0x67)  /* Capture/Compare match A */
 #define  EVENT_GPT2_CAPTURE_COMPARE_B     (0x68)  /* Capture/Compare match B */
 #define  EVENT_GPT2_COMPARE_C             (0x69)  /* Compare match C */
 #define  EVENT_GPT2_COMPARE_D             (0x6A)  /* Compare match D */
 #define  EVENT_GPT2_COMPARE_E             (0x6B)  /* Compare match E */
 #define  EVENT_GPT2_COMPARE_F             (0x6C)  /* Compare match F */
 #define  EVENT_GPT2_COUNTER_OVERFLOW      (0x6D)  /* Overflow */
 #define  EVENT_GPT2_COUNTER_UNDERFLOW     (0x6E)  /* Underflow */
 #define  EVENT_GPT3_CAPTURE_COMPARE_A     (0x6F)  /* Capture/Compare match A */
 #define  EVENT_GPT3_CAPTURE_COMPARE_B     (0x70)  /* Capture/Compare match B */
 #define  EVENT_GPT3_COMPARE_C             (0x71)  /* Compare match C */
 #define  EVENT_GPT3_COMPARE_D             (0x72)  /* Compare match D */
 #define  EVENT_GPT3_COMPARE_E             (0x73)  /* Compare match E */
 #define  EVENT_GPT3_COMPARE_F             (0x74)  /* Compare match F */
 #define  EVENT_GPT3_COUNTER_OVERFLOW      (0x75)  /* Overflow */
 #define  EVENT_GPT3_COUNTER_UNDERFLOW     (0x76)  /* Underflow */
 #define  EVENT_GPT4_CAPTURE_COMPARE_A     (0x77)  /* Capture/Compare match A */
 #define  EVENT_GPT4_CAPTURE_COMPARE_B     (0x78)  /* Capture/Compare match B */
 #define  EVENT_GPT4_COMPARE_C             (0x79)  /* Compare match C */
 #define  EVENT_GPT4_COMPARE_D             (0x7A)  /* Compare match D */
 #define  EVENT_GPT4_COMPARE_E             (0x7B)  /* Compare match E */
 #define  EVENT_GPT4_COMPARE_F             (0x7C)  /* Compare match F */
 #define  EVENT_GPT4_COUNTER_OVERFLOW      (0x7D)  /* Overflow */
 #define  EVENT_GPT4_COUNTER_UNDERFLOW     (0x7E)  /* Underflow */
 #define  EVENT_GPT5_CAPTURE_COMPARE_A     (0x7F)  /* Capture/Compare match A */
 #define  EVENT_GPT5_CAPTURE_COMPARE_B     (0x80)  /* Capture/Compare match B */
 #define  EVENT_GPT5_COMPARE_C             (0x81)  /* Compare match C */
 #define  EVENT_GPT5_COMPARE_D             (0x82)  /* Compare match D */
 #define  EVENT_GPT5_COMPARE_E             (0x83)  /* Compare match E */
 #define  EVENT_GPT5_COMPARE_F             (0x84)  /* Compare match F */
 #define  EVENT_GPT5_COUNTER_OVERFLOW      (0x85)  /* Overflow */
 #define  EVENT_GPT5_COUNTER_UNDERFLOW     (0x86)  /* Underflow */
 #define  EVENT_GPT6_CAPTURE_COMPARE_A     (0x87)  /* Capture/Compare match A */
 #define  EVENT_GPT6_CAPTURE_COMPARE_B     (0x88)  /* Capture/Compare match B */
 #define  EVENT_GPT6_COMPARE_C             (0x89)  /* Compare match C */
 #define  EVENT_GPT6_COMPARE_D             (0x8A)  /* Compare match D */
 #define  EVENT_GPT6_COMPARE_E             (0x8B)  /* Compare match E */
 #define  EVENT_GPT6_COMPARE_F             (0x8C)  /* Compare match F */
 #define  EVENT_GPT6_COUNTER_OVERFLOW      (0x8D)  /* Overflow */
 #define  EVENT_GPT6_COUNTER_UNDERFLOW     (0x8E)  /* Underflow */
 #define  EVENT_GPT7_CAPTURE_COMPARE_A     (0x8F)  /* Capture/Compare match A */
 #define  EVENT_GPT7_CAPTURE_COMPARE_B     (0x90)  /* Capture/Compare match B */
 #define  EVENT_GPT7_COMPARE_C             (0x91)  /* Compare match C */
 #define  EVENT_GPT7_COMPARE_D             (0x92)  /* Compare match D */
 #define  EVENT_GPT7_COMPARE_E             (0x93)  /* Compare match E */
 #define  EVENT_GPT7_COMPARE_F             (0x94)  /* Compare match F */
 #define  EVENT_GPT7_COUNTER_OVERFLOW      (0x95)  /* Overflow */
 #define  EVENT_GPT7_COUNTER_UNDERFLOW     (0x96)  /* Underflow */
 #define  EVENT_OPS_UVW_EDGE               (0x97)  /* UVW edge event */
 #define  EVENT_SCI0_RXI                   (0x98)  /* Receive data full */
 #define  EVENT_SCI0_TXI                   (0x99)  /* Transmit data empty */
 #define  EVENT_SCI0_TEI                   (0x9A)  /* Transmit end */
 #define  EVENT_SCI0_ERI                   (0x9B)  /* Receive error */
 #define  EVENT_SCI0_AM                    (0x9C)  /* Address match event */
 #define  EVENT_SCI0_RXI_OR_ERI            (0x9D)  /* Receive data full/Receive error */
 #define  EVENT_SCI1_RXI                   (0x9E)  /* Receive data full */
 #define  EVENT_SCI1_TXI                   (0x9F)  /* Transmit data empty */
 #define  EVENT_SCI1_TEI                   (0xA0)  /* Transmit end */
 #define  EVENT_SCI1_ERI                   (0xA1)  /* Receive error */
 #define  EVENT_SCI1_AM                    (0xA2)  /* Address match event */
 #define  EVENT_SCI2_RXI                   (0xA3)  /* Receive data full */
 #define  EVENT_SCI2_TXI                   (0xA4)  /* Transmit data empty */
 #define  EVENT_SCI2_TEI                   (0xA5)  /* Transmit end */
 #define  EVENT_SCI2_ERI                   (0xA6)  /* Receive error */
 #define  EVENT_SCI2_AM                    (0xA7)  /* Address match event */
 #define  EVENT_SCI9_RXI                   (0xA8)  /* Receive data full */
 #define  EVENT_SCI9_TXI                   (0xA9)  /* Transmit data empty */
 #define  EVENT_SCI9_TEI                   (0xAA)  /* Transmit end */
 #define  EVENT_SCI9_ERI                   (0xAB)  /* Receive error */
 #define  EVENT_SCI9_AM                    (0xAC)  /* Address match event */
 #define  EVENT_SPI0_RXI                   (0xAD)  /* Receive buffer full */
 #define  EVENT_SPI0_TXI                   (0xAE)  /* Transmit buffer empty */
 #define  EVENT_SPI0_IDLE                  (0xAF)  /* Idle */
 #define  EVENT_SPI0_ERI                   (0xB0)  /* Error */
 #define  EVENT_SPI0_TEI                   (0xB1)  /* Transmission complete event */
 #define  EVENT_SPI1_RXI                   (0xB2)  /* Receive buffer full */
 #define  EVENT_SPI1_TXI                   (0xB3)  /* Transmit buffer empty */
 #define  EVENT_SPI1_IDLE                  (0xB4)  /* Idle */
 #define  EVENT_SPI1_ERI                   (0xB5)  /* Error */
 #define  EVENT_SPI1_TEI                   (0xB6)  /* Transmission complete event */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_RA4M1_HARDWARE_RA4M1_ICU_H */
