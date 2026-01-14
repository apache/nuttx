/****************************************************************************
 * arch/arm/src/ra4/hardware/ra_icu.h
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

#ifndef __ARCH_ARM_SRC_RA4M1_HARDWARE_RA_ICU_H
#define __ARCH_ARM_SRC_RA4M1_HARDWARE_RA_ICU_H

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

/* Register Offsets *********************************************************/

#define R_ICU_IRQCR_OFFSET                0x0000  /* IRQ Control Register (8-bits) */
#define R_ICU_NMISR_OFFSET                0x0140  /* Non-Maskable Interrupt Status Register (16-bits) */
#define R_ICU_NMIER_OFFSET                0x0120  /* Non-Maskable Interrupt Enable Register (16-bits) */
#define R_ICU_NMICLR_OFFSET               0x0130  /* Non-Maskable Interrupt Status Clear Register (16-bits) */
#define R_ICU_NMICR_OFFSET                0x0100  /* NMI Pin Interrupt Control Register (8-bits) */
#define R_ICU_IELSR_OFFSET                0x0300  /* ICU Event Link Setting Register (32-bits) */
#define R_ICU_DELSR_OFFSET                0x0280  /* DMAC Event Link Setting Register (32-bits) */
#define R_ICU_SELSR0_OFFSET               0x0200  /* SYS Event Link Setting Register (16-bits) */
#define R_ICU_WUPEN_OFFSET                0x01a0  /* Wake Up Interrupt Enable Register (32-bits) */

/* Register Addresses *******************************************************/

#define R_ICU_IRQCR(p)                    (R_ICU_BASE + R_ICU_IRQCR_OFFSET + (p)*0x0001)
#define R_ICU_NMISR                       (R_ICU_BASE + R_ICU_NMISR_OFFSET)
#define R_ICU_NMIER                       (R_ICU_BASE + R_ICU_NMIER_OFFSET)
#define R_ICU_NMICLR                      (R_ICU_BASE + R_ICU_NMICLR_OFFSET)
#define R_ICU_NMICR                       (R_ICU_BASE + R_ICU_NMICR_OFFSET)
#define R_ICU_IELSR(p)                    (R_ICU_BASE + R_ICU_IELSR_OFFSET + (p)*0x0004)
#define R_ICU_DELSR(p)                    (R_ICU_BASE + R_ICU_DELSR_OFFSET + (p)*0x0004)
#define R_ICU_SELSR0                      (R_ICU_BASE + R_ICU_SELSR0_OFFSET)
#define R_ICU_WUPEN                       (R_ICU_BASE + R_ICU_WUPEN_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* IRQ Control Register (8-bits) */

#define R_ICU_IRQCR_SIZE                  15
#define R_ICU_IRQCR_FLTEN                 (1 <<  7) /* 80: IRQ Digital Filter Enable */
#define R_ICU_IRQCR_FCLKSEL_SHIFT         (4)       /* 10: IRQ Digital Filter Sampling Clock Select */
#define R_ICU_IRQCR_FCLKSEL_MASK          (0x03)
#define R_ICU_IRQCR_IRQMD                 (0)       /* 01: IRQ Detection Sense Select */
#define R_ICU_IRQCR_IRQMD_MASK            (0x03)

/* Non-Maskable Interrupt Status Register (16-bits) */

#define R_ICU_NMISR_SPEST                 (1 << 12) /* 1000: CPU Stack pointer monitor Interrupt Status Flag */
#define R_ICU_NMISR_BUSMST                (1 << 11) /* 800: MPU Bus Master Error Interrupt Status Flag */
#define R_ICU_NMISR_BUSSST                (1 << 10) /* 400: MPU Bus Slave Error Interrupt Status Flag */
#define R_ICU_NMISR_RECCST                (1 <<  9) /* 200: RAM ECC Error Interrupt Status Flag */
#define R_ICU_NMISR_RPEST                 (1 <<  8) /* 100: RAM Parity Error Interrupt Status Flag */
#define R_ICU_NMISR_NMIST                 (1 <<  7) /* 80: NMI Status Flag */
#define R_ICU_NMISR_OSTST                 (1 <<  6) /* 40: Oscillation Stop Detection Interrupt Status Flag */
#define R_ICU_NMISR_VBATTST               (1 <<  4) /* 10: VBATT monitor Interrupt Status Flag */
#define R_ICU_NMISR_LVD2ST                (1 <<  3) /* 08: Voltage-Monitoring 2 Interrupt Status Flag */
#define R_ICU_NMISR_LVD1ST                (1 <<  2) /* 04: Voltage-Monitoring 1 Interrupt Status Flag */
#define R_ICU_NMISR_WDTST                 (1 <<  1) /* 02: WDT Underflow/Refresh Error Status Flag */
#define R_ICU_NMISR_IWDTST                (1 <<  0) /* 01: IWDT Underflow/Refresh Error Status Flag */

/* Non-Maskable Interrupt Enable Register (16-bits) */

#define R_ICU_NMIER_SPEEN                 (1 << 12) /* 1000: CPU Stack pointer monitor Interrupt Enable */
#define R_ICU_NMIER_BUSMEN                (1 << 11) /* 800: MPU Bus Master Error Interrupt Enable */
#define R_ICU_NMIER_BUSSEN                (1 << 10) /* 400: MPU Bus Slave Error Interrupt Enable */
#define R_ICU_NMIER_RECCEN                (1 <<  9) /* 200: RAM ECC Error Interrupt Enable */
#define R_ICU_NMIER_RPEEN                 (1 <<  8) /* 100: RAM Parity Error Interrupt Enable */
#define R_ICU_NMIER_NMIEN                 (1 <<  7) /* 80: NMI Pin Interrupt Enable */
#define R_ICU_NMIER_OSTEN                 (1 <<  6) /* 40: Oscillation Stop Detection Interrupt Enable */
#define R_ICU_NMIER_VBATTEN               (1 <<  4) /* 10: VBATT monitor Interrupt Enable */
#define R_ICU_NMIER_LVD2EN                (1 <<  3) /* 08: Voltage-Monitoring 2 Interrupt Enable */
#define R_ICU_NMIER_LVD1EN                (1 <<  2) /* 04: Voltage-Monitoring 1 Interrupt Enable */
#define R_ICU_NMIER_WDTEN                 (1 <<  1) /* 02: WDT Underflow/Refresh Error Interrupt Enable */
#define R_ICU_NMIER_IWDTEN                (1 <<  0) /* 01: IWDT Underflow/Refresh Error Interrupt Enable */

/* Non-Maskable Interrupt Status Clear Register (16-bits) */

#define R_ICU_NMICLR_SPECLR               (1 << 12) /* 1000: CPU Stack Pointer Monitor Interrupt Clear */
#define R_ICU_NMICLR_BUSMCLR              (1 << 11) /* 800: Bus Master Error Clear */
#define R_ICU_NMICLR_BUSSCLR              (1 << 10) /* 400: Bus Slave Error Clear */
#define R_ICU_NMICLR_RECCCLR              (1 <<  9) /* 200: SRAM ECC Error Clear */
#define R_ICU_NMICLR_RPECLR               (1 <<  8) /* 100: SRAM Parity Error Clear */
#define R_ICU_NMICLR_NMICLR               (1 <<  7) /* 80: NMI Clear */
#define R_ICU_NMICLR_OSTCLR               (1 <<  6) /* 40: OST Clear */
#define R_ICU_NMICLR_VBATTCLR             (1 <<  4) /* 10: VBATT Clear */
#define R_ICU_NMICLR_LVD2CLR              (1 <<  3) /* 08: LVD2 Clear */
#define R_ICU_NMICLR_LVD1CLR              (1 <<  2) /* 04: LVD1 Clear */
#define R_ICU_NMICLR_WDTCLR               (1 <<  1) /* 02: WDT Clear */
#define R_ICU_NMICLR_IWDTCLR              (1 <<  0) /* 01: IWDT Clear */

/* NMI Pin Interrupt Control Register (8-bits) */

#define R_ICU_NMICR_NFLTEN                (1 <<  7) /* 80: NMI Digital Filter Enable */
#define R_ICU_NMICR_NFCLKSEL_SHIFT        (4)       /* 10: NMI Digital Filter Sampling Clock Select */
#define R_ICU_NMICR_NFCLKSEL_MASK         (0x03)
#define R_ICU_NMICR_NMIMD                 (1 <<  0) /* 01: NMI Detection Set */

/* ICU Event Link Setting Register (32-bits) */

#define R_ICU_IELSR_SIZE                  32
#define R_ICU_IELSR_DTCE                  (1 << 24) /* 1000000: DTC Activation Enable */
#define R_ICU_IELSR_IR                    (1 << 16) /* 10000: Interrupt Status Flag */
#define R_ICU_IELSR_IELS_SHIFT            (0)       /* 01: ICU Event selection to NVIC Set the number for the event signal to be linked . */
#define R_ICU_IELSR_IELS_MASK             (0xff)

/* DMAC Event Link Setting Register (32-bits) */

#define R_ICU_DELSR_SIZE                  4
#define R_ICU_DELSR_DELS_SHIFT            (0)     /* 01: Event selection to DMAC Start request */
#define R_ICU_DELSR_DELS_MASK             (0xff)

/* SYS Event Link Setting Register (16-bits) */

#define R_ICU_SELSR0_SELS_SHIFT           (0)     /* 01: SYS Event Link Select */
#define R_ICU_SELSR0_SELS_MASK            (0xff)

/* Wake Up Interrupt Enable Register (32-bits) */

#define R_ICU_WUPEN_IIC0WUPEN             (1 << 31) /* 80000000: IIC0 address match interrupt S/W standby returns enable */
#define R_ICU_WUPEN_AGT1CBWUPEN           (1 << 30) /* 40000000: AGT1 compare match B interrupt S/W standby returns enable */
#define R_ICU_WUPEN_AGT1CAWUPEN           (1 << 29) /* 20000000: AGT1 compare match A interrupt S/W standby returns enable */
#define R_ICU_WUPEN_AGT1UDWUPEN           (1 << 28) /* 10000000: AGT1 underflow interrupt S/W standby returns enable */
#define R_ICU_WUPEN_USBFSWUPEN            (1 << 27) /* 8000000: USBFS interrupt S/W standby returns enable */
#define R_ICU_WUPEN_RTCPRDWUPEN           (1 << 25) /* 2000000: RCT period interrupt S/W standby returns enable */
#define R_ICU_WUPEN_RTCALMWUPEN           (1 << 24) /* 1000000: RTC alarm interrupt S/W standby returns enable */
#define R_ICU_WUPEN_ACMPLP0WUPEN          (1 << 23) /* 800000: ACMPLP0 interrupt S/W standby returns enable */
#define R_ICU_WUPEN_VBATTWUPEN            (1 << 20) /* 100000: VBATT monitor interrupt S/W standby returns enable */
#define R_ICU_WUPEN_LVD2WUPEN             (1 << 19) /* 80000: LVD2 interrupt S/W standby returns enable */
#define R_ICU_WUPEN_LVD1WUPEN             (1 << 18) /* 40000: LVD1 interrupt S/W standby returns enable */
#define R_ICU_WUPEN_KEYWUPEN              (1 << 17) /* 20000: Key interrupt S/W standby returns enable */
#define R_ICU_WUPEN_IWDTWUPEN             (1 << 16) /* 10000: IWDT interrupt S/W standby returns enable */
#define R_ICU_WUPEN_IRQWUPEN15            (1 << 15) /* 8000: IRQ15 interrupt S/W standby returns enable */
#define R_ICU_WUPEN_IRQWUPEN14            (1 << 14) /* 4000: IRQ14 interrupt S/W standby returns enable */
#define R_ICU_WUPEN_IRQWUPEN12            (1 << 12) /* 1000: IRQ12 interrupt S/W standby returns enable */
#define R_ICU_WUPEN_IRQWUPEN11            (1 << 11) /* 800: IRQ11 interrupt S/W standby returns enable */
#define R_ICU_WUPEN_IRQWUPEN10            (1 << 10) /* 400: IRQ10 interrupt S/W standby returns enable */
#define R_ICU_WUPEN_IRQWUPEN9             (1 <<  9) /* 200: IRQ9 interrupt S/W standby returns enable */
#define R_ICU_WUPEN_IRQWUPEN8             (1 <<  8) /* 100: IRQ8 interrupt S/W standby returns enable */
#define R_ICU_WUPEN_IRQWUPEN7             (1 <<  7) /* 80: IRQ7 interrupt S/W standby returns enable */
#define R_ICU_WUPEN_IRQWUPEN6             (1 <<  6) /* 40: IRQ6 interrupt S/W standby returns enable */
#define R_ICU_WUPEN_IRQWUPEN5             (1 <<  5) /* 20: IRQ5 interrupt S/W standby returns enable */
#define R_ICU_WUPEN_IRQWUPEN4             (1 <<  4) /* 10: IRQ4 interrupt S/W standby returns enable */
#define R_ICU_WUPEN_IRQWUPEN3             (1 <<  3) /* 08: IRQ3 interrupt S/W standby returns enable */
#define R_ICU_WUPEN_IRQWUPEN2             (1 <<  2) /* 04: IRQ2 interrupt S/W standby returns enable */
#define R_ICU_WUPEN_IRQWUPEN1             (1 <<  1) /* 02: IRQ1 interrupt S/W standby returns enable */
#define R_ICU_WUPEN_IRQWUPEN0             (1 <<  0) /* 01: IRQ0 interrupt S/W standby returns enable */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_RA4M1_HARDWARE_RA_ICU_H */
