/****************************************************************************
 * arch/arm/src/ra8m1/hardware/ra8m1_icu.h
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

#ifndef __ARCH_ARM_SRC_RA8M1_HARDWARE_RA8M1_ICU_H
#define __ARCH_ARM_SRC_RA8M1_HARDWARE_RA8M1_ICU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/ra8m1_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define R_ICU_IRQCR_OFFSET                  0x0000  /* IRQ Control Register %s (8-bits) */
#define R_ICU_NMICR_OFFSET                  0x0010  /* NMI Pin Interrupt Control Register (8-bits) */
#define R_ICU_SWIRQ_S_OFFSET                0x6010  /* Software Interrupt Request Register for Secure Interrupt (8-bits) */
#define R_ICU_SWIRQ_NS_OFFSET               0x6020  /* Software Interrupt Request Register for Non-secure Interrupt (8-bits) */
#define R_ICU_IENMIER_OFFSET                0x6060  /* Integrated Error NMI Interrupt Enable Register for CPU (16-bits) */
#define R_ICU_NMIER_OFFSET                  0x6100  /* Non-Maskable Interrupt Enable Register (16-bits) */
#define R_ICU_NMICLR_OFFSET                 0x6110  /* Non-Maskable Interrupt Status Clear Register (16-bits) */
#define R_ICU_NMISR_OFFSET                  0x6120  /* Non-Maskable Interrupt Status Register (16-bits) */
#define R_ICU_WUPEN0_OFFSET                 0x61a0  /* Wake Up Interrupt Enable Register 0 (32-bits) */
#define R_ICU_WUPEN1_OFFSET                 0x61a4  /* Wake Up Interrupt Enable Register 1 (32-bits) */
#define R_ICU_SELSR0_OFFSET                 0x6200  /* SYS Event Link Setting Register (16-bits) */
#define R_ICU_IELSR_OFFSET                  0x6300  /* ICU Event Link Setting Register %s (32-bits) */

/* Register Addresses *******************************************************/

/* ICU Registers */

#define R_ICU_IRQCR(p)                     (R_ICU_BASE + R_ICU_IRQCR_OFFSET + (p)*0x0001)
#define R_ICU_NMICR                        (R_ICU_BASE + R_ICU_NMICR_OFFSET)
#define R_ICU_SWIRQ_S                      (R_ICU_BASE + R_ICU_SWIRQ_S_OFFSET)
#define R_ICU_SWIRQ_NS                     (R_ICU_BASE + R_ICU_SWIRQ_NS_OFFSET)
#define R_ICU_IENMIER                      (R_ICU_BASE + R_ICU_IENMIER_OFFSET)
#define R_ICU_NMIER                        (R_ICU_BASE + R_ICU_NMIER_OFFSET)
#define R_ICU_NMICLR                       (R_ICU_BASE + R_ICU_NMICLR_OFFSET)
#define R_ICU_NMISR                        (R_ICU_BASE + R_ICU_NMISR_OFFSET)
#define R_ICU_WUPEN0                       (R_ICU_BASE + R_ICU_WUPEN0_OFFSET)
#define R_ICU_WUPEN1                       (R_ICU_BASE + R_ICU_WUPEN1_OFFSET)
#define R_ICU_SELSR0                       (R_ICU_BASE + R_ICU_SELSR0_OFFSET)
#define R_ICU_IELSR(p)                     (R_ICU_BASE + R_ICU_IELSR_OFFSET + (p)*0x0004)

/* Register Bitfield Definitions ********************************************/

/* IRQ Control Register %s (8-bits) *****************************************/

#define R_ICU_IRQCR_SIZE 16
#define R_ICU_IRQCR_IRQMD_SHIFT (0)
#define R_ICU_IRQCR_IRQMD_MASK (0x3)
#  define R_ICU_IRQCR_IRQMD_FALLING_EDGE (0 << R_ICU_IRQCR_IRQMD_SHIFT)              /* Falling edge */
#  define R_ICU_IRQCR_IRQMD_RISING_EDGE (1 << R_ICU_IRQCR_IRQMD_SHIFT)               /* Rising edge */
#  define R_ICU_IRQCR_IRQMD_RISING_AND_FALLING_EDGES (2 << R_ICU_IRQCR_IRQMD_SHIFT)  /* Rising and falling edges */
#  define R_ICU_IRQCR_IRQMD_LOW_LEVEL (3 << R_ICU_IRQCR_IRQMD_SHIFT)                 /* Low level */
#define R_ICU_IRQCR_FCLKSEL_SHIFT (4)
#define R_ICU_IRQCR_FCLKSEL_MASK (0x3)
#  define R_ICU_IRQCR_FCLKSEL_PCLKB (0 << R_ICU_IRQCR_FCLKSEL_SHIFT)                 /* PCLKB */
#  define R_ICU_IRQCR_FCLKSEL_PCLKB_8 (1 << R_ICU_IRQCR_FCLKSEL_SHIFT)               /* PCLKB/8 */
#  define R_ICU_IRQCR_FCLKSEL_PCLKB_32 (2 << R_ICU_IRQCR_FCLKSEL_SHIFT)              /* PCLKB/32 */
#  define R_ICU_IRQCR_FCLKSEL_PCLKB_64 (3 << R_ICU_IRQCR_FCLKSEL_SHIFT)              /* PCLKB/64 */
#define R_ICU_IRQCR_FLTEN (1 <<  7)                                                  /* 80: IRQ Digital Filter Enable */

/* NMI Pin Interrupt Control Register (8-bits) ******************************/

#define R_ICU_NMICR_NMIMD (1 <<  0)                                        /* 01: NMI Detection Set */
#define R_ICU_NMICR_NFCLKSEL_SHIFT (4)
#define R_ICU_NMICR_NFCLKSEL_MASK (0x3)
#  define R_ICU_NMICR_NFCLKSEL_PCLKB (0 << R_ICU_NMICR_NFCLKSEL_SHIFT)     /* PCLKB */
#  define R_ICU_NMICR_NFCLKSEL_PCLKB_8 (1 << R_ICU_NMICR_NFCLKSEL_SHIFT)   /* PCLKB/8 */
#  define R_ICU_NMICR_NFCLKSEL_PCLKB_32 (2 << R_ICU_NMICR_NFCLKSEL_SHIFT)  /* PCLKB/32 */
#  define R_ICU_NMICR_NFCLKSEL_PCLKB_64 (3 << R_ICU_NMICR_NFCLKSEL_SHIFT)  /* PCLKB/64 */
#define R_ICU_NMICR_NFLTEN (1 <<  7)                                       /* 80: NMI Digital Filter Enable */

/* Software Interrupt Request Register for Secure Interrupt (8-bits) ********/

#define R_ICU_SWIRQ_S_SWIRQS (1 <<  0)  /* 01: Generates an interrupt for the other CPU subsystem. */

/* Software Interrupt Request Register for Non-secure Interrupt (8-bits) ****/

#define R_ICU_SWIRQ_NS_SWIRQNS (1 <<  0)  /* 01: Generates an interrupt for the other CPU subsystem. */

/* Integrated Error NMI Interrupt Enable Register for CPU (16-bits) *********/

#define R_ICU_IENMIER_CMEN (1 <<  0)   /* 01: Integrated Common Memory error nmi Enable */
#define R_ICU_IENMIER_LMEN (1 <<  1)   /* 02: Integrated Local Memory error nmi Enable */
#define R_ICU_IENMIER_BUSEN (1 <<  2)  /* 04: Integrated BUS error nmi Enable */

/* Non-Maskable Interrupt Enable Register (16-bits) *************************/

#define R_ICU_NMIER_IWDTEN (1 <<  0)  /* 01: IWDT Underflow/Refresh Error Interrupt Enable */
#define R_ICU_NMIER_WDTEN (1 <<  1)   /* 02: WDT Underflow/Refresh Error Interrupt Enable */
#define R_ICU_NMIER_PVD1EN (1 <<  2)  /* 04: Voltage-Monitoring 1 Interrupt Enable */
#define R_ICU_NMIER_PVD2EN (1 <<  3)  /* 08: Voltage-Monitoring 2 Interrupt Enable */
#define R_ICU_NMIER_OSTEN (1 <<  6)   /* 40: Oscillation Stop Detection Interrupt Enable */
#define R_ICU_NMIER_NMIEN (1 <<  7)   /* 80: NMI Pin Interrupt Enable */
#define R_ICU_NMIER_BUSEN (1 << 12)   /* 1000: BUS error Interrupt Enable */
#define R_ICU_NMIER_CMEN (1 << 13)    /* 2000: Common Memory error Interrupt Enable */
#define R_ICU_NMIER_LUEN (1 << 15)    /* 8000: LockUp Interrupt Enable */

/* Non-Maskable Interrupt Status Clear Register (16-bits) *******************/

#define R_ICU_NMICLR_IWDTCLR (1 <<  0)  /* 01: IWDT Clear */
#define R_ICU_NMICLR_WDTCLR (1 <<  1)   /* 02: WDT Clear */
#define R_ICU_NMICLR_PVD1CLR (1 <<  2)  /* 04: PVD1 Clear */
#define R_ICU_NMICLR_PVD2CLR (1 <<  3)  /* 08: PVD2 Clear */
#define R_ICU_NMICLR_OSTCLR (1 <<  6)   /* 40: OST Clear */
#define R_ICU_NMICLR_NMICLR (1 <<  7)   /* 80: NMI Clear */
#define R_ICU_NMICLR_BUSCLR (1 << 12)   /* 1000: Bus Clear */
#define R_ICU_NMICLR_CMCLR (1 << 13)    /* 2000: CM Clear */
#define R_ICU_NMICLR_LUCLR (1 << 15)    /* 8000: LU Clear */

/* Non-Maskable Interrupt Status Register (16-bits) *************************/

#define R_ICU_NMISR_IWDTST (1 <<  0)  /* 01: IWDT Underflow/Refresh Error Status Flag */
#define R_ICU_NMISR_WDTST (1 <<  1)   /* 02: WDT Underflow/Refresh Error Status Flag */
#define R_ICU_NMISR_PVD1ST (1 <<  2)  /* 04: Voltage-Monitoring 1 Interrupt Status Flag */
#define R_ICU_NMISR_PVD2ST (1 <<  3)  /* 08: Voltage-Monitoring 2 Interrupt Status Flag */
#define R_ICU_NMISR_OSTST (1 <<  6)   /* 40: Oscillation Stop Detection Interrupt Status Flag */
#define R_ICU_NMISR_NMIST (1 <<  7)   /* 80: NMI Status Flag */
#define R_ICU_NMISR_BUSST (1 << 12)   /* 1000: BUS error Interrupt Status Flag */
#define R_ICU_NMISR_CMST (1 << 13)    /* 2000: Common Memory error Interrupt Status Flag */
#define R_ICU_NMISR_LUST (1 << 15)    /* 8000: LockUp Interrupt Status Flag */

/* Wake Up Interrupt Enable Register 0 (32-bits) ****************************/

#define R_ICU_WUPEN0_IRQWUPEN0 (1 <<  0)    /* 01: IRQ0 Interrupt Deep Sleep/Software Standby Returns Enable bit */
#define R_ICU_WUPEN0_IRQWUPEN1 (1 <<  1)    /* 02: IRQ1 Interrupt Deep Sleep/Software Standby Returns Enable bit */
#define R_ICU_WUPEN0_IRQWUPEN2 (1 <<  2)    /* 04: IRQ2 Interrupt Deep Sleep/Software Standby Returns Enable bit */
#define R_ICU_WUPEN0_IRQWUPEN3 (1 <<  3)    /* 08: IRQ3 Interrupt Deep Sleep/Software Standby Returns Enable bit */
#define R_ICU_WUPEN0_IRQWUPEN4 (1 <<  4)    /* 10: IRQ4 Interrupt Deep Sleep/Software Standby Returns Enable bit */
#define R_ICU_WUPEN0_IRQWUPEN5 (1 <<  5)    /* 20: IRQ5 Interrupt Deep Sleep/Software Standby Returns Enable bit */
#define R_ICU_WUPEN0_IRQWUPEN6 (1 <<  6)    /* 40: IRQ6 Interrupt Deep Sleep/Software Standby Returns Enable bit */
#define R_ICU_WUPEN0_IRQWUPEN7 (1 <<  7)    /* 80: IRQ7 Interrupt Deep Sleep/Software Standby Returns Enable bit */
#define R_ICU_WUPEN0_IRQWUPEN8 (1 <<  8)    /* 100: IRQ8 Interrupt Deep Sleep/Software Standby Returns Enable bit */
#define R_ICU_WUPEN0_IRQWUPEN9 (1 <<  9)    /* 200: IRQ9 Interrupt Deep Sleep/Software Standby Returns Enable bit */
#define R_ICU_WUPEN0_IRQWUPEN10 (1 << 10)   /* 400: IRQ10 Interrupt Deep Sleep/Software Standby Returns Enable bit */
#define R_ICU_WUPEN0_IRQWUPEN11 (1 << 11)   /* 800: IRQ11 Interrupt Deep Sleep/Software Standby Returns Enable bit */
#define R_ICU_WUPEN0_IRQWUPEN12 (1 << 12)   /* 1000: IRQ12 Interrupt Deep Sleep/Software Standby Returns Enable bit */
#define R_ICU_WUPEN0_IRQWUPEN13 (1 << 13)   /* 2000: IRQ13 Interrupt Deep Sleep/Software Standby Returns Enable bit */
#define R_ICU_WUPEN0_IRQWUPEN14 (1 << 14)   /* 4000: IRQ14 Interrupt Deep Sleep/Software Standby Returns Enable bit */
#define R_ICU_WUPEN0_IRQWUPEN15 (1 << 15)   /* 8000: IRQ15 Interrupt Deep Sleep/Software Standby Returns Enable bit */
#define R_ICU_WUPEN0_IWDTWUPEN (1 << 16)    /* 10000: IWDT Interrupt Deep Sleep/Software Standby Returns Enable bit */
#define R_ICU_WUPEN0_PVD1WUPEN (1 << 18)    /* 40000: PVD1 Interrupt Deep Sleep/Software Standby Returns Enable bit */
#define R_ICU_WUPEN0_PVD2WUPEN (1 << 19)    /* 80000: PVD2 Interrupt Deep Sleep/Software Standby Returns Enable bit */
#define R_ICU_WUPEN0_VBATTWUPEN (1 << 20)   /* 100000: VBATT Monitor Interrupt Deep Sleep/Software Standby Returns Enable bit */
#define R_ICU_WUPEN0_RTCALMWUPEN (1 << 24)  /* 1000000: RTC Alarm Interrupt Deep Sleep/Software Standby Returns Enable bit */
#define R_ICU_WUPEN0_RTCPRDWUPEN (1 << 25)  /* 2000000: RCT Period Interrupt Deep Sleep/Software Standby Returns Enable bit */
#define R_ICU_WUPEN0_USBHSWUPEN (1 << 26)   /* 4000000: USBHS Interrupt Deep Sleep/Software Standby Returns Enable bit */
#define R_ICU_WUPEN0_USBFSWUPEN (1 << 27)   /* 8000000: USBFS0 Interrupt Deep Sleep/Software Standby Returns Enable bit */
#define R_ICU_WUPEN0_AGT1UDWUPEN (1 << 28)  /* 10000000: AGT1 Underflow Interrupt Deep Sleep/Software Standby Returns Enable bit */
#define R_ICU_WUPEN0_AGT1CAWUPEN (1 << 29)  /* 20000000: AGT1 Compare Match A Interrupt Deep Sleep/Software Standby Returns Enable bit */
#define R_ICU_WUPEN0_AGT1CBWUPEN (1 << 30)  /* 40000000: AGT1 Compare Match B Interrupt Deep Sleep/Software Standby Returns Enable bit */
#define R_ICU_WUPEN0_RIIC0WUPEN (1 << 31)   /* 80000000: RIIC0 Address Match Interrupt Deep Sleep/Software Standby Returns Enable bit */

/* Wake Up Interrupt Enable Register 1 (32-bits) ****************************/

#define R_ICU_WUPEN1_COMPHS0WUPEN (1 <<  3)  /* 08: Comparator-HS0 Interrupt Deep Sleep/Software Standby Returns Enable bit */
#define R_ICU_WUPEN1_ULP0UWUPEN (1 <<  8)    /* 100: ULPT0 Underflow Interrupt Deep Sleep/Software Standby Returns Enable bit */
#define R_ICU_WUPEN1_ULP0AWUPEN (1 <<  9)    /* 200: ULPT0 Compare Match A Interrupt Deep Sleep/Software Standby Returns Enable bit */
#define R_ICU_WUPEN1_ULP0BWUPEN (1 << 10)    /* 400: ULPT0 Compare Match B Interrupt Deep Sleep/Software Standby Returns Enable bit */
#define R_ICU_WUPEN1_I3CWUPEN (1 << 11)      /* 800: I3C Wakeup Condition Detection Interrupt Deep Sleep/Software Standby Returns Enable bit */
#define R_ICU_WUPEN1_ULP1UWUPEN (1 << 12)    /* 1000: ULPT1 Underflow Interrupt Deep Sleep/Software Standby Returns Enable bit */
#define R_ICU_WUPEN1_ULP1AWUPEN (1 << 13)    /* 2000: ULPT1 Compare Match A Interrupt Deep Sleep/Software Standby Returns Enable bit */
#define R_ICU_WUPEN1_ULP1BWUPEN (1 << 14)    /* 4000: ULPT1 Compare Match B Interrupt Deep Sleep/Software Standby Returns Enable bit */

/* SYS Event Link Setting Register (16-bits) ********************************/

#define R_ICU_SELSR0_SELS_SHIFT (0)
#define R_ICU_SELSR0_SELS_MASK (0x1ff)
#  define R_ICU_SELSR0_SELS_V000000000 (0 << R_ICU_SELSR0_SELS_SHIFT)  /* Disable event output to the associated low-power mode module */

/* ICU Event Link Setting Register %s (32-bits) *****************************/

#define R_ICU_IELSR_SIZE 96
#define R_ICU_IELSR_IELS_SHIFT (0)
#define R_ICU_IELSR_IELS_MASK (0x1ff)
#  define R_ICU_IELSR_IELS_NOTHING_IS_SELECTED (0 << R_ICU_IELSR_IELS_SHIFT)  /* Nothing is selected */
#define R_ICU_IELSR_IR (1 << 16)                                              /* 10000: Interrupt Status Flag */
#define R_ICU_IELSR_DTCE (1 << 24)                                            /* 1000000: DTC Activation Enable */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_RA8M1_HARDWARE_RA8M1_ICU_H */
