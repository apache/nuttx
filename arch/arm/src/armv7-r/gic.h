/****************************************************************************
 * arch/arm/src/armv7-r/gic.h
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

/* Reference:
 *   Cortex™-A9 MPCore, Revision: r4p1, Technical Reference Manual, ARM DDI
 *   0407I (ID091612).
 *
 *   Includes some removed registers from the r2p2 version as well. ARM DDI
 *   0407F (ID050110)
 */

#ifndef __ARCH_ARM_SRC_ARMV7_R_GIC_H
#define __ARCH_ARM_SRC_ARMV7_R_GIC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "nuttx/config.h"

#ifndef __ASSEMBLY__
#  include <sys/types.h>
#  include <stdint.h>
#endif

#include "mpcore.h"
#include "arm_internal.h"

#ifdef CONFIG_ARMV7R_HAVE_GICv2

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Generic indexing helpers *************************************************/

/* 1x32 bit field per register */

#define GIC_INDEX1(n)              (n)                       /* 1 field per word */
#define GIC_OFFSET1(n)             (GIC_INDEX1(n) << 2)      /* 32-bit word offset */
#define GIC_SHIFT1(n)              (0)                       /* No shift */
#define GIC_MASK1(n)               (0xffffffff)              /* Whole word */

/* 2x16 bit field per register */

#define GIC_INDEX2(n)              (n >> 1)                  /* 2 fields per word */
#define GIC_OFFSET2(n)             (GIC_INDEX2(n) << 2)      /* 32-bit word offset */
#define GIC_SHIFT2(n)              (((n) & 1) << 4)          /* Shift 16-bits per field */
#define GIC_MASK2(n)               (0xffff << GIC_SHIFT2(n)) /* 16-bit mask */

/* 4x8 bit field per register */

#define GIC_INDEX4(n)              (n >> 2)                  /* 4 fields per word */
#define GIC_OFFSET4(n)             (GIC_INDEX4(n) << 2)      /* 32-bit word offset */
#define GIC_SHIFT4(n)              (((n) & 3) << 3)          /* Shift 8-bits per field */
#define GIC_MASK4(n)               (0xff << GIC_SHIFT4(n))   /* 8-bit mask */

/* 8x4 bit field per register */

#define GIC_INDEX8(n)              (n >> 3)                  /* 8 fields per word */
#define GIC_OFFSET8(n)             (GIC_INDEX8(n) << 2)      /* 32-bit word offset */
#define GIC_SHIFT8(n)              (((n) & 7) << 2)          /* Shift 4-bits per field */
#define GIC_MASK8(n)               (15 << GIC_SHIFT8(n))     /* 4-bit mask */

/* 16x2 bit field per register */

#define GIC_INDEX16(n)             (n >> 4)                  /* 16 fields per word */
#define GIC_OFFSET16(n)            (GIC_INDEX16(n) << 2)     /* 32-bit word offset */
#define GIC_SHIFT16(n)             (((n) & 15) << 1)         /* Shift 2-bits per field */
#define GIC_MASK16(n)              (3 << GIC_SHIFT16(n))     /* 2-bit mask */

/* 32x1 bit field per register */

#define GIC_INDEX32(n)             (n >> 5)                  /* 32 fields per word */
#define GIC_OFFSET32(n)            (GIC_INDEX32(n) << 2)     /* 32-bit word offset */
#define GIC_SHIFT32(n)             ((n) & 31)                /* Shift 1-bit per field */
#define GIC_MASK32(n)              (1 << GIC_SHIFT32(n))     /* 1-bit mask */

/* GIC Register Offsets *****************************************************/

/* CPU Interface registers */

#define GIC_ICCICR_OFFSET          0x0000    /* CPU Interface Control Register */
#define GIC_ICCPMR_OFFSET          0x0004    /* Interrupt Priority Mask Register */
#define GIC_ICCBPR_OFFSET          0x0008    /* Binary point Register */
#define GIC_ICCIAR_OFFSET          0x000c    /* Interrupt Acknowledge */
#define GIC_ICCEOIR_OFFSET         0x0010    /* End of interrupt */
#define GIC_ICCRPR_OFFSET          0x0014    /* Running interrupt */
#define GIC_ICCHPIR_OFFSET         0x0018    /* Highest pending interrupt */
#define GIC_ICCABPR_OFFSET         0x001c    /* Aliased Non-secure Binary Point Register */
#define GIC_ICCAIAR_OFFSET         0x0020    /* Aliased Interrupt Acknowledge Register */
#define GIC_ICCAEOIR_OFFSET        0x0024    /* Aliased End of Interrupt Register */
#define GIC_ICCAHPIR_OFFSET        0x0028    /* Aliased Highest Priority Pending Interrupt Register */
                                             /* 0x002c-0x003c: Reserved */
                                             /* 0x0040-0x00cf: Implementation defined */
#define GIC_ICCAPR1_OFFSET         0x00d0    /* Active Priorities Register 1 */
#define GIC_ICCAPR2_OFFSET         0x00d4    /* Active Priorities Register 2 */
#define GIC_ICCAPR3_OFFSET         0x00d8    /* Active Priorities Register 3 */
#define GIC_ICCAPR4_OFFSET         0x00dc    /* Active Priorities Register 4 */
#define GIC_ICCNSAPR1_OFFSET       0x00e0    /* Non-secure Active Priorities Register 1 */
#define GIC_ICCNSAPR2_OFFSET       0x00e4    /* Non-secure Active Priorities Register 2 */
#define GIC_ICCNSAPR3_OFFSET       0x00e8    /* Non-secure Active Priorities Register 3 */
#define GIC_ICCNSAPR4_OFFSET       0x00ec    /* Non-secure Active Priorities Register 4 */
                                             /* 0x00ed-0x00f8: Reserved */
#define GIC_ICCIDR_OFFSET          0x00fc    /* CPU Interface Implementer ID Register */
#define GIC_ICCDIR_OFFSET          0x1000    /* Deactivate Interrupt Register */

/* Distributor Registers */

#define GIC_ICDDCR_OFFSET          0x0000    /* Distributor Control Register */
#define GIC_ICDICTR_OFFSET         0x0004    /* Interrupt Controller Type Register */
#define GIC_ICDIIDR_OFFSET         0x0008    /* Distributor Implementer ID Register */
                                             /* 0x000c-0x001c: Reserved */
                                             /* 0x0020-0x003c: Implementation defined */
                                             /* 0x0040-0x007c: Reserved */

/* Interrupt Security Registers: 0x0080-0x009c */

#define GIC_ICDISR_OFFSET(n)       (0x0080 + GIC_OFFSET32(n))

/* Interrupt Set-Enable Registers: 0x0100-0x011c */

#define GIC_ICDISER_OFFSET(n)      (0x0100 + GIC_OFFSET32(n))

/* Interrupt Clear-Enable Registers: 0x0180-0x019c */

#define GIC_ICDICER_OFFSET(n)      (0x0180 + GIC_OFFSET32(n))

/* Interrupt Set-Pending Registers: 0x0200-0x027c */

#define GIC_ICDISPR_OFFSET(n)      (0x0200 + GIC_OFFSET32(n))

/* Interrupt Clear-Pending Registers: 0x0280-0x02fc */

#define GIC_ICDICPR_OFFSET(n)      (0x0280 + GIC_OFFSET32(n))

/* GICv2 Interrupt Set-Active Registers: 0x0300-0x31c */

#define GIC_ICDSAR_OFFSET(n)       (0x0300 + GIC_OFFSET32(n))

/* Interrupt Clear-Active Registers: 0x380-0x3fc */

#define GIC_ICDCAR_OFFSET(n)       (0x0380 + GIC_OFFSET32(n))

/* Interrupt Priority Registers: 0x0400-0x04fc */

#define GIC_ICDIPR_OFFSET(n)       (0x0400 + GIC_OFFSET4(n))

/* 0x0500-0x07fc: Reserved */

/* Interrupt Processor Target Registers: 0x0800-0x08fc */

#define GIC_ICDIPTR_OFFSET(n)      (0x0800 + GIC_OFFSET4(n))

/* 0x0900-0x0bfc: Reserved */

/* Interrupt Configuration Registers: 0x0c00-0x0c3c */

#define GIC_ICDICFR_OFFSET(n)      (0x0c00 + GIC_OFFSET16(n))

/* 0x0d00-0x0dfc: Implementation defined */

/* PPI Status Register: 0x0d00 */

/* SPI Status Registers: 0x0d04-0x0d1c */

#define GIC_ICDPPISR_OFFSET        0x0d00    /* PPI Status Register */
#define GIC_ICDSPISR_OFFSET(n)     (0x0d00 + GIC_OFFSET32(n))

/* 0x0d80-0x0dfc: Reserved */

/* Non-secure Access Control Registers, optional: 00xe00-0x0efc */

#define GIC_ICDNSACR_OFFSET(n)     (0x0e00 + GIC_OFFSET16(n))

/* Software Generated Interrupt Register: 0x0f00 */

#define GIC_ICDSGIR_OFFSET         0x0f00    /* Software Generated Interrupt Register */

/* 0x0f04-0x0f0c: Reserved */

/* SGI Clear-Pending Registers: 0x0f10-0x0f1c */

#define GIC_ICDSCPR_OFFSET(n)      (0x0f10 + GIC_OFFSET8(n))

/* SGI Set-Pending Registers: 0x0f20-0x0f2c */

#define GIC_ICDSSPR_OFFSET(n)      (0x0f20 + GIC_OFFSET8(n))

/* 0x0f30-0x0fcc: Reserved */

/* 0x0fd0-0x0ffc: Implementation defined */

/* Peripheral Identification Registers: 0x0fd0-0xfe8 */

#define GIC_ICDPIDR_OFFSET(n)      (0x0fd0 + ((n) << 2))

/* Component Identification Registers: 0x0ff0-0x0ffc */

#define GIC_ICDCIDR_OFFSET(n)      (0x0ff0 + ((n) << 2))

/* GIC Register Addresses ***************************************************/

/* The Interrupt Controller is a single functional unit that is located in a
 * Cortex-A9 MPCore design.  There is one interrupt interface per Cortex-A9
 * processor.  Registers are memory mapped and accessed through a chip-
 * specific private memory spaced (see mpcore.h).
 */

/* CPU Interface registers */

#define GIC_ICCICR                 (MPCORE_ICC_VBASE+GIC_ICCICR_OFFSET)
#define GIC_ICCPMR                 (MPCORE_ICC_VBASE+GIC_ICCPMR_OFFSET)
#define GIC_ICCBPR                 (MPCORE_ICC_VBASE+GIC_ICCBPR_OFFSET)
#define GIC_ICCIAR                 (MPCORE_ICC_VBASE+GIC_ICCIAR_OFFSET)
#define GIC_ICCEOIR                (MPCORE_ICC_VBASE+GIC_ICCEOIR_OFFSET)
#define GIC_ICCRPR                 (MPCORE_ICC_VBASE+GIC_ICCRPR_OFFSET)
#define GIC_ICCHPIR                (MPCORE_ICC_VBASE+GIC_ICCHPIR_OFFSET)
#define GIC_ICCABPR                (MPCORE_ICC_VBASE+GIC_ICCABPR_OFFSET)
#define GIC_ICCAIAR                (MPCORE_ICC_VBASE+GIC_ICCAIAR_OFFSET)
#define GIC_ICCAEOIR               (MPCORE_ICC_VBASE+GIC_ICCAEOIR_OFFSET)
#define GIC_ICCAHPIR               (MPCORE_ICC_VBASE+GIC_ICCAHPIR_OFFSET)
#define GIC_ICCAPR1                (MPCORE_ICC_VBASE+GIC_ICCAPR1_OFFSET)
#define GIC_ICCAPR2                (MPCORE_ICC_VBASE+GIC_ICCAPR2_OFFSET)
#define GIC_ICCAPR3                (MPCORE_ICC_VBASE+GIC_ICCAPR3_OFFSET)
#define GIC_ICCAPR4                (MPCORE_ICC_VBASE+GIC_ICCAPR4_OFFSET)
#define GIC_ICCNSAPR1              (MPCORE_ICC_VBASE+GIC_ICCNSAPR1_OFFSET)
#define GIC_ICCNSAPR2              (MPCORE_ICC_VBASE+GIC_ICCNSAPR2_OFFSET)
#define GIC_ICCNSAPR3              (MPCORE_ICC_VBASE+GIC_ICCNSAPR3_OFFSET)
#define GIC_ICCNSAPR4              (MPCORE_ICC_VBASE+GIC_ICCNSAPR4_OFFSET)
#define GIC_ICCIDR                 (MPCORE_ICC_VBASE+GIC_ICCIDR_OFFSET)
#define GIC_ICCDIR                 (MPCORE_ICC_VBASE+GIC_ICCDIR_OFFSET)

/* Distributor Registers */

#define GIC_ICDDCR                 (MPCORE_ICD_VBASE+GIC_ICDDCR_OFFSET)
#define GIC_ICDICTR                (MPCORE_ICD_VBASE+GIC_ICDICTR_OFFSET)
#define GIC_ICDIIDR                (MPCORE_ICD_VBASE+GIC_ICDIIDR_OFFSET)
#define GIC_ICDISR(n)              (MPCORE_ICD_VBASE+GIC_ICDISR_OFFSET(n))
#define GIC_ICDISER(n)             (MPCORE_ICD_VBASE+GIC_ICDISER_OFFSET(n))
#define GIC_ICDICER(n)             (MPCORE_ICD_VBASE+GIC_ICDICER_OFFSET(n))
#define GIC_ICDISPR(n)             (MPCORE_ICD_VBASE+GIC_ICDISPR_OFFSET(n))
#define GIC_ICDICPR(n)             (MPCORE_ICD_VBASE+GIC_ICDICPR_OFFSET(n))
#define GIC_ICDSAR(n)              (MPCORE_ICD_VBASE+GIC_ICDSAR_OFFSET(n))
#define GIC_ICDCAR(n)              (MPCORE_ICD_VBASE+GIC_ICDCAR_OFFSET(n))
#define GIC_ICDIPR(n)              (MPCORE_ICD_VBASE+GIC_ICDIPR_OFFSET(n))
#define GIC_ICDIPTR(n)             (MPCORE_ICD_VBASE+GIC_ICDIPTR_OFFSET(n))
#define GIC_ICDICFR(n)             (MPCORE_ICD_VBASE+GIC_ICDICFR_OFFSET(n))
#define GIC_ICDPPISR               (MPCORE_ICD_VBASE+GIC_ICDPPISR_OFFSET)
#define GIC_ICDSPISR(n)            (MPCORE_ICD_VBASE+GIC_ICDSPISR_OFFSET(n))
#define GIC_ICDNSACR(n)            (MPCORE_ICD_VBASE+GIC_ICDNSACR_OFFSET(n))
#define GIC_ICDSGIR                (MPCORE_ICD_VBASE+GIC_ICDSGIR_OFFSET)
#define GIC_ICDSCPR(n)             (MPCORE_ICD_VBASE+GIC_ICDSCPR_OFFSET(n))
#define GIC_ICDSSPR(n)             (MPCORE_ICD_VBASE+GIC_ICDSSPR_OFFSET(n))
#define GIC_ICDPIDR(n)             (MPCORE_ICD_VBASE+GIC_ICDPIDR_OFFSET(n))
#define GIC_ICDCIDR(n)             (MPCORE_ICD_VBASE+GIC_ICDCIDR_OFFSET(n))

/* GIC Register Bit Definitions *********************************************/

/* CPU Interface registers */

/* CPU Interface Control Register -- without security extensions */

#define GIC_ICCICR_ENABLE          (1 << 0)  /* Bit 0:  Enable the CPU interface for this GIC */
                                             /* Bits 1-31: Reserved */

/* CPU Interface Control Register -- with security extensions,
 * non-secure copy
 */

#define GIC_ICCICRU_ENABLEGRP1     (1 << 0)  /* Bit 0:  Enable Group 1 interrupts for the CPU */
                                             /* Bits 1-4: Reserved */
#define GIC_ICCICRU_FIQBYPDISGRP1  (1 << 5)  /* Bit 5:  FIQ disabled for CPU Group 1*/
#define GIC_ICCICRU_IRQBYPDISGRP1  (1 << 6)  /* Bit 6:  IRQ disabled for CPU Group 1*/
                                             /* Bits 7-8: Reserved */
#define GIC_ICCICRU_EOIMODENS      (1 << 9)  /* Bit 9:  Control EIOIR access (non-secure) */
                                             /* Bits 10-31: Reserved */

/* CPU Interface Control Register -- with security extensions,
 * secure copy
 */

#define GIC_ICCICRS_ENABLEGRP0     (1 << 0)  /* Bit 0:  Enable Group 0 interrupts for the CPU */
#define GIC_ICCICRS_ENABLEGRP1     (1 << 1)  /* Bit 1:  Enable Group 1 interrupts for the CPU */
#define GIC_ICCICRS_ACKTCTL        (1 << 2)  /* Bit 2:  Group 1 interrupt activation control */
#define GIC_ICCICRS_FIQEN          (1 << 3)  /* Bit 3:  Signal Group 0 via FIQ */
#define GIC_ICCICRS_CBPR           (1 << 4)  /* Bit 4:  Control Group 0/1 Pre-emption */
#define GIC_ICCICRS_FIQBYPDISGRP0  (1 << 5)  /* Bit 5:  FIQ disabled for CPU Group 0 */
#define GIC_ICCICRS_IRQBYPDISGRP0  (1 << 6)  /* Bit 6:  IRQ disabled for CPU Group 0 */
#define GIC_ICCICRS_FIQBYPDISGRP1  (1 << 7)  /* Bit 5:  FIQ disabled for CPU Group 1 */
#define GIC_ICCICRS_IRQBYPDISGRP1  (1 << 8)  /* Bit 6:  IRQ disabled for CPU Group 1 */
#define GIC_ICCICRS_EOIMODES       (1 << 9)  /* Bit 6:  Control EIOIR access (secure) */
#define GIC_ICCICRS_EOIMODENS      (1 << 10) /* Bit 10: Control EIOIR access (non-secure) */
                                             /* Bits 11-31: Reserved */

/* Interrupt Priority Mask Register.  Priority values are 8-bit unsigned
 * binary. A GIC supports a minimum of 16 and a maximum of 256 priority
 * levels.  As a result, PMR settings make sense.
 */

#define GIC_ICCPMR_SHIFT           (0)       /* Bits 0-7: Priority mask */
#define GIC_ICCPMR_MASK            (0xff << GIC_ICCPMR_SHIFT)
#  define GIC_ICCPMR_VALUE(n)      ((uint32_t)(n) << GIC_ICCPMR_SHIFT)
                                             /* Bits 8-31: Reserved */

/* Binary point Register and Aliased Non-secure Binary Point Register.
 * Priority values are 8-bit unsigned binary. A GIC supports a minimum of
 * 16 and a maximum of 256 priority levels.  As a result, not all binary
 * point settings make sense.
 */

#define GIC_ICCBPR_SHIFT           (0)       /* Bits 0-2: Binary point */
#define GIC_ICCBPR_MASK            (7 << GIC_ICCBPR_SHIFT)
#  define GIC_ICCBPR_1_7           (0 << GIC_ICCBPR_SHIFT) /* Priority bits [7:1] compared for pre-emption */
#  define GIC_ICCBPR_2_7           (1 << GIC_ICCBPR_SHIFT) /* Priority bits [7:2] compared for pre-emption */
#  define GIC_ICCBPR_3_7           (2 << GIC_ICCBPR_SHIFT) /* Priority bits [7:3] compared for pre-emption */
#  define GIC_ICCBPR_4_7           (3 << GIC_ICCBPR_SHIFT) /* Priority bits [7:4] compared for pre-emption */
#  define GIC_ICCBPR_5_7           (4 << GIC_ICCBPR_SHIFT) /* Priority bits [7:5] compared for pre-emption */
#  define GIC_ICCBPR_6_7           (5 << GIC_ICCBPR_SHIFT) /* Priority bits [7:6] compared for pre-emption */
#  define GIC_ICCBPR_7_7           (6 << GIC_ICCBPR_SHIFT) /* Priority bit [7] compared for pre-emption */
#  define GIC_ICCBPR_NOPREMPT      (7 << GIC_ICCBPR_SHIFT) /* No pre-emption is performed */

                                             /* Bits 3-31: Reserved */

/* Interrupt Acknowledge Register */

#define GIC_ICCIAR_INTID_SHIFT     (0)       /* Bits 0-9: Interrupt ID */
#define GIC_ICCIAR_INTID_MASK      (0x3ff << GIC_ICCIAR_INTID_SHIFT)
#  define GIC_ICCIAR_INTID(n)      ((uint32_t)(n) << GIC_ICCIAR_INTID_SHIFT)
#define GIC_ICCIAR_CPUSRC_SHIFT    (10)      /* Bits 10-12: CPU source ID */
#define GIC_ICCIAR_CPUSRC_MASK     (7 << GIC_ICCIAR_CPUSRC_SHIFT)
#  define GIC_ICCIAR_CPUSRC(n)     ((uint32_t)(n) << GIC_ICCIAR_CPUSRC_SHIFT)

                                             /* Bits 13-31: Reserved */

/* End of Interrupt Register */

#define GIC_ICCEOIR_SPURIOUSNS     (0x3fe)
#define GIC_ICCEOIR_SPURIOUS       (0x3ff)

#define GIC_ICCEOIR_INTID_SHIFT    (0)       /* Bits 0-9: Interrupt ID */
#define GIC_ICCEOIR_INTID_MASK     (0x3ff << GIC_ICCEOIR_INTID_SHIFT)
#  define GIC_ICCEOIR_INTID(n)     ((uint32_t)(n) << GIC_ICCEOIR_INTID_SHIFT)
#define GIC_ICCEOIR_CPUSRC_SHIFT   (10)      /* Bits 10-12: CPU source ID */
#define GIC_ICCEOIR_CPUSRC_MASK    (7 << GIC_ICCEOIR_CPUSRC_SHIFT)
#  define GIC_ICCEOIR_CPUSRC(n)    ((uint32_t)(n) << GIC_ICCEOIR_CPUSRC_SHIFT)

                                             /* Bits 13-31: Reserved */

/* Running Interrupt Register */

#define GIC_ICCRPR_PRIO_SHIFT      (0)       /* Bits 0-7: Priority mask */
#define GIC_ICCRPR_PRIO_MASK       (0xff << GIC_ICCRPR_PRIO_SHIFT)
#  define GIC_ICCRPR_PRIO_VALUE(n) ((uint32_t)(n) << GIC_ICCRPR_PRIO_SHIFT)

                                             /* Bits 8-31: Reserved */

/* Highest Pending Interrupt Register */

#define GIC_ICCHPIR_INTID_SHIFT    (0)       /* Bits 0-9: Interrupt ID */
#define GIC_ICCHPIR_INTID_MASK     (0x3ff << GIC_ICCHPIR_INTID_SHIFT)
#  define GIC_ICCHPIR_INTID(n)     ((uint32_t)(n) << GIC_ICCHPIR_INTID_SHIFT)
#define GIC_ICCHPIR_CPUSRC_SHIFT   (10)      /* Bits 10-12: CPU source ID */
#define GIC_ICCHPIR_CPUSRC_MASK    (7 << GIC_ICCHPIR_CPUSRC_SHIFT)
#  define GIC_ICCHPIR_CPUSRC(n)    ((uint32_t)(n) << GIC_ICCHPIR_CPUSRC_SHIFT)

                                             /* Bits 13-31: Reserved */

/* CPU Interface Implementer ID Register */

#define GIC_ICCIDR_IMPL_SHIFT      (0)       /* Bits 0-11:  Implementer */
#define GIC_ICCIDR_IMPL_MASK       (0xfff << GIC_ICCIDR_IMPL_SHIFT)
#define GIC_ICCIDR_REVISION_SHIFT  (12)      /* Bits 12-15: Revision number */
#define GIC_ICCIDR_REVISION_MASK   (15 << GIC_ICCIDR_REVISION_SHIFT)
#define GIC_ICCIDR_ARCHNO_SHIFT    (16)      /* Bits 16-19: Architecture number */
#define GIC_ICCIDR_ARCHNO_MASK     (15 << GIC_ICCIDR_ARCHNO_SHIFT)
#define GIC_ICCIDR_PARTNO_SHIFT    (20)      /* Bits 20-31: Part number */
#define GIC_ICCIDR_PARTNO_MASK     (0xfff << GIC_ICCIDR_PARTNO_SHIFT)

/* Deactivate Interrupt Register */

#define GIC_ICCDIR_INTID_SHIFT     (0)       /* Bits 0-9: Interrupt ID */
#define GIC_ICCDIR_INTID_MASK      (0x3ff << GIC_ICCHPIR_INTID_SHIFT)
#  define GIC_ICCDIR_INTID(n)      ((uint32_t)(n) << GIC_ICCHPIR_INTID_SHIFT)
#define GIC_ICCDIR_CPUSRC_SHIFT    (10)      /* Bits 10-12: CPU source ID */
#define GIC_ICCDIR_CPUSRC_MASK     (7 << GIC_ICCHPIR_CPUSRC_SHIFT)
#  define GIC_ICCDIR_CPUSRC(n)     ((uint32_t)(n) << GIC_ICCHPIR_CPUSRC_SHIFT)

/* Distributor Registers */

/* Distributor Control Register -- without security extensions */

#define GIC_ICDDCR_ENABLE          (1 << 0)  /* Bit 0: Enable forwarding of interrupts */
                                             /* Bits 1-31: Reserved */

/* Distributor Control Register -- with security extensions */

#define GIC_ICDDCR_ENABLEGRP0      (1 << 0)  /* Bit 0: Enable forwarding of Group 0 interrupts */
#define GIC_ICDDCR_ENABLEGRP1      (1 << 1)  /* Bit 1: Enable forwarding of Group 1 interrupts */
                                             /* Bits 2-31: Reserved */

/* Interrupt Controller Type Register */

#define GIC_ICDICTR_ITLINES_SHIFT  (0)       /* Bits 0-4: It lines number */
#define GIC_ICDICTR_ITLINES_MASK   (0x1f << GIC_ICDICTR_ITLINES_SHIFT)
#define GIC_ICDICTR_CPUNO_SHIFT    (5)       /* Bits 5-7: CPU number */
#define GIC_ICDICTR_CPUNO_MASK     (7 << GIC_ICDICTR_CPUNO_SHIFT)
                                             /* Bits 8-9: Reserved */
#define GIC_ICDICTR_SECEXTNS       (1 << 10) /* Bit 10: Number of security domains */
#define GIC_ICDICTR_LSPI_SHIFT     (11)      /* Bits 11-15: Number of Lockable Shared Peripheral Interrupts */
#define GIC_ICDICTR_LSPI_MASK      (0x1f << GIC_ICDICTR_LSPI_SHIFT)
                                             /* Bits 16-31: Reserved */

/* Distributor Implementer ID Register */

#define GIC_ICDIIDR_IMPL_SHIFT      (0)      /* Bits 0-11: Implementer */
#define GIC_ICDIIDR_IMPL_MASK       (0xfff << GIC_ICDIIDR_IMPL_SHIFT)
#define GIC_ICDIIDR_REVISION_SHIFT  (12)     /* Bits 12-15: Revision number */
#define GIC_ICDIIDR_REVISION_MASK   (0xf << GIC_ICDIIDR_REVISION_SHIFT)
#define GIC_ICDIIDR_VARIANT_SHIFT   (16)     /* Bits 16-19 Variant number */
#define GIC_ICDIIDR_VARIANT_MASK    (0xf << GIC_ICDIIDR_VARIANT_SHIFT)
                                             /* Bits 20-23: Reserved */
#define GIC_ICDIIDR_PRODUCTID_SHIFT (24)     /* Bits 24-31: Product id */
#define GIC_ICDIIDR_PRODUCTID_MASK  (0xff << GIC_ICDIIDR_PRODUCTID_SHIFT)

/* Interrupt Security Registers: 0x0080-0x009c */

#define GIC_ICDISR_INT(n)           GIC_MASK32(n)

/* Interrupt Set-Enable.
 *
 * NOTE:
 * In the Cortex-A9 MPCore, SGIs are always enabled. The corresponding bits
 * in the ICDISERn are read as one, write ignored
 */

#define GIC_ICDISER_INT(n)         GIC_MASK32(n)

/* Interrupt Clear-Enable.
 *
 * NOTE:
 * In the Cortex-A9 MPCore, SGIs are always enabled. The corresponding bits
 * in the ICDICERn are read as one, write ignored
 */

#define GIC_ICDICER_INT(n)         GIC_MASK32(n)

/* Interrupt Set-Pending */

#define GIC_ICDISPR_INT(n)         GIC_MASK32(n)

/* Interrupt Clear-Pending */

#define GIC_ICDICPR_INT(n)         GIC_MASK32(n)

/* GICv2 Interrupt Set-Active Registers */

#define GIC_ICDSAR_INT(n)          GIC_MASK32(n)

/* Interrupt Clear-Active Registers */

#define GIC_ICDCAR_INT(n)          GIC_MASK32(n)

/* Interrupt Priority Registers */

#define GIC_ICDIPR_ID_SHIFT(n)     GIC_SHIFT4(n)
#define GIC_ICDIPR_ID_MASK(n)      GIC_MASK4(n)
#  define GIC_ICDIPR_ID(n,p)       ((uint32_t)(p) << GIC_SHIFT4(n))

/* Interrupt Processor Target Registers */

#define CPU0_TARGET                (1 << 0)
#define CPU1_TARGET                (1 << 1)
#define CPU2_TARGET                (1 << 2)
#define CPU3_TARGET                (1 << 3)

#define GIC_ICDIPTR_ID_SHIFT(n)    GIC_SHIFT4(n)
#define GIC_ICDIPTR_ID_MASK(n)     GIC_MASK4(n)
#  define GIC_ICDIPTR_ID(n,t)      ((uint32_t)(t) <<GIC_SHIFT4(n))

/* Interrupt Configuration Register */

#define INT_ICDICFR_NN             0         /* Bit n: 0= N-N Model */
#define INT_ICDICFR_1N             1         /* Bit n: 1= 1-N Model */
#define INT_ICDICFR_LEVEL          0         /* Bit n+1: 0=Level sensitive */
#define INT_ICDICFR_EDGE           2         /* Bit n+2: 1=Edge sensitive */

#define GIC_ICDICFR_ID_SHIFT(n)    GIC_SHIFT16(n)
#define GIC_ICDICFR_ID_MASK(n)     GIC_MASK16(n)
#  define GIC_ICDICFR_ID(n,c)      ((uint32_t)(c) << GIC_SHIFT16(n))

/* PPI Status Register */

#define GIC_ICDPPISR_PPI(n)        (1 << ((n) + 11)) /* Bits 11-15:  PPI(n) status, n=0-4 */

#  define GIC_ICDPPISR_GTM         (1 << 11) /* Bit 11:  PPI[0], Global Timer */
#  define GIC_ICDPPISR_NFIQ        (1 << 12) /* Bit 12:  PPI[1], FIQ, active low */
#  define GIC_ICDPPISR_PTM         (1 << 13) /* Bit 13:  PPI[2], Private Timer */
#  define GIC_ICDPPISR_PWDT        (1 << 14) /* Bit 14:  PPI[3], Private Watchdog */
#  define GIC_ICDPPISR_NIRQ        (1 << 15) /* Bit 15:  PPI[3], IRQ, active low */

/* SPI Status Registers */

#define GIC_ICDSPISR_INT(n)        GIC_MASK32(n)

/* Non-secure Access Control Registers, optional */

#define GIC_ICDNSACR_NONE          0
#define GIC_ICDNSACR_SET           1
#define GIC_ICDNSACR_CLEAR         2
#define GIC_ICDNSACR_ROUTE         3

#define GIC_ICDNSACR_ID_SHIFT(n)   GIC_SHIFT16(n)
#define GIC_ICDNSACR_ID_MASK(n)    GIC_MASK16(n)
#  define GIC_ICDNSACR_ID(n,p)     ((uint32_t)(p) << GIC_SHIFT16(n))

/* Software Generated Interrupt Register */

#define GIC_ICDSGIR_INTID_SHIFT       (0)    /* Bits 0-9: Interrupt ID */
#define GIC_ICDSGIR_INTID_MASK        (0x3ff << GIC_ICDSGIR_INTID_SHIFT)
#  define GIC_ICDSGIR_INTID(n)        ((uint32_t)(n) << GIC_ICDSGIR_INTID_SHIFT)
                                             /* Bits 10-14: Reserved */
#define GIC_ICDSGIR_NSATT             (1 << 15)
#define GIC_ICDSGIR_CPUTARGET_SHIFT   (16)   /* Bits 16-23: CPU target */
#define GIC_ICDSGIR_CPUTARGET_MASK    (0xff << GIC_ICDSGIR_CPUTARGET_SHIFT)
#  define GIC_ICDSGIR_CPUTARGET(n)    ((uint32_t)(n) << GIC_ICDSGIR_CPUTARGET_SHIFT)
#define GIC_ICDSGIR_TGTFILTER_SHIFT   (24)   /* Bits 24-25: Target filter */
#define GIC_ICDSGIR_TGTFILTER_MASK    (3 << GIC_ICDSGIR_TGTFILTER_SHIFT)
#  define GIC_ICDSGIR_TGTFILTER_LIST  (0 << GIC_ICDSGIR_TGTFILTER_SHIFT) /* Interrupt sent to CPUs CPU target list */
#  define GIC_ICDSGIR_TGTFILTER_OTHER (1 << GIC_ICDSGIR_TGTFILTER_SHIFT) /* Interrupt is sent to all but requesting CPU */
#  define GIC_ICDSGIR_TGTFILTER_THIS  (2 << GIC_ICDSGIR_TGTFILTER_SHIFT) /* Interrupt is sent to requesting CPU only */
                                                                         /* Bits 26-31: Reserved */

/* Interrupt IDs ************************************************************/

/* The Global Interrupt Controller (GIC) collects up to 224 interrupt
 * requests and provides a memory mapped interface to each of the CPU core.
 *
 * The first 32 interrupts are used for interrupts that are private to the
 * CPUs interface. Other shared interrupts besides the also hooked up to
 * the GIC in the same order.  The shared interrupt sources are MCU-
 * specific and documented in MCU-specific header files.
 *
 * Each interrupt can be configured as a normal or a secure interrupt.
 * Software force registers and software priority masking are also
 * supported. The following table describes the private RM interrupt
 * sources.
 */

/* Private Peripheral Interrupts (PPI) **************************************/

/* Each Cortex-A9 processor has private interrupts, ID0-ID15, that can only
 * be triggered by software. These interrupts are aliased so that there is
 * no requirement for a requesting Cortex-A9 processor to determine its own
 * CPU ID when it deals with SGIs.  The priority of an SGI depends on the
 * value set by the receiving Cortex-A9 processor in the banked SGI priority
 * registers, not the priority set by the sending Cortex-A9 processor.
 *
 * NOTE: If CONFIG_SMP is enabled then SGI1 and SGI2 are used for inter-CPU
 * task management.
 */

#define GIC_IRQ_SGI0              0 /* Software Generated Interrupt (SGI) 0 */
#define GIC_IRQ_SGI1              1 /* Software Generated Interrupt (SGI) 1 */
#define GIC_IRQ_SGI2              2 /* Software Generated Interrupt (SGI) 2 */
#define GIC_IRQ_SGI3              3 /* Software Generated Interrupt (SGI) 3 */
#define GIC_IRQ_SGI4              4 /* Software Generated Interrupt (SGI) 4 */
#define GIC_IRQ_SGI5              5 /* Software Generated Interrupt (SGI) 5 */
#define GIC_IRQ_SGI6              6 /* Software Generated Interrupt (SGI) 6 */
#define GIC_IRQ_SGI7              7 /* Software Generated Interrupt (SGI) 7 */
#define GIC_IRQ_SGI8              8 /* Software Generated Interrupt (SGI) 8 */
#define GIC_IRQ_SGI9              9 /* Software Generated Interrupt (SGI) 9 */
#define GIC_IRQ_SGI10            10 /* Software Generated Interrupt (SGI) 10 */
#define GIC_IRQ_SGI11            11 /* Software Generated Interrupt (SGI) 11 */
#define GIC_IRQ_SGI12            12 /* Software Generated Interrupt (SGI) 12 */
#define GIC_IRQ_SGI13            13 /* Software Generated Interrupt (SGI) 13 */
#define GIC_IRQ_SGI14            14 /* Software Generated Interrupt (SGI) 14 */
#define GIC_IRQ_SGI15            15 /* Software Generated Interrupt (SGI) 15 */

#define GIC_IRQ_VM               25 /* Virtual Maintenance Interrupt (VM) PPI(6) */
#define GIC_IRQ_HTM              26 /* Hypervisor Timer (HTM) PPI(5) */
#define GIC_IRQ_VTM              27 /* Virtual Timer (VTM) PPI(4) */
#define GIC_IRQ_FIQ              28 /* Fast Interrupt Request (nFIQ) PPI(0) */
#define GIC_IRQ_STM              29 /* Secure Physical Timer (STM) PPI(1) */
#define GIC_IRQ_PTM              30 /* Non-secure Physical Timer (PTM) PPI(2) */
#define GIC_IRQ_IRQ              31 /* Interrupt Request (nIRQ) PPI(3) */

/* Shared Peripheral Interrupts (SPI) follow */

#define GIC_IRQ_SPI              32 /* First SPI interrupt ID */

#ifdef CONFIG_ARCH_TRUSTZONE_SECURE
#  define GIC_SMP_CPUSTART       GIC_IRQ_SGI9
#  define GIC_SMP_CPUPAUSE       GIC_IRQ_SGI10
#  define GIC_SMP_CPUCALL        GIC_IRQ_SGI11
#  define GIC_SMP_CPUPAUSE_ASYNC GIC_IRQ_SGI12
#else
#  define GIC_SMP_CPUSTART       GIC_IRQ_SGI1
#  define GIC_SMP_CPUPAUSE       GIC_IRQ_SGI2
#  define GIC_SMP_CPUCALL        GIC_IRQ_SGI3
#  define GIC_SMP_CPUPAUSE_ASYNC GIC_IRQ_SGI4
#endif

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Name: arm_gic_nlines
 *
 * Description:
 *   Return the number of interrupt lines supported by this GIC
 *   implementation (include both PPIs (32) and SPIs).
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The number of interrupt lines.
 *
 ****************************************************************************/

static inline unsigned int arm_gic_nlines(void)
{
  uint32_t regval;
  uint32_t field;

  /* Get the number of interrupt lines. */

  regval = getreg32(GIC_ICDICTR);
  field  = (regval & GIC_ICDICTR_ITLINES_MASK) >> GIC_ICDICTR_ITLINES_SHIFT;
  return (field + 1) << 5;
}

/****************************************************************************
 * Name: arm_cpu_sgi
 *
 * Description:
 *   Perform a Software Generated Interrupt (SGI).  If CONFIG_SMP is
 *   selected, then the SGI is sent to all CPUs specified in the CPU set.
 *   That set may include the current CPU.
 *
 *   If CONFIG_SMP is not selected, the cpuset is ignored and SGI is sent
 *   only to the current CPU.
 *
 * Input Parameters:
 *   sgi    - The SGI interrupt ID (0-15)
 *   cpuset - The set of CPUs to receive the SGI
 *
 * Returned Value:
 *   OK is always returned at present.
 *
 ****************************************************************************/

static inline void arm_cpu_sgi(int sgi, unsigned int cpuset)
{
  uint32_t regval;

#ifdef CONFIG_SMP
  regval = GIC_ICDSGIR_INTID(sgi) | GIC_ICDSGIR_CPUTARGET(cpuset) |
           GIC_ICDSGIR_TGTFILTER_LIST;
#else
  regval = GIC_ICDSGIR_INTID(sgi) | GIC_ICDSGIR_CPUTARGET(0) |
           GIC_ICDSGIR_TGTFILTER_THIS;
#endif

#if defined(CONFIG_ARCH_TRUSTZONE_SECURE)
  if (sgi >= GIC_IRQ_SGI0 && sgi <= GIC_IRQ_SGI7)
#endif
    {
      /* Set NSATT be 1: forward the SGI specified in the SGIINTID field to a
       * specified CPU interfaces only if the SGI is configured as Group 1 on
       * that interface.
       * For non-secure context, the configuration of GIC_ICDSGIR_NSATT_GRP1
       * is not mandatory in the GICv2 specification, but for SMP scenarios,
       * this value needs to be configured, otherwise issues may occur in the
       * SMP scenario.
       */

      regval |= GIC_ICDSGIR_NSATT_GRP1;
    }

  putreg32(regval, GIC_ICDSGIR);
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#  define EXTERN extern "C"
extern "C"
{
#else
#  define EXTERN extern
#endif

/****************************************************************************
 * Name: arm_gic0_initialize
 *
 * Description:
 *   Perform common, one-time GIC initialization on CPU0 only.  Both
 *   arm_gic0_initialize() must be called on CPU0; arm_gic_initialize() must
 *   be called for all CPUs.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void arm_gic0_initialize(void);

/****************************************************************************
 * Name: arm_gic_initialize
 *
 * Description:
 *   Perform common GIC initialization for the current CPU (all CPUs)
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void arm_gic_initialize(void);

/****************************************************************************
 * Name: arm_gic_irq_trigger
 *
 * Description:
 *   Set the trigger type for the specificd IRQ source and the current CPU.
 *
 *   Since this API is not supported on all architectures, it should be
 *   avoided in common implementations where possible.
 *
 * Input Parameters:
 *   irq - The interrupt request to modify.
 *   edge - False: Active HIGH level sensitive, True: Rising edge sensitive
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

int arm_gic_irq_trigger(int irq, bool edge);

/****************************************************************************
 * Name: arm_decodeirq
 *
 * Description:
 *   This function is called from the IRQ vector handler in arm_vectors.S.
 *   At this point, the interrupt has been taken and the registers have
 *   been saved on the stack.  This function simply needs to determine the
 *   the irq number of the interrupt and then to call arm_doirq to dispatch
 *   the interrupt.
 *
 *  Input Parameters:
 *   regs - A pointer to the register save area on the stack.
 *
 ****************************************************************************/

uint32_t *arm_decodeirq(uint32_t *regs);

/****************************************************************************
 * Name: arm_start_handler
 *
 * Description:
 *   This is the handler for SGI1.  This handler simply returns from the
 *   interrupt, restoring the state of the new task at the head of the ready
 *   to run list.
 *
 * Input Parameters:
 *   Standard interrupt handling
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SMP
int arm_start_handler(int irq, void *context, void *arg);
#endif

/****************************************************************************
 * Name: arm_pause_handler
 *
 * Description:
 *   This is the handler for SGI2.  It performs the following operations:
 *
 *   1. It saves the current task state at the head of the current assigned
 *      task list.
 *   2. It waits on a spinlock, then
 *   3. Returns from interrupt, restoring the state of the new task at the
 *      head of the ready to run list.
 *
 * Input Parameters:
 *   Standard interrupt handling
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SMP
int arm_pause_handler(int irq, void *context, void *arg);
#endif

/****************************************************************************
 * Name: arm_pause_async_handler
 *
 * Description:
 *   This is the handler for async pause.
 *
 *   1. It saves the current task state at the head of the current assigned
 *      task list.
 *   2. It porcess g_delivertasks
 *   3. Returns from interrupt, restoring the state of the new task at the
 *      head of the ready to run list.
 *
 * Input Parameters:
 *   Standard interrupt handling
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SMP
int arm_pause_async_handler(int irq, void *context, void *arg);
#endif

/****************************************************************************
 * Name: arm_gic_dump
 *
 * Description:
 *   Dump GIC registers to the SYSLOG device
 *
 * Input Parameters:
 *   msg - Message to print with the register data
 *   all - True: Dump all IRQs; False: Dump only registers for this IRQ
 *   irq - if all == false, then dump only this IRQ.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_IRQ_INFO
void arm_gic_dump(const char *msg, bool all, int irq);
#else
#  define arm_gic_dump(m,a,i)
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* CONFIG_ARMV7R_HAVE_GICv2 */
#endif /* __ARCH_ARM_SRC_ARMV7_R_GIC_H */
