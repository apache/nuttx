/****************************************************************************
 * arch/arm64/src/common/arm64_gicv2.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/irq.h>
#include <nuttx/pci/pci.h>

#include "arm64_internal.h"
#include "arm64_gic.h"

#if CONFIG_ARM64_GIC_VERSION == 2

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GIC Distributor / Redistributor Register Interface Base Addresses */

#define MPCORE_ICD_VBASE CONFIG_GICD_BASE
#define MPCORE_ICC_VBASE CONFIG_GICR_BASE

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
#define GIC_ICDSGIR_NSATT_SHIFT       (15)
#define GIC_ICDSGIR_NSATT_MASK        (1 << GIC_ICDSGIR_NSATT_SHIFT)
#  define GIC_ICDSGIR_NSATT_GRP0      (0 << GIC_ICDSGIR_NSATT_SHIFT)
#  define GIC_ICDSGIR_NSATT_GRP1      (1 << GIC_ICDSGIR_NSATT_SHIFT)

#define GIC_ICDSGIR_CPUTARGET_SHIFT   (16)   /* Bits 16-23: CPU target */
#define GIC_ICDSGIR_CPUTARGET_MASK    (0xff << GIC_ICDSGIR_CPUTARGET_SHIFT)
#  define GIC_ICDSGIR_CPUTARGET(n)    ((uint32_t)(n) << GIC_ICDSGIR_CPUTARGET_SHIFT)
#define GIC_ICDSGIR_TGTFILTER_SHIFT   (24)   /* Bits 24-25: Target filter */
#define GIC_ICDSGIR_TGTFILTER_MASK    (3 << GIC_ICDSGIR_TGTFILTER_SHIFT)
#  define GIC_ICDSGIR_TGTFILTER_LIST  (0 << GIC_ICDSGIR_TGTFILTER_SHIFT) /* Interrupt sent to CPUs CPU target list */
#  define GIC_ICDSGIR_TGTFILTER_OTHER (1 << GIC_ICDSGIR_TGTFILTER_SHIFT) /* Interrupt is sent to all but requesting CPU */
#  define GIC_ICDSGIR_TGTFILTER_THIS  (2 << GIC_ICDSGIR_TGTFILTER_SHIFT) /* Interrupt is sent to requesting CPU only */
                                                                         /* Bits 26-31: Reserved */

/* Peripheral ID2 Register */

#define GIC_ICPIDR2                   (6)

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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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

#if defined(CONFIG_ARCH_TRUSTZONE_SECURE)
/****************************************************************************
 * Name: up_set_secure_irq
 *
 * Description:
 *   Secure an IRQ
 *
 ****************************************************************************/

void up_secure_irq(int irq, bool secure)
{
  unsigned int val;

  if (secure)
    {
      val = getreg32(GIC_ICDISR(irq)) & (~GIC_ICDISR_INT(irq));  /* group 0 */
    }
  else
    {
      val = getreg32(GIC_ICDISR(irq)) | GIC_ICDISR_INT(irq);     /* group 1 */
    }

  putreg32(val, GIC_ICDISR(irq));
}

/****************************************************************************
 * Name: up_secure_irq_all
 *
 * Description:
 *   Secure all IRQ
 *
 ****************************************************************************/

void up_secure_irq_all(bool secure)
{
  unsigned int nlines = arm_gic_nlines();
  unsigned int irq;

  for (irq = 0; irq < nlines; irq += 32)
    {
      if (secure)
        {
          putreg32(0x00000000, GIC_ICDISR(irq));   /* group 0 */
        }
      else
        {
          putreg32(0xffffffff, GIC_ICDISR(irq));   /* group 1 */
        }
    }
}
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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
 * Name: gic_validate_dist_version
 *
 * Description:
 *   Verify that GIC Version is 2.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) if GIC Version is 2; -ENODEV if GIC Version isn't 2.
 *
 ****************************************************************************/

static int gic_validate_dist_version(void)
{
  uint32_t  reg;

  /* Read the CPU Interface Implementer ID Register */

  reg = getreg32(GIC_ICCIDR) & GIC_ICCIDR_ARCHNO_MASK;

  /* GIC Version should be 2 */

  if (reg == (0x2 << GIC_ICCIDR_ARCHNO_SHIFT))
    {
      sinfo("GICv2 detected\n");
    }
  else
    {
      sinfo("GICv2 not detected\n");
      return -ENODEV;
    }

  return 0;
}

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

static void arm_gic0_initialize(void)
{
  unsigned int nlines = arm_gic_nlines();
  unsigned int irq;

  /* Initialize SPIs.  The following should be done only by CPU0. */

  /* A processor in Secure State sets:
   *
   * 1. Which interrupts are non-secure (ICDISR).  All set to zero (group
   *    0).
   * 2. Trigger mode of the SPI (ICDICFR). All fields set to 0b01->Level
   *    sensitive, 1-N model.
   * 3. Interrupt Clear-Enable (ICDICER)
   * 3. Priority of the SPI using the priority set register (ICDIPR).
   *    Priority values are 8-bit unsigned binary. A GIC supports a
   *    minimum of 16 and a maximum of 256 priority levels. Here all
   *    are set to the middle priority 128 (0x80).
   * 4. Target that receives the SPI interrupt (ICDIPTR).  Set all to
   *    CPU0.
   */

  /* Registers with 1-bit per interrupt */

  for (irq = GIC_IRQ_SPI; irq < nlines; irq += 32)
    {
      putreg32(0x00000000, GIC_ICDISR(irq));   /* SPIs group 0 */
      putreg32(0xffffffff, GIC_ICDICER(irq));  /* SPIs disabled */
    }

  /* Registers with 2-bits per interrupt */

  for (irq = GIC_IRQ_SPI; irq < nlines; irq += 16)
    {
      putreg32(0x55555555, GIC_ICDICFR(irq));  /* SPIs level sensitive */
    }

  /* Registers with 8-bits per interrupt */

  for (irq = GIC_IRQ_SPI; irq < nlines; irq += 4)
    {
      putreg32(0x80808080, GIC_ICDIPR(irq));   /* SPI priority */
      putreg32(0x01010101, GIC_ICDIPTR(irq));  /* SPI on CPU0 */
    }

#ifdef CONFIG_ARM64_GICV2M
  arm64_gic_v2m_initialize();
#endif

#ifdef CONFIG_SMP
  /* Attach SGI interrupt handlers. This attaches the handler to all CPUs. */

  DEBUGVERIFY(irq_attach(GIC_SMP_CPUPAUSE, arm64_pause_handler, NULL));
  DEBUGVERIFY(irq_attach(GIC_SMP_CPUPAUSE_ASYNC,
                         arm64_pause_async_handler, NULL));
  DEBUGVERIFY(irq_attach(GIC_SMP_CPUCALL,
                         nxsched_smp_call_handler, NULL));
#endif
}

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

static void arm_gic_initialize(void)
{
  uint32_t iccicr;
  uint32_t icddcr;

  /* Initialize PPIs.  The following steps need to be done by all CPUs */

  /* Initialize SGIs and PPIs.  NOTE: A processor in non-secure state cannot
   * program its interrupt security registers and must get a secure processor
   * to program the registers.
   */

  /* Registers with 1-bit per interrupt */

#ifdef CONFIG_ARCH_TRUSTZONE_SECURE
  /* per-CPU inerrupts config:
   * ID0-ID7(SGI)  for Non-secure interrupts
   * ID8-ID15(SGI)  for Secure interrupts.
   * All PPI config as secure interrupts.
   */

  putreg32(0x000000ff, GIC_ICDISR(0));
#else
  putreg32(0x00000000, GIC_ICDISR(0));      /* SGIs and PPIs secure */
#endif
  putreg32(0xfe000000, GIC_ICDICER(0));     /* PPIs disabled */

  /* Registers with 8-bits per interrupt */

  putreg32(0x80808080, GIC_ICDIPR(0));      /* SGI[3:0] priority */
  putreg32(0x80808080, GIC_ICDIPR(4));      /* SGI[4:7] priority */
  putreg32(0x80808080, GIC_ICDIPR(8));      /* SGI[8:11] priority */
  putreg32(0x80808080, GIC_ICDIPR(12));     /* SGI[12:15] priority */
  putreg32(0x80808000, GIC_ICDIPR(24));     /* PPI[0] priority */
  putreg32(0x80808080, GIC_ICDIPR(28));     /* PPI[1:4] priority */

  /* Set the binary point register.
   *
   * Priority values are 8-bit unsigned binary.  The binary point is a 3-bit
   * field; the value n (n=0-6) specifies that bits (n+1) through bit 7 are
   * used in the comparison for interrupt pre-emption.  A GIC supports a
   * minimum of 16 and a maximum of 256 priority levels so not all binary
   * point settings may be meaningul. The special value n=7
   * (GIC_ICCBPR_NOPREMPT) disables pre-emption.  We disable all pre-emption
   * here to prevent nesting of interrupt handling.
   */

  putreg32(GIC_ICCBPR_NOPREMPT, GIC_ICCBPR);

  /* Program the idle priority in the PMR */

  putreg32(GIC_ICCPMR_MASK, GIC_ICCPMR);

  /* Configure the  CPU Interface Control Register */

  iccicr  = getreg32(GIC_ICCICR);

#if defined(CONFIG_ARCH_TRUSTZONE_SECURE)
  /* Clear secure state ICCICR bits to be configured below */

  iccicr &= ~(GIC_ICCICRS_FIQEN | GIC_ICCICRS_ACKTCTL | GIC_ICCICRS_CBPR |
              GIC_ICCICRS_EOIMODES | GIC_ICCICRS_EOIMODENS |
              GIC_ICCICRS_ENABLEGRP0 | GIC_ICCICRS_ENABLEGRP1 |
              GIC_ICCICRS_FIQBYPDISGRP0 | GIC_ICCICRS_IRQBYPDISGRP0 |
              GIC_ICCICRS_FIQBYPDISGRP1 | GIC_ICCICRS_IRQBYPDISGRP1);

#else
  /* Clear non-secure state ICCICR bits to be configured below */

  iccicr &= ~(GIC_ICCICRU_EOIMODENS | GIC_ICCICRU_ENABLEGRP1 |
              GIC_ICCICRU_FIQBYPDISGRP1 | GIC_ICCICRU_IRQBYPDISGRP1);

#endif

#if defined(CONFIG_ARCH_TRUSTZONE_SECURE)
  /* Set FIQn=1 if secure interrupts are to signal using nfiq_c.
   *
   * NOTE:  Only for processors that operate in secure state.
   * REVISIT: Do I need to do this?
   */

  iccicr |= GIC_ICCICRS_FIQEN;

  /* Program the AckCtl bit to select the required interrupt acknowledge
   * behavior.
   *
   * NOTE: Only for processors that operate in both secure and non-secure
   * state.
   */

  iccicr |= GIC_ICCICRS_ACKTCTL;
#endif

#ifdef CONFIG_ARM_GIC_EOIMODE
#  if defined(CONFIG_ARCH_TRUSTZONE_SECURE)
  /* Set EnableS=1 to enable CPU interface to signal secure interrupts.
   *
   * NOTE:  Only for processors that operate in secure state.
   */

  iccicr |= GIC_ICCICRS_EOIMODES;
#  else
  /* Set EnableNS=1 to enable the CPU to signal non-secure interrupts.
   *
   * NOTE:  Only for processors that operate in non-secure state.
   */

  iccicr |= GIC_ICCICRU_EOIMODENS;
#  endif
#endif

#if defined(CONFIG_ARCH_TRUSTZONE_SECURE)
  /* Enable the Group 0 interrupts, FIQEn and disable Group 0/1
   * bypass.
   */

  iccicr |= (GIC_ICCICRS_ENABLEGRP0 | GIC_ICCICRS_ENABLEGRP1 |
             GIC_ICCICRS_FIQBYPDISGRP0 | GIC_ICCICRS_IRQBYPDISGRP0 |
             GIC_ICCICRS_FIQBYPDISGRP1 | GIC_ICCICRS_IRQBYPDISGRP1);
  icddcr  = (GIC_ICDDCR_ENABLEGRP0 | GIC_ICDDCR_ENABLEGRP1);

#else
  /* Enable the Group 1 interrupts and disable Group 1 bypass. */

  iccicr |= (GIC_ICCICRU_ENABLEGRP1 | GIC_ICCICRU_FIQBYPDISGRP1 |
             GIC_ICCICRU_IRQBYPDISGRP1);
  icddcr  = GIC_ICDDCR_ENABLE;

#endif

  /* Write the final ICCICR value to enable the GIC. */

  putreg32(iccicr, GIC_ICCICR);

  /* Write the ICDDCR value to enable the forwarding of interrupt by the
   * distributor.
   */

  putreg32(icddcr, GIC_ICDDCR);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm64_decodeirq
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

uint64_t *arm64_decodeirq(uint64_t * regs)
{
  uint32_t regval;
  int irq;

  /* Read the interrupt acknowledge register and get the interrupt ID */

  regval = getreg32(GIC_ICCIAR);
  irq    = (regval & GIC_ICCIAR_INTID_MASK) >> GIC_ICCIAR_INTID_SHIFT;

#ifdef CONFIG_ARM_GIC_EOIMODE
  putreg32(regval, GIC_ICCEOIR);
#endif

  /* Ignore spurions IRQs.  ICCIAR will report 1023 if there is no pending
   * interrupt.
   */

  DEBUGASSERT(irq < NR_IRQS || irq >= 1022);
  if (irq < NR_IRQS)
    {
      /* Dispatch the interrupt */

      regs = arm64_doirq(irq, regs);
    }

  /* Write to the end-of-interrupt register */

#ifdef CONFIG_ARM_GIC_EOIMODE
  putreg32(regval, GIC_ICCDIR);
#else
  putreg32(regval, GIC_ICCEOIR);
#endif
  return regs;
}

/****************************************************************************
 * Name: arm64_decodefiq
 *
 * Description:
 *   This function is called from the FIQ vector handler in arm_vectors.S.
 *   At this point, the interrupt has been taken and the registers have
 *   been saved on the stack.  This function simply needs to determine the
 *   the irq number of the interrupt and then to call arm_doirq to dispatch
 *   the interrupt.
 *
 *  Input Parameters:
 *   regs - A pointer to the register save area on the stack.
 *
 ****************************************************************************/

uint64_t *arm64_decodefiq(uint64_t *regs)
{
  uint32_t regval;
  int irq;

  /* Read the interrupt acknowledge register and get the interrupt ID */

  regval = getreg32(GIC_ICCIAR);
  irq    = (regval & GIC_ICCIAR_INTID_MASK) >> GIC_ICCIAR_INTID_SHIFT;

#ifdef CONFIG_ARM_GIC_EOIMODE
  putreg32(regval, GIC_ICCEOIR);
#endif

  /* Ignore spurions IRQs.  ICCIAR will report 1023 if there is no pending
   * interrupt.
   */

  DEBUGASSERT(irq < NR_IRQS || irq >= 1022);

  if (irq < NR_IRQS)
    {
      /* Dispatch the interrupt */

      regs = arm64_doirq(irq, regs);
    }

  /* Write to the end-of-interrupt register */

#ifdef CONFIG_ARM_GIC_EOIMODE
  putreg32(regval, GIC_ICCDIR);
#else
  putreg32(regval, GIC_ICCEOIR);
#endif
  return regs;
}

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   On many architectures, there are three levels of interrupt enabling: (1)
 *   at the global level, (2) at the level of the interrupt controller,
 *   and (3) at the device level.  In order to receive interrupts, they
 *   must be enabled at all three levels.
 *
 *   This function implements enabling of the device specified by 'irq'
 *   at the interrupt controller level if supported by the architecture
 *   (up_irq_restore() supports the global level, the device level is
 *   hardware specific).
 *
 *   Since this API is not supported on all architectures, it should be
 *   avoided in common implementations where possible.
 *
 ****************************************************************************/

void up_enable_irq(int irq)
{
  /* Ignore invalid interrupt IDs.  Also, in the Cortex-A9 MPCore, SGIs are
   * always enabled. The corresponding bits in the ICDISERn are read as
   * one, write ignored.
   */

  if (irq > GIC_IRQ_SGI15 && irq < NR_IRQS)
    {
      uintptr_t regaddr;

      /* Write '1' to the corresponding bit in the distributor Interrupt
       * Set-Enable Register (ICDISER)
       */

      regaddr = GIC_ICDISER(irq);
      putreg32(GIC_ICDISER_INT(irq), regaddr);
    }
}

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   This function implements disabling of the device specified by 'irq'
 *   at the interrupt controller level if supported by the architecture
 *   (up_irq_save() supports the global level, the device level is hardware
 *   specific).
 *
 *   Since this API is not supported on all architectures, it should be
 *   avoided in common implementations where possible.
 *
 ****************************************************************************/

void up_disable_irq(int irq)
{
  /* Ignore invalid interrupt IDs.  Also, in the Cortex-A9 MPCore, SGIs are
   * always enabled. The corresponding bits in the ICDISERn are read as
   * one, write ignored.
   */

  if (irq > GIC_IRQ_SGI15 && irq < NR_IRQS)
    {
      uintptr_t regaddr;

      /* Write '1' to the corresponding bit in the distributor Interrupt
       * Clear-Enable Register (ICDISER)
       */

      regaddr = GIC_ICDICER(irq);
      putreg32(GIC_ICDICER_INT(irq), regaddr);
    }
}

/****************************************************************************
 * Name: up_prioritize_irq
 *
 * Description:
 *   Set the priority of an IRQ.
 *
 *   Since this API is not supported on all architectures, it should be
 *   avoided in common implementations where possible.
 *
 ****************************************************************************/

int up_prioritize_irq(int irq, int priority)
{
  DEBUGASSERT(irq >= 0 && irq < NR_IRQS && priority >= 0 && priority <= 255);

  /* Ignore invalid interrupt IDs */

  if (irq >= 0 && irq < NR_IRQS)
    {
      uintptr_t regaddr;
      uint32_t regval;

      /* Write the new priority to the corresponding field in the in the
       * distributor Interrupt Priority Register (GIC_ICDIPR).
       */

      regaddr = GIC_ICDIPR(irq);
      regval  = getreg32(regaddr);
      regval &= ~GIC_ICDIPR_ID_MASK(irq);
      regval |= GIC_ICDIPR_ID(irq, priority);
      putreg32(regval, regaddr);

      return OK;
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: up_affinity_irq
 *
 * Description:
 *   Set an IRQ affinity by software.
 *
 ****************************************************************************/

void up_affinity_irq(int irq, cpu_set_t cpuset)
{
  if (irq >= GIC_IRQ_SPI && irq < NR_IRQS)
    {
      uintptr_t regaddr;
      uint32_t regval;

      /* Write the new cpuset to the corresponding field in the in the
       * distributor Interrupt Processor Target Register (GIC_ICDIPTR).
       */

      regaddr = GIC_ICDIPTR(irq);
      regval  = getreg32(regaddr);
      regval &= ~GIC_ICDIPTR_ID_MASK(irq);
      regval |= GIC_ICDIPTR_ID(irq, cpuset);
      putreg32(regval, regaddr);
    }
}

/****************************************************************************
 * Name: up_trigger_irq
 *
 * Description:
 *   Perform a Software Generated Interrupt (SGI).  If CONFIG_SMP is
 *   selected, then the SGI is sent to all CPUs specified in the CPU set.
 *   That set may include the current CPU.
 *
 *   If CONFIG_SMP is not selected, the cpuset is ignored and SGI is sent
 *   only to the current CPU.
 *
 * Input Parameters
 *   irq    - The SGI interrupt ID (0-15)
 *   cpuset - The set of CPUs to receive the SGI
 *
 ****************************************************************************/

void up_trigger_irq(int irq, cpu_set_t cpuset)
{
  if (irq >= 0 && irq <= GIC_IRQ_SGI15)
    {
      arm_cpu_sgi(irq, cpuset);
    }
  else if (irq >= 0 && irq < NR_IRQS)
    {
      uintptr_t regaddr;

      /* Write '1' to the corresponding bit in the distributor Interrupt
       * Set-Pending (ICDISPR)
       */

      regaddr = GIC_ICDISPR(irq);
      putreg32(GIC_ICDISPR_INT(irq), regaddr);
    }
}

/****************************************************************************
 * Name: arm64_gicv_irq_trigger
 *
 * Description:
 *   Set the trigger type for the specified IRQ source and the current CPU.
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

int arm64_gicv_irq_trigger(int irq, bool edge)
{
  uintptr_t regaddr;
  uint32_t regval;
  uint32_t intcfg;

  if (irq > GIC_IRQ_SGI15 && irq < NR_IRQS)
    {
      /* Get the address of the Interrupt Configuration Register for this
       * irq.
       */

      regaddr = GIC_ICDICFR(irq);

      /* Get the new Interrupt configuration bit setting */

      intcfg = (edge ? (INT_ICDICFR_EDGE | INT_ICDICFR_1N) : INT_ICDICFR_1N);

      /* Write the correct interrupt trigger to the Interrupt Configuration
       * Register.
       */

      regval  = getreg32(regaddr);
      regval &= ~GIC_ICDICFR_ID_MASK(irq);
      regval |= GIC_ICDICFR_ID(irq, intcfg);
      putreg32(regval, regaddr);

      return OK;
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: arm64_gic_irq_set_priority
 *
 * Description:
 *   Set the interrupt priority and type.
 *
 *   If CONFIG_SMP is not selected, the cpuset is ignored and SGI is sent
 *   only to the current CPU.
 *
 * Input Parameters
 *   intid  - The SGI interrupt ID (0-15)
 *   prio   - The interrupt priority
 *   flags  - Bit IRQ_TYPE_EDGE is 1 if interrupt should be edge-triggered
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void arm64_gic_irq_set_priority(unsigned int intid, unsigned int prio,
                                uint32_t flags)
{
  int ret;

  /* Disable the interrupt */

  up_disable_irq(intid);

  /* Set the interrupt priority */

  ret = up_prioritize_irq(intid, prio);
  DEBUGASSERT(ret == OK);

  /* Configure interrupt type */

  if (!GIC_IS_SGI(intid))
    {
      if (flags & IRQ_TYPE_EDGE)
        {
          ret = arm64_gicv_irq_trigger(intid, true);
          DEBUGASSERT(ret == OK);
        }
      else
        {
          ret = arm64_gicv_irq_trigger(intid, false);
          DEBUGASSERT(ret == OK);
        }
    }
}

/****************************************************************************
 * Name: arm64_gic_initialize
 *
 * Description:
 *   Initialize GIC. Called by CPU0 only.
 *
 * Input Parameters
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

int arm64_gic_initialize(void)
{
  int err;

  /* Verify that GIC Version is 2 */

  err = gic_validate_dist_version();
  if (err)
    {
      sinfo("no distributor detected, giving up ret=%d\n", err);
      return err;
    }

  /* CPU0-specific initialization for GIC */

  arm_gic0_initialize();

  /* CPU-generic initialization for GIC */

  arm_gic_initialize();

  return 0;
}

#ifdef CONFIG_SMP

/****************************************************************************
 * Name: arm64_gic_secondary_init
 *
 * Description:
 *   Initialize GIC. Called by all CPUs except CPU0.
 *
 * Input Parameters
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void arm64_gic_secondary_init(void)
{
  /* CPU-generic initialization for GIC */

  arm_gic_initialize();
}

/****************************************************************************
 * Name: arm64_gic_raise_sgi
 *
 * Description:
 *   Raise software generated interrupt to the target
 *
 * Input Parameters
 *   sgi    - The SGI interrupt ID (0-15)
 *   cpuset - The set of CPUs to receive the SGI
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void arm64_gic_raise_sgi(unsigned int sgi, uint16_t cpuset)
{
  arm_cpu_sgi(sgi, cpuset);
}

#  ifdef CONFIG_SMP
/****************************************************************************
 * Name: up_send_smp_call
 *
 * Description:
 *   Send smp call to target cpu.
 *
 * Input Parameters:
 *   cpuset - The set of CPUs to receive the SGI.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void up_send_smp_call(cpu_set_t cpuset)
{
  up_trigger_irq(GIC_SMP_CPUCALL, cpuset);
}
#  endif
#endif /* CONFIG_SMP */

/****************************************************************************
 * Name: up_get_legacy_irq
 *
 * Description:
 *   Reserve vector for legacy
 *
 ****************************************************************************/

int up_get_legacy_irq(uint32_t devfn, uint8_t line, uint8_t pin)
{
#if CONFIG_ARM64_GICV2_LEGACY_IRQ0 >= 0
  uint8_t slot;
  uint8_t tmp;

  UNUSED(line);
  slot = PCI_SLOT(devfn);
  tmp = (pin - 1 + slot) % 4;
  return CONFIG_ARM64_GICV2_LEGACY_IRQ0 + tmp;
#else
  return -ENOTSUP;
#endif
}

#endif /* CONFIG_ARM64_GIC_VERSION == 2 */
