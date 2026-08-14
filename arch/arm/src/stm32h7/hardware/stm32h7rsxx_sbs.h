/****************************************************************************
 * arch/arm/src/stm32h7/hardware/stm32h7rsxx_sbs.h
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

#ifndef __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32H7RSXX_SBS_H
#define __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32H7RSXX_SBS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "stm32_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_SBS_BOOTSR_OFFSET           0x0000 /* Boot status register */
#define STM32_SBS_HDPLCR_OFFSET           0x0010 /* Hide protection control */
#define STM32_SBS_HDPLSR_OFFSET           0x0014 /* Hide protection status */
#define STM32_SBS_DBGCR_OFFSET            0x0020 /* Debug control register */
#define STM32_SBS_DBGLOCKR_OFFSET         0x0024 /* Debug lock register */
#define STM32_SBS_RSSCMDR_OFFSET          0x0034 /* RSS command register */
#define STM32_SBS_PMCR_OFFSET             0x0100 /* Product mode and configuration */
#define STM32_SBS_FPUIMR_OFFSET           0x0104 /* FPU interrupt mask */
#define STM32_SBS_MESR_OFFSET             0x0108 /* Memory erase status */
#define STM32_SBS_CCCSR_OFFSET            0x0110 /* Compensation cell control/status */
#define STM32_SBS_CCVALR_OFFSET           0x0114 /* Compensation cell value */
#define STM32_SBS_CCSWVALR_OFFSET         0x0118 /* Compensation cell software value */
#define STM32_SBS_BKLOCKR_OFFSET          0x0120 /* Break lockup register */
#define STM32_SBS_EXTICR_OFFSET(p)        (0x0130 + ((p) & 0x000c))
#define STM32_SBS_EXTICR1_OFFSET          0x0130 /* EXTI configuration register 1 */
#define STM32_SBS_EXTICR2_OFFSET          0x0134 /* EXTI configuration register 2 */
#define STM32_SBS_EXTICR3_OFFSET          0x0138 /* EXTI configuration register 3 */
#define STM32_SBS_EXTICR4_OFFSET          0x013c /* EXTI configuration register 4 */

/* Register Addresses *******************************************************/

#define STM32_SBS_BOOTSR                  (STM32_SBS_BASE + STM32_SBS_BOOTSR_OFFSET)
#define STM32_SBS_HDPLCR                  (STM32_SBS_BASE + STM32_SBS_HDPLCR_OFFSET)
#define STM32_SBS_HDPLSR                  (STM32_SBS_BASE + STM32_SBS_HDPLSR_OFFSET)
#define STM32_SBS_DBGCR                   (STM32_SBS_BASE + STM32_SBS_DBGCR_OFFSET)
#define STM32_SBS_DBGLOCKR                (STM32_SBS_BASE + STM32_SBS_DBGLOCKR_OFFSET)
#define STM32_SBS_RSSCMDR                 (STM32_SBS_BASE + STM32_SBS_RSSCMDR_OFFSET)
#define STM32_SBS_PMCR                    (STM32_SBS_BASE + STM32_SBS_PMCR_OFFSET)
#define STM32_SBS_FPUIMR                  (STM32_SBS_BASE + STM32_SBS_FPUIMR_OFFSET)
#define STM32_SBS_MESR                    (STM32_SBS_BASE + STM32_SBS_MESR_OFFSET)
#define STM32_SBS_CCCSR                   (STM32_SBS_BASE + STM32_SBS_CCCSR_OFFSET)
#define STM32_SBS_CCVALR                  (STM32_SBS_BASE + STM32_SBS_CCVALR_OFFSET)
#define STM32_SBS_CCSWVALR                (STM32_SBS_BASE + STM32_SBS_CCSWVALR_OFFSET)
#define STM32_SBS_BKLOCKR                 (STM32_SBS_BASE + STM32_SBS_BKLOCKR_OFFSET)
#define STM32_SBS_EXTICR(p)               (STM32_SBS_BASE + STM32_SBS_EXTICR_OFFSET(p))
#define STM32_SBS_EXTICR1                 (STM32_SBS_BASE + STM32_SBS_EXTICR1_OFFSET)
#define STM32_SBS_EXTICR2                 (STM32_SBS_BASE + STM32_SBS_EXTICR2_OFFSET)
#define STM32_SBS_EXTICR3                 (STM32_SBS_BASE + STM32_SBS_EXTICR3_OFFSET)
#define STM32_SBS_EXTICR4                 (STM32_SBS_BASE + STM32_SBS_EXTICR4_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Boot and protection registers */

#define SBS_BOOTSR_INITVTOR_SHIFT         (0) /* Bits 0-31: Initial vector */
#define SBS_BOOTSR_INITVTOR_MASK          (0xffffffff << SBS_BOOTSR_INITVTOR_SHIFT)
#define SBS_HDPLCR_INCR_HDPL_SHIFT        (0) /* Bits 0-7: Increment HDPL */
#define SBS_HDPLCR_INCR_HDPL_MASK         (0xff << SBS_HDPLCR_INCR_HDPL_SHIFT)
#define SBS_HDPLSR_HDPL_SHIFT             (0) /* Bits 0-7: Current HDPL */
#define SBS_HDPLSR_HDPL_MASK              (0xff << SBS_HDPLSR_HDPL_SHIFT)

/* Debug registers */

#define SBS_DBGCR_AP_UNLOCK_SHIFT         (0)  /* Bits 0-7: Access port unlock */
#define SBS_DBGCR_AP_UNLOCK_MASK          (0xff << SBS_DBGCR_AP_UNLOCK_SHIFT)
#define SBS_DBGCR_DBG_UNLOCK_SHIFT        (8)  /* Bits 8-15: Debug unlock */
#define SBS_DBGCR_DBG_UNLOCK_MASK         (0xff << SBS_DBGCR_DBG_UNLOCK_SHIFT)
#define SBS_DBGCR_DBG_AUTH_HDPL_SHIFT     (16) /* Bits 16-23: Debug auth HDPL */
#define SBS_DBGCR_DBG_AUTH_HDPL_MASK      (0xff << SBS_DBGCR_DBG_AUTH_HDPL_SHIFT)
#define SBS_DBGLOCKR_DBGCFG_LOCK_SHIFT    (0)  /* Bits 0-7: Debug config lock */
#define SBS_DBGLOCKR_DBGCFG_LOCK_MASK     (0xff << SBS_DBGLOCKR_DBGCFG_LOCK_SHIFT)
#define SBS_RSSCMDR_RSSCMD_SHIFT          (0)  /* Bits 0-15: RSS command */
#define SBS_RSSCMDR_RSSCMD_MASK           (0xffff << SBS_RSSCMDR_RSSCMD_SHIFT)

/* Product mode and configuration register */

#define SBS_PMCR_BOOSTEN                  (1 << 8)  /* Booster enable */
#define SBS_PMCR_BOOSTVDDSEL              (1 << 9)  /* Booster VDD selection */
#define SBS_PMCR_ETH_PHYSEL_SHIFT         (21)      /* Bits 21-23: PHY selection */
#define SBS_PMCR_ETH_PHYSEL_MASK          (7 << SBS_PMCR_ETH_PHYSEL_SHIFT)
#  define SBS_PMCR_ETH_PHYSEL_GMII_MII    (0 << SBS_PMCR_ETH_PHYSEL_SHIFT)
#  define SBS_PMCR_ETH_PHYSEL_RMII        (4 << SBS_PMCR_ETH_PHYSEL_SHIFT)
#define SBS_PMCR_AXISRAM_WS               (1 << 28) /* AXI SRAM wait state */

/* FPU interrupt mask and memory erase status registers */

#define SBS_FPUIMR_FPU_IE_MASK            (0x3f << 0) /* Bits 0-5: FPU IRQ enable */
#define SBS_FPUIMR_FPU_IE_INVALID         (1 << 0)    /* Invalid operation */
#define SBS_FPUIMR_FPU_IE_DIVZERO         (1 << 1)    /* Divide by zero */
#define SBS_FPUIMR_FPU_IE_UNDERFLOW       (1 << 2)    /* Underflow */
#define SBS_FPUIMR_FPU_IE_OVERFLOW        (1 << 3)    /* Overflow */
#define SBS_FPUIMR_FPU_IE_DENORMAL        (1 << 4)    /* Input denormal */
#define SBS_FPUIMR_FPU_IE_INEXACT         (1 << 5)    /* Inexact result */
#define SBS_MESR_MEF                      (1 << 0)    /* Memory erase flag */

/* Compensation cell registers */

#define SBS_CCCSR_COMP_EN                 (1 << 0)  /* Compensation cell enable */
#define SBS_CCCSR_COMP_CODESEL            (1 << 1)  /* Compensation code select */
#define SBS_CCCSR_XSPI1_COMP_EN           (1 << 2)  /* XSPI1 compensation enable */
#define SBS_CCCSR_XSPI1_COMP_CODESEL      (1 << 3)  /* XSPI1 code select */
#define SBS_CCCSR_XSPI2_COMP_EN           (1 << 4)  /* XSPI2 compensation enable */
#define SBS_CCCSR_XSPI2_COMP_CODESEL      (1 << 5)  /* XSPI2 code select */
#define SBS_CCCSR_COMP_RDY                (1 << 8)  /* Compensation cell ready */
#define SBS_CCCSR_XSPI1_COMP_RDY          (1 << 9)  /* XSPI1 compensation ready */
#define SBS_CCCSR_XSPI2_COMP_RDY          (1 << 10) /* XSPI2 compensation ready */
#define SBS_CCCSR_IOHSLV                  (1 << 16) /* High-speed low-voltage I/O */
#define SBS_CCCSR_XSPI1_IOHSLV            (1 << 17) /* XSPI1 high-speed I/O */
#define SBS_CCCSR_XSPI2_IOHSLV            (1 << 18) /* XSPI2 high-speed I/O */

#define SBS_CCVALR_NSRC_SHIFT             (0)  /* Bits 0-3: NMOS compensation */
#define SBS_CCVALR_NSRC_MASK              (15 << SBS_CCVALR_NSRC_SHIFT)
#define SBS_CCVALR_PSRC_SHIFT             (4)  /* Bits 4-7: PMOS compensation */
#define SBS_CCVALR_PSRC_MASK              (15 << SBS_CCVALR_PSRC_SHIFT)
#define SBS_CCVALR_XSPI1_NSRC_SHIFT       (8)  /* Bits 8-11: XSPI1 NMOS value */
#define SBS_CCVALR_XSPI1_NSRC_MASK        (15 << SBS_CCVALR_XSPI1_NSRC_SHIFT)
#define SBS_CCVALR_XSPI1_PSRC_SHIFT       (12) /* Bits 12-15: XSPI1 PMOS value */
#define SBS_CCVALR_XSPI1_PSRC_MASK        (15 << SBS_CCVALR_XSPI1_PSRC_SHIFT)
#define SBS_CCVALR_XSPI2_NSRC_SHIFT       (16) /* Bits 16-19: XSPI2 NMOS value */
#define SBS_CCVALR_XSPI2_NSRC_MASK        (15 << SBS_CCVALR_XSPI2_NSRC_SHIFT)
#define SBS_CCVALR_XSPI2_PSRC_SHIFT       (20) /* Bits 20-23: XSPI2 PMOS value */
#define SBS_CCVALR_XSPI2_PSRC_MASK        (15 << SBS_CCVALR_XSPI2_PSRC_SHIFT)

#define SBS_CCSWVALR_SW_NSRC_SHIFT        (0)  /* Bits 0-3: Software NMOS value */
#define SBS_CCSWVALR_SW_NSRC_MASK         (15 << SBS_CCSWVALR_SW_NSRC_SHIFT)
#define SBS_CCSWVALR_SW_PSRC_SHIFT        (4)  /* Bits 4-7: Software PMOS value */
#define SBS_CCSWVALR_SW_PSRC_MASK         (15 << SBS_CCSWVALR_SW_PSRC_SHIFT)
#define SBS_CCSWVALR_XSPI1_SW_NSRC_SHIFT  (8)  /* Bits 8-11: XSPI1 NMOS value */
#define SBS_CCSWVALR_XSPI1_SW_NSRC_MASK   (15 << SBS_CCSWVALR_XSPI1_SW_NSRC_SHIFT)
#define SBS_CCSWVALR_XSPI1_SW_PSRC_SHIFT  (12) /* Bits 12-15: XSPI1 PMOS value */
#define SBS_CCSWVALR_XSPI1_SW_PSRC_MASK   (15 << SBS_CCSWVALR_XSPI1_SW_PSRC_SHIFT)
#define SBS_CCSWVALR_XSPI2_SW_NSRC_SHIFT  (16) /* Bits 16-19: XSPI2 NMOS value */
#define SBS_CCSWVALR_XSPI2_SW_NSRC_MASK   (15 << SBS_CCSWVALR_XSPI2_SW_NSRC_SHIFT)
#define SBS_CCSWVALR_XSPI2_SW_PSRC_SHIFT  (20) /* Bits 20-23: XSPI2 PMOS value */
#define SBS_CCSWVALR_XSPI2_SW_PSRC_MASK   (15 << SBS_CCSWVALR_XSPI2_SW_PSRC_SHIFT)

/* Break lockup register */

#define SBS_BKLOCKR_PVD_BL                (1 << 2)  /* PVD break lock */
#define SBS_BKLOCKR_FLASHECC_BL           (1 << 3)  /* FLASH ECC break lock */
#define SBS_BKLOCKR_CM7LCKUP_BL           (1 << 6)  /* Cortex-M7 lockup break */
#define SBS_BKLOCKR_BKRAMECC_BL           (1 << 7)  /* Backup RAM ECC break */
#define SBS_BKLOCKR_DTCMECC_BL            (1 << 13) /* DTCM ECC break */
#define SBS_BKLOCKR_ITCMECC_BL            (1 << 14) /* ITCM ECC break */
#define SBS_BKLOCKR_ARAM3ECC_BL           (1 << 21) /* AXI SRAM3 ECC break */
#define SBS_BKLOCKR_ARAM1ECC_BL           (1 << 23) /* AXI SRAM1 ECC break */

/* External interrupt configuration registers */

#define SBS_EXTICR_EXTI_SHIFT(p)          (((p) & 3) << 2)
#define SBS_EXTICR_PORT_MASK              (15)
#define SBS_EXTICR_EXTI_MASK(p)           (SBS_EXTICR_PORT_MASK << SBS_EXTICR_EXTI_SHIFT(p))

#define SBS_EXTICR_PORTA                  0
#define SBS_EXTICR_PORTB                  1
#define SBS_EXTICR_PORTC                  2
#define SBS_EXTICR_PORTD                  3
#define SBS_EXTICR_PORTE                  4
#define SBS_EXTICR_PORTF                  5
#define SBS_EXTICR_PORTG                  6
#define SBS_EXTICR_PORTH                  7
#define SBS_EXTICR_PORTM                  12
#define SBS_EXTICR_PORTN                  13
#define SBS_EXTICR_PORTO                  14
#define SBS_EXTICR_PORTP                  15

#endif /* __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32H7RSXX_SBS_H */
