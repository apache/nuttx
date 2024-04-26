/****************************************************************************
 * arch/arm/src/armv8-r/arm_gic.h
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
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_ARMV8_R_ARM_GIC_H
#define __ARCH_ARM_SRC_ARMV8_R_ARM_GIC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/irq.h>
#include <arch/chip/chip.h>

#include "arm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GIC Distributor register Interface Base Addresses
 * Arm® Generic Interrupt Controller Architecture Specification
 * GIC architecture version 3 and version 4
 */

#define GIC_DIST_BASE           CONFIG_GICD_BASE
#define GICD_CTLR               (GIC_DIST_BASE + 0x0)
#define GICD_TYPER              (GIC_DIST_BASE + 0x4)
#define GICD_IIDR               (GIC_DIST_BASE + 0x8)
#define GICD_STATUSR            (GIC_DIST_BASE + 0x10)
#define GICD_SETSPI_NSR         (GIC_DIST_BASE + 0x40)
#define GICD_CLRSPI_NSR         (GIC_DIST_BASE + 0x48)
#define GICD_SETSPI_SR          (GIC_DIST_BASE + 0x50)
#define GICD_CLRSPI_SR          (GIC_DIST_BASE + 0x58)
#define GICD_IGROUPRn           (GIC_DIST_BASE + 0x80)
#define GICD_ISENABLERn         (GIC_DIST_BASE + 0x100)
#define GICD_ICENABLERn         (GIC_DIST_BASE + 0x180)
#define GICD_ISPENDRn           (GIC_DIST_BASE + 0x200)
#define GICD_ICPENDRn           (GIC_DIST_BASE + 0x280)
#define GICD_ISACTIVERn         (GIC_DIST_BASE + 0x300)
#define GICD_ICACTIVERn         (GIC_DIST_BASE + 0x380)
#define GICD_IPRIORITYRn        (GIC_DIST_BASE + 0x400)
#define GICD_ITARGETSRn         (GIC_DIST_BASE + 0x800)
#define GICD_ICFGRn             (GIC_DIST_BASE + 0xc00)
#define GICD_SGIR               (GIC_DIST_BASE + 0xf00)
#define GICD_IDREGS             (GIC_DIST_BASE + 0xFFD0)
#define GICD_PIDR2              (GIC_DIST_BASE + 0xFFE8)

/* Offsets from GICD base or GICR(n) SGI_base */

#define GIC_DIST_IGROUPR        0x0080
#define GIC_DIST_ISENABLER      0x0100
#define GIC_DIST_ICENABLER      0x0180
#define GIC_DIST_ISPENDR        0x0200
#define GIC_DIST_ICPENDR        0x0280
#define GIC_DIST_ISACTIVER      0x0300
#define GIC_DIST_ICACTIVER      0x0380
#define GIC_DIST_IPRIORITYR     0x0400
#define GIC_DIST_ITARGETSR      0x0800
#define GIC_DIST_ICFGR          0x0c00
#define GIC_DIST_IGROUPMODR     0x0d00
#define GIC_DIST_SGIR           0x0f00

/* GICD GICR common access macros */

#define IGROUPR(base, n)        (base + GIC_DIST_IGROUPR + (n) * 4)
#define ISENABLER(base, n)      (base + GIC_DIST_ISENABLER + (n) * 4)
#define ICENABLER(base, n)      (base + GIC_DIST_ICENABLER + (n) * 4)
#define ISPENDR(base, n)        (base + GIC_DIST_ISPENDR + (n) * 4)
#define ICPENDR(base, n)        (base + GIC_DIST_ICPENDR + (n) * 4)
#define IPRIORITYR(base, n)     (base + GIC_DIST_IPRIORITYR + n)
#define ITARGETSR(base, n)      (base + GIC_DIST_ITARGETSR + (n) * 4)
#define ICFGR(base, n)          (base + GIC_DIST_ICFGR + (n) * 4)
#define IGROUPMODR(base, n)     (base + GIC_DIST_IGROUPMODR + (n) * 4)

/* GICD_PIDR2 : Peripheral ID2 Register
 * bit assignments
 * [31:8] - IMPLEMENTATION DEFINED
 * [7:4] ArchRev 0x1. GICv1.
 *               0x2. GICv2.
 *               0x3. GICv3.
 *               0x4. GICv4.
 * [3:0] - IMPLEMENTATION DEFINED.
 */
#define GICD_PIDR2_ARCH_MASK        0xf0
#define GICD_PIDR2_ARCH_GICV2       0x20
#define GICD_PIDR2_ARCH_GICV3       0x30
#define GICD_PIDR2_ARCH_GICV4       0x40

/* GICD_TYPER : Interrupt Controller Type Register
 * Arm® Generic Interrupt Controller Architecture Specification
 * GIC architecture version 3 and version 4
 */
#define GICD_TYPER_RSS              BIT(26)
#define GICD_TYPER_LPIS             BIT(17)
#define GICD_TYPER_MBIS             BIT(16)
#define GICD_TYPER_ESPI             BIT(8)
#define GICD_TYPER_ID_BITS(typer)   ((((typer) >> 19) & 0x1f) + 1)
#define GICD_TYPER_NUM_LPIS(typer)  ((((typer) >> 11) & 0x1f) + 1)
#define GICD_TYPER_SPIS(typer)      ((((typer) & 0x1f) + 1) * 32)
#define GICD_TYPER_ESPIS(typer) \
  (((typer) & GICD_TYPER_ESPI) ? GICD_TYPER_SPIS((typer) >> 27) : 0)

/* Common Helper Constants */
#define GIC_SGI_INT_BASE            0
#define GIC_PPI_INT_BASE            16
#define GIC_IS_SGI(intid)           (((intid) >= GIC_SGI_INT_BASE) && \
                                     ((intid) < GIC_PPI_INT_BASE))

#define GIC_SPI_INT_BASE            32
#define GIC_NUM_INTR_PER_REG        32
#define GIC_NUM_CFG_PER_REG         16
#define GIC_NUM_PRI_PER_REG         4

/* GIC idle priority : value '0xff' will allow all interrupts */

#define GIC_IDLE_PRIO               0xff

/* Priority levels 0:255 */

#define GIC_PRI_MASK                0xff

/* '0xa0'is used to initialize each interrtupt default priority.
 * This is an arbitrary value in current context.
 * Any value '0x80' to '0xff' will work for both NS and S state.
 * The values of individual interrupt and default has to be chosen
 * carefully if PMR and BPR based nesting and preemption has to be done.
 */

#define GIC_INT_DEF_PRI_X4          0xa0a0a0a0

/* GICD_CTLR : Distributor Control Register
 *
 * [31](RO)  RWP Register Write Pending:
 *            -- 0 No register write in progress
 *            -- 1 Register write in progress
 * [30:8]    - Reserved -
 * [7](RW)   E1NWF Enable 1 of N Wakeup Functionality  0
 * [6](RO)   DS Disable Security status:
 *            -- 0 The gicd_ctlr_ds signal was LOW when the GIC
 *                 exited reset. Therefore, the Distributor supports
 *                 two Security states and Non-secure accesses cannot
 *                 access and modify registers that control Group 0
 *                 interrupts.
 *            -- 1 The gicd_ctlr_ds signal was HIGH when the GIC
 *                 exited reset. Therefore, the Distributor only supports
 *                 a single Security state and Non-secure accesses
 *                 can access and modify registers that control
 *                 Group 0 interrupts.
 * [5](RO)   ARE_NS Affinity Routing Enable, Non-secure state
 * [4](RO)   ARE_S Affinity Routing Enable, Secure state
 * [3]       - Reserved -
 * [2](RW)   EnableGrp1S Enable Secure Group 1 interrupts
 * [1](RW)   EnableGrp1NS Enable Non-secure Group 1 interrupts
 * [0](RW)   EnableGrp0 Enable Group 0 interrupts
 */

#define GICD_CTLR_ENABLE_G0         0
#define GICD_CTLR_ENABLE_G1NS       1
#define GICD_CTLR_ENABLE_G1S        2
#define GICD_CTRL_ARE_S             4
#define GICD_CTRL_ARE_NS            5
#define GICD_CTRL_DS                6
#define GICD_CGRL_E1NWF             7

/* GICD_CTLR Register write progress bit */
#define GICD_CTLR_RWP               31

/* GICR_CTLR */
#define GICR_CTLR_ENABLE_LPIS       BIT(0)
#define GICR_CTLR_RWP               3

/* GICD_TYPER.ITLinesNumber 0:4 */
#define GICD_TYPER_ITLINESNUM_MASK  0x1f

/* GICR： Re-Distributor registers, offsets from RD_base(n) */
#define GICR_CTLR                   0x0000
#define GICR_IIDR                   0x0004
#define GICR_TYPER                  0x0008
#define GICR_STATUSR                0x0010
#define GICR_WAKER                  0x0014
#define GICR_PWRR                   0x0024
#define GICR_SETLPIR                0x0040
#define GICR_CLRLPIR                0x0048
#define GICR_PROPBASER              0x0070
#define GICR_PENDBASER              0x0078
#define GICR_INVLPIR                0x00A0
#define GICR_INVALLR                0x00B0
#define GICR_SYNCR                  0x00C0
#define GICR_MOVLPIR                0x0100
#define GICR_MOVALLR                0x0110
#define GICR_IDREGS                 0xFFD0
#define GICR_PIDR2                  0xFFE8

/* GICR_PIDR2 : Peripheral ID2 Register
 * bit assignments are the same as those for GICD_PIDR2)
 * [31:8] - IMPLEMENTATION DEFINED
 * [7:4] ArchRev 0x1. GICv1.
 *               0x2. GICv2.
 *               0x3. GICv3.
 *               0x4. GICv4.
 * [3:0] - IMPLEMENTATION DEFINED.
 */

#define GICR_PIDR2_ARCH_MASK        0xf0
#define GICR_PIDR2_ARCH_GICV3       0x30
#define GICR_PIDR2_ARCH_GICV4       0x40

/* GICR_TYPER : Redistributor Type Register
 * Arm® Generic Interrupt Controller Architecture Specification
 * GIC architecture version 3 and version 4
 * chapter 9.11.35 for detail descriptions
 */

#define GICR_TYPER_PLPIS            BIT(0)
#define GICR_TYPER_VLPIS            BIT(1)
#define GICR_TYPER_DIRECTLPIS       BIT(3)
#define GICR_TYPER_LAST             BIT(4)

/* GICR_WAKER */
#define GICR_WAKER_PS               1
#define GICR_WAKER_CA               2

/* SGI base is at 64K offset from Redistributor */
#define GICR_SGI_BASE_OFF           0x10000

/* GICD_ICFGR */
#define GICD_ICFGR_MASK             BIT_MASK(2)
#define GICD_ICFGR_TYPE             BIT(1)

/* BIT(0) reserved for IRQ_ZERO_LATENCY */
#define IRQ_TYPE_LEVEL              BIT(1)
#define IRQ_TYPE_EDGE               BIT(2)

#define GIC_SPI_INT_BASE            32
#define GIC_SPI_MAX_INTID           1019
#define GIC_IS_SPI(intid)   (((intid) >= GIC_SPI_INT_BASE) && \
                             ((intid) <= GIC_SPI_MAX_INTID))

/* GITCD_IROUTER */
#define GIC_DIST_IROUTER            0x6000
#define IROUTER(base, n)    (base + GIC_DIST_IROUTER + (n) * 8)

/* BIT(0) reserved for IRQ_ZERO_LATENCY */
#define IRQ_TYPE_LEVEL              BIT(1)
#define IRQ_TYPE_EDGE               BIT(2)

#define IRQ_DEFAULT_PRIORITY        0xa0

#define GIC_IRQ_SGI0              0
#define GIC_IRQ_SGI1              1
#define GIC_IRQ_SGI2              2
#define GIC_IRQ_SGI3              3
#define GIC_IRQ_SGI4              4
#define GIC_IRQ_SGI5              5
#define GIC_IRQ_SGI6              6
#define GIC_IRQ_SGI7              7
#define GIC_IRQ_SGI8              8
#define GIC_IRQ_SGI9              9
#define GIC_IRQ_SGI10            10
#define GIC_IRQ_SGI11            11
#define GIC_IRQ_SGI12            12
#define GIC_IRQ_SGI13            13
#define GIC_IRQ_SGI14            14
#define GIC_IRQ_SGI15            15

/* register constants */
#define ICC_SRE_ELX_SRE_BIT         BIT(0)
#define ICC_SRE_ELX_DFB_BIT         BIT(1)
#define ICC_SRE_ELX_DIB_BIT         BIT(2)
#define ICC_SRE_EL3_EN_BIT          BIT(3)

/* ICC SGI macros */
#define SGIR_TGT_MASK               (0xffff)
#define SGIR_AFF1_SHIFT             (16)
#define SGIR_AFF2_SHIFT             (32)
#define SGIR_AFF3_SHIFT             (48)
#define SGIR_AFF_MASK               (0xf)
#define SGIR_INTID_SHIFT            (24)
#define SGIR_INTID_MASK             (0xf)
#define SGIR_IRM_SHIFT              (40)
#define SGIR_IRM_MASK               (0x1)
#define SGIR_IRM_TO_AFF             (0)
#define SGIR_IRM_TO_ALL             (1)

#define GICV3_SGIR_VALUE(_aff3, _aff2, _aff1, _intid, _irm, _tgt) \
  ((((uint64_t)(_aff3) & SGIR_AFF_MASK) << SGIR_AFF3_SHIFT) |     \
   (((uint64_t)(_irm) & SGIR_IRM_MASK) << SGIR_IRM_SHIFT) |       \
   (((uint64_t)(_aff2) & SGIR_AFF_MASK) << SGIR_AFF2_SHIFT) |     \
   (((_intid) & SGIR_INTID_MASK) << SGIR_INTID_SHIFT) |           \
   (((_aff1) & SGIR_AFF_MASK) << SGIR_AFF1_SHIFT) |               \
   ((_tgt) & SGIR_TGT_MASK))

#ifdef CONFIG_ARCH_TRUSTZONE_SECURE
#  define GIC_SMP_CPUSTART          GIC_IRQ_SGI9
#  define GIC_SMP_CPUPAUSE          GIC_IRQ_SGI10
#  define GIC_SMP_CPUCALL           GIC_IRQ_SGI11
#  define GIC_SMP_CPUPAUSE_ASYNC    GIC_IRQ_SGI12
#else
#  define GIC_SMP_CPUSTART          GIC_IRQ_SGI1
#  define GIC_SMP_CPUPAUSE          GIC_IRQ_SGI2
#  define GIC_SMP_CPUCALL           GIC_IRQ_SGI3
#  define GIC_SMP_CPUPAUSE_ASYNC    GIC_IRQ_SGI4
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

bool arm_gic_irq_is_enabled(unsigned int intid);
int  arm_gic_initialize(void);
void arm_gic_irq_set_priority(unsigned int intid, unsigned int prio,
                                uint32_t flags);
int arm_gic_irq_trigger(unsigned int intid, uint32_t flags);

int arm_gic_raise_sgi(unsigned int sgi_id, uint16_t target_list);

#ifdef CONFIG_SMP

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

int arm_pause_handler(int irq, void *context, void *arg);

#ifdef CONFIG_SMP
int arm_pause_async_handler(int irq, void *context, void *arg);
#endif

void arm_gic_secondary_init(void);

#endif

/****************************************************************************
 * Name: arm_get_mpid
 *
 * Description:
 *   The function from cpu index to get cpu mpid which is reading
 * from mpidr_el1 register. Different ARM64 Core will use different
 * Affn define, the mpidr_el1 value is not CPU number, So we need
 * to change CPU number to mpid and vice versa
 *
 ****************************************************************************/

#ifdef CONFIG_SMP
uint64_t arm_get_mpid(int cpu);
#else
#  define arm_get_mpid(cpu) GET_MPIDR()
#endif /* CONFIG_SMP */

#endif /* __ARCH_ARM_SRC_ARMV8_R_ARM_GIC_H */
