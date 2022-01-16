/****************************************************************************
 * arch/arm/src/armv7-m/nvic.h
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

#ifndef __ARCH_ARM_SRC_ARMV7_M_NVIC_H
#define __ARCH_ARM_SRC_ARMV7_M_NVIC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Exception/interrupt vector numbers ***************************************/

                                              /* Vector  0: Reset stack
                                               *            pointer value
                                               */

                                               /* Vector  1: Reset */
#define NVIC_IRQ_NMI                    (2)    /* Vector  2: Non-Maskable Interrupt (NMI) */
#define NVIC_IRQ_HARDFAULT              (3)    /* Vector  3: Hard fault */
#define NVIC_IRQ_MEMFAULT               (4)    /* Vector  4: Memory management (MPU) */
#define NVIC_IRQ_BUSFAULT               (5)    /* Vector  5: Bus fault */
#define NVIC_IRQ_USAGEFAULT             (6)    /* Vector  6: Usage fault */
                                               /* Vectors 7-10: Reserved */
#define NVIC_IRQ_SVCALL                 (11)   /* Vector 11: SVC call */
#define NVIC_IRQ_DBGMONITOR             (12)   /* Vector 12: Debug Monitor */
                                               /* Vector 13: Reserved */
#define NVIC_IRQ_PENDSV                 (14)   /* Vector 14: Pendable system service request */
#define NVIC_IRQ_SYSTICK                (15)   /* Vector 15: System tick */

/* External interrupts (vectors >= 16).
 * These definitions are chip-specific
 */

#define NVIC_IRQ_FIRST                  (16)    /* Vector number of the first interrupt */

/* NVIC base address ********************************************************/

#define ARMV7M_NVIC_BASE                0xe000e000

/* NVIC register offsets ****************************************************/

#define NVIC_ICTR_OFFSET                0x0004 /* Interrupt controller type register */
#define NVIC_SYSTICK_CTRL_OFFSET        0x0010 /* SysTick control and status register */
#define NVIC_SYSTICK_RELOAD_OFFSET      0x0014 /* SysTick reload value register */
#define NVIC_SYSTICK_CURRENT_OFFSET     0x0018 /* SysTick current value register */
#define NVIC_SYSTICK_CALIB_OFFSET       0x001c /* SysTick calibration value register */

#define NVIC_IRQ_ENABLE_OFFSET(n)       (0x0100 + 4*((n) >> 5))
#define NVIC_IRQ0_31_ENABLE_OFFSET      0x0100 /* IRQ 0-31 set enable register */
#define NVIC_IRQ32_63_ENABLE_OFFSET     0x0104 /* IRQ 32-63 set enable register */
#define NVIC_IRQ64_95_ENABLE_OFFSET     0x0108 /* IRQ 64-95 set enable register */
#define NVIC_IRQ96_127_ENABLE_OFFSET    0x010c /* IRQ 96-127 set enable register */
#define NVIC_IRQ128_159_ENABLE_OFFSET   0x0110 /* IRQ 128-159 set enable register */
#define NVIC_IRQ160_191_ENABLE_OFFSET   0x0114 /* IRQ 160-191 set enable register */
#define NVIC_IRQ192_223_ENABLE_OFFSET   0x0118 /* IRQ 192-223 set enable register */
#define NVIC_IRQ224_239_ENABLE_OFFSET   0x011c /* IRQ 224-239 set enable register */

#define NVIC_IRQ_CLEAR_OFFSET(n)        (0x0180 + 4*((n) >> 5))
#define NVIC_IRQ0_31_CLEAR_OFFSET       0x0180 /* IRQ 0-31 clear enable register */
#define NVIC_IRQ32_63_CLEAR_OFFSET      0x0184 /* IRQ 32-63 clear enable register */
#define NVIC_IRQ64_95_CLEAR_OFFSET      0x0188 /* IRQ 64-95 clear enable register */
#define NVIC_IRQ96_127_CLEAR_OFFSET     0x018c /* IRQ 96-127 clear enable register */
#define NVIC_IRQ128_159_CLEAR_OFFSET    0x0190 /* IRQ 128-159 clear enable register */
#define NVIC_IRQ160_191_CLEAR_OFFSET    0x0194 /* IRQ 160-191 clear enable register */
#define NVIC_IRQ192_223_CLEAR_OFFSET    0x0198 /* IRQ 192-223 clear enable register */
#define NVIC_IRQ224_239_CLEAR_OFFSET    0x019c /* IRQ 224-2391 clear enable register */

#define NVIC_IRQ_PEND_OFFSET(n)         (0x0200 + 4*((n) >> 5))
#define NVIC_IRQ0_31_PEND_OFFSET        0x0200 /* IRQ 0-31 set pending register */
#define NVIC_IRQ32_63_PEND_OFFSET       0x0204 /* IRQ 32-63 set pending register */
#define NVIC_IRQ64_95_PEND_OFFSET       0x0208 /* IRQ 64-95 set pending register */
#define NVIC_IRQ96_127_PEND_OFFSET      0x020c /* IRQ 96-127 set pending register */
#define NVIC_IRQ128_159_PEND_OFFSET     0x0210 /* IRQ 128-159 set pending register */
#define NVIC_IRQ160_191_PEND_OFFSET     0x0214 /* IRQ 160-191 set pending register */
#define NVIC_IRQ192_223_PEND_OFFSET     0x0218 /* IRQ 192-2231 set pending register */
#define NVIC_IRQ224_239_PEND_OFFSET     0x021c /* IRQ 224-2391 set pending register */

#define NVIC_IRQ_CLRPEND_OFFSET(n)      (0x0280 + 4*((n) >> 5))
#define NVIC_IRQ0_31_CLRPEND_OFFSET     0x0280 /* IRQ 0-31 clear pending register */
#define NVIC_IRQ32_63_CLRPEND_OFFSET    0x0284 /* IRQ 32-63 clear pending register */
#define NVIC_IRQ64_95_CLRPEND_OFFSET    0x0288 /* IRQ 64-95 clear pending register */
#define NVIC_IRQ96_127_CLRPEND_OFFSET   0x028c /* IRQ 96-127 clear pending register */
#define NVIC_IRQ128_159_CLRPEND_OFFSET  0x0290 /* IRQ 128-159 clear pending register */
#define NVIC_IRQ160_191_CLRPEND_OFFSET  0x0294 /* IRQ 160-191 clear pending register */
#define NVIC_IRQ192_223_CLRPEND_OFFSET  0x0298 /* IRQ 192-223 clear pending register */
#define NVIC_IRQ224_239_CLRPEND_OFFSET  0x029c /* IRQ 224-239 clear pending register */

#define NVIC_IRQ_ACTIVE_OFFSET(n)       (0x0300 + 4*((n) >> 5))
#define NVIC_IRQ0_31_ACTIVE_OFFSET      0x0300 /* IRQ 0-31 active bit register */
#define NVIC_IRQ32_63_ACTIVE_OFFSET     0x0304 /* IRQ 32-63 active bit register */
#define NVIC_IRQ64_95_ACTIVE_OFFSET     0x0308 /* IRQ 64-95 active bit register */
#define NVIC_IRQ96_127_ACTIVE_OFFSET    0x030c /* IRQ 96-127 active bit register */
#define NVIC_IRQ128_159_ACTIVE_OFFSET   0x0310 /* IRQ 128-159 active bit register */
#define NVIC_IRQ160_191_ACTIVE_OFFSET   0x0314 /* IRQ 160-191 active bit register */
#define NVIC_IRQ192_223_ACTIVE_OFFSET   0x0318 /* IRQ 192-223 active bit register */
#define NVIC_IRQ224_239_ACTIVE_OFFSET   0x031c /* IRQ 224-239 active bit register */

#define NVIC_IRQ_PRIORITY_OFFSET(n)     (0x0400 + 4*((n) >> 2))
#define NVIC_IRQ0_3_PRIORITY_OFFSET     0x0400 /* IRQ 0-3 priority register */
#define NVIC_IRQ4_7_PRIORITY_OFFSET     0x0404 /* IRQ 4-7 priority register */
#define NVIC_IRQ8_11_PRIORITY_OFFSET    0x0408 /* IRQ 8-11 priority register */
#define NVIC_IRQ12_15_PRIORITY_OFFSET   0x040c /* IRQ 12-15 priority register */
#define NVIC_IRQ16_19_PRIORITY_OFFSET   0x0410 /* IRQ 16-19 priority register */
#define NVIC_IRQ20_23_PRIORITY_OFFSET   0x0414 /* IRQ 20-23 priority register */
#define NVIC_IRQ24_27_PRIORITY_OFFSET   0x0418 /* IRQ 24-29 priority register */
#define NVIC_IRQ28_31_PRIORITY_OFFSET   0x041c /* IRQ 28-31 priority register */
#define NVIC_IRQ32_35_PRIORITY_OFFSET   0x0420 /* IRQ 32-35 priority register */
#define NVIC_IRQ36_39_PRIORITY_OFFSET   0x0424 /* IRQ 36-39 priority register */
#define NVIC_IRQ40_43_PRIORITY_OFFSET   0x0428 /* IRQ 40-43 priority register */
#define NVIC_IRQ44_47_PRIORITY_OFFSET   0x042c /* IRQ 44-47 priority register */
#define NVIC_IRQ48_51_PRIORITY_OFFSET   0x0430 /* IRQ 48-51 priority register */
#define NVIC_IRQ52_55_PRIORITY_OFFSET   0x0434 /* IRQ 52-55 priority register */
#define NVIC_IRQ56_59_PRIORITY_OFFSET   0x0438 /* IRQ 56-59 priority register */
#define NVIC_IRQ60_63_PRIORITY_OFFSET   0x043c /* IRQ 60-63 priority register */
#define NVIC_IRQ64_67_PRIORITY_OFFSET   0x0440 /* IRQ 64-67 priority register */
#define NVIC_IRQ68_71_PRIORITY_OFFSET   0x0444 /* IRQ 68-71 priority register */
#define NVIC_IRQ72_75_PRIORITY_OFFSET   0x0448 /* IRQ 72-75 priority register */
#define NVIC_IRQ76_79_PRIORITY_OFFSET   0x044c /* IRQ 76-79 priority register */
#define NVIC_IRQ80_83_PRIORITY_OFFSET   0x0450 /* IRQ 80-83 priority register */
#define NVIC_IRQ84_87_PRIORITY_OFFSET   0x0454 /* IRQ 84-87 priority register */
#define NVIC_IRQ88_91_PRIORITY_OFFSET   0x0458 /* IRQ 88-91 priority register */
#define NVIC_IRQ92_95_PRIORITY_OFFSET   0x045c /* IRQ 92-95 priority register */
#define NVIC_IRQ96_99_PRIORITY_OFFSET   0x0460 /* IRQ 96-99 priority register */
#define NVIC_IRQ100_103_PRIORITY_OFFSET 0x0464 /* IRQ 100-103 priority register */
#define NVIC_IRQ104_107_PRIORITY_OFFSET 0x0468 /* IRQ 104-107 priority register */
#define NVIC_IRQ108_111_PRIORITY_OFFSET 0x046c /* IRQ 108-111 priority register */
#define NVIC_IRQ112_115_PRIORITY_OFFSET 0x0470 /* IRQ 112-115 priority register */
#define NVIC_IRQ116_119_PRIORITY_OFFSET 0x0474 /* IRQ 116-119 priority register */
#define NVIC_IRQ120_123_PRIORITY_OFFSET 0x0478 /* IRQ 120-123 priority register */
#define NVIC_IRQ124_127_PRIORITY_OFFSET 0x047c /* IRQ 124-127 priority register */
#define NVIC_IRQ128_131_PRIORITY_OFFSET 0x0480 /* IRQ 128-131 priority register */
#define NVIC_IRQ132_135_PRIORITY_OFFSET 0x0484 /* IRQ 132-135 priority register */
#define NVIC_IRQ136_139_PRIORITY_OFFSET 0x0488 /* IRQ 136-139 priority register */
#define NVIC_IRQ140_143_PRIORITY_OFFSET 0x048c /* IRQ 140-143 priority register */
#define NVIC_IRQ144_147_PRIORITY_OFFSET 0x0490 /* IRQ 144-147 priority register */
#define NVIC_IRQ148_151_PRIORITY_OFFSET 0x0494 /* IRQ 148-151 priority register */
#define NVIC_IRQ152_155_PRIORITY_OFFSET 0x0498 /* IRQ 152-155 priority register */
#define NVIC_IRQ156_159_PRIORITY_OFFSET 0x049c /* IRQ 156-159 priority register */
#define NVIC_IRQ160_163_PRIORITY_OFFSET 0x04a0 /* IRQ 160-163 priority register */
#define NVIC_IRQ164_167_PRIORITY_OFFSET 0x04a4 /* IRQ 164-167 priority register */
#define NVIC_IRQ168_171_PRIORITY_OFFSET 0x04a8 /* IRQ 168-171 priority register */
#define NVIC_IRQ172_175_PRIORITY_OFFSET 0x04ac /* IRQ 172-175 priority register */
#define NVIC_IRQ176_179_PRIORITY_OFFSET 0x04b0 /* IRQ 176-179 priority register */
#define NVIC_IRQ180_183_PRIORITY_OFFSET 0x04b4 /* IRQ 180-183 priority register */
#define NVIC_IRQ184_187_PRIORITY_OFFSET 0x04b8 /* IRQ 184-187 priority register */
#define NVIC_IRQ188_191_PRIORITY_OFFSET 0x04bc /* IRQ 188-191 priority register */
#define NVIC_IRQ192_195_PRIORITY_OFFSET 0x04c0 /* IRQ 192-195 priority register */
#define NVIC_IRQ196_199_PRIORITY_OFFSET 0x04c4 /* IRQ 196-199 priority register */
#define NVIC_IRQ200_203_PRIORITY_OFFSET 0x04c8 /* IRQ 200-203 priority register */
#define NVIC_IRQ204_207_PRIORITY_OFFSET 0x04cc /* IRQ 204-207 priority register */
#define NVIC_IRQ208_211_PRIORITY_OFFSET 0x04d0 /* IRQ 208-211 priority register */
#define NVIC_IRQ212_215_PRIORITY_OFFSET 0x04d4 /* IRQ 212-215 priority register */
#define NVIC_IRQ216_219_PRIORITY_OFFSET 0x04d8 /* IRQ 216-219 priority register */
#define NVIC_IRQ220_223_PRIORITY_OFFSET 0x04dc /* IRQ 220-223 priority register */
#define NVIC_IRQ224_227_PRIORITY_OFFSET 0x04e0 /* IRQ 224-227 priority register */
#define NVIC_IRQ228_231_PRIORITY_OFFSET 0x04e4 /* IRQ 228-231 priority register */
#define NVIC_IRQ232_235_PRIORITY_OFFSET 0x04e8 /* IRQ 232-235 priority register */
#define NVIC_IRQ236_239_PRIORITY_OFFSET 0x04ec /* IRQ 236-239 priority register */

/* System Control Block (SCB) */

#define NVIC_CPUID_BASE_OFFSET          0x0d00 /* CPUID base register */
#define NVIC_INTCTRL_OFFSET             0x0d04 /* Interrupt control state register */
#define NVIC_VECTAB_OFFSET              0x0d08 /* Vector table offset register */
#define NVIC_AIRCR_OFFSET               0x0d0c /* Application interrupt/reset control registr */
#define NVIC_SYSCON_OFFSET              0x0d10 /* System control register */
#define NVIC_CFGCON_OFFSET              0x0d14 /* Configuration control register */
#define NVIC_SYSH_PRIORITY_OFFSET(n)    (0x0d14 + 4*((n) >> 2))
#define NVIC_SYSH4_7_PRIORITY_OFFSET    0x0d18 /* System handlers 4-7 priority register */
#define NVIC_SYSH8_11_PRIORITY_OFFSET   0x0d1c /* System handler 8-11 priority register */
#define NVIC_SYSH12_15_PRIORITY_OFFSET  0x0d20 /* System handler 12-15 priority register */
#define NVIC_SYSHCON_OFFSET             0x0d24 /* System handler control and state register */
#define NVIC_CFAULTS_OFFSET             0x0d28 /* Configurable fault status register */
#define NVIC_HFAULTS_OFFSET             0x0d2c /* Hard fault status register */
#define NVIC_DFAULTS_OFFSET             0x0d30 /* Debug fault status register */
#define NVIC_MEMMANAGE_ADDR_OFFSET      0x0d34 /* Mem manage address register */
#define NVIC_BFAULT_ADDR_OFFSET         0x0d38 /* Bus fault address register */
#define NVIC_AFAULTS_OFFSET             0x0d3c /* Auxiliary fault status register */
#define NVIC_PFR0_OFFSET                0x0d40 /* Processor feature register 0 */
#define NVIC_PFR1_OFFSET                0x0d44 /* Processor feature register 1 */
#define NVIC_DFR0_OFFSET                0x0d48 /* Debug feature register 0 */
#define NVIC_AFR0_OFFSET                0x0d4c /* Auxiliary feature register 0 */
#define NVIC_MMFR0_OFFSET               0x0d50 /* Memory model feature register 0 */
#define NVIC_MMFR1_OFFSET               0x0d54 /* Memory model feature register 1 */
#define NVIC_MMFR2_OFFSET               0x0d58 /* Memory model feature register 2 */
#define NVIC_MMFR3_OFFSET               0x0d5c /* Memory model feature register 3 */
#define NVIC_ISAR0_OFFSET               0x0d60 /* ISA feature register 0 */
#define NVIC_ISAR1_OFFSET               0x0d64 /* ISA feature register 1 */
#define NVIC_ISAR2_OFFSET               0x0d68 /* ISA feature register 2 */
#define NVIC_ISAR3_OFFSET               0x0d6c /* ISA feature register 3 */
#define NVIC_ISAR4_OFFSET               0x0d70 /* ISA feature register 4 */
#define NVIC_CLIDR_OFFSET               0x0d78 /* Cache Level ID register (Cortex-M7) */
#define NVIC_CTR_OFFSET                 0x0d7c /* Cache Type register (Cortex-M7) */
#define NVIC_CCSIDR_OFFSET              0x0d80 /* Cache Size ID Register (Cortex-M7) */
#define NVIC_CSSELR_OFFSET              0x0d84 /* Cache Size Selection Register (Cortex-M7) */
#define NVIC_CPACR_OFFSET               0x0d88 /* Coprocessor Access Control Register */
#define NVIC_DHCSR_OFFSET               0x0df0 /* Debug Halting Control and Status Register */
#define NVIC_DCRSR_OFFSET               0x0df4 /* Debug Core Register Selector Register */
#define NVIC_DCRDR_OFFSET               0x0df8 /* Debug Core Register Data Register */
#define NVIC_DEMCR_OFFSET               0x0dfc /* Debug Exception and Monitor Control Register */
#define NVIC_STIR_OFFSET                0x0f00 /* Software trigger interrupt register */
#define NVIC_FPCCR_OFFSET               0x0f34 /* Floating-point Context Control Register */
#define NVIC_FPCAR_OFFSET               0x0f38 /* Floating-point Context Address Register */
#define NVIC_FPDSCR_OFFSET              0x0f3c /* Floating-point Default Status Control Register */
#define NVIC_MVFR0_OFFSET               0x0f40 /* Media and VFP Feature Register 0 */
#define NVIC_MVFR1_OFFSET               0x0f44 /* Media and VFP Feature Register 1 */
#define NVIC_MVFR2_OFFSET               0x0f48 /* Media and VFP Feature Register 2 */
#define NVIC_ICIALLU_OFFSET             0x0f50 /* I-Cache Invalidate All to PoU (Cortex-M7) */
#define NVIC_ICIMVAU_OFFSET             0x0f58 /* I-Cache Invalidate by MVA to PoU (Cortex-M7) */
#define NVIC_DCIMVAC_OFFSET             0x0f5c /* D-Cache Invalidate by MVA to PoC (Cortex-M7) */
#define NVIC_DCISW_OFFSET               0x0f60 /* D-Cache Invalidate by Set-way (Cortex-M7) */
#define NVIC_DCCMVAU_OFFSET             0x0f64 /* D-Cache Clean by MVA to PoU (Cortex-M7) */
#define NVIC_DCCMVAC_OFFSET             0x0f68 /* D-Cache Clean by MVA to PoC (Cortex-M7) */
#define NVIC_DCCSW_OFFSET               0x0f6c /* D-Cache Clean by Set-way (Cortex-M7) */
#define NVIC_DCCIMVAC_OFFSET            0x0f70 /* D-Cache Clean and Invalidate by MVA to PoC (Cortex-M7) */
#define NVIC_DCCISW_OFFSET              0x0f74 /* D-Cache Clean and Invalidate by Set-way (Cortex-M7) */
#define NVIC_BPIALL_OFFSET              0x0f78 /* Branch predictor invalidate all (Cortex-M7) */
#define NVIC_ITCMCR_OFFSET              0x0f90 /* Instruction Tightly-Coupled Memory Control Register */
#define NVIC_DTCMCR_OFFSET              0x0f94 /* Data Tightly-Coupled Memory Control Registers */
#define NVIC_AHBPCR_OFFSET              0x0f98 /* AHBP Control Register */
#define NVIC_CACR_OFFSET                0x0f9c /* L1 Cache Control Register */
#define NVIC_AHBSCR_OFFSET              0x0fa0 /* AHB Slave Control Register */
#define NVIC_ABFSR_OFFSET               0x0fa8 /* Auxiliary Bus Fault Status */
#define NVIC_PID4_OFFSET                0x0fd0 /* Peripheral identification register (PID4) */
#define NVIC_PID5_OFFSET                0x0fd4 /* Peripheral identification register (PID5) */
#define NVIC_PID6_OFFSET                0x0fd8 /* Peripheral identification register (PID6) */
#define NVIC_PID7_OFFSET                0x0fdc /* Peripheral identification register (PID7) */
#define NVIC_PID0_OFFSET                0x0fe0 /* Peripheral identification register bits 7:0 (PID0) */
#define NVIC_PID1_OFFSET                0x0fe4 /* Peripheral identification register bits 15:8 (PID1) */
#define NVIC_PID2_OFFSET                0x0fe8 /* Peripheral identification register bits 23:16 (PID2) */
#define NVIC_PID3_OFFSET                0x0fec /* Peripheral identification register bits 23:16 (PID3) */
#define NVIC_CID0_OFFSET                0x0ff0 /* Component identification register bits 7:0 (CID0) */
#define NVIC_CID1_OFFSET                0x0ff4 /* Component identification register bits 15:8 (CID0) */
#define NVIC_CID2_OFFSET                0x0ff8 /* Component identification register bits 23:16 (CID0) */
#define NVIC_CID3_OFFSET                0x0ffc /* Component identification register bits 23:16 (CID0) */

/* NVIC register addresses **************************************************/

#define NVIC_ICTR                       (ARMV7M_NVIC_BASE + NVIC_ICTR_OFFSET)
#define NVIC_SYSTICK_CTRL               (ARMV7M_NVIC_BASE + NVIC_SYSTICK_CTRL_OFFSET)
#define NVIC_SYSTICK_RELOAD             (ARMV7M_NVIC_BASE + NVIC_SYSTICK_RELOAD_OFFSET)
#define NVIC_SYSTICK_CURRENT            (ARMV7M_NVIC_BASE + NVIC_SYSTICK_CURRENT_OFFSET)
#define NVIC_SYSTICK_CALIB              (ARMV7M_NVIC_BASE + NVIC_SYSTICK_CALIB_OFFSET)

#define NVIC_IRQ_ENABLE(n)              (ARMV7M_NVIC_BASE + NVIC_IRQ_ENABLE_OFFSET(n))
#define NVIC_IRQ0_31_ENABLE             (ARMV7M_NVIC_BASE + NVIC_IRQ0_31_ENABLE_OFFSET)
#define NVIC_IRQ32_63_ENABLE            (ARMV7M_NVIC_BASE + NVIC_IRQ32_63_ENABLE_OFFSET)
#define NVIC_IRQ64_95_ENABLE            (ARMV7M_NVIC_BASE + NVIC_IRQ64_95_ENABLE_OFFSET)
#define NVIC_IRQ96_127_ENABLE           (ARMV7M_NVIC_BASE + NVIC_IRQ96_127_ENABLE_OFFSET)
#define NVIC_IRQ128_159_ENABLE          (ARMV7M_NVIC_BASE + NVIC_IRQ128_159_ENABLE_OFFSET)
#define NVIC_IRQ160_191_ENABLE          (ARMV7M_NVIC_BASE + NVIC_IRQ160_191_ENABLE_OFFSET)
#define NVIC_IRQ192_223_ENABLE          (ARMV7M_NVIC_BASE + NVIC_IRQ192_223_ENABLE_OFFSET)
#define NVIC_IRQ224_239_ENABLE          (ARMV7M_NVIC_BASE + NVIC_IRQ224_239_ENABLE_OFFSET)

#define NVIC_IRQ_CLEAR(n)               (ARMV7M_NVIC_BASE + NVIC_IRQ_CLEAR_OFFSET(n))
#define NVIC_IRQ0_31_CLEAR              (ARMV7M_NVIC_BASE + NVIC_IRQ0_31_CLEAR_OFFSET)
#define NVIC_IRQ32_63_CLEAR             (ARMV7M_NVIC_BASE + NVIC_IRQ32_63_CLEAR_OFFSET)
#define NVIC_IRQ64_95_CLEAR             (ARMV7M_NVIC_BASE + NVIC_IRQ64_95_CLEAR_OFFSET)
#define NVIC_IRQ96_127_CLEAR            (ARMV7M_NVIC_BASE + NVIC_IRQ96_127_CLEAR_OFFSET)
#define NVIC_IRQ128_159_CLEAR           (ARMV7M_NVIC_BASE + NVIC_IRQ128_159_CLEAR_OFFSET)
#define NVIC_IRQ160_191_CLEAR           (ARMV7M_NVIC_BASE + NVIC_IRQ160_191_CLEAR_OFFSET)
#define NVIC_IRQ192_223_CLEAR           (ARMV7M_NVIC_BASE + NVIC_IRQ192_223_CLEAR_OFFSET)
#define NVIC_IRQ224_239_CLEAR           (ARMV7M_NVIC_BASE + NVIC_IRQ224_239_CLEAR_OFFSET)

#define NVIC_IRQ_PEND(n)                (ARMV7M_NVIC_BASE + NVIC_IRQ_PEND_OFFSET(n))
#define NVIC_IRQ0_31_PEND               (ARMV7M_NVIC_BASE + NVIC_IRQ0_31_PEND_OFFSET)
#define NVIC_IRQ32_63_PEND              (ARMV7M_NVIC_BASE + NVIC_IRQ32_63_PEND_OFFSET)
#define NVIC_IRQ64_95_PEND              (ARMV7M_NVIC_BASE + NVIC_IRQ64_95_PEND_OFFSET)
#define NVIC_IRQ96_127_PEND             (ARMV7M_NVIC_BASE + NVIC_IRQ96_127_PEND_OFFSET)
#define NVIC_IRQ128_159_PEND            (ARMV7M_NVIC_BASE + NVIC_IRQ128_159_PEND_OFFSET)
#define NVIC_IRQ160_191_PEND            (ARMV7M_NVIC_BASE + NVIC_IRQ160_191_PEND_OFFSET)
#define NVIC_IRQ192_223_PEND            (ARMV7M_NVIC_BASE + NVIC_IRQ192_223_PEND_OFFSET)
#define NVIC_IRQ224_239_PEND            (ARMV7M_NVIC_BASE + NVIC_IRQ224_239_PEND_OFFSET)

#define NVIC_IRQ_CLRPEND(n)             (ARMV7M_NVIC_BASE + NVIC_IRQ_CLRPEND_OFFSET(n))
#define NVIC_IRQ0_31_CLRPEND            (ARMV7M_NVIC_BASE + NVIC_IRQ0_31_CLRPEND_OFFSET)
#define NVIC_IRQ32_63_CLRPEND           (ARMV7M_NVIC_BASE + NVIC_IRQ32_63_CLRPEND_OFFSET)
#define NVIC_IRQ64_95_CLRPEND           (ARMV7M_NVIC_BASE + NVIC_IRQ64_95_CLRPEND_OFFSET)
#define NVIC_IRQ96_127_CLRPEND          (ARMV7M_NVIC_BASE + NVIC_IRQ96_127_CLRPEND_OFFSET)
#define NVIC_IRQ128_159_CLRPEND         (ARMV7M_NVIC_BASE + NVIC_IRQ128_159_CLRPEND_OFFSET)
#define NVIC_IRQ160_191_CLRPEND         (ARMV7M_NVIC_BASE + NVIC_IRQ160_191_CLRPEND_OFFSET)
#define NVIC_IRQ192_223_CLRPEND         (ARMV7M_NVIC_BASE + NVIC_IRQ192_223_CLRPEND_OFFSET)
#define NVIC_IRQ224_239_CLRPEND         (ARMV7M_NVIC_BASE + NVIC_IRQ224_239_CLRPEND_OFFSET)

#define NVIC_IRQ_ACTIVE(n)              (ARMV7M_NVIC_BASE + NVIC_IRQ_ACTIVE_OFFSET(n))
#define NVIC_IRQ0_31_ACTIVE             (ARMV7M_NVIC_BASE + NVIC_IRQ0_31_ACTIVE_OFFSET)
#define NVIC_IRQ32_63_ACTIVE            (ARMV7M_NVIC_BASE + NVIC_IRQ32_63_ACTIVE_OFFSET)
#define NVIC_IRQ64_95_ACTIVE            (ARMV7M_NVIC_BASE + NVIC_IRQ64_95_ACTIVE_OFFSET)
#define NVIC_IRQ96_127_ACTIVE           (ARMV7M_NVIC_BASE + NVIC_IRQ96_127_ACTIVE_OFFSET)
#define NVIC_IRQ128_159_ACTIVE          (ARMV7M_NVIC_BASE + NVIC_IRQ128_159_ACTIVE_OFFSET)
#define NVIC_IRQ160_191_ACTIVE          (ARMV7M_NVIC_BASE + NVIC_IRQ160_191_ACTIVE_OFFSET)
#define NVIC_IRQ192_223_ACTIVE          (ARMV7M_NVIC_BASE + NVIC_IRQ192_223_ACTIVE_OFFSET)
#define NVIC_IRQ224_239_ACTIVE          (ARMV7M_NVIC_BASE + NVIC_IRQ224_239_ACTIVE_OFFSET)

#define NVIC_IRQ_PRIORITY(n)            (ARMV7M_NVIC_BASE + NVIC_IRQ_PRIORITY_OFFSET(n))
#define NVIC_IRQ0_3_PRIORITY            (ARMV7M_NVIC_BASE + NVIC_IRQ0_3_PRIORITY_OFFSET)
#define NVIC_IRQ4_7_PRIORITY            (ARMV7M_NVIC_BASE + NVIC_IRQ4_7_PRIORITY_OFFSET)
#define NVIC_IRQ8_11_PRIORITY           (ARMV7M_NVIC_BASE + NVIC_IRQ8_11_PRIORITY_OFFSET)
#define NVIC_IRQ12_15_PRIORITY          (ARMV7M_NVIC_BASE + NVIC_IRQ12_15_PRIORITY_OFFSET)
#define NVIC_IRQ16_19_PRIORITY          (ARMV7M_NVIC_BASE + NVIC_IRQ16_19_PRIORITY_OFFSET)
#define NVIC_IRQ20_23_PRIORITY          (ARMV7M_NVIC_BASE + NVIC_IRQ20_23_PRIORITY_OFFSET)
#define NVIC_IRQ24_27_PRIORITY          (ARMV7M_NVIC_BASE + NVIC_IRQ24_27_PRIORITY_OFFSET)
#define NVIC_IRQ28_31_PRIORITY          (ARMV7M_NVIC_BASE + NVIC_IRQ28_31_PRIORITY_OFFSET)
#define NVIC_IRQ32_35_PRIORITY          (ARMV7M_NVIC_BASE + NVIC_IRQ32_35_PRIORITY_OFFSET)
#define NVIC_IRQ36_39_PRIORITY          (ARMV7M_NVIC_BASE + NVIC_IRQ36_39_PRIORITY_OFFSET)
#define NVIC_IRQ40_43_PRIORITY          (ARMV7M_NVIC_BASE + NVIC_IRQ40_43_PRIORITY_OFFSET)
#define NVIC_IRQ44_47_PRIORITY          (ARMV7M_NVIC_BASE + NVIC_IRQ44_47_PRIORITY_OFFSET)
#define NVIC_IRQ48_51_PRIORITY          (ARMV7M_NVIC_BASE + NVIC_IRQ48_51_PRIORITY_OFFSET)
#define NVIC_IRQ52_55_PRIORITY          (ARMV7M_NVIC_BASE + NVIC_IRQ52_55_PRIORITY_OFFSET)
#define NVIC_IRQ56_59_PRIORITY          (ARMV7M_NVIC_BASE + NVIC_IRQ56_59_PRIORITY_OFFSET)
#define NVIC_IRQ60_63_PRIORITY          (ARMV7M_NVIC_BASE + NVIC_IRQ60_63_PRIORITY_OFFSET)
#define NVIC_IRQ64_67_PRIORITY          (ARMV7M_NVIC_BASE + NVIC_IRQ64_67_PRIORITY_OFFSET)
#define NVIC_IRQ68_71_PRIORITY          (ARMV7M_NVIC_BASE + NVIC_IRQ68_71_PRIORITY_OFFSET)
#define NVIC_IRQ72_75_PRIORITY          (ARMV7M_NVIC_BASE + NVIC_IRQ72_75_PRIORITY_OFFSET)
#define NVIC_IRQ76_79_PRIORITY          (ARMV7M_NVIC_BASE + NVIC_IRQ76_79_PRIORITY_OFFSET)
#define NVIC_IRQ80_83_PRIORITY          (ARMV7M_NVIC_BASE + NVIC_IRQ80_83_PRIORITY_OFFSET)
#define NVIC_IRQ84_87_PRIORITY          (ARMV7M_NVIC_BASE + NVIC_IRQ84_87_PRIORITY_OFFSET)
#define NVIC_IRQ88_91_PRIORITY          (ARMV7M_NVIC_BASE + NVIC_IRQ88_91_PRIORITY_OFFSET)
#define NVIC_IRQ92_95_PRIORITY          (ARMV7M_NVIC_BASE + NVIC_IRQ92_95_PRIORITY_OFFSET)
#define NVIC_IRQ96_99_PRIORITY          (ARMV7M_NVIC_BASE + NVIC_IRQ96_99_PRIORITY_OFFSET)
#define NVIC_IRQ100_103_PRIORITY        (ARMV7M_NVIC_BASE + NVIC_IRQ100_103_PRIORITY_OFFSET)
#define NVIC_IRQ104_107_PRIORITY        (ARMV7M_NVIC_BASE + NVIC_IRQ104_107_PRIORITY_OFFSET)
#define NVIC_IRQ108_111_PRIORITY        (ARMV7M_NVIC_BASE + NVIC_IRQ108_111_PRIORITY_OFFSET)
#define NVIC_IRQ112_115_PRIORITY        (ARMV7M_NVIC_BASE + NVIC_IRQ112_115_PRIORITY_OFFSET)
#define NVIC_IRQ116_119_PRIORITY        (ARMV7M_NVIC_BASE + NVIC_IRQ116_119_PRIORITY_OFFSET)
#define NVIC_IRQ120_123_PRIORITY        (ARMV7M_NVIC_BASE + NVIC_IRQ120_123_PRIORITY_OFFSET)
#define NVIC_IRQ124_127_PRIORITY        (ARMV7M_NVIC_BASE + NVIC_IRQ124_127_PRIORITY_OFFSET)
#define NVIC_IRQ128_131_PRIORITY        (ARMV7M_NVIC_BASE + NVIC_IRQ128_131_PRIORITY_OFFSET)
#define NVIC_IRQ132_135_PRIORITY        (ARMV7M_NVIC_BASE + NVIC_IRQ132_135_PRIORITY_OFFSET)
#define NVIC_IRQ136_139_PRIORITY        (ARMV7M_NVIC_BASE + NVIC_IRQ136_139_PRIORITY_OFFSET)
#define NVIC_IRQ140_143_PRIORITY        (ARMV7M_NVIC_BASE + NVIC_IRQ140_143_PRIORITY_OFFSET)
#define NVIC_IRQ144_147_PRIORITY        (ARMV7M_NVIC_BASE + NVIC_IRQ144_147_PRIORITY_OFFSET)
#define NVIC_IRQ148_151_PRIORITY        (ARMV7M_NVIC_BASE + NVIC_IRQ148_151_PRIORITY_OFFSET)
#define NVIC_IRQ152_155_PRIORITY        (ARMV7M_NVIC_BASE + NVIC_IRQ152_155_PRIORITY_OFFSET)
#define NVIC_IRQ156_159_PRIORITY        (ARMV7M_NVIC_BASE + NVIC_IRQ156_159_PRIORITY_OFFSET)
#define NVIC_IRQ160_163_PRIORITY        (ARMV7M_NVIC_BASE + NVIC_IRQ160_163_PRIORITY_OFFSET)
#define NVIC_IRQ164_167_PRIORITY        (ARMV7M_NVIC_BASE + NVIC_IRQ164_167_PRIORITY_OFFSET)
#define NVIC_IRQ168_171_PRIORITY        (ARMV7M_NVIC_BASE + NVIC_IRQ168_171_PRIORITY_OFFSET)
#define NVIC_IRQ172_175_PRIORITY        (ARMV7M_NVIC_BASE + NVIC_IRQ172_175_PRIORITY_OFFSET)
#define NVIC_IRQ176_179_PRIORITY        (ARMV7M_NVIC_BASE + NVIC_IRQ176_179_PRIORITY_OFFSET)
#define NVIC_IRQ180_183_PRIORITY        (ARMV7M_NVIC_BASE + NVIC_IRQ180_183_PRIORITY_OFFSET)
#define NVIC_IRQ184_187_PRIORITY        (ARMV7M_NVIC_BASE + NVIC_IRQ184_187_PRIORITY_OFFSET)
#define NVIC_IRQ188_191_PRIORITY        (ARMV7M_NVIC_BASE + NVIC_IRQ188_191_PRIORITY_OFFSET)
#define NVIC_IRQ192_195_PRIORITY        (ARMV7M_NVIC_BASE + NVIC_IRQ192_195_PRIORITY_OFFSET)
#define NVIC_IRQ196_199_PRIORITY        (ARMV7M_NVIC_BASE + NVIC_IRQ196_199_PRIORITY_OFFSET)
#define NVIC_IRQ200_203_PRIORITY        (ARMV7M_NVIC_BASE + NVIC_IRQ200_203_PRIORITY_OFFSET)
#define NVIC_IRQ204_207_PRIORITY        (ARMV7M_NVIC_BASE + NVIC_IRQ204_207_PRIORITY_OFFSET)
#define NVIC_IRQ208_211_PRIORITY        (ARMV7M_NVIC_BASE + NVIC_IRQ208_211_PRIORITY_OFFSET)
#define NVIC_IRQ212_215_PRIORITY        (ARMV7M_NVIC_BASE + NVIC_IRQ212_215_PRIORITY_OFFSET)
#define NVIC_IRQ216_219_PRIORITY        (ARMV7M_NVIC_BASE + NVIC_IRQ216_219_PRIORITY_OFFSET)
#define NVIC_IRQ220_223_PRIORITY        (ARMV7M_NVIC_BASE + NVIC_IRQ220_223_PRIORITY_OFFSET)
#define NVIC_IRQ224_227_PRIORITY        (ARMV7M_NVIC_BASE + NVIC_IRQ224_227_PRIORITY_OFFSET)
#define NVIC_IRQ228_231_PRIORITY        (ARMV7M_NVIC_BASE + NVIC_IRQ228_231_PRIORITY_OFFSET)
#define NVIC_IRQ232_235_PRIORITY        (ARMV7M_NVIC_BASE + NVIC_IRQ232_235_PRIORITY_OFFSET)

#define NVIC_CPUID_BASE                 (ARMV7M_NVIC_BASE + NVIC_CPUID_BASE_OFFSET)
#define NVIC_INTCTRL                    (ARMV7M_NVIC_BASE + NVIC_INTCTRL_OFFSET)
#define NVIC_VECTAB                     (ARMV7M_NVIC_BASE + NVIC_VECTAB_OFFSET)
#define NVIC_AIRCR                      (ARMV7M_NVIC_BASE + NVIC_AIRCR_OFFSET)
#define NVIC_SYSCON                     (ARMV7M_NVIC_BASE + NVIC_SYSCON_OFFSET)
#define NVIC_CFGCON                     (ARMV7M_NVIC_BASE + NVIC_CFGCON_OFFSET)
#define NVIC_SYSH_PRIORITY(n)           (ARMV7M_NVIC_BASE + NVIC_SYSH_PRIORITY_OFFSET(n))
#define NVIC_SYSH4_7_PRIORITY           (ARMV7M_NVIC_BASE + NVIC_SYSH4_7_PRIORITY_OFFSET)
#define NVIC_SYSH8_11_PRIORITY          (ARMV7M_NVIC_BASE + NVIC_SYSH8_11_PRIORITY_OFFSET)
#define NVIC_SYSH12_15_PRIORITY         (ARMV7M_NVIC_BASE + NVIC_SYSH12_15_PRIORITY_OFFSET)
#define NVIC_SYSHCON                    (ARMV7M_NVIC_BASE + NVIC_SYSHCON_OFFSET)
#define NVIC_CFAULTS                    (ARMV7M_NVIC_BASE + NVIC_CFAULTS_OFFSET)
#define NVIC_HFAULTS                    (ARMV7M_NVIC_BASE + NVIC_HFAULTS_OFFSET)
#define NVIC_DFAULTS                    (ARMV7M_NVIC_BASE + NVIC_DFAULTS_OFFSET)
#define NVIC_MEMMANAGE_ADDR             (ARMV7M_NVIC_BASE + NVIC_MEMMANAGE_ADDR_OFFSET)
#define NVIC_BFAULT_ADDR                (ARMV7M_NVIC_BASE + NVIC_BFAULT_ADDR_OFFSET)
#define NVIC_AFAULTS                    (ARMV7M_NVIC_BASE + NVIC_AFAULTS_OFFSET)
#define NVIC_PFR0                       (ARMV7M_NVIC_BASE + NVIC_PFR0_OFFSET)
#define NVIC_PFR1                       (ARMV7M_NVIC_BASE + NVIC_PFR1_OFFSET)
#define NVIC_DFR0                       (ARMV7M_NVIC_BASE + NVIC_DFR0_OFFSET)
#define NVIC_AFR0                       (ARMV7M_NVIC_BASE + NVIC_AFR0_OFFSET)
#define NVIC_MMFR0                      (ARMV7M_NVIC_BASE + NVIC_MMFR0_OFFSET)
#define NVIC_MMFR1                      (ARMV7M_NVIC_BASE + NVIC_MMFR1_OFFSET)
#define NVIC_MMFR2                      (ARMV7M_NVIC_BASE + NVIC_MMFR2_OFFSET)
#define NVIC_MMFR3                      (ARMV7M_NVIC_BASE + NVIC_MMFR3_OFFSET)
#define NVIC_ISAR0                      (ARMV7M_NVIC_BASE + NVIC_ISAR0_OFFSET)
#define NVIC_ISAR1                      (ARMV7M_NVIC_BASE + NVIC_ISAR1_OFFSET)
#define NVIC_ISAR2                      (ARMV7M_NVIC_BASE + NVIC_ISAR2_OFFSET)
#define NVIC_ISAR3                      (ARMV7M_NVIC_BASE + NVIC_ISAR3_OFFSET)
#define NVIC_ISAR4                      (ARMV7M_NVIC_BASE + NVIC_ISAR4_OFFSET)
#define NVIC_CLIDR                      (ARMV7M_NVIC_BASE + NVIC_CLIDR_OFFSET)
#define NVIC_CTR                        (ARMV7M_NVIC_BASE + NVIC_CTR_OFFSET)
#define NVIC_CCSIDR                     (ARMV7M_NVIC_BASE + NVIC_CCSIDR_OFFSET)
#define NVIC_CSSELR                     (ARMV7M_NVIC_BASE + NVIC_CSSELR_OFFSET)
#define NVIC_CPACR                      (ARMV7M_NVIC_BASE + NVIC_CPACR_OFFSET)
#define NVIC_DHCSR                      (ARMV7M_NVIC_BASE + NVIC_DHCSR_OFFSET)
#define NVIC_DCRSR                      (ARMV7M_NVIC_BASE + NVIC_DCRSR_OFFSET)
#define NVIC_DCRDR                      (ARMV7M_NVIC_BASE + NVIC_DCRDR_OFFSET)
#define NVIC_DEMCR                      (ARMV7M_NVIC_BASE + NVIC_DEMCR_OFFSET)
#define NVIC_STIR                       (ARMV7M_NVIC_BASE + NVIC_STIR_OFFSET)
#define NVIC_FPCCR                      (ARMV7M_NVIC_BASE + NVIC_FPCCR_OFFSET)
#define NVIC_FPCAR                      (ARMV8M_NVIC_BASE + NVIC_FPCAR_OFFSET)
#define NVIC_FPDSCR                     (ARMV8M_NVIC_BASE + NVIC_FPDSCR_OFFSET)
#define NVIC_MVFR0                      (ARMV8M_NVIC_BASE + NVIC_MVFR0_OFFSET)
#define NVIC_MVFR1                      (ARMV8M_NVIC_BASE + NVIC_MVFR1_OFFSET)
#define NVIC_MVFR2                      (ARMV8M_NVIC_BASE + NVIC_MVFR2_OFFSET)
#define NVIC_ICIALLU                    (ARMV7M_NVIC_BASE + NVIC_ICIALLU_OFFSET)
#define NVIC_ICIMVAU                    (ARMV7M_NVIC_BASE + NVIC_ICIMVAU_OFFSET)
#define NVIC_DCIMVAU                    (ARMV7M_NVIC_BASE + NVIC_DCIMVAU_OFFSET)
#define NVIC_DCIMVAC                    (ARMV7M_NVIC_BASE + NVIC_DCIMVAC_OFFSET)
#define NVIC_DCISW                      (ARMV7M_NVIC_BASE + NVIC_DCISW_OFFSET)
#define NVIC_DCCMVAU                    (ARMV7M_NVIC_BASE + NVIC_DCCMVAU_OFFSET)
#define NVIC_DCCMVAC                    (ARMV7M_NVIC_BASE + NVIC_DCCMVAC_OFFSET)
#define NVIC_DCCSW                      (ARMV7M_NVIC_BASE + NVIC_DCCSW_OFFSET)
#define NVIC_DCCIMVAC                   (ARMV7M_NVIC_BASE + NVIC_DCCIMVAC_OFFSET)
#define NVIC_DCCISW                     (ARMV7M_NVIC_BASE + NVIC_DCCISW_OFFSET)
#define NVIC_BPIALL                     (ARMV7M_NVIC_BASE + NVIC_BPIALL_OFFSET)
#define NVIC_ITCMCR                     (ARMV7M_NVIC_BASE + NVIC_ITCMCR_OFFSET)
#define NVIC_DTCMCR                     (ARMV7M_NVIC_BASE + NVIC_DTCMCR_OFFSET)
#define NVIC_AHBPCR                     (ARMV7M_NVIC_BASE + NVIC_AHBPCR_OFFSET)
#define NVIC_CACR                       (ARMV7M_NVIC_BASE + NVIC_CACR_OFFSET)
#define NVIC_AHBSCR                     (ARMV7M_NVIC_BASE + NVIC_AHBSCR_OFFSET)
#define NVIC_ABFSR                      (ARMV7M_NVIC_BASE + NVIC_ABFSR_OFFSET)
#define NVIC_PID4                       (ARMV7M_NVIC_BASE + NVIC_PID4_OFFSET)
#define NVIC_PID5                       (ARMV7M_NVIC_BASE + NVIC_PID5_OFFSET)
#define NVIC_PID6                       (ARMV7M_NVIC_BASE + NVIC_PID6_OFFSET)
#define NVIC_PID7                       (ARMV7M_NVIC_BASE + NVIC_PID7_OFFSET)
#define NVIC_PID0                       (ARMV7M_NVIC_BASE + NVIC_PID0_OFFSET)
#define NVIC_PID1                       (ARMV7M_NVIC_BASE + NVIC_PID1_OFFSET)
#define NVIC_PID2                       (ARMV7M_NVIC_BASE + NVIC_PID2_OFFSET)
#define NVIC_PID3                       (ARMV7M_NVIC_BASE + NVIC_PID3_OFFSET)
#define NVIC_CID0                       (ARMV7M_NVIC_BASE + NVIC_CID0_OFFSET)
#define NVIC_CID1                       (ARMV7M_NVIC_BASE + NVIC_CID1_OFFSET)
#define NVIC_CID2                       (ARMV7M_NVIC_BASE + NVIC_CID2_OFFSET)
#define NVIC_CID3                       (ARMV7M_NVIC_BASE + NVIC_CID3_OFFSET)

/* NVIC register bit definitions ********************************************/

/* Interrupt controller type (INCTCTL_TYPE) */

#define NVIC_ICTR_INTLINESNUM_SHIFT     0    /* Bits 0-3: Number of interrupt inputs / 32 - 1 */
#define NVIC_ICTR_INTLINESNUM_MASK      (15 << NVIC_ICTR_INTLINESNUM_SHIFT)

/* SysTick control and status register (SYSTICK_CTRL) */

#define NVIC_SYSTICK_CTRL_ENABLE        (1 << 0)  /* Bit 0:  Enable */
#define NVIC_SYSTICK_CTRL_TICKINT       (1 << 1)  /* Bit 1:  Tick interrupt */
#define NVIC_SYSTICK_CTRL_CLKSOURCE     (1 << 2)  /* Bit 2:  Clock source */
#define NVIC_SYSTICK_CTRL_COUNTFLAG     (1 << 16) /* Bit 16: Count Flag */

/* SysTick reload value register (SYSTICK_RELOAD) */

#define NVIC_SYSTICK_RELOAD_SHIFT       0         /* Bits 23-0: Timer reload value */
#define NVIC_SYSTICK_RELOAD_MASK        (0x00ffffff << NVIC_SYSTICK_RELOAD_SHIFT)

/* SysTick current value register (SYSTICK_CURRENT) */

#define NVIC_SYSTICK_CURRENT_SHIFT      0         /* Bits 23-0: Timer current value */
#define NVIC_SYSTICK_CURRENT_MASK       (0x00ffffff << NVIC_SYSTICK_RELOAD_SHIFT)

/* SysTick calibration value register (SYSTICK_CALIB) */

#define NVIC_SYSTICK_CALIB_TENMS_SHIFT  0         /* Bits 23-0: Calibration value */
#define NVIC_SYSTICK_CALIB_TENMS_MASK   (0x00ffffff << NVIC_SYSTICK_CALIB_TENMS_SHIFT)
#define NVIC_SYSTICK_CALIB_SKEW         (1 << 30) /* Bit 30: Calibration value inexact */
#define NVIC_SYSTICK_CALIB_NOREF        (1 << 31) /* Bit 31: No external reference clock */

/* Interrupt control state register (INTCTRL) */

#define NVIC_INTCTRL_NMIPENDSET         (1 << 31) /* Bit 31: Set pending NMI bit */
#define NVIC_INTCTRL_PENDSVSET          (1 << 28) /* Bit 28: Set pending PendSV bit */
#define NVIC_INTCTRL_PENDSVCLR          (1 << 27) /* Bit 27: Clear pending PendSV bit */
#define NVIC_INTCTRL_PENDSTSET          (1 << 26) /* Bit 26: Set pending SysTick bit */
#define NVIC_INTCTRL_PENDSTCLR          (1 << 25) /* Bit 25: Clear pending SysTick bit */
#define NVIC_INTCTRL_ISPREEMPOT         (1 << 23) /* Bit 23: Pending active next cycle */
#define NVIC_INTCTRL_ISRPENDING         (1 << 22) /* Bit 22: Interrupt pending flag */
#define NVIC_INTCTRL_VECTPENDING_SHIFT  12        /* Bits 21-12: Pending ISR number field */
#define NVIC_INTCTRL_VECTPENDING_MASK   (0x3ff << NVIC_INTCTRL_VECTPENDING_SHIFT)
#define NVIC_INTCTRL_RETTOBASE          (1 << 11) /* Bit 11: no other exceptions pending */
#define NVIC_INTCTRL_VECTACTIVE_SHIFT   0         /* Bits 8-0: Active ISR number */
#define NVIC_INTCTRL_VECTACTIVE_MASK    (0x1ff << NVIC_INTCTRL_VECTACTIVE_SHIFT)

/* System control register (SYSCON) */

                                                  /* Bit 0:  Reserved */
#define NVIC_SYSCON_SLEEPONEXIT         (1 << 1)  /* Bit 1:  Sleep-on-exit (returning from Handler to Thread mode) */
#define NVIC_SYSCON_SLEEPDEEP           (1 << 2)  /* Bit 2: Use deep sleep in low power mode */
                                                  /* Bit 3:  Reserved */
#define NVIC_SYSCON_SEVONPEND           (1 << 4)  /* Bit 4: Send Event on Pending bit */
                                                  /* Bits 5-31: Reserved */

/* Configuration control register (CFGCON) */

#define NVIC_CFGCON_NONBASETHRDENA      (1 << 0)  /* Bit 0: How processor enters thread mode */
#define NVIC_CFGCON_USERSETMPEND        (1 << 1)  /* Bit 1: Enables unprivileged access to STIR */
#define NVIC_CFGCON_UNALIGNTRP          (1 << 3)  /* Bit 3: Enables unaligned access traps */
#define NVIC_CFGCON_DIV0TRP             (1 << 4)  /* Bit 4: Enables fault on divide-by-zero */
#define NVIC_CFGCON_BFHFNMIGN           (1 << 8)  /* Bit 8: Disables data bus faults */
#define NVIC_CFGCON_STKALIGN            (1 << 9)  /* Bit 9: Indicates stack alignment on exception */
                                                  /* Cortex-M7: */
#define NVIC_CFGCON_DC                  (1 << 16) /* Bit 16: Data cache enable */
#define NVIC_CFGCON_IC                  (1 << 17) /* Bit 17: Instruction cache enable */
#define NVIC_CFGCON_BP                  (1 << 18) /* Bit 18: Branch prediction enable */

/* System handler 4-7 priority register */

#define NVIC_SYSH_PRIORITY_PR4_SHIFT    0
#define NVIC_SYSH_PRIORITY_PR4_MASK     (0xff << NVIC_SYSH_PRIORITY_PR4_SHIFT)
#define NVIC_SYSH_PRIORITY_PR5_SHIFT    8
#define NVIC_SYSH_PRIORITY_PR5_MASK     (0xff << NVIC_SYSH_PRIORITY_PR5_SHIFT)
#define NVIC_SYSH_PRIORITY_PR6_SHIFT    16
#define NVIC_SYSH_PRIORITY_PR6_MASK     (0xff << NVIC_SYSH_PRIORITY_PR6_SHIFT)
#define NVIC_SYSH_PRIORITY_PR7_SHIFT    24
#define NVIC_SYSH_PRIORITY_PR7_MASK     (0xff << NVIC_SYSH_PRIORITY_PR7_SHIFT)

/* System handler 8-11 priority register */

#define NVIC_SYSH_PRIORITY_PR8_SHIFT    0
#define NVIC_SYSH_PRIORITY_PR8_MASK     (0xff << NVIC_SYSH_PRIORITY_PR8_SHIFT)
#define NVIC_SYSH_PRIORITY_PR9_SHIFT    8
#define NVIC_SYSH_PRIORITY_PR9_MASK     (0xff << NVIC_SYSH_PRIORITY_PR9_SHIFT)
#define NVIC_SYSH_PRIORITY_PR10_SHIFT   16
#define NVIC_SYSH_PRIORITY_PR10_MASK    (0xff << NVIC_SYSH_PRIORITY_PR10_SHIFT)
#define NVIC_SYSH_PRIORITY_PR11_SHIFT   24
#define NVIC_SYSH_PRIORITY_PR11_MASK    (0xff << NVIC_SYSH_PRIORITY_PR11_SHIFT)

/* System handler 12-15 priority register */

#define NVIC_SYSH_PRIORITY_PR12_SHIFT   0
#define NVIC_SYSH_PRIORITY_PR12_MASK    (0xff << NVIC_SYSH_PRIORITY_PR12_SHIFT)
#define NVIC_SYSH_PRIORITY_PR13_SHIFT   8
#define NVIC_SYSH_PRIORITY_PR13_MASK    (0xff << NVIC_SYSH_PRIORITY_PR13_SHIFT)
#define NVIC_SYSH_PRIORITY_PR14_SHIFT   16
#define NVIC_SYSH_PRIORITY_PR14_MASK    (0xff << NVIC_SYSH_PRIORITY_PR14_SHIFT)
#define NVIC_SYSH_PRIORITY_PR15_SHIFT   24
#define NVIC_SYSH_PRIORITY_PR15_MASK    (0xff << NVIC_SYSH_PRIORITY_PR15_SHIFT)

/* Application Interrupt and Reset Control Register (AIRCR) */

#define NVIC_AIRCR_VECTRESET            (1 << 0)  /* Bit 0:  VECTRESET */
#define NVIC_AIRCR_VECTCLRACTIVE        (1 << 1)  /* Bit 1:  Reserved for debug use */
#define NVIC_AIRCR_SYSRESETREQ          (1 << 2)  /* Bit 2:  System reset */
                                                  /* Bits 2-7:  Reserved */
#define NVIC_AIRCR_PRIGROUP_SHIFT       (8)       /* Bits 8-14: PRIGROUP */
#define NVIC_AIRCR_PRIGROUP_MASK        (7 << NVIC_AIRCR_PRIGROUP_SHIFT)
#define NVIC_AIRCR_ENDIANNESS           (1 << 15) /* Bit 15: 1=Big endian */
#define NVIC_AIRCR_VECTKEY_SHIFT        (16)      /* Bits 16-31: VECTKEY */
#define NVIC_AIRCR_VECTKEY_MASK         (0xffff << NVIC_AIRCR_VECTKEY_SHIFT)
#define NVIC_AIRCR_VECTKEY              (0x05fa << NVIC_AIRCR_VECTKEY_SHIFT)
#define NVIC_AIRCR_VECTKEYSTAT_SHIFT    (16)      /* Bits 16-31: VECTKEYSTAT */
#define NVIC_AIRCR_VECTKEYSTAT_MASK     (0xffff << NVIC_AIRCR_VECTKEYSTAT_SHIFT)
#define NVIC_AIRCR_VECTKEYSTAT          (0xfa05 << NVIC_AIRCR_VECTKEYSTAT_SHIFT)

/* System handler control and state register (SYSHCON) */

#define NVIC_SYSHCON_MEMFAULTACT        (1 << 0)  /* Bit 0:  MemManage is active */
#define NVIC_SYSHCON_BUSFAULTACT        (1 << 1)  /* Bit 1:  BusFault is active */
#define NVIC_SYSHCON_USGFAULTACT        (1 << 3)  /* Bit 3:  UsageFault is active */
#define NVIC_SYSHCON_SVCALLACT          (1 << 7)  /* Bit 7:  SVCall is active */
#define NVIC_SYSHCON_MONITORACT         (1 << 8)  /* Bit 8:  Monitor is active */
#define NVIC_SYSHCON_PENDSVACT          (1 << 10) /* Bit 10: PendSV is active */
#define NVIC_SYSHCON_SYSTICKACT         (1 << 11) /* Bit 11: SysTick is active */
#define NVIC_SYSHCON_USGFAULTPENDED     (1 << 12) /* Bit 12: Usage fault is pended */
#define NVIC_SYSHCON_MEMFAULTPENDED     (1 << 13) /* Bit 13: MemManage is pended */
#define NVIC_SYSHCON_BUSFAULTPENDED     (1 << 14) /* Bit 14: BusFault is pended */
#define NVIC_SYSHCON_SVCALLPENDED       (1 << 15) /* Bit 15: SVCall is pended */
#define NVIC_SYSHCON_MEMFAULTENA        (1 << 16) /* Bit 16: MemFault enabled */
#define NVIC_SYSHCON_BUSFAULTENA        (1 << 17) /* Bit 17: BusFault enabled */
#define NVIC_SYSHCON_USGFAULTENA        (1 << 18) /* Bit 18: UsageFault enabled */

/* SCB Configurable Fault Status Register Definitions */

#define NVIC_CFAULTS_MEMFAULTSR_MASK    (0xff)    /* Memory Manage Fault Status Register Mask */
#define NVIC_CFAULTS_BUSFAULTSR_MASK    (0xff << 8)
                                                  /* Bus Fault Status Register Mask */
#define NVIC_CFAULTS_USGFAULTSR_MASK    (0xffff << 16)
                                                  /* Usage Fault Status Register Mask */

/* MemManage Fault Status Register
 * (part of SCB Configurable Fault Status Register)
 */

#define NVIC_CFAULTS_IACCVIOL           (1 << 0)  /* Bit 0:  IACCVIOL Mask */
#define NVIC_CFAULTS_DACCVIOL           (1 << 1)  /* Bit 1:  DACCVIOL Mask */
#define NVIC_CFAULTS_MUNSTKERR          (1 << 3)  /* Bit 3:  MUNSTKERR Mask */
#define NVIC_CFAULTS_MSTKERR            (1 << 4)  /* Bit 4:  MSTKERR Mask */
#define NVIC_CFAULTS_MLSPERR            (1 << 5)  /* Bit 5:  MLSPERR Mask */
#define NVIC_CFAULTS_MMARVALID          (1 << 7)  /* Bit 7:  MMARVALID Mask */

/* BusFault Status Register
 * (part of SCB Configurable Fault Status Register)
 */

#define NVIC_CFAULTS_IBUSERR            (1 << 8)  /* Bit 8:  IBUSERR Mask */
#define NVIC_CFAULTS_PRECISERR          (1 << 9)  /* Bit 9:  PRECISERR Mask */
#define NVIC_CFAULTS_IMPRECISERR        (1 << 10) /* Bit 10: IMPRECISERR Mask */
#define NVIC_CFAULTS_UNSTKERR           (1 << 11) /* Bit 11: UNSTKERR Mask */
#define NVIC_CFAULTS_STKERR             (1 << 12) /* Bit 12: STKERR Mask */
#define NVIC_CFAULTS_LSPERR             (1 << 13) /* Bit 13: LSPERR Mask */
#define NVIC_CFAULTS_BFARVALID          (1 << 15) /* Bit 15: BFARVALID Mask */

/* UsageFault Status Register
 * (part of SCB Configurable Fault Status Register)
 */

#define NVIC_CFAULTS_UNDEFINSTR         (1 << 16) /* Bit 16: UNDEFINSTR Mask */
#define NVIC_CFAULTS_INVSTATE           (1 << 17) /* Bit 17: INVSTATE Mask */
#define NVIC_CFAULTS_INVPC              (1 << 18) /* Bit 18: INVPC Mask */
#define NVIC_CFAULTS_NOCP               (1 << 19) /* Bit 19: NOCP Mask */
#define NVIC_CFAULTS_STKOF              (1 << 20) /* Bit 20: STKOF Mask */
#define NVIC_CFAULTS_UNALIGNED          (1 << 24) /* Bit 24: UNALIGNED Mask */
#define NVIC_CFAULTS_DIVBYZERO          (1 << 25) /* Bit 25: DIVBYZERO Mask */

/* SCB Hard Fault Status Register Definitions */

#define NVIC_HFAULTS_VECTTBL            (1 << 1)  /* Bit 1:  VECTTBL Mask */
#define NVIC_HFAULTS_FORCED             (1 << 30) /* Bit 30: FORCED Mask */
#define NVIC_HFAULTS_DEBUGEVT           (1 << 31) /* Bit 31: DEBUGEVT Mask */

/* Cache Level ID register (Cortex-M7) */

#define NVIC_CLIDR_L1CT_SHIFT           (0)      /* Bits 0-2: Level 1 cache type */
#define NVIC_CLIDR_L1CT_MASK            (7 << NVIC_CLIDR_L1CT_SHIFT)
#  define NVIC_CLIDR_L1CT_ICACHE        (1 << NVIC_CLIDR_LOC_SHIFT)
#  define NVIC_CLIDR_L1CT_DCACHE        (2 << NVIC_CLIDR_LOC_SHIFT)
#define NVIC_CLIDR_LOC_SHIFT            (24)      /* Bits 24-26: Level of Coherency */
#define NVIC_CLIDR_LOC_MASK             (7 << NVIC_CLIDR_LOC_SHIFT)
#  define NVIC_CLIDR_LOC_IMPLEMENTED    (1 << NVIC_CLIDR_LOC_SHIFT)
#  define NVIC_CLIDR_LOC_UNIMPLEMENTED  (0 << NVIC_CLIDR_LOC_SHIFT)
#define NVIC_CLIDR_LOUU_SHIFT           (27)      /* Bits 27-29: Level of Unification Uniprocessor */
#define NVIC_CLIDR_LOUU_MASK            (7 << NVIC_CLIDR_LOUU_SHIFT)
#  define NVIC_CLIDR_LOUU_IMPLEMENTED   (1 << NVIC_CLIDR_LOUU_SHIFT)
#  define NVIC_CLIDR_LOUU_UNIMPLEMENTED (0 << NVIC_CLIDR_LOUU_SHIFT)

/* Cache Type register (Cortex-M7) */

#define NVIC_CTR_IMINLINE_SHIFT         (0)       /* Bits 0-3: ImInLine */
#define NVIC_CTR_IMINLINE_MASK          (15 << NVIC_CTR_IMINLINE_SHIFT)
#define NVIC_CTR_DMINLINE_SHIFT         (16)      /* Bits 16-19: DmInLine */
#define NVIC_CTR_DMINLINE_MASK          (15 << NVIC_CTR_DMINLINE_SHIFT)
#define NVIC_CTR_ERG_SHIFT              (20)      /* Bits 20-23: ERG */
#define NVIC_CTR_ERG_MASK               (15 << NVIC_CTR_ERG_SHIFT)
#define NVIC_CTR_CWG_SHIFT              (24)      /* Bits 24-27: ERG */
#define NVIC_CTR_CWG_MASK               (15 << NVIC_CTR_CWG_SHIFT)
#define NVIC_CTR_FORMAT_SHIFT           (29)      /* Bits 29-31: Format */
#define NVIC_CTR_FORMAT_MASK            (7 << NVIC_CTR_FORMAT_SHIFT)

/* Cache Size ID Register (Cortex-M7) */

#define NVIC_CCSIDR_LINESIZE_SHIFT      (0)       /* Bits 0-2: Number of words in each cache line */
#define NVIC_CCSIDR_LINESIZE_MASK       (7 << NVIC_CCSIDR_LINESIZE_SHIFT)
#define NVIC_CCSIDR_ASSOCIATIVITY_SHIFT (3)       /* Bits 3-12: Number of ways - 1 */
#define NVIC_CCSIDR_ASSOCIATIVITY_MASK  (0x3ff << NVIC_CCSIDR_ASSOCIATIVITY_SHIFT)
#define NVIC_CCSIDR_NUMSETS_SHIFT       (13)      /* Bits 13-27: Number of sets - 1 */
#define NVIC_CCSIDR_NUMSETS_MASK        (0x7fff << NVIC_CCSIDR_NUMSETS_SHIFT)
#define NVIC_CCSIDR_WA_SHIFT            (1 << 28) /* Bits 28: Write Allocation support */
#define NVIC_CCSIDR_RA_SHIFT            (1 << 29) /* Bits 29: Read Allocation support */
#define NVIC_CCSIDR_WB_SHIFT            (1 << 30) /* Bits 30: Write-Back support */
#define NVIC_CCSIDR_WT_SHIFT            (1 << 31) /* Bits 31: Write-Through support */

/* Cache Size Selection Register (Cortex-M7) */

#define NVIC_CSSELR_IND                 (1 << 0)  /* Bit 0: Selects either instruction or data cache */
#  define NVIC_CSSELR_IND_ICACHE        (0 << 0)  /*   0=Instruction Cache */
#  define NVIC_CSSELR_IND_DCACHE        (1 << 0)  /*   1=Data Cache */

#define NVIC_CSSELR_LEVEL_SHIFT         (1)       /* Bit 1-3: Selects cache level */
#define NVIC_CSSELR_LEVEL_MASK          (7 << NVIC_CSSELR_LEVEL_SHIFT)
  #define NVIC_CSSELR_LEVEL_1           (0 << NVIC_CSSELR_LEVEL_SHIFT)

/* Coprocessor Access Control Register (CPACR) */

#define NVIC_CPACR_CP_SHIFT(n)          (2 * (n))
#define NVIC_CPACR_CP_MASK(n)           (3 << NVIC_CPACR_CP_SHIFT(n))
#  define NVIC_CPACR_CP_DENY(n)         (0 << NVIC_CPACR_CP_SHIFT(n))
#  define NVIC_CPACR_CP_PRIV(n)         (1 << NVIC_CPACR_CP_SHIFT(n))
#  define NVIC_CPACR_CP_FULL(n)         (3 << NVIC_CPACR_CP_SHIFT(n))

/* Debug Exception and Monitor Control Register (DEMCR) */

#define NVIC_DEMCR_VCCORERESET          (1 << 0)  /* Bit 0:  Reset Vector Catch */
#define NVIC_DEMCR_VCMMERR              (1 << 4)  /* Bit 4:  Debug trap on Memory Management faults */
#define NVIC_DEMCR_VCNOCPERR            (1 << 5)  /* Bit 5:  Debug trap on Usage Fault access to non-present coprocessor */
#define NVIC_DEMCR_VCCHKERR             (1 << 6)  /* Bit 6:  Debug trap on Usage Fault enabled checking errors */
#define NVIC_DEMCR_VCSTATERR            (1 << 7)  /* Bit 7:  Debug trap on Usage Fault state error */
#define NVIC_DEMCR_VCBUSERR             (1 << 8)  /* Bit 8:  Debug Trap on normal Bus error */
#define NVIC_DEMCR_VCINTERR             (1 << 9)  /* Bit 9:  Debug Trap on interrupt/exception service errors */
#define NVIC_DEMCR_VCHARDERR            (1 << 10) /* Bit 10: Debug trap on Hard Fault */
#define NVIC_DEMCR_MONEN                (1 << 16) /* Bit 16: Enable the debug monitor */
#define NVIC_DEMCR_MONPEND              (1 << 17) /* Bit 17: Pend the monitor to activate when priority permits */
#define NVIC_DEMCR_MONSTEP              (1 << 18) /* Bit 18: Steps the core */
#define NVIC_DEMCR_MONREQ               (1 << 19) /* Bit 19: Monitor wake-up mode */
#define NVIC_DEMCR_TRCENA               (1 << 24) /* Bit 24: Enable trace and debug blocks */

/*  Floating-Point Context Control Register (FPCCR) */

#define NVIC_FPCCR_LSPACT               (1 << 0)  /* Bit 0:  Lazy state preservation active */
#define NVIC_FPCCR_USER                 (1 << 1)  /* Bit 1:  User privilege */
#define NVIC_FPCCR_THREAD               (1 << 3)  /* Bit 3:  Thread mode */
#define NVIC_FPCCR_HFRDY                (1 << 4)  /* Bit 4:  HardFault ready */
#define NVIC_FPCCR_MMRDY                (1 << 5)  /* Bit 5:  MemManage ready */
#define NVIC_FPCCR_BFRDY                (1 << 6)  /* Bit 6:  BusFault ready */
#define NVIC_FPCCR_MONRDY               (1 << 8)  /* Bit 8:  DebugMonitor ready */
#define NVIC_FPCCR_SPLIMVIOL            (1 << 9)  /* Bit 9:  Stack pointer limit violation */
#define NVIC_FPCCR_UFRDY                (1 << 10) /* Bit 10: UsageFault ready */
#define NVIC_FPCCR_CLRONRET             (1 << 28) /* Bit 28: Clear on return */
#define NVIC_FPCCR_LSPEN                (1 << 30) /* Bit 30: Lazy state preservation enable */
#define NVIC_FPCCR_ASPEN                (1 << 31) /* Bit 31: Automatic state preservation enable */

/* Instruction Tightly-Coupled Memory Control Register (ITCMCR) */

/* Data Tightly-Coupled Memory Control Registers (DTCMCR */

#define NVIC_TCMCR_EN                   (1 << 0)  /* Bit 9:  TCM enable */
#define NVIC_TCMCR_RMW                  (1 << 1)  /* Bit 1:  Read-Modify-Write (RMW) enable */
#define NVIC_TCMCR_RETEN                (1 << 2)  /* Bit 2:  Retry phase enable */
#define NVIC_TCMCR_SZ_SHIFT             (3)       /* Bits 3-6: Size of the TCM */
#define NVIC_TCMCR_SZ_MASK              (15 << NVIC_TCMCR_SZ_SHIFT)
#  define NVIC_TCMCR_SZ_NONE            (0 << NVIC_TCMCR_SZ_SHIFT) /* No TCM implemented */
#  define NVIC_TCMCR_SZ_4KB             (3 << NVIC_TCMCR_SZ_SHIFT)
#  define NVIC_TCMCR_SZ_8KB             (4 << NVIC_TCMCR_SZ_SHIFT)
#  define NVIC_TCMCR_SZ_16KB            (5 << NVIC_TCMCR_SZ_SHIFT)
#  define NVIC_TCMCR_SZ_32KB            (6 << NVIC_TCMCR_SZ_SHIFT)
#  define NVIC_TCMCR_SZ_64KB            (7 << NVIC_TCMCR_SZ_SHIFT)
#  define NVIC_TCMCR_SZ_128KB           (8 << NVIC_TCMCR_SZ_SHIFT)
#  define NVIC_TCMCR_SZ_256KB           (9 << NVIC_TCMCR_SZ_SHIFT)
#  define NVIC_TCMCR_SZ_512KB           (10 << NVIC_TCMCR_SZ_SHIFT)
#  define NVIC_TCMCR_SZ_1MB             (11 << NVIC_TCMCR_SZ_SHIFT)
#  define NVIC_TCMCR_SZ_2MB             (12 << NVIC_TCMCR_SZ_SHIFT)
#  define NVIC_TCMCR_SZ_4MB             (13 << NVIC_TCMCR_SZ_SHIFT)
#  define NVIC_TCMCR_SZ_8MB             (14 << NVIC_TCMCR_SZ_SHIFT)
#  define NVIC_TCMCR_SZ_16MB            (15 << NVIC_TCMCR_SZ_SHIFT)

/* AHBP Control Register (AHBPCR, Cortex-M7) */

#define NVIC_AHBPCR_EN                  (1 << 0)  /* Bit 0: AHBP enable */
#define NVIC_AHBPCR_SZ_SHIFT            (1)       /* Bits 1-3: AHBP size */
#define NVIC_AHBPCR_SZ_MASK             (7 << NVIC_AHBPCR_SZ_SHIFT)
#  define NVIC_AHBPCR_SZ_DISABLED       (0 << NVIC_AHBPCR_SZ_SHIFT)
#  define NVIC_AHBPCR_SZ_64MB           (1 << NVIC_AHBPCR_SZ_SHIFT)
#  define NVIC_AHBPCR_SZ_128MB          (2 << NVIC_AHBPCR_SZ_SHIFT)
#  define NVIC_AHBPCR_SZ_256MB          (3 << NVIC_AHBPCR_SZ_SHIFT)
#  define NVIC_AHBPCR_SZ_512MB          (4 << NVIC_AHBPCR_SZ_SHIFT)

/* L1 Cache Control Register (CACR, Cortex-M7) */

#define NVIC_CACR_SIWT                  (1 << 0)  /* Bit 0:  Shared cacheable-is-WT for data cache */
#define NVIC_CACR_ECCDIS                (1 << 1)  /* Bit 1:  Enables ECC in the instruction and data cache */
#define NVIC_CACR_FORCEWT               (1 << 2)  /* Bit 2:  Enables Force Write-Through in the data cache */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_ARMV7_M_NVIC_H */
