/****************************************************************************
 * arch/risc-v/src/rv32m1/hardware/rv32m1_lpit.h
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

#ifndef __ARCH_RISCV_SRC_RV32M1_HARDWARE_RV32M1_LPIT_H
#define __ARCH_RISCV_SRC_RV32M1_HARDWARE_RV32M1_LPIT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define RV32M1_LPIT_VERID_OFFSET        0x0000 /* Version ID */
#define RV32M1_LPIT_PARAM_OFFSET        0x0004 /* Parameter */
#define RV32M1_LPIT_MCR_OFFSET          0x0008 /* Module Control */
#define RV32M1_LPIT_MSR_OFFSET          0x000c /* Module Status Register */
#define RV32M1_LPIT_MIER_OFFSET         0x0010 /* Moduel Interrupt Enable */
#define RV32M1_LPIT_SETTEN_OFFSET       0x0014 /* Set Timer Enable */
#define RV32M1_LPIT_CLRTEN_OFFSET       0x0018 /* Clear Timer Enable */
#define RV32M1_LPIT_TVAL0_OFFSET        0x0020 /* Timer Channel 0 Value */
#define RV32M1_LPIT_CVAL0_OFFSET        0x0024 /* Current Timer Channel 0 Value */
#define RV32M1_LPIT_TCTRL0_OFFSET       0x0028 /* Timer Channel 0 Control */
#define RV32M1_LPIT_TVAL1_OFFSET        0x0030 /* Timer Channel 1 Value */
#define RV32M1_LPIT_CVAL1_OFFSET        0x0034 /* Current Timer Channel 1 Value */
#define RV32M1_LPIT_TCTRL1_OFFSET       0x0048 /* Timer Channel 1 Control */
#define RV32M1_LPIT_TVAL2_OFFSET        0x0040 /* Timer Channel 2 Value */
#define RV32M1_LPIT_CVAL2_OFFSET        0x0044 /* Current Timer Channel 2 Value */
#define RV32M1_LPIT_TCTRL2_OFFSET       0x0048 /* Timer Channel 2 Control */
#define RV32M1_LPIT_TVAL3_OFFSET        0x0050 /* Timer Channel 3 Value */
#define RV32M1_LPIT_CVAL3_OFFSET        0x0054 /* Current Timer Channel 3 Value */
#define RV32M1_LPIT_TCTRL3_OFFSET       0x0058 /* Timer Channel 3 Control */

/* Register Bitfield Definitions ********************************************/

#define LPIT_PARAM_EXT_TRIG_SHIFT      (8) /* Bit[15:8]: Number of External Trigger Inputs */
#define LPIT_PARAM_EXT_TRIG_MASK       (0xff << LPIT_PARAM_EXT_TRIG_SHIFT)

#define LPIT_PARAM_CHANNEL_SHIFT       (0) /* Bit[7:0]: Number of Timer Channels */
#define LPIT_PARAM_CHANNEL_MASK        (0xff << LPIT_PARAM_CHANNEL_SHIFT)

#define LPIT_MCR_DBG_EN                (1 << 3) /* Stop Timer when in Debug Mode */
#define LPIT_MCR_DOZE_EN               (1 << 2) /* DOZE Mode Enable */
#define LPIT_MCR_SW_RST                (1 << 1) /* Software Reset Bit */
#define LPIT_MCR_M_CEN                 (1 << 0) /* Module Clock Enable */

#define LPIT_MSR_TIF3                  (1 << 3) /* Channel 3 Timer Interrupt Flag */
#define LPIT_MSR_TIF2                  (1 << 2) /* Channel 2 Timer Interrupt Flag */
#define LPIT_MSR_TIF1                  (1 << 1) /* Channel 1 Timer Interrupt Flag */
#define LPIT_MSR_TIF0                  (1 << 0) /* Channel 0 Timer Interrupt Flag */

#define LPIT_MIER_TIE3                 (1 << 3) /* Channel 3 Timer Interrupt Enable */
#define LPIT_MIER_TIE2                 (1 << 2) /* Channel 2 Timer Interrupt Enable */
#define LPIT_MIER_TIE1                 (1 << 1) /* Channel 1 Timer Interrupt Enable */
#define LPIT_MIER_TIE0                 (1 << 0) /* Channel 0 Timer Interrupt Enable */

#define LPIT_TCTRL_TRG_SEL_SHIFT       (27) /* Bit[27:24]: Trigger Select */
#define LPIT_TCTRL_TRG_SEL_MASK        (0xf << LPIT_TCTRL_TRG_SEL_SHIFT)
#define LPIT_TCTRL_TRG_SEL_CHAN0       (0   << LPIT_TCTRL_TRG_SEL_SHIFT)
#define LPIT_TCTRL_TRG_SEL_CHAN1       (1   << LPIT_TCTRL_TRG_SEL_SHIFT)
#define LPIT_TCTRL_TRG_SEL_CHAN2       (2   << LPIT_TCTRL_TRG_SEL_SHIFT)
#define LPIT_TCTRL_TRG_SEL_CHAN3       (3   << LPIT_TCTRL_TRG_SEL_SHIFT)

#define LPIT_TCTRL_TRG_SRC_SHIFT       (23) /* Bit23: Trigger Source */
#define LPIT_TCTRL_TRG_SRC_MASK        (1 << LPIT_TCTRL_TRG_SRC_SHIFT)
#define LPIT_TCTRL_TRG_SRC_EXTER       (0 << LPIT_TCTRL_TRG_SRC_SHIFT) /* external */
#define LPIT_TCTRL_TRG_SRC_INTER       (1 << LPIT_TCTRL_TRG_SRC_SHIFT) /* internal */

#define LPIT_TCTRL_TROT                (1 << 18) /* Timer Reload On Trigger */
#define LPIT_TCTRL_TSOI                (1 << 17) /* Timer Stop On Interrupt */
#define LPIT_TCTRL_TSOT                (1 << 16) /* Timer Start On Trigger */

#define LPIT_TCTRL_MODE_SHIFT          (2)
#define LPIT_TCTRL_MODE_MASK           (3 << LPIT_TCTRL_MODE_SHIFT)
#define LPIT_TCTRL_MODE_32PC           (0 << LPIT_TCTRL_MODE_SHIFT) /* 32 Bit periodic Counter */
#define LPIT_TCTRL_MODE_D16PC          (1 << LPIT_TCTRL_MODE_SHIFT) /* Dual 16-bit periodic Counter */
#define LPIT_TCTRL_MODE_32TA           (2 << LPIT_TCTRL_MODE_SHIFT) /* 32 bit Trigger Accumulator */
#define LPIT_TCTRL_MODE_32TIC          (3 << LPIT_TCTRL_MODE_SHIFT) /* 32 bit Trigger Input Capture */

#define LPIT_TCTRL_CHAIN               (1 << 1) /* Chain Channel */
#define LPIT_TCTRL_T_EN                (1 << 0) /* Timer Enable */

#endif /* __ARCH_RISCV_SRC_RV32M1_HARDWARE_RV32M1_LPIT_H */
