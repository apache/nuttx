/****************************************************************************
 * arch/risc-v/src/rv32m1/hardware/rv32m1_wdog.h
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

#ifndef __ARCH_RISCV_SRC_RV32M1_HARDWARE_RV32M1_WDOG_H
#define __ARCH_RISCV_SRC_RV32M1_HARDWARE_RV32M1_WDOG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define RV32M1_WDOG_CS_OFFSET            0x0000 /* Control and Status */
#define RV32M1_WDOG_CNT_OFFSET           0x0004 /* Counter */
#define RV32M1_WDOG_TOVAL_OFFSET         0x0008 /* Timeout Value */
#define RV32M1_WDOG_WIN_OFFSET           0x000c /* Window */

/* Register Address *********************************************************/

#if defined(CONFIG_ARCH_CHIP_RV32M1_RI5CY)
#  define RV32M1_WDOG_BASE       RV32M1_WDOG0_BASE
#elif defined(CONFIG_ARCH_CHIP_RV32M1_ZERORISCY)
#  define RV32M1_WDOG_BASE       RV32M1_WDOG1_BASE
#else 
#  error "Unsupported RV32M1 Watch dog"
#endif

#define RV32M1_WDOG0_CS         (RV32M1_WDOG0_BASE + RV32M1_WDOG_CS_OFFSET)
#define RV32M1_WDOG0_CNT        (RV32M1_WDOG0_BASE + RV32M1_WDOG_CNT_OFFSET)
#define RV32M1_WDOG0_TOVAL      (RV32M1_WDOG0_BASE + RV32M1_WDOG_TOVAL_OFFSET)
#define RV32M1_WDOG0_WIN        (RV32M1_WDOG0_BASE + RV32M1_WDOG_WIN_OFFSET)

#define RV32M1_WDOG_CS          (RV32M1_WDOG_BASE + RV32M1_WDOG_CS_OFFSET)
#define RV32M1_WDOG_CNT         (RV32M1_WDOG_BASE + RV32M1_WDOG_CNT_OFFSET)
#define RV32M1_WDOG_TOVAL       (RV32M1_WDOG_BASE + RV32M1_WDOG_TOVAL_OFFSET)
#define RV32M1_WDOG_WIN         (RV32M1_WDOG_BASE + RV32M1_WDOG_WIN_OFFSET)

/* Register Bitfield Definitions ********************************************/

#define WDOG_CS_WIN              (1 << 15) /* Bit15: Window */
#define WDOG_CS_FLG              (1 << 14) /* Bit14: Interrupt Flag */
#define WDOG_CS_CMD32EN          (1 << 13) /* Bit13: support 32bit command */
#define WDOG_CS_PRES             (1 << 12) /* Bit12: Prescaler, '1' enables 256 prescaler */
#define WDOG_CS_ULK              (1 << 11) /* Bit11: Unlock status */
#define WDOG_CS_RCS              (1 << 10) /* Bit10: Reconfiguration Success */

#define WDOG_CS_CLK_SHIFT        (8) /* Bit[9:8]: Watchdog Clock */
#define WDOG_CS_CLK_MASK         (3 << WDOG_CS_CLK_SHIFT)
#define WDOG_CS_CLK_BUS          (0 << WDOG_CS_CLK_SHIFT) /* Bus Clock */
#define WDOG_CS_CLK_LPO          (1 << WDOG_CS_CLK_SHIFT) /* LPO Clock(1KHz) */
#define WDOG_CS_CLK_INT          (1 << WDOG_CS_CLK_SHIFT) /* Internal Clock */
#define WDOG_CS_CLK_EXR          (1 << WDOG_CS_CLK_SHIFT) /* External reference Clock */

#define WDOG_CS_EN               (1 << 7) /* Bit7: Enable */
#define WDOG_CS_INT              (1 << 6) /* Bit6: Interrupt */
#define WDOG_CS_UPDATE           (1 << 5) /* Bit5: Allow updates */

#define WDOG_CS_TST_SHIFT        (3)
#define WDOG_CS_TST_MASK         (3 << WDOG_CS_TST_SHIFT)
#define WDOG_CS_TST_DISABLED     (0 << WDOG_CS_TST_SHIFT)
#define WDOG_CS_TST_UEN          (1 << WDOG_CS_TST_SHIFT) /* User Mode Enable */
#define WDOG_CS_TST_TENL         (2 << WDOG_CS_TST_SHIFT) /* Test Mode Enable with the low byte used */
#define WDOG_CS_TST_TENH         (3 << WDOG_CS_TST_SHIFT) /* Test Mode Enable with the high byte used */

#define WDOG_CS_DBG              (1 << 2) /* Debug Enable */
#define WDOG_CS_WAIT             (1 << 1) /* Wait Enable */
#define WDOG_CS_STOP             (1 << 0) /* Stop Enable */

/* The unlock magic number */

#define WDOG_CNT_UNLOCK          (0xd928c520)

#endif /* __ARCH_RISCV_SRC_RV32M1_HARDWARE_RV32M1_WDOG_H */
