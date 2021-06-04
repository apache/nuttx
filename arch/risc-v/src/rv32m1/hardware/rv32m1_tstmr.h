/****************************************************************************
 * arch/risc-v/src/rv32m1/hardware/rv32m1_tstmr.h
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

#ifndef __ARCH_RISCV_SRC_RV32M1_HARDWARE_RV32M1_TSTMR_H
#define __ARCH_RISCV_SRC_RV32M1_HARDWARE_RV32M1_TSTMR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define RV32M1_TSTMR_LOW_OFFSET         0x0000 /* Time Stamp Timer Register Low */
#define RV32M1_TSTMR_HIGH_OFFSET        0x0004 /* Time Stamp Timer Register High */

/* Register Address *********************************************************/

#define RV32M1_TSTMRA_LOW       (RV32M1_TSTMRA_BASE + RV32M1_TSTMR_LOW_OFFSET)
#define RV32M1_TSTMRA_HIGH      (RV32M1_TSTMRA_BASE + RV32M1_TSTMR_HIGH_OFFSET)

#define RV32M1_TSTMRB_LOW       (RV32M1_TSTMRB_BASE + RV32M1_TSTMR_LOW_OFFSET)
#define RV32M1_TSTMRB_HIGH      (RV32M1_TSTMRB_BASE + RV32M1_TSTMR_HIGH_OFFSET)

#ifdef CONFIG_ARCH_CHIP_RV32M1_RI5CY
#  define RV32M1_TSTMR_BASE     RV32M1_TSTMRA_BASE
#else
#  define RV32M1_TSTMR_BASE     RV32M1_TSTMRB_BASE
#endif

#define RV32M1_TSTMR_LOW        (RV32M1_TSTMR_BASE + RV32M1_TSTMR_LOW_OFFSET)
#define RV32M1_TSTMR_HIGH       (RV32M1_TSTMR_BASE + RV32M1_TSTMR_HIGH_OFFSET)

#endif /* __ARCH_RISCV_SRC_RV32M1_HARDWARE_RV32M1_TSTMR_H */
