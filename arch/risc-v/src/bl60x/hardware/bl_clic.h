/****************************************************************************
 * arch/risc-v/src/bl60x/hardware/bl_clint.h
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

#ifndef __ARCH_RISCV_SRC_BL60X_HARDWARE_BL_CLINT_H
#define __ARCH_RISCV_SRC_BL60X_HARDWARE_BL_CLINT_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BL_CLIC_MSIP           (BL_CLIC_CTRL_BASE + 0x0000)  /* size 4 */
#define BL_CLIC_MTIMECMP       (BL_CLIC_CTRL_BASE + 0x4000)  /* size 8 */
#define BL_CLIC_MTIME          (BL_CLIC_CTRL_BASE + 0xBFF8)  /* size 8 */

#define BL_CLIC_INTIP          (BL_CLIC_HART0_BASE + 0x000)
#define BL_CLIC_INTIE          (BL_CLIC_HART0_BASE + 0x400)
#define BL_CLIC_INTCFG         (BL_CLIC_HART0_BASE + 0x800)
#define BL_CLIC_CFG            (BL_CLIC_HART0_BASE + 0xC00)

#endif /* __ARCH_RISCV_SRC_BL60X_HARDWARE_BL_CLINT_H */
