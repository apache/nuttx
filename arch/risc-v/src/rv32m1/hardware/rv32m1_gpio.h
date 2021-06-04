/****************************************************************************
 * arch/risc-v/src/rv32m1/hardware/rv32m1_gpio.h
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

#ifndef __ARCH_RISCV_SRC_RV32M1_HARDWARE_RV32M1_GPIO_H
#define __ARCH_RISCV_SRC_RV32M1_HARDWARE_RV32M1_GPIO_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define RV32M1_GPIO_PDOR_OFFSET            0x0000 /* Port Data Output */
#define RV32M1_GPIO_PSOR_OFFSET            0x0004 /* Port Set Output */
#define RV32M1_GPIO_PCOR_OFFSET            0x0008 /* Port Clear Output */
#define RV32M1_GPIO_PTOR_OFFSET            0x000c /* Port Toggle Output */
#define RV32M1_GPIO_PDIR_OFFSET            0x0010 /* Port Data Input */
#define RV32M1_GPIO_PDDR_OFFSET            0x0014 /* Port Data Direction */

/* Register Bitfield Definitions ********************************************/

#endif /* __ARCH_RISCV_SRC_RV32M1_HARDWARE_RV32M1_GPIO_H */
