/****************************************************************************
 * arch/risc-v/src/bl602/hardware/clic.h
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

#ifndef __ARCH_RISCV_SRC_BL602_HARDWARE_CLIC_H
#define __ARCH_RISCV_SRC_BL602_HARDWARE_CLIC_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CLIC_CTRL_ADDR  0x02000000
#define CLIC_HART0_ADDR 0x02800000

#define CLIC_MSIP          0x0000
#define CLIC_MSIP_size     0x4
#define CLIC_MTIMECMP      0x4000
#define CLIC_MTIMECMP_size 0x8
#define CLIC_MTIME         0xBFF8
#define CLIC_MTIME_size    0x8

#define CLIC_INTIP  0x000
#define CLIC_INTIE  0x400
#define CLIC_INTCFG 0x800
#define CLIC_CFG    0xc00

#endif /* __ARCH_RISCV_SRC_BL602_HARDWARE_CLIC_H */
