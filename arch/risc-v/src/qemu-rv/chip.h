/****************************************************************************
 * arch/risc-v/src/qemu-rv/chip.h
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

#ifndef __ARCH_RISCV_SRC_QEMU_RV_CHIP_H
#define __ARCH_RISCV_SRC_QEMU_RV_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/* Include the chip capabilities file */

#include <arch/qemu-rv/chip.h>

#ifndef __ASSEMBLY__

/* Include the chip interrupt definition file */

/* Serial initial function defined in uart_16550.c */

extern void up_earlyserialinit(void);
extern void up_serialinit(void);

#endif

#include "qemu_rv_memorymap.h"

#include "hardware/qemu_rv_clint.h"
#include "hardware/qemu_rv_memorymap.h"
#include "hardware/qemu_rv_plic.h"

#endif /* __ARCH_RISCV_SRC_QEMU_RV_CHIP_H */
