/****************************************************************************
 * arch/risc-v/src/rv32m1/chip.h
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

#ifndef __ARCH_RISCV_SRC_RV32M1_CHIP_H
#define __ARCH_RISCV_SRC_RV32M1_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/* Include the chip capabilities file */

#include <arch/rv32m1/chip.h>

#ifndef __ASSEMBLY__

/* Include the chip interrupt definition file */

#include <arch/rv32m1/irq.h>

#endif

/* Include the chip memmap */

#include "rv32m1_memorymap.h"

/* Include the chip pinmap */

#include "hardware/rv32m1_pinmap.h"

/* Include the chip pcc */

#include "hardware/rv32m1_pcc.h"

/* Include the chip intmux */

#include "hardware/rv32m1_intmux.h"

/* Incluce the linker file */

#include "rv32m1_linker.h"

#endif /* __ARCH_RISCV_SRC_RV32M1_CHIP_H */
