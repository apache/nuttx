/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_fpga_canfd.h
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

#ifndef __ARCH_RISCV_SRC_MPFS_MPFS_FPGA_CANFD_H
#define __ARCH_RISCV_SRC_MPFS_MPFS_FPGA_CANFD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Check if CAN-FD support is enabled. */

#ifdef CONFIG_MPFS_CANFD

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/board/board.h>
#include "hardware/mpfs_fpga_canfd.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_fpga_canfd_init
 *
 * Description:
 *   Initialize a CANFD block.
 *
 * Returned Value:
 *   OK on success, Negated errno on failure
 *
 ****************************************************************************/

int mpfs_fpga_canfd_init(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_MPFS_CANFD */
#endif /* __ARCH_RISCV_SRC_MPFS_MPFS_FPGA_CANFD_H */
