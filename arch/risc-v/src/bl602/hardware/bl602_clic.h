/****************************************************************************
 * arch/risc-v/src/bl602/hardware/bl602_clic.h
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

#ifndef __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_CLIC_H
#define __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_CLIC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "bl602_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define BL602_CLIC_MSIP_OFFSET           0x0000
#define BL602_CLIC_MTIMECMP_OFFSET       0x4000
#define BL602_CLIC_MTIME_OFFSET          0xbff8

#define BL602_CLIC_INTIP_OFFSET          0x000
#define BL602_CLIC_INTIE_OFFSET          0x400
#define BL602_CLIC_INTCFG_OFFSET         0x800
#define BL602_CLIC_CFG_OFFSET            0xc00

/* Register definitions *****************************************************/

#define BL602_CLIC_MSIP       (BL602_CLIC_CTRL_BASE + BL602_CLIC_MSIP_OFFSET)
#define BL602_CLIC_MTIMECMP   (BL602_CLIC_CTRL_BASE + BL602_CLIC_MTIMECMP_OFFSET)
#define BL602_CLIC_MTIME      (BL602_CLIC_CTRL_BASE + BL602_CLIC_MTIME_OFFSET)

#define BL602_CLIC_INTIP      (BL602_CLIC_HART0_BASE + BL602_CLIC_INTIP_OFFSET)
#define BL602_CLIC_INTIE      (BL602_CLIC_HART0_BASE + BL602_CLIC_INTIE_OFFSET)
#define BL602_CLIC_INTCFG     (BL602_CLIC_HART0_BASE + BL602_CLIC_INTCFG_OFFSET)
#define BL602_CLIC_CFG        (BL602_CLIC_HART0_BASE + BL602_CLIC_CFG_OFFSET)

#endif /* __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_CLIC_H */
