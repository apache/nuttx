/****************************************************************************
 * arch/risc-v/src/bl602/hardware/bl602_cks.h
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

#ifndef __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_CKS_H
#define __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_CKS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "bl602_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define BL602_CKS_CONFIG_OFFSET      0x000000  /* cks_config */
#define BL602_CKS_DATA_IN_OFFSET     0x000004  /* data_in */
#define BL602_CKS_OUT_OFFSET         0x000008  /* cks_out */

/* Register definitions *****************************************************/

#define BL602_CKS_CONFIG      (BL602_CKS_BASE + BL602_CKS_CONFIG_OFFSET)
#define BL602_CKS_DATA_IN     (BL602_CKS_BASE + BL602_CKS_DATA_IN_OFFSET)
#define BL602_CKS_OUT         (BL602_CKS_BASE + BL602_CKS_OUT_OFFSET)

/* Register bit definitions *************************************************/

#define CKS_CONFIG_CR_CKS_BYTE_SWAP      (1 << 1)
#define CKS_CKS_CONFIG_CR_CKS_CLR        (1 << 0)

#define CKS_DATA_IN_MASK                 (0xff)

#define CKS_OUT_MASK                     (0xffff)

#endif /* __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_CKS_H */
