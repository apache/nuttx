/****************************************************************************
 * arch/risc-v/src/bl602/hardware/bl602_cci.h
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

#ifndef __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_CCI_H
#define __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_CCI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "bl602_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define BL602_CCI_CFG_OFFSET    0x000000  /* cci_cfg */
#define BL602_CCI_ADDR_OFFSET   0x000004  /* cci_addr */
#define BL602_CCI_WDATA_OFFSET  0x000008  /* cci_wdata */
#define BL602_CCI_RDATA_OFFSET  0x00000c  /* cci_rdata */
#define BL602_CCI_CTL_OFFSET    0x000010  /* cci_ctl */

/* Register definitions *****************************************************/

#define BL602_CCI_CFG    (BL602_CCI_BASE + BL602_CCI_CFG_OFFSET)
#define BL602_CCI_ADDR   (BL602_CCI_BASE + BL602_CCI_ADDR_OFFSET)
#define BL602_CCI_WDATA  (BL602_CCI_BASE + BL602_CCI_WDATA_OFFSET)
#define BL602_CCI_RDATA  (BL602_CCI_BASE + BL602_CCI_RDATA_OFFSET)
#define BL602_CCI_CTL    (BL602_CCI_BASE + BL602_CCI_CTL_OFFSET)

/* Register bit definitions *************************************************/

#define CCI_CFG_REG_MCCI_CLK_INV          (1 << 9)
#define CCI_CFG_REG_SCCI_CLK_INV          (1 << 8)
#define CCI_CFG_CFG_CCI1_PRE_READ         (1 << 7)
#define CCI_CFG_REG_DIV_M_CCI_SCLK_SHIFT  (5)
#define CCI_CFG_REG_DIV_M_CCI_SCLK_MASK   (0x03 << CCI_CFG_REG_DIV_M_CCI_SCLK_SHIFT)
#define CCI_CFG_REG_M_CCI_SCLK_EN         (1 << 4)
#define CCI_CFG_CCI_MAS_HW_MODE           (1 << 3)
#define CCI_CFG_CCI_MAS_SEL_CCI2          (1 << 2)
#define CCI_CFG_CCI_SLV_SEL_CCI2          (1 << 1)
#define CCI_CFG_CCI_EN                    (1 << 0)

#define CCI_CTL_AHB_STATE_SHIFT           (2)
#define CCI_CTL_AHB_STATE_MASK            (0x03 << CCI_CTL_AHB_STATE_SHIFT)
#define CCI_CTL_CCI_READ_FLAG             (1 << 1)
#define CCI_CTL_CCI_WRITE_FLAG            (1 << 0)

#endif /* __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_CCI_H */
