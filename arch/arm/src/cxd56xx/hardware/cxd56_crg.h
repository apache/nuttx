/****************************************************************************
 * arch/arm/src/cxd56xx/hardware/cxd56_crg.h
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

#ifndef __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD56_CRG_H
#define __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD56_CRG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CXD56_CRG_GEAR_AHB              (CXD56_CRG_BASE + 0x0000)
#define CXD56_CRG_GEAR_IMG_UART         (CXD56_CRG_BASE + 0x0004)
#define CXD56_CRG_GEAR_IMG_SPI          (CXD56_CRG_BASE + 0x0008)
#define CXD56_CRG_GEAR_PER_SDIO         (CXD56_CRG_BASE + 0x000c)
#define CXD56_CRG_GEAR_PER_USB          (CXD56_CRG_BASE + 0x0010)
#define CXD56_CRG_GEAR_M_IMG_VENB       (CXD56_CRG_BASE + 0x0014)
#define CXD56_CRG_GEAR_N_IMG_VENB       (CXD56_CRG_BASE + 0x0018)
#define CXD56_CRG_GEAR_IMG_WSPI         (CXD56_CRG_BASE + 0x001c)
#define CXD56_CRG_CKEN_EMMC             (CXD56_CRG_BASE + 0x0020)

#define CXD56_CRG_RESET                 (CXD56_CRG_BASE + 0x0030)

#define CXD56_CRG_CK_GATE_AHB           (CXD56_CRG_BASE + 0x0040)

#define CXD56_CRG_CK_MON_EN             (CXD56_CRG_BASE + 0x0050)

#define CXD56_CRG_APP_TILE_CLK_GATING_ENB  (CXD56_ADSP_BASE + 0x02001084)

/* RESET register bits ******************************************************/

#define XRS_AUD     (1<<0)
#define XRS_IMG     (1<<4)
#define XRS_USB     (1<<8)
#define XRS_SDIO    (1<<9)
#define XRS_MMC     (1<<10)
#define XRS_MMC_CRG (1<<11)
#define XRS_DSP_GEN (1<<22)

/* CK_GATE_AHB register bits ************************************************/

#define CK_GATE_AUD  (1<<0)
#define CK_GATE_IMG  (1<<4)
#define CK_GATE_USB  (1<<8)
#define CK_GATE_SDIO (1<<9)
#define CK_GATE_MMC  (1<<10)

#endif
