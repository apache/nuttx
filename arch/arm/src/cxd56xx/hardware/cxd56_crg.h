/****************************************************************************
 * arch/arm/src/cxd56xx/hardware/cxd56_crg.h
 *
 *   Copyright (C) 2008-2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
