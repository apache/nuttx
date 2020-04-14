/****************************************************************************
 * arch/arm/src/cxd56xx/hardware/cxd56_ge2d.h
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

#ifndef __ARCH_ARM_SRC_CXD56XX_CXD56_GE2D_H
#define __ARCH_ARM_SRC_CXD56XX_CXD56_GE2D_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/cxd5602_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GE2D_INTR_ENABLE                     (CXD56_GE2D_BASE+0x00)
#define GE2D_INTR_STAT                       (CXD56_GE2D_BASE+0x04)
#define GE2D_ADDRESS_DESCRIPTOR_START        (CXD56_GE2D_BASE+0x08)
#define GE2D_STATUS                          (CXD56_GE2D_BASE+0x0c) /* Read */
#define GE2D_CMD_DESCRIPTOR                  (CXD56_GE2D_BASE+0x0c) /* Write */
#define GE2D_STAT_NORMAL_DESCRIPTOR_ADDRESS  (CXD56_GE2D_BASE+0x10)
#define GE2D_STAT_CURRENT_DESCRIPTOR_ADDRESS (CXD56_GE2D_BASE+0x14)
#define GE2D_AHB_BURST_MODE                  (CXD56_GE2D_BASE+0x40)

/* Interrupt bits */

#define GE2D_INTR_WR_ERR   (1 << 17)
#define GE2D_INTR_RD_ERR   (1 << 16)
#define GE2D_INTR_DSD      (1 << 8)
#define GE2D_INTR_NDE      (1 << 3)
#define GE2D_INTR_NDB      (1 << 2)
#define GE2D_INTR_NDF      (1 << 1)
#define GE2D_INTR_HPU      (1 << 0)

#define GE2D_INTR_ALL (GE2D_INTR_WR_ERR | GE2D_INTR_RD_ERR | \
                       GE2D_INTR_DSD | GE2D_INTR_NDE | GE2D_INTR_NDB | \
                       GE2D_INTR_NDF | GE2D_INTR_HPU)

/* Status bits */

#define GE2D_STAT_ISER     (1 << 24)
#define GE2D_STAT_NDCR     (1 << 8)
#define GE2D_STAT_SREQ     (1 << 2)
#define GE2D_STAT_PREQ     (1 << 1)
#define GE2D_STAT_NREQ     (1 << 0)

/* Running control */

#define GE2D_NOP   0
#define GE2D_EXEC  1
#define GE2D_STOP  3

#endif /* __ARCH_ARM_SRC_CXD56XX_CXD56_GE2D_H */
