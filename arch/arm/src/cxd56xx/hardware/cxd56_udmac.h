/****************************************************************************
 * arch/arm/src/cxd56xx/hardware/cxd56_udmac.h
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

#ifndef __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD56_UDMAC_H
#define __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD56_UDMAC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/cxd5602_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

/* Common Register Offsets */

#define CXD56_UDMAC_DMA_STATUS            0x000
#define CXD56_UDMAC_DMA_CFG               0x004
#define CXD56_UDMAC_CTRL_BASE_PTR         0x008
#define CXD56_UDMAC_ALT_CTRL_BASE_PTR     0x00c
#define CXD56_UDMAC_DMA_WAITONREQ_STATUS  0x010
#define CXD56_UDMAC_CHNL_SW_REQUEST       0x014
#define CXD56_UDMAC_CHNL_USEBURST_SET     0x018
#define CXD56_UDMAC_CHNL_USEBURST_CLR     0x01c
#define CXD56_UDMAC_CHNL_REQ_MASK_SET     0x020
#define CXD56_UDMAC_CHNL_REQ_MASK_CLR     0x024
#define CXD56_UDMAC_CHNL_ENABLE_SET       0x028
#define CXD56_UDMAC_CHNL_ENABLE_CLR       0x02c
#define CXD56_UDMAC_CHNL_PRI_ALT_SET      0x030
#define CXD56_UDMAC_CHNL_PRI_ALT_CLR      0x034
#define CXD56_UDMAC_CHNL_PRIORITY_SET     0x038
#define CXD56_UDMAC_CHNL_PRIORITY_CLR     0x03c
#define CXD56_UDMAC_ERR_CLR               0x04c
#define CXD56_UDMAC_DMA_DONE              0x050
#define CXD56_UDMAC_DMA_ERR               0x054

#define CXD56_DMA_STATUS            (CXD56_DMAC0_BASE + CXD56_UDMAC_DMA_STATUS)
#define CXD56_DMA_CFG               (CXD56_DMAC0_BASE + CXD56_UDMAC_DMA_CFG)
#define CXD56_DMA_CTRLBASE          (CXD56_DMAC0_BASE + CXD56_UDMAC_CTRL_BASE_PTR)
#define CXD56_DMA_ALTCTRLBASE       (CXD56_DMAC0_BASE + CXD56_UDMAC_ALT_CTRL_BASE_PTR)
#define CXD56_DMA_CHWAITSTATUS      (CXD56_DMAC0_BASE + CXD56_UDMAC_DMA_WAITONREQ_STATUS)
#define CXD56_DMA_CHSWREQUEST       (CXD56_DMAC0_BASE + CXD56_UDMAC_CHNL_SW_REQUEST)
#define CXD56_DMA_CHUSEBURSTS       (CXD56_DMAC0_BASE + CXD56_UDMAC_CHNL_USEBURST_SET)
#define CXD56_DMA_CHUSEBURSTC       (CXD56_DMAC0_BASE + CXD56_UDMAC_CHNL_USEBURST_CLR)
#define CXD56_DMA_CHREQMASKS        (CXD56_DMAC0_BASE + CXD56_UDMAC_CHNL_REQ_MASK_SET)
#define CXD56_DMA_CHREQMASKC        (CXD56_DMAC0_BASE + CXD56_UDMAC_CHNL_REQ_MASK_CLR)
#define CXD56_DMA_CHENS             (CXD56_DMAC0_BASE + CXD56_UDMAC_CHNL_ENABLE_SET)
#define CXD56_DMA_CHENC             (CXD56_DMAC0_BASE + CXD56_UDMAC_CHNL_ENABLE_CLR)
#define CXD56_DMA_CHALTS            (CXD56_DMAC0_BASE + CXD56_UDMAC_CHNL_PRI_ALT_SET)
#define CXD56_DMA_CHALTC            (CXD56_DMAC0_BASE + CXD56_UDMAC_CHNL_PRI_ALT_CLR)
#define CXD56_DMA_CHPRIS            (CXD56_DMAC0_BASE + CXD56_UDMAC_CHNL_PRIORITY_SET)
#define CXD56_DMA_CHPRIC            (CXD56_DMAC0_BASE + CXD56_UDMAC_CHNL_PRIORITY_CLR)
#define CXD56_DMA_ERRORC            (CXD56_DMAC0_BASE + CXD56_UDMAC_ERR_CLR)
#define CXD56_DMA_DONE              (CXD56_DMAC0_BASE + CXD56_UDMAC_DMA_DONE)
#define CXD56_DMA_ERR               (CXD56_DMAC0_BASE + CXD56_UDMAC_DMA_ERR)

#define DMA_CFG_EN          1

#define CXD56_DMA_NCHANNELS 32

#define DMA_CTRL_SRC_PROT_NON_PRIVILEGED (0 << 18)
#define DMA_CTRL_SRC_PROT_PRIVILEGED     (1 << 18)
#define DMA_CTRL_DST_PROT_NON_PRIVILEGED (0 << 21)
#define DMA_CTRL_DST_PROT_PRIVILEGED     (1 << 21)

#define DMA_CTRL_SRC_SIZE_BYTE     (0 << 24)
#define DMA_CTRL_SRC_SIZE_HALFWORD (1 << 24)
#define DMA_CTRL_SRC_SIZE_WORD     (2 << 24)
#define DMA_CTRL_SRC_SIZE_NONE     (3 << 24)

#define DMA_CTRL_SRC_INC_BYTE     (0 << 26)
#define DMA_CTRL_SRC_INC_HALFWORD (1 << 26)
#define DMA_CTRL_SRC_INC_WORD     (2 << 26)
#define DMA_CTRL_SRC_INC_NONE     (3 << 26)

#define DMA_CTRL_DST_SIZE_BYTE     (0 << 28)
#define DMA_CTRL_DST_SIZE_HALFWORD (1 << 28)
#define DMA_CTRL_DST_SIZE_WORD     (2 << 28)
#define DMA_CTRL_DST_SIZE_NONE     (3 << 28)

#define DMA_CTRL_DST_INC_BYTE     (0 << 30)
#define DMA_CTRL_DST_INC_HALFWORD (1 << 30)
#define DMA_CTRL_DST_INC_WORD     (2 << 30)
#define DMA_CTRL_DST_INC_NONE     (3 << 30)

#define DMA_CTRL_R_POWER_1        (0 << 14)
#define DMA_CTRL_R_POWER_2        (1 << 14)
#define DMA_CTRL_R_POWER_4        (2 << 14)
#define DMA_CTRL_R_POWER_8        (3 << 14)
#define DMA_CTRL_R_POWER_16       (4 << 14)
#define DMA_CTRL_R_POWER_32       (5 << 14)
#define DMA_CTRL_R_POWER_64       (6 << 14)
#define DMA_CTRL_R_POWER_128      (7 << 14)
#define DMA_CTRL_R_POWER_256      (8 << 14)
#define DMA_CTRL_R_POWER_512      (9 << 14)
#define DMA_CTRL_R_POWER_1024     (10 << 14)

#define DMA_CTRL_NEXT_USEBURST    (1 << 3)

#define DMA_CTRL_CYCLE_CTRL_INVALID                    0x0 /* Invalid cycle type */
#define DMA_CTRL_CYCLE_CTRL_BASIC                      0x1 /* Basic cycle type */
#define DMA_CTRL_CYCLE_CTRL_AUTO                       0x2 /* Auto cycle type */
#define DMA_CTRL_CYCLE_CTRL_PINGPONG                   0x3 /* PingPong cycle type */
#define DMA_CTRL_CYCLE_CTRL_MEM_SCATTER_GATHER         0x4 /* Memory scatter gather cycle type */
#define DMA_CTRL_CYCLE_CTRL_MEM_SCATTER_GATHER_ALT     0x5 /* Memory scatter gather using alternate structure  */
#define DMA_CTRL_CYCLE_CTRL_PER_SCATTER_GATHER         0x6 /* Peripheral scatter gather cycle type */
#define DMA_CTRL_CYCLE_CTRL_PER_SCATTER_GATHER_ALT     0x7 /* Peripheral scatter gather cycle type using alternate structure */

#define DMA_CTRL_N_MINUS_1(n) ((n) << 4)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct dma_descriptor_s
{
    volatile uintptr_t srcend;
    volatile uintptr_t dstend;
    volatile uint32_t  ctrl;
    volatile uint32_t  user;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD56_UDMAC_H */
