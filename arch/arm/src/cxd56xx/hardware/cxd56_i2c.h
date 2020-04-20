/****************************************************************************
 * arch/arm/src/cxd56xx/hardware/cxd56_i2c.h
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

#ifndef __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD56_I2C_H
#define __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD56_I2C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define CXD56_IC_CON                      0x0000
#define CXD56_IC_TAR                      0x0004
#define CXD56_IC_SAR                      0x0008
#define CXD56_IC_HS_MADDR                 0x000C
#define CXD56_IC_DATA_CMD                 0x0010
#define CXD56_IC_SS_SCL_HCNT              0x0014
#define CXD56_IC_SS_SCL_LCNT              0x0018
#define CXD56_IC_FS_SCL_HCNT              0x001C
#define CXD56_IC_FS_SCL_LCNT              0x0020
#define CXD56_IC_HS_SCL_HCNT              0x0024
#define CXD56_IC_HS_SCL_LCNT              0x0028
#define CXD56_IC_INTR_STAT                0x002C
#define CXD56_IC_INTR_MASK                0x0030
#define CXD56_IC_RAW_INTR_STAT            0x0034
#define CXD56_IC_RX_TL                    0x0038
#define CXD56_IC_TX_TL                    0x003C
#define CXD56_IC_CLR_INTR                 0x0040
#define CXD56_IC_CLR_RX_UNDER             0x0044
#define CXD56_IC_CLR_RX_OVER              0x0048
#define CXD56_IC_CLR_TX_OVER              0x004C
#define CXD56_IC_CLR_RD_REQ               0x0050
#define CXD56_IC_CLR_TX_ABRT              0x0054
#define CXD56_IC_CLR_RX_DONE              0x0058
#define CXD56_IC_CLR_ACTIVITY             0x005C
#define CXD56_IC_CLR_STOP_DET             0x0060
#define CXD56_IC_CLR_START_DET            0x0064
#define CXD56_IC_CLR_GEN_CALL             0x0068
#define CXD56_IC_ENABLE                   0x006C
#define CXD56_IC_STATUS                   0x0070
#define CXD56_IC_TXFLR                    0x0074
#define CXD56_IC_RXFLR                    0x0078
#define CXD56_IC_SDA_HOLD                 0x007C
#define CXD56_IC_TX_ABRT_SOURCE           0x0080
#define CXD56_IC_SLV_DATA_NACK_ONLY       0x0084
#define CXD56_IC_DMA_CR                   0x0088
#define CXD56_IC_DMA_TDLR                 0x008C
#define CXD56_IC_DMA_RDLR                 0x0090
#define CXD56_IC_SDA_SETUP                0x0094
#define CXD56_IC_ACK_GENERAL_CALL         0x0098
#define CXD56_IC_ENABLE_STATUS            0x009C
#define CXD56_IC_FS_SPKLEN                0x00A0
#define CXD56_IC_HS_SPKLEN                0x00A4
#define CXD56_IC_TXDATA                   0x00C0
#define CXD56_IC_COMP_PARAM_1             0x00F4
#define CXD56_IC_COMP_VERSION             0x00F8
#define CXD56_IC_COMP_TYPE                0x00FC

/* Register bit definitions *************************************************/

/* IC_CON */

#define IC_RX_FIFO_FULL_HLD_CTRL     (1u<<9)
#define IC_TX_EMPTY_CTRL             (1u<<8)
#define IC_STOP_DET_IFADDRESSED      (1u<<7)
#define IC_SLAVE_DISABLE             (1u<<6)
#define IC_RESTART_EN                (1u<<5)
#define IC_10BITADDR_MASTER          (1u<<4)
#define IC_10BITADDR_SLAVE           (1u<<3)
#define IC_MAX_SPEED_MODE            (3u<<1)
#define IC_MASTER_MODE               (1u<<0)

#define IC_SPEED_SS                  (1u<<1)
#define IC_SPEED_FS                  (2u<<1)
#define IC_SPEED_HS                  (3u<<1)

/* IC_DATA_CMD */

#define CMD_FIRST_DATA_BYTE          (1u<<11)
#define CMD_RESTART                  (1u<<10)
#define CMD_STOP                     (1u<<9)
#define CMD_READ                     (1u<<8)
#define CMD_DAT                      0x000000FFu

/* IC_INTR_STAT     */

/* IC_INTR_MASK     */

/* IC_RAW_INTR_STAT */

#define INTR_MST_ON_HOLD          (1u<<13)
#define INTR_RESTART_DET          (1u<<12)
#define INTR_GEN_CALL             (1u<<11)
#define INTR_START_DET            (1u<<10)
#define INTR_STOP_DET             (1u<< 9)
#define INTR_ACTIVITY             (1u<< 8)
#define INTR_RX_DONE              (1u<< 7)
#define INTR_TX_ABRT              (1u<< 6)
#define INTR_RD_REQ               (1u<< 5)
#define INTR_TX_EMPTY             (1u<< 4)
#define INTR_TX_OVER              (1u<< 3)
#define INTR_RX_FULL              (1u<< 2)
#define INTR_RX_OVER              (1u<< 1)
#define INTR_RX_UNDER             (1u<< 0)

/* IC_STATUS */

#define STATUS_SLV_ACTIVITY       (1u<< 6)
#define STATUS_MST_ACTIVITY       (1u<< 5)
#define STATUS_RFF                (1u<< 4)
#define STATUS_RFNE               (1u<< 3)
#define STATUS_TFE                (1u<< 2)
#define STATUS_TFNF               (1u<< 1)
#define STATUS_ACTIVITY           (1u<< 0)

/* IC_ENABLE_STATUS */

#define ESTATUS_SLV_FIFO_FLUSHED  (1u<< 2)
#define ESTATUS_SLV_RX_ABORTED    (1u<< 1)
#define ESTATUS_IC_EN             (1u<< 0)

/* IC_TX_ABRT_SOURCE */

#define ABRT_USER_ABRT            (1u<<16)
#define ABRT_SLVRD_INTX           (1u<<15)
#define ABRT_SLV_ARBLOST          (1u<<14)
#define ABRT_SLVFLUSH_TXFIFO      (1u<<13)
#define ABRT_ARB_LOST             (1u<<12)
#define ABRT_MASTER_DIS           (1u<<11)
#define ABRT_10B_RD_NORSTRT       (1u<<10)
#define ABRT_SBYTE_NORSTRT        (1u<< 9)
#define ABRT_HS_NORSTRT           (1u<< 8)
#define ABRT_SBYTE_ACKDET         (1u<< 7)
#define ABRT_HS_ACKDET            (1u<< 6)
#define ABRT_GCALL_READ           (1u<< 5)
#define ABRT_GCALL_NOACK          (1u<< 4)
#define ABRT_TXDATA_NOACK         (1u<< 3)
#define ABRT_10ADDR2_NOACK        (1u<< 2)
#define ABRT_10ADDR1_NOACK        (1u<< 1)
#define ABRT_7B_ADDR_NOACK        (1u<< 0)

/* IC_DMA_CR */

#define DMA_TDMAE                 (1u<< 1)
#define DMA_RDMAE                 (1u<< 0)

/* IC_COMP_PARAM_1 */

#define COMP_PARAM_1_TX_BUFFER_DEPTH     0x00FF0000UL
#define COMP_PARAM_1_RX_BUFFER_DEPTH     0x0000FF00UL
#define COMP_PARAM_1_ADD_ENCODED_PARAMS  0x00000080UL
#define COMP_PARAM_1_HAS_DMA             0x00000040UL
#define COMP_PARAM_1_INTR_IO             0x00000020UL
#define COMP_PARAM_1_HC_COUNT_VALUES     0x00000010UL
#define COMP_PARAM_1_MAX_SPEED_MODE      0x0000000CUL
#define COMP_PARAM_1_APB_DATA_WIDTH      0x00000003UL

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD56_I2C_H */
