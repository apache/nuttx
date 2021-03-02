/****************************************************************************
 * arch/arm/src/rp2040/hardware/rp2040_i2c.h
 *
 * Generated from rp2040.svd originally provided by
 *   Raspberry Pi (Trading) Ltd.
 *
 * Copyright 2020 (c) 2020 Raspberry Pi (Trading) Ltd.
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
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_RP2040_HARDWARE_RP2040_I2C_H
#define __ARCH_ARM_SRC_RP2040_HARDWARE_RP2040_I2C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/rp2040_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define RP2040_I2C_IC_CON_OFFSET                 0x000000  /* I2C Control Register */
#define RP2040_I2C_IC_TAR_OFFSET                 0x000004  /* I2C Target Address Register */
#define RP2040_I2C_IC_SAR_OFFSET                 0x000008  /* I2C Slave Address Register */
#define RP2040_I2C_IC_DATA_CMD_OFFSET            0x000010  /* I2C Rx/Tx Data Buffer and Command Register */
#define RP2040_I2C_IC_SS_SCL_HCNT_OFFSET         0x000014  /* Standard Speed I2C Clock SCL High Count Register */
#define RP2040_I2C_IC_SS_SCL_LCNT_OFFSET         0x000018  /* Standard Speed I2C Clock SCL Low Count Register */
#define RP2040_I2C_IC_FS_SCL_HCNT_OFFSET         0x00001c  /* Fast Mode or Fast Mode Plus I2C Clock SCL High Count Register */
#define RP2040_I2C_IC_FS_SCL_LCNT_OFFSET         0x000020  /* Fast Mode or Fast Mode Plus I2C Clock SCL Low Count Register */
#define RP2040_I2C_IC_INTR_STAT_OFFSET           0x00002c  /* I2C Interrupt Status Register */
#define RP2040_I2C_IC_INTR_MASK_OFFSET           0x000030  /* I2C Interrupt Mask Register */
#define RP2040_I2C_IC_RAW_INTR_STAT_OFFSET       0x000034  /* I2C Raw Interrupt Status Register */
#define RP2040_I2C_IC_RX_TL_OFFSET               0x000038  /* I2C Receive FIFO Threshold Register */
#define RP2040_I2C_IC_TX_TL_OFFSET               0x00003c  /* I2C Transmit FIFO Threshold Register */
#define RP2040_I2C_IC_CLR_INTR_OFFSET            0x000040  /* Clear Combined and Individual Interrupt Register */
#define RP2040_I2C_IC_CLR_RX_UNDER_OFFSET        0x000044  /* Clear RX_UNDER Interrupt Register */
#define RP2040_I2C_IC_CLR_RX_OVER_OFFSET         0x000048  /* Clear RX_OVER Interrupt Register */
#define RP2040_I2C_IC_CLR_TX_OVER_OFFSET         0x00004c  /* Clear TX_OVER Interrupt Register */
#define RP2040_I2C_IC_CLR_RD_REQ_OFFSET          0x000050  /* Clear RD_REQ Interrupt Register */
#define RP2040_I2C_IC_CLR_TX_ABRT_OFFSET         0x000054  /* Clear TX_ABRT Interrupt Register */
#define RP2040_I2C_IC_CLR_RX_DONE_OFFSET         0x000058  /* Clear RX_DONE Interrupt Register */
#define RP2040_I2C_IC_CLR_ACTIVITY_OFFSET        0x00005c  /* Clear ACTIVITY Interrupt Register */
#define RP2040_I2C_IC_CLR_STOP_DET_OFFSET        0x000060  /* Clear STOP_DET Interrupt Register */
#define RP2040_I2C_IC_CLR_START_DET_OFFSET       0x000064  /* Clear START_DET Interrupt Register */
#define RP2040_I2C_IC_CLR_GEN_CALL_OFFSET        0x000068  /* Clear GEN_CALL Interrupt Register */
#define RP2040_I2C_IC_ENABLE_OFFSET              0x00006c  /* I2C Enable Register */
#define RP2040_I2C_IC_STATUS_OFFSET              0x000070  /* I2C Status Register */
#define RP2040_I2C_IC_TXFLR_OFFSET               0x000074  /* I2C Transmit FIFO Level Register */
#define RP2040_I2C_IC_RXFLR_OFFSET               0x000078  /* I2C Receive FIFO Level Register */
#define RP2040_I2C_IC_SDA_HOLD_OFFSET            0x00007c  /* I2C SDA Hold Time Length Register */
#define RP2040_I2C_IC_TX_ABRT_SOURCE_OFFSET      0x000080  /* I2C Transmit Abort Source Register */
#define RP2040_I2C_IC_SLV_DATA_NACK_ONLY_OFFSET  0x000084  /* Generate Slave Data NACK Register */
#define RP2040_I2C_IC_DMA_CR_OFFSET              0x000088  /* DMA Control Register */
#define RP2040_I2C_IC_DMA_TDLR_OFFSET            0x00008c  /* DMA Transmit Data Level Register */
#define RP2040_I2C_IC_DMA_RDLR_OFFSET            0x000090  /* I2C Receive Data Level Register */
#define RP2040_I2C_IC_SDA_SETUP_OFFSET           0x000094  /* I2C SDA Setup Register */
#define RP2040_I2C_IC_ACK_GENERAL_CALL_OFFSET    0x000098  /* I2C ACK General Call Register */
#define RP2040_I2C_IC_ENABLE_STATUS_OFFSET       0x00009c  /* I2C Enable Status Register */
#define RP2040_I2C_IC_FS_SPKLEN_OFFSET           0x0000a0  /* I2C SS, FS or FM+ spike suppression limit */
#define RP2040_I2C_IC_CLR_RESTART_DET_OFFSET     0x0000a8  /* Clear RESTART_DET Interrupt Register */
#define RP2040_I2C_IC_COMP_PARAM_1_OFFSET        0x0000f4  /* Component Parameter Register 1 */
#define RP2040_I2C_IC_COMP_VERSION_OFFSET        0x0000f8  /* I2C Component Version Register */
#define RP2040_I2C_IC_COMP_TYPE_OFFSET           0x0000fc  /* I2C Component Type Register */

/* Register definitions *****************************************************/

#define RP2040_I2C_IC_CON(n)                (RP2040_I2C_BASE(n) + RP2040_I2C_IC_CON_OFFSET)
#define RP2040_I2C_IC_TAR(n)                (RP2040_I2C_BASE(n) + RP2040_I2C_IC_TAR_OFFSET)
#define RP2040_I2C_IC_SAR(n)                (RP2040_I2C_BASE(n) + RP2040_I2C_IC_SAR_OFFSET)
#define RP2040_I2C_IC_DATA_CMD(n)           (RP2040_I2C_BASE(n) + RP2040_I2C_IC_DATA_CMD_OFFSET)
#define RP2040_I2C_IC_SS_SCL_HCNT(n)        (RP2040_I2C_BASE(n) + RP2040_I2C_IC_SS_SCL_HCNT_OFFSET)
#define RP2040_I2C_IC_SS_SCL_LCNT(n)        (RP2040_I2C_BASE(n) + RP2040_I2C_IC_SS_SCL_LCNT_OFFSET)
#define RP2040_I2C_IC_FS_SCL_HCNT(n)        (RP2040_I2C_BASE(n) + RP2040_I2C_IC_FS_SCL_HCNT_OFFSET)
#define RP2040_I2C_IC_FS_SCL_LCNT(n)        (RP2040_I2C_BASE(n) + RP2040_I2C_IC_FS_SCL_LCNT_OFFSET)
#define RP2040_I2C_IC_INTR_STAT(n)          (RP2040_I2C_BASE(n) + RP2040_I2C_IC_INTR_STAT_OFFSET)
#define RP2040_I2C_IC_INTR_MASK(n)          (RP2040_I2C_BASE(n) + RP2040_I2C_IC_INTR_MASK_OFFSET)
#define RP2040_I2C_IC_RAW_INTR_STAT(n)      (RP2040_I2C_BASE(n) + RP2040_I2C_IC_RAW_INTR_STAT_OFFSET)
#define RP2040_I2C_IC_RX_TL(n)              (RP2040_I2C_BASE(n) + RP2040_I2C_IC_RX_TL_OFFSET)
#define RP2040_I2C_IC_TX_TL(n)              (RP2040_I2C_BASE(n) + RP2040_I2C_IC_TX_TL_OFFSET)
#define RP2040_I2C_IC_CLR_INTR(n)           (RP2040_I2C_BASE(n) + RP2040_I2C_IC_CLR_INTR_OFFSET)
#define RP2040_I2C_IC_CLR_RX_UNDER(n)       (RP2040_I2C_BASE(n) + RP2040_I2C_IC_CLR_RX_UNDER_OFFSET)
#define RP2040_I2C_IC_CLR_RX_OVER(n)        (RP2040_I2C_BASE(n) + RP2040_I2C_IC_CLR_RX_OVER_OFFSET)
#define RP2040_I2C_IC_CLR_TX_OVER(n)        (RP2040_I2C_BASE(n) + RP2040_I2C_IC_CLR_TX_OVER_OFFSET)
#define RP2040_I2C_IC_CLR_RD_REQ(n)         (RP2040_I2C_BASE(n) + RP2040_I2C_IC_CLR_RD_REQ_OFFSET)
#define RP2040_I2C_IC_CLR_TX_ABRT(n)        (RP2040_I2C_BASE(n) + RP2040_I2C_IC_CLR_TX_ABRT_OFFSET)
#define RP2040_I2C_IC_CLR_RX_DONE(n)        (RP2040_I2C_BASE(n) + RP2040_I2C_IC_CLR_RX_DONE_OFFSET)
#define RP2040_I2C_IC_CLR_ACTIVITY(n)       (RP2040_I2C_BASE(n) + RP2040_I2C_IC_CLR_ACTIVITY_OFFSET)
#define RP2040_I2C_IC_CLR_STOP_DET(n)       (RP2040_I2C_BASE(n) + RP2040_I2C_IC_CLR_STOP_DET_OFFSET)
#define RP2040_I2C_IC_CLR_START_DET(n)      (RP2040_I2C_BASE(n) + RP2040_I2C_IC_CLR_START_DET_OFFSET)
#define RP2040_I2C_IC_CLR_GEN_CALL(n)       (RP2040_I2C_BASE(n) + RP2040_I2C_IC_CLR_GEN_CALL_OFFSET)
#define RP2040_I2C_IC_ENABLE(n)             (RP2040_I2C_BASE(n) + RP2040_I2C_IC_ENABLE_OFFSET)
#define RP2040_I2C_IC_STATUS(n)             (RP2040_I2C_BASE(n) + RP2040_I2C_IC_STATUS_OFFSET)
#define RP2040_I2C_IC_TXFLR(n)              (RP2040_I2C_BASE(n) + RP2040_I2C_IC_TXFLR_OFFSET)
#define RP2040_I2C_IC_RXFLR(n)              (RP2040_I2C_BASE(n) + RP2040_I2C_IC_RXFLR_OFFSET)
#define RP2040_I2C_IC_SDA_HOLD(n)           (RP2040_I2C_BASE(n) + RP2040_I2C_IC_SDA_HOLD_OFFSET)
#define RP2040_I2C_IC_TX_ABRT_SOURCE(n)     (RP2040_I2C_BASE(n) + RP2040_I2C_IC_TX_ABRT_SOURCE_OFFSET)
#define RP2040_I2C_IC_SLV_DATA_NACK_ONLY(n) (RP2040_I2C_BASE(n) + RP2040_I2C_IC_SLV_DATA_NACK_ONLY_OFFSET)
#define RP2040_I2C_IC_DMA_CR(n)             (RP2040_I2C_BASE(n) + RP2040_I2C_IC_DMA_CR_OFFSET)
#define RP2040_I2C_IC_DMA_TDLR(n)           (RP2040_I2C_BASE(n) + RP2040_I2C_IC_DMA_TDLR_OFFSET)
#define RP2040_I2C_IC_DMA_RDLR(n)           (RP2040_I2C_BASE(n) + RP2040_I2C_IC_DMA_RDLR_OFFSET)
#define RP2040_I2C_IC_SDA_SETUP(n)          (RP2040_I2C_BASE(n) + RP2040_I2C_IC_SDA_SETUP_OFFSET)
#define RP2040_I2C_IC_ACK_GENERAL_CALL(n)   (RP2040_I2C_BASE(n) + RP2040_I2C_IC_ACK_GENERAL_CALL_OFFSET)
#define RP2040_I2C_IC_ENABLE_STATUS(n)      (RP2040_I2C_BASE(n) + RP2040_I2C_IC_ENABLE_STATUS_OFFSET)
#define RP2040_I2C_IC_FS_SPKLEN(n)          (RP2040_I2C_BASE(n) + RP2040_I2C_IC_FS_SPKLEN_OFFSET)
#define RP2040_I2C_IC_CLR_RESTART_DET(n)    (RP2040_I2C_BASE(n) + RP2040_I2C_IC_CLR_RESTART_DET_OFFSET)
#define RP2040_I2C_IC_COMP_PARAM_1(n)       (RP2040_I2C_BASE(n) + RP2040_I2C_IC_COMP_PARAM_1_OFFSET)
#define RP2040_I2C_IC_COMP_VERSION(n)       (RP2040_I2C_BASE(n) + RP2040_I2C_IC_COMP_VERSION_OFFSET)
#define RP2040_I2C_IC_COMP_TYPE(n)          (RP2040_I2C_BASE(n) + RP2040_I2C_IC_COMP_TYPE_OFFSET)

/* Register bit definitions *************************************************/

#define RP2040_I2C_IC_CON_STOP_DET_IF_MASTER_ACTIVE          (1 << 10) /* Master issues the STOP_DET interrupt irrespective of whether master is active or not */
#define RP2040_I2C_IC_CON_RX_FIFO_FULL_HLD_CTRL              (1 << 9)  /* Hold bus when RX_FIFO is full */
#define RP2040_I2C_IC_CON_TX_EMPTY_CTRL                      (1 << 8)  /* Controlled generation of TX_EMPTY interrupt */
#define RP2040_I2C_IC_CON_STOP_DET_IFADDRESSED               (1 << 7)  /* slave issues STOP_DET intr only if addressed */
#define RP2040_I2C_IC_CON_IC_SLAVE_DISABLE                   (1 << 6)  /* Slave mode is disabled */
#define RP2040_I2C_IC_CON_IC_RESTART_EN                      (1 << 5)  /* Master restart enabled */
#define RP2040_I2C_IC_CON_IC_10BITADDR_MASTER                (1 << 4)  /* Master 10Bit addressing mode */
#define RP2040_I2C_IC_CON_IC_10BITADDR_SLAVE                 (1 << 3)  /* Slave 10Bit addressing */
#define RP2040_I2C_IC_CON_SPEED_SHIFT                        (1)       /* These bits control at which speed the DW_apb_i2c operates */
#define RP2040_I2C_IC_CON_SPEED_MASK                         (0x03 << RP2040_I2C_IC_CON_SPEED_SHIFT)
#define RP2040_I2C_IC_CON_SPEED_STANDARD                     (0x1 << RP2040_I2C_IC_CON_SPEED_SHIFT)
#define RP2040_I2C_IC_CON_SPEED_FAST                         (0x2 << RP2040_I2C_IC_CON_SPEED_SHIFT)
#define RP2040_I2C_IC_CON_SPEED_HIGH                         (0x3 << RP2040_I2C_IC_CON_SPEED_SHIFT)
#define RP2040_I2C_IC_CON_MASTER_MODE                        (1 << 0)  /* Master mode is enabled */

#define RP2040_I2C_IC_TAR_SPECIAL                            (1 << 11) /* Enables programming of GENERAL_CALL or START_BYTE transmission */
#define RP2040_I2C_IC_TAR_GC_OR_START                        (1 << 10) /* START byte transmission */
#define RP2040_I2C_IC_TAR_MASK                               (0x3ff)   /* This is the target address for any master transaction. */

#define RP2040_I2C_IC_SAR_MASK                               (0x3ff)   /* The IC_SAR holds the slave address when the I2C is operating as a slave. */

#define RP2040_I2C_IC_DATA_CMD_FIRST_DATA_BYTE               (1 << 11) /* Non sequential data byte received */
#define RP2040_I2C_IC_DATA_CMD_RESTART                       (1 << 10) /* Issue RESTART before this command */
#define RP2040_I2C_IC_DATA_CMD_STOP                          (1 << 9)  /* Issue STOP after this command */
#define RP2040_I2C_IC_DATA_CMD_CMD                           (1 << 8)  /* Master Read Command */
#define RP2040_I2C_IC_DATA_CMD_DAT_MASK                      (0xff)    /* This register contains the data to be transmitted or received on the I2C bus. */

#define RP2040_I2C_IC_SS_SCL_HCNT_MASK                       (0xffff)  /* This register must be set before any I2C bus transaction can take place to ensure proper I/O timing. */

#define RP2040_I2C_IC_SS_SCL_LCNT_MASK                       (0xffff)  /* This register must be set before any I2C bus transaction can take place to ensure proper I/O timing. */

#define RP2040_I2C_IC_FS_SCL_HCNT_MASK                       (0xffff)  /* This register must be set before any I2C bus transaction can take place to ensure proper I/O timing. */

#define RP2040_I2C_IC_FS_SCL_LCNT_MASK                       (0xffff)  /* This register must be set before any I2C bus transaction can take place to ensure proper I/O timing. */

#define RP2040_I2C_IC_INTR_STAT_R_MASTER_ON_HOLD             (1 << 13) /* R_MASTER_ON_HOLD interrupt is active */
#define RP2040_I2C_IC_INTR_STAT_R_RESTART_DET                (1 << 12) /* R_RESTART_DET interrupt is active */
#define RP2040_I2C_IC_INTR_STAT_R_GEN_CALL                   (1 << 11) /* R_GEN_CALL interrupt is active */
#define RP2040_I2C_IC_INTR_STAT_R_START_DET                  (1 << 10) /* R_START_DET interrupt is active */
#define RP2040_I2C_IC_INTR_STAT_R_STOP_DET                   (1 << 9)  /* R_STOP_DET interrupt is active */
#define RP2040_I2C_IC_INTR_STAT_R_ACTIVITY                   (1 << 8)  /* R_ACTIVITY interrupt is active */
#define RP2040_I2C_IC_INTR_STAT_R_RX_DONE                    (1 << 7)  /* R_RX_DONE interrupt is active */
#define RP2040_I2C_IC_INTR_STAT_R_TX_ABRT                    (1 << 6)  /* R_TX_ABRT interrupt is active */
#define RP2040_I2C_IC_INTR_STAT_R_RD_REQ                     (1 << 5)  /* R_RD_REQ interrupt is active */
#define RP2040_I2C_IC_INTR_STAT_R_TX_EMPTY                   (1 << 4)  /* R_TX_EMPTY interrupt is active */
#define RP2040_I2C_IC_INTR_STAT_R_TX_OVER                    (1 << 3)  /* R_TX_OVER interrupt is active */
#define RP2040_I2C_IC_INTR_STAT_R_RX_FULL                    (1 << 2)  /* R_RX_FULL interrupt is active */
#define RP2040_I2C_IC_INTR_STAT_R_RX_OVER                    (1 << 1)  /* R_RX_OVER interrupt is active */
#define RP2040_I2C_IC_INTR_STAT_R_RX_UNDER                   (1 << 0)  /* RX_UNDER interrupt is active */

#define RP2040_I2C_IC_INTR_MASK_M_MASTER_ON_HOLD_READ_ONLY   (1 << 13) /* MASTER_ON_HOLD interrupt is unmasked */
#define RP2040_I2C_IC_INTR_MASK_M_RESTART_DET                (1 << 12) /* RESTART_DET interrupt is unmasked */
#define RP2040_I2C_IC_INTR_MASK_M_GEN_CALL                   (1 << 11) /* GEN_CALL interrupt is unmasked */
#define RP2040_I2C_IC_INTR_MASK_M_START_DET                  (1 << 10) /* START_DET interrupt is unmasked */
#define RP2040_I2C_IC_INTR_MASK_M_STOP_DET                   (1 << 9)  /* STOP_DET interrupt is unmasked */
#define RP2040_I2C_IC_INTR_MASK_M_ACTIVITY                   (1 << 8)  /* ACTIVITY interrupt is unmasked */
#define RP2040_I2C_IC_INTR_MASK_M_RX_DONE                    (1 << 7)  /* RX_DONE interrupt is unmasked */
#define RP2040_I2C_IC_INTR_MASK_M_TX_ABRT                    (1 << 6)  /* TX_ABORT interrupt is unmasked */
#define RP2040_I2C_IC_INTR_MASK_M_RD_REQ                     (1 << 5)  /* RD_REQ interrupt is unmasked */
#define RP2040_I2C_IC_INTR_MASK_M_TX_EMPTY                   (1 << 4)  /* TX_EMPTY interrupt is unmasked */
#define RP2040_I2C_IC_INTR_MASK_M_TX_OVER                    (1 << 3)  /* TX_OVER interrupt is unmasked */
#define RP2040_I2C_IC_INTR_MASK_M_RX_FULL                    (1 << 2)  /* RX_FULL interrupt is unmasked */
#define RP2040_I2C_IC_INTR_MASK_M_RX_OVER                    (1 << 1)  /* RX_OVER interrupt is unmasked */
#define RP2040_I2C_IC_INTR_MASK_M_RX_UNDER                   (1 << 0)  /* RX_UNDER interrupt is unmasked */

#define RP2040_I2C_IC_RAW_INTR_STAT_MASTER_ON_HOLD           (1 << 13) /* MASTER_ON_HOLD interrupt is active */
#define RP2040_I2C_IC_RAW_INTR_STAT_RESTART_DET              (1 << 12) /* RESTART_DET interrupt is active */
#define RP2040_I2C_IC_RAW_INTR_STAT_GEN_CALL                 (1 << 11) /* GEN_CALL interrupt is active */
#define RP2040_I2C_IC_RAW_INTR_STAT_START_DET                (1 << 10) /* START_DET interrupt is active */
#define RP2040_I2C_IC_RAW_INTR_STAT_STOP_DET                 (1 << 9)  /* STOP_DET interrupt is active */
#define RP2040_I2C_IC_RAW_INTR_STAT_ACTIVITY                 (1 << 8)  /* RAW_INTR_ACTIVITY interrupt is active */
#define RP2040_I2C_IC_RAW_INTR_STAT_RX_DONE                  (1 << 7)  /* RX_DONE interrupt is active */
#define RP2040_I2C_IC_RAW_INTR_STAT_TX_ABRT                  (1 << 6)  /* TX_ABRT interrupt is active */
#define RP2040_I2C_IC_RAW_INTR_STAT_RD_REQ                   (1 << 5)  /* RD_REQ interrupt is active */
#define RP2040_I2C_IC_RAW_INTR_STAT_TX_EMPTY                 (1 << 4)  /* TX_EMPTY interrupt is active */
#define RP2040_I2C_IC_RAW_INTR_STAT_TX_OVER                  (1 << 3)  /* TX_OVER interrupt is active */
#define RP2040_I2C_IC_RAW_INTR_STAT_RX_FULL                  (1 << 2)  /* RX_FULL interrupt is active */
#define RP2040_I2C_IC_RAW_INTR_STAT_RX_OVER                  (1 << 1)  /* RX_OVER interrupt is active */
#define RP2040_I2C_IC_RAW_INTR_STAT_RX_UNDER                 (1 << 0)  /* RX_UNDER interrupt is active */

#define RP2040_I2C_IC_RX_TL_RX_TL_MASK                       (0xff)    /* Receive FIFO Threshold Level. */

#define RP2040_I2C_IC_TX_TL_TX_TL_MASK                       (0xff)    /* Transmit FIFO Threshold Level. */

#define RP2040_I2C_IC_CLR_INTR_CLR_INTR                      (1 << 0)  /* Read this register to clear the combined interrupt, all individual interrupts, and the IC_TX_ABRT_SOURCE register. */

#define RP2040_I2C_IC_CLR_RX_UNDER_CLR_RX_UNDER              (1 << 0)  /* Read this register to clear the RX_UNDER interrupt (bit 0) of the IC_RAW_INTR_STAT register. */

#define RP2040_I2C_IC_CLR_RX_OVER_CLR_RX_OVER                (1 << 0)  /* Read this register to clear the RX_OVER interrupt (bit 1) of the IC_RAW_INTR_STAT register. */

#define RP2040_I2C_IC_CLR_TX_OVER_CLR_TX_OVER                (1 << 0)  /* Read this register to clear the TX_OVER interrupt (bit 3) of the IC_RAW_INTR_STAT register. */

#define RP2040_I2C_IC_CLR_RD_REQ_CLR_RD_REQ                  (1 << 0)  /* Read this register to clear the RD_REQ interrupt (bit 5) of the IC_RAW_INTR_STAT register. */

#define RP2040_I2C_IC_CLR_TX_ABRT_CLR_TX_ABRT                (1 << 0)  /* Read this register to clear the TX_ABRT interrupt (bit 6) of the IC_RAW_INTR_STAT register, and the IC_TX_ABRT_SOURCE register. */

#define RP2040_I2C_IC_CLR_RX_DONE_CLR_RX_DONE                (1 << 0)  /* Read this register to clear the RX_DONE interrupt (bit 7) of the IC_RAW_INTR_STAT register. */

#define RP2040_I2C_IC_CLR_ACTIVITY_CLR_ACTIVITY              (1 << 0)  /* Reading this register clears the ACTIVITY interrupt if the I2C is not active anymore. */

#define RP2040_I2C_IC_CLR_STOP_DET_CLR_STOP_DET              (1 << 0)  /* Read this register to clear the STOP_DET interrupt (bit 9) of the IC_RAW_INTR_STAT register. */

#define RP2040_I2C_IC_CLR_START_DET_CLR_START_DET            (1 << 0)  /* Read this register to clear the START_DET interrupt (bit 10) of the IC_RAW_INTR_STAT register. */

#define RP2040_I2C_IC_CLR_GEN_CALL_CLR_GEN_CALL              (1 << 0)  /* Read this register to clear the GEN_CALL interrupt (bit 11) of IC_RAW_INTR_STAT register. */

#define RP2040_I2C_IC_ENABLE_TX_CMD_BLOCK                    (1 << 2)  /* Tx Command execution blocked */
#define RP2040_I2C_IC_ENABLE_ABORT                           (1 << 1)  /* ABORT operation in progress */
#define RP2040_I2C_IC_ENABLE_ENABLE                          (1 << 0)  /* I2C is enabled */

#define RP2040_I2C_IC_STATUS_SLV_ACTIVITY                    (1 << 6)  /* Slave not idle */
#define RP2040_I2C_IC_STATUS_MST_ACTIVITY                    (1 << 5)  /* Master not idle */
#define RP2040_I2C_IC_STATUS_RFF                             (1 << 4)  /* Rx FIFO is full */
#define RP2040_I2C_IC_STATUS_RFNE                            (1 << 3)  /* Rx FIFO not empty */
#define RP2040_I2C_IC_STATUS_TFE                             (1 << 2)  /* Tx FIFO is empty */
#define RP2040_I2C_IC_STATUS_TFNF                            (1 << 1)  /* Tx FIFO not full */
#define RP2040_I2C_IC_STATUS_ACTIVITY                        (1 << 0)  /* I2C is active */

#define RP2040_I2C_IC_TXFLR_TXFLR_MASK                       (0x1f)  /* Transmit FIFO Level. */

#define RP2040_I2C_IC_RXFLR_RXFLR_MASK                       (0x1f)  /* Receive FIFO Level. */

#define RP2040_I2C_IC_SDA_HOLD_IC_SDA_RX_HOLD_SHIFT          (16)      /* Sets the required SDA hold time in units of ic_clk period, when DW_apb_i2c acts as a receiver. */
#define RP2040_I2C_IC_SDA_HOLD_IC_SDA_RX_HOLD_MASK           (0xff << RP2040_I2C_IC_SDA_HOLD_IC_SDA_RX_HOLD_SHIFT)
#define RP2040_I2C_IC_SDA_HOLD_IC_SDA_TX_HOLD_MASK           (0xffff)  /* Sets the required SDA hold time in units of ic_clk period, when DW_apb_i2c acts as a transmitter. */

#define RP2040_I2C_IC_TX_ABRT_SOURCE_TX_FLUSH_CNT_SHIFT      (23)      /* This field indicates the number of Tx FIFO Data Commands which are flushed due to TX_ABRT interrupt. */
#define RP2040_I2C_IC_TX_ABRT_SOURCE_TX_FLUSH_CNT_MASK       (0x1ff << RP2040_I2C_IC_TX_ABRT_SOURCE_TX_FLUSH_CNT_SHIFT)
#define RP2040_I2C_IC_TX_ABRT_SOURCE_ABRT_USER_ABRT          (1 << 16) /* Transfer abort detected by master */
#define RP2040_I2C_IC_TX_ABRT_SOURCE_ABRT_SLVRD_INTX         (1 << 15) /* Slave trying to transmit to remote master in read mode */
#define RP2040_I2C_IC_TX_ABRT_SOURCE_ABRT_SLV_ARBLOST        (1 << 14) /* Slave lost arbitration to remote master */
#define RP2040_I2C_IC_TX_ABRT_SOURCE_ABRT_SLVFLUSH_TXFIFO    (1 << 13) /* Slave flushes existing data in TX-FIFO upon getting read command */
#define RP2040_I2C_IC_TX_ABRT_SOURCE_ARB_LOST                (1 << 12) /* Master or Slave-Transmitter lost arbitration */
#define RP2040_I2C_IC_TX_ABRT_SOURCE_ABRT_MASTER_DIS         (1 << 11) /* User initiating master operation when MASTER disabled */
#define RP2040_I2C_IC_TX_ABRT_SOURCE_ABRT_10B_RD_NORSTRT     (1 << 10) /* Master trying to read in 10Bit addressing mode when RESTART disabled */
#define RP2040_I2C_IC_TX_ABRT_SOURCE_ABRT_SBYTE_NORSTRT      (1 << 9)  /* User trying to send START byte when RESTART disabled */
#define RP2040_I2C_IC_TX_ABRT_SOURCE_ABRT_HS_NORSTRT         (1 << 8)  /* User trying to switch Master to HS mode when RESTART disabled */
#define RP2040_I2C_IC_TX_ABRT_SOURCE_ABRT_SBYTE_ACKDET       (1 << 7)  /* ACK detected for START byte */
#define RP2040_I2C_IC_TX_ABRT_SOURCE_ABRT_HS_ACKDET          (1 << 6)  /* HS Master code ACKed in HS Mode */
#define RP2040_I2C_IC_TX_ABRT_SOURCE_ABRT_GCALL_READ         (1 << 5)  /* GCALL is followed by read from bus */
#define RP2040_I2C_IC_TX_ABRT_SOURCE_ABRT_GCALL_NOACK        (1 << 4)  /* GCALL not ACKed by any slave */
#define RP2040_I2C_IC_TX_ABRT_SOURCE_ABRT_TXDATA_NOACK       (1 << 3)  /* Transmitted data not ACKed by addressed slave */
#define RP2040_I2C_IC_TX_ABRT_SOURCE_ABRT_10ADDR2_NOACK      (1 << 2)  /* Byte 2 of 10Bit Address not ACKed by any slave */
#define RP2040_I2C_IC_TX_ABRT_SOURCE_ABRT_10ADDR1_NOACK      (1 << 1)  /* Byte 1 of 10Bit Address not ACKed by any slave */
#define RP2040_I2C_IC_TX_ABRT_SOURCE_ABRT_7B_ADDR_NOACK      (1 << 0)  /* This abort is generated because of NOACK for 7-bit address */

#define RP2040_I2C_IC_SLV_DATA_NACK_ONLY_NACK                (1 << 0)  /* Slave receiver generates NACK upon data reception only */

#define RP2040_I2C_IC_DMA_CR_TDMAE                           (1 << 1)  /* Transmit FIFO DMA channel enabled */
#define RP2040_I2C_IC_DMA_CR_RDMAE                           (1 << 0)  /* Receive FIFO DMA channel enabled */

#define RP2040_I2C_IC_DMA_TDLR_DMATDL_MASK                   (0x0f)    /* Transmit Data Level. */

#define RP2040_I2C_IC_DMA_RDLR_DMARDL_MASK                   (0x0f)    /* Receive Data Level. */

#define RP2040_I2C_IC_SDA_SETUP_SDA_SETUP_MASK               (0xff)    /* SDA Setup. */

#define RP2040_I2C_IC_ACK_GENERAL_CALL_ACK_GEN_CALL          (1 << 0)  /* Generate ACK for a General Call */

#define RP2040_I2C_IC_ENABLE_STATUS_SLV_RX_DATA_LOST         (1 << 2)  /* Slave RX Data is lost */
#define RP2040_I2C_IC_ENABLE_STATUS_SLV_DISABLED_WHILE_BUSY  (1 << 1)  /* Slave is disabled when it is active */
#define RP2040_I2C_IC_ENABLE_STATUS_IC_EN                    (1 << 0)  /* I2C enabled */

#define RP2040_I2C_IC_FS_SPKLEN_MASK                         (0xff)    /* This register must be set before any I2C bus transaction can take place to ensure stable operation. */

#define RP2040_I2C_IC_CLR_RESTART_DET_CLR_RESTART_DET        (1 << 0)  /* Read this register to clear the RESTART_DET interrupt (bit 12) of IC_RAW_INTR_STAT register. */

#define RP2040_I2C_IC_COMP_PARAM_1_TX_BUFFER_DEPTH_SHIFT     (16)      /* TX Buffer Depth = 16 */
#define RP2040_I2C_IC_COMP_PARAM_1_TX_BUFFER_DEPTH_MASK      (0xff << RP2040_I2C_IC_COMP_PARAM_1_TX_BUFFER_DEPTH_SHIFT)
#define RP2040_I2C_IC_COMP_PARAM_1_RX_BUFFER_DEPTH_SHIFT     (8)       /* RX Buffer Depth = 16 */
#define RP2040_I2C_IC_COMP_PARAM_1_RX_BUFFER_DEPTH_MASK      (0xff << RP2040_I2C_IC_COMP_PARAM_1_RX_BUFFER_DEPTH_SHIFT)
#define RP2040_I2C_IC_COMP_PARAM_1_ADD_ENCODED_PARAMS        (1 << 7)  /* Encoded parameters not visible */
#define RP2040_I2C_IC_COMP_PARAM_1_HAS_DMA                   (1 << 6)  /* DMA handshaking signals are enabled */
#define RP2040_I2C_IC_COMP_PARAM_1_INTR_IO                   (1 << 5)  /* COMBINED Interrupt outputs */
#define RP2040_I2C_IC_COMP_PARAM_1_HC_COUNT_VALUES           (1 << 4)  /* Programmable count values for each mode. */
#define RP2040_I2C_IC_COMP_PARAM_1_MAX_SPEED_MODE_SHIFT      (2)       /* MAX SPEED MODE = FAST MODE */
#define RP2040_I2C_IC_COMP_PARAM_1_MAX_SPEED_MODE_MASK       (0x03 << RP2040_I2C_IC_COMP_PARAM_1_MAX_SPEED_MODE_SHIFT)
#define RP2040_I2C_IC_COMP_PARAM_1_APB_DATA_WIDTH_MASK       (0x03)    /* APB data bus width is 32 bits */

#endif /* __ARCH_ARM_SRC_RP2040_HARDWARE_RP2040_I2C_H */
