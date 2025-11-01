/****************************************************************************
 * arch/risc-v/src/mpfs/hardware/mpfs_can.h
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

#ifndef __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_CAN_H
#define __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_CAN_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MPFS_CAN_INT_STATUS_OFFSET                  (0x000)
#define MPFS_CAN_INT_ENABLE_OFFSET                  (0x004)
#define MPFS_CAN_RX_BUF_STATUS_OFFSET               (0x008)
#define MPFS_CAN_TX_BUF_STATUS_OFFSET               (0x00c)
#define MPFS_CAN_ERROR_STATUS_OFFSET                (0x010)
#define MPFS_CAN_CAN_COMMAND_OFFSET                 (0x014)
#define MPFS_CAN_CAN_CONFIG_OFFSET                  (0x018)
#define MPFS_CAN_ECR_OFFSET                         (0x01c)
#define MPFS_CAN_TX_MSG_OFFSET                      (0x020)
#define MPFS_CAN_TX_MSG_CTRL_CMD_SHIFT              (0)
#define MPFS_CAN_TX_MSG_ID_SHIFT                    (4)
#define MPFS_CAN_TX_MSG_DATA_HIGH_SHIFT             (8)
#define MPFS_CAN_TX_MSG_DATA_LOW_SHIFT              (12)
#define MPFS_CAN_TX_MSG_TOTAL_SIZE                  (16)
#define MPFS_CAN_RX_MSG_OFFSET                      (0x220)
#define MPFS_CAN_RX_MSG_CTRL_CMD_SHIFT              (0)
#define MPFS_CAN_RX_MSG_ID_SHIFT                    (4)
#define MPFS_CAN_RX_MSG_DATA_HIGH_SHIFT             (8)
#define MPFS_CAN_RX_MSG_DATA_LOW_SHIFT              (12)
#define MPFS_CAN_RX_MSG_AMR_SHIFT                   (16)
#define MPFS_CAN_RX_MSG_ACR_SHIFT                   (20)
#define MPFS_CAN_RX_MSG_AMR_DATA_SHIFT              (24)
#define MPFS_CAN_RX_MSG_ACR_DATA_SHIFT              (28)
#define MPFS_CAN_RX_MSG_TOTAL_SIZE                  (32)

/* Interrupt status register */
#define MPFS_CAN_INT_STATUS_SST_FAILURE             (1 << 15)  /* Single-shot tx err */
#define MPFS_CAN_INT_STATUS_STUCK_AT_0              (1 << 14)  /* Stuck at 0 err */
#define MPFS_CAN_INT_STATUS_RTR_MSG                 (1 << 13)  /* RTR autorep msg sent */
#define MPFS_CAN_INT_STATUS_RX_MSG                  (1 << 12)  /* RX msg available */
#define MPFS_CAN_INT_STATUS_TX_MSG                  (1 << 11)  /* Msg transmitted */
#define MPFS_CAN_INT_STATUS_RX_MSG_LOSS             (1 << 10)  /* RX msg loss */
#define MPFS_CAN_INT_STATUS_BUS_OFF                 (1 << 9)   /* Bus Off */
#define MPFS_CAN_INT_STATUS_CRC_ERR                 (1 << 8)   /* CRC err */
#define MPFS_CAN_INT_STATUS_FORM_ERR                (1 << 7)   /* Format err */
#define MPFS_CAN_INT_STATUS_ACK_ERR                 (1 << 6)   /* Acknowledge err */
#define MPFS_CAN_INT_STATUS_STUFF_ERR               (1 << 5)   /* Bit stuffing err */
#define MPFS_CAN_INT_STATUS_BIT_ERR                 (1 << 4)   /* Bit err */
#define MPFS_CAN_INT_STATUS_OVR_LOAD                (1 << 3)   /* Overload msg err */
#define MPFS_CAN_INT_STATUS_ARB_LOSS                (1 << 2)   /* Arbitration loss err */

/* Interrupt enable register */
#define MPFS_CAN_INT_ENABLE_SST_FAILURE_INT_ENBL    (1 << 15)  /* Single shot transmission failure interrupt enable */
#define MPFS_CAN_INT_ENABLE_STUCK_AT_0_INT_ENBL     (1 << 14)  /* Stuck at dominant error interrupt enable */
#define MPFS_CAN_INT_ENABLE_RTR_MSG_INT_ENBL        (1 << 13)  /* RTR auto-reply message sent interrupt enable */
#define MPFS_CAN_INT_ENABLE_RX_MSG_INT_ENBL         (1 << 12)  /* Receive message available interrupt enable */
#define MPFS_CAN_INT_ENABLE_TX_MSG_INT_ENBL         (1 << 11)  /* Message transmitted interrupt enable */
#define MPFS_CAN_INT_ENABLE_RX_MSG_LOSS_INT_ENBL    (1 << 10)  /* Received message loss interrupt enable */
#define MPFS_CAN_INT_ENABLE_BUS_OFF_INT_ENBL        (1 << 9)   /* Bus off interrupt enable */
#define MPFS_CAN_INT_ENABLE_CRC_ERR_INT_ENBL        (1 << 8)   /* CRC error interrupt enable */
#define MPFS_CAN_INT_ENABLE_FORM_ERR_INT_ENBL       (1 << 7)   /* Format error interrupt enable */
#define MPFS_CAN_INT_ENABLE_ACK_ERR_INT_ENBL        (1 << 6)   /* Acknowledge error interrupt enable */
#define MPFS_CAN_INT_ENABLE_STUFF_ERR_INT_ENBL      (1 << 5)   /* Bit stuffing error interrupt enable */
#define MPFS_CAN_INT_ENABLE_BIT_ERR_INT_ENBL        (1 << 4)   /* Bit error interrupt enable */
#define MPFS_CAN_INT_ENABLE_OVR_LOAD_INT_ENBL       (1 << 3)   /* Overload message detected interrupt enable */
#define MPFS_CAN_INT_ENABLE_ARB_LOSS_INT_ENBL       (1 << 2)   /* Arbitration loss interrupt enable */
#define MPFS_CAN_INT_ENABLE_INT_ENBL                (1 << 0)   /* Global interrupt enable flag */

/* Error status register */
#define MPFS_CAN_ERROR_STATUS_RXGTE96               (1 << 19)
#define MPFS_CAN_ERROR_STATUS_TXGTE96               (1 << 18)
#define MPFS_CAN_ERROR_STATUS_ERROR_STATE_SHIFT     (16)
#define MPFS_CAN_ERROR_STATUS_ERROR_STATE           (0x03 << MPFS_CAN_ERROR_STATUS_ERROR_STATE_SHIFT)
#define MPFS_CAN_ERROR_STATUS_RX_ERR_CNT_SHIFT      (8)
#define MPFS_CAN_ERROR_STATUS_RX_ERR_CNT            (0xff << MPFS_CAN_ERROR_STATUS_RX_ERR_CNT_SHIFT)
#define MPFS_CAN_ERROR_STATUS_TX_ERR_CNT_SHIFT      (0)
#define MPFS_CAN_ERROR_STATUS_TX_ERR_CNT            (0xff << MPFS_CAN_ERROR_STATUS_TX_ERR_CNT_SHIFT)

/* CAN command register */
#define MPFS_CAN_CAN_COMMAND_REVISION_CONTROL_SHIFT (16)
#define MPFS_CAN_CAN_COMMAND_REVISION_CONTROL       (0xffff << MPFS_CAN_CAN_COMMAND_REVISION_CONTROL_SHIFT)
#define MPFS_CAN_CAN_COMMAND_SRAM_TEST_MODE         (1 << 3)
#define MPFS_CAN_CAN_COMMAND_LOOPBACK_TEST_MODE     (1 << 2)
#define MPFS_CAN_CAN_COMMAND_LISTEN_ONLY_MODE       (1 << 1)
#define MPFS_CAN_CAN_COMMAND_RUN_STOP_MODE          (1 << 0)

/* CAN config register */
#define MPFS_CAN_CAN_CONFIG_CFG_BITRATE_SHIFT       (16)
#define MPFS_CAN_CAN_CONFIG_CFG_BITRATE             (0x7fff << MPFS_CAN_CAN_CONFIG_CFG_BITRATE_SHIFT)
#define MPFS_CAN_CAN_CONFIG_ECR_MODE                (1 << 14)
#define MPFS_CAN_CAN_CONFIG_SWAP_ENDIAN             (1 << 13)
#define MPFS_CAN_CAN_CONFIG_CFG_ARBITER             (1 << 12)
#define MPFS_CAN_CAN_CONFIG_CFG_TSEG1_SHIFT         (8)
#define MPFS_CAN_CAN_CONFIG_CFG_TSEG1               (0x0f << MPFS_CAN_CAN_CONFIG_CFG_TSEG1_SHIFT)
#define MPFS_CAN_CAN_CONFIG_CFG_TSEG2_SHIFT         (5)
#define MPFS_CAN_CAN_CONFIG_CFG_TSEG2               (0x07 << MPFS_CAN_CAN_CONFIG_CFG_TSEG2_SHIFT)
#define MPFS_CAN_CAN_CONFIG_AUTO_RESTART            (1 << 4)
#define MPFS_CAN_CAN_CONFIG_CFG_SJW_SHIFT           (2)
#define MPFS_CAN_CAN_CONFIG_CFG_SJW                 (0x03 << MPFS_CAN_CAN_CONFIG_CFG_SJW_SHIFT)
#define MPFS_CAN_CAN_CONFIG_SAMPLING_MODE           (1 << 1)
#define MPFS_CAN_CAN_CONFIG_EDGE_MODE               (1 << 0)

/* Msg ID */
#define MPFS_CAN_MSG_ID_SHIFT                       (3)
#define MPFS_CAN_MSG_IDE_SHIFT                      (18)

/* TX msg control command register */
#define MPFS_CAN_TX_MSG_CTRL_CMD_WPN_B              (1 << 23)  /* Write protect not B */
#define MPFS_CAN_TX_MSG_CTRL_CMD_RTR                (1 << 21)  /* RTR bit; 0: Regular message, 1: RTR message */
#define MPFS_CAN_TX_MSG_CTRL_CMD_IDE                (1 << 20)  /* Extended identifier bit; 0: Standard format, 1: Extended format */
#define MPFS_CAN_TX_MSG_CTRL_CMD_DLC_SHIFT          (16)       /* Data length code shift */
#define MPFS_CAN_TX_MSG_CTRL_CMD_DLC                (0x0f << MPFS_CAN_TX_MSG_CTRL_CMD_DLC_SHIFT)
                                                               /* Data length code; 0-8: Number of data bytes, 9-15: 8 data bytes */
#define MPFS_CAN_TX_MSG_CTRL_CMD_WPN_A              (1 << 3)   /* Write protect not A */
#define MPFS_CAN_TX_MSG_CTRL_CMD_TX_INT_EBL         (1 << 2)   /* Transmit interrupt enable; 0: Disabled, 1: Enabled */
#define MPFS_CAN_TX_MSG_CTRL_CMD_TX_ABORT           (1 << 1)   /* Transmit abort; 0: Idle, 1: Request abort */
#define MPFS_CAN_TX_MSG_CTRL_CMD_TX_REQ             (1 << 0)   /* Transmit request; 0: Idle, 1: Request transmission */

/* RX msg control command register */
#define MPFS_CAN_RX_MSG_CTRL_CMD_WPNH               (1 << 23)  /* Write protect not high */
#define MPFS_CAN_RX_MSG_CTRL_CMD_RTR                (1 << 21)  /* RTR bit; 0: Regular message, 1: RTR message */
#define MPFS_CAN_RX_MSG_CTRL_CMD_IDE                (1 << 20)  /* Extended identifier bit; 0: Standard format, 1: Extended format */
#define MPFS_CAN_RX_MSG_CTRL_CMD_DLC_SHIFT          (16)       /* Data length code shift */
#define MPFS_CAN_RX_MSG_CTRL_CMD_DLC                (0x0f << MPFS_CAN_RX_MSG_CTRL_CMD_DLC_SHIFT)
                                                               /* Data length code; 0-8: Number of data bytes, 9-15: 8 data bytes */
#define MPFS_CAN_RX_MSG_CTRL_CMD_WPNL               (1 << 7)   /* Write protect not low */
#define MPFS_CAN_RX_MSG_CTRL_CMD_LF                 (1 << 6)   /* Link flag; 0: Not linked, 1: Linked with next buffer */
#define MPFS_CAN_RX_MSG_CTRL_CMD_RX_INT_ENABLE      (1 << 5)   /* Receive interrupt enable; 0: Disabled, 1: Enabled */
#define MPFS_CAN_RX_MSG_CTRL_CMD_RTR_REPLY          (1 << 4)   /* Automatic RTR message reply; 0: Disabled, 1: Enabled */
#define MPFS_CAN_RX_MSG_CTRL_CMD_RX_BUFFER_EBL      (1 << 3)   /* Transaction buffer enable; 0: Disabled, 1: Enabled */
#define MPFS_CAN_RX_MSG_CTRL_CMD_RTR_ABORT          (1 << 2)   /* RTR abort request; 0: Idle, 1: Request removal of pending RTR message */
#define MPFS_CAN_RX_MSG_CTRL_CMD_RTRP               (1 << 1)   /* RTR reply pending; 0: No request pending, 1: Request pending */
#define MPFS_CAN_RX_MSG_CTRL_CMD_MSGAV_RTRS         (1 << 0)   /* Message available/RTR sent; 0: Idle, 1: New message available or RTR auto-reply sent */

/* Hardware filter code (ACR) and mask (AMR) registers */
#define MPFS_CAN_ACR_AMR_ID_SHIFT                   (3)        /* Identifier bits shift */
#define MPFS_CAN_ACR_AMR_ID                         (0x1fffffff << MPFS_CAN_RX_MSG_AMR_ID_SHIFT)
                                                               /* Identifier bits [31:3] */
#define MPFS_CAN_ACR_AMR_IDE                        (1 << 2)   /* IDE bit; 0: Check against ACR, 1: Don't care */
#define MPFS_CAN_ACR_AMR_RTR                        (1 << 1)   /* RTR bit; 0: Check against ACR, 1: Don't care */

#endif /* __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_CAN_H */
