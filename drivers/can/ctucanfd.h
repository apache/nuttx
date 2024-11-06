/****************************************************************************
 * drivers/can/ctucanfd.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __DRIVERS_CAN_CTUCANFD_H
#define __DRIVERS_CAN_CTUCANFD_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CAN FD Core memory map */

#define CTUCANFD_CTRL                (0x000)
#define CTUCANFD_TXT1                (0x100)
#define CTUCANFD_TXT2                (0x200)
#define CTUCANFD_TXT3                (0x300)
#define CTUCANFD_TXT4                (0x400)
#define CTUCANFD_TXT5                (0x500)
#define CTUCANFD_TXT6                (0x600)
#define CTUCANFD_TXT7                (0x700)
#define CTUCANFD_TXT8                (0x800)
#define CTUCANFD_TXT_SIZE            (0x100)

/* Control registers */

#define CTUCANFD_VERID               (0x00)
#define CTUCANFD_SET_MODE            (0x04)
#define CTUCANFD_STATUS              (0x08)
#define CTUCANFD_CMD                 (0x0c)
#define CTUCANFD_INTSTAT             (0x10)
#define CTUCANFD_INTENSET            (0x14)
#define CTUCANFD_INTENCLR            (0x18)
#define CTUCANFD_INTMASKSET          (0x1c)
#define CTUCANFD_INTMASKCLR          (0x20)
#define CTUCANFD_BTR                 (0x24)
#define CTUCANFD_BTRFD               (0x28)
#define CTUCANFD_FAULT               (0x2c)
#define CTUCANFD_ERRCNT              (0x30)
#define CTUCANFD_ERR                 (0x34)
#define CTUCANFD_CTRPRES             (0x38)
#define CTUCANFD_FLTR_A_MSK          (0x3c)
#define CTUCANFD_FLTR_A_VAL          (0x40)
#define CTUCANFD_FLTR_B_MSK          (0x44)
#define CTUCANFD_FLTR_B_VAL          (0x48)
#define CTUCANFD_FLTR_C_MSK          (0x4c)
#define CTUCANFD_FLTR_C_VAL          (0x50)
#define CTUCANFD_FLTR_RAN_L          (0x54)
#define CTUCANFD_FLTR_RAN_H          (0x58)
#define CTUCANFD_FLTR                (0x5c)
#define CTUCANFD_RXMEMINFO           (0x60)
#define CTUCANFD_RXPOINTERS          (0x64)
#define CTUCANFD_RXSETSTAT           (0x68)
#define CTUCANFD_RXDATA              (0x6c)
#define CTUCANFD_TXSTAT              (0x70)
#define CTUCANFD_TXINFOCMD           (0x74)
#define CTUCANFD_TXPRIO              (0x78)
#define CTUCANFD_MISC                (0x7c)
#define CTUCANFD_SSP                 (0x80)
#define CTUCANFD_RXFRCTR             (0x84)
#define CTUCANFD_TXFRCTR             (0x88)
#define CTUCANFD_DEBUG               (0x8c)
#define CTUCANFD_YOLO                (0x90)
#define CTUCANFD_TS_L                (0x94)
#define CTUCANFD_TS_H                (0x98)

/* Mode */

#define CTUCANFD_MODE_RST            (1 << 0)  /* Soft reset */
#define CTUCANFD_MODE_BMM            (1 << 1)  /* Bus monitoring mode */
#define CTUCANFD_MODE_STM            (1 << 2)  /* Self test mode */
#define CTUCANFD_MODE_AFM            (1 << 3)  /* Acceptance filters mode */
#define CTUCANFD_MODE_FDE            (1 << 4)  /* Flexible data rate mode */
#define CTUCANFD_MODE_TTTM           (1 << 5)  /* Time triggered transmission mode */
#define CTUCANFD_MODE_ROM            (1 << 6)  /* Restricted operation mode */
#define CTUCANFD_MODE_ACF            (1 << 7)  /* Acknowledge forbidden mode */
#define CTUCANFD_MODE_TSTM           (1 << 8)  /* Test mode */
#define CTUCANFD_MODE_RXBAM          (1 << 9)  /* TX buffer automatic mode */
#define CTUCANFD_MODE_TXBBM          (1 << 10) /* TXT buffer backup mode */
#define CTUCANFD_MODE_SAM            (1 << 11) /* Self-acknowledge mode */
#define CTUCANFD_MODE_ERFM           (1 << 12) /* Error frame receive mode */

/* Settings */

#define CTUCANFD_SET_SHFIT           (16)      /* Shift in 32-bit access mode */

#define CTUCANFD_SET_RTRLE           (1 << 0)  /* Retransmit limit enable */
#define CTUCANFD_SET_RTRTH_SHIFT     (1)       /* Retransmit limit threshold shift */
#define CTUCANFD_SET_ILBP            (1 << 5)  /* Internal loop back mode */
#define CTUCANFD_SET_ENA             (1 << 6)  /* Main enable bif of CTU CAN FD */
#define CTUCANFD_SET_NISOFD          (1 << 7)  /* Non ISO FD */
#define CTUCANFD_SET_PEX             (1 << 8)  /* Protocol exception handling */
#define CTUCANFD_SET_TBFBO           (1 << 9)  /* TX failed for all buffers when bus-off */
#define CTUCANFD_SET_FDRF            (1 << 10) /* Frame filter drop remote frames */
#define CTUCANFD_SET_PCHKE           (1 << 11) /* Enable parity checks in TXT and RX buffers */

/* Command */

#define CTUCANFD_CMD_RXRPMV          (1 << 1)  /* RX buffer read pointer move */
#define CTUCANFD_CMD_RRB             (1 << 2)  /* Release RX buffer */
#define CTUCANFD_CMD_CDO             (1 << 3)  /* Clear data overrun flag in RX buffer */
#define CTUCANFD_CMD_ERCRST          (1 << 4)  /* Error counters reset */
#define CTUCANFD_CMD_RXFCRST         (1 << 5)  /* Clear RX bus traffic counter */
#define CTUCANFD_CMD_TXFCRST         (1 << 6)  /* Clear TX bus traffic counter */
#define CTUCANFD_CMD_CPEXS           (1 << 7)  /* Clear protocol exception status */
#define CTUCANFD_CMD_CRXPE           (1 << 8)  /* Clear STATUS[RXPE] flag */
#define CTUCANFD_CMD_CTXPE           (1 << 9)  /* Clear STATUS[TXPE] flag */
#define CTUCANFD_CMD_CTXDPE          (1 << 10) /* Clear STATUS[TXDPE] flag */

/* Interrupts */

#define CTUCANFD_INT_RXI             (1 << 0)  /* Frame received interrupt */
#define CTUCANFD_INT_TXI             (1 << 1)  /* Frame transmitted interrupt */
#define CTUCANFD_INT_EWLI            (1 << 2)  /* Error warning limit interrupt */
#define CTUCANFD_INT_DOI             (1 << 3)  /* Data overrun interrupt */
#define CTUCANFD_INT_FCSI            (1 << 4)  /* Fault confinement state changed interrupt */
#define CTUCANFD_INT_ALI             (1 << 5)  /* Arbitration lost interrupt */
#define CTUCANFD_INT_BEI             (1 << 6)  /* Bus error interrupt */
#define CTUCANFD_INT_OFI             (1 << 7)  /* Overload frame interrupt */
#define CTUCANFD_INT_RXFI            (1 << 8)  /* RX buffer full interrupt */
#define CTUCANFD_INT_BSI             (1 << 9)  /* Bit rate shifted interrupt */
#define CTUCANFD_INT_RBNEI           (1 << 10) /* RX buffer not empty interrupt */
#define CTUCANFD_INT_TXBHCI          (1 << 11) /* TXT buffer HW command interrupt */

/* RX_STATUS and RX_SETTINGS */

#define CTUCANFD_RXSTAT_RXE          (1 << 0) /* RX buffer is empty */
#define CTUCANFD_RXSTAT_RXF          (1 << 1) /* RX buffer is full */
#define CTUCANFD_RXSTAT_RXMOF        (1 << 2) /* RX buffer middle of frame */
#define CTUCANFD_RXSTAT_RXFRC_SHIFT  (4)      /* RX buffer frame count */
#define CTUCANFD_RXSTAT_RXFRC_MASK   (0x7ff << CTUCANFD_RXSTAT_RXFRC_SHIFT)

/* TX_STATUS */

#define CTUCANFD_TXSTAT_SHIFT        (4)   /* TXyS shift */
#define CTUCANFD_TXSTAT_MASK         (0xf) /* TXyS mask */
#define CTUCANFD_TXSTAT_GET(val, i)  (((val) >> (CTUCANFD_TXSTAT_SHIFT * (i))) & \
                                      CTUCANFD_TXSTAT_MASK)
#define CTUCANFD_TXSTAT_NOTEXIST     (0)   /* TXT buffer doesn't exist */
#define CTUCANFD_TXSTAT_RDY          (1)   /* "Ready" state */
#define CTUCANFD_TXSTAT_TRAN         (2)   /* "TX in porgress" state */
#define CTUCANFD_TXSTAT_ABTP         (3)   /* "Abort in progress" state */
#define CTUCANFD_TXSTAT_TOK          (4)   /* "TX OK" state */
#define CTUCANFD_TXSTAT_ERR          (6)   /* "Failed" state */
#define CTUCANFD_TXSTAT_ABT          (7)   /* "Aborted" state */
#define CTUCANFD_TXSTAT_ETY          (8)   /* "Empty" state */
#define CTUCANFD_TXSTAT_PER          (9)   /* "Parity error" state */

/* TX_COMMAND and TXTB_INFO */

#define CTUCANFD_TXCMD_TXCE          (1 << 0)  /* "set empty" */
#define CTUCANFD_TXCMD_TXCR          (1 << 1)  /* "set ready" */
#define CTUCANFD_TXCMD_TXCA          (1 << 2)  /* "set abort" */
#define CTUCANFD_TXCMD_TXB_SHIFT     (8)
#define CTUCANFD_TXCMD_TXB1          (1 << 8)  /* TXT buffer 1 */
#define CTUCANFD_TXCMD_TXB2          (1 << 9)  /* TXT buffer 2 */
#define CTUCANFD_TXCMD_TXB3          (1 << 10) /* TXT buffer 3 */
#define CTUCANFD_TXCMD_TXB4          (1 << 11) /* TXT buffer 4 */
#define CTUCANFD_TXCMD_TXB5          (1 << 12) /* TXT buffer 5 */
#define CTUCANFD_TXCMD_TXB6          (1 << 13) /* TXT buffer 6 */
#define CTUCANFD_TXCMD_TXB7          (1 << 14) /* TXT buffer 7 */
#define CTUCANFD_TXCMD_TXB8          (1 << 15) /* TXT buffer 8 */

/* TX_PRIORITY */

#define CTUCANFD_TXPRIO_SHIFT        (4)

/* TX buffer */

#define CTUCANFD_TXBUF_FMT           (0x00)
#define CTUCANFD_TXBUF_ID            (0x04)
#define CTUCANFD_TXBUF_TSL           (0x08)
#define CTUCANFD_TXBUF_TSU           (0x0c)
#define CTUCANFD_TXBUF_DATA          (0x10)
#define CTUCANFD_TXBUF_TEST          (0x05)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* CAN FD frame format */

begin_packed_struct struct ctucanfd_frame_fmt_s
{
  uint32_t dlc:4;               /* DLC */
  uint32_t erf:1;               /* Error frame flag */
  uint32_t rtr:1;               /* Remote frame flag */
  uint32_t ide:1;               /* Extended identifier type */
  uint32_t fdf:1;               /* Flexible data-rate format */
  uint32_t lbpf:1;              /* Loop-back frame */
  uint32_t brs:1;               /* Bit rate shift */
  uint32_t esi_rsv:1;           /* Error state indicator */
  uint32_t rwcnt:4;             /* Size without FRAME_FORMAT WORD */
  uint32_t erf_pos:4;           /* Error frame position */
  uint32_t erf_erp:1;           /* Error pasive state */
  uint32_t erf_type:3;          /* Error frame type */
  uint32_t ivld:1;              /* Valid identifier */
  uint32_t lbtbi:3;             /* Loop-back TXT index */
  uint32_t _reserved:4;         /* Reserved */
} end_packed_struct;

union ctucanfd_frame_fmt_u
{
  struct ctucanfd_frame_fmt_s s;
  uint32_t                    u32;
};

/* CAN FD frame ID */

begin_packed_struct struct ctucanfd_frame_id_s
{
  uint32_t id_ext:18;           /* Extended identifier */
  uint32_t id:11;               /* Base identifier */
  uint32_t _res:3;              /* Reserved */
} end_packed_struct;

union ctucanfd_frame_id_u
{
  struct ctucanfd_frame_id_s s;
  uint32_t                   u32;
};

/* CAN FD frame test */

begin_packed_struct struct ctucanfd_frame_tst_s
{
  uint32_t fstc:1;              /* Flip stuff count field bit */
  uint32_t fcrc:1;              /* Flip CRC field bit */
  uint32_t sdlc:1;              /* Swap DLC in TX */
  uint32_t _res1:4;             /* Reserved */
  uint32_t tprm:4;              /* Test parameter */
  uint32_t _res2:3;             /* Reserved */
} end_packed_struct;

/* CAN FD frame format */

begin_packed_struct struct ctucanfd_frame_s
{
  struct ctucanfd_frame_fmt_s fmt;       /* Frame format */
  struct ctucanfd_frame_id_s  id;        /* Frame ID */
  uint64_t                    timestamp; /* Frame timestamp */
  uint8_t                     data[64];  /* Frame data */
  uint32_t                    test;      /* Frame test */
} end_packed_struct;

#endif /* __DRIVERS_CAN_CTUCANFD_H */
