/****************************************************************************
 * arch/arm/src/lpc54xx/hardware/lpc54_sdmmc.h
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

#ifndef __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC54_SDMMC_H
#define __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC54_SDMMC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/lpc54_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LPC54_TXFIFO_DEPTH               32
#define LPC54_TXFIFO_WIDTH               4
#define LPC54_RXFIFO_DEPTH               32
#define LPC54_RXFIFO_WIDTH               4

/* MCI register offsets (with respect to the MCI base) **********************/

#define LPC54_SDMMC_CTRL_OFFSET          0x0000 /* Control register */
#define LPC54_SDMMC_PWREN_OFFSET         0x0004 /* Power Enable Register */
#define LPC54_SDMMC_CLKDIV_OFFSET        0x0008 /* Clock-divider register */
#define LPC54_SDMMC_CLKENA_OFFSET        0x0010 /* Clock-enable register */
#define LPC54_SDMMC_TMOUT_OFFSET         0x0014 /* Time-out register */
#define LPC54_SDMMC_CTYPE_OFFSET         0x0018 /* Card-type register */
#define LPC54_SDMMC_BLKSIZ_OFFSET        0x001c /* Block-size register */
#define LPC54_SDMMC_BYTCNT_OFFSET        0x0020 /* Byte-count register */
#define LPC54_SDMMC_INTMASK_OFFSET       0x0024 /* Interrupt-mask register */
#define LPC54_SDMMC_CMDARG_OFFSET        0x0028 /* Command-argument register */
#define LPC54_SDMMC_CMD_OFFSET           0x002c /* Command register */
#define LPC54_SDMMC_RESP0_OFFSET         0x0030 /* Response-0 register */
#define LPC54_SDMMC_RESP1_OFFSET         0x0034 /* Response-1 register */
#define LPC54_SDMMC_RESP2_OFFSET         0x0038 /* Response-2 register */
#define LPC54_SDMMC_RESP3_OFFSET         0x003c /* Response-3 register */
#define LPC54_SDMMC_MINTSTS_OFFSET       0x0040 /* Masked interrupt-status register */
#define LPC54_SDMMC_RINTSTS_OFFSET       0x0044 /* Raw interrupt-status register */
#define LPC54_SDMMC_STATUS_OFFSET        0x0048 /* Status register */
#define LPC54_SDMMC_FIFOTH_OFFSET        0x004c /* FIFO threshold register */
#define LPC54_SDMMC_CDETECT_OFFSET       0x0050 /* Card-detect register value */
#define LPC54_SDMMC_WRTPRT_OFFSET        0x0054 /* Write-protect register */
#define LPC54_SDMMC_TCBCNT_OFFSET        0x005c /* Transferred CIU card byte count */
#define LPC54_SDMMC_TBBCNT_OFFSET        0x0060 /* Transferred host to BIU-FIFO byte count */
#define LPC54_SDMMC_DEBNCE_OFFSET        0x0064 /* Debounce count register */
#define LPC54_SDMMC_RSTN_OFFSET          0x0078 /* Hardware Reset */
#define LPC54_SDMMC_BMOD_OFFSET          0x0080 /* Bus Mode Register */
#define LPC54_SDMMC_PLDMND_OFFSET        0x0084 /* Poll Demand Register */
#define LPC54_SDMMC_DBADDR_OFFSET        0x0088 /* Descriptor List Base Address Register */
#define LPC54_SDMMC_IDSTS_OFFSET         0x008c /* Internal DMAC Status Register */
#define LPC54_SDMMC_IDINTEN_OFFSET       0x0090 /* Internal DMAC Interrupt Enable Register */
#define LPC54_SDMMC_DSCADDR_OFFSET       0x0094 /* Current Host Descriptor Address Register */
#define LPC54_SDMMC_BUFADDR_OFFSET       0x0098 /* Current Buffer Descriptor Address Register */
#define LPC54_SDMMC_CARDTHRCTL_OFFSET    0x0100 /* Card threshold control */
#define LPC54_SDMMC_BACKENDPWRL_OFFSET   0x0104 /* Power control */

#define LPC54_SDMMC_DATA_OFFSET          0x0200 /* Data FIFO read/write (>=) */

/* MCI register (virtual) addresses *****************************************/

#define LPC54_SDMMC_CTRL                 (LPC54_SDMMC_BASE + LPC54_SDMMC_CTRL_OFFSET)
#define LPC54_SDMMC_PWREN                (LPC54_SDMMC_BASE + LPC54_SDMMC_PWREN_OFFSET)
#define LPC54_SDMMC_CLKDIV               (LPC54_SDMMC_BASE + LPC54_SDMMC_CLKDIV_OFFSET)
#define LPC54_SDMMC_CLKENA               (LPC54_SDMMC_BASE + LPC54_SDMMC_CLKENA_OFFSET)
#define LPC54_SDMMC_TMOUT                (LPC54_SDMMC_BASE + LPC54_SDMMC_TMOUT_OFFSET)
#define LPC54_SDMMC_CTYPE                (LPC54_SDMMC_BASE + LPC54_SDMMC_CTYPE_OFFSET)
#define LPC54_SDMMC_BLKSIZ               (LPC54_SDMMC_BASE + LPC54_SDMMC_BLKSIZ_OFFSET)
#define LPC54_SDMMC_BYTCNT               (LPC54_SDMMC_BASE + LPC54_SDMMC_BYTCNT_OFFSET)
#define LPC54_SDMMC_INTMASK              (LPC54_SDMMC_BASE + LPC54_SDMMC_INTMASK_OFFSET)
#define LPC54_SDMMC_CMDARG               (LPC54_SDMMC_BASE + LPC54_SDMMC_CMDARG_OFFSET)
#define LPC54_SDMMC_CMD                  (LPC54_SDMMC_BASE + LPC54_SDMMC_CMD_OFFSET)
#define LPC54_SDMMC_RESP0                (LPC54_SDMMC_BASE + LPC54_SDMMC_RESP0_OFFSET)
#define LPC54_SDMMC_RESP1                (LPC54_SDMMC_BASE + LPC54_SDMMC_RESP1_OFFSET)
#define LPC54_SDMMC_RESP2                (LPC54_SDMMC_BASE + LPC54_SDMMC_RESP2_OFFSET)
#define LPC54_SDMMC_RESP3                (LPC54_SDMMC_BASE + LPC54_SDMMC_RESP3_OFFSET)
#define LPC54_SDMMC_MINTSTS              (LPC54_SDMMC_BASE + LPC54_SDMMC_MINTSTS_OFFSET)
#define LPC54_SDMMC_RINTSTS              (LPC54_SDMMC_BASE + LPC54_SDMMC_RINTSTS_OFFSET)
#define LPC54_SDMMC_STATUS               (LPC54_SDMMC_BASE + LPC54_SDMMC_STATUS_OFFSET)
#define LPC54_SDMMC_FIFOTH               (LPC54_SDMMC_BASE + LPC54_SDMMC_FIFOTH_OFFSET)
#define LPC54_SDMMC_CDETECT              (LPC54_SDMMC_BASE + LPC54_SDMMC_CDETECT_OFFSET)
#define LPC54_SDMMC_WRTPRT               (LPC54_SDMMC_BASE + LPC54_SDMMC_WRTPRT_OFFSET)
#define LPC54_SDMMC_TCBCNT               (LPC54_SDMMC_BASE + LPC54_SDMMC_TCBCNT_OFFSET)
#define LPC54_SDMMC_TBBCNT               (LPC54_SDMMC_BASE + LPC54_SDMMC_TBBCNT_OFFSET)
#define LPC54_SDMMC_TBBCNT               (LPC54_SDMMC_BASE + LPC54_SDMMC_TBBCNT_OFFSET)
#define LPC54_SDMMC_DEBNCE               (LPC54_SDMMC_BASE + LPC54_SDMMC_DEBNCE_OFFSET)
#define LPC54_SDMMC_RSTN                 (LPC54_SDMMC_BASE + LPC54_SDMMC_RSTN_OFFSET)
#define LPC54_SDMMC_BMOD                 (LPC54_SDMMC_BASE + LPC54_SDMMC_BMOD_OFFSET)
#define LPC54_SDMMC_PLDMND               (LPC54_SDMMC_BASE + LPC54_SDMMC_PLDMND_OFFSET)
#define LPC54_SDMMC_DBADDR               (LPC54_SDMMC_BASE + LPC54_SDMMC_DBADDR_OFFSET)
#define LPC54_SDMMC_IDSTS                (LPC54_SDMMC_BASE + LPC54_SDMMC_IDSTS_OFFSET)
#define LPC54_SDMMC_IDINTEN              (LPC54_SDMMC_BASE + LPC54_SDMMC_IDINTEN_OFFSET)
#define LPC54_SDMMC_DSCADDR              (LPC54_SDMMC_BASE + LPC54_SDMMC_DSCADDR_OFFSET)
#define LPC54_SDMMC_BUFADDR              (LPC54_SDMMC_BASE + LPC54_SDMMC_BUFADDR_OFFSET)
#define LPC54_SDMMC_CARDTHRCTL           (LPC54_SDMMC_BASE + LPC54_SDMMC_CARDTHRCTL_OFFSET)
#define LPC54_SDMMC_BACKENDPWRL          (LPC54_SDMMC_BASE + LPC54_SDMMC_BACKENDPWRL_OFFSET)

#define LPC54_SDMMC_DATA                 (LPC54_SDMMC_BASE + LPC54_SDMMC_DATA_OFFSET)

/* MCI register bit definitions *********************************************/

/* Control register CTRL */

#define SDMMC_CTRL_CNTLRRESET            (1 << 0)  /* Bit 0:  Reset Module controller */
#define SDMMC_CTRL_FIFORESET             (1 << 1)  /* Bit 1:  Reset to data FIFO to reset FIFO pointers */
#define SDMMC_CTRL_DMARESET              (1 << 2)  /* Bit 2:  Reset internal DMA interface control logic */
                                                   /* Bit 3:  Reserved */
#define SDMMC_CTRL_INTENABLE             (1 << 4)  /* Bit 4:  Enable interrupts */
                                                   /* Bit 5:  Reserved */
#define SDMMC_CTRL_READWAIT              (1 << 6)  /* Bit 6:  Assert read wait */
#define SDMMC_CTRL_SENDIRQRESP           (1 << 7)  /* Bit 7:  Send auto IRQ response */
#define SDMMC_CTRL_ABORTREAD             (1 << 8)  /* Bit 8:  Reset data state-machine (suspend sequence) */
#define SDMMC_CTRL_SENDCCSD              (1 << 9)  /* Bit 9:  Send CCSD to CE-ATA device */
#define SDMMC_CTRL_AUTOSTOP              (1 << 10) /* Bit 10: Send STOP after CCSD to CE-ATA device */
#define SDMMC_CTRL_CEATAINT              (1 << 11) /* Bit 11: CE-ATA device interrupts enabled */
                                                   /* Bits 12-15:  Reserved */
#define SDMMC_CTRL_CDVA0                 (1 << 16) /* Bit 16: Controls SD_VOLT0 pin */
#define SDMMC_CTRL_CDVA1                 (1 << 17) /* Bit 17: Controls SD_VOLT1 pin */
#define SDMMC_CTRL_CDVA2                 (1 << 18) /* Bit 18: Controls SD_VOLT2 pin */
                                                   /* Bits 19-23:  Reserved */
                                                   /* Bit 24: Reserved - always write it as 0 */
#define SDMMC_CTRL_INTDMA                (1 << 25) /* Bit 25: SD/MMC DMA use */
                                                   /* Bits 26-31:  Reserved */

/* Power Enable Register (PWREN) */

#define SDMMC_PWREN                      (1 << 0)  /* Bit 0: Power on/off switch */
                                                   /* Bits 1-31:  Reserved */

/* Clock divider register CLKDIV
 *
 * Clock division is 2*n.
 * For example, value of 0 means divide by 2 * 0 = 0 (no division,
 * bypass), value of 1 means divide by 2 * 1 = 2, value of 255 means
 * divide by 2 * 255 = 510, and so on.
 */

#define SDMMC_CLKDIV0_SHIFT              (0)       /* Bits 0-7: Clock divider 0 value */
#define SDMMC_CLKDIV0_MASK               (255 << SDMMC_CLKDIV0_SHIFT)
#  define SDMMC_CLKDIV0(n)               ((((n) + 1) >> 1) << SDMMC_CLKDIV0_SHIFT)
                                                   /* Bits 8-31:  Reserved */

/* Clock enable register CLKENA */

#define SDMMC_CLKENA_ENABLE              (1 << 0)  /* Bit 0:  Clock enable */
                                                   /* Bits 1-15:  Reserved */
#define SDMMC_CLKENA_LOWPOWER            (1 << 16) /* Bit 16: Low-power mode */
                                                   /* Bits 17-31:  Reserved */

/* Timeout register TMOUT */

#define SDMMC_TMOUT_RESPONSE_SHIFT        (0)       /* Bits 0-7: Response timeout value */
#define SDMMC_TMOUT_RESPONSE_MASK         (255 << SDMMC_TMOUT_RESPONSE_SHIFT)
#define SDMMC_TMOUT_DATA_SHIFT            (8)       /* Bits 8-31: Data Read Timeout value */
#define SDMMC_TMOUT_DATA_MASK             (0x00ffffff << SDMMC_TMOUT_DATA_SHIFT)

/* Card type register CTYPE */

#define SDMMC_CTYPE_WIDTH1                (0)       /* 1-bit mode */
#define SDMMC_CTYPE_WIDTH4                (1 << 0)  /* Bit 0:  4-bit mode */
                                                    /* Bits 1-15:  Reserved */
#define SDMMC_CTYPE_WIDTH8                (1 << 16) /* Bit 16: 8-bit mode */
                                                    /* Bits 17-31:  Reserved */

/* Blocksize register BLKSIZ */

#define SDMMC_BLKSIZ_SHIFT                (0)       /* Bits 0-15: Block size */
#define SDMMC_BLKSIZ_MASK                 (0xffff << SDMMC_BLKSIZ_SHIFT)
                                                    /* Bits 16-31:  Reserved */

/* Interrupt mask register INTMASK
 * Masked interrupt status register MINTSTS
 * Raw interrupt status register RINTSTS
 */

#define SDMMC_INT_CDET                    (1 << 0)  /* Bit 0:  Card detect */
#define SDMMC_INT_RE                      (1 << 1)  /* Bit 1:  Response error */
#define SDMMC_INT_CDONE                   (1 << 2)  /* Bit 2:  Command done */
#define SDMMC_INT_DTO                     (1 << 3)  /* Bit 3:  Data transfer over */
#define SDMMC_INT_TXDR                    (1 << 4)  /* Bit 4:  Transmit FIFO data request */
#define SDMMC_INT_RXDR                    (1 << 5)  /* Bit 5:  Receive FIFO data request */
#define SDMMC_INT_RCRC                    (1 << 6)  /* Bit 6:  Response CRC error */
#define SDMMC_INT_DCRC                    (1 << 7)  /* Bit 7:  Data CRC error */
#define SDMMC_INT_RTO                     (1 << 8)  /* Bit 8:  Response timeout */
#define SDMMC_INT_DRTO                    (1 << 9)  /* Bit 9:  Data read timeout */
#define SDMMC_INT_HTO                     (1 << 10) /* Bit 10: Data starvation-by-cpu timeout */
#define SDMMC_INT_FRUN                    (1 << 11) /* Bit 11: FIFO underrun/overrun error */
#define SDMMC_INT_HLE                     (1 << 12) /* Bit 12: Hardware locked write error */
#define SDMMC_INT_SBE                     (1 << 13) /* Bit 13: Start-bit error */
#define SDMMC_INT_ACD                     (1 << 14) /* Bit 14: Auto command done */
#define SDMMC_INT_EBE                     (1 << 15) /* Bit 15: End-bit error (read)/Write no CRC */
#define SDMMC_INT_SDMMC                   (1 << 16) /* Bit 16: SD/MMC interrupt */
                                                    /* Bits 17-31: Reserved */
#define SDMMC_INT_ALL                     (0x1ffff)

/* Command register CMD */

#define SDMMC_CMD_CMDINDEX_SHIFT          (0)       /* Bits 0-5: 5:0 Command index */
#define SDMMC_CMD_CMDINDEX_MASK           (63 << SDMMC_CMD_CMDINDEX_SHIFT)

#define SDMMC_CMD_RESPONSE                (1 << 6)  /* Bit 6:  Response expected from card */
#define SDMMC_CMD_LONGRESP                (1 << 7)  /* Bit 7:  Long response expected from card */
#define SDMMC_CMD_WAITRESP_SHIFT          (6)       /* Bits 6-7: Response expected */
#define SDMMC_CMD_WAITRESP_MASK           (3 << SDMMC_CMD_WAITRESP_SHIFT)
#  define SDMMC_CMD_NORESPONSE            (0 << SDMMC_CMD_WAITRESP_SHIFT) /* x0: No response */
#  define SDMMC_CMD_SHORTRESPONSE         (1 << SDMMC_CMD_WAITRESP_SHIFT) /* 01: Short response */
#  define SDMMC_CMD_LONGRESPONSE          (3 << SDMMC_CMD_WAITRESP_SHIFT) /* 11: Long response */

#define SDMMC_CMD_RESPCRC                 (1 << 8)  /* Bit 8:  Check response CRC */
#define SDMMC_CMD_DATAXFREXPTD            (1 << 9)  /* Bit 9:  Data transfer expected (read/write) */
#define SDMMC_CMD_WRITE                   (1 << 10) /* Bit 10: Write to card */
#define SDMMC_CMD_XFRMODE                 (1 << 11) /* Bit 11: Stream data transfer command */
#define SDMMC_CMD_AUTOSTOP                (1 << 12) /* Bit 12: Send stop command at end of data transfer */
#define SDMMC_CMD_WAITPREV                (1 << 13) /* Bit 13: Wait previous transfer complete before sending */
#define SDMMC_CMD_STOPABORT               (1 << 14) /* Bit 14: Stop current data transfer */
#define SDMMC_CMD_SENDINIT                (1 << 15) /* Bit 15: Send initialization sequence before command */
                                                    /* Bits 16-20:  Reserved */
#define SDMMC_CMD_UPDCLOCK                (1 << 21) /* Bit 21: Update clock register value (no command) */
#define SDMMC_CMD_READCEATA               (1 << 22) /* Bit 22: Performing read access on CE-ATA device */
#define SDMMC_CMD_CCSEXPTD                (1 << 23) /* Bit 23: Expect command completion from CE-ATA device */
#define SDMMC_CMD_ENABOOT                 (1 << 24) /* Bit 24: Enable Boot */
#define SDMMC_CMD_BACKEXPTED              (1 << 25) /* Bit 25: Expect Boot Acknowledge */
#define SDMMC_CMD_DISBOOT                 (1 << 26) /* Bit 26: Disable Boot */
#define SDMMC_CMD_BOOTMODE                (1 << 27) /* Bit 27: Boot Mode */
#define SDMMC_CMD_VSWITCH                 (1 << 28) /* Bit 28: Voltage switch bit */
#define SDMMC_CMD_USEHOLD                 (1 << 29) /* Bit 29: Use Hold Registered */
                                                    /* Bit 30: Reserved */
#define SDMMC_CMD_STARTCMD                (1 << 31) /* Bit 31: Start command */

/* Status register STATUS */

#define SDMMC_STATUS_RXWMARK              (1 << 0)  /* Bit 0:  FIFO reached Receive watermark level */
#define SDMMC_STATUS_TXWMARK              (1 << 1)  /* Bit 1:  FIFO reached Transmit watermark level */
#define SDMMC_STATUS_FIFOEMPTY            (1 << 2)  /* Bit 2:  FIFO is empty */
#define SDMMC_STATUS_FIFOFULL             (1 << 3)  /* Bit 3:  FIFO is full */
#define SDMMC_STATUS_FSMSTATE_SHIFT       (4)       /* Bits 4-7: 7:4 Command FSM states */
#define SDMMC_STATUS_FSMSTATE_MASK        (15 << SDMMC_STATUS_FSMSTATE_SHIFT)
#  define SDMMC_STATUS_FSMSTATE_IDLE      (0  << SDMMC_STATUS_FSMSTATE_SHIFT) /* Idle */
#  define SDMMC_STATUS_FSMSTATE_INIT      (1  << SDMMC_STATUS_FSMSTATE_SHIFT) /* Send init sequence */
#  define SDMMC_STATUS_FSMSTATE_TXSTART   (2  << SDMMC_STATUS_FSMSTATE_SHIFT) /* Tx cmd start bit */
#  define SDMMC_STATUS_FSMSTATE_TXTXBIT   (3  << SDMMC_STATUS_FSMSTATE_SHIFT) /* Tx cmd tx bit */
#  define SDMMC_STATUS_FSMSTATE_TXCMDARG  (4  << SDMMC_STATUS_FSMSTATE_SHIFT) /* Tx cmd index + arg */
#  define SDMMC_STATUS_FSMSTATE_TXCMDCRC  (5  << SDMMC_STATUS_FSMSTATE_SHIFT) /* Tx cmd crc7 */
#  define SDMMC_STATUS_FSMSTATE_TXEND     (6  << SDMMC_STATUS_FSMSTATE_SHIFT) /* Tx cmd end bit */
#  define SDMMC_STATUS_FSMSTATE_RXSTART   (7  << SDMMC_STATUS_FSMSTATE_SHIFT) /* Rx resp start bit */
#  define SDMMC_STATUS_FSMSTATE_RXIRQ     (8  << SDMMC_STATUS_FSMSTATE_SHIFT) /* Rx resp IRQ response */
#  define SDMMC_STATUS_FSMSTATE_RXTXBIT   (9  << SDMMC_STATUS_FSMSTATE_SHIFT) /* Rx resp tx bit */
#  define SDMMC_STATUS_FSMSTATE_RXCMD     (10 << SDMMC_STATUS_FSMSTATE_SHIFT) /* Rx resp cmd idx */
#  define SDMMC_STATUS_FSMSTATE_RXRESP    (11 << SDMMC_STATUS_FSMSTATE_SHIFT) /* Rx resp data */
#  define SDMMC_STATUS_FSMSTATE_RXRESPCRC (12 << SDMMC_STATUS_FSMSTATE_SHIFT) /* Rx resp crc7 */
#  define SDMMC_STATUS_FSMSTATE_RXEND     (13 << SDMMC_STATUS_FSMSTATE_SHIFT) /* Rx resp end bit */
#  define SDMMC_STATUS_FSMSTATE_WAITNCC   (14 << SDMMC_STATUS_FSMSTATE_SHIFT) /* Cmd path wait NCC */
#  define SDMMC_STATUS_FSMSTATE_WAITTURN  (15 << SDMMC_STATUS_FSMSTATE_SHIFT) /* Wait; CMD-to-resp turnaround */

#define SDMMC_STATUS_DAT3                 (1 << 8)  /* Bit 8:  DAT3=1: Card present */
#define SDMMC_STATUS_DATABUSY             (1 << 9)  /* Bit 9:  Card data busy */
#define SDMMC_STATUS_MCBUSY               (1 << 10) /* Bit 10: Data transmit/receive state machine busy */
#define SDMMC_STATUS_RESPINDEX_SHIFT      (11)      /* Bits 11-16: Index of previous response */
#define SDMMC_STATUS_RESPINDEX_MASK       (63 << SDMMC_STATUS_RESPINDEX_SHIFT)
#define SDMMC_STATUS_FIFOCOUNT_SHIFT      (17)      /* Bits 17-29: FIFO count */
#define SDMMC_STATUS_FIFOCOUNT_MASK       (0x1fff << SDMMC_STATUS_FIFOCOUNT_SHIFT)
#define SDMMC_STATUS_DMAACK               (1 << 30) /* Bit 30: DMA acknowledge signal state */
#define SDMMC_STATUS_DMAREQ               (1 << 31) /* Bit 31: DMA request signal state */

/* FIFO threshold register FIFOTH */

#define SDMMC_FIFOTH_TXWMARK_SHIFT        (0)       /* Bits 0-11: FIFO threshold level when transmitting */
#define SDMMC_FIFOTH_TXWMARK_MASK         (0xfff << SDMMC_FIFOTH_TXWMARK_SHIFT)
#  define SDMMC_FIFOTH_TXWMARK(n)         ((uint32_t)(n) << SDMMC_FIFOTH_TXWMARK_SHIFT)
                                                    /* Bits 12-15: Reserved */
#define SDMMC_FIFOTH_RXWMARK_SHIFT        (16)      /* Bits 16-27: FIFO threshold level when receiving */
#define SDMMC_FIFOTH_RXWMARK_MASK         (0xfff << SDMMC_FIFOTH_RXWMARK_SHIFT)
#  define SDMMC_FIFOTH_RXWMARK(n)         ((uint32_t)(n) << SDMMC_FIFOTH_RXWMARK_SHIFT)
#define SDMMC_FIFOTH_DMABURST_SHIFT       (28)      /* Bits 28-30: Burst size for multiple transaction */
#define SDMMC_FIFOTH_DMABURST_MASK        (7 << SDMMC_FIFOTH_DMABURST_SHIFT)
#  define SDMMC_FIFOTH_DMABURST_1XFR      (0 << SDMMC_FIFOTH_DMABURST_SHIFT) /* 1 transfer */
#  define SDMMC_FIFOTH_DMABURST_4XFRS     (1 << SDMMC_FIFOTH_DMABURST_SHIFT) /* 4 transfers */
#  define SDMMC_FIFOTH_DMABURST_8XFRS     (2 << SDMMC_FIFOTH_DMABURST_SHIFT) /* 8 transfers */
#  define SDMMC_FIFOTH_DMABURST_16XFRS    (3 << SDMMC_FIFOTH_DMABURST_SHIFT) /* 16 transfers */
#  define SDMMC_FIFOTH_DMABURST_32XFRS    (4 << SDMMC_FIFOTH_DMABURST_SHIFT) /* 32 transfers */
#  define SDMMC_FIFOTH_DMABURST_64XFRS    (5 << SDMMC_FIFOTH_DMABURST_SHIFT) /* 64 transfers */
#  define SDMMC_FIFOTH_DMABURST_128XFRS   (6 << SDMMC_FIFOTH_DMABURST_SHIFT) /* 128 transfers */
#  define SDMMC_FIFOTH_DMABURST_256XFRS   (7 << SDMMC_FIFOTH_DMABURST_SHIFT) /* 256 transfers */

                                                    /* Bit 31: Reserved */

/* Card detect register CDETECT */

#define SDMMC_CDETECT_NOTPRESENT          (1 << 0)  /* Bit 0: Card detect */
                                                    /* Bit 1-31: Reserved */

/* Write protect register WRTPRT */

#define SDMMC_WRTPRT_PROTECTED            (1 << 0)  /* Bit 0: Write protect */
                                                    /* Bit 1-31: Reserved */

/* Debounce count register */

#define SDMMC_DEBNCE_MASK                 0x00ffffff /* Bits 0-23: Debounce count */
                                                     /* Bit 24-31: Reserved */

/* Hardware Reset */

#define SDMMC_RSTN                        (1 << 0)  /* Bit 0: Hardware reset */
                                                    /* Bit 1-31: Reserved */

/* Bus Mode Register */

#define SDMMC_BMOD_SWR                    (1 << 0)  /* Bit 0:  Software Reset */
#define SDMMC_BMOD_FB                     (1 << 1)  /* Bit 1:  Fixed Burst */
#define SDMMC_BMOD_DSL_SHIFT              (2)       /* Bits 2-6: Descriptor Skip Length */
#define SDMMC_BMOD_DSL_MASK               (31 << SDMMC_BMOD_DSL_SHIFT)
#  define SDMMC_BMOD_DSL(n)               ((uint32_t)(n) << SDMMC_BMOD_DSL_SHIFT)
#define SDMMC_BMOD_DE                     (1 << 7)  /* Bit 7:  SD/MMC DMA Enable */
#define SDMMC_BMOD_PBL_SHIFT              (8)       /* Bits 8-10: Programmable Burst Length */
#define SDMMC_BMOD_PBL_MASK               (7 << SDMMC_BMOD_PBL_SHIFT)
#  define SDMMC_BMOD_PBL_1XFRS            (0 << SDMMC_BMOD_PBL_SHIFT) /* 1 transfer */
#  define SDMMC_BMOD_PBL_4XFRS            (1 << SDMMC_BMOD_PBL_SHIFT) /* 4 transfers */
#  define SDMMC_BMOD_PBL_8XFRS            (2 << SDMMC_BMOD_PBL_SHIFT) /* 8 transfers */
#  define SDMMC_BMOD_PBL_16XFRS           (3 << SDMMC_BMOD_PBL_SHIFT) /* 16 transfers */
#  define SDMMC_BMOD_PBL_32XFRS           (4 << SDMMC_BMOD_PBL_SHIFT) /* 32 transfers */
#  define SDMMC_BMOD_PBL_64XFRS           (5 << SDMMC_BMOD_PBL_SHIFT) /* 64 transfers */
#  define SDMMC_BMOD_PBL_128XFRS          (6 << SDMMC_BMOD_PBL_SHIFT) /* 128 transfers */
#  define SDMMC_BMOD_PBL_256XFRS          (7 << SDMMC_BMOD_PBL_SHIFT) /* 256 transfers */

                                                    /* Bits 11-31: Reserved */

/* Internal DMAC Status Register */

#define SDMMC_IDSTS_TI                    (1 << 0)  /* Bit 0:  Transmit Interrupt */
#define SDMMC_IDSTS_RI                    (1 << 1)  /* Bit 1:  Receive Interrupt */
#define SDMMC_IDSTS_FBE                   (1 << 2)  /* Bit 2:  Fatal Bus Error Interrupt */
                                                    /* Bit 3:  Reserved */
#define SDMMC_IDSTS_DU                    (1 << 4)  /* Bit 4:  Descriptor Unavailable Interrupt */
#define SDMMC_IDSTS_CES                   (1 << 5)  /* Bit 5:  Card Error Summary */
                                                    /* Bits 6-7: Reserved */
#define SDMMC_IDSTS_NIS                   (1 << 8)  /* Bit 8:  Normal Interrupt Summary */
#define SDMMC_IDSTS_AIS                   (1 << 9)  /* Bit 9:  Abnormal Interrupt Summary */
#define SDMMC_IDSTS_EB_SHIFT              (10)      /* Bits 10-12: Error Bits */
#define SDMMC_IDSTS_EB_MASK               (7 << SDMMC_IDSTS_EB_SHIFT)
#  define SDMMC_IDSTS_EB_TXHABORT         (1 << SDMMC_IDSTS_EB_SHIFT) /* Host Abort received during transmission */
#  define SDMMC_IDSTS_EB_RXHABORT         (2 << SDMMC_IDSTS_EB_SHIFT) /* Host Abort received during reception */

#define SDMMC_IDSTS_FSM_SHIFT             (13)      /* Bits 13-16: DMAC state machine present state */
#define SDMMC_IDSTS_FSM_MASK              (15 << SDMMC_IDSTS_FSM_SHIFT)
#  define SDMMC_IDSTS_FSM_DMAIDLE         (0 << SDMMC_IDSTS_FSM_SHIFT) /* DMA_IDLE*/
#  define SDMMC_IDSTS_FSM_DMASUSP         (1 << SDMMC_IDSTS_FSM_SHIFT) /* DMA_SUSPEND */
#  define SDMMC_IDSTS_FSM_DESCRD          (2 << SDMMC_IDSTS_FSM_SHIFT) /* DESC_RD */
#  define SDMMC_IDSTS_FSM_DESCCHK         (3 << SDMMC_IDSTS_FSM_SHIFT) /* DESC_CHK */
#  define SDMMC_IDSTS_FSM_DMARDREQW       (4 << SDMMC_IDSTS_FSM_SHIFT) /* DMA_RD_REQ_WAIT */
#  define SDMMC_IDSTS_FSM_DMAWRREQW       (5 << SDMMC_IDSTS_FSM_SHIFT) /* DMA_WR_REQ_WAIT */
#  define SDMMC_IDSTS_FSM_DMARD           (6 << SDMMC_IDSTS_FSM_SHIFT) /* DMA_RD */
#  define SDMMC_IDSTS_FSM_DMAWR           (7 << SDMMC_IDSTS_FSM_SHIFT) /* DMA_WR */
#  define SDMMC_IDSTS_FSM_DMACLOSE        (8 << SDMMC_IDSTS_FSM_SHIFT) /* DESC_CLOSE */

                                                    /* Bits 17-31: Reserved */

/* Internal DMAC Interrupt Enable Register */

#define SDMMC_IDINTEN_TI                  (1 << 0)  /* Bit 0:  Transmit Interrupt */
#define SDMMC_IDINTEN_RI                  (1 << 1)  /* Bit 1:  Receive Interrupt */
#define SDMMC_IDINTEN_FBE                 (1 << 2)  /* Bit 2:  Fatal Bus Error Interrupt */
                                                    /* Bit 3:  Reserved */
#define SDMMC_IDINTEN_DU                  (1 << 4)  /* Bit 4:  Descriptor Unavailable Interrupt */
#define SDMMC_IDINTEN_CES                 (1 << 5)  /* Bit 5:  Card Error Summary */
                                                    /* Bits 6-7: Reserved */
#define SDMMC_IDINTEN_NIS                 (1 << 8)  /* Bit 8:  Normal Interrupt Summary */
#define SDMMC_IDINTEN_AIS                 (1 << 9)  /* Bit 9:  Abnormal Interrupt Summary */
                                                    /* Bits 10-31: Reserved */
#define SDMMC_IDINTEN_ALL                 0x00000333

/* Card threshold control */

#define SDMMC_CARDTHRCTL_CARDRDTHREN      (1 << 0)  /* Bit 0:  Card read threshold enable */
#define SDMMC_CARDTHRCTL_BSYCLRINTEN      (1 << 1)  /* Bit 1:  Busy clear interrupt enable */
                                                    /* Bits 1-15: Reserved */
#define SDMMC_CARDTHRCTL_CARDTHRESHOLD_SHIFT (16)   /* Bits 16-23: Card threshold size */
#define SDMMC_CARDTHRCTL_CARDTHRESHOLD_MASK  (0xff << SDMMC_CARDTHRCTL_CARDTHRESHOLD_SHIFT)

/* Power control */

#define SDMMC_BACKENDPWRL                 (1 << 0)  /* Bit 0: Back-end Power control for card application. */

#endif /* __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC54_SDMMC_H */
