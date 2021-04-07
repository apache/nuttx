/****************************************************************************
 * include/nuttx/sdio.h
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

#ifndef __INCLUDE_NUTTX_SDIO_H
#define __INCLUDE_NUTTX_SDIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include <nuttx/wqueue.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SDIO events needed by the driver
 *
 * Wait events are used for event-waiting by SDIO_WAITENABLE and
 * SDIO_EVENTWAIT
 */

#define SDIOWAIT_CMDDONE       (1 << 0) /* Bit 0: Command complete */
#define SDIOWAIT_RESPONSEDONE  (1 << 1) /* Bit 1: Response to command available */
#define SDIOWAIT_TRANSFERDONE  (1 << 2) /* Bit 2: Data transfer/DMA done */
#define SDIOWAIT_TIMEOUT       (1 << 3) /* Bit 3: Timeout */
#define SDIOWAIT_ERROR         (1 << 4) /* Bit 4: Some other error occurred */

/* SDIOWAIT_WRCOMPLETE (optional) : Certain SDIO driver can use D0 busy
 *   signaling to detect Write Complete.  Used of D0 busy signaling will
 *   avoid potentially very long (600Ms+) busy waiting in the MMCSD driver.
 *
 *   To implement D0 Busy signaling, the underlying SDIO driver must be
 *   capable of switching the D0 GPIO to be a rising edge sensitive
 *   interrupt pin. It must then, condition that pin to detect the rising
 *   edge on receipt of SDWAIT_WRCOMPLETE in the SDIO_WAITENABLE call and
 *   return it back to regular SDIO mode, when either the ISR fires or pin
 *   is found to be high in the SDIO_EVENTWAIT call.
 */

#if defined(CONFIG_MMCSD_SDIOWAIT_WRCOMPLETE)
#  define SDIOWAIT_WRCOMPLETE  (1 << 5) /* Bit 5: Hardware Write Completion */
#  define SDIOWAIT_ALLEVENTS   0x3f
#else
#  define SDIOWAIT_ALLEVENTS   0x1f
#endif

/* Media events are used for enable/disable registered event callbacks */

#define SDIOMEDIA_EJECTED       (1 << 0) /* Bit 0: Media removed */
#define SDIOMEDIA_INSERTED      (1 << 1) /* Bit 1: Media inserted */

/* Commands are bit-encoded to provide as much information to the SDIO
 * driver as possible in 32-bits.  The encoding is as follows:
 *
 * ---- ---- ---- ---- ---T TTRR RRCC CCCC
 *
 * CCCCCC - Bits  0-5:  6-bit command index (Range 9-63)
 * RRRR   - Bits  6-9:  4-bit response code (R1, R1B, R2-5)
 * TTT    - Bits 10-12: Data transfer type
 */

/* MMC, SD, SDIO Common Indices */

#define MMCSD_CMDIDX_SHIFT (0)
#define MMCSD_CMDIDX_MASK  (0x3f << MMCSD_CMDIDX_SHIFT)
#  define MMCSD_CMDIDX0    0  /* GO_IDLE_STATE: Resets all cards to idle state
                               * -Broadcast, no response */
#  define MMC_CMDIDX1      1  /* SEND_OP_COND: Sends capacity support information
                               * -Broadcast, R3 response, 31:0=OCR */
#  define MMCSD_CMDIDX2    2  /* ALL_SEND_CID
                               * -Broadcast, R2 response */
#  define MMC_CMDIDX3      3  /* SET_RELATIVE_ADDR
                               * -Addressed Command, R1 response 31:16=RCA */
#  define SD_CMDIDX3       3  /* SEND_RELATIVE_ADDR
                               * -Addressed Command, R6 response 31:16=RCA */
#  define MMCSD_CMDIDX4    4  /* SET_DSR
                               * -Broadcast command, no response 31:16=RCA */
#  define SDIO_CMDIDX5     5  /* SDIO_SEND_OP_COND
                               * -Addressed Command, R4 response 47:16=IO_OCR */
#  define MMCSD_CMDIDX6    6  /* HS_SWITCH: Checks switchable function */
#  define MMCSD_CMDIDX7    7  /* SELECT/DESELECT CARD
                               * -Addressed Command, R1 response 31:16=RCA */
#  define MMCSD_CMDIDX8    8  /* SD:  IF_COND: Sends SD Memory Card interface condition
                               *      R7 response;
                               * MMC: get extended CSD register 512 bytes R1 response */
#  define MMCSD_CMDIDX9    9  /* SEND_CSD: Asks  card to send its card specific data (CSD)
                               * -Addressed Command, R2 response 31:16=RCA */
#  define MMCSD_CMDIDX10  10  /* SEND_CID: Asks card to send its card identification (CID)
                               * -Addressed Command, R2 response 31:16=RCA */
#  define MMC_CMDIDX11    11  /* READ_DAT_UNTIL_STOP
                               * -Addressed data transfer command, R1 response 31:0=DADR */
#  define MMCSD_CMDIDX12  12  /* STOP_TRANSMISSION: Forces the card to stop transmission
                               * -Addressed Command, R1b response */
#  define MMCSD_CMDIDX13  13  /* SEND_STATUS: Asks card to send its status register
                               * -Addressed Command, R1 response 31:16=RCA */
#  define MMCSD_CMDIDX14  14  /* HS_BUSTEST_READ: */
#  define MMCSD_CMDIDX15  15  /* GO_INACTIVE_STATE
                               * Addressed Command, Response 31:16=RCA */
#  define MMCSD_CMDIDX16  16  /* SET_BLOCKLEN: Sets a block length (in bytes)
                               * -Addressed Command, R1 response 31:0=BLEN */
#  define MMCSD_CMDIDX17  17  /* READ_SINGLE_BLOCK: Reads a block of the selected size
                               * -Addressed data transfer command, R1 response 31:0=DADR */
#  define MMCSD_CMDIDX18  18  /* READ_MULTIPLE_BLOCK: Continuously transfers blocks from card to host
                               * -Addressed data transfer command, R1 response 31:0=DADR */
#  define MMCSD_CMDIDX19  19  /* HS_BUSTEST_WRITE: */
#  define MMC_CMDIDX20    20  /* WRITE_DAT_UNTIL_STOP: (MMC)
                               * -Addressed data transfer command, R1 response 31:0=DADR R1 */
#  define MMC_CMDIDX23    23  /* SET_BLOCK_COUNT: (MMC)
                               * -Addressed command, R1 response 31:0=DADR */
#  define MMCSD_CMDIDX24  24  /* WRITE_BLOCK: Writes a block of the selected size
                               * -Addressed data transfer command, R1 response 31:0=DADR */
#  define MMCSD_CMDIDX25  25  /* WRITE_MULTIPLE_BLOCK: Continuously writes blocks of data
                               * -Addressed data transfer command, R1 response 31:0=DADR */
#  define MMCSD_CMDIDX26  26  /* PROGRAM_CID: (Manufacturers only)
                               * -Addressed data transfer command, R1 response */
#  define MMCSD_CMDIDX27  27  /* PROGRAM_CSD: Set programmable bits of the CSD
                               * -Addressed data transfer command, R1 response */
#  define MMCSD_CMDIDX28  28  /* SET_WRITE_PROT: Sets the write protection bit of group
                               * -Addressed Command, R1b response 31:0=DADR */
#  define MMCSD_CMDIDX29  29  /* CLR_WRITE_PROT: Clears the write protection bit of group
                               * -Addressed Command, R1b response 31:0=DADR */
#  define MMCSD_CMDIDX30  30  /* SEND_WRITE_PROT: Asks card to send state of write protection bits
                               * -Addressed data transfer command, R1 response 31:0=WADR */
#  define SD_CMDIDX32     32  /* ERASE_GRP_START: Sets address of first block to erase (SD)
                               * -Addressed Command, R1 response 31:0=DADR */
#  define SD_CMDIDX33     33  /* ERASE_GRP_END: Sets address of last block to erase (SD)
                               * -Addressed Command, R1 response 31:0=DADR */
#  define MMC_CMDIDX34    34  /* UNTAG_SECTOR: (MMC)
                               * -Addressed Command, R1 response 31:0=DADR */
#  define MMC_CMDIDX35    35  /* TAG_ERASE_GROUP_START: Sets address of first block to erase (MMC)
                               * -Addressed Command, R1 response 31:0=DADR */
#  define MMC_CMDIDX36    36  /* TAG_ERASE_GROUP_END: Sets address of last block to erase (MMC)
                               * -Addressed Command, R1 response 31:0=DADR */
#  define MMC_CMDIDX37    37  /* UNTAG_ERASE_GROUP: (MMC)
                               * -Addressed Command, R1 response 31:0=DADR */
#  define MMCSD_CMDIDX38  38  /* ERASE: Erases all previously selected write blocks
                               * -Addressed Command, R1b response */
#  define MMC_CMDIDX39    39  /* FAST_IO: (MMC)
                               * -Addressed Command, R4 response (Complex) */
#  define MMC_CMDIDX40    40  /* GO_IRQ_STATE: (MMC)
                               * -Broadcast command, R5 response */
#  define MMCSD_CMDIDX42  42  /* LOCK_UNLOCK: Used to Set/Reset the Password or lock/unlock card
                               * -Addressed data transfer command, R1b response */
#  define SD_CMDIDX55     55  /* APP_CMD: Tells card that the next command is an application specific command
                               * - Addressed Command, R1 response 31:16=RCA */
#  define MMCSD_CMDIDX56  56  /* GEN_CMD: Used transfer a block to or get block from card
                               * -Addressed data transfer command, R1 Response */

/* SD/SDIO APP commands (must be preceded by CMD55) */

#  define SD_ACMDIDX6      6  /* SET_BUS_WIDTH:
                               * -Addressed Command, R1 response 1:0=BUSW */
#  define SD_ACMDIDX13    13  /* SD_STATUS: Send the SD Status
                               * -Addressed data transfer command, R1 response */
#  define SD_ACMDIDX18    18  /* SECURE_READ_MULTIPLE_BLOCK: */
#  define SD_ACMDIDX22    22  /* SEND_NUM_WR_BLOCKS: Send number of the errorfree blocks
                               * -Addressed data transfer command, R1 response */
#  define SD_ACMDIDX23    23  /* SET_WR_BLK_ERASE_COUNT: Set number blocks to erase before writing
                               * -Addressed Command, R1 response 22:0=NBLK */
#  define SD_ACMDIDX25    25  /* SECURE_WRITE_MULTIPLE_BLOCK: */
#  define SD_ACMDIDX38    38  /* SECURE_ERASE: */
#  define SD_ACMDIDX41    41  /* SD_SEND_OP_COND: Sends host capacity support information
                               * -Broadcast command, R3 response 31:0=OCR */
#  define SD_ACMDIDX42    42  /* SET_CLR_CARD_DETECT: Connect/disconnect pull-up resistor on CS
                               * Addressed Command, R1 response 0:0=CD */
#  define SD_ACMDIDX43    43  /* GET_MKB: */
#  define SD_ACMDIDX44    44  /* GET_MID: */
#  define SD_ACMDIDX45    45  /* SET_CER_RN1: */
#  define SD_ACMDIDX46    46  /* GET_CER_RN2: */
#  define SD_ACMDIDX47    47  /* SET_CER_RES2: */
#  define SD_ACMDIDX48    48  /* GET_CER_RES1/WRITE_MKB: */
#  define SD_ACMDIDX49    49  /* CHANGE_SECURE_AREA: */
#  define SD_ACMDIDX51    51  /* SEND_SCR: Reads the SD Configuration Register (SCR)
                               * Addressed data transfer command, R1 response */
#  define SD_ACMDIDX52    52  /* IO_RW_DIRECT: (SDIO only)
                               * -R5 response, 23:16=status 15:8=data */
#  define SD_ACMDIDX53    53  /* IO_RW_EXTENDED: (SDIO only)
                               * -R5 response, 23:16=status */

/* Response Encodings:
 *
 * xxxx xxxx xxxx xxxx OSMX XXRR RRCC CCCC
 *
 * x - Bit not used
 * C - Bits 0-5:   Command index
 * R - Bits 6-9:   Response type
 * X - Bits 10-12: Data transfer type
 * M - Bit 13:     MMC Multiblock transfer
 * S - Bit 14:     Stop data transfer
 * O - Bit 15:     Open drain
 * D - But 16:     SDIO Multiblock transfer
 */

#define MMCSD_RESPONSE_SHIFT (6)
#define MMCSD_RESPONSE_MASK  (15 << MMCSD_RESPONSE_SHIFT)
#  define MMCSD_NO_RESPONSE  (0)
#  define MMCSD_R1_RESPONSE  (1 << MMCSD_RESPONSE_SHIFT) /* 48-bit */
#  define MMCSD_R1B_RESPONSE (2 << MMCSD_RESPONSE_SHIFT) /* 48-bit */
#  define MMCSD_R2_RESPONSE  (3 << MMCSD_RESPONSE_SHIFT) /* 128-bit */
#  define MMCSD_R3_RESPONSE  (4 << MMCSD_RESPONSE_SHIFT) /* 48-bit */
#  define MMCSD_R4_RESPONSE  (5 << MMCSD_RESPONSE_SHIFT) /* 48-bit */
#  define MMCSD_R5_RESPONSE  (6 << MMCSD_RESPONSE_SHIFT) /* 48-bit */
#  define MMCSD_R6_RESPONSE  (7 << MMCSD_RESPONSE_SHIFT) /* 48-bit */
#  define MMCSD_R7_RESPONSE  (8 << MMCSD_RESPONSE_SHIFT) /* 48-bit */

/* Data Transfer Type */

#define MMCSD_DATAXFR_SHIFT (10)
#define MMCSD_DATAXFR_MASK  (7 << MMCSD_DATAXFR_SHIFT)
#  define MMCSD_DATAXFR     (1 << MMCSD_DATAXFR_SHIFT)
#  define MMCSD_STREAM      (2 << MMCSD_DATAXFR_SHIFT)
#  define MMCSD_WRXFR       (4 << MMCSD_DATAXFR_SHIFT)

#  define MMCSD_NODATAXFR   (0)
#  define MMCSD_RDSTREAM    (MMCSD_STREAM)
#  define MMCSD_WRSTREAM    (MMCSD_STREAM|MMCSD_WRXFR)
#  define MMCSD_RDDATAXFR   (MMCSD_DATAXFR)
#  define MMCSD_WRDATAXFR   (MMCSD_DATAXFR|MMCSD_WRXFR)

#define MMCSD_MULTIBLOCK    (1 << 13)
#define MMCSD_STOPXFR       (1 << 14)

/* Other options */

#define MMCSD_OPENDRAIN     (1 << 15)
#define SDIO_MULTIBLOCK     (1 << 16)

/* Fully decorated MMC, SD, SDIO commands */

#define MMCSD_CMD0      (MMCSD_CMDIDX0 |MMCSD_NO_RESPONSE |MMCSD_NODATAXFR)
#define MMC_CMD1        (MMC_CMDIDX1   |MMCSD_R3_RESPONSE |MMCSD_NODATAXFR |MMCSD_OPENDRAIN)
#define MMCSD_CMD2      (MMCSD_CMDIDX2 |MMCSD_R2_RESPONSE |MMCSD_NODATAXFR)
#define MMC_CMD3        (MMC_CMDIDX3   |MMCSD_R1_RESPONSE |MMCSD_NODATAXFR)
#define SD_CMD3         (SD_CMDIDX3    |MMCSD_R6_RESPONSE |MMCSD_NODATAXFR)
#define MMCSD_CMD4      (MMCSD_CMDIDX4 |MMCSD_NO_RESPONSE |MMCSD_NODATAXFR)
#define SDIO_CMD5       (SDIO_CMDIDX5  |MMCSD_R4_RESPONSE |MMCSD_NODATAXFR)
#define MMCSD_CMD6      (MMCSD_CMDIDX6 |MMCSD_R1_RESPONSE |MMCSD_RDDATAXFR)
#define MMCSD_CMD7S     (MMCSD_CMDIDX7 |MMCSD_R1B_RESPONSE|MMCSD_NODATAXFR)
#define MMCSD_CMD7D     (MMCSD_CMDIDX7 |MMCSD_NO_RESPONSE |MMCSD_NODATAXFR)  /* No response when de-selecting card */
#define SD_CMD8         (MMCSD_CMDIDX8 |MMCSD_R7_RESPONSE |MMCSD_NODATAXFR)
#define MMC_CMD8        (MMCSD_CMDIDX8 |MMCSD_R1_RESPONSE |MMCSD_RDDATAXFR)
#define MMCSD_CMD9      (MMCSD_CMDIDX9 |MMCSD_R2_RESPONSE |MMCSD_NODATAXFR)
#define MMCSD_CMD10     (MMCSD_CMDIDX10|MMCSD_R2_RESPONSE |MMCSD_NODATAXFR)
#define MMC_CMD11       (MMC_CMDIDX11  |MMCSD_R1_RESPONSE |MMCSD_RDSTREAM )
#define MMCSD_CMD12     (MMCSD_CMDIDX12|MMCSD_R1B_RESPONSE|MMCSD_NODATAXFR |MMCSD_STOPXFR)
#define MMCSD_CMD13     (MMCSD_CMDIDX13|MMCSD_R1_RESPONSE |MMCSD_NODATAXFR)
#define MMCSD_CMD14     (MMCSD_CMDIDX14|MMCSD_R1_RESPONSE |MMCSD_NODATAXFR)
#define MMCSD_CMD15     (MMCSD_CMDIDX15|MMCSD_NO_RESPONSE |MMCSD_NODATAXFR)
#define MMCSD_CMD16     (MMCSD_CMDIDX16|MMCSD_R1_RESPONSE |MMCSD_NODATAXFR)
#define MMCSD_CMD17     (MMCSD_CMDIDX17|MMCSD_R1_RESPONSE |MMCSD_RDDATAXFR)
#define MMCSD_CMD18     (MMCSD_CMDIDX18|MMCSD_R1_RESPONSE |MMCSD_RDDATAXFR |MMCSD_MULTIBLOCK)
#define MMCSD_CMD19     (MMCSD_CMDIDX19|MMCSD_R1_RESPONSE |MMCSD_NODATAXFR)
#define MMC_CMD20       (MMC_CMDIDX20  |MMCSD_R1B_RESPONSE|MMCSD_WRSTREAM )
#define MMC_CMD23       (MMC_CMDIDX23  |MMCSD_R1_RESPONSE |MMCSD_NODATAXFR)
#define MMCSD_CMD24     (MMCSD_CMDIDX24|MMCSD_R1_RESPONSE |MMCSD_WRDATAXFR)
#define MMCSD_CMD25     (MMCSD_CMDIDX25|MMCSD_R1_RESPONSE |MMCSD_WRDATAXFR |MMCSD_MULTIBLOCK)
#define MMCSD_CMD26     (MMCSD_CMDIDX26|MMCSD_R1_RESPONSE |MMCSD_WRDATAXFR)
#define MMCSD_CMD27     (MMCSD_CMDIDX27|MMCSD_R1_RESPONSE |MMCSD_WRDATAXFR)
#define MMCSD_CMD28     (MMCSD_CMDIDX28|MMCSD_R1B_RESPONSE|MMCSD_NODATAXFR)
#define MMCSD_CMD29     (MMCSD_CMDIDX29|MMCSD_R1B_RESPONSE|MMCSD_NODATAXFR)
#define MMCSD_CMD30     (MMCSD_CMDIDX30|MMCSD_R1_RESPONSE |MMCSD_RDDATAXFR)
#define SD_CMD32        (SD_CMDIDX32   |MMCSD_R1_RESPONSE |MMCSD_NODATAXFR)
#define SD_CMD33        (SD_CMDIDX33   |MMCSD_R1_RESPONSE |MMCSD_NODATAXFR)
#define MMC_CMD34       (MMC_CMDIDX34  |MMCSD_R1_RESPONSE |MMCSD_NODATAXFR)
#define MMC_CMD35       (MMC_CMDIDX35  |MMCSD_R1_RESPONSE |MMCSD_NODATAXFR)
#define MMC_CMD36       (MMC_CMDIDX36  |MMCSD_R1_RESPONSE |MMCSD_NODATAXFR)
#define MMC_CMD37       (MMC_CMDIDX37  |MMCSD_R1_RESPONSE |MMCSD_NODATAXFR)
#define MMCSD_CMD38     (MMCSD_CMDIDX38|MMCSD_R1B_RESPONSE|MMCSD_NODATAXFR)
#define MMC_CMD39       (MMC_CMDIDX39  |MMCSD_R4_RESPONSE |MMCSD_NODATAXFR)
#define MMC_CMD40       (MMC_CMDIDX40  |MMCSD_R5_RESPONSE |MMCSD_NODATAXFR)
#define MMCSD_CMD42     (MMCSD_CMDIDX42|MMCSD_R1B_RESPONSE|MMCSD_WRDATAXFR)
#define SD_CMD55        (SD_CMDIDX55   |MMCSD_R1_RESPONSE |MMCSD_NODATAXFR)
#define MMCSD_CMD56     (MMCSD_CMDIDX56|MMCSD_R1_RESPONSE |MMCSD_RDDATAXFR)

/* SD/SDIO APP commands (must be preceded by CMD55) */

#define SD_ACMD6        (SD_ACMDIDX6   |MMCSD_R1_RESPONSE |MMCSD_NODATAXFR)
#define SD_ACMD13       (SD_ACMDIDX13  |MMCSD_R1_RESPONSE |MMCSD_RDDATAXFR)
#define SD_ACMD18       (SD_ACMDIDX18  |MMCSD_R1_RESPONSE |MMCSD_NODATAXFR)
#define SD_ACMD22       (SD_ACMDIDX22  |MMCSD_R1_RESPONSE |MMCSD_RDDATAXFR)
#define SD_ACMD23       (SD_ACMDIDX23  |MMCSD_R1_RESPONSE |MMCSD_NODATAXFR)
#define SD_ACMD25       (SD_ACMDIDX25  |MMCSD_R1_RESPONSE |MMCSD_NODATAXFR)
#define SD_ACMD38       (SD_ACMDIDX38  |MMCSD_R1_RESPONSE |MMCSD_NODATAXFR)
#define SD_ACMD41       (SD_ACMDIDX41  |MMCSD_R3_RESPONSE |MMCSD_NODATAXFR)
#define SD_ACMD42       (SD_ACMDIDX42  |MMCSD_R1_RESPONSE |MMCSD_NODATAXFR)
#define SD_ACMD43       (SD_ACMDIDX43  |MMCSD_R1_RESPONSE |MMCSD_NODATAXFR)
#define SD_ACMD44       (SD_ACMDIDX44  |MMCSD_R1_RESPONSE |MMCSD_NODATAXFR)
#define SD_ACMD45       (SD_ACMDIDX45  |MMCSD_R1_RESPONSE |MMCSD_NODATAXFR)
#define SD_ACMD46       (SD_ACMDIDX46  |MMCSD_R1_RESPONSE |MMCSD_NODATAXFR)
#define SD_ACMD47       (SD_ACMDIDX47  |MMCSD_R1_RESPONSE |MMCSD_NODATAXFR)
#define SD_ACMD48       (SD_ACMDIDX48  |MMCSD_R1_RESPONSE |MMCSD_NODATAXFR)
#define SD_ACMD49       (SD_ACMDIDX49  |MMCSD_R1_RESPONSE |MMCSD_NODATAXFR)
#define SD_ACMD51       (SD_ACMDIDX51  |MMCSD_R1_RESPONSE |MMCSD_RDDATAXFR)
#define SD_ACMD52       (SD_ACMDIDX52  |MMCSD_R5_RESPONSE |MMCSD_NODATAXFR)
#define SD_ACMD52ABRT   (SD_ACMDIDX52  |MMCSD_R1_RESPONSE |MMCSD_NODATAXFR|MMCSD_STOPXFR)
#define SD_ACMD53       (SD_ACMDIDX53  |MMCSD_R5_RESPONSE |MMCSD_NODATAXFR)

/* SDIO Card Common Control Registers definitions
 * see https://www.sdcard.org/developers/overview/sdio/
 *                             sdio_spec/Simplified_SDIO_Card_Spec.pdf
 */

#define SDIO_CCCR_REV                    0x00 /* CCCR/SDIO Revision        */
#define SDIO_CCCR_SD_SPEC_REV            0x01 /* SD Specification Revision */
#define SDIO_CCCR_IOEN                   0x02 /* I/O Enable                */
#define SDIO_CCCR_IORDY                  0x03 /* I/O Ready                 */
#define SDIO_CCCR_INTEN                  0x04 /* Int Enable                */
#define SDIO_CCCR_INTPEND                0x05 /* Int Pending               */
#define SDIO_CCCR_IOABORT                0x06 /* I/O Abort                 */
#define SDIO_CCCR_BUS_IF                 0x07 /* Bus Interface Control     */
#define SDIO_CCCR_CARD_CAP               0x08 /* Card Capability          */
#define SDIO_CCCR_CCP                    0x09 /* Common CIS Pointer        */
#define SDIO_CCCR_BUS_SUSP               0x0C /* Bus Suspend               */
#define SDIO_CCCR_FUNCSEL                0x0D /* Function Select           */
#define SDIO_CCCR_EXEC_FLAGS             0x0E /* Exec Flags                */
#define SDIO_CCCR_RDY_FLAGS              0x0F /* Ready Flags               */
#define SDIO_CCCR_FN0_BLKSIZE_0          0x10 /* FN0 Block Size            */
#define SDIO_CCCR_FN0_BLKSIZE_1          0x11 /* FN0 Block Size            */
#define SDIO_CCCR_POWER                  0x12 /* Power Control             */
#define SDIO_CCCR_HIGHSPEED              0x13 /* High-Speed                */
#define SDIO_CCCR_RFU                    0x14 /* Reserved for future use   */
#define SDIO_CCCR_VENDOR                 0xF0 /* Reserved for Vendors      */

#define SDIO_CCCR_BUS_IF_WIDTH_MASK      0x03 /* Bus width configuration   */
#define SDIO_CCCR_BUS_IF_1_BIT           0x01 /* 1 bit bus width setting   */
#define SDIO_CCCR_BUS_IF_4_BITS          0x02 /* 4 bits bus width setting  */

#define SDIO_FBR_SHIFT   8                     /* FBR bit shift            */
#define SDIO_FN1_BR_BASE (1 << SDIO_FBR_SHIFT) /* Func 1 registers base    */
#define SDIO_FN2_BR_BASE (2 << SDIO_FBR_SHIFT) /* Func 2 registers base    */
#define SDIO_FN3_BR_BASE (3 << SDIO_FBR_SHIFT) /* Func 3 registers base    */
#define SDIO_FN4_BR_BASE (4 << SDIO_FBR_SHIFT) /* Func 4 registers base    */
#define SDIO_FN5_BR_BASE (5 << SDIO_FBR_SHIFT) /* Func 5 registers base    */
#define SDIO_FN6_BR_BASE (6 << SDIO_FBR_SHIFT) /* Func 6 registers base    */
#define SDIO_FN7_BR_BASE (7 << SDIO_FBR_SHIFT) /* Func 7 registers base    */

/****************************************************************************
 * Name: SDIO_LOCK
 *
 * Description:
 *   Lock/unlock the SDIO bus, preventing it from any other transaction
 *   while locked.
 *
 * Input Parameters:
 *   dev      - An instance of the SDIO device interface
 *   state    - TRUE/FALSE
 *
 * Returned Value:
 *   OK on success.
 *
 ****************************************************************************/

#define SDIO_LOCK(dev,state)  ((dev)->lock(dev,state))

/****************************************************************************
 * Name: SDIO_RESET
 *
 * Description:
 *   Reset the SDIO controller.  Undo all setup and initialization.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#define SDIO_RESET(dev) ((dev)->reset(dev))

/****************************************************************************
 * Name: SDIO_CAPABILITIES
 *
 * Description:
 *   Get capabilities (and limitations) of the SDIO driver (optional)
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *
 * Returned Value:
 *   Returns a bitset of status values (see SDIO_CAPS_* defines)
 *
 ****************************************************************************/

#define SDIO_CAPABILITIES(dev) ((dev)->capabilities(dev))

/* SDIO capability bits */

#define SDIO_CAPS_1BIT_ONLY       0x01 /* Bit 0=1: Supports only 1-bit operation */
#define SDIO_CAPS_DMASUPPORTED    0x02 /* Bit 1=1: Supports DMA data transfers */
#define SDIO_CAPS_DMABEFOREWRITE  0x04 /* Bit 2=1: Executes DMA before write command */
#define SDIO_CAPS_4BIT            0x08 /* Bit 3=1: Supports 4 bit operation */
#define SDIO_CAPS_8BIT            0x10 /* Bit 4=1: Supports 8 bit operation */

/****************************************************************************
 * Name: SDIO_STATUS
 *
 * Description:
 *   Get SDIO status.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *
 * Returned Value:
 *   Returns a bitset of status values (see SDIO_STATUS_* defines)
 *
 ****************************************************************************/

#define SDIO_STATUS(dev)        ((dev)->status(dev))

/* SDIO status bits */

#define SDIO_STATUS_PRESENT     0x01 /* Bit 0=1: SDIO card present */
#define SDIO_STATUS_WRPROTECTED 0x02 /* Bit 1=1: SDIO card write protected */

#define SDIO_PRESENT(dev)       ((SDIO_STATUS(dev) & SDIO_STATUS_PRESENT) != 0)
#define SDIO_WRPROTECTED(dev)   ((SDIO_STATUS(dev) & SDIO_STATUS_WRPROTECTED) != 0)

/****************************************************************************
 * Name: SDIO_WIDEBUS
 *
 * Description:
 *   Called after change in Bus width has been selected (via ACMD6).  Most
 *   controllers will need to perform some special operations to work
 *   correctly in the new bus mode.
 *
 * Input Parameters:
 *   dev  - An instance of the SDIO device interface
 *   wide - true: wide bus (4-bit) bus mode enabled
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#define SDIO_WIDEBUS(dev,wide) ((dev)->widebus(dev,wide))

/****************************************************************************
 * Name: SDIO_CLOCK
 *
 * Description:
 *   Enable/disable SDIO clocking
 *
 * Input Parameters:
 *   dev  - An instance of the SDIO device interface
 *   rate - Specifies the clocking to use (see enum sdio_clock_e)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#define SDIO_CLOCK(dev,rate) ((dev)->clock(dev,rate))

/****************************************************************************
 * Name: SDIO_ATTACH
 *
 * Description:
 *   Attach and prepare interrupts
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *
 * Returned Value:
 *   OK on success; A negated errno on failure.
 *
 ****************************************************************************/

#define SDIO_ATTACH(dev) ((dev)->attach(dev))

/****************************************************************************
 * Name: SDIO_SENDCMD
 *
 * Description:
 *   Send the SDIO command
 *
 * Input Parameters:
 *   dev  - An instance of the SDIO device interface
 *   cmd  - The command to send.  See 32-bit command definitions above.
 *   arg  - 32-bit argument required with some commands
 *   data - A reference to data required with some commands
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#define SDIO_SENDCMD(dev,cmd,arg) ((dev)->sendcmd(dev,cmd,arg))

/****************************************************************************
 * Name: SDIO_BLOCKLEN
 *
 * Description:
 *   Some hardware needs to be informed of the selected blocksize and the
 *   number of blocks.  Others just work on the byte stream.
 *
 * Input Parameters:
 *   dev      - An instance of the SDIO device interface
 *   blocklen - The selected block size.
 *   nblocks  - The number of blocks to be transferred.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_BLOCKSETUP
#  define SDIO_BLOCKSETUP(dev,blocklen,nblocks) ((dev)->blocksetup(dev,blocklen,nblocks))
#else
#  define SDIO_BLOCKSETUP(dev,blocklen,nblocks)
#endif

/****************************************************************************
 * Name: SDIO_RECVSETUP
 *
 * Description:
 *   Setup hardware in preparation for data transfer from the card in non-DMA
 *   (interrupt driven mode).  This method will do whatever controller setup
 *   is necessary.  This would be called for SD memory just BEFORE sending
 *   CMD13 (SEND_STATUS), CMD17 (READ_SINGLE_BLOCK), CMD18
 *   (READ_MULTIPLE_BLOCKS), ACMD51 (SEND_SCR), etc.  Normally,
 *   SDIO_WAITEVENT will be called to receive the indication that the
 *   transfer is complete.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *   buffer - Address of the buffer in which to receive the data
 *   nbytes - The number of bytes in the transfer
 *
 * Returned Value:
 *   Number of bytes sent on success; a negated errno on failure
 *
 ****************************************************************************/

#define SDIO_RECVSETUP(dev,buffer,nbytes) ((dev)->recvsetup(dev,buffer,nbytes))

/****************************************************************************
 * Name: SDIO_SENDSETUP
 *
 * Description:
 *   Setup hardware in preparation for data transfer from the card.  This
 *   method will do whatever controller setup is necessary.  This would be
 *   called for SD memory just AFTER sending CMD24 (WRITE_BLOCK), CMD25
 *   (WRITE_MULTIPLE_BLOCK), ... and before SDIO_SENDDATA is called.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *   buffer - Address of the buffer containing the data to send
 *   nbytes - The number of bytes in the transfer
 *
 * Returned Value:
 *   Number of bytes sent on success; a negated errno on failure
 *
 ****************************************************************************/

#define SDIO_SENDSETUP(dev,buffer,nbytes) ((dev)->sendsetup(dev,buffer,nbytes))

/****************************************************************************
 * Name: SDIO_CANCEL
 *
 * Description:
 *   Cancel the data transfer setup of SDIO_RECVSETUP, SDIO_SENDSETUP,
 *   SDIO_DMARECVSETUP or SDIO_DMASENDSETUP.  This must be called to cancel
 *   the data transfer setup if, for some reason, you cannot perform the
 *   transfer.
 *
 * Input Parameters:
 *   dev  - An instance of the SDIO device interface
 *
 * Returned Value:
 *   OK is success; a negated errno on failure
 *
 ****************************************************************************/

#define SDIO_CANCEL(dev) ((dev)->cancel(dev))

/****************************************************************************
 * Name: SDIO_WAITRESPONSE
 *
 * Description:
 *   Poll-wait for the response to the last command to be ready.  This
 *   function should be called even after sending commands that have no
 *   response (such as CMD0) to make sure that the hardware is ready to
 *   receive the next command.
 *
 * Input Parameters:
 *   dev  - An instance of the SDIO device interface
 *   cmd  - The command that was sent.  See 32-bit command definitions above.
 *
 * Returned Value:
 *   OK is success; a negated errno on failure
 *
 ****************************************************************************/

#define SDIO_WAITRESPONSE(dev,cmd) ((dev)->waitresponse(dev,cmd))

/****************************************************************************
 * Name: SDIO_RECVRx
 *
 * Description:
 *   Receive response to SDIO command.  Only the critical payload is
 *   returned -- that is 32 bits for 48 bit status and 128 bits for 136 bit
 *   status.  The driver implementation should verify the correctness of
 *   the remaining, non-returned bits (CRCs, CMD index, etc.).
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *   Rx - Buffer in which to receive the response
 *
 * Returned Value:
 *   Number of bytes sent on success; a negated errno on failure.  Here a
 *   failure means only a faiure to obtain the requested response (due to
 *   transport problem -- timeout, CRC, etc.).  The implementation only
 *   assures that the response is returned intacta and does not check errors
 *   within the response itself.
 *
 ****************************************************************************/

#define SDIO_RECVR1(dev,cmd,r1) ((dev)->recv_r1(dev,cmd,r1)) /* 48-bit */
#define SDIO_RECVR2(dev,cmd,r2) ((dev)->recv_r2(dev,cmd,r2)) /* 136-bit */
#define SDIO_RECVR3(dev,cmd,r3) ((dev)->recv_r3(dev,cmd,r3)) /* 48-bit */
#define SDIO_RECVR4(dev,cmd,r4) ((dev)->recv_r4(dev,cmd,r4)) /* 48-bit */
#define SDIO_RECVR5(dev,cmd,r5) ((dev)->recv_r5(dev,cmd,r5)) /* 48-bit */
#define SDIO_RECVR6(dev,cmd,r6) ((dev)->recv_r6(dev,cmd,r6)) /* 48-bit */
#define SDIO_RECVR7(dev,cmd,r7) ((dev)->recv_r7(dev,cmd,r7)) /* 48-bit */

/****************************************************************************
 * Name: SDIO_WAITENABLE
 *
 * Description:
 *   Enable/disable of a set of SDIO wait events.  This is part of the
 *   the SDIO_WAITEVENT sequence.  The set of to-be-waited-for events is
 *   configured before calling either calling SDIO_DMARECVSETUP,
 *   SDIO_DMASENDSETUP, or or SDIO_WAITEVENT.  This is the recommended
 *   ordering:
 *
 *     SDIO_WAITENABLE:    Discard any pending interrupts, enable event(s)
 *                         of interest
 *     SDIO_DMARECVSETUP/
 *     SDIO_DMASENDSETUP:  Setup the logic that will trigger the event the
 *                         event(s) of interest
 *     SDIO_WAITEVENT:     Wait for the event of interest (which might
 *                         already have occurred)
 *
 *   This sequence should eliminate race conditions between the command/
 *   transfer setup and the subsequent events.
 *
 *   The enabled events persist until either (1) SDIO_WAITENABLE is called
 *   again specifying a different set of wait events, or (2) SDIO_EVENTWAIT
 *   returns.
 *
 * Input Parameters:
 *   dev      - An instance of the SDIO device interface
 *   eventset - A bitset of events to enable or disable (see SDIOWAIT_*
 *              definitions). 0=disable; 1=enable.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#define SDIO_WAITENABLE(dev,eventset,timeout)  ((dev)->waitenable(dev,eventset,timeout))

/****************************************************************************
 * Name: SDIO_EVENTWAIT
 *
 * Description:
 *   Wait for one of the enabled events to occur (or a timeout).  Note that
 *   all events enabled by SDIO_WAITEVENTS are disabled when SDIO_EVENTWAIT
 *   returns.  SDIO_WAITEVENTS must be called again before SDIO_EVENTWAIT
 *   can be used again.
 *
 * Input Parameters:
 *   dev     - An instance of the SDIO device interface
 *   timeout - Maximum time in milliseconds to wait.  Zero means immediate
 *             timeout with no wait.  The timeout value is ignored if
 *             SDIOWAIT_TIMEOUT is not included in the waited-for eventset.
 *
 * Returned Value:
 *   Event set containing the event(s) that ended the wait.  Should always
 *   be non-zero.  All events are disabled after the wait concludes.
 *
 ****************************************************************************/

#define SDIO_EVENTWAIT(dev)  ((dev)->eventwait(dev))

/****************************************************************************
 * Name: SDIO_CALLBACKENABLE
 *
 * Description:
 *   Enable/disable of a set of SDIO callback events.  This is part of the
 *   the SDIO callback sequence.  The set of events is configured to enabled
 *   callbacks to the function provided in SDIO_REGISTERCALLBACK.
 *
 *   Events are automatically disabled once the callback is performed and no
 *   further callback events will occur until they are again enabled by
 *   calling this method.
 *
 * Input Parameters:
 *   dev      - An instance of the SDIO device interface
 *   eventset - A bitset of events to enable or disable (see SDIOMEDIA_*
 *              definitions). 0=disable; 1=enable.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#define SDIO_CALLBACKENABLE(dev,eventset)  ((dev)->callbackenable(dev,eventset))

/****************************************************************************
 * Name: SDIO_REGISTERCALLBACK
 *
 * Description:
 *   Register a callback that that will be invoked on any media status
 *   change.  Callbacks should not be made from interrupt handlers, rather
 *   interrupt level events should be handled by calling back on the work
 *   thread.
 *
 *   When this method is called, all callbacks should be disabled until they
 *   are enabled via a call to SDIO_CALLBACKENABLE.
 *
 *   NOTE: High-priority work queue support is required.
 *
 * Input Parameters:
 *   dev -      Device-specific state data
 *   callback - The function to call on the media change
 *   arg -      A caller provided value to return with the callback
 *
 * Returned Value:
 *   0 on success; negated errno on failure.
 *
 ****************************************************************************/

#if defined(CONFIG_SCHED_WORKQUEUE) && defined(CONFIG_SCHED_HPWORK)
#  define SDIO_REGISTERCALLBACK(d,c,a) ((d)->registercallback(d,c,a))
#endif

/****************************************************************************
 * Name: SDIO_DMAPREFLIGHT
 *
 * Description:
 *   Preflight an SDIO DMA operation.  If the buffer is not well-formed for
 *   SDIO DMA transfer (alignment, size, etc.) returns an error.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *   buffer - The memory to DMA to/from
 *   buflen - The size of the DMA transfer in bytes
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

#if defined(CONFIG_SDIO_DMA) && defined(CONFIG_ARCH_HAVE_SDIO_PREFLIGHT)
#  define SDIO_DMAPREFLIGHT(dev,buffer,len) \
    ((dev)->dmapreflight?(dev)->dmapreflight(dev,buffer,len):OK)
#else
#  define SDIO_DMAPREFLIGHT(dev,buffer,len) (OK)
#endif

/****************************************************************************
 * Name: SDIO_DMARECVSETUP
 *
 * Description:
 *   Setup to perform a read DMA.  If the processor supports a data cache,
 *   then this method will also make sure that the contents of the DMA memory
 *   and the data cache are coherent.  For read transfers this may mean
 *   invalidating the data cache.  Upon return, DMA is enable and waiting.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *   buffer - The memory to DMA from
 *   buflen - The size of the DMA transfer in bytes
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_DMA
#  define SDIO_DMARECVSETUP(dev,buffer,len) ((dev)->dmarecvsetup(dev,buffer,len))
#else
#  define SDIO_DMARECVSETUP(dev,buffer,len) (-ENOSYS)
#endif

/****************************************************************************
 * Name: SDIO_DMASENDSETUP
 *
 * Description:
 *   Setup to perform a write DMA.  If the processor supports a data cache,
 *   then this method will also make sure that the contents of the DMA memory
 *   and the data cache are coherent.  For write transfers, this may mean
 *   flushing the data cache.  Upon return, DMA is enable and waiting.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *   buffer - The memory to DMA into
 *   buflen - The size of the DMA transfer in bytes
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_DMA
#  define SDIO_DMASENDSETUP(dev,buffer,len) ((dev)->dmasendsetup(dev,buffer,len))
#else
#  define SDIO_DMASENDSETUP(dev,buffer,len) (-ENOSYS)
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Various clocking used by the SDIO driver */

enum sdio_clock_e
{
  CLOCK_SDIO_DISABLED = 0, /* Clock is disabled */
  CLOCK_IDMODE,            /* Initial ID mode clocking (<400KHz) */
  CLOCK_MMC_TRANSFER,      /* MMC normal operation clocking */
  CLOCK_SD_TRANSFER_1BIT,  /* SD normal operation clocking (narrow 1-bit mode) */
  CLOCK_SD_TRANSFER_4BIT   /* SD normal operation clocking (wide 4-bit mode) */
};

/* Event set.  A uint8_t is big enough to hold a set of 8-events.  If more
 * are needed, change this to a uint16_t.
 */

typedef uint8_t sdio_eventset_t;

/* Capabilities set.  A uint8_t is big enough to hold a set of
 * 8-capabilities/limitations.  If more are needed, change this to a
 * uint16_t.
 */

typedef uint8_t sdio_capset_t;

/* Status set.  A uint8_t is big enough to hold a set of 8 status bits.
 * If more are needed, change this to a uint16_t.
 */

typedef uint8_t sdio_statset_t;

/* This structure defines the interface between the NuttX SDIO driver and
 * the chip- or board-specific SDIO interface.  This interface is only used
 * in architectures that support SDIO 1- or 4-bit data buses.  For SDIO
 * support this interface is registered with the NuttX SDIO driver by
 * calling sdio_slotinitialize().
 */

struct sdio_dev_s
{
  /* See descriptions of each method in the access macros provided
   * above.
   */

  /* Mutual exclusion */

#ifdef CONFIG_SDIO_MUXBUS
  int   (*lock)(FAR struct sdio_dev_s *dev, bool lock);
#endif

  /* Initialization/setup */

  void  (*reset)(FAR struct sdio_dev_s *dev);
  sdio_capset_t (*capabilities)(FAR struct sdio_dev_s *dev);
  sdio_statset_t (*status)(FAR struct sdio_dev_s *dev);
  void  (*widebus)(FAR struct sdio_dev_s *dev, bool enable);
  void  (*clock)(FAR struct sdio_dev_s *dev, enum sdio_clock_e rate);
  int   (*attach)(FAR struct sdio_dev_s *dev);

  /* Command/Status/Data Transfer */

  int   (*sendcmd)(FAR struct sdio_dev_s *dev, uint32_t cmd, uint32_t arg);
#ifdef CONFIG_SDIO_BLOCKSETUP
  void  (*blocksetup)(FAR struct sdio_dev_s *dev, unsigned int blocklen,
          unsigned int nblocks);
#endif
  int   (*recvsetup)(FAR struct sdio_dev_s *dev, FAR uint8_t *buffer,
          size_t nbytes);
  int   (*sendsetup)(FAR struct sdio_dev_s *dev, FAR const uint8_t *buffer,
          size_t nbytes);
  int   (*cancel)(FAR struct sdio_dev_s *dev);

  int   (*waitresponse)(FAR struct sdio_dev_s *dev, uint32_t cmd);
  int   (*recv_r1)(FAR struct sdio_dev_s *dev, uint32_t cmd, uint32_t *R1);
  int   (*recv_r2)(FAR struct sdio_dev_s *dev, uint32_t cmd, uint32_t R2[4]);
  int   (*recv_r3)(FAR struct sdio_dev_s *dev, uint32_t cmd, uint32_t *R3);
  int   (*recv_r4)(FAR struct sdio_dev_s *dev, uint32_t cmd, uint32_t *R4);
  int   (*recv_r5)(FAR struct sdio_dev_s *dev, uint32_t cmd, uint32_t *R5);
  int   (*recv_r6)(FAR struct sdio_dev_s *dev, uint32_t cmd, uint32_t *R6);
  int   (*recv_r7)(FAR struct sdio_dev_s *dev, uint32_t cmd, uint32_t *R7);

  /* Event/Callback support */

  void  (*waitenable)(FAR struct sdio_dev_s *dev, sdio_eventset_t eventset,
          uint32_t timeout);
  sdio_eventset_t (*eventwait)(FAR struct sdio_dev_s *dev);
  void  (*callbackenable)(FAR struct sdio_dev_s *dev,
          sdio_eventset_t eventset);

#if defined(CONFIG_SCHED_WORKQUEUE) && defined(CONFIG_SCHED_HPWORK)
  int   (*registercallback)(FAR struct sdio_dev_s *dev,
          worker_t callback, void *arg);
#endif

  /* DMA.  CONFIG_SDIO_DMA should be set if the driver supports BOTH DMA
   * and non-DMA transfer modes.  If the driver supports only one mode
   * CONFIG_SDIO_DMA is not required.
   */

#ifdef CONFIG_SDIO_DMA
#ifdef CONFIG_ARCH_HAVE_SDIO_PREFLIGHT
  int   (*dmapreflight)(FAR struct sdio_dev_s *dev,
          FAR const uint8_t *buffer, size_t buflen);
#endif
  int   (*dmarecvsetup)(FAR struct sdio_dev_s *dev, FAR uint8_t *buffer,
          size_t buflen);
  int   (*dmasendsetup)(FAR struct sdio_dev_s *dev,
          FAR const uint8_t *buffer, size_t buflen);
#endif /* CONFIG_SDIO_DMA */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __INCLUDE_NUTTX_SDIO_H */
