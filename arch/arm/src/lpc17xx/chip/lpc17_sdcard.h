/************************************************************************************
 * arch/arm/src/lpc17xx/chip/lpc17_sdcard.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC17XX_CHIP_LPC17_SDCARD_H
#define __ARCH_ARM_SRC_LPC17XX_CHIP_LPC17_SDCARD_H

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define LPC17_SDCARD_PWR_OFFSET       0x0000 /* SD card power control register */
#define LPC17_SDCARD_CLOCK_OFFSET     0x0004 /* SD card clock control register */
#define LPC17_SDCARD_ARG_OFFSET       0x0008 /* SD card argument register */
#define LPC17_SDCARD_CMD_OFFSET       0x000c /* SD card command register */
#define LPC17_SDCARD_RESPCMD_OFFSET   0x0010 /* SD card command response register */
#define LPC17_SDCARD_RESP_OFFSET(n)   (0x0010+4*(n))
#  define LPC17_SDCARD_RESP0_OFFSET   0x0014 /* SD card response 1 register */
#  define LPC17_SDCARD_RESP1_OFFSET   0x0018 /* SD card response 2 register */
#  define LPC17_SDCARD_RESP2_OFFSET   0x001c /* SD card response 3 register */
#  define LPC17_SDCARD_RESP3_OFFSET   0x0020 /* SD card response 4 register */
#define LPC17_SDCARD_DTIMER_OFFSET    0x0024 /* SD card data timer register */
#define LPC17_SDCARD_DLEN_OFFSET      0x0028 /* SD card data length register */
#define LPC17_SDCARD_DCTRL_OFFSET     0x002c /* SD card data control register */
#define LPC17_SDCARD_DCOUNT_OFFSET    0x0030 /* SD card data counter register */
#define LPC17_SDCARD_STATUS_OFFSET    0x0034 /* SD card status register */
#define LPC17_SDCARD_CLEAR_OFFSET     0x0038 /* SD card interrupt clear register */
#define LPC17_SDCARD_MASK0_OFFSET     0x003c /* SD card mask register */
#define LPC17_SDCARD_FIFOCNT_OFFSET   0x0048 /* SD card FIFO counter register */
#define LPC17_SDCARD_FIFO_OFFSET      0x0080 /* SD card data FIFO register */

/* Register Addresses ***************************************************************/

#define LPC17_SDCARD_PWR              (LPC17_MCI_BASE+LPC17_SDCARD_PWR_OFFSET)
#define LPC17_SDCARD_CLOCK            (LPC17_MCI_BASE+LPC17_SDCARD_CLOCK_OFFSET)
#define LPC17_SDCARD_ARG              (LPC17_MCI_BASE+LPC17_SDCARD_ARG_OFFSET)
#define LPC17_SDCARD_CMD              (LPC17_MCI_BASE+LPC17_SDCARD_CMD_OFFSET)
#define LPC17_SDCARD_RESPCMD          (LPC17_MCI_BASE+LPC17_SDCARD_RESPCMD_OFFSET)
#define LPC17_SDCARD_RESP(n)          (LPC17_MCI_BASE+LPC17_SDCARD_RESP_OFFSET(n))
#define LPC17_SDCARD_RESP0            (LPC17_MCI_BASE+LPC17_SDCARD_RESP0_OFFSET)
#define LPC17_SDCARD_RESP1            (LPC17_MCI_BASE+LPC17_SDCARD_RESP1_OFFSET)
#define LPC17_SDCARD_RESP2            (LPC17_MCI_BASE+LPC17_SDCARD_RESP2_OFFSET)
#define LPC17_SDCARD_RESP3            (LPC17_MCI_BASE+LPC17_SDCARD_RESP3_OFFSET)
#define LPC17_SDCARD_DTIMER           (LPC17_MCI_BASE+LPC17_SDCARD_DTIMER_OFFSET)
#define LPC17_SDCARD_DLEN             (LPC17_MCI_BASE+LPC17_SDCARD_DLEN_OFFSET)
#define LPC17_SDCARD_DCTRL            (LPC17_MCI_BASE+LPC17_SDCARD_DCTRL_OFFSET)
#define LPC17_SDCARD_DCOUNT           (LPC17_MCI_BASE+LPC17_SDCARD_DCOUNT_OFFSET)
#define LPC17_SDCARD_STATUS           (LPC17_MCI_BASE+LPC17_SDCARD_STATUS_OFFSET)
#define LPC17_SDCARD_CLEAR            (LPC17_MCI_BASE+LPC17_SDCARD_CLEAR_OFFSET)
#define LPC17_SDCARD_MASK0            (LPC17_MCI_BASE+LPC17_SDCARD_MASK0_OFFSET)
#define LPC17_SDCARD_FIFOCNT          (LPC17_MCI_BASE+LPC17_SDCARD_FIFOCNT_OFFSET)
#define LPC17_SDCARD_FIFO             (LPC17_MCI_BASE+LPC17_SDCARD_FIFO_OFFSET)

/* Register Bitfield Definitions ****************************************************/

/* MCI Power Control Registers - PWR - 0x400c 0000*/

#define SDCARD_PWR_CTRL_SHIFT         (0)       /* Bits 0-1: Power supply control bits */
#define SDCARD_PWR_CTRL_MASK          (3 << SDCARD_PWR_CTRL_SHIFT)
#  define SDCARD_PWR_CTRL_OFF         (0 << SDCARD_PWR_CTRL_SHIFT) /* 00: Power-off: card clock stopped */
#  define SDCARD_PWR_CTRL_PWRUP       (2 << SDCARD_PWR_CTRL_SHIFT) /* 10: Reserved power-up */
#  define SDCARD_PWR_CTRL_ON          (3 << SDCARD_PWR_CTRL_SHIFT) /* 11: Power-on: card is clocked */
                                                /* Bits 2-5 Reserved */
#define SDCARD_PWR_OPENDRAIN          (1 << 6)  /* SD_CMD Output Control */
#define SDCARD_PWR_ROD                (1 << 7)  /* Rod Control */
                                                /* Bits 8-31: Reserved */
#define SDCARD_PWR_RESET              (0)       /* Reset value */

/* MCI Clock Control Register - CLOCK - 0x400c 0004 */

#define SDCARD_CLOCK_CLKDIV_SHIFT     (0)       /* Bits 7-0: Clock divide factor */
#define SDCARD_CLOCK_CLKDIV_MASK      (0xff << SDCARD_CLOCK_CLKDIV_SHIFT)
#define SDCARD_CLOCK_CLKEN            (1 << 8)  /* Bit 8: Clock enable bit */
#define SDCARD_CLOCK_PWRSAV           (1 << 9)  /* Bit 9: Power saving configuration bit */
#define SDCARD_CLOCK_BYPASS           (1 << 10) /* Bit 10: Clock divider bypass enable bit */
#define SDCARD_CLOCK_WIDBUS           (1 << 11) /* Bit 11: Wide bus mode enable bit */
#  define SDCARD_CLOCK_WIDBUS_D1      (0)       /* 0: Default (SDIO_D0) */
#  define SDCARD_CLOCK_WIDBUS_D4      (SDCARD_CLOCK_WIDBUS) /* 1: 4-wide (SDIO_D[3:0]) */
                                                /* Bits 12-31: Reserved */

#define SDCARD_CLOCK_RESET            (0)       /* Reset value */

/* MCI Argument Register - ARGUMENT - 0x400c 0008 has no bitfields */

#define SDCARD_ARG_RESET              (0)       /* Reset value */

/* MCI Command Register - COMMAND - 0x400c 000c */

#define SDCARD_CMD_INDEX_SHIFT        (0)       /* Bits 0-5: Command Index */
#define SDCARD_CMD_INDEX_MASK         (0x3f << SDCARD_CMD_INDEX_SHIFT)
#define SDCARD_CMD_WAITRESP_SHIFT     (6)       /* Bits 7-6: Wait for response bits */
#define SDCARD_CMD_WAITRESP_MASK      (3 << SDCARD_CMD_WAITRESP_SHIFT)
#  define SDCARD_CMD_NORESPONSE       (0 << SDCARD_CMD_WAITRESP_SHIFT) /* 00/01: No response */
#  define SDCARD_CMD_SHORTRESPONSE    (1 << SDCARD_CMD_WAITRESP_SHIFT) /* 10: Short response */
#  define SDCARD_CMD_LONGRESPONSE     (3 << SDCARD_CMD_WAITRESP_SHIFT) /* 11: Long response */
#define SDCARD_CMD_WAITINT            (1 << 8)  /* Bit 8: CPSM waits for interrupt request */
#define SDCARD_CMD_WAITPEND           (1 << 9)  /* Bit 9: CPSM Waits for ends of data transfer */
#define SDCARD_CMD_CPSMEN             (1 << 10) /* Bit 10: Command path state machine enable */
                                                /* Bits 11-31: Reserved */

#define SDCARD_CMD_RESET              (0)       /* Reset value */

/* MCI Command Response Register - RESPCOMMAND - 0x400c 0010 */

#define SDCARD_RESPCMD_SHIFT          (0)       /* Bits 0-5: Resopnse Command index */
#define SDCARD_RESPCMD_MASK           (0x3f << SDCARD_RESPCMD_SHIFT)
                                                /* Bits 6-31: Reserved */

/* MCI Response Registers RESPONSE0-3 - 0x400c 0014, 0x400c 0018,
   No bitfields                         0x400c 001c, 0x400c 0020 */


/* MCI - Data Timer Register DATATIMER - 0x400c 0024 */
/* No bitfields */

#define SDCARD_DTIMER_RESET           (0)       /* Reset value */

/* MCI - Data Length Register DATALENGTH - 0x400C 0028 */

#define SDCARD_DATALENGTH_SHIFT       (0)       /* Bits 0-15: Data length value */
#define SDCARD_DATALENGTH_MASK        (0xffff << SDCARD_DATALENGTH_SHIFT)
                                                /* Bits 16-31: Reserved */

#define SDCARD_DLEN_RESET             (0)       /* Reset value */

/* MCI - Data Control Register - DATACTRL - 0x400c 002c */

#define SDCARD_DCTRL_DTEN             (1 << 0)  /* Bit 0: Data transfer enabled bit */
#define SDCARD_DCTRL_DTDIR            (1 << 1)  /* Bit 1: Data transfer direction */
#define SDCARD_DCTRL_DTMODE           (1 << 2)  /* Bit 2: Data transfer mode */
#define SDCARD_DCTRL_DMAEN            (1 << 3)  /* Bit 3: DMA enable bit */
#define SDCARD_DCTRL_DBLOCKSIZE_SHIFT (4)       /* Bits 4-7: Data block size */
#define SDCARD_DCTRL_DBLOCKSIZE_MASK  (15 << SDCARD_DCTRL_DBLOCKSIZE_SHIFT)
#  define SDCARD_DCTRL_1BYTE          (0 << SDCARD_DCTRL_DBLOCKSIZE_SHIFT)
#  define SDCARD_DCTRL_2BYTES         (1 << SDCARD_DCTRL_DBLOCKSIZE_SHIFT)
#  define SDCARD_DCTRL_4BYTES         (2 << SDCARD_DCTRL_DBLOCKSIZE_SHIFT)
#  define SDCARD_DCTRL_8BYTES         (3 << SDCARD_DCTRL_DBLOCKSIZE_SHIFT)
#  define SDCARD_DCTRL_16BYTES        (4 << SDCARD_DCTRL_DBLOCKSIZE_SHIFT)
#  define SDCARD_DCTRL_32BYTES        (5 << SDCARD_DCTRL_DBLOCKSIZE_SHIFT)
#  define SDCARD_DCTRL_64BYTES        (6 << SDCARD_DCTRL_DBLOCKSIZE_SHIFT)
#  define SDCARD_DCTRL_128BYTES       (7 << SDCARD_DCTRL_DBLOCKSIZE_SHIFT)
#  define SDCARD_DCTRL_256BYTES       (8 << SDCARD_DCTRL_DBLOCKSIZE_SHIFT)
#  define SDCARD_DCTRL_512BYTES       (9 << SDCARD_DCTRL_DBLOCKSIZE_SHIFT)
#  define SDCARD_DCTRL_1KBYTE         (10 << SDCARD_DCTRL_DBLOCKSIZE_SHIFT)
#  define SDCARD_DCTRL_2KBYTES        (11 << SDCARD_DCTRL_DBLOCKSIZE_SHIFT)
                                                /* Bits 8-31: Reserved */

#define SDCARD_DCTRL_RESET            (0)       /* Reset value */

/* MCI - Data Length Register DATALENGTH - 0x400c 0028 */

#define SDCARD_DATACOUNT_SHIFT        (0)        /* Bits 0-15: Remaining data */
#define SDCARD_DATACOUNT_MASK         (0xffff << SDCARD_DATACOUNT_SHIFT)
                                                /* Bits 16-31: Reserved */

/* MCI - Status Register -Status - 0x400c 0034 */

#define SDCARD_STATUS_CCRCFAIL        (1 << 0)  /* Bit 0: Command response CRC fail */
#define SDCARD_STATUS_DCRCFAIL        (1 << 1)  /* Bit 1: Data block CRC fail */
#define SDCARD_STATUS_CTIMEOUT        (1 << 2)  /* Bit 2: Command response timeout */
#define SDCARD_STATUS_DTIMEOUT        (1 << 3)  /* Bit 3: Data timeout */
#define SDCARD_STATUS_TXUNDERR        (1 << 4)  /* Bit 4: Transmit FIFO underrun error */
#define SDCARD_STATUS_RXOVERR         (1 << 5)  /* Bit 5: Received FIFO overrun error */
#define SDCARD_STATUS_CMDREND         (1 << 6)  /* Bit 6: Command response received  */
#define SDCARD_STATUS_CMDSENT         (1 << 7)  /* Bit 7: Command sent  */
#define SDCARD_STATUS_DATAEND         (1 << 8)  /* Bit 8: Data end */
#define SDCARD_STATUS_STBITERR        (1 << 9)  /* Bit 9: Start bit not detected  */
#define SDCARD_STATUS_DBCKEND         (1 << 10) /* Bit 10: Data block sent/received  */
#define SDCARD_STATUS_CMDACT          (1 << 11) /* Bit 11: Command transfer in progress */
#define SDCARD_STATUS_TXACT           (1 << 12) /* Bit 12: Data transmit in progress */
#define SDCARD_STATUS_RXACT           (1 << 13) /* Bit 13: Data receive in progress */
#define SDCARD_STATUS_TXFIFOHE        (1 << 14) /* Bit 14: Transmit FIFO half empty */
#define SDCARD_STATUS_RXFIFOHF        (1 << 15) /* Bit 15: Receive FIFO half full */
#define SDCARD_STATUS_TXFIFOF         (1 << 16) /* Bit 16: Transmit FIFO full */
#define SDCARD_STATUS_RXFIFOF         (1 << 17) /* Bit 17: Receive FIFO full */
#define SDCARD_STATUS_TXFIFOE         (1 << 18) /* Bit 18: Transmit FIFO empty */
#define SDCARD_STATUS_RXFIFOE         (1 << 19) /* Bit 19: Receive FIFO empty */
#define SDCARD_STATUS_TXDAVL          (1 << 20) /* Bit 20: Data available in transmit FIFO */
#define SDCARD_STATUS_RXDAVL          (1 << 21) /* Bit 21: Data available in receive FIFO */
                                                /* Bits 22-31: Reserved */

/* MCI - Clear Register CLEAR - 0x400c 0038 */

#define SDCARD_CLEAR_CCRCFAILC        (1 << 0)  /* Bit 0: CCRCFAIL flag clear bit */
#define SDCARD_CLEAR_DCRCFAILC        (1 << 1)  /* Bit 1: DCRCFAIL flag clear bit */
#define SDCARD_CLEAR_CTIMEOUTC        (1 << 2)  /* Bit 2: CTIMEOUT flag clear bit */
#define SDCARD_CLEAR_DTIMEOUTC        (1 << 3)  /* Bit 3: DTIMEOUT flag clear bit */
#define SDCARD_CLEAR_TXUNDERRC        (1 << 4)  /* Bit 4: TXUNDERR flag clear bit */
#define SDCARD_CLEAR_RXOVERRC         (1 << 5)  /* Bit 5: RXOVERR flag clear bit */
#define SDCARD_CLEAR_CMDRENDC         (1 << 6)  /* Bit 6: CMDREND flag clear bit */
#define SDCARD_CLEAR_CMDSENTC         (1 << 7)  /* Bit 7: CMDSENT flag clear bit */
#define SDCARD_CLEAR_DATAENDC         (1 << 8)  /* Bit 8: DATAEND flag clear bit */
#define SDCARD_CLEAR_STBITERRC        (1 << 9)  /* Bit 9: STBITERR flag clear bit */
#define SDCARD_CLEAR_DBCKENDC         (1 << 10) /* Bit 10: DBCKEND flag clear bit */
                                                /* Bits 11-31: Reserved */

#define SDCARD_CLEAR_RESET            0x000007ff
#define SDCARD_CLEAR_STATICFLAGS      0x000005ff

/* MCI - Interrupt Mask Registers - MASK0 - 0x400c 003c */

#define SDCARD_MASK0_CCRCFAILIE       (1 << 0)  /* Bit 0: Command CRC fail interrupt enable */
#define SDCARD_MASK0_DCRCFAILIE       (1 << 1)  /* Bit 1: Data CRC fail interrupt enable */
#define SDCARD_MASK0_CTIMEOUTIE       (1 << 2)  /* Bit 2: Command timeout interrupt enable */
#define SDCARD_MASK0_DTIMEOUTIE       (1 << 3)  /* Bit 3: Data timeout interrupt enable */
#define SDCARD_MASK0_TXUNDERRIE       (1 << 4)  /* Bit 4: Tx FIFO underrun error interrupt enable */
#define SDCARD_MASK0_RXOVERRIE        (1 << 5)  /* Bit 5: Rx FIFO overrun error interrupt enable */
#define SDCARD_MASK0_CMDRENDIE        (1 << 6)  /* Bit 6: Command response received interrupt enable */
#define SDCARD_MASK0_CMDSENTIE        (1 << 7)  /* Bit 7: Command sent interrupt enable */
#define SDCARD_MASK0_DATAENDIE        (1 << 8)  /* Bit 8: Data end interrupt enable */
#define SDCARD_MASK0_STBITERRIE       (1 << 9)  /* Bit 9: Start bit error interrupt enable */
#define SDCARD_MASK0_DBCKENDIE        (1 << 10) /* Bit 10: Data block end interrupt enable */
#define SDCARD_MASK0_CMDACTIE         (1 << 11) /* Bit 11: Command acting interrupt enable */
#define SDCARD_MASK0_TXACTIE          (1 << 12) /* Bit 12: Data transmit acting interrupt enable */
#define SDCARD_MASK0_RXACTIE          (1 << 13) /* Bit 13: Data receive acting interrupt enable */
#define SDCARD_MASK0_TXFIFOHEIE       (1 << 14) /* Bit 14: Tx FIFO half empty interrupt enable */
#define SDCARD_MASK0_RXFIFOHFIE       (1 << 15) /* Bit 15: Rx FIFO half full interrupt enable */
#define SDCARD_MASK0_TXFIFOFIE        (1 << 16) /* Bit 16: Tx FIFO full interrupt enable */
#define SDCARD_MASK0_RXFIFOFIE        (1 << 17) /* Bit 17: Rx FIFO full interrupt enable */
#define SDCARD_MASK0_TXFIFOEIE        (1 << 18) /* Bit 18: Tx FIFO empty interrupt enable */
#define SDCARD_MASK0_RXFIFOEIE        (1 << 19) /* Bit 19: Rx FIFO empty interrupt enable */
#define SDCARD_MASK0_TXDAVLIE         (1 << 20) /* Bit 20: Data available in Tx FIFO interrupt enable */
#define SDCARD_MASK0_RXDAVLIE         (1 << 21) /* Bit 21: Data available in Rx FIFO interrupt enable */
                                                /* Bits 22-31: Reserved */
#define SDCARD_MASK0_RESET            (0)

/* MCI - FIFO Counter Register (FIFOCNT - 0x400c 0048 */

#define SDCARD_FIFOCNT_SHIFT          (0)       /* Bits 0-14: Remaining data */
#define SDCARD_FIFOCNT_MASK           (0x7fff << SDCARD_FIFOCNT_SHIFT)
                                                /* Bits 15-31: Reserved */

/* MCI - Data FIFO Register - FIFO - 0x400c 0080 to 0x400c 00bc */
/* The receive and transmit FIFOs can be read or written as 32 bit wide registers.
 * The FIFOs contain 16 entries on 16 sequential addresses.
 */

#endif /* __ARCH_ARM_SRC_LPC17XX_CHIP_LPC17_SDCARD_H */

