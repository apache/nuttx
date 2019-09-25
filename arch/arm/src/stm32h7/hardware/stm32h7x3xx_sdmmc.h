/************************************************************************************
 * arch/arm/src/stm32h7/hardware/stm32h7x3xx_sdmmc.h
 *
 *   Copyright (C) 2009, 2011-2016, 2019 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
 *            Jukka Laitinen <jukka.laitinen@iki.fi>
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

#ifndef __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32H7X3XX_SDMMC_H
#define __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32H7X3XX_SDMMC_H

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define STM32_SDMMC_POWER_OFFSET              0x0000 /* SDMMC power control register */
#define STM32_SDMMC_CLKCR_OFFSET              0x0004 /* SDMMC clock control register */
#define STM32_SDMMC_ARG_OFFSET                0x0008 /* SDMMC argument register */
#define STM32_SDMMC_CMD_OFFSET                0x000c /* SDMMC command register */
#define STM32_SDMMC_RESPCMD_OFFSET            0x0010 /* SDMMC command response register */
#define STM32_SDMMC_RESP_OFFSET(n)            (0x0010+4*(n))
#define STM32_SDMMC_RESP1_OFFSET              0x0014 /* SDMMC response 1 register */
#define STM32_SDMMC_RESP2_OFFSET              0x0018 /* SDMMC response 2 register */
#define STM32_SDMMC_RESP3_OFFSET              0x001c /* SDMMC response 3 register */
#define STM32_SDMMC_RESP4_OFFSET              0x0020 /* SDMMC response 4 register */
#define STM32_SDMMC_DTIMER_OFFSET             0x0024 /* SDMMC data timer register */
#define STM32_SDMMC_DLEN_OFFSET               0x0028 /* SDMMC data length register */
#define STM32_SDMMC_DCTRL_OFFSET              0x002c /* SDMMC data control register */
#define STM32_SDMMC_DCOUNT_OFFSET             0x0030 /* SDMMC data counter register */
#define STM32_SDMMC_STA_OFFSET                0x0034 /* SDMMC status register */
#define STM32_SDMMC_ICR_OFFSET                0x0038 /* SDMMC interrupt clear register */
#define STM32_SDMMC_MASK_OFFSET               0x003c /* SDMMC mask register */
#define STM32_SDMMC_IDMACTRLR_OFFSET          0x0050 /* SDMMC DMA control register */
#define STM32_SDMMC_IDMABSIZER_OFFSET         0x0054 /* SDMMC DMA size register */
#define STM32_SDMMC_IDMABASE0R_OFFSET         0x0058 /* SDMMC IDMA buffer 0 base register */
#define STM32_SDMMC_IDMABASE1R_OFFSET         0x005c /* SDMMC IDMA buffer 1 base register */
#define STM32_SDMMC_FIFO_OFFSET               0x0080 /* SDMMC data FIFO register */

/* Register Bitfield Definitions ****************************************************/

#define STM32_SDMMC_POWER_PWRCTRL_SHIFT       (0)       /* Bits 0-1: Power supply control bits */
#define STM32_SDMMC_POWER_PWRCTRL_MASK        (3 << STM32_SDMMC_POWER_PWRCTRL_SHIFT)
#  define STM32_SDMMC_POWER_PWRCTRL_OFF       (0 << STM32_SDMMC_POWER_PWRCTRL_SHIFT) /* 00: Power-off: card clock stopped */
#  define STM32_SDMMC_POWER_PWRCTRL_CYCLE     (2 << STM32_SDMMC_POWER_PWRCTRL_SHIFT) /* 10: Reserved power-up */
#  define STM32_SDMMC_POWER_PWRCTRL_ON        (3 << STM32_SDMMC_POWER_PWRCTRL_SHIFT) /* 11: Power-on: card is clocked */

#define STM32_SDMMC_POWER_RESET               (0)       /* Reset value */

#define STM32_SDMMC_CLKCR_CLKDIV_SHIFT        (0)       /* Bits 9-0: Clock divide factor */
#define STM32_SDMMC_CLKCR_CLKDIV_MASK         (0x3ff << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)
#define STM32_SDMMC_CLKCR_PWRSAV              (1 << 12)  /* Bit 12: Power saving configuration bit */
#define STM32_SDMMC_CLKCR_WIDBUS_SHIFT        (14)      /* Bits 15-14: Wide bus mode enable bits */
#define STM32_SDMMC_CLKCR_WIDBUS_MASK         (3 << STM32_SDMMC_CLKCR_WIDBUS_SHIFT)
#  define STM32_SDMMC_CLKCR_WIDBUS_D1         (0 << STM32_SDMMC_CLKCR_WIDBUS_SHIFT) /* 00: Default (STM32_SDMMC_D0) */
#  define STM32_SDMMC_CLKCR_WIDBUS_D4         (1 << STM32_SDMMC_CLKCR_WIDBUS_SHIFT) /* 01: 4-wide (STM32_SDMMC_D[3:0]) */
#  define STM32_SDMMC_CLKCR_WIDBUS_D8         (2 << STM32_SDMMC_CLKCR_WIDBUS_SHIFT) /* 10: 8-wide (STM32_SDMMC_D[7:0]) */
#define STM32_SDMMC_CLKCR_NEGEDGE             (1 << 16) /* Bit 16: STM32_SDMMC_CK dephasing selection bit */
#define STM32_SDMMC_CLKCR_HWFC_EN             (1 << 17) /* Bit 17: HW Flow Control enable */
#define STM32_SDMMC_CLKCR_DDR                 (1 << 18) /* Bit 18: Data rate signaling selection */
#define STM32_SDMMC_CLKCR_BUS_SPEED           (1 << 19) /* Bit 19: Bus speed mode selection */
#define STM32_SDMMC_CLKCR_SELCLKRX_SHIFT      (20)      /* Bits 21-20: Receive clock selection */
#define STM32_SDMMC_CLKCR_SELCLKRX_MASK       (3 << STM32_SDMMC_CLKCR_SELCLKRX_SHIFT)
#  define STM32_SDMMC_CLKCR_SELCLKRX_IO_IN    (0 << STM32_SDMMC_CLKCR_SELCLKRX_SHIFT) /* 00: sdmmc_io_in_ck selected as receive clock */
#  define STM32_SDMMC_CLKCR_SELCLKRX_CKIN     (1 << STM32_SDMMC_CLKCR_SELCLKRX_SHIFT) /* 01: SDMMC_CKIN feedback clock selected as receive clock */
#  define STM32_SDMMC_CLKCR_SELCLKRX_FB       (2 << STM32_SDMMC_CLKCR_SELCLKRX_SHIFT) /* 10: sdmmc_fb_ck tuned feedback clock selected as receive clock. */
                                                                                      /* 11: Reserved (select sdmmc_io_in_ck) */

#define STM32_SDMMC_CLKCR_RESET               (0)       /* Reset value */

#define STM32_SDMMC_ARG_RESET                 (0)       /* Reset value */

#define STM32_SDMMC_CMD_CMDINDEX_SHIFT        (0)
#define STM32_SDMMC_CMD_CMDINDEX_MASK         (0x3f << STM32_SDMMC_CMD_CMDINDEX_SHIFT)
#define STM32_SDMMC_CMD_CMDTRANS              (1 << 6)  /* Bit 6: The CPSM treats the command as a data transfer command */
#define STM32_SDMMC_CMD_CMDSTOP               (1 << 7)  /* Bit 7: The CPSM treats the command as a Stop Transmission command */
#define STM32_SDMMC_CMD_WAITRESP_SHIFT        (8)       /* Bits 8-9: Wait for response bits */
#define STM32_SDMMC_CMD_WAITRESP_MASK         (3 << STM32_SDMMC_CMD_WAITRESP_SHIFT)
#  define STM32_SDMMC_CMD_NORESPONSE          (0 << STM32_SDMMC_CMD_WAITRESP_SHIFT) /* 00/10: No response */
#  define STM32_SDMMC_CMD_SHORTRESPONSE       (1 << STM32_SDMMC_CMD_WAITRESP_SHIFT) /* 01: Short response */
#  define STM32_SDMMC_CMD_SHORTRESPONSE_NOCRC (2 << STM32_SDMMC_CMD_WAITRESP_SHIFT) /* 01: Short response, no CRC */
#  define STM32_SDMMC_CMD_LONGRESPONSE        (3 << STM32_SDMMC_CMD_WAITRESP_SHIFT) /* 11: Long response */
#define STM32_SDMMC_CMD_WAITINT               (1 << 10)  /* Bit 10: CPSM waits for interrupt request */
#define STM32_SDMMC_CMD_WAITPEND              (1 << 11)  /* Bit 11: CPSM Waits for ends of data transfer */
#define STM32_SDMMC_CMD_CPSMEN                (1 << 12) /* Bit 12: Command path state machine enable */
#define STM32_SDMMC_CMD_DTHOLD                (1 << 13) /* Bit 13: Hold new data block transmission and reception in the DPSM */
#define STM32_SDMMC_CMD_BOOTMODE              (1 << 14) /* Bit 14: Select bootmode */
#define STM32_SDMMC_CMD_BOOTEN                (1 << 15) /* Bit 15: Enable boot mode */
#define STM32_SDMMC_CMD_SUSPEND               (1 << 16) /* Bit 16: SD I/O suspend command */

#define STM32_SDMMC_CMD_RESET                 (0)       /* Reset value */

#define STM32_SDMMC_RESPCMD_SHIFT             (0)
#define STM32_SDMMC_RESPCMD_MASK              (0x3f << STM32_SDMMC_RESPCMD_SHIFT)

#define STM32_SDMMC_DTIMER_RESET              (0)       /* Reset value */

#define STM32_SDMMC_DLEN_SHIFT                (0)
#define STM32_SDMMC_DLEN_MASK                 (0x01ffffff << STM32_SDMMC_DLEN_SHIFT)

#define STM32_SDMMC_DLEN_RESET                (0)       /* Reset value */

#define STM32_SDMMC_DCTRL_DTEN                (1 << 0)  /* Bit 0: Data transfer enabled bit */
#define STM32_SDMMC_DCTRL_DTDIR               (1 << 1)  /* Bit 1: Data transfer direction */
#define STM32_SDMMC_DCTRL_DTMODE_SHIFT        (2)  /* Bits 2-3: Data transfer mode */
#define STM32_SDMMC_DCTRL_DTMODE_MASK         (3 << STM32_SDMMC_DCTRL_DTMODE_SHIFT)
#  define STM32_SDMMC_DCTRL_DTMODE_END        (0 << STM32_SDMMC_DCTRL_DTMODE_SHIFT)
#  define STM32_SDMMC_DCTRL_DTMODE_SDIO       (1 << STM32_SDMMC_DCTRL_DTMODE_SHIFT)
#  define STM32_SDMMC_DCTRL_DTMODE_EMMC       (2 << STM32_SDMMC_DCTRL_DTMODE_SHIFT)
#  define STM32_SDMMC_DCTRL_DTMODE_BLOCK      (3 << STM32_SDMMC_DCTRL_DTMODE_SHIFT)

#define STM32_SDMMC_DCTRL_DBLOCKSIZE_SHIFT    (4)       /* Bits 7-4: Data block size */
#define STM32_SDMMC_DCTRL_DBLOCKSIZE_MASK     (0xf << STM32_SDMMC_DCTRL_DBLOCKSIZE_SHIFT)
#  define STM32_SDMMC_DCTRL_1BYTE             (0 << STM32_SDMMC_DCTRL_DBLOCKSIZE_SHIFT)
#  define STM32_SDMMC_DCTRL_2BYTES            (1 << STM32_SDMMC_DCTRL_DBLOCKSIZE_SHIFT)
#  define STM32_SDMMC_DCTRL_4BYTES            (2 << STM32_SDMMC_DCTRL_DBLOCKSIZE_SHIFT)
#  define STM32_SDMMC_DCTRL_8BYTES            (3 << STM32_SDMMC_DCTRL_DBLOCKSIZE_SHIFT)
#  define STM32_SDMMC_DCTRL_16BYTES           (4 << STM32_SDMMC_DCTRL_DBLOCKSIZE_SHIFT)
#  define STM32_SDMMC_DCTRL_32BYTES           (5 << STM32_SDMMC_DCTRL_DBLOCKSIZE_SHIFT)
#  define STM32_SDMMC_DCTRL_64BYTES           (6 << STM32_SDMMC_DCTRL_DBLOCKSIZE_SHIFT)
#  define STM32_SDMMC_DCTRL_128BYTES          (7 << STM32_SDMMC_DCTRL_DBLOCKSIZE_SHIFT)
#  define STM32_SDMMC_DCTRL_256BYTES          (8 << STM32_SDMMC_DCTRL_DBLOCKSIZE_SHIFT)
#  define STM32_SDMMC_DCTRL_512BYTES          (9 << STM32_SDMMC_DCTRL_DBLOCKSIZE_SHIFT)
#  define STM32_SDMMC_DCTRL_1KBYTE            (10 << STM32_SDMMC_DCTRL_DBLOCKSIZE_SHIFT)
#  define STM32_SDMMC_DCTRL_2KBYTES           (11 << STM32_SDMMC_DCTRL_DBLOCKSIZE_SHIFT)
#  define STM32_SDMMC_DCTRL_4KBYTES           (12 << STM32_SDMMC_DCTRL_DBLOCKSIZE_SHIFT)
#  define STM32_SDMMC_DCTRL_8KBYTES           (13 << STM32_SDMMC_DCTRL_DBLOCKSIZE_SHIFT)
#  define STM32_SDMMC_DCTRL_16KBYTES          (14 << STM32_SDMMC_DCTRL_DBLOCKSIZE_SHIFT)
#define STM32_SDMMC_DCTRL_RWSTART             (1 << 8)  /* Bit 8: Read wait start */
#define STM32_SDMMC_DCTRL_RWSTOP              (1 << 9)  /* Bit 9: Read wait stop */
#define STM32_SDMMC_DCTRL_RWMOD               (1 << 10) /* Bit 10: Read wait mode */
#define STM32_SDMMC_DCTRL_SDIOEN              (1 << 11) /* Bit 11: SD I/O enable functions */
#define STM32_SDMMC_DCTRL_BOOTACKEN           (1 << 12) /* Bit 12: Enable reception of boot ack */
#define STM32_SDMMC_DCTRL_FIFORST             (1 << 13) /* Bit 13: FIFO reset */

#define STM32_SDMMC_DCTRL_RESET               (0)       /* Reset value */

#define STM32_SDMMC_DCOUNT_SHIFT              (0)
#define STM32_SDMMC_DCOUNT_MASK               (0x01ffffff << STM32_SDMMC_DCOUNT_SHIFT)

#define STM32_SDMMC_STA_CCRCFAIL              (1 << 0)  /* Bit 0: Command response CRC fail */
#define STM32_SDMMC_STA_DCRCFAIL              (1 << 1)  /* Bit 1: Data block CRC fail */
#define STM32_SDMMC_STA_CTIMEOUT              (1 << 2)  /* Bit 2: Command response timeout */
#define STM32_SDMMC_STA_DTIMEOUT              (1 << 3)  /* Bit 3: Data timeout */
#define STM32_SDMMC_STA_TXUNDERR              (1 << 4)  /* Bit 4: Transmit FIFO underrun error */
#define STM32_SDMMC_STA_RXOVERR               (1 << 5)  /* Bit 5: Received FIFO overrun error */
#define STM32_SDMMC_STA_CMDREND               (1 << 6)  /* Bit 6: Command response received  */
#define STM32_SDMMC_STA_CMDSENT               (1 << 7)  /* Bit 7: Command sent  */
#define STM32_SDMMC_STA_DATAEND               (1 << 8)  /* Bit 8: Data end */
#define STM32_SDMMC_STA_DHOLD                 (1 << 9)  /* Bit 9: Data transfer hold  */
#define STM32_SDMMC_STA_DBCKEND               (1 << 10) /* Bit 10: Data block sent/received  */
#define STM32_SDMMC_STA_DABORT                (1 << 11) /* Bit 11: Data transfer aborted by CMD12 */
#define STM32_SDMMC_STA_DPSMACT               (1 << 12) /* Bit 12: Data path state machine active */
#define STM32_SDMMC_STA_CPSMACT               (1 << 13) /* Bit 13: Command path state machine active */
#define STM32_SDMMC_STA_TXFIFOHE              (1 << 14) /* Bit 14: Transmit FIFO half empty */
#define STM32_SDMMC_STA_RXFIFOHF              (1 << 15) /* Bit 15: Receive FIFO half full */
#define STM32_SDMMC_STA_TXFIFOF               (1 << 16) /* Bit 16: Transmit FIFO full */
#define STM32_SDMMC_STA_RXFIFOF               (1 << 17) /* Bit 17: Receive FIFO full */
#define STM32_SDMMC_STA_TXFIFOE               (1 << 18) /* Bit 18: Transmit FIFO empty */
#define STM32_SDMMC_STA_RXFIFOE               (1 << 19) /* Bit 19: Receive FIFO empty */
#define STM32_SDMMC_STA_BUSYD0                (1 << 20) /* Bit 20: Card signals busy on SDMMC_D0 */
#define STM32_SDMMC_STA_BUSYD0END             (1 << 21) /* Bit 21: card SDMMC_D0 signal changed from busy to NOT busy */
#define STM32_SDMMC_STA_SDIOIT                (1 << 22) /* Bit 22: SDIO interrupt received */
#define STM32_SDMMC_STA_ACKFAIL               (1 << 23) /* Bit 23: Boot ack check fail */
#define STM32_SDMMC_STA_ACKTIMEOUT            (1 << 24) /* Bit 24: Boot ack timeout */
#define STM32_SDMMC_STA_VSWEND                (1 << 25) /* Bit 25: Voltage switch critical timing section completion */
#define STM32_SDMMC_STA_CKSTOP                (1 << 26) /* Bit 26: SDMMC_CK stopped in Voltage switch procedure */
#define STM32_SDMMC_STA_IDMATE                (1 << 27) /* Bit 27: IDMA transfer error */
#define STM32_SDMMC_STA_IDMABTC               (1 << 28) /* Bit 28: IDMA buffer transfer complete */

#define STM32_SDMMC_ICR_CCRCFAILC             (1 << 0)  /* Bit 0: CCRCFAIL flag clear bit */
#define STM32_SDMMC_ICR_DCRCFAILC             (1 << 1)  /* Bit 1: DCRCFAIL flag clear bit */
#define STM32_SDMMC_ICR_CTIMEOUTC             (1 << 2)  /* Bit 2: CTIMEOUT flag clear bit */
#define STM32_SDMMC_ICR_DTIMEOUTC             (1 << 3)  /* Bit 3: DTIMEOUT flag clear bit */
#define STM32_SDMMC_ICR_TXUNDERRC             (1 << 4)  /* Bit 4: TXUNDERR flag clear bit */
#define STM32_SDMMC_ICR_RXOVERRC              (1 << 5)  /* Bit 5: RXOVERR flag clear bit */
#define STM32_SDMMC_ICR_CMDRENDC              (1 << 6)  /* Bit 6: CMDREND flag clear bit */
#define STM32_SDMMC_ICR_CMDSENTC              (1 << 7)  /* Bit 7: CMDSENT flag clear bit */
#define STM32_SDMMC_ICR_DATAENDC              (1 << 8)  /* Bit 8: DATAEND flag clear bit */
#define STM32_SDMMC_ICR_DHOLDC                (1 << 9)  /* Bit 9: DHOLD flag clear bit */
#define STM32_SDMMC_ICR_DBCKENDC              (1 << 10) /* Bit 10: DBCKEND flag clear bit */
#define STM32_SDMMC_ICR_DABORTC               (1 << 11) /* Bit 11: DABORT flag clear bit */
#define STM32_SDMMC_ICR_BUSYD0ENDC            (1 << 21) /* Bit 21: BUSYD0END flag clear bit */
#define STM32_SDMMC_ICR_SDIOITC               (1 << 22) /* Bit 22: SDIOIT flag clear bit */
#define STM32_SDMMC_ICR_ACKFAILC              (1 << 23) /* Bit 23: ACKFAIL flag clear bit */
#define STM32_SDMMC_ICR_ACKTIMEOUTC           (1 << 24) /* Bit 24: ACKTIMEOUT flag clear bit */
#define STM32_SDMMC_ICR_VSWENDC               (1 << 25) /* Bit 25: VSWEND flag clear bit */
#define STM32_SDMMC_ICR_CKSTOPC               (1 << 26) /* Bit 26: CKSTOP flag clear bit */
#define STM32_SDMMC_ICR_IDMATEC               (1 << 27) /* Bit 27: IDMA transfer error clear bit */
#define STM32_SDMMC_ICR_IDMABTCC              (1 << 28) /* Bit 28: IDMA buffer transfer complete clear bit */

#define STM32_SDMMC_ICR_RESET                 0x1fe00fff
#define STM32_SDMMC_ICR_ALLFLAGS              0x1fe00fff

#define STM32_SDMMC_MASK_CCRCFAILIE           (1 << 0)  /* Bit 0: Command CRC fail interrupt enable */
#define STM32_SDMMC_MASK_DCRCFAILIE           (1 << 1)  /* Bit 1: Data CRC fail interrupt enable */
#define STM32_SDMMC_MASK_CTIMEOUTIE           (1 << 2)  /* Bit 2: Command timeout interrupt enable */
#define STM32_SDMMC_MASK_DTIMEOUTIE           (1 << 3)  /* Bit 3: Data timeout interrupt enable */
#define STM32_SDMMC_MASK_TXUNDERRIE           (1 << 4)  /* Bit 4: Tx FIFO underrun error interrupt enable */
#define STM32_SDMMC_MASK_RXOVERRIE            (1 << 5)  /* Bit 5: Rx FIFO overrun error interrupt enable */
#define STM32_SDMMC_MASK_CMDRENDIE            (1 << 6)  /* Bit 6: Command response received interrupt enable */
#define STM32_SDMMC_MASK_CMDSENTIE            (1 << 7)  /* Bit 7: Command sent interrupt enable */
#define STM32_SDMMC_MASK_DATAENDIE            (1 << 8)  /* Bit 8: Data end interrupt enable */
#define STM32_SDMMC_MASK_DHOLDIE              (1 << 9)  /* Bit 9: Data hold interrupt enable */
#define STM32_SDMMC_MASK_DBCKENDIE            (1 << 10) /* Bit 10: Data block end interrupt enable */
#define STM32_SDMMC_MASK_DABOTRTIE            (1 << 11) /* Bit 11: Data transfer aborted interrupt enable */
#define STM32_SDMMC_MASK_TXFIFOHEIE           (1 << 14) /* Bit 14: Tx FIFO half empty interrupt enable */
#define STM32_SDMMC_MASK_RXFIFOHFIE           (1 << 15) /* Bit 15: Rx FIFO half full interrupt enable */
#define STM32_SDMMC_MASK_RXFIFOFIE            (1 << 17) /* Bit 17: Rx FIFO full interrupt enable */
#define STM32_SDMMC_MASK_TXFIFOFIE            (1 << 18) /* Bit 18: Tx FIFO full interrupt enable */
#define STM32_SDMMC_MASK_BUSYD0ENDIE          (1 << 21) /* Bit 21: BUSYD0END interrupt enable */
#define STM32_SDMMC_MASK_SDIOITIE             (1 << 22) /* Bit 22: SDIO mode interrupt received interrupt enable */
#define STM32_SDMMC_MASK_ACKFAILIE            (1 << 23) /* Bit 23: Acknowledgment Fail interrupt enable */
#define STM32_SDMMC_MASK_ACKTIMEOUTIE         (1 << 24) /* Bit 24: Acknowledgment timeout interrupt enable */
#define STM32_SDMMC_MASK_VSWENDIE             (1 << 25) /* Bit 25: Voltage switch critical timing section completion interrupt enable */
#define STM32_SDMMC_MASK_CKSTOPIE             (1 << 26) /* Bit 26: Voltage Switch clock stopped interrupt enable */
#define STM32_SDMMC_MASK_IDMABTCIE            (1 << 28) /* Bit 28: IDMA buffer transfer complete interrupt enable */

#define STM32_SDMMC_MASK_RESET                (0)

#define STM32_SDMMC_IDMACTRLR_IDMAEN          (1 << 0) /* Bit 0: IDMA enable */
#define STM32_SDMMC_IDMACTRLR_IDMABMODE       (1 << 1) /* Bit 1: Buffer mode selection */
#define STM32_SDMMC_IDMACTRLR_IDMABACT        (1 << 2) /* Bit 2: Double buffer mode active buffer indication */

#define STM32_SDMMC_IDMABSIZER_SHIFT          (5)      /* Bits 12-5: Number of bytes per buffer divided by 32 */
#define STM32_SDMMC_IDMABSIZER_MASK           (0xff << STM32_SDMMC_IDMABSIZER_SHIFT)

#define STM32_SDMMC_FIFOCNT_SHIFT             (0)
#define STM32_SDMMC_FIFOCNT_MASK              (0x0ffffff << STM32_SDMMC_FIFOCNT_SHIFT)

#endif /* __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32H7X3XX_SDMMC_H */
