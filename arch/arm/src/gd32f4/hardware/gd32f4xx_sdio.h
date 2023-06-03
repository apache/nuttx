/****************************************************************************
 * arch/arm/src/gd32f4/hardware/gd32f4xx_sdio.h
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

#ifndef __ARCH_ARM_SRC_GD32F4_HARDWARE_GD32F4XX_SDIO_H
#define __ARCH_ARM_SRC_GD32F4_HARDWARE_GD32F4XX_SDIO_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define GD32_SDIO_PWRCTL_OFFSET    0x0000 /* SDIO power control register offset */
#define GD32_SDIO_CLKCTL_OFFSET    0x0004 /* SDI clock control register offset*/
#define GD32_SDIO_CMDAGMT_OFFSET   0x0008 /* SDIO argument register offset*/
#define GD32_SDIO_CMDCTL_OFFSET    0x000C /* SDIO command register offset*/
#define GD32_SDIO_RSPCMDIDX_OFFSET 0x0010 /* SDIO command response register offset*/
#define GD32_SDIO_RESP0_OFFSET     0x0014 /* SDIO response register 0 offset*/
#define GD32_SDIO_RESP1_OFFSET     0x0018 /* SDIO response register 1 offset*/
#define GD32_SDIO_RESP2_OFFSET     0x001c /* SDIO response register 2 offset*/
#define GD32_SDIO_RESP3_OFFSET     0x0020 /* SDIO response register 3 offset*/
#define GD32_SDIO_DATATO_OFFSET    0x0024 /* SDIO data timeout register offset*/
#define GD32_SDIO_DATALEN_OFFSET   0x0028 /* SDIO data length register offset*/
#define GD32_SDIO_DATACTL_OFFSET   0x002c /* SDIO data control register offset*/
#define GD32_SDIO_DATACNT_OFFSET   0x0030 /* SDIO data counter register offset*/
#define GD32_SDIO_STAT_OFFSET      0x0034 /* SDIO status register offset*/
#define GD32_SDIO_INTC_OFFSET      0x0038 /* SDIO interrupt clear register offset*/
#define GD32_SDIO_INTEN_OFFSET     0x003c /* SDIO interrupt enable register offset */
#define GD32_SDIO_FIFOCNT_OFFSET   0x0048 /* SDIO FIFO counter register offset*/
#define GD32_SDIO_FIFO_OFFSET      0x0080 /* SDIO FIFO data register offset*/

/* Register Addresses *******************************************************/

#define GD32_SDIO_PWRCTL          (GD32_SDIO_BASE+GD32_SDIO_PWRCTL_OFFSET)    /* SDIO power control register */
#define GD32_SDIO_CLKCTL          (GD32_SDIO_BASE+GD32_SDIO_CLKCTL_OFFSET)    /* SDI clock control register */
#define GD32_SDIO_CMDAGMT         (GD32_SDIO_BASE+GD32_SDIO_CMDAGMT_OFFSET)   /* SDIO argument register */
#define GD32_SDIO_CMDCTL          (GD32_SDIO_BASE+GD32_SDIO_CMDCTL_OFFSET)    /* SDIO command register */
#define GD32_SDIO_RSPCMDIDX       (GD32_SDIO_BASE+GD32_SDIO_RSPCMDIDX_OFFSET) /* SDIO command response register */
#define GD32_SDIO_RESP0           (GD32_SDIO_BASE+GD32_SDIO_RESP0_OFFSET)     /* SDIO response register 0 */
#define GD32_SDIO_RESP1           (GD32_SDIO_BASE+GD32_SDIO_RESP1_OFFSET)     /* SDIO response register 1 */
#define GD32_SDIO_RESP2           (GD32_SDIO_BASE+GD32_SDIO_RESP2_OFFSET)     /* SDIO response register 2 */
#define GD32_SDIO_RESP3           (GD32_SDIO_BASE+GD32_SDIO_RESP3_OFFSET)     /* SDIO response register 3 */
#define GD32_SDIO_DATATO          (GD32_SDIO_BASE+GD32_SDIO_DATATO_OFFSET)    /* SDIO data timeout register */
#define GD32_SDIO_DATALEN         (GD32_SDIO_BASE+GD32_SDIO_DATALEN_OFFSET)   /* SDIO data length register */
#define GD32_SDIO_DATACTL         (GD32_SDIO_BASE+GD32_SDIO_DATACTL_OFFSET)   /* SDIO data control register */
#define GD32_SDIO_DATACNT         (GD32_SDIO_BASE+GD32_SDIO_DATACNT_OFFSET)   /* SDIO data counter register */
#define GD32_SDIO_STAT            (GD32_SDIO_BASE+GD32_SDIO_STAT_OFFSET)      /* SDIO status register */
#define GD32_SDIO_INTC            (GD32_SDIO_BASE+GD32_SDIO_INTC_OFFSET)      /* SDIO interrupt clear register */
#define GD32_SDIO_INTEN           (GD32_SDIO_BASE+GD32_SDIO_INTEN_OFFSET)     /* SDIO interrupt enable register */
#define GD32_SDIO_FIFOCNT         (GD32_SDIO_BASE+GD32_SDIO_FIFOCNT_OFFSET)   /* SDIO FIFO counter register */
#define GD32_SDIO_FIFO            (GD32_SDIO_BASE+GD32_SDIO_FIFO_OFFSET)      /* SDIO FIFO data register */

/* Register Bitfield Definitions ********************************************/

/* SDIO_PWRCTL */

#define SDIO_PWRCTL_SHIFT       (0)       /* Bits 0-1: SDIO power control bits */
#define SDIO_PWRCTL_MASK        (3 << SDIO_PWRCTL_SHIFT)
#define SDIO_PWRCTL_OFF         (0 << SDIO_PWRCTL_SHIFT) /* 00: Power-off: card clock stopped */
#define SDIO_PWRCTL_PWRUP       (2 << SDIO_PWRCTL_SHIFT) /* 10: Reserved power-up */
#define SDIO_PWRCTL_ON          (3 << SDIO_PWRCTL_SHIFT) /* 11: Power-on: card is clocked */

#define SDIO_PWRCTL_RESET               (0)       /* Reset value */

/* SDIO_CLKCTL */

#define SDIO_CLKCTL_CLKDIV_SHIFT         (0)       /* Bits 7-0: clock division */
#define SDIO_CLKCTL_CLKDIV_MASK          (0xff << SDIO_CLKCTL_CLKDIV_SHIFT)
#define SDIO_CLKCTL_CLKEN                (1 << 8)  /* Bit 8: SDIO_CLK clock output enable bit */
#define SDIO_CLKCTL_CLKPWRSAV            (1 << 9)  /* Bit 9: SDIO_CLK clock dynamic switch on/off for power saving */
#define SDIO_CLKCTL_CLKBYP               (1 << 10) /* Bit 10: clock bypass enable bit */
#define SDIO_CLKCTL_WIDBUS_SHIFT         (11)      /* Bits 12-11: SDIO card bus mode control bit */
#define SDIO_CLKCTL_WIDBUS_MASK          (3 << SDIO_CLKCTL_WIDBUS_SHIFT)
#define SDIO_CLKCTL_WIDBUS_D1            (0 << SDIO_CLKCTL_WIDBUS_SHIFT) /* 00: Default (SDIO_D0) */
#define SDIO_CLKCTL_WIDBUS_D4            (1 << SDIO_CLKCTL_WIDBUS_SHIFT) /* 01: 4-wide (SDIO_D[3:0]) */
#define SDIO_CLKCTL_WIDBUS_D8            (2 << SDIO_CLKCTL_WIDBUS_SHIFT) /* 10: 8-wide (SDIO_D[7:0]) */

#define SDIO_CLKCTL_NEGEDGE              (1 << 13) /* Bit 13: SDIO_CK dephasing selection bit */
#define SDIO_CLKCTL_HWFC_EN              (1 << 14) /* Bit 14: HW Flow Control enable */

#define SDIO_CLKCTL_RESET                (0)       /* Reset value */

/* SDIO_CMDAGMT */

#define SDIO_CMDAGMT_RESET               (0)       /* Reset value */

/* SDIO_CMDCTL */

#define SDIO_CMDCTL_CMDINDEX_SHIFT       (0)
#define SDIO_CMDCTL_CMDINDEX_MASK        (0x3f << SDIO_CMDCTL_CMDINDEX_SHIFT)
#define SDIO_CMDCTL_WAITRESP_SHIFT       (6)       /* Bits 7-6: command response type bits */
#define SDIO_CMDCTL_WAITRESP_MASK        (3 << SDIO_CMDCTL_WAITRESP_SHIFT)
#define SDIO_CMDCTL_NORESPONSE           (0 << SDIO_CMDCTL_WAITRESP_SHIFT) /* 00/10: No response */
#define SDIO_CMDCTL_SHORTRESPONSE        (1 << SDIO_CMDCTL_WAITRESP_SHIFT) /* 01: Short response */
#define SDIO_CMDCTL_LONGRESPONSE         (3 << SDIO_CMDCTL_WAITRESP_SHIFT) /* 11: Long response */

#define SDIO_CMDCTL_INTWAIT              (1 << 8)  /* Bit 8: nterrupt wait instead of timeout */
#define SDIO_CMDCTL_WAITDEND             (1 << 9)  /* Bit 9: wait for ends of data transfer */
#define SDIO_CMDCTL_CSMEN                (1 << 10) /* Bit 10: command state machine(CSM) enable bit */
#define SDIO_CMDCTL_SUSPEND              (1 << 11) /* Bit 11: SD I/O suspend command(SD I/O only) */
#define SDIO_CMDCTL_ENDCMDC              (1 << 12) /* Bit 12: CMD completion signal enabled (CE-ATA only) */
#define SDIO_CMDCTL_NINTEN               (1 << 13) /* Bit 13: no CE-ATA interrupt (CE-ATA only) */
#define SDIO_CMDCTL_ATAEN                (1 << 14) /* Bit 14: CE-ATA command enable(CE-ATA only) */

#define SDIO_CMDCTL_RESET                (0)       /* Reset value */

/* SDIO_RSPCMDIDX */

#define SDIO_RSPCMDIDX_SHIFT             (0)
#define SDIO_RSPCMDIDX_MASK              (0x3f << SDIO_RSPCMDIDX_SHIFT)

/* SDIO_DATATO */

#define SDIO_DATATO_RESET                (0)       /* Reset value */

/* SDIO_DATALEN */

#define SDIO_DATALEN_SHIFT               (0)
#define SDIO_DATALEN_MASK                (0x01ffffff << SDIO_DATALEN_SHIFT)

#define SDIO_DATALEN_RESET               (0)       /* Reset value */

/* SDIO_DATACTL */

#define SDIO_DATACTL_DATAEN              (1 << 0)  /* Bit 0: data transfer enabled bit */
#define SDIO_DATACTL_DATADIR             (1 << 1)  /* Bit 1: data transfer direction */
#define SDIO_DATACTL_TRANSMOD            (1 << 2)  /* Bit 2: data transfer mode */
#define SDIO_DATACTL_DMAEN               (1 << 3)  /* Bit 3: DMA enable bit */
#define SDIO_DATACTL_DBLOCKSIZE_SHIFT    (4)       /* Bits 7-4: data block size */
#define SDIO_DATACTL_DBLOCKSIZE_MASK     (15 << SDIO_DATACTL_DBLOCKSIZE_SHIFT)
#define SDIO_DATACTL_1BYTE               (0 << SDIO_DATACTL_DBLOCKSIZE_SHIFT)
#define SDIO_DATACTL_2BYTES              (1 << SDIO_DATACTL_DBLOCKSIZE_SHIFT)
#define SDIO_DATACTL_4BYTES              (2 << SDIO_DATACTL_DBLOCKSIZE_SHIFT)
#define SDIO_DATACTL_8BYTES              (3 << SDIO_DATACTL_DBLOCKSIZE_SHIFT)
#define SDIO_DATACTL_16BYTES             (4 << SDIO_DATACTL_DBLOCKSIZE_SHIFT)
#define SDIO_DATACTL_32BYTES             (5 << SDIO_DATACTL_DBLOCKSIZE_SHIFT)
#define SDIO_DATACTL_64BYTES             (6 << SDIO_DATACTL_DBLOCKSIZE_SHIFT)
#define SDIO_DATACTL_128BYTES            (7 << SDIO_DATACTL_DBLOCKSIZE_SHIFT)
#define SDIO_DATACTL_256BYTES            (8 << SDIO_DATACTL_DBLOCKSIZE_SHIFT)
#define SDIO_DATACTL_512BYTES            (9 << SDIO_DATACTL_DBLOCKSIZE_SHIFT)
#define SDIO_DATACTL_1KBYTE              (10 << SDIO_DATACTL_DBLOCKSIZE_SHIFT)
#define SDIO_DATACTL_2KBYTES             (11 << SDIO_DATACTL_DBLOCKSIZE_SHIFT)
#define SDIO_DATACTL_4KBYTES             (12 << SDIO_DATACTL_DBLOCKSIZE_SHIFT)
#define SDIO_DATACTL_8KBYTES             (13 << SDIO_DATACTL_DBLOCKSIZE_SHIFT)
#define SDIO_DATACTL_16KBYTES            (14 << SDIO_DATACTL_DBLOCKSIZE_SHIFT)
#define SDIO_DATACTL_RWEN                (1 << 8)  /* Bit 8: read wait mode enabled(SD I/O only) */
#define SDIO_DATACTL_RWSTOP              (1 << 9)  /* Bit 9: read wait stop(SD I/O only) */
#define SDIO_DATACTL_RWTYPE              (1 << 10) /* Bit 10: read wait type(SD I/O only) */
#define SDIO_DATACTL_IOEN                (1 << 11) /* Bit 11: SD I/O specific function enable(SD I/O only) */

#define SDIO_DATACTL_RESET               (0)       /* Reset value */

/* SDIO_DATACOUNT */

#define SDIO_DATACOUNT_SHIFT           (0)
#define SDIO_DATACOUNT_MASK            (0x01ffffff << SDIO_DATACOUNT_SHIFT)

/* SDIO_STAT */

#define SDIO_STAT_CCRCERR               (1 << 0)  /* Bit 0: command response received (CRC check failed) */
#define SDIO_STAT_DTCRCERR              (1 << 1)  /* Bit 1: data block sent/received (CRC check failed) */
#define SDIO_STAT_CMDTMOUT              (1 << 2)  /* Bit 2: command response timeout */
#define SDIO_STAT_DTTMOUT               (1 << 3)  /* Bit 3: data timeout */
#define SDIO_STAT_TXURE                 (1 << 4)  /* Bit 4: transmit FIFO underrun error occurs */
#define SDIO_STAT_RXORE                 (1 << 5)  /* Bit 5: received FIFO overrun error occurs */
#define SDIO_STAT_CMDRECV               (1 << 6)  /* Bit 6: command response received (CRC check passed)  */
#define SDIO_STAT_CMDSEND               (1 << 7)  /* Bit 7: command sent (no response required)  */
#define SDIO_STAT_DTEND                 (1 << 8)  /* Bit 8: data end (data counter, SDIO_DATACNT, is zero) */
#define SDIO_STAT_STBITE                (1 << 9)  /* Bit 9: start bit error in the bus  */
#define SDIO_STAT_DTBLKEND              (1 << 10) /* Bit 10:  data block sent/received (CRC check passed)  */
#define SDIO_STAT_CMDRUN                (1 << 11) /* Bit 11: command transmission in progress */
#define SDIO_STAT_TXRUN                 (1 << 12) /* Bit 12: data transmission in progress */
#define SDIO_STAT_RXRUN                 (1 << 13) /* Bit 13: data reception in progress */
#define SDIO_STAT_TFH                   (1 << 14) /* Bit 14: transmit FIFO is half empty: at least 8 words can be written into the FIFO */
#define SDIO_STAT_RFH                   (1 << 15) /* Bit 15: receive FIFO is half full: at least 8 words can be read in the FIFO */
#define SDIO_STAT_TFF                   (1 << 16) /* Bit 16: transmit FIFO is full */
#define SDIO_STAT_RFF                   (1 << 17) /* Bit 17: receive FIFO is full */
#define SDIO_STAT_TFE                   (1 << 18) /* Bit 18: transmit FIFO is empty */
#define SDIO_STAT_RFE                   (1 << 19) /* Bit 19: receive FIFO is empty */
#define SDIO_STAT_TXDTVAL               (1 << 20) /* Bit 20: data is valid in transmit FIFO */
#define SDIO_STAT_RXDTVAL               (1 << 21) /* Bit 21: data is valid in receive FIFO */
#define SDIO_STAT_SDIOINT               (1 << 22) /* Bit 22: SD I/O interrupt received */
#define SDIO_STAT_ATAEND                (1 << 23) /* Bit 23: CE-ATA command completion signal received (only for CMD61) */

/* SDIO_INTC */

#define SDIO_INTC_CCRCERRC              (1 << 0)  /* Bit 0: CCRCERR flag clear bit */
#define SDIO_INTC_DTCRCERRC             (1 << 1)  /* Bit 1: DTCRCERR flag clear bit */
#define SDIO_INTC_CMDTMOUTC             (1 << 2)  /* Bit 2: CMDTMOUT flag clear bit */
#define SDIO_INTC_DTTMOUTC              (1 << 3)  /* Bit 3: DTTMOUT flag clear bit */
#define SDIO_INTC_TXUREC                (1 << 4)  /* Bit 4: TXURE flag clear bit */
#define SDIO_INTC_RXOREC                (1 << 5)  /* Bit 5: RXORE flag clear bit */
#define SDIO_INTC_CMDRECVC              (1 << 6)  /* Bit 6: CMDRECV flag clear bit */
#define SDIO_INTC_CMDSENDC              (1 << 7)  /* Bit 7: CMDSEND flag clear bit */
#define SDIO_INTC_DTENDC                (1 << 8)  /* Bit 8: DTEND flag clear bit */
#define SDIO_INTC_STBITEC               (1 << 9)  /* Bit 9: STBITE flag clear bit */
#define SDIO_INTC_DTBLKENDC             (1 << 10) /* Bit 10: DTBLKEND flag clear bit */
#define SDIO_INTC_SDIOINTC              (1 << 22) /* Bit 22: SDIOINT flag clear bit */
#define SDIO_INTC_ATAENDC               (1 << 23) /* Bit 23: ATAEND flag clear bit */

#define SDIO_INTC_RESET                 0x00c007ff
#define SDIO_INTC_STATICFLAGS           0x000005ff

/* SDIO_INTEN */

#define SDIO_INTEN_CCRCERRIE            (1 << 0)  /* Bit 0: command response CRC fail interrupt enable */
#define SDIO_INTEN_DTCRCERRIE           (1 << 1)  /* Bit 1: data CRC fail interrupt enable */
#define SDIO_INTEN_CMDTMOUTIE           (1 << 2)  /* Bit 2: command response timeout interrupt enable */
#define SDIO_INTEN_DTTMOUTIE            (1 << 3)  /* Bit 3: data timeout interrupt enable */
#define SDIO_INTEN_TXUREIE              (1 << 4)  /* Bit 4: transmit FIFO underrun error interrupt enable */
#define SDIO_INTEN_RXOREIE              (1 << 5)  /* Bit 5: received FIFO overrun error interrupt enable */
#define SDIO_INTEN_CMDRECVIE            (1 << 6)  /* Bit 6: command response received interrupt enable */
#define SDIO_INTEN_CMDSENDIE            (1 << 7)  /* Bit 7: command sent interrupt enable */
#define SDIO_INTEN_DTENDIE              (1 << 8)  /* Bit 8: data end interrupt enable */
#define SDIO_INTEN_STBITEIE             (1 << 9)  /* Bit 9: start bit error interrupt enable */
#define SDIO_INTEN_DTBLKENDIE           (1 << 10) /* Bit 10: data block end interrupt enable */
#define SDIO_INTEN_CMDRUNIE             (1 << 11) /* Bit 11: command transmission interrupt enable */
#define SDIO_INTEN_TXRUNIE              (1 << 12) /* Bit 12: data transmission interrupt enable */
#define SDIO_INTEN_RXRUNIE              (1 << 13) /* Bit 13: data reception interrupt enable */
#define SDIO_INTEN_TFHIE                (1 << 14) /* Bit 14: transmit FIFO half empty interrupt enable */
#define SDIO_INTEN_RFHIE                (1 << 15) /* Bit 15: receive FIFO half full interrupt enable */
#define SDIO_INTEN_TFFIE                (1 << 16) /* Bit 16: transmit FIFO full interrupt enable */
#define SDIO_INTEN_RFFIE                (1 << 17) /* Bit 17: receive FIFO full interrupt enable */
#define SDIO_INTEN_TFEIE                (1 << 18) /* Bit 18: transmit FIFO empty interrupt enable */
#define SDIO_INTEN_RFEIE                (1 << 19) /* Bit 19: receive FIFO empty interrupt enable */
#define SDIO_INTEN_TXDTVALIE            (1 << 20) /* Bit 20: data valid in transmit FIFO interrupt enable */
#define SDIO_INTEN_RXDTVALIE            (1 << 21) /* Bit 21: data valid in receive FIFO interrupt enable */
#define SDIO_INTEN_SDIOINTIE            (1 << 22) /* Bit 22: SD I/O interrupt received interrupt enable */
#define SDIO_INTEN_ATAENDIE             (1 << 23) /* Bit 23: CE-ATA command completion signal received interrupt enable */

#define SDIO_INTEN_RESET                (0)

/* SDIO dividers. Note that slower clocking is required when DMA is disabled
 * in order to avoid RX overrun/TX underrun errors due to delayed responses
 * to service FIFOs in interrupt driven mode.  These values have not been
 * tuned!!!
 *
 * SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(118+2)=400 KHz
 */

#define SDIO_INIT_CLKDIV (118 << SDIO_CLKCTL_CLKDIV_SHIFT)

/* DMA ON:  SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(1+2)=16 MHz
 * DMA OFF: SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(2+2)=12 MHz
 */

#ifdef CONFIG_SDIO_DMA
#  define SDIO_MMCXFR_CLKDIV (1 << SDIO_CLKCTL_CLKDIV_SHIFT)
#else
#  define SDIO_MMCXFR_CLKDIV (2 << SDIO_CLKCTL_CLKDIV_SHIFT)
#endif

#ifdef CONFIG_SDIO_DMA
#  define SDIO_SDXFR_CLKDIV (1 << SDIO_CLKCTL_CLKDIV_SHIFT)
#else
#  define SDIO_SDXFR_CLKDIV (2 << SDIO_CLKCTL_CLKDIV_SHIFT)
#endif

#endif /* __ARCH_ARM_SRC_GD32F4_HARDWARE_GD32F4XX_SDIO_H */

