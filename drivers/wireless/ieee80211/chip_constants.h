/*
 * Copyright (c) 2015 Broadcom
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * 3. Neither the name of Broadcom nor the names of other contributors to this
 * software may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * 4. This software may not be used as a standalone product, and may only be used as
 * incorporated in your product or device that incorporates Broadcom wireless connectivity
 * products and solely for the purpose of enabling the functionalities of such Broadcom products.
 *
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY WARRANTIES OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT, ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef INCLUDED_CHIP_CONSTANTS_H_
#define INCLUDED_CHIP_CONSTANTS_H_

// #include "wwd_wlioctl.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *             Chip Constants
 ******************************************************/

#define BCM_43362_CHIP_ID 43362

/******************************************************
 *             Architecture Constants
 ******************************************************/

/* General chip stats */
#define CHIP_RAM_SIZE      0x3C000

/* Backplane architecture */
#define CHIPCOMMON_BASE_ADDRESS  0x18000000    /* Chipcommon core register region   */
#define DOT11MAC_BASE_ADDRESS    0x18001000    /* dot11mac core register region     */
#define SDIO_BASE_ADDRESS        0x18002000    /* SDIOD Device core register region */
#define WLAN_ARMCM3_BASE_ADDRESS 0x18003000    /* ARMCM3 core register region       */
#define SOCSRAM_BASE_ADDRESS     0x18004000    /* SOCSRAM core register region      */
#define BACKPLANE_ADDRESS_MASK   0x7FFF

#define CHIP_STA_INTERFACE   0
#define CHIP_AP_INTERFACE    1
#define CHIP_P2P_INTERFACE   2

/* Maximum value of bus data credit difference */
#define CHIP_MAX_BUS_DATA_CREDIT_DIFF    7

/* Chipcommon registers */
#define CHIPCOMMON_GPIO_CONTROL ((uint32_t) (CHIPCOMMON_BASE_ADDRESS + 0x6C) )

/******************************************************
 *             SDIO Constants
 ******************************************************/
/* CurrentSdiodProgGuide r23 */

/* Base registers */
#define SDIO_CORE                    ((uint32_t) (SDIO_BASE_ADDRESS + 0x00) )
#define SDIO_INT_STATUS              ((uint32_t) (SDIO_BASE_ADDRESS + 0x20) )
#define SDIO_TO_SB_MAILBOX           ((uint32_t) (SDIO_BASE_ADDRESS + 0x40) )
#define SDIO_TO_SB_MAILBOX_DATA      ((uint32_t) (SDIO_BASE_ADDRESS + 0x48) )
#define SDIO_TO_HOST_MAILBOX_DATA    ((uint32_t) (SDIO_BASE_ADDRESS + 0x4C) )
#define SDIO_TO_SB_MAIL_BOX          ((uint32_t) (SDIO_BASE_ADDRESS + 0x40) )
#define SDIO_INT_HOST_MASK           ((uint32_t) (SDIO_BASE_ADDRESS + 0x24) )
#define SDIO_FUNCTION_INT_MASK       ((uint32_t) (SDIO_BASE_ADDRESS + 0x34) )

/* SDIO Function 0 (SDIO Bus) register addresses */

/* SDIO Device CCCR offsets */
/* TODO: What does CIS/CCCR stand for? */
/* CCCR accesses do not require backpane clock */
#define SDIOD_CCCR_REV             ( (uint32_t)  0x00 )    /* CCCR/SDIO Revision */
#define SDIOD_CCCR_SDREV           ( (uint32_t)  0x01 )    /* SD Revision */
#define SDIOD_CCCR_IOEN            ( (uint32_t)  0x02 )    /* I/O Enable */
#define SDIOD_CCCR_IORDY           ( (uint32_t)  0x03 )    /* I/O Ready */
#define SDIOD_CCCR_INTEN           ( (uint32_t)  0x04 )    /* Interrupt Enable */
#define SDIOD_CCCR_INTPEND         ( (uint32_t)  0x05 )    /* Interrupt Pending */
#define SDIOD_CCCR_IOABORT         ( (uint32_t)  0x06 )    /* I/O Abort */
#define SDIOD_CCCR_BICTRL          ( (uint32_t)  0x07 )    /* Bus Interface control */
#define SDIOD_CCCR_CAPABLITIES     ( (uint32_t)  0x08 )    /* Card Capabilities */
#define SDIOD_CCCR_CISPTR_0        ( (uint32_t)  0x09 )    /* Common CIS Base Address Pointer Register 0 (LSB) */
#define SDIOD_CCCR_CISPTR_1        ( (uint32_t)  0x0A )    /* Common CIS Base Address Pointer Register 1 */
#define SDIOD_CCCR_CISPTR_2        ( (uint32_t)  0x0B )    /* Common CIS Base Address Pointer Register 2 (MSB - only bit 1 valid)*/
#define SDIOD_CCCR_BUSSUSP         ( (uint32_t)  0x0C )    /*  */
#define SDIOD_CCCR_FUNCSEL         ( (uint32_t)  0x0D )    /*  */
#define SDIOD_CCCR_EXECFLAGS       ( (uint32_t)  0x0E )    /*  */
#define SDIOD_CCCR_RDYFLAGS        ( (uint32_t)  0x0F )    /*  */
#define SDIOD_CCCR_BLKSIZE_0       ( (uint32_t)  0x10 )    /* Function 0 (Bus) SDIO Block Size Register 0 (LSB) */
#define SDIOD_CCCR_BLKSIZE_1       ( (uint32_t)  0x11 )    /* Function 0 (Bus) SDIO Block Size Register 1 (MSB) */
#define SDIOD_CCCR_POWER_CONTROL   ( (uint32_t)  0x12 )    /* Power Control */
#define SDIOD_CCCR_SPEED_CONTROL   ( (uint32_t)  0x13 )    /* Bus Speed Select  (control device entry into high-speed clocking mode)  */
#define SDIOD_CCCR_UHS_I           ( (uint32_t)  0x14 )    /* UHS-I Support */
#define SDIOD_CCCR_DRIVE           ( (uint32_t)  0x15 )    /* Drive Strength */
#define SDIOD_CCCR_INTEXT          ( (uint32_t)  0x16 )    /* Interrupt Extension */
#define SDIOD_SEP_INT_CTL          ( (uint32_t)  0xF2 )    /* Separate Interrupt Control*/
#define SDIOD_CCCR_F1INFO          ( (uint32_t) 0x100 )    /* Function 1 (Backplane) Info */
#define SDIOD_CCCR_F1HP            ( (uint32_t) 0x102 )    /* Function 1 (Backplane) High Power */
#define SDIOD_CCCR_F1CISPTR_0      ( (uint32_t) 0x109 )    /* Function 1 (Backplane) CIS Base Address Pointer Register 0 (LSB) */
#define SDIOD_CCCR_F1CISPTR_1      ( (uint32_t) 0x10A )    /* Function 1 (Backplane) CIS Base Address Pointer Register 1       */
#define SDIOD_CCCR_F1CISPTR_2      ( (uint32_t) 0x10B )    /* Function 1 (Backplane) CIS Base Address Pointer Register 2 (MSB - only bit 1 valid) */
#define SDIOD_CCCR_F1BLKSIZE_0     ( (uint32_t) 0x110 )    /* Function 1 (Backplane) SDIO Block Size Register 0 (LSB) */
#define SDIOD_CCCR_F1BLKSIZE_1     ( (uint32_t) 0x111 )    /* Function 1 (Backplane) SDIO Block Size Register 1 (MSB) */
#define SDIOD_CCCR_F2INFO          ( (uint32_t) 0x200 )    /* Function 2 (WLAN Data FIFO) Info */
#define SDIOD_CCCR_F2HP            ( (uint32_t) 0x202 )    /* Function 2 (WLAN Data FIFO) High Power */
#define SDIOD_CCCR_F2CISPTR_0      ( (uint32_t) 0x209 )    /* Function 2 (WLAN Data FIFO) CIS Base Address Pointer Register 0 (LSB) */
#define SDIOD_CCCR_F2CISPTR_1      ( (uint32_t) 0x20A )    /* Function 2 (WLAN Data FIFO) CIS Base Address Pointer Register 1       */
#define SDIOD_CCCR_F2CISPTR_2      ( (uint32_t) 0x20B )    /* Function 2 (WLAN Data FIFO) CIS Base Address Pointer Register 2 (MSB - only bit 1 valid) */
#define SDIOD_CCCR_F2BLKSIZE_0     ( (uint32_t) 0x210 )    /* Function 2 (WLAN Data FIFO) SDIO Block Size Register 0 (LSB) */
#define SDIOD_CCCR_F2BLKSIZE_1     ( (uint32_t) 0x211 )    /* Function 2 (WLAN Data FIFO) SDIO Block Size Register 1 (MSB) */
#define SDIOD_CCCR_F3INFO          ( (uint32_t) 0x300 )    /* Function 3 (Bluetooth Data FIFO) Info */
#define SDIOD_CCCR_F3HP            ( (uint32_t) 0x302 )    /* Function 3 (Bluetooth Data FIFO) High Power */
#define SDIOD_CCCR_F3CISPTR_0      ( (uint32_t) 0x309 )    /* Function 3 (Bluetooth Data FIFO) CIS Base Address Pointer Register 0 (LSB) */
#define SDIOD_CCCR_F3CISPTR_1      ( (uint32_t) 0x30A )    /* Function 3 (Bluetooth Data FIFO) CIS Base Address Pointer Register 1       */
#define SDIOD_CCCR_F3CISPTR_2      ( (uint32_t) 0x30B )    /* Function 3 (Bluetooth Data FIFO) CIS Base Address Pointer Register 2 (MSB - only bit 1 valid) */
#define SDIOD_CCCR_F3BLKSIZE_0     ( (uint32_t) 0x310 )    /* Function 3 (Bluetooth Data FIFO) SDIO Block Size Register 0 (LSB) */
#define SDIOD_CCCR_F3BLKSIZE_1     ( (uint32_t) 0x311 )    /* Function 3 (Bluetooth Data FIFO) SDIO Block Size Register 1 (MSB) */


/* SDIO Function 1 (Backplane) register addresses */
/* Addresses 0x00000000 - 0x0000FFFF are directly access the backplane
 * throught the backplane window. Addresses above 0x0000FFFF are
 * registers relating to backplane access, and do not require a backpane
 * clock to access them
 */
#define SDIO_GPIO_SELECT              ( (uint32_t) 0x10005 )
#define SDIO_GPIO_OUTPUT              ( (uint32_t) 0x10006 )
#define SDIO_GPIO_ENABLE              ( (uint32_t) 0x10007 )
#define SDIO_FUNCTION2_WATERMARK      ( (uint32_t) 0x10008 )
#define SDIO_DEVICE_CONTROL           ( (uint32_t) 0x10009 )
#define SDIO_BACKPLANE_ADDRESS_LOW    ( (uint32_t) 0x1000A )
#define SDIO_BACKPLANE_ADDRESS_MID    ( (uint32_t) 0x1000B )
#define SDIO_BACKPLANE_ADDRESS_HIGH   ( (uint32_t) 0x1000C )
#define SDIO_FRAME_CONTROL            ( (uint32_t) 0x1000D )
#define SDIO_CHIP_CLOCK_CSR           ( (uint32_t) 0x1000E )
#define SDIO_PULL_UP                  ( (uint32_t) 0x1000F )
#define SDIO_READ_FRAME_BC_LOW        ( (uint32_t) 0x1001B )
#define SDIO_READ_FRAME_BC_HIGH       ( (uint32_t) 0x1001C )

#define I_HMB_SW_MASK                 ( (uint32_t) 0x000000F0 )
#define I_HMB_FRAME_IND               ( 1<<6 )
#define FRAME_AVAILABLE_MASK          I_HMB_SW_MASK


/******************************************************
 *             SPI Constants
 ******************************************************/
/* GSPI v1 */
#define SPI_FRAME_CONTROL              ( (uint32_t) 0x1000D )

/* Register addresses */
#define SPI_BUS_CONTROL                ( (uint32_t) 0x0000 )
#define SPI_RESPONSE_DELAY             ( (uint32_t) 0x0001 )
#define SPI_STATUS_ENABLE              ( (uint32_t) 0x0002 )
#define SPI_RESET_BP                   ( (uint32_t) 0x0003 )    /* (corerev >= 1) */
#define SPI_INTERRUPT_REGISTER         ( (uint32_t) 0x0004 )    /* 16 bits - Interrupt status */
#define SPI_INTERRUPT_ENABLE_REGISTER  ( (uint32_t) 0x0006 )    /* 16 bits - Interrupt mask */
#define SPI_STATUS_REGISTER            ( (uint32_t) 0x0008 )    /* 32 bits */
#define SPI_FUNCTION1_INFO             ( (uint32_t) 0x000C )    /* 16 bits */
#define SPI_FUNCTION2_INFO             ( (uint32_t) 0x000E )    /* 16 bits */
#define SPI_FUNCTION3_INFO             ( (uint32_t) 0x0010 )    /* 16 bits */
#define SPI_READ_TEST_REGISTER         ( (uint32_t) 0x0014 )    /* 32 bits */
#define SPI_RESP_DELAY_F0              ( (uint32_t) 0x001c )    /* 8 bits (corerev >= 3) */
#define SPI_RESP_DELAY_F1              ( (uint32_t) 0x001d )    /* 8 bits (corerev >= 3) */
#define SPI_RESP_DELAY_F2              ( (uint32_t) 0x001e )    /* 8 bits (corerev >= 3) */
#define SPI_RESP_DELAY_F3              ( (uint32_t) 0x001f )    /* 8 bits (corerev >= 3) */

/******************************************************
 *             Bit Masks
 ******************************************************/

/* SDIOD_CCCR_REV Bits */
#define SDIO_REV_SDIOID_MASK       ( (uint32_t)  0xF0 )    /* SDIO spec revision number */
#define SDIO_REV_CCCRID_MASK       ( (uint32_t)  0x0F )    /* CCCR format version number */

/* SDIOD_CCCR_SDREV Bits */
#define SD_REV_PHY_MASK            ( (uint32_t)  0x0F )    /* SD format version number */

/* SDIOD_CCCR_IOEN Bits */
#define SDIO_FUNC_ENABLE_1         ( (uint32_t)  0x02 )    /* function 1 I/O enable */
#define SDIO_FUNC_ENABLE_2         ( (uint32_t)  0x04 )    /* function 2 I/O enable */
#define SDIO_FUNC_ENABLE_3         ( (uint32_t)  0x08 )    /* function 3 I/O enable */

/* SDIOD_CCCR_IORDY Bits */
#define SDIO_FUNC_READY_1          ( (uint32_t)  0x02 )    /* function 1 I/O ready */
#define SDIO_FUNC_READY_2          ( (uint32_t)  0x04 )    /* function 2 I/O ready */
#define SDIO_FUNC_READY_3          ( (uint32_t)  0x08 )    /* function 3 I/O ready */

/* SDIOD_CCCR_INTEN Bits */
#define INTR_CTL_MASTER_EN         ( (uint32_t)  0x01 )    /* interrupt enable master */
#define INTR_CTL_FUNC1_EN          ( (uint32_t)  0x02 )    /* interrupt enable for function 1 */
#define INTR_CTL_FUNC2_EN          ( (uint32_t)  0x04 )    /* interrupt enable for function 2 */

/* SDIOD_SEP_INT_CTL Bits */
#define SEP_INTR_CTL_MASK          ( (uint32_t)  0x01 )    /* out-of-band interrupt mask */
#define SEP_INTR_CTL_EN            ( (uint32_t)  0x02 )    /* out-of-band interrupt output enable */
#define SEP_INTR_CTL_POL           ( (uint32_t)  0x04 )    /* out-of-band interrupt polarity */

/* SDIOD_CCCR_INTPEND Bits */
#define INTR_STATUS_FUNC1          ( (uint32_t)  0x02 )    /* interrupt pending for function 1 */
#define INTR_STATUS_FUNC2          ( (uint32_t)  0x04 )    /* interrupt pending for function 2 */
#define INTR_STATUS_FUNC3          ( (uint32_t)  0x08 )    /* interrupt pending for function 3 */

/* SDIOD_CCCR_IOABORT Bits */
#define IO_ABORT_RESET_ALL         ( (uint32_t)  0x08 )    /* I/O card reset */
#define IO_ABORT_FUNC_MASK         ( (uint32_t)  0x07 )    /* abort selction: function x */

/* SDIOD_CCCR_BICTRL Bits */
#define BUS_CARD_DETECT_DIS        ( (uint32_t)  0x80 )    /* Card Detect disable */
#define BUS_SPI_CONT_INTR_CAP      ( (uint32_t)  0x40 )    /* support continuous SPI interrupt */
#define BUS_SPI_CONT_INTR_EN       ( (uint32_t)  0x20 )    /* continuous SPI interrupt enable */
#define BUS_SD_DATA_WIDTH_MASK     ( (uint32_t)  0x03 )    /* bus width mask */
#define BUS_SD_DATA_WIDTH_4BIT     ( (uint32_t)  0x02 )    /* bus width 4-bit mode */
#define BUS_SD_DATA_WIDTH_1BIT     ( (uint32_t)  0x00 )    /* bus width 1-bit mode */

/* SDIOD_CCCR_CAPABLITIES Bits */
#define SDIO_CAP_4BLS              ( (uint32_t)  0x80 )    /* 4-bit support for low speed card */
#define SDIO_CAP_LSC               ( (uint32_t)  0x40 )    /* low speed card */
#define SDIO_CAP_E4MI              ( (uint32_t)  0x20 )    /* enable interrupt between block of data in 4-bit mode */
#define SDIO_CAP_S4MI              ( (uint32_t)  0x10 )    /* support interrupt between block of data in 4-bit mode */
#define SDIO_CAP_SBS               ( (uint32_t)  0x08 )    /* support suspend/resume */
#define SDIO_CAP_SRW               ( (uint32_t)  0x04 )    /* support read wait */
#define SDIO_CAP_SMB               ( (uint32_t)  0x02 )    /* support multi-block transfer */
#define SDIO_CAP_SDC               ( (uint32_t)  0x01 )    /* Support Direct commands during multi-byte transfer */

/* SDIOD_CCCR_POWER_CONTROL Bits */
#define SDIO_POWER_SMPC            ( (uint32_t)  0x01 )    /* supports master power control (RO) */
#define SDIO_POWER_EMPC            ( (uint32_t)  0x02 )    /* enable master power control (allow > 200mA) (RW) */

/* SDIOD_CCCR_SPEED_CONTROL Bits */
#define SDIO_SPEED_SHS             ( (uint32_t)  0x01 )    /* supports high-speed [clocking] mode (RO) */
#define SDIO_SPEED_EHS             ( (uint32_t)  0x02 )    /* enable high-speed [clocking] mode (RW) */



/* GSPI */
#define SPI_READ_TEST_REGISTER_VALUE   ( (uint32_t) 0xFEEDBEAD )
#define SPI_READ_TEST_REG_LSB          ( ( ( SPI_READ_TEST_REGISTER_VALUE      )     ) & 0xff )
#define SPI_READ_TEST_REG_LSB_SFT1     ( ( ( SPI_READ_TEST_REGISTER_VALUE << 1 )     ) & 0xff )
#define SPI_READ_TEST_REG_LSB_SFT2     ( ( ( SPI_READ_TEST_REGISTER_VALUE << 1 ) + 1 ) & 0xff )
#define SPI_READ_TEST_REG_LSB_SFT3     ( ( ( SPI_READ_TEST_REGISTER_VALUE +1  ) << 1 ) & 0xff )


/* SPI_BUS_CONTROL Bits */
#define WORD_LENGTH_32             ( (uint32_t)  0x01 )    /* 0/1 16/32 bit word length */
#define ENDIAN_BIG                 ( (uint32_t)  0x02 )    /* 0/1 Little/Big Endian */
#define CLOCK_PHASE                ( (uint32_t)  0x04 )    /* 0/1 clock phase delay */
#define CLOCK_POLARITY             ( (uint32_t)  0x08 )    /* 0/1 Idle state clock polarity is low/high */
#define HIGH_SPEED_MODE            ( (uint32_t)  0x10 )    /* 1/0 High Speed mode / Normal mode */
#define INTR_POLARITY_HIGH         ( (uint32_t)  0x20 )    /* 1/0 Interrupt active polarity is high/low */
#define WAKE_UP                    ( (uint32_t)  0x80 )    /* 0/1 Wake-up command from Host to WLAN */

/* SPI_RESPONSE_DELAY Bit mask */
#define RESPONSE_DELAY_MASK 0xFF    /* Configurable rd response delay in multiples of 8 bits */

/* SPI_STATUS_ENABLE Bits */
#define STATUS_ENABLE              ( (uint32_t)  0x01 )    /* 1/0 Status sent/not sent to host after read/write */
#define INTR_WITH_STATUS           ( (uint32_t)  0x02 )    /* 0/1 Do-not / do-interrupt if status is sent */
#define RESP_DELAY_ALL             ( (uint32_t)  0x04 )    /* Applicability of resp delay to F1 or all func's read */
#define DWORD_PKT_LEN_EN           ( (uint32_t)  0x08 )    /* Packet len denoted in dwords instead of bytes */
#define CMD_ERR_CHK_EN             ( (uint32_t)  0x20 )    /* Command error check enable */
#define DATA_ERR_CHK_EN            ( (uint32_t)  0x40 )    /* Data error check enable */



/* SPI_RESET_BP Bits*/
#define RESET_ON_WLAN_BP_RESET     ( (uint32_t)  0x04 )    /* enable reset for WLAN backplane */
#define RESET_ON_BT_BP_RESET       ( (uint32_t)  0x08 )    /* enable reset for BT backplane */
#define RESET_SPI                  ( (uint32_t)  0x80 )    /* reset the above enabled logic */



/* SPI_INTERRUPT_REGISTER and SPI_INTERRUPT_ENABLE_REGISTER Bits */
#define DATA_UNAVAILABLE           ( (uint32_t) 0x0001 )   /* Requested data not available; Clear by writing a "1" */
#define F2_F3_FIFO_RD_UNDERFLOW    ( (uint32_t) 0x0002 )
#define F2_F3_FIFO_WR_OVERFLOW     ( (uint32_t) 0x0004 )
#define COMMAND_ERROR              ( (uint32_t) 0x0008 )   /* Cleared by writing 1 */
#define DATA_ERROR                 ( (uint32_t) 0x0010 )   /* Cleared by writing 1 */
#define F2_PACKET_AVAILABLE        ( (uint32_t) 0x0020 )
#define F3_PACKET_AVAILABLE        ( (uint32_t) 0x0040 )
#define F1_OVERFLOW                ( (uint32_t) 0x0080 )   /* Due to last write. Bkplane has pending write requests */
#define MISC_INTR0                 ( (uint32_t) 0x0100 )
#define MISC_INTR1                 ( (uint32_t) 0x0200 )
#define MISC_INTR2                 ( (uint32_t) 0x0400 )
#define MISC_INTR3                 ( (uint32_t) 0x0800 )
#define MISC_INTR4                 ( (uint32_t) 0x1000 )
#define F1_INTR                    ( (uint32_t) 0x2000 )
#define F2_INTR                    ( (uint32_t) 0x4000 )
#define F3_INTR                    ( (uint32_t) 0x8000 )




/* SPI_STATUS_REGISTER Bits */
#define STATUS_DATA_NOT_AVAILABLE  ( (uint32_t) 0x00000001 )
#define STATUS_UNDERFLOW           ( (uint32_t) 0x00000002 )
#define STATUS_OVERFLOW            ( (uint32_t) 0x00000004 )
#define STATUS_F2_INTR             ( (uint32_t) 0x00000008 )
#define STATUS_F3_INTR             ( (uint32_t) 0x00000010 )
#define STATUS_F2_RX_READY         ( (uint32_t) 0x00000020 )
#define STATUS_F3_RX_READY         ( (uint32_t) 0x00000040 )
#define STATUS_HOST_CMD_DATA_ERR   ( (uint32_t) 0x00000080 )
#define STATUS_F2_PKT_AVAILABLE    ( (uint32_t) 0x00000100 )
#define STATUS_F2_PKT_LEN_MASK     ( (uint32_t) 0x000FFE00 )
#define STATUS_F2_PKT_LEN_SHIFT    ( (uint32_t) 9          )
#define STATUS_F3_PKT_AVAILABLE    ( (uint32_t) 0x00100000 )
#define STATUS_F3_PKT_LEN_MASK     ( (uint32_t) 0xFFE00000 )
#define STATUS_F3_PKT_LEN_SHIFT    ( (uint32_t) 21         )





/* SDIO_CHIP_CLOCK_CSR Bits */
#define SBSDIO_FORCE_ALP           ( (uint32_t)  0x01 )    /* Force ALP request to backplane */
#define SBSDIO_FORCE_HT            ( (uint32_t)  0x02 )    /* Force HT request to backplane */
#define SBSDIO_FORCE_ILP           ( (uint32_t)  0x04 )    /* Force ILP request to backplane */
#define SBSDIO_ALP_AVAIL_REQ       ( (uint32_t)  0x08 )    /* Make ALP ready (power up xtal) */
#define SBSDIO_HT_AVAIL_REQ        ( (uint32_t)  0x10 )    /* Make HT ready (power up PLL) */
#define SBSDIO_FORCE_HW_CLKREQ_OFF ( (uint32_t)  0x20 )    /* Squelch clock requests from HW */
#define SBSDIO_ALP_AVAIL           ( (uint32_t)  0x40 )    /* Status: ALP is ready */
#define SBSDIO_HT_AVAIL            ( (uint32_t)  0x80 )    /* Status: HT is ready */
#define SBSDIO_Rev8_HT_AVAIL       ( (uint32_t)  0x40 )
#define SBSDIO_Rev8_ALP_AVAIL      ( (uint32_t)  0x80 )


/* SDIO_FRAME_CONTROL Bits */
#define SFC_RF_TERM                ( (uint32_t) (1 << 0) ) /* Read Frame Terminate */
#define SFC_WF_TERM                ( (uint32_t) (1 << 1) ) /* Write Frame Terminate */
#define SFC_CRC4WOOS               ( (uint32_t) (1 << 2) ) /* HW reports CRC error for write out of sync */
#define SFC_ABORTALL               ( (uint32_t) (1 << 3) ) /* Abort cancels all in-progress frames */

/* SDIO_TO_SB_MAIL_BOX bits corresponding to intstatus bits */
#define SMB_NAK                    ( (uint32_t) (1 << 0) ) /* To SB Mailbox Frame NAK */
#define SMB_INT_ACK                ( (uint32_t) (1 << 1) ) /* To SB Mailbox Host Interrupt ACK */
#define SMB_USE_OOB                ( (uint32_t) (1 << 2) ) /* To SB Mailbox Use OOB Wakeup */
#define SMB_DEV_INT                ( (uint32_t) (1 << 3) ) /* To SB Mailbox Miscellaneous Interrupt */


#define WL_CHANSPEC_BAND_MASK             (0xf000)
#define WL_CHANSPEC_BAND_5G               (0x1000)
#define WL_CHANSPEC_BAND_2G               (0x2000)
#define WL_CHANSPEC_CTL_SB_MASK           (0x0300)
#define WL_CHANSPEC_CTL_SB_LOWER          (0x0100)
#define WL_CHANSPEC_CTL_SB_UPPER          (0x0200)
#define WL_CHANSPEC_CTL_SB_NONE           (0x0300)
#define WL_CHANSPEC_BW_MASK               (0x0C00)
#define WL_CHANSPEC_BW_10                 (0x0400)
#define WL_CHANSPEC_BW_20                 (0x0800)
#define WL_CHANSPEC_BW_40                 (0x0C00)


// /* CIS accesses require backpane clock */
// 
// 
// #define CHIP_HAS_BSSID_CNT_IN_ASSOC_PARAMS
// #define CHIP_FIRMWARE_SUPPORTS_PM_LIMIT_IOVAR
// 
// struct ether_addr;
// struct wl_join_scan_params;
// 
// typedef struct wl_assoc_params
// {
//     struct ether_addr bssid;
// #ifdef CHIP_HAS_BSSID_CNT_IN_ASSOC_PARAMS
//     uint16_t    bssid_cnt;
// #endif /* ifdef CHIP_HAS_BSSID_CNT_IN_ASSOC_PARAMS */
//     uint32_t     chanspec_num;
//     chanspec_t  chanspec_list[1];
// } wl_assoc_params_t;
// #define WL_ASSOC_PARAMS_FIXED_SIZE     (sizeof(wl_assoc_params_t) - sizeof(wl_chanspec_t))
// typedef wl_assoc_params_t wl_reassoc_params_t;
// #define WL_REASSOC_PARAMS_FIXED_SIZE    WL_ASSOC_PARAMS_FIXED_SIZE
// typedef wl_assoc_params_t wl_join_assoc_params_t;
// #define WL_JOIN_ASSOC_PARAMS_FIXED_SIZE        WL_ASSOC_PARAMS_FIXED_SIZE
// typedef struct wl_join_params
// {
//     wlc_ssid_t         ssid;
//     struct wl_assoc_params  params;
// } wl_join_params_t;
// #define WL_JOIN_PARAMS_FIXED_SIZE     (sizeof(wl_join_params_t) - sizeof(wl_chanspec_t))
// 
// /* extended join params */
// typedef struct wl_extjoin_params
// {
//         wlc_ssid_t ssid; /* {0, ""}: wildcard scan */
//         struct wl_join_scan_params scan_params;
//         wl_join_assoc_params_t assoc_params; /* optional field, but it must include the fixed portion of the wl_join_assoc_params_t struct when it does present. */
// } wl_extjoin_params_t;
// #define WL_EXTJOIN_PARAMS_FIXED_SIZE    (sizeof(wl_extjoin_params_t) - sizeof(chanspec_t))
// 
// typedef wl_cnt_ver_six_t  wiced_counters_t;

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* ifndef INCLUDED_CHIP_CONSTANTS_H_ */
