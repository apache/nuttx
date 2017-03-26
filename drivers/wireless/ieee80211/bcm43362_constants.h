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

#ifndef BCM43362_CONSTANTS_H_
#define BCM43362_CONSTANTS_H_

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
 *             Bit Masks
 ******************************************************/

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

#endif /* ifndef BCM43362_CONSTANTS_H_ */
