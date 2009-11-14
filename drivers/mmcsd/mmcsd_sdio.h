/********************************************************************************************
 * drivers/mmcsd/mmcsd_sdio.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 ********************************************************************************************/

#ifndef __DRIVERS_MMCSD_MMCSD_SDIO_H
#define __DRIVERS_MMCSD_MMCSD_SDIO_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

/********************************************************************************************
 * Pre-Processor Definitions
 ********************************************************************************************/

/* CMD8 Argument:
 *    [31:12]: Reserved (shall be set to '0') *    [11:8]: Supply Voltage (VHS) 0x1 (Range: 2.7-3.6 V)
 *    [7:0]: Check Pattern (recommended 0xaa)
 * CMD8 Response: R7
 */

#define MMCSD_CMD8VOLTAGE_SHIFT     8          /* Bits 8-11: Supply voltage */
#define MMCSD_CMD8VOLTAGE_MASK      (0x0f << MMCSD_CMD8VOLTAGE_SHIFT)
#  define MMCSD_CMD8VOLTAGE_27      (0x01 << MMCSD_CMD8VOLTAGE_SHIFT) /* 2.7-3.6V */
#define MMCSD_CMD8ECHO_SHIFT        0          /* Bits 0-7: Check pattern */
#define MMCSD_CMD8ECHO_MASK         (0xff << MMCSD_CMD8ECHO_SHIFT)
#  define MMCSD_CMD8CHECKPATTERN    (0xaa << MMCSD_CMD8ECHO_SHIFT)

/* ACMD41 argument */

#define MMCD_ACMD41_VOLTAGEWINDOW   0x80100000
#define MMCD_ACMD41_HIGHCAPACITY    (1 << 30)
#define MMCD_ACMD41_STDCAPACITY     (0)

/* R1 Card Status bit definitions */

#define MMCSD_R1_OUTOFRANGE         (1 << 31)  /* Bad argument */
#define MMCSD_R1_ADDRESSERROR       (1 << 30)  /* Bad address */
#define MMCSD_R1_BLOCKLENERROR      (1 << 29)  /* Bad block length */
#define MMCSD_R1_ERASESEQERROR      (1 << 28)  /* Erase cmd error */
#define MMCSD_R1_ERASEPARAM         (1 << 27)  /* Bad write blocks */
#define MMCSD_R1_WPVIOLATION        (1 << 26)  /* Erase access failure */
#define MMCSD_R1_CARDISLOCKED       (1 << 25)  /* Card is locked */
#define MMCSD_R1_LOCKUNLOCKFAILED   (1 << 24)  /* Password error */
#define MMCSD_R1_COMCRCERROR        (1 << 23)  /* CRC error */
#define MMCSD_R1_ILLEGALCOMMAND     (1 << 22)  /* Bad command */
#define MMCSD_R1_CARDECCFAILED      (1 << 21)  /* Failed to correct data */
#define MMCSD_R1_CCERROR            (1 << 20)  /* Card controller error */
#define MMCSD_R1_ERROR              (1 << 19)  /* General error */
#define MMCSD_R1_UNDERRUN           (1 << 18)  /* Underrun (MMC only) */
#define MMCSD_R1_OVERRRUN           (1 << 17)  /* Overrun (MMC only) */
#define MMCSD_R1_CIDCSDOVERWRITE    (1 << 16)  /* CID/CSD error */
#define MMCSD_R1_WPERASESKIP        (1 << 15)  /* Not all erased */
#define MMCSD_R1_CARDECCDISABLED    (1 << 14)  /* Internal ECC not used */
#define MMCSD_R1_ERASERESET         (1 << 13)  /* Reset sequence cleared */
#define MMCSD_R1_STATE_SHIFT        (9)        /* Current card state */
#define MMCSD_R1_STATE_MASK         (15 << MMCSD_R1_STATE_SHIFT)
                                                                /* Card identification mode states */
#  define MMCSD_R1_STATE_IDLE       (0 << MMCSD_R1_STATE_SHIFT) /* 0=Idle state */
#  define MMCSD_R1_STATE_READY      (1 << MMCSD_R1_STATE_SHIFT) /* 1=Ready state */
#  define MMCSD_R1_STATE_IDENT      (2 << MMCSD_R1_STATE_SHIFT) /* 2=Identification state */
                                                                /* Data transfer states */
#  define MMCSD_R1_STATE_STBY       (3 << MMCSD_R1_STATE_SHIFT) /* 3=Standby state */
#  define MMCSD_R1_STATE_TRAN       (4 << MMCSD_R1_STATE_SHIFT) /* 4=Transfer state */
#  define MMCSD_R1_STATE_DATA       (5 << MMCSD_R1_STATE_SHIFT) /* 5=Sending data state */
#  define MMCSD_R1_STATE_RCV        (6 << MMCSD_R1_STATE_SHIFT) /* 6=Receiving data state */
#  define MMCSD_R1_STATE_PRG        (7 << MMCSD_R1_STATE_SHIFT) /* 7=Programming state */
#  define MMCSD_R1_STATE_DIS        (8 << MMCSD_R1_STATE_SHIFT) /* 8=Disconnect state */
#define MMCSD_R1_READYFORDATA       (1 << 8)   /* Buffer empty */
#define MMCSD_R1_APPCMD             (1 << 5)   /* Next CMD is ACMD */
#define MMCSD_R1_AKESEQERROR        (1 << 3)   /* Authentication error */
#define MMCSD_R1_ERRORMASK          0xfdffe008 /* Error mask */

#define IS_STATE(v,s)               (((v)&MMCSD_R1_CURRENTSTATE_MASK)==(s))

/* R3 (OCR) */

#define MMC_VDD_20_36               0x00ffff00 /* VDD voltage 2.0-3.6 */

#define MMCSD_VDD_145_150           (1 << 0)   /* VDD voltage 1.45 - 1.50 */
#define MMCSD_VDD_150_155           (1 << 1)   /* VDD voltage 1.50 - 1.55 */
#define MMCSD_VDD_155_160           (1 << 2)   /* VDD voltage 1.55 - 1.60 */
#define MMCSD_VDD_160_165           (1 << 3)   /* VDD voltage 1.60 - 1.65 */
#define MMCSD_VDD_165_170           (1 << 4)   /* VDD voltage 1.65 - 1.70 */
#define MMCSD_VDD_17_18             (1 << 5)   /* VDD voltage 1.7 - 1.8 */
#define MMCSD_VDD_18_19             (1 << 6)   /* VDD voltage 1.8 - 1.9 */
#define MMCSD_VDD_19_20             (1 << 7)   /* VDD voltage 1.9 - 2.0 */
#define MMCSD_VDD_20_21             (1 << 8)   /* VDD voltage 2.0-2.1 */
#define MMCSD_VDD_21_22             (1 << 9)   /* VDD voltage 2.1-2.2 */
#define MMCSD_VDD_22_23             (1 << 10)  /* VDD voltage 2.2-2.3 */
#define MMCSD_VDD_23_24             (1 << 11)  /* VDD voltage 2.3-2.4 */
#define MMCSD_VDD_24_25             (1 << 12)  /* VDD voltage 2.4-2.5 */
#define MMCSD_VDD_25_26             (1 << 13)  /* VDD voltage 2.5-2.6 */
#define MMCSD_VDD_26_27             (1 << 14)  /* VDD voltage 2.6-2.7 */
#define MMCSD_VDD_27_28             (1 << 15)  /* VDD voltage 2.7-2.8 */
#define MMCSD_VDD_28_29             (1 << 16)  /* VDD voltage 2.8-2.9 */
#define MMCSD_VDD_29_30             (1 << 17)  /* VDD voltage 2.9-3.0 */
#define MMCSD_VDD_30_31             (1 << 18)  /* VDD voltage 3.0-3.1 */
#define MMCSD_VDD_31_32             (1 << 19)  /* VDD voltage 3.1-3.2 */
#define MMCSD_VDD_32_33             (1 << 20)  /* VDD voltage 3.2-3.3 */
#define MMCSD_VDD_33_34             (1 << 21)  /* VDD voltage 3.3-3.4 */
#define MMCSD_VDD_34_35             (1 << 22)  /* VDD voltage 3.4-3.5 */
#define MMCSD_VDD_35_36             (1 << 23)  /* VDD voltage 3.5-3.6 */
#define MMCD_R3_HIGHCAPACITY        (1 << 30)  /* TRUE: Card supports block addressing */
#define MMCSD_CARD_BUSY             (1 << 31)  /* Card power-up busy bit */

/* Last 4 bytes of the 48-bit R7 response */

#define MMCSD_R7VERSION_SHIFT       28         /* Bits 28-31: Command version number */
#define MMCSD_R7VERSION_MASK        (0x0f << MMCSD_R7VERSION_SHIFT)
#define MMCSD_R7VOLTAGE_SHIFT       8          /* Bits 8-11: Voltage accepted */
#define MMCSD_R7VOLTAGE_MASK        (0x0f << MMCSD_R7VOLTAGE_SHIFT)
#  define MMCSD_R7VOLTAGE_27        (0x01 << MMCSD_R7VOLTAGE_SHIFT) /* 2.7-3.6V */
#define MMCSD_R7ECHO_SHIFT          0          /* Bits 0-7: Echoed check pattern */
#define MMCSD_R7ECHO_MASK           (0xff << MMCSD_R7ECHO_SHIFT)
#  define MMCSD_R7CHECKPATTERN      (0xaa << MMCSD_R7ECHO_SHIFT)

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/* Decoded CID register */

struct mmcsd_cid_s
{
  ubyte  mid;       /* 127:120  8-bit Manufacturer ID */
  uint16 oid;       /* 119:104 16-bit OEM/Application ID (ascii) */
  ubyte  pnm[6];    /* 103:64  40-bit Product Name (ascii) + null terminator */
  ubyte  prv;       /*  63:56   8-bit Product revision */
  uint32 psn;       /*  55:24  32-bit Product serial number */
                    /*  23:20   4-bit (reserved) */
  uint16 mdt;       /*  19:8   12-bit Manufacturing date */
  ubyte  crc;       /*   7:1    7-bit CRC7 */
                    /*   0:0    1-bit (not used) */
};

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/


#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __DRIVERS_MMCSD_MMCSD_SDIO_H */
