/****************************************************************************
 * drivers/mmcsd/mmcsd_spi.h
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

#ifndef __DRIVERS_MMCSD_MMCSD_SPI_H
#define __DRIVERS_MMCSD_MMCSD_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* SPI *******************************************************************/
/* SPI 8-bit R1 response */

#define MMCSD_SPIR1_OK            0x00 /* No error bits set */
#define MMCSD_SPIR1_IDLESTATE     0x01 /* Idle state */
#define MMCSD_SPIR1_ERASERESET    0x02 /* Erase reset */
#define MMCSD_SPIR1_ILLEGALCMD    0x04 /* Illegal command */
#define MMCSD_SPIR1_CRCERROR      0x08 /* Com CRC error */
#define MMCSD_SPIR1_ERASEERROR    0x10 /* Erase sequence error */
#define MMCSD_SPIR1_ADDRERROR     0x20 /* Address error */
#define MMCSD_SPIR1_PARAMERROR    0x40 /* Parameter error */

/* SPI 8-bit R2 response */

#define MMCSD_SPIR2_CARDLOCKED    0x0001 /* Card is locked */
#define MMCSD_SPIR2_WPERASESKIP   0x0002 /* WP erase skip */
#define MMCSD_SPIR2_LOCKFAIL      0x0002 /* Lock/unlock cmd failed */
#define MMCSD_SPIR2_ERROR         0x0004 /* Error */
#define MMCSD_SPIR2_CCERROR       0x0008 /* CC error */
#define MMCSD_SPIR2_CARDECCFAIL   0x0010 /* Card ECC failed */
#define MMCSD_SPIR2_WPVIOLATION   0x0020 /* WP violoation */
#define MMCSD_SPIR2_ERASEPARAM    0x0040 /* Erase parameter */
#define MMCSD_SPIR2_OUTOFRANGE    0x0080 /* Out of range */
#define MMCSD_SPIR2_CSDOVERWRITE  0x0080 /* CSD overwrite */
#define MMCSD_SPIR2_IDLESTATE     0x0100 /* In idle state */
#define MMCSD_SPIR2_ERASERESET    0x0200 /* Erase reset */
#define MMCSD_SPIR2_ILLEGALCMD    0x0400 /* Illegal command */
#define MMCSD_SPIR2_CRCERROR      0x0800 /* Com CRC error */
#define MMCSD_SPIR2_ERASEERROR    0x1000 /* Erase sequence error */
#define MMCSD_SPIR2_ADDRERROR     0x2000 /* Address error */
#define MMCSD_SPIR2_PARAMERROR    0x4000 /* Parameter error */

/* Last 4 bytes of the 5 byte R7 response */

#define MMCSD_SPIR7_VERSION_SHIFT 28     /* Bits 28-31: Command version number */
#define MMCSD_SPIR7_VERSION_MASK  (0x0f << MMCSD_SPIR7_VERSION_SHIFT)
#define MMCSD_SPIR7_VOLTAGE_SHIFT 8      /* Bits 8-11: Voltage accepted */
#define MMCSD_SPIR7_VOLTAGE_MASK  (0x0f << MMCSD_SPIR7_VOLTAGE_SHIFT)
#define MMCSD_SPIR7_ECHO_SHIFT    0      /* Bits 0-7: Echoed check pattern */
#define MMCSD_SPIR7_ECHO_MASK     (0x0f << MMCSD_SPIR7_ECHO_SHIFT)

/* Data Response */

#define MMCSD_SPIDR_MASK          0x1f   /* Mask for valid data response bits */
#define MMCSD_SPIDR_ACCEPTED      0x05   /* Data accepted */
#define MMCSD_SPIDR_CRCERROR      0x0b   /* Data rejected due to CRC error */
#define MMCSD_SPIDR_WRERROR       0x0d   /* Data rejected due to write error */

/* Data Tokens */

#define MMCSD_SPIDT_STARTBLKSNGL  0xfe   /* First byte of block, single block */
#define MMCSD_SPIDT_STARTBLKMULTI 0xfc   /* First byte of block, multi-block */
#define MMCSD_SPIDT_STOPTRANS     0xfd   /* Stop transmission */

/* Data error token */

#define MMCSD_SPIDET_UPPER        0xf0   /* The upper four bits are zero */
#define MMCSD_SPIDET_ERROR        0x01   /* Error */
#define MMCSD_SPIDET_CCERROR      0x02   /* CC error */
#define MMCSD_SPIDET_CARDECCFAIL  0x04   /* Card ECC failed */
#define MMCSD_SPIDET_OUTOFRANGE   0x08   /* Out of range */

/* Operating Conditions register */

#define MMCSD_OCR_V27             (1 << 15) /* Bit 15: 2.7-2.8V */
#define MMCSD_OCR_V28             (1 << 16) /* Bit 16: 2.8-2.9V */
#define MMCSD_OCR_V29             (1 << 17) /* Bit 17: 2.9-3.0V */
#define MMCSD_OCR_V30             (1 << 18) /* Bit 18: 3.0-3.1V */
#define MMCSD_OCR_V31             (1 << 19) /* Bit 19: 3.1-3.2V */
#define MMCSD_OCR_V32             (1 << 20) /* Bit 20: 3.2-3.3V */
#define MMCSD_OCR_V33             (1 << 21) /* Bit 21: 3.3-3.4V */
#define MMCSD_OCR_V34             (1 << 22) /* Bit 22: 3.4-3.5V */
#define MMCSD_OCR_V35             (1 << 23) /* Bit 23: 3.5-3.6V */
#define MMCSD_OCR_CCS             (1 << 30) /* Bit 30: Card capacity status */
#define MMCSD_OCR_BUSY            (1 << 31) /* Bit 31: Card powered up status bit */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __DRIVERS_MMCSD_MMCSD_SPI_H */
