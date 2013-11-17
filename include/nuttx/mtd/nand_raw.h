/****************************************************************************
 * include/nuttx/mtd/nand_raw.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This logic was based largely on Atmel sample code with modifications for
 * better integration with NuttX.  The Atmel sample code has a BSD
 * compatibile license that requires this copyright notice:
 *
 *   Copyright (c) 2012, Atmel Corporation
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
 * 3. Neither the names NuttX nor Atmel nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
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

#ifndef __INCLUDE_NUTTX_MTD_NAND_RAW_H
#define __INCLUDE_NUTTX_MTD_NAND_RAW_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/mtd/mtd.h>
#include <nuttx/mtd/nand_model.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Nand flash commands */

#define COMMAND_READ_1                  0x00
#define COMMAND_READ_2                  0x30
#define COMMAND_COPYBACK_READ_1         0x00
#define COMMAND_COPYBACK_READ_2         0x35
#define COMMAND_COPYBACK_PROGRAM_1      0x85
#define COMMAND_COPYBACK_PROGRAM_2      0x10
#define COMMAND_RANDOM_OUT              0x05
#define COMMAND_RANDOM_OUT_2            0xe0
#define COMMAND_RANDOM_IN               0x85
#define COMMAND_READID                  0x90
#define COMMAND_WRITE_1                 0x80
#define COMMAND_WRITE_2                 0x10
#define COMMAND_ERASE_1                 0x60
#define COMMAND_ERASE_2                 0xd0
#define COMMAND_STATUS                  0x70
#define COMMAND_RESET                   0xff

/* Nand flash commands (small blocks) */

#define COMMAND_READ_A                  0x00
#define COMMAND_READ_C                  0x50

/* NAND access macros */

#define WRITE_COMMAND8(raw, command) \
    {*((volatile uint8_t *)raw->cmdaddr) = (uint8_t)command;}
#define WRITE_COMMAND16(raw, command) \
    {*((volatile uint16_t *)raw->cmdaddr) = (uint16_t)command;}
#define WRITE_ADDRESS8(raw, address) \
    {*((volatile uint8_t *)raw->addraddr) = (uint8_t)address;}
#define WRITE_ADDRESS16(raw, address) \
    {*((volatile uint16_t *)raw->addraddr) = (uint16_t)address;}
#define WRITE_DATA8(raw, data) \
    {*((volatile uint8_t *)raw->dataaddr) = (uint8_t)data;}
#define READ_DATA8(raw) \
    (*((volatile uint8_t *)raw->dataaddr))
#define WRITE_DATA16(raw, data) \
    {*((volatile uint16_t *) raw->dataaddr) = (uint16_t)data;}
#define READ_DATA16(raw) \
    (*((volatile uint16_t *)raw->dataaddr))

/****************************************************************************
 * Public Types
 ****************************************************************************/
/* This type represents the visible portion of the lower-half, raw NAND MTD
 * device.  Rules:
 *
 * 1. The struct mtd_dev_s must appear at the beginning of the definition so
 *    that you can freely cast between pointers to struct mtd_dev_s and struct
 *    nand_raw_s.
 * 2. The lower-half driver may freely append additional information after
 *    this required header information.
 */

struct nand_raw_s
{
  struct mtd_dev_s mtd;      /* Externally visible part of the driver */
  struct nand_model_s model; /* The NAND model storage */
  uintptr_t cmdaddr;         /* NAND command address base */
  uintptr_t addraddr;        /* NAND address address base */
  uintptr_t dataaddr;        /* NAND data address */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nand_chipid
 *
 * Description:
 *   Reads and returns the identifiers of a NAND FLASH chip
 *
 * Input Parameters:
 *   raw - Pointer to a struct nand_raw_s instance.
 *
 * Returned Value:
 *   id1|(id2<<8)|(id3<<16)|(id4<<24)
 *
 ****************************************************************************/

uint32_t nand_chipid(FAR struct nand_raw_s *raw);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __INCLUDE_NUTTX_MTD_NAND_RAW_H */
