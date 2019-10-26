/****************************************************************************
 * include/nuttx/drivers/ramdisk.h
 *
 *   Copyright (C) 2008-2009, 2012-2013, 2015-2016 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_DRIVERS_RAMDISK_H
#define __INCLUDE_NUTTX_DRIVERS_RAMDISK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Values for rdflags */

#define RDFLAG_WRENABLED       (1 << 0) /* Bit 0: 1=Can write to RAM disk */
#define RDFLAG_FUNLINK         (1 << 1) /* Bit 1: 1=Free memory when unlinked */

/* For internal use by the driver only */

#define RDFLAG_UNLINKED        (1 << 2) /* Bit 2: 1=Driver has been unlinked */

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: ramdisk_register or romdisk_register
 *
 * Description:
 *   Non-standard function to register a ramdisk or a romdisk
 *
 * Input Parameters:
 *   minor:         Selects suffix of device named /dev/ramN, N={1,2,3...}
 *   nsectors:      Number of sectors on device
 *   sectize:       The size of one sector
 *   rdflags:       See RDFLAG_* definitions
 *   buffer:        RAM disk backup memory
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_FS_WRITABLE
int ramdisk_register(int minor, FAR uint8_t *buffer, uint32_t nsectors,
                     uint16_t sectsize, uint8_t rdflags);
#define romdisk_register(m,b,n,s) ramdisk_register(m,(FAR uint8_t *)b,n,s,0)
#else
int romdisk_register(int minor, FAR const uint8_t *buffer, uint32_t nsectors,
                     uint16_t sectsize);
#endif

/****************************************************************************
 * Name: mkrd
 *
 * Description:
 *   This is a wrapper function around ramdisk_register.  It combines the
 *   necessary operations to create a RAM disk into a single callable
 *   function.  Memory for the RAM disk is allocated, appropriated, from
 *   the kernel heap (in build modes where there is a distinct kernel heap).
 *
 * Input Parameters:
 *   minor:         Selects suffix of device named /dev/ramN, N={1,2,3...}
 *   nsectors:      Number of sectors on device
 *   sectize:       The size of one sector
 *   rdflags:       See RDFLAG_* definitions.  Typically
 *                  RDFLAG_WRENABLED | RDFLAG_FUNLINK
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_DRVR_MKRD
int mkrd(int minor, uint32_t nsectors, uint16_t sectsize, uint8_t rdflags);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_DRIVERS_RAMDISK_H */
