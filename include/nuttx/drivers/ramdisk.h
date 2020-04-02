/****************************************************************************
 * include/nuttx/drivers/ramdisk.h
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

int ramdisk_register(int minor, FAR uint8_t *buffer, uint32_t nsectors,
                     uint16_t sectsize, uint8_t rdflags);
#define romdisk_register(m,b,n,s) ramdisk_register(m,(FAR uint8_t *)b,n,s,0)

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
