/************************************************************************************
 * include/nuttx/eeprom/eeprom.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * EEPROM IOCTL commands
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
 ************************************************************************************/

/* This file includes common definitions to be used in all EEPROM drivers
 * (when applicable).
 */

#ifndef __INCLUDE_NUTTX_EEPROM_EEPROM_H
#define __INCLUDE_NUTTX_EEPROM_EEPROM_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <sys/types.h>

#include <nuttx/fs/ioctl.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* EEPROM IOCTL Commands ************************************************************/

#define EEPIOC_GEOMETRY     _EEPIOC(0x000)  /* Similar to BIOC_GEOMETRY:
                                             * Return the geometry of the EEPROM
                                             * device.
                                             * IN:  Pointer to writable instance  of
                                             *      struct eeprom_geometry_s in which
                                             *      to return the geometry.
                                             * OUT: Data return in user-provided
                                             *      buffer. */

/************************************************************************************
 * Type Definitions
 ************************************************************************************/

struct eeprom_geometry_s
{
  blkcnt_t  npages;   /* Number of pages on the device */
  blksize_t sectsize; /* Size of one sector in bytes */
  blksize_t pagesize; /* Size of one page in bytes */
};

#endif /* __INCLUDE_NUTTX_EEPROM_EEPROM_H */
