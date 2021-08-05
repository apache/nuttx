/****************************************************************************
 * include/nuttx/zoneinfo.h
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

#ifndef __INCLUDE_NUTTX_ZONEINFO_H
#define __INCLUDE_NUTTX_ZONEINFO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_LIBC_ZONEINFO_ROMFS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* If the TZ database is built as a ROMFS file system, then these variables
 * provide (1) the address of the array in FLASH/ROM that contains the
 * ROMFS file system image, and (2) the size of the ROMFS file system image
 * in bytes.  This is sufficient information to permit external logic to
 * mount the ROMF file system.
 */

EXTERN unsigned char romfs_zoneinfo_img[];
EXTERN unsigned int  romfs_zoneinfo_img_len;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_LIBC_ZONEINFO_ROMFS */
#endif /* __INCLUDE_NUTTX_ZONEINFO_H */
