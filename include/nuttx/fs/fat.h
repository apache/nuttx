/****************************************************************************
 * include/nuttx/fs/fat.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __INCLUDE_NUTTX_FS_FAT_H
#define __INCLUDE_NUTTX_FS_FAT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* File attribute bits in FAT directory entry */

#define FATATTR_READONLY  0x01
#define FATATTR_HIDDEN    0x02
#define FATATTR_SYSTEM    0x04
#define FATATTR_VOLUMEID  0x08
#define FATATTR_DIRECTORY 0x10
#define FATATTR_ARCHIVE   0x20

#define FATATTR_LONGNAME \
  (FATATTR_READONLY|FATATTR_HIDDEN|FATATTR_SYSTEM|FATATTR_VOLUMEID)

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

typedef uint8_t fat_attrib_t;

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
 * Name: fat_getattrib and fat_setattrib
 *
 * Description:
 *   Non-standard functions to get and set FAT file/directory attributes
 *
 ****************************************************************************/

int fat_getattrib(FAR const char *path, FAR fat_attrib_t *attrib);
int fat_setattrib(FAR const char *path, fat_attrib_t setbits,
                  fat_attrib_t clearbits);


#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_FS_FAT_H */
