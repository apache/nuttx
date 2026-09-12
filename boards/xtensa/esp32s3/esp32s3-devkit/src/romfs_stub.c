/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-devkit/src/romfs_stub.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>

#ifndef HAVE_ROMFS_BOOT

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Placeholder for the boot ROMFS image of a kernel build.  It lets the
 * kernel link before apps/tools/mkromfsimg.sh has generated the real
 * romfs_boot.c.  A kernel built against this placeholder has no user-space
 * programs and so cannot start its init process; esp32s3_bringup() says so
 * on the console.
 *
 * HAVE_ROMFS_BOOT is defined by src/Make.defs once the generated image is in
 * place, and this file then contributes nothing.
 */

const unsigned char aligned_data(4) romfs_img[] =
{
  0x00
};

const unsigned int romfs_img_len = 1;

#endif /* !HAVE_ROMFS_BOOT */

/****************************************************************************
 * Public Functions
 ****************************************************************************/
