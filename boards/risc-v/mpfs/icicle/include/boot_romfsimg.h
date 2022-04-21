/****************************************************************************
 * boards/risc-v/mpfs/icicle/include/boot_romfsimg.h
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

#ifndef __BOARDS_RISC_V_MPFS_ICICLE_INCLUDE_BOOT_ROMFSIMG_H
#define __BOARDS_RISC_V_MPFS_ICICLE_INCLUDE_BOOT_ROMFSIMG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>
const unsigned char aligned_data(4) romfs_img[] =
{
  0x00
};
unsigned int romfs_img_len = 1;

#endif /* __BOARDS_RISC_V_MPFS_ICICLE_INCLUDE_BOOT_ROMFSIMG_H */
