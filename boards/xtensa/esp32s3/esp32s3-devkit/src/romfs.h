/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-devkit/src/romfs.h
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

#ifndef __BOARDS_XTENSA_ESP32S3_ESP32S3_DEVKIT_SRC_ROMFS_H
#define __BOARDS_XTENSA_ESP32S3_ESP32S3_DEVKIT_SRC_ROMFS_H

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* The boot ROMFS image holding the user-space programs of a kernel build.
 * The real image is generated from apps/bin by apps/tools/mkromfsimg.sh into
 * romfs_boot.c; romfs_stub.c provides a weak, empty placeholder so that the
 * kernel still links before that step has been run.
 */

extern const unsigned char romfs_img[];
extern const unsigned int romfs_img_len;

#endif /* __BOARDS_XTENSA_ESP32S3_ESP32S3_DEVKIT_SRC_ROMFS_H */
