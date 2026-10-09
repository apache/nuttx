/****************************************************************************
 * include/nuttx/fs/cookie.h
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

#ifndef __INCLUDE_NUTTX_FS_COOKIE_H
#define __INCLUDE_NUTTX_FS_COOKIE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <sys/types.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Definitions for custom stream operations with fopencookie. The
 * implementation is as defined in Standard C library (libc). The only
 * difference is that we use off_t instead of off64_t. This means
 * off_t is int64_t if CONFIG_FS_LARGEFILE is defined and int32_t if not.
 *
 * These callbacks can either lead to custom functions if fopencookie is used
 * or to standard file system functions if not.
 *
 * They are in their own header because stdio.h needs them, and a program
 * of a kernel build must not see the rest of nuttx/fs/fs.h.
 */

typedef CODE ssize_t cookie_read_function_t(FAR void *cookie, FAR char *buf,
                                            size_t size);
typedef CODE ssize_t cookie_write_function_t(FAR void *cookie,
                                             FAR const char *buf,
                                             size_t size);
typedef CODE off_t cookie_seek_function_t(FAR void *cookie,
                                          FAR off_t *offset,
                                          int whence);
typedef CODE int cookie_close_function_t(FAR void *cookie);

typedef struct cookie_io_functions_t
{
  FAR cookie_read_function_t *read;
  FAR cookie_write_function_t *write;
  FAR cookie_seek_function_t *seek;
  FAR cookie_close_function_t *close;
} cookie_io_functions_t;

#endif /* __INCLUDE_NUTTX_FS_COOKIE_H */
