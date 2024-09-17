/****************************************************************************
 * include/nuttx/fs/uio.h
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

#ifndef __INCLUDE_NUTTX_FS_UIO_H
#define __INCLUDE_NUTTX_FS_UIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: iovec_total_len
 *
 * Description:
 *   Return the total length of data in bytes.
 *   Or -EOVERFLOW.
 *
 ****************************************************************************/

struct iovec;
ssize_t iovec_total_len(FAR const struct iovec *iov, int iovcnt);

/****************************************************************************
 * Name: iovec_compat_readv
 *
 * Description:
 *   Emulate readv using file_operation::compat_read.
 *
 ****************************************************************************/

struct file;
ssize_t iovec_compat_readv(FAR struct file *filep,
                           FAR const struct iovec *iov,
                           int iovcnt);

/****************************************************************************
 * Name: iovec_compat_writev
 *
 * Description:
 *   Emulate writev using file_operation::compat_write.
 *
 ****************************************************************************/

ssize_t iovec_compat_writev(FAR struct file *filep,
                            FAR const struct iovec *iov,
                            int iovcnt);

#endif /* __INCLUDE_NUTTX_FS_UIO_H */
