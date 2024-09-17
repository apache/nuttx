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
 * Public Type Definitions
 ****************************************************************************/

/* The structure to describe an user I/O operation.
 *
 * At this point, this is a bare minimum for readv/writev.
 * In the future, we might extend this for other things like
 * the file offset for pread/pwrite.
 *
 * This structure was inspired by BSDs.
 * (Thus it doesn't have the NuttX-style "_s" suffix.)
 */

struct uio
{
  FAR const struct iovec *uio_iov;
  int uio_iovcnt;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: uio_total_len
 *
 * Description:
 *   Return the total length of data in bytes.
 *   Or -EOVERFLOW.
 *
 ****************************************************************************/

ssize_t uio_total_len(FAR const struct uio *uio);

#endif /* __INCLUDE_NUTTX_FS_UIO_H */
