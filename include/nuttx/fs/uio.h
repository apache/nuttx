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
  size_t uio_resid;         /* the remaining bytes in the request */
  size_t uio_offset_in_iov; /* offset in uio_iov[0].iov_base */

#if 0 /* notyet; planned for pread/pwrite */
  off_t uio_offset;         /* offset in the file */
#endif
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: uio_advance
 *
 * Description:
 *   Advance the pointer/offset in uio by the specified amount.
 *
 ****************************************************************************/

void uio_advance(FAR struct uio *uio, size_t sz);

/****************************************************************************
 * Name: uio_init
 *
 * Description:
 *   Initialize the uio structure with reasonable default values.
 *
 * Return Value:
 *   0 on success. A negative error number on an error.
 *
 *   -EINVAL: The total size of the given iovec is too large.
 *            (Note: NetBSD's readv returns EINVAL in that case.
 *            I (yamamoto) couldn't find the specification in POSIX.)
 *
 ****************************************************************************/

int uio_init(FAR struct uio *uio, FAR const struct iovec *iov, int iovcnt);

/****************************************************************************
 * Name: uio_copyto
 *
 * Description:
 *   Copy data to the linear buffer from uio.
 *
 ****************************************************************************/

void uio_copyfrom(FAR struct uio *uio, size_t offset, FAR const void *buf,
                  size_t len);

/****************************************************************************
 * Name: uio_copyto
 *
 * Description:
 *   Copy data to the linear buffer from uio.
 *
 ****************************************************************************/

void uio_copyto(FAR struct uio *uio, size_t offset, FAR void *buf,
                size_t len);

#endif /* __INCLUDE_NUTTX_FS_UIO_H */
