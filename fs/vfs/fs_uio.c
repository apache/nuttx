/****************************************************************************
 * fs/vfs/fs_uio.c
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

#include <nuttx/config.h>

#include <sys/uio.h>
#include <sys/types.h>

#include <nuttx/fs/fs.h>

#include <assert.h>
#include <errno.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: uio_resid
 *
 * Description:
 *   Return the remaining length of data in bytes.
 *   Or -EOVERFLOW.
 *
 ****************************************************************************/

ssize_t uio_resid(FAR const struct uio *uio)
{
  const struct iovec *iov = uio->uio_iov;
  int iovcnt = uio->uio_iovcnt;
  size_t offset_in_iov = uio->uio_offset_in_iov;
  size_t len = 0;
  int i;

  for (i = 0; i < iovcnt; i++)
    {
      DEBUGASSERT(offset_in_iov <= iov[i].iov_len);
      if (SSIZE_MAX - len < iov[i].iov_len - offset_in_iov)
        {
          return -EOVERFLOW;
        }

      len += iov[i].iov_len - offset_in_iov;
      offset_in_iov = 0;
    }

  return len;
}

/****************************************************************************
 * Name: uio_advance
 *
 * Description:
 *   Advance the pointer/offset in uio by the specified amount.
 *
 ****************************************************************************/

void uio_advance(FAR struct uio *uio, size_t sz)
{
  FAR const struct iovec *iov = uio->uio_iov;
  int iovcnt = uio->uio_iovcnt;
  size_t offset_in_iov = uio->uio_offset_in_iov;

  DEBUGASSERT(sz <= uio_resid(uio));
  while (iovcnt > 0)
    {
      DEBUGASSERT(offset_in_iov <= iov->iov_len);
      if (sz < iov->iov_len - offset_in_iov)
        {
          offset_in_iov += sz;
          break;
        }

      sz -= iov->iov_len - offset_in_iov;
      iov++;
      iovcnt--;
      offset_in_iov = 0;
    }

  uio->uio_iov = iov;
  uio->uio_iovcnt = iovcnt;
  uio->uio_offset_in_iov = offset_in_iov;
}

/****************************************************************************
 * Name: uio_init
 *
 * Description:
 *   Initialize the uio structure with reasonable default values.
 *
 ****************************************************************************/

void uio_init(FAR struct uio *uio, FAR const struct iovec *iov, int iovcnt)
{
  memset(uio, 0, sizeof(*uio));
  uio->uio_iov = iov;
  uio->uio_iovcnt = iovcnt;
}

/****************************************************************************
 * Name: uio_copyfrom
 *
 * Description:
 *   Copy data from the linear buffer to uio.
 *
 ****************************************************************************/

void uio_copyfrom(FAR struct uio *uio, size_t offset, FAR const void *buf,
                  size_t len)
{
  FAR const struct iovec *iov = uio->uio_iov;

  DEBUGASSERT(uio_resid(uio) >= 0);
  DEBUGASSERT(len <= uio_resid(uio));
  DEBUGASSERT(offset <= uio_resid(uio) - len);
  DEBUGASSERT(SSIZE_MAX - offset >= uio->uio_offset_in_iov);

  offset += uio->uio_offset_in_iov;
  while (offset > iov->iov_len)
    {
      offset -= iov->iov_len;
      iov++;
    }

  while (len > 0)
    {
      size_t blen = len;
      if (blen > iov->iov_len - offset)
        {
          blen = iov->iov_len - offset;
        }

      memcpy((FAR uint8_t *)iov->iov_base + offset, buf, blen);

      len -= blen;
      if (len == 0)
        {
          break;
        }

      buf = (const uint8_t *)buf + blen;
      iov++;
      offset = 0;
    }
}

/****************************************************************************
 * Name: uio_copyto
 *
 * Description:
 *   Copy data to the linear buffer from uio.
 *
 ****************************************************************************/

void uio_copyto(FAR struct uio *uio, size_t offset, FAR void *buf,
                size_t len)
{
  FAR const struct iovec *iov = uio->uio_iov;

  DEBUGASSERT(uio_resid(uio) >= 0);
  DEBUGASSERT(len <= uio_resid(uio));
  DEBUGASSERT(offset <= uio_resid(uio) - len);
  DEBUGASSERT(SSIZE_MAX - offset >= uio->uio_offset_in_iov);

  offset += uio->uio_offset_in_iov;
  while (offset > iov->iov_len)
    {
      offset -= iov->iov_len;
      iov++;
    }

  while (len > 0)
    {
      size_t blen = len;
      if (blen > iov->iov_len - offset)
        {
          blen = iov->iov_len - offset;
        }

      memcpy(buf, (FAR const uint8_t *)iov->iov_base + offset, blen);

      len -= blen;
      if (len == 0)
        {
          break;
        }

      buf = (uint8_t *)buf + blen;
      iov++;
      offset = 0;
    }
}
