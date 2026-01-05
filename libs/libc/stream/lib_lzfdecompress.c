/****************************************************************************
 * libs/libc/stream/lib_lzfdecompress.c
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

#include <assert.h>
#include <errno.h>
#include <sys/endian.h>
#include <sys/param.h>
#include <nuttx/streams.h>

#include "libc.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef CODE int (*lzf_foreach_cb)(FAR struct lib_lzfsistream_s *stream,
                                   size_t blklen, size_t blkclen,
                                   off_t offset, FAR void *arg);

struct lzf_uncompress_s
{
  FAR uint8_t *buffer;
  size_t size;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lzf_header_foreach
 ****************************************************************************/

static int lzf_header_foreach(FAR struct lib_lzfsistream_s *lzfstream,
                              lzf_foreach_cb cb, FAR void *arg)
{
  FAR struct lib_sistream_s *stream = lzfstream->backend;
  struct lzf_type1_header_s hdr;
  size_t hdrsize;
  size_t blkclen;
  size_t blklen;
  int ret;

  for (; ; )
    {
      ret = lib_stream_seek(stream, lzfstream->blkcoff, SEEK_SET);
      if (ret < 0)
        {
          return ret;
        }

      ret = lib_stream_gets(stream, &hdr, sizeof(hdr));
      if (ret < sizeof(hdr))
        {
          return ret < 0 ? ret : -EINVAL;
        }

      if (hdr.lzf_magic[0] != 'Z' || hdr.lzf_magic[1] != 'V')
        {
          return -EINVAL;
        }

      if (LZF_TYPE0_HDR == hdr.lzf_type)
        {
          struct lzf_type0_header_s *hdr0 =
                           (struct lzf_type0_header_s *)&hdr;
          hdrsize = LZF_TYPE0_HDR_SIZE;
          blklen = be16toh(*(uint16_t *)hdr0->lzf_len);
          blkclen = blklen;
        }
      else if (LZF_TYPE1_HDR == hdr.lzf_type)
        {
          hdrsize = LZF_TYPE1_HDR_SIZE;
          blkclen = be16toh(*(uint16_t *)hdr.lzf_clen);
          blklen = be16toh(*(uint16_t *)hdr.lzf_ulen);
        }
      else
        {
          return -EINVAL;
        }

      ret = cb(lzfstream, blklen, blkclen, lzfstream->blkoff, arg);
      if (ret <= 0)
        {
          break;
        }

      lzfstream->blkcoff += blkclen + hdrsize;
      lzfstream->blkoff += blklen;
    }

  return ret;
}

/****************************************************************************
 * Name: lzf_uncompress
 ****************************************************************************/

static int lzf_uncompress(FAR struct lib_lzfsistream_s *lzfstream,
                          size_t blklen, size_t blkclen,
                          off_t offset, FAR void *arg)
{
  /* Implementation of memory copy from stream based on header information */

  FAR struct lzf_uncompress_s *uncompress =
                              (FAR struct lzf_uncompress_s *)arg;
  FAR struct lib_sistream_s *stream = lzfstream->backend;
  size_t ncopy;
  int ret;

  if (uncompress->size == 0 ||
      lzfstream->offset < offset || lzfstream->offset >= offset + blklen)
    {
      return uncompress->size;
    }

  ret = lib_stream_gets(stream, lzfstream->in, blkclen);
  if (ret < blkclen)
    {
      return ret < 0 ? ret : -EIO;
    }

  if (lzf_decompress(lzfstream->in, blkclen,
                     lzfstream->out, LZF_STREAM_BLOCKSIZE) != blklen)
    {
      return -EINVAL;
    }

  offset = lzfstream->offset - offset;
  blklen -= offset;
  ncopy = MIN(uncompress->size, blklen);

  memcpy(uncompress->buffer, lzfstream->out + offset, ncopy);

  lzfstream->offset += ncopy;
  uncompress->buffer += ncopy;
  uncompress->size -= ncopy;

  return uncompress->size;
}

/****************************************************************************
 * Name: lzfsistream_gets
 ****************************************************************************/

static ssize_t lzfsistream_gets(FAR struct lib_sistream_s *self,
                                FAR void *buffer, size_t len)
{
  FAR struct lib_lzfsistream_s *stream =
                               (FAR struct lib_lzfsistream_s *)self;
  struct lzf_uncompress_s uncompress =
    {
      .buffer = buffer,
      .size = len
    };

  int ret = lzf_header_foreach(stream, lzf_uncompress, &uncompress);
  if (ret < 0)
    {
      return ret;
    }

  return len - uncompress.size;
}

/****************************************************************************
 * Name: lzfsistream_getc
 ****************************************************************************/

static int lzfsistream_getc(FAR struct lib_sistream_s *self)
{
  char c;
  return lzfsistream_gets(self, &c, 1) == 1 ? c : EOF;
}

/****************************************************************************
 * Name: lzfsistream_seek
 ****************************************************************************/

static off_t lzfsistream_seek(FAR struct lib_sistream_s *self, off_t offset,
                              int whence)
{
  FAR struct lib_lzfsistream_s *stream =
                               (FAR struct lib_lzfsistream_s *)self;

  DEBUGASSERT(self);

  switch (whence)
    {
      case SEEK_CUR:
        offset += stream->offset;
      case SEEK_SET:
        if (offset >= 0)
          {
            break;
          }

      case SEEK_END:
      default:
        return -EINVAL;
    }

  stream->offset = offset;
  if (offset < stream->blkoff)
    {
      stream->blkcoff = 0;
      stream->blkoff = 0;
    }

  /* Return the new position */

  return stream->offset;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_lzfsistream
 *
 * Description:
 *   Initializes a stream for use with a fixed-size memory buffer.
 *
 * Input Parameters:
 *   instream    - User allocated, uninitialized instance of struct
 *                 lib_lzfsistream_s to be initialized.
 *   bufstart    - Address of the beginning of the fixed-size memory buffer
 *   buflen      - Size of the fixed-sized memory buffer in bytes
 *
 * Returned Value:
 *   None (instream initialized).
 *
 ****************************************************************************/

void lib_lzfsistream(FAR struct lib_lzfsistream_s *sistream,
                     FAR struct lib_sistream_s *backend)
{
  memset(sistream, 0, sizeof(*sistream));

  sistream->common.getc = lzfsistream_getc;
  sistream->common.gets = lzfsistream_gets;
  sistream->common.seek = lzfsistream_seek;
  sistream->backend = backend;
}
