/****************************************************************************
 * libs/libc/stream/lib_mtdoutstream.c
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

#include <unistd.h>
#include <nuttx/streams.h>
#include <nuttx/fs/fs.h>
#include <nuttx/mtd/mtd.h>

#include "libc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && defined(CONFIG_MTD)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mtdoutstream_bwrite_cache
 ****************************************************************************/

static ssize_t
mtdoutstream_bwrite_cache(FAR struct lib_mtdoutstream_s *stream,
                          FAR struct mtd_dev_s *mtd, size_t block)
{
  size_t nblkpererase = stream->geo.erasesize / stream->geo.blocksize;
  ssize_t ret;

  if (block % nblkpererase == 0)
    {
      ret = MTD_ERASE(mtd, block / nblkpererase, 1);
      if (ret < 0)
        {
          return ret;
        }
    }

  ret = MTD_BWRITE(mtd, block, 1, stream->cache);
  if (ret < 0)
    {
      return ret;
    }

  return ret;
}

static ssize_t mtdoutstream_bwrite(FAR struct lib_mtdoutstream_s *stream,
                                   FAR struct mtd_dev_s *mtd,
                                   size_t sblock, size_t nblock,
                                   FAR const unsigned char *buf)
{
  size_t nblkpererase = stream->geo.erasesize / stream->geo.blocksize;
  size_t serase = (sblock + nblkpererase - 1) / nblkpererase;
  size_t eerase = (sblock + nblock + nblkpererase - 1) / nblkpererase;
  ssize_t ret;

  if (serase != eerase)
    {
      ret = MTD_ERASE(mtd, serase, eerase - serase);
      if (ret < 0)
        {
          return ret;
        }
    }

  ret = MTD_BWRITE(mtd, sblock, nblock, buf);
  if (ret < 0)
    {
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: mtdoutstream_flush
 ****************************************************************************/

static int mtdoutstream_flush(FAR struct lib_outstream_s *self)
{
  FAR struct lib_mtdoutstream_s *stream =
    (FAR struct lib_mtdoutstream_s *)self;
  FAR struct mtd_dev_s *mtd = stream->inode->u.i_mtd;
  size_t blocksize = stream->geo.blocksize;
  int ret = OK;

  if (self->nput % blocksize > 0)
    {
#ifdef CONFIG_MTD_BYTE_WRITE
      /* If byte write, flush won't be needed */

      if (mtd->write == NULL)
#endif
        {
          ret = mtdoutstream_bwrite_cache(stream, mtd,
                                          self->nput / blocksize);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: mtdoutstream_puts
 ****************************************************************************/

static ssize_t mtdoutstream_puts(FAR struct lib_outstream_s *self,
                                 FAR const void *buf, size_t len)
{
  FAR struct lib_mtdoutstream_s *stream =
    (FAR struct lib_mtdoutstream_s *)self;
  FAR struct mtd_dev_s *mtd = stream->inode->u.i_mtd;
  size_t erasesize = stream->geo.erasesize;
  size_t blocksize = stream->geo.blocksize;
  FAR const unsigned char *ptr = buf;
  size_t remain = len;
  ssize_t ret;

  if (self->nput + len > erasesize * stream->geo.neraseblocks)
    {
      return -ENOSPC;
    }

#ifdef CONFIG_MTD_BYTE_WRITE
  if (mtd->write != NULL)
    {
      size_t serase = (self->nput + erasesize - 1) / erasesize;
      size_t eerase = (self->nput + len + erasesize - 1) / erasesize;

      if (serase != eerase)
        {
          ret = MTD_ERASE(mtd, serase, eerase - serase);
          if (ret < 0)
            {
              return ret;
            }
        }

      ret = MTD_WRITE(mtd, self->nput, len, buf);
      if (ret < 0)
        {
          return ret;
        }

      self->nput += len;
    }
  else
#endif
    {
      while (remain > 0)
        {
          off_t sblock = self->nput / blocksize;
          off_t offset = self->nput % blocksize;

          if (offset > 0)
            {
              size_t copying = offset + remain > blocksize ?
                               blocksize - offset : remain;

              memcpy(stream->cache + offset, ptr, copying);

              ptr        += copying;
              offset     += copying;
              self->nput += copying;
              remain     -= copying;

              if (offset == blocksize)
                {
                  ret = mtdoutstream_bwrite_cache(stream, mtd, sblock);
                  if (ret < 0)
                    {
                      return ret;
                    }
                }
            }
          else if (remain < blocksize)
            {
              ret = MTD_BREAD(mtd, sblock, 1, stream->cache);
              if (ret < 0)
                {
                  return ret;
                }

              memcpy(stream->cache, ptr, remain);
              self->nput += remain;
              remain      = 0;
            }
          else
            {
              size_t nblock = remain / blocksize;
              size_t copying = nblock * blocksize;

              ret = mtdoutstream_bwrite(stream, mtd, sblock, nblock, ptr);
              if (ret < 0)
                {
                  return ret;
                }

              ptr        += copying;
              self->nput += copying;
              remain     -= copying;
            }
        }
    }

  return len;
}

/****************************************************************************
 * Name: mtdoutstream_putc
 ****************************************************************************/

static void mtdoutstream_putc(FAR struct lib_outstream_s *self, int ch)
{
  char tmp = ch;
  mtdoutstream_puts(self, &tmp, 1);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_mtdoutstream_close
 *
 * Description:
 *  close mtd driver stream backend
 *
 * Input Parameters:
 *   stream  - User allocated, uninitialized instance of struct
 *                lib_mtdoutstream_s to be initialized.
 *
 * Returned Value:
 *   None (User allocated instance initialized).
 *
 ****************************************************************************/

void lib_mtdoutstream_close(FAR struct lib_mtdoutstream_s *stream)
{
  if (stream != NULL)
    {
      if (stream->cache != NULL)
        {
          mtdoutstream_flush(&stream->common);
          lib_free(stream->cache);
          stream->cache = NULL;
        }

      if (stream->inode != NULL)
        {
          close_mtddriver(stream->inode);
          stream->inode = NULL;
        }
    }
}

/****************************************************************************
 * Name: lib_mtdoutstream_open
 *
 * Description:
 *  mtd driver stream backend
 *
 * Input Parameters:
 *   stream   - User allocated, uninitialized instance of struct
 *                lib_mtdoutstream_s to be initialized.
 *   name     - The full path of mtd device.
 *
 * Returned Value:
 *   Returns zero on success or a negated errno on failure
 *
 ****************************************************************************/

int lib_mtdoutstream_open(FAR struct lib_mtdoutstream_s *stream,
                          FAR const char *name)
{
  FAR struct inode *node = NULL;
  int ret;

  if (stream == NULL || name == NULL)
    {
      return -EINVAL;
    }

  ret = find_mtddriver(name, &node);
  if (ret < 0)
    {
      return ret;
    }

  memset(stream, 0, sizeof(*stream));

  if (node->u.i_mtd->ioctl == NULL ||
      node->u.i_mtd->erase == NULL ||
      node->u.i_mtd->bwrite == NULL ||
      node->u.i_mtd->ioctl(node->u.i_mtd, MTDIOC_GEOMETRY,
                           (unsigned long)&stream->geo) < 0 ||
      stream->geo.blocksize <= 0 ||
      stream->geo.erasesize <= 0 ||
      stream->geo.neraseblocks <= 0)
    {
      close_mtddriver(node);
      return -EINVAL;
    }

  /* If mtd driver support the byte write,
   * the temp buffer is not needed at all.
   */

#ifdef CONFIG_MTD_BYTE_WRITE
  if (node->u.i_mtd->write == NULL)
#endif
    {
      stream->cache = lib_malloc(stream->geo.blocksize);
      if (stream->cache == NULL)
        {
          close_mtddriver(node);
          return -ENOMEM;
        }
    }

  stream->inode        = node;
  stream->common.putc  = mtdoutstream_putc;
  stream->common.puts  = mtdoutstream_puts;
  stream->common.flush = mtdoutstream_flush;

  return OK;
}

#endif /* CONFIG_MTD */
