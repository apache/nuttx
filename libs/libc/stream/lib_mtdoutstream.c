/****************************************************************************
 * libs/libc/stream/lib_mtdoutstream.c
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
 * Name: mtdoutstream_flush
 ****************************************************************************/

static int mtdoutstream_flush(FAR struct lib_outstream_s *this)
{
  FAR struct lib_mtdoutstream_s *stream =
    (FAR struct lib_mtdoutstream_s *)this;
  size_t erasesize = stream->geo.erasesize;
  size_t nblkpererase = erasesize / stream->geo.blocksize;
  int ret = OK;

  if (this->nput % erasesize > 0)
    {
#ifdef CONFIG_MTD_BYTE_WRITE
      /* if byte write, flush won't be needed */

      if (stream->inode->u.i_mtd->write == NULL)
#endif
        {
          ret = MTD_ERASE(stream->inode->u.i_mtd, this->nput / erasesize, 1);
          if (ret < 0)
            {
              return ret;
            }

          ret = MTD_BWRITE(stream->inode->u.i_mtd,
                           this->nput / erasesize * nblkpererase,
                           nblkpererase, stream->cache);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: mtdoutstream_puts
 ****************************************************************************/

static int mtdoutstream_puts(FAR struct lib_outstream_s *this,
                             FAR const void *buf, int len)
{
  FAR struct lib_mtdoutstream_s *stream =
    (FAR struct lib_mtdoutstream_s *)this;
  size_t erasesize = stream->geo.erasesize;
  size_t nblkpererase = erasesize / stream->geo.blocksize;
  FAR struct inode *inode = stream->inode;
  FAR const unsigned char *ptr = buf;
  size_t remain = len;
  int ret;

  if (this->nput + len > erasesize * stream->geo.neraseblocks)
    {
      return -ENOSPC;
    }

#ifdef CONFIG_MTD_BYTE_WRITE
  if (stream->inode->u.i_mtd->write != NULL)
    {
      if (this->nput % stream->geo.erasesize == 0)
        {
          ret = MTD_ERASE(inode->u.i_mtd,
                          this->nput / stream->geo.erasesize, 1);
          if (ret < 0)
            {
              return ret;
            }
        }

      ret = MTD_WRITE(inode->u.i_mtd, this->nput, len, buf);
      if (ret < 0)
        {
          return ret;
        }

      this->nput += len;
    }
  else
#endif
    {
      while (remain > 0)
        {
          size_t sblock = this->nput / erasesize;
          size_t offset = this->nput % erasesize;

          if (offset > 0)
            {
              size_t copyin = offset + remain > erasesize ?
                              erasesize - offset : remain;

              memcpy(stream->cache + offset, ptr, copyin);

              ptr        += copyin;
              offset     += copyin;
              this->nput += copyin;
              remain     -= copyin;

              if (offset == erasesize)
                {
                  ret = MTD_ERASE(inode->u.i_mtd, sblock, 1);
                  if (ret < 0)
                    {
                      return ret;
                    }

                  ret = MTD_BWRITE(inode->u.i_mtd, sblock * nblkpererase,
                                   nblkpererase, stream->cache);
                  if (ret < 0)
                    {
                      return ret;
                    }
                }
            }
          else if (remain < erasesize)
            {
              /* erase content to all 0 before caching,
               * so no random content will be flushed
               */

              memset(stream->cache, 0, stream->geo.erasesize);
              memcpy(stream->cache, ptr, remain);
              this->nput += remain;
              remain      = 0;
            }
          else if (remain >= erasesize)
            {
              size_t copyin = (remain / erasesize) * erasesize;

              ret = MTD_ERASE(inode->u.i_mtd, sblock,
                              remain / erasesize);
              if (ret < 0)
                {
                  return ret;
                }

              ret = MTD_BWRITE(inode->u.i_mtd, sblock * nblkpererase,
                               remain / erasesize * nblkpererase,
                               ptr);
              if (ret < 0)
                {
                  return ret;
                }

              ptr        += copyin;
              this->nput += copyin;
              remain     -= copyin;
            }
        }
    }

  return len;
}

/****************************************************************************
 * Name: mtdoutstream_putc
 ****************************************************************************/

static void mtdoutstream_putc(FAR struct lib_outstream_s *this, int ch)
{
  char tmp = ch;
  mtdoutstream_puts(this, &tmp, 1);
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
      if (stream->inode != NULL)
        {
          close_mtddriver(stream->inode);
          stream->inode = NULL;
        }

      if (stream->cache != NULL)
        {
          lib_free(stream->cache);
          stream->cache = NULL;
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

#ifdef CONFIG_MTD_BYTE_WRITE
  if (node->u.i_mtd->write == NULL)
#endif
    {
      stream->cache = lib_zalloc(stream->geo.erasesize);
      if (stream->cache == NULL)
        {
          close_mtddriver(node);
          return -ENOMEM;
        }
    }

  stream->inode        = node;
  stream->public.putc  = mtdoutstream_putc;
  stream->public.puts  = mtdoutstream_puts;
  stream->public.flush = mtdoutstream_flush;

  return OK;
}

#endif /* CONFIG_MTD */
