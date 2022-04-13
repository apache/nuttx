/****************************************************************************
 * libs/libc/stream/lib_blkoutstream.c
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

#include "libc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_DISABLE_MOUNTPOINT

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: blkoutstream_flush
 ****************************************************************************/

static int blkoutstream_flush(FAR struct lib_outstream_s *this)
{
  FAR struct lib_blkoutstream_s *stream =
                                 (FAR struct lib_blkoutstream_s *)this;
  size_t sectorsize = stream->geo.geo_sectorsize;
  int ret = OK;

  if (this->nput % sectorsize > 0)
    {
      ret = stream->inode->u.i_bops->write(stream->inode, stream->cache,
                                           this->nput / sectorsize, 1);
    }

  return ret;
}

/****************************************************************************
 * Name: blkoutstream_puts
 ****************************************************************************/

static int blkoutstream_puts(FAR struct lib_outstream_s *this,
                             FAR const void *buf, int len)
{
  FAR struct lib_blkoutstream_s *stream =
                                 (FAR struct lib_blkoutstream_s *)this;
  size_t sectorsize = stream->geo.geo_sectorsize;
  FAR struct inode *inode = stream->inode;
  FAR const unsigned char *ptr = buf;
  size_t remain = len;
  int ret;

  while (remain > 0)
    {
      size_t sblock = this->nput / sectorsize;
      size_t offset = this->nput % sectorsize;

      if (offset > 0)
        {
          size_t copyin = offset + remain > sectorsize ?
                          sectorsize - offset : remain;

          memcpy(stream->cache + offset, ptr, copyin);

          ptr        += copyin;
          offset     += copyin;
          this->nput += copyin;
          remain     -= copyin;

          if (offset == stream->geo.geo_sectorsize)
            {
              ret = inode->u.i_bops->write(inode, stream->cache, sblock, 1);
              if (ret < 0)
                {
                  return ret;
                }
            }
        }
      else if (remain < stream->geo.geo_sectorsize)
        {
          memcpy(stream->cache, ptr, remain);
          this->nput += remain;
          remain      = 0;
        }
      else if (remain >= stream->geo.geo_sectorsize)
        {
          size_t copyin = (remain / stream->geo.geo_sectorsize) *
                                    stream->geo.geo_sectorsize;

          ret = inode->u.i_bops->write(inode, ptr, sblock,
                                       remain / stream->geo.geo_sectorsize);
          if (ret < 0)
            {
              return ret;
            }

          ptr        += copyin;
          this->nput += copyin;
          remain     -= copyin;
        }
    }

  return len;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_blkoutstream_close
 *
 * Description:
 *  close block driver stream backend
 *
 * Input Parameters:
 *   stream  - User allocated, uninitialized instance of struct
 *                lib_blkoutstream_s to be initialized.
 *
 * Returned Value:
 *   None (User allocated instance initialized).
 *
 ****************************************************************************/

void lib_blkoutstream_close(FAR struct lib_blkoutstream_s *stream)
{
  if (stream != NULL)
    {
      if (stream->inode != NULL)
        {
          close_blockdriver(stream->inode);
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
 * Name: lib_blkoutstream_open
 *
 * Description:
 *  block driver stream backend
 *
 * Input Parameters:
 *   stream  - User allocated, uninitialized instance of struct
 *                lib_blkoutstream_s to be initialized.
 *   name    - The full path to the block driver to be opened.
 *
 * Returned Value:
 *   Returns zero on success or a negated errno on failure
 *
 ****************************************************************************/

int lib_blkoutstream_open(FAR struct lib_blkoutstream_s *stream,
                          FAR const char *name)
{
  FAR struct inode *inode = NULL;
  int ret;

  if (stream == NULL || name == NULL)
    {
      return -EINVAL;
    }

  ret = open_blockdriver(name, 0, &inode);
  if (ret < 0)
    {
      return ret;
    }

  memset(stream, 0, sizeof(*stream));

  if (inode->u.i_bops->geometry == NULL ||
      inode->u.i_bops->write == NULL ||
      inode->u.i_bops->geometry(inode, &stream->geo) < 0 ||
      stream->geo.geo_sectorsize <= 0 ||
      stream->geo.geo_nsectors <= 0)
    {
      close_blockdriver(inode);
      return -EINVAL;
    }

  stream->cache = lib_malloc(stream->geo.geo_sectorsize);
  if (stream->cache == NULL)
    {
      close_blockdriver(inode);
      return -ENOMEM;
    }

  stream->inode        = inode;
  stream->public.puts  = blkoutstream_puts;
  stream->public.flush = blkoutstream_flush;

  return OK;
}
#endif
