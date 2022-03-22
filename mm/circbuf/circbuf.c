/****************************************************************************
 * mm/circbuf/circbuf.c
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

/* Note about locking: There is no locking required while only one reader
 * and one writer is using the circular buffer.
 * For multiple writer and one reader there is only a need to lock the
 * writer. And vice versa for only one writer and multiple reader there is
 * only a need to lock the reader.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mm/circbuf.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: circbuf_init
 *
 * Description:
 *   Initialize a circular buffer.
 *
 * Input Parameters:
 *   circ  - Address of the circular buffer to be used.
 *   base  - A pointer to circular buffer's internal buffer. It can be
 *           provided by caller because sometimes the creation of buffer
 *           is special or needs to preallocated, eg: DMA buffer.
 *           If NULL, a buffer of the given size will be allocated.
 *   bytes - The size of the internal buffer.
 *
 * Returned Value:
 *   Zero on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

int circbuf_init(FAR struct circbuf_s *circ, FAR void *base, size_t bytes)
{
  DEBUGASSERT(circ);
  DEBUGASSERT(!base || bytes);

  circ->external = !!base;

  if (!base && bytes)
    {
      base = kmm_malloc(bytes);
      if (!base)
        {
          return -ENOMEM;
        }
    }

  circ->base = base;
  circ->size = bytes;
  circ->head = 0;
  circ->tail = 0;

  return 0;
}

/****************************************************************************
 * Name: circbuf_resize
 *
 * Description:
 *   Resize a circular buffer (change buffer size).
 *
 * Input Parameters:
 *   circ  - Address of the circular buffer to be used.
 *   bytes - The size of the internal buffer.
 *
 * Returned Value:
 *   Zero on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

int circbuf_resize(FAR struct circbuf_s *circ, size_t bytes)
{
  FAR void *tmp = NULL;
  size_t len = 0;

  DEBUGASSERT(circ);
  DEBUGASSERT(!circ->external);
  if (bytes == circ->size)
    {
      return 0;
    }

  if (bytes)
    {
      tmp = kmm_malloc(bytes);
      if (!tmp)
        {
          return -ENOMEM;
        }

      len = circbuf_used(circ);
      if (bytes < len)
        {
          circbuf_skip(circ, len - bytes);
          len = bytes;
        }

      circbuf_read(circ, tmp, len);
    }

  kmm_free(circ->base);

  circ->base = tmp;
  circ->size = bytes;
  circ->head = len;
  circ->tail = 0;

  return 0;
}

/****************************************************************************
 * Name: circbuf_reset
 *
 * Description:
 *   Remove the entire circular buffer content.
 *
 * Input Parameters:
 *   circ  - Address of the circular buffer to be used.
 ****************************************************************************/

void circbuf_reset(FAR struct circbuf_s *circ)
{
  DEBUGASSERT(circ);
  circ->head = circ->tail = 0;
}

/****************************************************************************
 * Name: circbuf_uninit
 *
 * Description:
 *   Free the circular buffer.
 *
 * Input Parameters:
 *   circ  - Address of the circular buffer to be used.
 ****************************************************************************/

void circbuf_uninit(FAR struct circbuf_s *circ)
{
  DEBUGASSERT(circ);

  if (!circ->external)
    {
      kmm_free(circ->base);
    }

  memset(circ, 0, sizeof(*circ));
}

/****************************************************************************
 * Name: circbuf_size
 *
 * Description:
 *   Return size of the circular buffer.
 *
 * Input Parameters:
 *   circ  - Address of the circular buffer to be used.
 ****************************************************************************/

size_t circbuf_size(FAR struct circbuf_s *circ)
{
  DEBUGASSERT(circ);
  return circ->size;
}

/****************************************************************************
 * Name: circbuf_used
 *
 * Description:
 *   Return the used bytes of the circular buffer.
 *
 * Input Parameters:
 *   circ  - Address of the circular buffer to be used.
 ****************************************************************************/

size_t circbuf_used(FAR struct circbuf_s *circ)
{
  DEBUGASSERT(circ);
  return circ->head - circ->tail;
}

/****************************************************************************
 * Name: circbuf_space
 *
 * Description:
 *   Return the remaining space of the circular buffer.
 *
 * Input Parameters:
 *   circ  - Address of the circular buffer to be used.
 ****************************************************************************/

size_t circbuf_space(FAR struct circbuf_s *circ)
{
  return circbuf_size(circ) - circbuf_used(circ);
}

/****************************************************************************
 * Name: circbuf_is_init
 *
 * Description:
 *   Return true if the circular buffer had been initialized.
 *
 * Input Parameters:
 *   circ  - Address of the circular buffer to be used.
 ****************************************************************************/

bool circbuf_is_init(FAR struct circbuf_s *circ)
{
  return !!circ->base;
}

/****************************************************************************
 * Name: circbuf_is_empty
 *
 * Description:
 *   Return true if the circular buffer is empty.
 *
 * Input Parameters:
 *   circ  - Address of the circular buffer to be used.
 ****************************************************************************/

bool circbuf_is_empty(FAR struct circbuf_s *circ)
{
  return !circbuf_used(circ);
}

/****************************************************************************
 * Name: circbuf_is_full
 *
 * Description:
 *   Return true if the circular buffer is full.
 *
 * Input Parameters:
 *   circ  - Address of the circular buffer to be used.
 ****************************************************************************/

bool circbuf_is_full(FAR struct circbuf_s *circ)
{
  return !circbuf_space(circ);
}

/****************************************************************************
 * Name: circbuf_peekat
 *
 * Description:
 *   Get data speicified position from the circular buffer without removing
 *
 * Note :
 *   That with only one concurrent reader and one concurrent writer,
 *   you don't need extra locking to use these api.
 *
 * Input Parameters:
 *   circ  - Address of the circular buffer to be used.
 *   pos   - Position to read.
 *   dst   - Address where to store the data.
 *   bytes - Number of bytes to get.
 *
 * Returned Value:
 *   The bytes of get data is returned if the peek data is successful;
 *   A negated errno value is returned on any failure.
 ****************************************************************************/

ssize_t circbuf_peekat(FAR struct circbuf_s *circ, size_t pos,
                       FAR void *dst, size_t bytes)
{
  size_t len;
  size_t off;

  DEBUGASSERT(circ);

  if (!circ->size)
    {
      return 0;
    }

  len = circbuf_used(circ);
  off = pos % circ->size;

  if (bytes > len)
    {
      bytes = len;
    }

  len = circ->size - off;
  if (bytes < len)
    {
      len = bytes;
    }

  memcpy(dst, circ->base + off, len);
  memcpy(dst + len, circ->base, bytes - len);

  return bytes;
}

/****************************************************************************
 * Name: circbuf_peek
 *
 * Description:
 *   Get data from the circular buffer without removing
 *
 * Note :
 *   That with only one concurrent reader and one concurrent writer,
 *   you don't need extra locking to use these api.
 *
 * Input Parameters:
 *   circ  - Address of the circular buffer to be used.
 *   dst   - Address where to store the data.
 *   bytes - Number of bytes to get.
 *
 * Returned Value:
 *   The bytes of get data is returned if the peek data is successful;
 *   A negated errno value is returned on any failure.
 ****************************************************************************/

ssize_t circbuf_peek(FAR struct circbuf_s *circ,
                     FAR void *dst, size_t bytes)
{
  return circbuf_peekat(circ, circ->tail, dst, bytes);
}

/****************************************************************************
 * Name: circbuf_read
 *
 * Description:
 *   Get data from the circular buffer.
 *
 * Note :
 *   That with only one concurrent reader and one concurrent writer,
 *   you don't need extra locking to use these api.
 *
 * Input Parameters:
 *   circ  - Address of the circular buffer to be used.
 *   dst   - Address where to store the data.
 *   bytes - Number of bytes to get.
 *
 * Returned Value:
 *   The bytes of get data is returned if the read data is successful;
 *   A negated errno value is returned on any failure.
 ****************************************************************************/

ssize_t circbuf_read(FAR struct circbuf_s *circ,
                     FAR void *dst, size_t bytes)
{
  DEBUGASSERT(circ);
  DEBUGASSERT(dst || !bytes);

  bytes = circbuf_peek(circ, dst, bytes);
  circ->tail += bytes;

  return bytes;
}

/****************************************************************************
 * Name: circbuf_skip
 *
 * Description:
 *   Skip data from the circular buffer.
 *
 * Note :
 *   That with only one concurrent reader and one concurrent writer,
 *   you don't need extra locking to use these api.
 *
 * Input Parameters:
 *   circ  - Address of the circular buffer to be used.
 *   bytes - Number of bytes to skip.
 *
 * Returned Value:
 *   The bytes of get data is returned if the skip data is successful;
 *   A negated errno value is returned on any failure.
 ****************************************************************************/

ssize_t circbuf_skip(FAR struct circbuf_s *circ, size_t bytes)
{
  size_t len;

  DEBUGASSERT(circ);

  len = circbuf_used(circ);

  if (bytes > len)
    {
      bytes = len;
    }

  circ->tail += bytes;

  return bytes;
}

/****************************************************************************
 * Name: circbuf_write
 *
 * Description:
 *   Write data to the circular buffer.
 *
 * Note :
 *   That with only one concurrent reader and one concurrent writer,
 *   you don't need extra locking to use these api.
 *
 * Input Parameters:
 *   circ  - Address of the circular buffer to be used.
 *   src   - The data to be added.
 *   bytes - Number of bytes to be added.
 *
 * Returned Value:
 *   The bytes of get data is returned if the write data is successful;
 *   A negated errno value is returned on any failure.
 ****************************************************************************/

ssize_t circbuf_write(FAR struct circbuf_s *circ,
                      FAR const void *src, size_t bytes)
{
  size_t space;
  size_t off;

  DEBUGASSERT(circ);
  DEBUGASSERT(src || !bytes);

  if (!circ->size)
    {
      return 0;
    }

  space = circbuf_space(circ);
  off = circ->head % circ->size;
  if (bytes > space)
    {
      bytes = space;
    }

  space = circ->size - off;
  if (bytes < space)
    {
      space = bytes;
    }

  memcpy(circ->base + off, src, space);
  memcpy(circ->base, src + space, bytes - space);
  circ->head += bytes;

  return bytes;
}

/****************************************************************************
 * Name: circbuf_overwrite
 *
 * Description:
 *   Write data to the circular buffer. It can overwrite old data when
 *   circular buffer don't have enough space to store data.
 *
 * Note:
 *   Usage circbuf_overwrite () is dangerous. It should be only called
 *   when the buffer is exclusived locked or when it is secured that no
 *   other thread is accessing the buffer.
 *
 * Input Parameters:
 *   circ  - Address of the circular buffer to be used.
 *   src   - The data to be added.
 *   bytes - Number of bytes to be added.
 *
 * Returned Value:
 *   The bytes length of overwrite is returned if it's successful;
 *   A negated errno value is returned on any failure.
 ****************************************************************************/

ssize_t circbuf_overwrite(FAR struct circbuf_s *circ,
                          FAR const void *src, size_t bytes)
{
  size_t overwrite = 0;
  size_t space;
  size_t off;

  DEBUGASSERT(circ);
  DEBUGASSERT(src || !bytes);

  if (!circ->size)
    {
      return 0;
    }

  if (bytes > circ->size)
    {
      src += bytes - circ->size;
      bytes = circ->size;
    }

  space = circbuf_space(circ);
  if (bytes > space)
    {
      overwrite = bytes - space;
    }

  off = circ->head % circ->size;
  space = circ->size - off;
  if (bytes < space)
    {
      space = bytes;
    }

  memcpy(circ->base + off, src, space);
  memcpy(circ->base, src + space, bytes - space);
  circ->head += bytes;
  circ->tail += overwrite;

  return overwrite;
}
