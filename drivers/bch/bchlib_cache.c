/****************************************************************************
 * drivers/bch/bchlib_cache.c
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

#include <sys/types.h>
#include <stdbool.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include "bch.h"

#if defined(CONFIG_BCH_ENCRYPTION)
#  include <nuttx/crypto/crypto.h>
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bch_xor
 ****************************************************************************/

#if defined(CONFIG_BCH_ENCRYPTION)
static void bch_xor(uint32_t *R, uint32_t *A, uint32_t *B)
{
  R[0] = A[0] ^ B[0];
  R[1] = A[1] ^ B[1];
  R[2] = A[2] ^ B[2];
  R[3] = A[3] ^ B[3];
}
#endif

/****************************************************************************
 * Name: bch_cypher
 ****************************************************************************/

#if defined(CONFIG_BCH_ENCRYPTION)
static int bch_cypher(FAR struct bchlib_s *bch, int encrypt)
{
  int blocks = bch->sectsize / 16;
  FAR uint32_t *buffer = (FAR uint32_t *)bch->buffer;
  int i;

  for (i = 0; i < blocks; i++, buffer += 16 / sizeof(uint32_t) )
    {
      uint32_t T[4];
      uint32_t X[4] =
      {
        bch->sector, 0, 0, i
      };

      aes_cypher(X, X, 16, NULL, bch->key, CONFIG_BCH_ENCRYPTION_KEY_SIZE,
                 AES_MODE_ECB, CYPHER_ENCRYPT);

      /* Xor-Encrypt-Xor */

      bch_xor(T, X, buffer);
      aes_cypher(T, T, 16, NULL, bch->key, CONFIG_BCH_ENCRYPTION_KEY_SIZE,
                 AES_MODE_ECB, encrypt);
      bch_xor(buffer, X, T);
    }

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bchlib_flushsector
 *
 * Description:
 *   Flush the current contents of the sector buffer (if dirty)
 *
 * Assumptions:
 *   Caller must assume mutual exclusion
 *
 ****************************************************************************/

int bchlib_flushsector(FAR struct bchlib_s *bch)
{
  FAR struct inode *inode;
  ssize_t ret = OK;

  /* Check if the sector has been modified and is out of synch with the
   * media.
   */

  if (bch->dirty)
    {
      inode = bch->inode;

#if defined(CONFIG_BCH_ENCRYPTION)
      /* Encrypt data as necessary */

      bch_cypher(bch, CYPHER_ENCRYPT);
#endif

      /* Write the sector to the media */

      ret = inode->u.i_bops->write(inode, bch->buffer, bch->sector, 1);
      if (ret < 0)
        {
          ferr("Write failed: %zd\n", ret);
          return (int)ret;
        }

#if defined(CONFIG_BCH_ENCRYPTION)
      /* Computation overhead to save memory for extra sector buffer
       * TODO: Add configuration switch for extra sector buffer
       */

      bch_cypher(bch, CYPHER_DECRYPT);
#endif

      /* The sector is now in sync with the media */

      bch->dirty = false;
    }

  return (int)ret;
}

/****************************************************************************
 * Name: bchlib_readsector
 *
 * Description:
 *   Flush the current contents of the sector buffer (if dirty)
 *
 * Assumptions:
 *   Caller must assume mutual exclusion
 *
 ****************************************************************************/

int bchlib_readsector(FAR struct bchlib_s *bch, size_t sector)
{
  FAR struct inode *inode;
  ssize_t ret = OK;

  if (bch->sector != sector)
    {
      inode = bch->inode;

      ret = bchlib_flushsector(bch);
      if (ret < 0)
        {
          ferr("Flush failed: %zd\n", ret);
          return (int)ret;
        }

      bch->sector = (size_t)-1;

      ret = inode->u.i_bops->read(inode, bch->buffer, sector, 1);
      if (ret < 0)
        {
          ferr("Read failed: %zd\n", ret);
          return (int)ret;
        }

      bch->sector = sector;
#if defined(CONFIG_BCH_ENCRYPTION)
      bch_cypher(bch, CYPHER_DECRYPT);
#endif
    }

  return (int)ret;
}
