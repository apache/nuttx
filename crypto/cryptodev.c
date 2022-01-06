/****************************************************************************
 * crypto/cryptodev.c
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
#include <string.h>
#include <poll.h>
#include <errno.h>

#include <nuttx/fs/fs.h>
#include <nuttx/drivers/drivers.h>

#include <nuttx/crypto/crypto.h>
#include <nuttx/crypto/cryptodev.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_CRYPTO_AES
#  define AES_CYPHER(mode) \
  aes_cypher(op->dst, op->src, op->len, op->iv, ses->key, ses->keylen, \
             mode, encrypt)
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods */

static ssize_t cryptodev_read(FAR struct file *filep,
                              FAR char *buffer,
                              size_t len);
static ssize_t cryptodev_write(FAR struct file *filep,
                               FAR const char *buffer,
                               size_t len);
static int cryptodev_ioctl(FAR struct file *filep,
                           int cmd,
                           unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_cryptodevops =
{
  NULL,               /* open   */
  NULL,               /* close  */
  cryptodev_read,     /* read   */
  cryptodev_write,    /* write  */
  NULL,               /* seek   */
  cryptodev_ioctl,    /* ioctl  */
  NULL                /* poll   */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL              /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static ssize_t cryptodev_read(FAR struct file *filep,
                              FAR char *buffer,
                              size_t len)
{
  return -EACCES;
}

static ssize_t cryptodev_write(FAR struct file *filep,
                               FAR const char *buffer,
                               size_t len)
{
  return -EACCES;
}

static int cryptodev_ioctl(FAR struct file *filep,
                           int cmd,
                           unsigned long arg)
{
  switch (cmd)
  {
  case CIOCGSESSION:
    {
      FAR struct session_op *ses = (FAR struct session_op *)arg;
      ses->ses = (uint32_t)ses;
      return OK;
    }

  case CIOCFSESSION:
    {
      return OK;
    }

#ifdef CONFIG_CRYPTO_AES
  case CIOCCRYPT:
    {
      FAR struct crypt_op *op    = (FAR struct crypt_op *)arg;
      FAR struct session_op *ses = (FAR struct session_op *)op->ses;
      int encrypt;

      switch (op->op)
        {
        case COP_ENCRYPT:
          encrypt = 1;
          break;

        case COP_DECRYPT:
          encrypt = 0;
          break;

        default:
          return -EINVAL;
        }

      switch (ses->cipher)
        {
        case CRYPTO_AES_ECB:
          return AES_CYPHER(AES_MODE_ECB);

        case CRYPTO_AES_CBC:
          return AES_CYPHER(AES_MODE_CBC);

        case CRYPTO_AES_CTR:
          return AES_CYPHER(AES_MODE_CTR);

        default:
           return -EINVAL;
        }
    }
#endif

  default:
    return -ENOTTY;
  }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void devcrypto_register(void)
{
  register_driver("/dev/crypto", &g_cryptodevops, 0666, NULL);
}
