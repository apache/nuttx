/****************************************************************************
 * crypto/cryptodev.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author:  Max Nekludov <macscomp@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
