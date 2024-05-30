/****************************************************************************
 * libs/libc/unistd/lib_crypt_r.c
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

#include <crypto/cryptodev.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define KEY_MAX 30000
#define SALT_MAX 8

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef struct cryptodev_context_s
{
  int fd;
  struct session_op session;
  struct crypt_op crypt;
}cryptodev_context_t;

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* 0 ... 63 => ascii - 64 */

static const unsigned char g_md5_itoa64[] =
  "./0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static FAR char *md5_to64(FAR char *s, unsigned long v, int n)
{
  while (--n >= 0)
    {
      *s++ = g_md5_itoa64[v & 0x3f];
      v >>= 6;
    }

  return s;
}

static int md5_init(FAR cryptodev_context_t *ctx)
{
  int ret;
  int fd;

  memset(ctx, 0, sizeof(cryptodev_context_t));
  fd = open("/dev/crypto", O_RDWR, 0);
  if (fd < 0)
    {
      return -get_errno();
    }

  ret = ioctl(fd, CRIOGET, &ctx->fd);
  close(fd);
  if (ret < 0)
    {
      return -get_errno();
    }

  ctx->session.mac = CRYPTO_MD5;
  ret = ioctl(ctx->fd, CIOCGSESSION, &ctx->session);
  if (ret < 0)
    {
      return -get_errno();
    }

  ctx->crypt.ses = ctx->session.ses;
  return ret;
}

static int md5_update(FAR cryptodev_context_t *ctx,
                      FAR const void *input, size_t ilen)
{
  int ret;
  ctx->crypt.op = COP_ENCRYPT;
  ctx->crypt.flags |= COP_FLAG_UPDATE;
  ctx->crypt.src = (caddr_t)input;
  ctx->crypt.len = ilen;

  ret = ioctl(ctx->fd, CIOCCRYPT, &ctx->crypt);
  return ret < 0 ? -get_errno() : ret;
}

static int md5_finish(FAR cryptodev_context_t *ctx,
                      unsigned char output[16])
{
  int ret;

  ctx->crypt.op = COP_ENCRYPT;
  ctx->crypt.flags = 0;
  ctx->crypt.mac = (caddr_t)output;
  ret = ioctl(ctx->fd, CIOCCRYPT, &ctx->crypt);
  if (ret < 0)
    {
      return -get_errno();
    }

  ioctl(ctx->fd, CIOCFSESSION, &ctx->session.ses);
  ctx->crypt.ses = 0;
  close(ctx->fd);
  memset(ctx, 0, sizeof(cryptodev_context_t));
  return ret;
}

/* UNIX password
 * Use MD5 for what it is best at...
 */

static FAR char *md5_crypt(FAR const char *key, FAR const char *setting,
                           FAR char *output)
{
  cryptodev_context_t ctx;
  unsigned char md[16];
  unsigned int i;
  unsigned int klen;
  unsigned int slen;
  FAR const char *salt;
  FAR char *p;
  static const unsigned char perm[][3] =
  {
    {
      0, 6, 12
    },
    {
      1, 7, 13
    },
    {
      2, 8, 14
    },
    {
      3, 9, 15
    },
    {
      4, 10, 5
    }
  };

  /* Reject large keys */

  klen = strnlen(key, KEY_MAX + 1);
  if (klen > KEY_MAX)
    {
      return 0;
    }

  /* Setting: $1$salt$ (closing $ is optional) */

  salt = setting + 3;
  for (i = 0; i < SALT_MAX && salt[i] && salt[i] != '$'; i++);
  slen = i;

  /* Md5(key salt key) */

  md5_init(&ctx);
  md5_update(&ctx, key, klen);
  md5_update(&ctx, salt, slen);
  md5_update(&ctx, key, klen);
  md5_finish(&ctx, md);

  /* Md5(key $1$ salt repeated-md weird-key[0]-0) */

  md5_init(&ctx);
  md5_update(&ctx, key, klen);
  md5_update(&ctx, setting, 3 + slen);
  for (i = klen; i > sizeof md; i -= sizeof md)
    {
      md5_update(&ctx, md, sizeof md);
    }

  md5_update(&ctx, md, i);
  md[0] = 0;
  for (i = klen; i; i >>= 1)
    {
      if (i & 1)
        {
          md5_update(&ctx, md, 1);
        }
      else
        {
          md5_update(&ctx, key, 1);
        }
    }

  md5_finish(&ctx, md);

  for (i = 0; i < 1000; i++)
    {
      md5_init(&ctx);
      if (i % 2)
        {
          md5_update(&ctx, key, klen);
        }
      else
        {
          md5_update(&ctx, md, sizeof md);
        }

      if (i % 3)
        {
          md5_update(&ctx, salt, slen);
        }

      if (i % 7)
        {
          md5_update(&ctx, key, klen);
        }

      if (i % 2)
        {
          md5_update(&ctx, md, sizeof md);
        }
      else
        {
          md5_update(&ctx, key, klen);
        }

      md5_finish(&ctx, md);
    }

  /* Output is $1$salt$hash */

  memcpy(output, setting, 3 + slen);
  p = output + 3 + slen;
  *p++ = '$';

  for (i = 0; i < 5; i++)
    {
      p = md5_to64(p, (md[perm[i][0]] << 16) |
              (md[perm[i][1]] << 8) | md[perm[i][2]], 4);
    }

  p = md5_to64(p, md[11], 2);
  *p = 0;

  return output;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR char *crypt_r(FAR const char *key, FAR const char *salt,
                  FAR char *output)
{
  /* First, check if we are supposed to be using the MD5
   * not support DES...
   */

  if (salt[0] == '$' && salt[1] == '1' && salt[2] == '$')
    {
      return md5_crypt(key, salt, output);
    }
  else
    {
      return NULL;
    }
}
