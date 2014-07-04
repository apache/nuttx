/****************************************************************************
 * include/nuttx/crypto/cryptodev.h
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

#ifndef __INCLUDE_NUTTX_CRYPTO_CRYPTODEV_H
#define __INCLUDE_NUTTX_CRYPTO_CRYPTODEV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

#define RIJNDAEL128_BLOCK_LEN   16
#define AES_BLOCK_LEN           RIJNDAEL128_BLOCK_LEN

#define CRYPTO_ALGORITHM_MIN    1
#define CRYPTO_AES_ECB          1
#define CRYPTO_AES_CBC          2
#define CRYPTO_AES_CTR          3
#define CRYPTO_ALGORITHM_MAX    1

#define CRYPTO_FLAG_HARDWARE    0x01000000 /* hardware accelerated */
#define CRYPTO_FLAG_SOFTWARE    0x02000000 /* software implementation */

#define COP_ENCRYPT             1
#define COP_DECRYPT             2
#define COP_F_BATCH             0x0008 /* Batch op if possible */

#define CIOCGSESSION            101
#define CIOCFSESSION            102
#define CIOCCRYPT               103

typedef char* caddr_t;

struct session_op
{
  uint32_t cipher;    /* ie. CRYPTO_AES_EBC */
  uint32_t mac;

  uint32_t keylen;    /* cipher key */
  caddr_t key;
  int mackeylen;      /* mac key */
  caddr_t mackey;

  uint32_t ses;       /* returns: session # */
};

struct crypt_op
{
  uint32_t ses;
  uint16_t op;        /* i.e. COP_ENCRYPT */
  uint16_t flags;
  unsigned len;
  caddr_t src, dst;   /* become iov[] inside kernel */
  caddr_t mac;        /* must be big enough for chosen MAC */
  caddr_t iv;
};

#endif /* __INCLUDE_NUTTX_CRYPTO_CRYPTODEV_H */
