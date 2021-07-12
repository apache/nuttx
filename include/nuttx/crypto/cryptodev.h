/****************************************************************************
 * include/nuttx/crypto/cryptodev.h
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

#ifndef __INCLUDE_NUTTX_CRYPTO_CRYPTODEV_H
#define __INCLUDE_NUTTX_CRYPTO_CRYPTODEV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
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

typedef char *caddr_t;

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
