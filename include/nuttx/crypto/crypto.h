/****************************************************************************
 * include/nuttx/crypto/crypto.h
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

#ifndef __INCLUDE_NUTTX_CRYPTO_CRYPTO_H
#define __INCLUDE_NUTTX_CRYPTO_CRYPTO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_CRYPTO_AES)
#  define AES_MODE_MIN 1

#  define AES_MODE_ECB 1
#  define AES_MODE_CBC 2
#  define AES_MODE_CTR 3
#  define AES_MODE_CFB 4

#  define AES_MODE_MAX 4

#  define AES_MODE_MAC 0x80000000

#  define AES_MODE_MASK 0xffff
#endif

#define CYPHER_ENCRYPT 1
#define CYPHER_DECRYPT 0

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int up_cryptoinitialize(void);

#if defined(CONFIG_CRYPTO_AES)
int aes_cypher(FAR void *out, FAR const void *in, size_t size,
               FAR const void *iv, FAR const void *key, size_t keysize,
               int mode, int encrypt);
#endif

#if defined(CONFIG_CRYPTO_ALGTEST)
int crypto_test(void);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __INCLUDE_NUTTX_CRYPTO_CRYPTO_H */
