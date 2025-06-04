/****************************************************************************
 * include/crypto/virtio_crypto.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __INCLUDE_CRYPTO_VIRTIO_CRYPTO_H
#define __INCLUDE_CRYPTO_VIRTIO_CRYPTO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <errno.h>
#include <crypto/cryptodev.h>
#include <nuttx/virtio/virtio-crypto.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VIRTIO_CRYPTO_MD5_HASH_LEN        16
#define VIRTIO_CRYPTO_SHA1_HASH_LEN       20
#define VIRTIO_CRYPTO_SHA224_HASH_LEN     28
#define VIRTIO_CRYPTO_SHA256_HASH_LEN     32
#define VIRTIO_CRYPTO_SHA384_HASH_LEN     48
#define VIRTIO_CRYPTO_SHA512_HASH_LEN     64
#define VIRTIO_CRYPTO_AESCMAC_KEY_LEN     16
#define VIRTIO_CRYPTO_AES_IV_LEN          16
#define VIRTIO_CRYPTO_CCM_IV_LEN          12
#define VIRTIO_CRYPTO_CHACHAPOLY_IV_LEN   12
#define VIRTIO_CRYPTO_DES_IV_LEN          8
#define VIRTIO_CRYPTO_GCM_IV_LEN          12
#define VIRTIO_CRYPTO_CCM_TAG_LEN         16
#define VIRTIO_CRYPTO_GCM_TAG_LEN         16
#define VIRTIO_CRYPTO_CHACHAPOLY_TAG_LEN  16
#define VIRTIO_CRYPTO_NOUNCE_LEN          4

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

static inline_function
int virtio_crypto_get_errcode(int verr)
{
  switch (verr)
  {
    case VIRTIO_CRYPTO_OK:
      return OK;
    case VIRTIO_CRYPTO_ERR:
      return -ENOSYS;
    case VIRTIO_CRYPTO_BADMSG:
      return -EBADMSG;
    case VIRTIO_CRYPTO_NOTSUPP:
      return -ENOTSUP;
    case VIRTIO_CRYPTO_INVSESS:
      return -EBADF;
    case VIRTIO_CRYPTO_NOSPC:
      return -ENOSPC;
    case VIRTIO_CRYPTO_KEY_REJECTED:
      return -EKEYREJECTED;
  }

  return -EINVAL;
}

static inline_function
int virtio_crypto_get_tag_len(int valg)
{
  switch (valg)
  {
    case VIRTIO_CRYPTO_AEAD_GCM:
      return VIRTIO_CRYPTO_GCM_TAG_LEN;
    case VIRTIO_CRYPTO_AEAD_CCM:
      return VIRTIO_CRYPTO_CCM_TAG_LEN;
    case VIRTIO_CRYPTO_AEAD_CHACHA20_POLY1305:
      return VIRTIO_CRYPTO_CHACHAPOLY_TAG_LEN;
  }

  return -EINVAL;
}

static inline_function
int virtio_crypto_get_iv_len(int vservice, int valg)
{
  if (vservice == VIRTIO_CRYPTO_SERVICE_CIPHER)
    {
      switch (valg)
      {
        case VIRTIO_CRYPTO_CIPHER_3DES_CBC:
          return VIRTIO_CRYPTO_DES_IV_LEN;
        case VIRTIO_CRYPTO_CIPHER_AES_CBC:
        case VIRTIO_CRYPTO_CIPHER_AES_CTR:
        case VIRTIO_CRYPTO_CIPHER_AES_XTS:
          return VIRTIO_CRYPTO_AES_IV_LEN;
      }
    }
  else if (vservice = VIRTIO_CRYPTO_SERVICE_AEAD)
    {
      switch (valg)
        {
          case VIRTIO_CRYPTO_AEAD_GCM:
            return VIRTIO_CRYPTO_GCM_IV_LEN;
          case VIRTIO_CRYPTO_AEAD_CCM:
            return VIRTIO_CRYPTO_CCM_IV_LEN;
          case VIRTIO_CRYPTO_AEAD_CHACHA20_POLY1305:
            return VIRTIO_CRYPTO_CHACHAPOLY_IV_LEN;
        }
    }

  return 0;
}

static inline_function
int virtio_crypto_get_auth_key_len(int valg)
{
  switch (valg)
    {
      case VIRTIO_CRYPTO_HASH_MD5:
        return VIRTIO_CRYPTO_MD5_HASH_LEN;
      case VIRTIO_CRYPTO_HASH_SHA1:
        return VIRTIO_CRYPTO_SHA1_HASH_LEN;
      case VIRTIO_CRYPTO_HASH_SHA_224:
        return VIRTIO_CRYPTO_SHA224_HASH_LEN;
      case VIRTIO_CRYPTO_HASH_SHA_256:
        return VIRTIO_CRYPTO_SHA256_HASH_LEN;
      case VIRTIO_CRYPTO_HASH_SHA_384:
        return VIRTIO_CRYPTO_SHA384_HASH_LEN;
      case VIRTIO_CRYPTO_HASH_SHA_512:
        return VIRTIO_CRYPTO_SHA512_HASH_LEN;
      case VIRTIO_CRYPTO_MAC_CMAC_AES:
        return VIRTIO_CRYPTO_AESCMAC_KEY_LEN;
    }

  return -EINVAL;
}

static inline_function
int virtio_crypto_get_hash_result_len(int valg)
{
  switch (valg)
    {
      case VIRTIO_CRYPTO_HASH_MD5:
        return VIRTIO_CRYPTO_MD5_HASH_LEN;
      case VIRTIO_CRYPTO_HASH_SHA1:
        return VIRTIO_CRYPTO_SHA1_HASH_LEN;
      case VIRTIO_CRYPTO_HASH_SHA_224:
        return VIRTIO_CRYPTO_SHA224_HASH_LEN;
      case VIRTIO_CRYPTO_HASH_SHA_256:
        return VIRTIO_CRYPTO_SHA256_HASH_LEN;
      case VIRTIO_CRYPTO_HASH_SHA_384:
        return VIRTIO_CRYPTO_SHA384_HASH_LEN;
      case VIRTIO_CRYPTO_HASH_SHA_512:
        return VIRTIO_CRYPTO_SHA512_HASH_LEN;
    }

  return -EINVAL;
}

static inline_function
int virtio_crypto_get_service(int alg)
{
  switch (alg)
    {
      case CRYPTO_3DES_CBC:
      case CRYPTO_BLF_CBC:
      case CRYPTO_CAST_CBC:
      case CRYPTO_AES_CBC:
      case CRYPTO_AES_192_CBC:
      case CRYPTO_AES_256_CBC:
      case CRYPTO_AES_CTR:
      case CRYPTO_AES_XTS:
      case CRYPTO_AES_OFB:
      case CRYPTO_AES_CFB_8:
      case CRYPTO_AES_CFB_128:
        return VIRTIO_CRYPTO_SERVICE_CIPHER;
      case CRYPTO_MD5:
      case CRYPTO_POLY1305:
      case CRYPTO_RIPEMD160:
      case CRYPTO_SHA1:
      case CRYPTO_SHA2_224:
      case CRYPTO_SHA2_256:
      case CRYPTO_SHA2_384:
      case CRYPTO_SHA2_512:
      case CRYPTO_CRC32:
        return VIRTIO_CRYPTO_SERVICE_HASH;
      case CRYPTO_MD5_HMAC:
      case CRYPTO_SHA1_HMAC:
      case CRYPTO_RIPEMD160_HMAC:
      case CRYPTO_SHA2_256_HMAC:
      case CRYPTO_SHA2_384_HMAC:
      case CRYPTO_SHA2_512_HMAC:
      case CRYPTO_AES_128_GMAC:
      case CRYPTO_AES_CMAC:
      case CRYPTO_AES_128_CMAC:
        return VIRTIO_CRYPTO_SERVICE_MAC;
      case CRYPTO_AES_GCM_16:
      case CRYPTO_CHACHA20_POLY1305:
      case CRYPTO_CHACHA20_POLY1305_MAC:
        return VIRTIO_CRYPTO_SERVICE_AEAD;
    }

  return -EINVAL;
}

static inline_function
int virtio_crypto_get_alg(int alg)
{
  switch (alg)
    {
      case CRYPTO_3DES_CBC:
        return VIRTIO_CRYPTO_CIPHER_3DES_CBC;
      case CRYPTO_AES_CBC:
      case CRYPTO_AES_192_CBC:
      case CRYPTO_AES_256_CBC:
        return VIRTIO_CRYPTO_CIPHER_AES_CBC;
      case CRYPTO_AES_CTR:
        return VIRTIO_CRYPTO_CIPHER_AES_CTR;
      case CRYPTO_AES_XTS:
        return VIRTIO_CRYPTO_CIPHER_AES_XTS;
      case CRYPTO_MD5_HMAC:
        return VIRTIO_CRYPTO_MAC_HMAC_MD5;
      case CRYPTO_SHA1_HMAC:
        return VIRTIO_CRYPTO_MAC_HMAC_SHA1;
      case CRYPTO_SHA2_256_HMAC:
        return VIRTIO_CRYPTO_MAC_HMAC_SHA_256;
      case CRYPTO_SHA2_384_HMAC:
        return VIRTIO_CRYPTO_MAC_HMAC_SHA_384;
      case CRYPTO_SHA2_512_HMAC:
        return VIRTIO_CRYPTO_MAC_HMAC_SHA_512;
      case CRYPTO_MD5:
        return VIRTIO_CRYPTO_HASH_MD5;
      case CRYPTO_SHA1:
        return VIRTIO_CRYPTO_HASH_SHA1;
      case CRYPTO_SHA2_224:
        return VIRTIO_CRYPTO_HASH_SHA_224;
      case CRYPTO_SHA2_256:
        return VIRTIO_CRYPTO_HASH_SHA_256;
      case CRYPTO_SHA2_384:
        return VIRTIO_CRYPTO_HASH_SHA_384;
      case CRYPTO_SHA2_512:
        return VIRTIO_CRYPTO_HASH_SHA_512;
      case CRYPTO_AES_128_GMAC:
        return VIRTIO_CRYPTO_MAC_GMAC_AES;
      case CRYPTO_AES_CMAC:
      case CRYPTO_AES_128_CMAC:
        return VIRTIO_CRYPTO_MAC_CMAC_AES;
      case CRYPTO_AES_GCM_16:
        return VIRTIO_CRYPTO_AEAD_GCM;
      case CRYPTO_CHACHA20_POLY1305:
      case CRYPTO_CHACHA20_POLY1305_MAC:
        return VIRTIO_CRYPTO_AEAD_CHACHA20_POLY1305;
    }

  return -EINVAL;
}

static inline_function
int virtio_crypto_get_cipher_alg(int alg)
{
  switch (alg)
  {
    case VIRTIO_CRYPTO_CIPHER_3DES_CBC:
      return CRYPTO_3DES_CBC;
    case VIRTIO_CRYPTO_CIPHER_AES_CBC:
      return CRYPTO_AES_CBC;
    case VIRTIO_CRYPTO_CIPHER_AES_CTR:
      return CRYPTO_AES_CTR;
    case VIRTIO_CRYPTO_CIPHER_AES_XTS:
      return CRYPTO_AES_XTS;
  }

  return -EINVAL;
}

static inline_function
int virtio_crypto_get_hash_alg(int alg)
{
  switch (alg)
  {
    case VIRTIO_CRYPTO_HASH_MD5:
      return CRYPTO_MD5;
    case VIRTIO_CRYPTO_HASH_SHA1:
      return CRYPTO_SHA1;
    case VIRTIO_CRYPTO_HASH_SHA_224:
      return CRYPTO_SHA2_224;
    case VIRTIO_CRYPTO_HASH_SHA_256:
      return CRYPTO_SHA2_256;
    case VIRTIO_CRYPTO_HASH_SHA_384:
      return CRYPTO_SHA2_384;
    case VIRTIO_CRYPTO_HASH_SHA_512:
      return CRYPTO_SHA2_512;
  }

  return -EINVAL;
}

static inline_function
int virtio_crypto_get_mac_alg(int alg)
{
  switch (alg)
  {
    case VIRTIO_CRYPTO_MAC_HMAC_MD5:
      return CRYPTO_MD5_HMAC;
    case VIRTIO_CRYPTO_MAC_HMAC_SHA1:
      return CRYPTO_SHA1_HMAC;
    case VIRTIO_CRYPTO_MAC_HMAC_SHA_256:
      return CRYPTO_SHA2_256_HMAC;
    case VIRTIO_CRYPTO_MAC_HMAC_SHA_384:
      return CRYPTO_SHA2_384_HMAC;
    case VIRTIO_CRYPTO_MAC_HMAC_SHA_512:
      return CRYPTO_SHA2_512_HMAC;
    case VIRTIO_CRYPTO_MAC_CMAC_AES:
      return CRYPTO_AES_128_CMAC;
  }

  return -EINVAL;
}

#endif /* __INCLUDE_CRYPTO_VIRTIO_CRYPTO_H */
