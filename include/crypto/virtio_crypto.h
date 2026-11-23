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
#include <string.h>
#include <sys/endian.h>

#include <crypto/asn1.h>
#include <crypto/cryptodev.h>
#include <nuttx/nuttx.h>
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

#define VIRTIO_CRYPTO_3DES_BLOCK_LEN      8
#define VIRTIO_CRYPTO_AES_BLOCK_LEN       16
#define VIRTIO_CRYPTO_CHACHA20_BLOCK_LEN  64

#define VIRTIO_CRYPTO_KEYID               0x01

/* RSA public keys:
 *  SubjectPublicKeyInfo  ::=  SEQUENCE  {          1 + 3
 *       algorithm            AlgorithmIdentifier,  1 + 1 (sequence)
 *                                                + 1 + 1 + 9 (rsa oid)
 *                                                + 1 + 1 (params null)
 *       subjectPublicKey     BIT STRING }          1 + 3 + (1 + below)
 *  RSAPublicKey ::= SEQUENCE {                     1 + 3
 *      modulus           INTEGER,  -- n            1 + 3 + MPI_MAX + 1
 *      publicExponent    INTEGER   -- e            1 + 3 + MPI_MAX + 1
 *  }
 */

#define VIRTIO_CRYPTO_RSA_PUB_DER_MAX_BYTES(bits)  (38 + 2 * (bits) / 8)

/* RSA private keys:
 *  RSAPrivateKey ::= SEQUENCE {                    1 + 3
 *      version           Version,                  1 + 1 + 1
 *      modulus           INTEGER,                  1 + 3 + MPI_MAX + 1
 *      publicExponent    INTEGER,                  1 + 3 + MPI_MAX + 1
 *      privateExponent   INTEGER,                  1 + 3 + MPI_MAX + 1
 *      prime1            INTEGER,                  1 + 3 + MPI_MAX / 2 + 1
 *      prime2            INTEGER,                  1 + 3 + MPI_MAX / 2 + 1
 *      exponent1         INTEGER,                  1 + 3 + MPI_MAX / 2 + 1
 *      exponent2         INTEGER,                  1 + 3 + MPI_MAX / 2 + 1
 *      coefficient       INTEGER,                  1 + 3 + MPI_MAX / 2 + 1
 *      otherPrimeInfos   OtherPrimeInfos OPTIONAL  0 (not supported)
 *  }
 */

#define VIRTIO_CRYPTO_RSA_MAX_SIZE_2(bits)  (((bits) / 8) / 2 +  \
                                             ((bits) / 8) % 2)
#define VIRTIO_CRYPTO_RSA_PRV_DER_MAX_BYTES(bits)              \
                                  (47 + 3 * ((bits) / 8)         \
                                  + 5 * VIRTIO_CRYPTO_RSA_MAX_SIZE_2(bits))
#define VIRTIO_CRYPTO_RSA_DER_MAX_BYTES(bits, is_priv)         \
        (is_priv ? VIRTIO_CRYPTO_RSA_PRV_DER_MAX_BYTES(bits) : \
                   VIRTIO_CRYPTO_RSA_PUB_DER_MAX_BYTES(bits))

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

static inline_function void
virtio_crypto_read_binary(FAR unsigned char **p, FAR unsigned char *mpi,
                          size_t len)
{
  FAR uint32_t *left;
  FAR uint32_t *right;
  uint32_t tmp;

  memset(mpi, 0, len);
  if (len != 0)
    {
      memcpy(mpi, *p, len);
    }

  for (left = (FAR uint32_t *)mpi,
       right = (FAR uint32_t *)(mpi + len - sizeof(*right));
       left <= right; left++, right--)
    {
      tmp    = betoh32(*left);
      *left  = betoh32(*right);
      *right = tmp;
    }

  *p += len;
}

static inline_function int
virtio_crypto_read_mpi(FAR unsigned char **pos, FAR unsigned char *end,
                       FAR unsigned char *buf, FAR struct crparam *mpi)
{
  size_t len;
  int ret;

  ret = asn1_get_tag(pos, end, &len, ASN1_INTEGER);
  if (ret != 0)
    {
      return ret;
    }

  /* Strip DER sign-extension: a leading 0x00 is only padding when the
   * next byte has MSB set.
   */

  if (len > 1 && **pos == 0 && ((*pos)[1] & 0x80) != 0)
    {
      (*pos)++;
      len--;
    }

  virtio_crypto_read_binary(pos, buf, len);
  if (len != 1 || *buf != 0)
    {
      mpi->crp_p = (caddr_t)buf;
      mpi->crp_nbits = len * 8;
    }

  return ALIGN_UP(len, 4); /* round up to aligned size */
}

/* RSAPublicKey ::= SEQUENCE {
 *     modulus           INTEGER,  -- n
 *     publicExponent    INTEGER   -- e
 * }
 */

static inline_function ssize_t
virtio_crypto_write_rsa_pubkey_der(FAR const unsigned char *n, size_t nlen,
                                   FAR const unsigned char *e, size_t elen,
                                   FAR unsigned char *buf, size_t size)
{
  FAR unsigned char *p;
  ssize_t len = 0;

  p = buf + size;

  /* Export E */

  ASN1_CHK_ADD(len, asn1_write_mpi(&p, buf, e, elen));

  /* Export N */

  ASN1_CHK_ADD(len, asn1_write_mpi(&p, buf, n, nlen));

  ASN1_CHK_ADD(len, asn1_write_len(&p, buf, len));
  ASN1_CHK_ADD(len, asn1_write_tag(&p, buf,
                                   ASN1_CONSTRUCTED | ASN1_SEQUENCE));
  return len;
}

/* RsaPrivKey ::= SEQUENCE {
 *     version     INTEGER
 *     n           INTEGER
 *     e           INTEGER
 *     d           INTEGER
 *     p           INTEGER
 *     q           INTEGER
 *     dp          INTEGER
 *     dq          INTEGER
 *     u           INTEGER
 *     otherPrimeInfos   OtherPrimeInfos OPTIONAL
 * }
 */

static inline_function ssize_t
virtio_crypto_write_rsa_key_der(FAR const unsigned char *n, size_t nlen,
                                FAR const unsigned char *e, size_t elen,
                                FAR const unsigned char *d, size_t dlen,
                                FAR const unsigned char *p, size_t plen,
                                FAR const unsigned char *q, size_t qlen,
                                FAR const unsigned char *dp, size_t dplen,
                                FAR const unsigned char *dq, size_t dqlen,
                                FAR const unsigned char *qp, size_t qplen,
                                FAR unsigned char *buf, size_t size)
{
  FAR unsigned char *pos;
  ssize_t len = 0;

  pos = buf + size;

  /* Export QP */

  ASN1_CHK_ADD(len, asn1_write_mpi(&pos, buf, qp, qplen));

  /* Export DQ */

  ASN1_CHK_ADD(len, asn1_write_mpi(&pos, buf, dq, dqlen));

  /* Export DP */

  ASN1_CHK_ADD(len, asn1_write_mpi(&pos, buf, dp, dplen));

  /* Export Q */

  ASN1_CHK_ADD(len, asn1_write_mpi(&pos, buf, q, qlen));

  /* Export P */

  ASN1_CHK_ADD(len, asn1_write_mpi(&pos, buf, p, plen));

  /* Export D */

  ASN1_CHK_ADD(len, asn1_write_mpi(&pos, buf, d, dlen));

  /* Export E */

  ASN1_CHK_ADD(len, asn1_write_mpi(&pos, buf, e, elen));

  /* Export N */

  ASN1_CHK_ADD(len, asn1_write_mpi(&pos, buf, n, nlen));

  ASN1_CHK_ADD(len, asn1_write_int(&pos, buf, 0));
  ASN1_CHK_ADD(len, asn1_write_len(&pos, buf, len));
  ASN1_CHK_ADD(len, asn1_write_tag(&pos, buf,
                                   ASN1_CONSTRUCTED | ASN1_SEQUENCE));
  return len;
}

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
int virtio_crypto_get_blocksize(int alg)
{
  switch (alg)
    {
      case CRYPTO_3DES_CBC:
        return VIRTIO_CRYPTO_3DES_BLOCK_LEN;
      case CRYPTO_AES_CBC:
      case CRYPTO_AES_192_CBC:
      case CRYPTO_AES_256_CBC:
      case CRYPTO_AES_CTR:
      case CRYPTO_AES_XTS:
      case CRYPTO_AES_OFB:
      case CRYPTO_AES_CFB_8:
      case CRYPTO_AES_CFB_128:
      case CRYPTO_AES_GCM_16:
        return VIRTIO_CRYPTO_AES_BLOCK_LEN;
      case CRYPTO_CHACHA20:
      case CRYPTO_CHACHA20_POLY1305:
        return VIRTIO_CRYPTO_CHACHA20_BLOCK_LEN;
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
      case CRYPTO_CHACHA20:
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
      case CRYPTO_SHA2_224_HMAC:
      case CRYPTO_SHA2_256_HMAC:
      case CRYPTO_SHA2_384_HMAC:
      case CRYPTO_SHA2_512_HMAC:
      case CRYPTO_AES_CMAC:
      case CRYPTO_AES_128_CMAC:
      case CRYPTO_AES_256_CMAC:
      case CRYPTO_PBKDF2_HMAC_SHA1:
      case CRYPTO_PBKDF2_HMAC_SHA256:
        return VIRTIO_CRYPTO_SERVICE_MAC;
      case CRYPTO_AES_GCM_16:
      case CRYPTO_AES_128_GMAC:
      case CRYPTO_AES_192_GMAC:
      case CRYPTO_AES_256_GMAC:
      case CRYPTO_CHACHA20_POLY1305:
      case CRYPTO_CHACHA20_POLY1305_MAC:
        return VIRTIO_CRYPTO_SERVICE_AEAD;
    }

  return -EINVAL;
}

static inline_function
int virtio_crypto_asym_get_alg(int kalg)
{
  switch (kalg)
    {
      case CRK_MOD_EXP:
      case CRK_MOD_EXP_CRT:
      case CRK_RSA_PKCS15_SIGN:
      case CRK_RSA_PKCS15_VERIFY:
      case CRK_RSA_PSS_SIGN:
      case CRK_RSA_PSS_VERIFY:
        return VIRTIO_CRYPTO_AKCIPHER_RSA;
      case CRK_ECDSA_SECP256R1_SIGN:
      case CRK_ECDSA_SECP256R1_VERIFY:
      case CRK_ECDSA_SECP256R1_GENKEY:
        return VIRTIO_CRYPTO_AKCIPHER_ECDSA;
      case CRK_ECDH_GEN_PUBLIC:
      case CRK_ECDH_COMPUTE_SHARED:
        return VIRTIO_CRYPTO_AKCIPHER_ECDH;
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
      case CRYPTO_CHACHA20:
        return VIRTIO_CRYPTO_CIPHER_CHACHA20;
      case CRYPTO_MD5_HMAC:
        return VIRTIO_CRYPTO_MAC_HMAC_MD5;
      case CRYPTO_SHA1_HMAC:
        return VIRTIO_CRYPTO_MAC_HMAC_SHA1;
      case CRYPTO_SHA2_224_HMAC:
        return VIRTIO_CRYPTO_MAC_HMAC_SHA_224;
      case CRYPTO_PBKDF2_HMAC_SHA1:
        return VIRTIO_CRYPTO_MAC_PBKDF2_HMAC_SHA1;
      case CRYPTO_PBKDF2_HMAC_SHA256:
        return VIRTIO_CRYPTO_MAC_PBKDF2_HMAC_SHA256;
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
      case CRYPTO_AES_192_GMAC:
      case CRYPTO_AES_256_GMAC:
        return VIRTIO_CRYPTO_MAC_GMAC_AES;
      case CRYPTO_AES_CMAC:
      case CRYPTO_AES_128_CMAC:
      case CRYPTO_AES_256_CMAC:
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
    case VIRTIO_CRYPTO_CIPHER_CHACHA20:
      return CRYPTO_CHACHA20;
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
    case VIRTIO_CRYPTO_MAC_HMAC_SHA_224:
      return CRYPTO_SHA2_224_HMAC;
    case VIRTIO_CRYPTO_MAC_PBKDF2_HMAC_SHA1:
      return CRYPTO_PBKDF2_HMAC_SHA1;
    case VIRTIO_CRYPTO_MAC_PBKDF2_HMAC_SHA256:
      return CRYPTO_PBKDF2_HMAC_SHA256;
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

static inline_function
int virtio_crypto_get_aead_alg(int alg)
{
  switch (alg)
  {
    case VIRTIO_CRYPTO_AEAD_GCM:
      return CRYPTO_AES_GCM_16;
    case VIRTIO_CRYPTO_AEAD_CHACHA20_POLY1305:
      return CRYPTO_CHACHA20_POLY1305;
  }

  return -EINVAL;
}

static inline_function
int virtio_crypto_ecdsa_get_curve(int kalg)
{
  switch (kalg)
    {
      case CRK_ECDSA_SECP256R1_SIGN:
      case CRK_ECDSA_SECP256R1_VERIFY:
      case CRK_ECDSA_SECP256R1_GENKEY:
      case CRK_ECDH_GEN_PUBLIC:
      case CRK_ECDH_COMPUTE_SHARED:
        return VIRTIO_CRYPTO_CURVE_NIST_P256;
    }

  return -EINVAL;
}

/* Per-curve field element length in bytes, per the virtio-crypto spec's
 * VIRTIO_CRYPTO_CURVE_* IDs. Only NIST_P256 has a backing cryptodev
 * implementation today; the rest are wired up so adding a new curve is
 * a matter of adding a case here plus the cryptodev/cryptosoft support,
 * not hunting down every hardcoded 32.
 */

static inline_function
int virtio_crypto_curve_get_len(uint32_t curve_id)
{
  switch (curve_id)
  {
    case VIRTIO_CRYPTO_CURVE_NIST_P192:
      return 24;
    case VIRTIO_CRYPTO_CURVE_NIST_P224:
      return 28;
    case VIRTIO_CRYPTO_CURVE_NIST_P256:
      return 32;
    case VIRTIO_CRYPTO_CURVE_NIST_P384:
      return 48;
    case VIRTIO_CRYPTO_CURVE_NIST_P521:
      return 66;
  }

  return -EINVAL;
}

#endif /* __INCLUDE_CRYPTO_VIRTIO_CRYPTO_H */
