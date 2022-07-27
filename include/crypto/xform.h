/****************************************************************************
 * include/crypto/xform.h
 * $OpenBSD: xform.h,v 1.32 2021/10/22 12:30:53 bluhm Exp $
 *
 * The author of this code is Angelos D. Keromytis (angelos@cis.upenn.edu)
 *
 * This code was written by Angelos D. Keromytis in Athens, Greece, in
 * February 2000. Network Security Technologies Inc. (NSTI) kindly
 * supported the development of this code.
 *
 * Copyright (c) 2000 Angelos D. Keromytis
 *
 * Permission to use, copy, and modify this software with or without fee
 * is hereby granted, provided that this entire notice is included in
 * all source code copies of any software which is or includes a copy or
 * modification of this software.
 *
 * THIS SOFTWARE IS BEING PROVIDED "AS IS", WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTY. IN PARTICULAR, NONE OF THE AUTHORS MAKES ANY
 * REPRESENTATION OR WARRANTY OF ANY KIND CONCERNING THE
 * MERCHANTABILITY OF THIS SOFTWARE OR ITS FITNESS FOR ANY PARTICULAR
 * PURPOSE.
 ****************************************************************************/

#ifndef __INCLUDE_CRYPTO_XFORM_H
#define __INCLUDE_CRYPTO_XFORM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <crypto/md5.h>
#include <crypto/sha1.h>
#include <crypto/rmd160.h>
#include <crypto/sha2.h>
#include <crypto/gmac.h>

#define AESCTR_NONCESIZE   4
#define AESCTR_IVSIZE      8
#define AESCTR_BLOCKSIZE   16

#define AES_XTS_BLOCKSIZE  16
#define AES_XTS_IVSIZE     8
#define AES_XTS_ALPHA      0x87 /* GF(2^128) generator polynomial */

/* Declarations */

struct auth_hash
{
  int type;
  FAR char *name;
  uint16_t keysize;
  uint16_t hashsize;
  uint16_t authsize;
  uint16_t ctxsize;
  uint16_t blocksize;
  CODE void (*init) (FAR void *);
  CODE void (*setkey) (FAR void *, FAR const uint8_t *, uint16_t);
  CODE void (*reinit) (FAR void *, FAR const uint8_t *, uint16_t);
  CODE int  (*update) (FAR void *, FAR const uint8_t *, uint16_t);
  CODE void (*final) (FAR uint8_t *, FAR void *);
};

struct enc_xform
{
  int type;
  FAR char *name;
  uint16_t blocksize;
  uint16_t ivsize;
  uint16_t minkey;
  uint16_t maxkey;
  uint16_t ctxsize;
  CODE void (*encrypt) (caddr_t, FAR uint8_t *);
  CODE void (*decrypt) (caddr_t, FAR uint8_t *);
  CODE int  (*setkey) (void *, FAR uint8_t *, int len);
  CODE void (*reinit) (caddr_t, FAR uint8_t *);
};

struct comp_algo
{
  int type;
  FAR char *name;
  size_t minlen;
  CODE uint32_t (*compress) (FAR uint8_t *, uint32_t, FAR uint8_t **);
  CODE uint32_t (*decompress) (FAR uint8_t *, uint32_t, FAR uint8_t **);
};

union authctx
{
  MD5_CTX md5ctx;
  SHA1_CTX sha1ctx;
  RMD160_CTX rmd160ctx;
  SHA2_CTX sha2_ctx;
  AES_GMAC_CTX aes_gmac_ctx;
};

extern const struct enc_xform enc_xform_3des;
extern const struct enc_xform enc_xform_blf;
extern const struct enc_xform enc_xform_cast5;
extern const struct enc_xform enc_xform_aes;
extern const struct enc_xform enc_xform_aes_ctr;
extern const struct enc_xform enc_xform_aes_gcm;
extern const struct enc_xform enc_xform_aes_gmac;
extern const struct enc_xform enc_xform_aes_xts;
extern const struct enc_xform enc_xform_chacha20_poly1305;
extern const struct enc_xform enc_xform_null;

extern const struct auth_hash auth_hash_hmac_md5_96;
extern const struct auth_hash auth_hash_hmac_sha1_96;
extern const struct auth_hash auth_hash_hmac_ripemd_160_96;
extern const struct auth_hash auth_hash_hmac_sha2_256_128;
extern const struct auth_hash auth_hash_hmac_sha2_384_192;
extern const struct auth_hash auth_hash_hmac_sha2_512_256;
extern const struct auth_hash auth_hash_gmac_aes_128;
extern const struct auth_hash auth_hash_gmac_aes_192;
extern const struct auth_hash auth_hash_gmac_aes_256;
extern const struct auth_hash auth_hash_chacha20_poly1305;

extern const struct comp_algo comp_algo_deflate;

#endif /* __INCLUDE_CRYPTO_XFORM_H */
