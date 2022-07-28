/****************************************************************************
 * crypto/xform.c
 * $OpenBSD: xform.c,v 1.61 2021/10/22 12:30:53 bluhm Exp $
 *
 * The authors of this code are John Ioannidis (ji@tla.org),
 * Angelos D. Keromytis (kermit@csd.uch.gr),
 * Niels Provos (provos@physnet.uni-hamburg.de),
 * Damien Miller (djm@mindrot.org) and
 * Mike Belopuhov (mikeb@openbsd.org).
 *
 * This code was written by John Ioannidis for BSD/OS in Athens, Greece,
 * in November 1995.
 *
 * Ported to OpenBSD and NetBSD, with additional transforms,
 * in December 1996,
 * by Angelos D. Keromytis.
 *
 * Additional transforms and features in 1997 and 1998 by
 * Angelos D. Keromytis and Niels Provos.
 *
 * Additional features in 1999 by Angelos D. Keromytis.
 *
 * AES XTS implementation in 2008 by Damien Miller
 *
 * AES-GCM-16 and Chacha20-Poly1305 AEAD modes by Mike Belopuhov.
 *
 * Copyright (C) 1995, 1996, 1997, 1998, 1999 by John Ioannidis,
 * Angelos D. Keromytis and Niels Provos.
 *
 * Copyright (C) 2001, Angelos D. Keromytis.
 *
 * Copyright (C) 2008, Damien Miller
 *
 * Copyright (C) 2010, 2015, Mike Belopuhov
 *
 * Permission to use, copy, and modify this software with or without fee
 * is hereby granted, provided that this entire notice is included in
 * all copies of any software which is or includes a copy or
 * modification of this software.
 * You may use this code under the GNU public license if you so wish. Please
 * contribute changes back to the authors under this freer than GPL license
 * so that we may further the use of strong encryption without limitations to
 * all.
 *
 * THIS SOFTWARE IS BEING PROVIDED "AS IS", WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTY. IN PARTICULAR, NONE OF THE AUTHORS MAKES ANY
 * REPRESENTATION OR WARRANTY OF ANY KIND CONCERNING THE
 * MERCHANTABILITY OF THIS SOFTWARE OR ITS FITNESS FOR ANY PARTICULAR
 * PURPOSE.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <errno.h>
#include <string.h>
#include <strings.h>
#include <sys/param.h>
#include <sys/time.h>

#include <crypto/md5.h>
#include <crypto/sha1.h>
#include <crypto/sha2.h>
#include <crypto/rmd160.h>
#include <crypto/blf.h>
#include <crypto/cast.h>
#include <crypto/rijndael.h>
#include <crypto/aes.h>
#include <crypto/cryptodev.h>
#include <crypto/xform.h>
#include <crypto/gmac.h>
#include <crypto/chachapoly.h>

#include "des_locl.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

extern void des_ecb3_encrypt(caddr_t, caddr_t, caddr_t,
                             caddr_t, caddr_t, int);

int des_set_key(FAR void *, caddr_t);
int des3_setkey(FAR void *, FAR uint8_t *, int);
int blf_setkey(FAR void *, FAR uint8_t *, int);
int cast5_setkey(FAR void *, FAR uint8_t *, int);
int aes_setkey_xform(FAR void *, FAR uint8_t *, int);
int aes_ctr_setkey(FAR void *, FAR uint8_t *, int);
int aes_xts_setkey(FAR void *, FAR uint8_t *, int);
int null_setkey(FAR void *, FAR uint8_t *, int);

void des3_encrypt(caddr_t, FAR uint8_t *);
void blf_encrypt(caddr_t, FAR uint8_t *);
void cast5_encrypt(caddr_t, FAR uint8_t *);
void aes_encrypt_xform(caddr_t, FAR uint8_t *);
void null_encrypt(caddr_t, FAR uint8_t *);
void aes_xts_encrypt(caddr_t, FAR uint8_t *);

void des3_decrypt(caddr_t, FAR uint8_t *);
void blf_decrypt(caddr_t, FAR uint8_t *);
void cast5_decrypt(caddr_t, FAR uint8_t *);
void aes_decrypt_xform(caddr_t, FAR uint8_t *);
void null_decrypt(caddr_t, FAR uint8_t *);
void aes_xts_decrypt(caddr_t, FAR uint8_t *);

void aes_ctr_crypt(caddr_t, FAR uint8_t *);

void aes_ctr_reinit(caddr_t, FAR uint8_t *);
void aes_xts_reinit(caddr_t, FAR uint8_t *);
void aes_gcm_reinit(caddr_t, FAR uint8_t *);

int md5update_int(FAR void *, FAR const uint8_t *, uint16_t);
int sha1update_int(FAR void *, FAR const uint8_t *, uint16_t);
int rmd160update_int(FAR void *, FAR const uint8_t *, uint16_t);
int sha256update_int(FAR void *, FAR const uint8_t *, uint16_t);
int sha384update_int(FAR void *, FAR const uint8_t *, uint16_t);
int sha512update_int(FAR void *, FAR const uint8_t *, uint16_t);

struct aes_ctr_ctx
{
  AES_CTX ac_key;
  uint8_t ac_block[AESCTR_BLOCKSIZE];
};

struct aes_xts_ctx
{
  rijndael_ctx key1;
  rijndael_ctx key2;
  uint8_t tweak[AES_XTS_BLOCKSIZE];
};

/* Helper */

void aes_xts_crypt(FAR struct aes_xts_ctx *, FAR uint8_t *, u_int);

/* Encryption instances */

const struct enc_xform enc_xform_3des =
{
  CRYPTO_3DES_CBC, "3DES",
  8, 8, 24, 24, 384,
  des3_encrypt,
  des3_decrypt,
  des3_setkey,
  NULL
};

const struct enc_xform enc_xform_blf =
{
  CRYPTO_BLF_CBC, "Blowfish",
  8, 8, 5, 56 /* 448 bits, max key */,
  sizeof(blf_ctx),
  blf_encrypt,
  blf_decrypt,
  blf_setkey,
  NULL
};

const struct enc_xform enc_xform_cast5 =
{
  CRYPTO_CAST_CBC, "CAST-128",
  8, 8, 5, 16,
  sizeof(cast_key),
  cast5_encrypt,
  cast5_decrypt,
  cast5_setkey,
  NULL
};

const struct enc_xform enc_xform_aes =
{
  CRYPTO_AES_CBC, "AES",
  16, 16, 16, 32,
  sizeof(AES_CTX),
  aes_encrypt_xform,
  aes_decrypt_xform,
  aes_setkey_xform,
  NULL
};

const struct enc_xform enc_xform_aes_ctr =
{
  CRYPTO_AES_CTR, "AES-CTR",
  16, 8, 16 + 4, 32 + 4,
  sizeof(struct aes_ctr_ctx),
  aes_ctr_crypt,
  aes_ctr_crypt,
  aes_ctr_setkey,
  aes_ctr_reinit
};

const struct enc_xform enc_xform_aes_gcm =
{
  CRYPTO_AES_GCM_16, "AES-GCM",
  1, 8, 16 + 4, 32 + 4,
  sizeof(struct aes_ctr_ctx),
  aes_ctr_crypt,
  aes_ctr_crypt,
  aes_ctr_setkey,
  aes_gcm_reinit
};

const struct enc_xform enc_xform_aes_gmac =
{
  CRYPTO_AES_GMAC, "AES-GMAC",
  1, 8, 16 + 4, 32 + 4, 0,
  NULL,
  NULL,
  NULL,
  NULL
};

const struct enc_xform enc_xform_aes_xts =
{
  CRYPTO_AES_XTS, "AES-XTS",
  16, 8, 32, 64,
  sizeof(struct aes_xts_ctx),
  aes_xts_encrypt,
  aes_xts_decrypt,
  aes_xts_setkey,
  aes_xts_reinit
};

const struct enc_xform enc_xform_chacha20_poly1305 =
{
  CRYPTO_CHACHA20_POLY1305, "CHACHA20-POLY1305",
  1, 8, 32 + 4, 32 + 4,
  sizeof(struct chacha20_ctx),
  chacha20_crypt,
  chacha20_crypt,
  chacha20_setkey,
  chacha20_reinit
};

const struct enc_xform enc_xform_null =
{
  CRYPTO_NULL, "NULL",
  4, 0, 0, 256, 0,
  null_encrypt,
  null_decrypt,
  null_setkey,
  NULL
};

/* Authentication instances */

const struct auth_hash auth_hash_hmac_md5_96 =
{
  CRYPTO_MD5_HMAC, "HMAC-MD5",
  16, 16, 12, sizeof(MD5_CTX), HMAC_MD5_BLOCK_LEN,
  (void (*) (FAR void *)) md5init, NULL, NULL,
  md5update_int,
  (void (*) (FAR uint8_t *, FAR void *)) md5final
};

const struct auth_hash auth_hash_hmac_sha1_96 =
{
  CRYPTO_SHA1_HMAC, "HMAC-SHA1",
  20, 20, 12, sizeof(SHA1_CTX), HMAC_SHA1_BLOCK_LEN,
  (void (*) (FAR void *)) sha1init, NULL, NULL,
  sha1update_int,
  (void (*) (FAR uint8_t *, FAR void *)) sha1final
};

const struct auth_hash auth_hash_hmac_ripemd_160_96 =
{
  CRYPTO_RIPEMD160_HMAC, "HMAC-RIPEMD-160",
  20, 20, 12, sizeof(RMD160_CTX), HMAC_RIPEMD160_BLOCK_LEN,
  (void (*)(FAR void *)) rmd160init, NULL, NULL,
  rmd160update_int,
  (void (*)(FAR uint8_t *, FAR void *)) rmd160final
};

const struct auth_hash auth_hash_hmac_sha2_256_128 =
{
  CRYPTO_SHA2_256_HMAC, "HMAC-SHA2-256",
  32, 32, 16, sizeof(SHA2_CTX), HMAC_SHA2_256_BLOCK_LEN,
  (void (*)(FAR void *)) sha256init, NULL, NULL,
  sha256update_int,
  (void (*)(FAR uint8_t *, FAR void *)) sha256final
};

const struct auth_hash auth_hash_hmac_sha2_384_192 =
{
  CRYPTO_SHA2_384_HMAC, "HMAC-SHA2-384",
  48, 48, 24, sizeof(SHA2_CTX), HMAC_SHA2_384_BLOCK_LEN,
  (void (*)(FAR void *)) sha384init, NULL, NULL,
  sha384update_int,
  (void (*)(FAR uint8_t *, FAR void *)) sha384final
};

const struct auth_hash auth_hash_hmac_sha2_512_256 =
{
  CRYPTO_SHA2_512_HMAC, "HMAC-SHA2-512",
  64, 64, 32, sizeof(SHA2_CTX), HMAC_SHA2_512_BLOCK_LEN,
  (void (*)(FAR void *)) sha512init, NULL, NULL,
  sha512update_int,
  (void (*)(FAR uint8_t *, FAR void *)) sha512final
};

const struct auth_hash auth_hash_gmac_aes_128 =
{
  CRYPTO_AES_128_GMAC, "GMAC-AES-128",
  16 + 4, GMAC_BLOCK_LEN, GMAC_DIGEST_LEN, sizeof(AES_GMAC_CTX),
  AESCTR_BLOCKSIZE, aes_gmac_init, aes_gmac_setkey, aes_gmac_reinit,
  aes_gmac_update, aes_gmac_final
};

const struct auth_hash auth_hash_gmac_aes_192 =
{
  CRYPTO_AES_192_GMAC, "GMAC-AES-192",
  24 + 4, GMAC_BLOCK_LEN, GMAC_DIGEST_LEN, sizeof(AES_GMAC_CTX),
  AESCTR_BLOCKSIZE, aes_gmac_init, aes_gmac_setkey, aes_gmac_reinit,
  aes_gmac_update, aes_gmac_final
};

const struct auth_hash auth_hash_gmac_aes_256 =
{
  CRYPTO_AES_256_GMAC, "GMAC-AES-256",
  32 + 4, GMAC_BLOCK_LEN, GMAC_DIGEST_LEN, sizeof(AES_GMAC_CTX),
  AESCTR_BLOCKSIZE, aes_gmac_init, aes_gmac_setkey, aes_gmac_reinit,
  aes_gmac_update, aes_gmac_final
};

const struct auth_hash auth_hash_chacha20_poly1305 =
{
  CRYPTO_CHACHA20_POLY1305_MAC, "CHACHA20-POLY1305",
  CHACHA20_KEYSIZE + CHACHA20_SALT, POLY1305_BLOCK_LEN, POLY1305_TAGLEN,
  sizeof(CHACHA20_POLY1305_CTX), CHACHA20_BLOCK_LEN,
  chacha20_poly1305_init, chacha20_poly1305_setkey,
  chacha20_poly1305_reinit, chacha20_poly1305_update,
  chacha20_poly1305_final
};

/* Encryption wrapper routines. */

void des3_encrypt(caddr_t key, FAR uint8_t *blk)
{
  des_ecb3_encrypt((caddr_t)blk, (caddr_t)blk, key, key + 128, key + 256, 1);
}

void des3_decrypt(caddr_t key, FAR uint8_t *blk)
{
  des_ecb3_encrypt((caddr_t)blk, (caddr_t)blk, key + 256, key + 128, key, 0);
}

int des3_setkey(FAR void *sched, FAR uint8_t *key, int len)
{
  if (des_set_key(key, sched) < 0 || des_set_key(key + 8, sched + 128)
      < 0 || des_set_key(key + 16, sched + 256) < 0)
    {
      return -1;
    }

  return 0;
}

void blf_encrypt(caddr_t key, FAR uint8_t *blk)
{
  blf_ecb_encrypt((FAR blf_ctx *) key, blk, 8);
}

void blf_decrypt(caddr_t key, FAR uint8_t *blk)
{
  blf_ecb_decrypt((FAR blf_ctx *) key, blk, 8);
}

int blf_setkey(FAR void *sched, FAR uint8_t *key, int len)
{
  blf_key((FAR blf_ctx *)sched, key, len);

  return 0;
}

int null_setkey(FAR void *sched, FAR uint8_t *key, int len)
{
  return 0;
}

void null_encrypt(caddr_t key, FAR uint8_t *blk)
{
}

void null_decrypt(caddr_t key, FAR uint8_t *blk)
{
}

void cast5_encrypt(caddr_t key, FAR uint8_t *blk)
{
  cast_encrypt((FAR cast_key *) key, blk, blk);
}

void cast5_decrypt(caddr_t key, FAR uint8_t *blk)
{
  cast_decrypt((FAR cast_key *) key, blk, blk);
}

int cast5_setkey(FAR void *sched, FAR uint8_t *key, int len)
{
  cast_setkey((FAR cast_key *)sched, key, len);

  return 0;
}

void aes_encrypt_xform(caddr_t key, FAR uint8_t *blk)
{
  aes_encrypt((FAR AES_CTX *)key, blk, blk);
}

void aes_decrypt_xform(caddr_t key, FAR uint8_t *blk)
{
  aes_decrypt((FAR AES_CTX *)key, blk, blk);
}

int aes_setkey_xform(FAR void *sched, FAR uint8_t *key, int len)
{
  return aes_setkey((FAR AES_CTX *)sched, key, len);
}

void aes_ctr_reinit(caddr_t key, FAR uint8_t *iv)
{
  FAR struct aes_ctr_ctx *ctx;

  ctx = (FAR struct aes_ctr_ctx *)key;
  bcopy(iv, ctx->ac_block + AESCTR_NONCESIZE, AESCTR_IVSIZE);

  /* reset counter */

  bzero(ctx->ac_block + AESCTR_NONCESIZE + AESCTR_IVSIZE, 4);
}

void aes_gcm_reinit(caddr_t key, FAR uint8_t *iv)
{
  FAR struct aes_ctr_ctx *ctx;

  ctx = (FAR struct aes_ctr_ctx *)key;
  bcopy(iv, ctx->ac_block + AESCTR_NONCESIZE, AESCTR_IVSIZE);

  /* reset counter */

  bzero(ctx->ac_block + AESCTR_NONCESIZE + AESCTR_IVSIZE, 4);
  ctx->ac_block[AESCTR_BLOCKSIZE - 1] = 1; /* GCM starts with 1 */
}

void aes_ctr_crypt(caddr_t key, FAR uint8_t *data)
{
  FAR struct aes_ctr_ctx *ctx;
  uint8_t keystream[AESCTR_BLOCKSIZE];
  int i;

  ctx = (FAR struct aes_ctr_ctx *)key;

  /* increment counter */

  for (i = AESCTR_BLOCKSIZE - 1;
        i >= AESCTR_NONCESIZE + AESCTR_IVSIZE; i--)
    {
      if (++ctx->ac_block[i])   /* continue on overflow */
        {
          break;
        }
    }

  aes_encrypt(&ctx->ac_key, ctx->ac_block, keystream);
  for (i = 0; i < AESCTR_BLOCKSIZE; i++)
    {
      data[i] ^= keystream[i];
    }

  explicit_bzero(keystream, sizeof(keystream));
}

int aes_ctr_setkey(FAR void *sched, FAR uint8_t *key, int len)
{
  FAR struct aes_ctr_ctx *ctx;

  if (len < AESCTR_NONCESIZE)
    {
      return -1;
    }

  ctx = (FAR struct aes_ctr_ctx *)sched;
  if (aes_setkey(&ctx->ac_key, key, len - AESCTR_NONCESIZE) != 0)
    {
      return -1;
    }

  bcopy(key + len - AESCTR_NONCESIZE, ctx->ac_block, AESCTR_NONCESIZE);
  return 0;
}

void aes_xts_reinit(caddr_t key, FAR uint8_t *iv)
{
  FAR struct aes_xts_ctx *ctx = (FAR struct aes_xts_ctx *)key;
  uint64_t blocknum;
  u_int i;

  /* Prepare tweak as E_k2(IV). IV is specified as LE representation
   * of a 64-bit block number which we allow to be passed in directly.
   */

  memcpy(&blocknum, iv, AES_XTS_IVSIZE);
  for (i = 0; i < AES_XTS_IVSIZE; i++)
    {
      ctx->tweak[i] = blocknum & 0xff;
      blocknum >>= 8;
    }

  /* Last 64 bits of IV are always zero */

  bzero(ctx->tweak + AES_XTS_IVSIZE, AES_XTS_IVSIZE);

  rijndael_encrypt(&ctx->key2, ctx->tweak, ctx->tweak);
}

void aes_xts_crypt(FAR struct aes_xts_ctx *ctx,
                   FAR uint8_t *data,
                   u_int do_encrypt)
{
  uint8_t block[AES_XTS_BLOCKSIZE];
  u_int i;
  u_int carry_in;
  u_int carry_out;

  for (i = 0; i < AES_XTS_BLOCKSIZE; i++)
    {
      block[i] = data[i] ^ ctx->tweak[i];
    }

  if (do_encrypt)
    {
      rijndael_encrypt(&ctx->key1, block, data);
    }
  else
    {
      rijndael_decrypt(&ctx->key1, block, data);
    }

  for (i = 0; i < AES_XTS_BLOCKSIZE; i++)
    {
      data[i] ^= ctx->tweak[i];
    }

  /* Exponentiate tweak */

  carry_in = 0;
  for (i = 0; i < AES_XTS_BLOCKSIZE; i++)
    {
      carry_out = ctx->tweak[i] & 0x80;
      ctx->tweak[i] = (ctx->tweak[i] << 1) | carry_in;
      carry_in = carry_out >> 7;
    }

  ctx->tweak[0] ^= (AES_XTS_ALPHA & -carry_in);
  explicit_bzero(block, sizeof(block));
}

void aes_xts_encrypt(caddr_t key, FAR uint8_t *data)
{
  aes_xts_crypt((FAR struct aes_xts_ctx *)key, data, 1);
}

void aes_xts_decrypt(caddr_t key, FAR uint8_t *data)
{
  aes_xts_crypt((FAR struct aes_xts_ctx *)key, data, 0);
}

int aes_xts_setkey(FAR void *sched, FAR uint8_t *key, int len)
{
  FAR struct aes_xts_ctx *ctx;

  if (len != 32 && len != 64)
    {
      return -1;
    }

  ctx = (FAR struct aes_xts_ctx *)sched;

  rijndael_set_key(&ctx->key1, key, len * 4);
  rijndael_set_key(&ctx->key2, key + (len / 2), len * 4);

  return 0;
}

/* And now for auth. */

int rmd160update_int(FAR void *ctx, FAR const uint8_t *buf, uint16_t len)
{
  rmd160update(ctx, buf, len);
  return 0;
}

int md5update_int(FAR void *ctx, FAR const uint8_t *buf, uint16_t len)
{
  md5update(ctx, buf, len);
  return 0;
}

int sha1update_int(FAR void *ctx, FAR const uint8_t *buf, uint16_t len)
{
  sha1update(ctx, buf, len);
  return 0;
}

int sha256update_int(FAR void *ctx, FAR const uint8_t *buf, uint16_t len)
{
  sha256update(ctx, buf, len);
  return 0;
}

int sha384update_int(FAR void *ctx, FAR const uint8_t *buf, uint16_t len)
{
  sha384update(ctx, buf, len);
  return 0;
}

int sha512update_int(FAR void *ctx, FAR const uint8_t *buf, uint16_t len)
{
  sha512update(ctx, buf, len);
  return 0;
}
