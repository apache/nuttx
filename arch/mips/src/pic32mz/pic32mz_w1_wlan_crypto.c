/****************************************************************************
 * arch/mips/src/pic32mz/pic32mz_w1_wlan_crypto.c
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

/* Crypto services the PIC32MZ-W1 WLAN library expects from the host
 * ([EX] interface of drv_pic32mzw1_crypto.c, see pic32mz_w1_wlan.c for the
 * tags).  Random, hash and HMAC, all that WPA/WPA2-Personal needs, use
 * NuttX's software crypto.  The big number and elliptic curve operations
 * (WPA3-Personal), DES (MSCHAPv2) and TLS (enterprise) are not implemented
 * and report an error.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>

#include <crypto/md5.h>
#include <crypto/sha1.h>
#include <crypto/sha2.h>

#include "pic32mz_w1_wlan.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* [EX] Hash identifiers (DRV_PIC32MZW_CRYPTO_HASH_T) */

#define WLAN_HASH_MD5           2
#define WLAN_HASH_SHA1          3
#define WLAN_HASH_SHA256        4
#define WLAN_HASH_SHA224        5
#define WLAN_HASH_SHA512        6
#define WLAN_HASH_SHA384        7

/* [EX] Return codes (DRV_PIC32MZW_CRYPTO_RETURN_T) */

#define WLAN_CRYPTO_COMPLETE    0
#define WLAN_CRYPTO_INVALID     3
#define WLAN_CRYPTO_ERROR       4

#define WLAN_HASH_MAXBLOCK      128
#define WLAN_HASH_MAXDIGEST     64

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* [EX] Input buffer descriptor (buffer_t) */

struct wlan_buffer_s
{
  FAR const uint8_t *data;
  uint16_t len;
};

union wlan_hashctx_u
{
  MD5_CTX md5;
  SHA1_CTX sha1;
  SHA2_CTX sha2;
};

struct wlan_hash_s
{
  uint8_t type;
  uint8_t blocklen;
  uint8_t digestlen;
  CODE void (*init)(FAR union wlan_hashctx_u *ctx);
  CODE void (*update)(FAR union wlan_hashctx_u *ctx, FAR const void *data,
                      size_t len);
  CODE void (*final)(FAR uint8_t *digest, FAR union wlan_hashctx_u *ctx);
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void md5_init(FAR union wlan_hashctx_u *c);
static void md5_update(FAR union wlan_hashctx_u *c, FAR const void *d,
                       size_t n);
static void md5_final(FAR uint8_t *o, FAR union wlan_hashctx_u *c);
static void sha1_init(FAR union wlan_hashctx_u *c);
static void sha1_update(FAR union wlan_hashctx_u *c, FAR const void *d,
                        size_t n);
static void sha1_final(FAR uint8_t *o, FAR union wlan_hashctx_u *c);
static void sha224_init(FAR union wlan_hashctx_u *c);
static void sha224_update(FAR union wlan_hashctx_u *c, FAR const void *d,
                          size_t n);
static void sha224_final(FAR uint8_t *o, FAR union wlan_hashctx_u *c);
static void sha256_init(FAR union wlan_hashctx_u *c);
static void sha256_update(FAR union wlan_hashctx_u *c, FAR const void *d,
                          size_t n);
static void sha256_final(FAR uint8_t *o, FAR union wlan_hashctx_u *c);
static void sha384_init(FAR union wlan_hashctx_u *c);
static void sha384_update(FAR union wlan_hashctx_u *c, FAR const void *d,
                          size_t n);
static void sha384_final(FAR uint8_t *o, FAR union wlan_hashctx_u *c);
static void sha512_init(FAR union wlan_hashctx_u *c);
static void sha512_update(FAR union wlan_hashctx_u *c, FAR const void *d,
                          size_t n);
static void sha512_final(FAR uint8_t *o, FAR union wlan_hashctx_u *c);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct wlan_hash_s g_wlan_hash[] =
{
  { WLAN_HASH_MD5,     64, 16, md5_init,    md5_update,    md5_final    },
  { WLAN_HASH_SHA1,    64, 20, sha1_init,   sha1_update,   sha1_final   },
  { WLAN_HASH_SHA224,  64, 28, sha224_init, sha224_update, sha224_final },
  { WLAN_HASH_SHA256,  64, 32, sha256_init, sha256_update, sha256_final },
  { WLAN_HASH_SHA384, 128, 48, sha384_init, sha384_update, sha384_final },
  { WLAN_HASH_SHA512, 128, 64, sha512_init, sha512_update, sha512_final },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void md5_init(FAR union wlan_hashctx_u *c)
{
  md5init(&c->md5);
}

static void md5_update(FAR union wlan_hashctx_u *c, FAR const void *d,
                       size_t n)
{
  md5update(&c->md5, d, n);
}

static void md5_final(FAR uint8_t *o, FAR union wlan_hashctx_u *c)
{
  md5final(o, &c->md5);
}

static void sha1_init(FAR union wlan_hashctx_u *c)
{
  sha1init(&c->sha1);
}

static void sha1_update(FAR union wlan_hashctx_u *c, FAR const void *d,
                        size_t n)
{
  sha1update(&c->sha1, d, n);
}

static void sha1_final(FAR uint8_t *o, FAR union wlan_hashctx_u *c)
{
  sha1final(o, &c->sha1);
}

static void sha224_init(FAR union wlan_hashctx_u *c)
{
  sha224init(&c->sha2);
}

static void sha224_update(FAR union wlan_hashctx_u *c, FAR const void *d,
                          size_t n)
{
  sha224update(&c->sha2, d, n);
}

static void sha224_final(FAR uint8_t *o, FAR union wlan_hashctx_u *c)
{
  sha224final(o, &c->sha2);
}

static void sha256_init(FAR union wlan_hashctx_u *c)
{
  sha256init(&c->sha2);
}

static void sha256_update(FAR union wlan_hashctx_u *c, FAR const void *d,
                          size_t n)
{
  sha256update(&c->sha2, d, n);
}

static void sha256_final(FAR uint8_t *o, FAR union wlan_hashctx_u *c)
{
  sha256final(o, &c->sha2);
}

static void sha384_init(FAR union wlan_hashctx_u *c)
{
  sha384init(&c->sha2);
}

static void sha384_update(FAR union wlan_hashctx_u *c, FAR const void *d,
                          size_t n)
{
  sha384update(&c->sha2, d, n);
}

static void sha384_final(FAR uint8_t *o, FAR union wlan_hashctx_u *c)
{
  sha384final(o, &c->sha2);
}

static void sha512_init(FAR union wlan_hashctx_u *c)
{
  sha512init(&c->sha2);
}

static void sha512_update(FAR union wlan_hashctx_u *c, FAR const void *d,
                          size_t n)
{
  sha512update(&c->sha2, d, n);
}

static void sha512_final(FAR uint8_t *o, FAR union wlan_hashctx_u *c)
{
  sha512final(o, &c->sha2);
}

static FAR const struct wlan_hash_s *wlan_hash_find(int type)
{
  int i;

  for (i = 0; i < sizeof(g_wlan_hash) / sizeof(g_wlan_hash[0]); i++)
    {
      if (g_wlan_hash[i].type == type)
        {
          return &g_wlan_hash[i];
        }
    }

  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

bool DRV_PIC32MZW_Crypto_Random(FAR uint8_t *out, uint16_t len)
{
  arc4random_buf(out, len);
  return true;
}

int DRV_PIC32MZW_Crypto_Hash(FAR const struct wlan_buffer_s *in, int count,
                             FAR uint8_t *digest, int type)
{
  FAR const struct wlan_hash_s *h = wlan_hash_find(type);
  union wlan_hashctx_u ctx;
  int i;

  if (h == NULL || in == NULL || digest == NULL)
    {
      syslog(LOG_ERR, "wlan: unsupported hash %d\n", type);
      return WLAN_CRYPTO_INVALID;
    }

  h->init(&ctx);
  for (i = 0; i < count; i++)
    {
      h->update(&ctx, in[i].data, in[i].len);
    }

  h->final(digest, &ctx);
  return WLAN_CRYPTO_COMPLETE;
}

/* HMAC (RFC 2104) over any of the hashes above.  The library calls the
 * key "salt".  The inputs and the digest may overlap.
 */

int DRV_PIC32MZW_Crypto_HMAC(FAR const uint8_t *key, uint16_t keylen,
                             FAR const struct wlan_buffer_s *in, int count,
                             FAR uint8_t *digest, int type)
{
  FAR const struct wlan_hash_s *h = wlan_hash_find(type);
  uint8_t pad[WLAN_HASH_MAXBLOCK];
  uint8_t inner[WLAN_HASH_MAXDIGEST];
  union wlan_hashctx_u ctx;
  int i;

  if (h == NULL || h->type == WLAN_HASH_MD5 || key == NULL || in == NULL ||
      digest == NULL)
    {
      syslog(LOG_ERR, "wlan: unsupported HMAC %d\n", type);
      return WLAN_CRYPTO_INVALID;
    }

  memset(pad, 0, sizeof(pad));
  if (keylen > h->blocklen)
    {
      h->init(&ctx);
      h->update(&ctx, key, keylen);
      h->final(pad, &ctx);
    }
  else
    {
      memcpy(pad, key, keylen);
    }

  for (i = 0; i < h->blocklen; i++)
    {
      pad[i] ^= 0x36;
    }

  h->init(&ctx);
  h->update(&ctx, pad, h->blocklen);
  for (i = 0; i < count; i++)
    {
      h->update(&ctx, in[i].data, in[i].len);
    }

  h->final(inner, &ctx);

  for (i = 0; i < h->blocklen; i++)
    {
      pad[i] ^= 0x36 ^ 0x5c;
    }

  h->init(&ctx);
  h->update(&ctx, pad, h->blocklen);
  h->update(&ctx, inner, h->digestlen);
  h->final(digest, &ctx);

  return WLAN_CRYPTO_COMPLETE;
}

/* Not implemented: every call reports an error.  The library only uses
 * them for WPA3-Personal (SAE), MSCHAPv2 and enterprise TLS.
 */

#define WLAN_CRYPTO_STUB(name) \
  int name(void) \
  { \
    syslog(LOG_ERR, "wlan: %s not implemented\n", #name); \
    return WLAN_CRYPTO_ERROR; \
  }

WLAN_CRYPTO_STUB(DRV_PIC32MZW_Crypto_DES_Ecb_Crypt)
WLAN_CRYPTO_STUB(DRV_PIC32MZW_Crypto_BigIntMod)
WLAN_CRYPTO_STUB(DRV_PIC32MZW_Crypto_BigIntModAdd)
WLAN_CRYPTO_STUB(DRV_PIC32MZW_Crypto_BigIntModSubtract)
WLAN_CRYPTO_STUB(DRV_PIC32MZW_Crypto_BigIntModMultiply)
WLAN_CRYPTO_STUB(DRV_PIC32MZW_Crypto_BigIntModExponentiate)
WLAN_CRYPTO_STUB(DRV_PIC32MZW_Crypto_ECCAdd)
WLAN_CRYPTO_STUB(DRV_PIC32MZW_Crypto_ECCBigIntModAddB)
WLAN_CRYPTO_STUB(DRV_PIC32MZW_Crypto_ECCBigIntModMultByA)
WLAN_CRYPTO_STUB(DRV_PIC32MZW_Crypto_ECCIsOnCurve)
WLAN_CRYPTO_STUB(DRV_PIC32MZW_Crypto_ECCMultiply)

/* [EX] Returns the curve's field, or NULL for an unknown curve */

FAR const uint8_t *DRV_PIC32MZW_Crypto_ECCGetField(int curve)
{
  syslog(LOG_ERR, "wlan: ECC not implemented\n");
  return NULL;
}

#define WLAN_TLS_STUB(name) \
  int name(void) \
  { \
    syslog(LOG_ERR, "wlan: %s not implemented\n", #name); \
    return 0; \
  }

WLAN_TLS_STUB(DRV_PIC32MZW_TLS_CreateSession)
WLAN_TLS_STUB(DRV_PIC32MZW_TLS_StartSession)
WLAN_TLS_STUB(DRV_PIC32MZW_TLS_TerminateSession)
WLAN_TLS_STUB(DRV_PIC32MZW_TLS_RecvBuffer)
WLAN_TLS_STUB(DRV_PIC32MZW_TLS_WriteBuffer)
WLAN_TLS_STUB(DRV_PIC32MZW_TLS_GenerateKey)
