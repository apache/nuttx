/****************************************************************************
 * crypto/testmngr.c
 *
 *   Copyright (C) 2014-2015 Gregory Nutt. All rights reserved.
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
#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/crypto/crypto.h>

#ifdef CONFIG_CRYPTO_ALGTEST

#include "testmngr.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef ARRAY_SIZE
#  define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

#if defined(CONFIG_CRYPTO_AES)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int do_test_aes(FAR struct cipher_testvec *test, int mode, int encrypt)
{
  FAR void *out = kmm_zalloc(test->rlen);

  int res = aes_cypher(out, test->input, test->ilen, test->iv, test->key,
                       test->klen, mode, encrypt);
  if (res == OK)
    {
      res = memcmp(out, test->result, test->rlen);
    }

  kmm_free(out);
  return res;
}

#define AES_CYPHER_TEST_ENCRYPT(mode, mode_str, count, template) \
  for (i = 0; i < count; i++) { \
    if (do_test_aes(template + i, mode, CYPHER_ENCRYPT)) { \
      crypterr("ERROR: Failed " mode_str " encrypt test #%i\n", i); \
      return -1; \
    } \
  }

#define AES_CYPHER_TEST_DECRYPT(mode, mode_str, count, template) \
  for (i = 0; i < count; i++) { \
    if (do_test_aes(template + i, mode, CYPHER_DECRYPT)) { \
      crypterr("ERROR: Failed " mode_str " decrypt test #%i\n", i); \
      return -1; \
    } \
  }

#define AES_CYPHER_TEST(mode, mode_str, enc_count, dec_count, enc_template, dec_template) \
  AES_CYPHER_TEST_ENCRYPT(mode, mode_str, enc_count, enc_template)\
  AES_CYPHER_TEST_DECRYPT(mode, mode_str, dec_count, dec_template)

static int test_aes(void)
{
  int i;

  AES_CYPHER_TEST(AES_MODE_ECB, "ECB", ARRAY_SIZE(aes_enc_tv_template),
                  ARRAY_SIZE(aes_dec_tv_template), aes_enc_tv_template,
                  aes_dec_tv_template)
  AES_CYPHER_TEST(AES_MODE_CBC, "CBC", ARRAY_SIZE(aes_cbc_enc_tv_template),
                  ARRAY_SIZE(aes_cbc_dec_tv_template),
                  aes_cbc_enc_tv_template, aes_cbc_dec_tv_template)
  AES_CYPHER_TEST(AES_MODE_CTR, "CTR", ARRAY_SIZE(aes_ctr_enc_tv_template),
                  ARRAY_SIZE(aes_ctr_dec_tv_template),
                  aes_ctr_enc_tv_template, aes_ctr_dec_tv_template)

  return OK;
}
#endif

int crypto_test(void)
{
#if defined(CONFIG_CRYPTO_AES)
  if (test_aes())
    {
      return -1;
    }
#endif

  return OK;
}

#else /* CONFIG_CRYPTO_ALGTEST */

int crypto_test(void)
{
  return OK;
}

#endif /* CONFIG_CRYPTO_ALGTEST */
