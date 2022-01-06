/****************************************************************************
 * crypto/testmngr.c
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

static int do_test_aes(FAR struct cipher_testvec *test,
                       int mode,
                       int encrypt)
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
