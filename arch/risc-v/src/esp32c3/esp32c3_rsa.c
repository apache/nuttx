/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_rsa.c
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

#ifdef CONFIG_ESP32C3_RSA_ACCELERATOR

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <debug.h>
#include <semaphore.h>

#include "riscv_internal.h"
#include "esp32c3_rsa.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ESP32C3_PKCS1_V15
#define RSA_EXPONENT_BLINDING 28

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rsa_check_context
 *
 * Description:
 *   Checks whether the context fields are set in such a way
 *   that the RSA primitives will be able to execute without error.
 *
 * Input Parameters:
 *   ctx      - The initialized RSA context to store the parameters in
 *   N        - The RSA modulus
 *   NL    - The Byte length of \p N
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static int rsa_check_context(struct esp32c3_rsa_context_s const *ctx,
                             int is_priv, int blinding_needed)
{
#if !defined(ESP32C3_RSA_NO_CRT)
  ((void) blinding_needed);
#endif

  if (ctx->len != esp32c3_mpi_size(&ctx->N) ||
      ctx->len > ESP32C3_MPI_MAX_SIZE)
    {
      return (ESP32C3_ERR_RSA_BAD_INPUT_DATA);
    }

  /* 1. Modular exponentiation needs positive, odd moduli */

  if (esp32c3_mpi_cmp_int(&ctx->N, 0) <= 0 ||
      esp32c3_mpi_get_bit(&ctx->N, 0) == 0)
    {
      return (ESP32C3_ERR_RSA_BAD_INPUT_DATA);
    }

#if !defined(ESP32C3_RSA_NO_CRT)
  if (is_priv &&
     (esp32c3_mpi_cmp_int(&ctx->P, 0) <= 0 ||
      esp32c3_mpi_get_bit(&ctx->P, 0) == 0 ||
      esp32c3_mpi_cmp_int(&ctx->Q, 0) <= 0 ||
      esp32c3_mpi_get_bit(&ctx->Q, 0) == 0))
    {
      return (ESP32C3_ERR_RSA_BAD_INPUT_DATA);
    }
#endif /* !ESP32C3_RSA_NO_CRT */

  /* 2. Exponents must be positive */

  /* Always need E for public key operations */

  if (esp32c3_mpi_cmp_int(&ctx->E, 0) <= 0)
    {
      return (ESP32C3_ERR_RSA_BAD_INPUT_DATA);
    }

#if defined(ESP32C3_RSA_NO_CRT)
  /* For private key operations, use D or DP & DQ as exponents */

  if (is_priv && esp32c3_mpi_cmp_int(&ctx->D, 0) <= 0)
    {
      return (ESP32C3_ERR_RSA_BAD_INPUT_DATA);
    }
#else
  if (is_priv &&
     (esp32c3_mpi_cmp_int(&ctx->DP, 0) <= 0 ||
      esp32c3_mpi_cmp_int(&ctx->DQ, 0) <= 0))
    {
      return (ESP32C3_ERR_RSA_BAD_INPUT_DATA);
    }
#endif /* ESP32C3_RSA_NO_CRT */

#if defined(ESP32C3_RSA_NO_CRT)
  if (is_priv && blinding_needed &&
     (esp32c3_mpi_cmp_int(&ctx->P, 0) <= 0 ||
      esp32c3_mpi_cmp_int(&ctx->Q, 0) <= 0))
    {
      return (ESP32C3_ERR_RSA_BAD_INPUT_DATA);
    }
#endif

#if !defined(ESP32C3_RSA_NO_CRT)
  if (is_priv && esp32c3_mpi_cmp_int(&ctx->QP, 0) <= 0)
    {
      return (ESP32C3_ERR_RSA_BAD_INPUT_DATA);
    }
#endif

  return OK;
}

/****************************************************************************
 * Name: rsa_prepare_blinding
 *
 * Description:
 *   Generate or update blinding values.
 *
 * Input Parameters:
 *   ctx      - The initialized RSA context to store the parameters in.
 *   f_rng    - The RNG function
 *   p_rng    - The RNG context to pass to \p f_rng
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static int rsa_prepare_blinding(struct esp32c3_rsa_context_s *ctx,
         int (*f_rng)(void *, unsigned char *, size_t), void *p_rng)
{
  int ret;
  int count = 0;
  struct esp32c3_mpi_s R;

  esp32c3_mpi_init(&R);

  if (ctx->VF.p != NULL)
    {
      /* We already have blinding values, just update them by squaring */

      ESP32C3_MPI_CHK(esp32c3_mpi_mul_mpi(&ctx->VI, &ctx->VI, &ctx->VI),
                      cleanup);
      ESP32C3_MPI_CHK(esp32c3_mpi_mod_mpi(&ctx->VI, &ctx->VI, &ctx->N),
                      cleanup);
      ESP32C3_MPI_CHK(esp32c3_mpi_mul_mpi(&ctx->VF, &ctx->VF, &ctx->VF),
                      cleanup);
      ESP32C3_MPI_CHK(esp32c3_mpi_mod_mpi(&ctx->VF, &ctx->VF, &ctx->N),
                      cleanup);

      goto cleanup;
    }

  /* Unblinding value: VF = random number, invertible mod N */

  do
    {
      if (count++ > 10)
        {
          ret = ESP32C3_ERR_RSA_RNG_FAILED;
          goto cleanup;
        }

      ESP32C3_MPI_CHK(esp32c3_mpi_fill_random(&ctx->VF, ctx->len - 1,
                                              f_rng, p_rng), cleanup);

      /* Compute VF^-1 as R * (R VF)^-1 to avoid leaks from inv_mod. */

      ESP32C3_MPI_CHK(esp32c3_mpi_fill_random(&R, ctx->len - 1,
                                              f_rng, p_rng), cleanup);
      ESP32C3_MPI_CHK(esp32c3_mpi_mul_mpi(&ctx->VI, &ctx->VF, &R),
                      cleanup);
      ESP32C3_MPI_CHK(esp32c3_mpi_mod_mpi(&ctx->VI, &ctx->VI, &ctx->N),
                      cleanup);

      ret = esp32c3_mpi_inv_mod(&ctx->VI, &ctx->VI, &ctx->N);
      if (ret != 0 && ret != ESP32C3_ERR_MPI_NOT_ACCEPTABLE)
        {
          goto cleanup;
        }
    }
  while (ret == ESP32C3_ERR_MPI_NOT_ACCEPTABLE);

  /* Finish the computation of VF^-1 = R * (R VF)^-1 */

  ESP32C3_MPI_CHK(esp32c3_mpi_mul_mpi(&ctx->VI, &ctx->VI, &R),
                  cleanup);
  ESP32C3_MPI_CHK(esp32c3_mpi_mod_mpi(&ctx->VI, &ctx->VI, &ctx->N),
                  cleanup);

  /* Blinding value: VI = VF^(-e) mod N */

  ESP32C3_MPI_CHK(esp32c3_mpi_exp_mod(&ctx->VI, &ctx->VI,
                                      &ctx->E, &ctx->N, &ctx->RN),
                  cleanup);

cleanup:
  esp32c3_mpi_free(&R);

  return ret;
}

#if defined(ESP32C3_PKCS1_V15)
/* Implementation of the PKCS1-V1_5-ENCRYPT function */

static int esp32c3_rsa_pkcs1_v15_encrypt(struct esp32c3_rsa_context_s *ctx,
                 int (*f_rng)(void *, unsigned char *, size_t),
                 void *p_rng,
                 int mode, size_t ilen,
                 const unsigned char *input,
                 unsigned char *output)
{
  size_t nb_pad;
  size_t olen;
  int ret;
  unsigned char *p = output;

  DEBUGASSERT(ctx != NULL);
  DEBUGASSERT(mode == ESP32C3_RSA_PRIVATE ||
              mode == ESP32C3_RSA_PUBLIC);
  DEBUGASSERT(output != NULL);
  DEBUGASSERT(input != NULL);

  if (mode == ESP32C3_RSA_PRIVATE && ctx->padding != ESP32C3_RSA_PKCS_V15)
    {
      return (ESP32C3_ERR_RSA_BAD_INPUT_DATA);
    }

  olen = ctx->len;

  /* first comparison checks for overflow */

  if (ilen + 11 < ilen || olen < ilen + 11)
    {
      return (ESP32C3_ERR_RSA_BAD_INPUT_DATA);
    }

  nb_pad = olen - 3 - ilen;

  *p++ = 0;
  if (mode == ESP32C3_RSA_PUBLIC)
    {
      if (f_rng == NULL)
        {
          return (ESP32C3_ERR_RSA_BAD_INPUT_DATA);
        }

      *p++ = ESP32C3_RSA_CRYPT;

      while (nb_pad-- > 0)
        {
          int rng_dl = 100;

          do
            {
              ret = f_rng(p_rng, p, 1);
            }
          while (*p == 0 && --rng_dl && ret == 0);

          /* Check if RNG failed to generate data */

          if (rng_dl == 0 || ret != 0)
            {
              return (ESP32C3_ERR_RSA_RNG_FAILED + ret);
            }

          p++;
        }
    }
  else
    {
      *p++ = ESP32C3_RSA_SIGN;

      while (nb_pad-- > 0)
        {
          *p++ = 0xff;
        }
    }

  *p++ = 0;
  memcpy(p, input, ilen);

  return ((mode == ESP32C3_RSA_PUBLIC)
          ? esp32c3_rsa_public(ctx, output, output)
          : esp32c3_rsa_private(ctx, f_rng, p_rng, output, output));
}
#endif /* ESP32C3_PKCS1_V15 */

#if defined(ESP32C3_PKCS1_V15)

/****************************************************************************
 * Name: all_or_nothing_int
 *
 * Description:
 *   Turn zero-or-nonzero into zero-or-all-bits-one, without branches.
 *
 * Input Parameters:
 *   value    - The value to analyze.
 *
 * Returned Value:
 *   Zero if \p value is zero, otherwise all-bits-one.
 *
 ****************************************************************************/

static unsigned all_or_nothing_int(unsigned value)
{
  return (- ((value | - value) >> (sizeof(value) * 8 - 1)));
}

/****************************************************************************
 * Name: size_greater_than
 *
 * Description:
 *   Check whether a size is out of bounds, without branches.
 *
 * Input Parameters:
 *   size     - Size to check.
 *   max      - Maximum desired value for \p size.
 *
 * Returned Value:
 *   \c 0 if `size <= max`, \c 1 if `size > max`.
 *
 ****************************************************************************/

static unsigned size_greater_than(size_t size, size_t max)
{
  /* Return the sign bit (1 for negative) of (max - size). */

  return ((max - size) >> (sizeof(size_t) * 8 - 1));
}

/****************************************************************************
 * Name: if_int
 *
 * Description:
 *   Choose between two integer values, without branches.
 *
 * Input Parameters:
 *   cond     - Condition to test.
 *   if1      - Value to use if \p cond is nonzero.
 *   if0      - Value to use if \p cond is zero.
 *
 * Returned Value:
 *   \c if1 if \p cond is nonzero, otherwise \c if0.
 *
 ****************************************************************************/

static unsigned if_int(unsigned cond, unsigned if1, unsigned if0)
{
  unsigned mask = all_or_nothing_int(cond);
  return ((mask & if1) | (~mask & if0));
}

/****************************************************************************
 * Name: mem_move_to_left
 *
 * Description:
 *   Shift some data towards the left inside a buffer without leaking
 *   the length of the data through side channels.
 *
 * Input Parameters:
 *   start    - Pointer to the start of the buffer.
 *   total    - Total size of the buffer.
 *   offset   - Offset from which to copy bytes.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void mem_move_to_left(void *start,
                size_t total,
                size_t offset)
{
  volatile unsigned char *buf = start;
  size_t i;
  size_t n;
  if (total == 0)
    {
      return;
    }

  for (i = 0; i < total; i++)
    {
      unsigned no_op = size_greater_than(total - offset, i);
      for (n = 0; n < total - 1; n++)
        {
          unsigned char current = buf[n];
          unsigned char next = buf[n + 1];
          buf[n] = if_int(no_op, current, next);
        }

      buf[total - 1] = if_int(no_op, buf[total - 1], 0);
    }
}

/* Implementation of the PKCS#1 v2.1 RSAES-PKCS1-V1_5-DECRYPT function */

static int esp32c3_rsa_pkcs1_v15_decrypt(struct esp32c3_rsa_context_s *ctx,
                 int (*f_rng)(void *, unsigned char *, size_t),
                 void *p_rng,
                 int mode, size_t *olen,
                 const unsigned char *input,
                 unsigned char *output,
                 size_t output_max_len)
{
  int ret;
  size_t ilen;
  size_t i;
  size_t plaintext_max_size;
  unsigned char buf[ESP32C3_MPI_MAX_SIZE];
  size_t pad_count = 0;
  unsigned bad = 0;
  unsigned char pad_done = 0;
  size_t plaintext_size = 0;
  unsigned output_too_large;

  DEBUGASSERT(ctx != NULL);
  DEBUGASSERT(mode == ESP32C3_RSA_PRIVATE ||
              mode == ESP32C3_RSA_PUBLIC);
  DEBUGASSERT(output_max_len == 0 || output != NULL);
  DEBUGASSERT(input != NULL);
  DEBUGASSERT(olen != NULL);

  ilen = ctx->len;
  plaintext_max_size = (output_max_len > ilen - 11 ?
               ilen - 11 :
               output_max_len);

  if (mode == ESP32C3_RSA_PRIVATE && ctx->padding != ESP32C3_RSA_PKCS_V15)
    {
      return (ESP32C3_ERR_RSA_BAD_INPUT_DATA);
    }

  if (ilen < 16 || ilen > sizeof(buf))
    {
      return (ESP32C3_ERR_RSA_BAD_INPUT_DATA);
    }

  ret = (mode == ESP32C3_RSA_PUBLIC)
      ? esp32c3_rsa_public(ctx, input, buf)
      : esp32c3_rsa_private(ctx, f_rng, p_rng, input, buf);

  if (ret != 0)
    {
      goto cleanup;
    }

  /* Check and get padding length in constant time */

  bad |= buf[0];

  if (mode == ESP32C3_RSA_PRIVATE)
    {
      /* Decode EME-PKCS1-v1_5 padding */

      bad |= buf[1] ^ ESP32C3_RSA_CRYPT;

      /* Read the whole buffer */

      for (i = 2; i < ilen; i++)
        {
          pad_done  |= ((buf[i] | (unsigned char)-buf[i]) >> 7) ^ 1;
          pad_count += ((pad_done | (unsigned char)-pad_done) >> 7) ^ 1;
        }
    }
  else
    {
      /* Decode EMSA-PKCS1-v1_5 padding */

      bad |= buf[1] ^ ESP32C3_RSA_SIGN;

      /* Read the whole buffer */

      for (i = 2; i < ilen; i++)
        {
          pad_done |= if_int(buf[i], 0, 1);
          pad_count += if_int(pad_done, 0, 1);
          bad |= if_int(pad_done, 0, buf[i] ^ 0xff);
        }
    }

  /* If pad_done is still zero, there's no data, only unfinished padding. */

  bad |= if_int(pad_done, 0, 1);

  /* There must be at least 8 bytes of padding. */

  bad |= size_greater_than(8, pad_count);
  plaintext_size = if_int(bad,
               (unsigned) plaintext_max_size,
               (unsigned) (ilen - pad_count - 3));
  output_too_large = size_greater_than(plaintext_size,
                      plaintext_max_size);

  /* Set ret without branches to avoid timing attacks */

  ret = - (int) if_int(bad, - ESP32C3_ERR_RSA_INVALID_PADDING,
          if_int(output_too_large, - ESP32C3_ERR_RSA_OUTPUT_TOO_LARGE,
              0));

  /* If the padding is bad or the plaintext is too large, zero the data */

  bad = all_or_nothing_int(bad | output_too_large);
  for (i = 11; i < ilen; i++)
    buf[i] &= ~bad;

  /* If the plaintext is too large, truncate it to the buffer size. */

  plaintext_size = if_int(output_too_large,
               (unsigned) plaintext_max_size,
               (unsigned) plaintext_size);

  /* Move the plaintext to the leftmost position */

  mem_move_to_left(buf + ilen - plaintext_max_size,
            plaintext_max_size,
            plaintext_max_size - plaintext_size);

  /* Copy the decrypted plaintext into the output buffer */

  memcpy(output, buf + ilen - plaintext_max_size, plaintext_max_size);

  /* Report the amount of data we copied to the output buffer */

  *olen = plaintext_size;

cleanup:
  memset(buf, 0, sizeof(buf));

  return ret;
}
#endif /* ESP32C3_PKCS1_V15 */

/****************************************************************************
 * Name: esp32c3_rsa_deduce_primes
 *
 * Description:
 *   Compute RSA prime moduli P, Q from public modulus N=PQ
 *   and a pair of private and public key.
 *
 * Input Parameters:
 *   N        - RSA modulus N = PQ, with P, Q to be found
 *   E        - RSA public exponent
 *   D        - RSA private exponent
 *   P        - Pointer to MPI holding first prime factor of N on success
 *   Q        - Pointer to MPI holding second prime factor of N on success
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static int esp32c3_rsa_deduce_primes(struct esp32c3_mpi_s const *N,
           struct esp32c3_mpi_s const *E, struct esp32c3_mpi_s const *D,
           struct esp32c3_mpi_s *P, struct esp32c3_mpi_s *Q)
{
  int ret = 0;

  uint16_t attempt;  /* Number of current attempt  */
  uint16_t iter;     /* Number of squares computed in the current attempt */

  uint16_t order;    /* Order of 2 in DE - 1 */

  struct esp32c3_mpi_s T;     /* Holds largest odd divisor of DE - 1   */
  struct esp32c3_mpi_s K;     /* Temporary holding the current candidate */

  const unsigned char primes[] =
  {
    2, 3,  5,  7,   11,   13,   17,   19,   23,
    29,   31,   37,   41,   43,   47,   53,   59,
    61,   67,   71,   73,   79,   83,   89,   97,
    101,  103,  107,  109,  113,  127,  131,  137,
    139,  149,  151,  157,  163,  167,  173,  179,
    181,  191,  193,  197,  199,  211,  223,  227,
    229,  233,  239,  241,  251
  };

  const size_t num_primes = sizeof(primes) / sizeof(*primes);

  if (P == NULL || Q == NULL || P->p != NULL || Q->p != NULL)
    {
      return (ESP32C3_ERR_MPI_BAD_INPUT_DATA);
    }

  if (esp32c3_mpi_cmp_int(N, 0) <= 0 ||
      esp32c3_mpi_cmp_int(D, 1) <= 0 ||
      esp32c3_mpi_cmp_mpi(D, N) >= 0 ||
      esp32c3_mpi_cmp_int(E, 1) <= 0 ||
      esp32c3_mpi_cmp_mpi(E, N) >= 0)
    {
      return (ESP32C3_ERR_MPI_BAD_INPUT_DATA);
    }

  /* Initializations and temporary changes */

  esp32c3_mpi_init(&K);
  esp32c3_mpi_init(&T);

  /* T := DE - 1 */

  ESP32C3_MPI_CHK(esp32c3_mpi_mul_mpi(&T, D,  E), cleanup);
  ESP32C3_MPI_CHK(esp32c3_mpi_sub_int(&T, &T, 1), cleanup);

  if ((order = (uint16_t) esp32c3_mpi_lsb(&T)) == 0)
    {
      ret = ESP32C3_ERR_MPI_BAD_INPUT_DATA;
      goto cleanup;
    }

  /* After this operation, T holds the largest odd divisor of DE - 1. */

  ESP32C3_MPI_CHK(esp32c3_mpi_shift_r(&T, order), cleanup);

  /* Skip trying 2 if N == 1 mod 8 */

  attempt = 0;
  if (N->p[0] % 8 == 1)
    {
        attempt = 1;
    }

  for (; attempt < num_primes; ++attempt)
    {
      esp32c3_mpi_lset(&K, primes[attempt]);

      /* Check if gcd(K,N) = 1 */

      ESP32C3_MPI_CHK(esp32c3_mpi_gcd(P, &K, N), cleanup);
      if (esp32c3_mpi_cmp_int(P, 1) != 0)
        {
          continue;
        }

      /* Go through K^T + 1, K^(2T) + 1, K^(4T) + 1, ... */

      ESP32C3_MPI_CHK(esp32c3_mpi_exp_mod(&K, &K, &T, N, Q), cleanup);

      for (iter = 1; iter <= order; ++iter)
        {
          /* Continuing to square K */

          if (esp32c3_mpi_cmp_int(&K, 1) == 0)
            {
              break;
            }

          ESP32C3_MPI_CHK(esp32c3_mpi_add_int(&K, &K, 1), cleanup);
          ESP32C3_MPI_CHK(esp32c3_mpi_gcd(P, &K, N), cleanup);

          if (esp32c3_mpi_cmp_int(P, 1) ==  1 &&
              esp32c3_mpi_cmp_mpi(P, N) == -1)
            {
              /* Q := N / P */

              ESP32C3_MPI_CHK(esp32c3_mpi_div_mpi(Q, NULL, N, P), cleanup);
              goto cleanup;
            }

          ESP32C3_MPI_CHK(esp32c3_mpi_sub_int(&K, &K, 1), cleanup);
          ESP32C3_MPI_CHK(esp32c3_mpi_mul_mpi(&K, &K, &K), cleanup);
          ESP32C3_MPI_CHK(esp32c3_mpi_mod_mpi(&K, &K, N), cleanup);
        }

      if (esp32c3_mpi_cmp_int(&K, 1) != 0)
        {
          break;
        }
    }

  ret = ESP32C3_ERR_MPI_BAD_INPUT_DATA;

cleanup:

  esp32c3_mpi_free(&K);
  esp32c3_mpi_free(&T);
  return ret;
}

/****************************************************************************
 * Name: esp32c3_rsa_deduce_private_exponent
 *
 * Description:
 *   Compute RSA private exponent from prime moduli and public key.
 *
 * Input Parameters:
 *   P        First prime factor of RSA modulus
 *   Q        Second prime factor of RSA modulus
 *   E        RSA public exponent
 *   D        Pointer to MPI holding the private exponent on success.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static int esp32c3_rsa_deduce_private_exponent(struct esp32c3_mpi_s const *P,
                     struct esp32c3_mpi_s const *Q,
                     struct esp32c3_mpi_s const *E,
                     struct esp32c3_mpi_s *D)
{
  int ret = 0;
  struct esp32c3_mpi_s K;
  struct esp32c3_mpi_s L;

  if (D == NULL || esp32c3_mpi_cmp_int(D, 0) != 0)
    {
      return (ESP32C3_ERR_MPI_BAD_INPUT_DATA);
    }

  if (esp32c3_mpi_cmp_int(P, 1) <= 0 ||
      esp32c3_mpi_cmp_int(Q, 1) <= 0 ||
      esp32c3_mpi_cmp_int(E, 0) == 0)
    {
      return (ESP32C3_ERR_MPI_BAD_INPUT_DATA);
    }

  esp32c3_mpi_init(&K);
  esp32c3_mpi_init(&L);

  /* Temporarily put K := P-1 and L := Q-1 */

  ESP32C3_MPI_CHK(esp32c3_mpi_sub_int(&K, P, 1), cleanup);
  ESP32C3_MPI_CHK(esp32c3_mpi_sub_int(&L, Q, 1), cleanup);

  /* Temporarily put D := gcd(P-1, Q-1) */

  ESP32C3_MPI_CHK(esp32c3_mpi_gcd(D, &K, &L), cleanup);

  /* K := LCM(P-1, Q-1) */

  ESP32C3_MPI_CHK(esp32c3_mpi_mul_mpi(&K, &K, &L), cleanup);
  ESP32C3_MPI_CHK(esp32c3_mpi_div_mpi(&K, NULL, &K, D), cleanup);

  /* Compute modular inverse of E in LCM(P-1, Q-1) */

  ESP32C3_MPI_CHK(esp32c3_mpi_inv_mod(D, E, &K), cleanup);

cleanup:

  esp32c3_mpi_free(&K);
  esp32c3_mpi_free(&L);

  return ret;
}

/****************************************************************************
 * Name: esp32c3_rsa_validate_crt
 *
 * Description:
 *   Check validity of core RSA parameters
 *
 * Input Parameters:
 *   P        - First prime factor of RSA modulus
 *   Q        - Second prime factor of RSA modulus
 *   D        - RSA private exponent
 *   DP       - MPI to check for D modulo P-1
 *   DQ       - MPI to check for D modulo P-1
 *   QP       - MPI to check for the modular inverse of Q modulo P.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static int esp32c3_rsa_validate_crt(
        const struct esp32c3_mpi_s *P,
        const struct esp32c3_mpi_s *Q,
        const struct esp32c3_mpi_s *D,
        const struct esp32c3_mpi_s *DP,
        const struct esp32c3_mpi_s *DQ,
        const struct esp32c3_mpi_s *QP)
{
  int ret = 0;

  struct esp32c3_mpi_s K;
  struct esp32c3_mpi_s L;

  esp32c3_mpi_init(&K);
  esp32c3_mpi_init(&L);

  /* Check that DP - D == 0 mod P - 1 */

  if (DP != NULL)
    {
      if (P == NULL)
        {
          ret = ESP32C3_ERR_RSA_BAD_INPUT_DATA;
          goto cleanup;
        }

      ESP32C3_MPI_CHK(esp32c3_mpi_sub_int(&K, P, 1), cleanup);
      ESP32C3_MPI_CHK(esp32c3_mpi_sub_mpi(&L, DP, D), cleanup);
      ESP32C3_MPI_CHK(esp32c3_mpi_mod_mpi(&L, &L, &K), cleanup);

      if (esp32c3_mpi_cmp_int(&L, 0) != 0)
        {
          ret = ESP32C3_ERR_RSA_KEY_CHECK_FAILED;
          goto cleanup;
        }
    }

  /* Check that DQ - D == 0 mod Q - 1 */

  if (DQ != NULL)
    {
      if (Q == NULL)
        {
          ret = ESP32C3_ERR_RSA_BAD_INPUT_DATA;
          goto cleanup;
        }

      ESP32C3_MPI_CHK(esp32c3_mpi_sub_int(&K, Q, 1), cleanup);
      ESP32C3_MPI_CHK(esp32c3_mpi_sub_mpi(&L, DQ, D), cleanup);
      ESP32C3_MPI_CHK(esp32c3_mpi_mod_mpi(&L, &L, &K), cleanup);

      if (esp32c3_mpi_cmp_int(&L, 0) != 0)
        {
          ret = ESP32C3_ERR_RSA_KEY_CHECK_FAILED;
          goto cleanup;
        }
    }

  /* Check that QP * Q - 1 == 0 mod P */

  if (QP != NULL)
    {
      if (P == NULL || Q == NULL)
        {
          ret = ESP32C3_ERR_RSA_BAD_INPUT_DATA;
          goto cleanup;
        }

      ESP32C3_MPI_CHK(esp32c3_mpi_mul_mpi(&K, QP, Q), cleanup);
      ESP32C3_MPI_CHK(esp32c3_mpi_sub_int(&K, &K, 1), cleanup);
      ESP32C3_MPI_CHK(esp32c3_mpi_mod_mpi(&K, &K, P), cleanup);
      if (esp32c3_mpi_cmp_int(&K, 0) != 0)
        {
          ret = ESP32C3_ERR_RSA_KEY_CHECK_FAILED;
          goto cleanup;
        }
    }

cleanup:

  /* Wrap MPI error codes by RSA check failure error code */

  if (ret != 0 &&
      ret != ESP32C3_ERR_RSA_KEY_CHECK_FAILED &&
      ret != ESP32C3_ERR_RSA_BAD_INPUT_DATA)
    {
      ret += ESP32C3_ERR_RSA_KEY_CHECK_FAILED;
    }

  esp32c3_mpi_free(&K);
  esp32c3_mpi_free(&L);

  return ret;
}

/****************************************************************************
 * Name: esp32c3_rsa_validate_params
 *
 * Description:
 *   Check validity of core RSA parameters
 *
 * Input Parameters:
 *   N        - RSA modulus N = PQ
 *   P        - First prime factor of N
 *   Q        - Second prime factor of N
 *   D        - RSA private exponent
 *   E        - RSA public exponent
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static int esp32c3_rsa_validate_params(const struct esp32c3_mpi_s *N,
                 const struct esp32c3_mpi_s *P,
                 const struct esp32c3_mpi_s *Q,
                 const struct esp32c3_mpi_s *D,
                 const struct esp32c3_mpi_s *E,
                 int (*f_rng)(void *, unsigned char *, size_t),
                 void *p_rng)
{
  int ret = 0;
  struct esp32c3_mpi_s K;
  struct esp32c3_mpi_s L;

  esp32c3_mpi_init(&K);
  esp32c3_mpi_init(&L);

  /* Step 1: Check that 1 < N = P * Q */

  if (P != NULL && Q != NULL && N != NULL)
    {
      ESP32C3_MPI_CHK(esp32c3_mpi_mul_mpi(&K, P, Q), cleanup);
      if (esp32c3_mpi_cmp_int(N, 1)  <= 0 ||
          esp32c3_mpi_cmp_mpi(&K, N) != 0)
        {
          ret = ESP32C3_ERR_RSA_KEY_CHECK_FAILED;
          goto cleanup;
        }
    }

  /* Step 2: Check and 1 < D, E < N if present */

  if (N != NULL && D != NULL && E != NULL)
    {
      if (esp32c3_mpi_cmp_int(D, 1) <= 0 ||
          esp32c3_mpi_cmp_int(E, 1) <= 0 ||
          esp32c3_mpi_cmp_mpi(D, N) >= 0 ||
          esp32c3_mpi_cmp_mpi(E, N) >= 0)
        {
          ret = ESP32C3_ERR_RSA_KEY_CHECK_FAILED;
          goto cleanup;
        }
    }

  /* Step 3: Check that D, E are inverse modulo P-1 and Q-1 */

  if (P != NULL && Q != NULL && D != NULL && E != NULL)
    {
      if (esp32c3_mpi_cmp_int(P, 1) <= 0 ||
          esp32c3_mpi_cmp_int(Q, 1) <= 0)
        {
          ret = ESP32C3_ERR_RSA_KEY_CHECK_FAILED;
          goto cleanup;
        }

      /* Compute DE-1 mod P-1 */

      ESP32C3_MPI_CHK(esp32c3_mpi_mul_mpi(&K, D, E), cleanup);
      ESP32C3_MPI_CHK(esp32c3_mpi_sub_int(&K, &K, 1), cleanup);
      ESP32C3_MPI_CHK(esp32c3_mpi_sub_int(&L, P, 1), cleanup);
      ESP32C3_MPI_CHK(esp32c3_mpi_mod_mpi(&K, &K, &L), cleanup);
      if (esp32c3_mpi_cmp_int(&K, 0) != 0)
        {
          ret = ESP32C3_ERR_RSA_KEY_CHECK_FAILED;
          goto cleanup;
        }

      /* Compute DE-1 mod Q-1 */

      ESP32C3_MPI_CHK(esp32c3_mpi_mul_mpi(&K, D, E), cleanup);
      ESP32C3_MPI_CHK(esp32c3_mpi_sub_int(&K, &K, 1), cleanup);
      ESP32C3_MPI_CHK(esp32c3_mpi_sub_int(&L, Q, 1), cleanup);
      ESP32C3_MPI_CHK(esp32c3_mpi_mod_mpi(&K, &K, &L), cleanup);
      if (esp32c3_mpi_cmp_int(&K, 0) != 0)
        {
          ret = ESP32C3_ERR_RSA_KEY_CHECK_FAILED;
          goto cleanup;
        }
    }

cleanup:

  esp32c3_mpi_free(&K);
  esp32c3_mpi_free(&L);

  /* Wrap MPI error codes by RSA check failure error code */

  if (ret != 0 && ret != ESP32C3_ERR_RSA_KEY_CHECK_FAILED)
    {
      ret += ESP32C3_ERR_RSA_KEY_CHECK_FAILED;
    }

  return ret;
}

/****************************************************************************
 * Name: esp32c3_rsa_deduce_crt
 *
 * Description:
 *   Generate RSA-CRT parameters
 *
 * Input Parameters:
 *   P        - First prime factor of N
 *   Q        - Second prime factor of N
 *   D        - RSA private exponent
 *   DP       - Output variable for D modulo P-1
 *   DQ       - Output variable for D modulo Q-1
 *   QP       - Output variable for the modular inverse of Q modulo P
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static int esp32c3_rsa_deduce_crt(const struct esp32c3_mpi_s *P,
                                  const struct esp32c3_mpi_s *Q,
                                  const struct esp32c3_mpi_s *D,
                                  struct esp32c3_mpi_s *DP,
                                  struct esp32c3_mpi_s *DQ,
                                  struct esp32c3_mpi_s *QP)
{
  int ret = 0;
  struct esp32c3_mpi_s K;
  esp32c3_mpi_init(&K);

  /* DP = D mod P-1 */

  if (DP != NULL)
    {
      ESP32C3_MPI_CHK(esp32c3_mpi_sub_int(&K, P, 1), cleanup);
      ESP32C3_MPI_CHK(esp32c3_mpi_mod_mpi(DP, D, &K), cleanup);
    }

  /* DQ = D mod Q-1 */

  if (DQ != NULL)
    {
      ESP32C3_MPI_CHK(esp32c3_mpi_sub_int(&K, Q, 1), cleanup);
      ESP32C3_MPI_CHK(esp32c3_mpi_mod_mpi(DQ, D, &K), cleanup);
    }

  /* QP = Q^{-1} mod P */

  if (QP != NULL)
    {
      ESP32C3_MPI_CHK(esp32c3_mpi_inv_mod(QP, Q, P), cleanup);
    }

cleanup:
  esp32c3_mpi_free(&K);

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32c3_rsa_import
 *
 * Description:
 *   Imports a set of core parameters into an RSA context.
 *
 * Input Parameters:
 *   ctx  - The initialized RSA context to store the parameters in
 *   N    - The RSA modulus
 *   P    - The first prime factor of \p N
 *   Q    - The second prime factor of \p N
 *   D    - The private exponent
 *   E    - The public exponent
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_rsa_import(struct esp32c3_rsa_context_s *ctx,
            const struct esp32c3_mpi_s *N,
            const struct esp32c3_mpi_s *P, const struct esp32c3_mpi_s *Q,
            const struct esp32c3_mpi_s *D, const struct esp32c3_mpi_s *E)
{
  int ret;
  DEBUGASSERT(ctx != NULL);

  if ((N != NULL && (ret = esp32c3_mpi_copy(&ctx->N, N)) != 0) ||
     (P != NULL && (ret = esp32c3_mpi_copy(&ctx->P, P)) != 0) ||
     (Q != NULL && (ret = esp32c3_mpi_copy(&ctx->Q, Q)) != 0) ||
     (D != NULL && (ret = esp32c3_mpi_copy(&ctx->D, D)) != 0) ||
     (E != NULL && (ret = esp32c3_mpi_copy(&ctx->E, E)) != 0))
    {
      return (ESP32C3_ERR_RSA_BAD_INPUT_DATA + ret);
    }

  if (N != NULL)
    {
      ctx->len = esp32c3_mpi_size(&ctx->N);
    }

  return OK;
}

/****************************************************************************
 * Name: esp32c3_rsa_import_raw
 *
 * Description:
 *   Imports core RSA parameters into an RSA context.
 *
 * Input Parameters:
 *   ctx      - The initialized RSA context to store the parameters in
 *   N        - The RSA modulus
 *   NL    - The Byte length of \p N
 *   P        - The first prime factor of \p N
 *   PL    - The Byte length of \p P
 *   Q        - The second prime factor of \p N
 *   QL    - The Byte length of \p Q
 *   D        - The private exponent
 *   DL    - The Byte length of \p D
 *   E        - The public exponent
 *   EL    - The Byte length of \p E
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_rsa_import_raw(struct esp32c3_rsa_context_s *ctx,
              unsigned char const *N, size_t NL,
              unsigned char const *P, size_t PL,
              unsigned char const *Q, size_t QL,
              unsigned char const *D, size_t DL,
              unsigned char const *E, size_t EL)
{
  int ret = 0;
  DEBUGASSERT(ctx != NULL);

  if (N != NULL)
    {
      ESP32C3_MPI_CHK(esp32c3_mpi_read_binary(&ctx->N, N, NL), cleanup);
      ctx->len = esp32c3_mpi_size(&ctx->N);
    }

  if (P != NULL)
    {
      ESP32C3_MPI_CHK(esp32c3_mpi_read_binary(&ctx->P, P, PL), cleanup);
    }

  if (Q != NULL)
    {
      ESP32C3_MPI_CHK(esp32c3_mpi_read_binary(&ctx->Q, Q, QL), cleanup);
    }

  if (D != NULL)
    {
      ESP32C3_MPI_CHK(esp32c3_mpi_read_binary(&ctx->D, D, DL), cleanup);
    }

  if (E != NULL)
    {
      ESP32C3_MPI_CHK(esp32c3_mpi_read_binary(&ctx->E, E, EL), cleanup);
    }

cleanup:

  if (ret != 0)
    {
      return (ESP32C3_ERR_RSA_BAD_INPUT_DATA + ret);
    }

  return OK;
}

/****************************************************************************
 * Name: esp32c3_rsa_complete
 *
 * Description:
 *   Completes an RSA context from a set of imported core parameters.
 *
 * Input Parameters:
 *   ctx      - The initialized RSA context holding imported parameters
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_rsa_complete(struct esp32c3_rsa_context_s *ctx)
{
  int ret = 0;
  int have_n;
  int have_p;
  int have_q;
  int have_d;
  int have_e;
#if !defined(ESP32C3_RSA_NO_CRT)
  int have_dp;
  int have_dq;
  int have_qp;
#endif
  int n_missing;
  int pq_missing;
  int d_missing;
  int is_pub;
  int is_priv;

  DEBUGASSERT(ctx != NULL);

  have_n = (esp32c3_mpi_cmp_int(&ctx->N, 0) != 0);
  have_p = (esp32c3_mpi_cmp_int(&ctx->P, 0) != 0);
  have_q = (esp32c3_mpi_cmp_int(&ctx->Q, 0) != 0);
  have_d = (esp32c3_mpi_cmp_int(&ctx->D, 0) != 0);
  have_e = (esp32c3_mpi_cmp_int(&ctx->E, 0) != 0);

#if !defined(ESP32C3_RSA_NO_CRT)
  have_dp = (esp32c3_mpi_cmp_int(&ctx->DP, 0) != 0);
  have_dq = (esp32c3_mpi_cmp_int(&ctx->DQ, 0) != 0);
  have_qp = (esp32c3_mpi_cmp_int(&ctx->QP, 0) != 0);
#endif

  /* Check whether provided parameters are enough */

  n_missing  =  have_p &&  have_q &&  have_d && have_e;
  pq_missing =  have_n && !have_p && !have_q &&  have_d && have_e;
  d_missing  =  have_p &&  have_q && !have_d && have_e;
  is_pub     =  have_n && !have_p && !have_q && !have_d && have_e;

  /* These three alternatives are mutually exclusive */

  is_priv = n_missing || pq_missing || d_missing;

  if (!is_priv && !is_pub)
    {
      return (ESP32C3_ERR_RSA_BAD_INPUT_DATA);
    }

  /* Step 1: Deduce N if P, Q are provided */

  if (!have_n && have_p && have_q)
    {
      if ((ret = esp32c3_mpi_mul_mpi(&ctx->N, &ctx->P,
                      &ctx->Q)) != 0)
        {
          return (ESP32C3_ERR_RSA_BAD_INPUT_DATA + ret);
        }

      ctx->len = esp32c3_mpi_size(&ctx->N);
    }

  /* Step 2: Deduce and verify all remaining core parameters */

  if (pq_missing)
    {
      ret = esp32c3_rsa_deduce_primes(&ctx->N, &ctx->E, &ctx->D,
                                      &ctx->P, &ctx->Q);
      if (ret != 0)
        {
          return (ESP32C3_ERR_RSA_BAD_INPUT_DATA + ret);
        }
    }
  else if (d_missing)
    {
      if ((ret = esp32c3_rsa_deduce_private_exponent(&ctx->P, &ctx->Q,
                                                     &ctx->E, &ctx->D)) != 0)
        {
          return (ESP32C3_ERR_RSA_BAD_INPUT_DATA + ret);
        }
    }

  /* Step 3: Deduce all additional parameters to RSA implementation */

#if !defined(ESP32C3_RSA_NO_CRT)
  if (is_priv && ! (have_dp && have_dq && have_qp))
    {
      ret = esp32c3_rsa_deduce_crt(&ctx->P, &ctx->Q, &ctx->D,
                                   &ctx->DP, &ctx->DQ, &ctx->QP);
      if (ret != 0)
        {
          return (ESP32C3_ERR_RSA_BAD_INPUT_DATA + ret);
        }
    }
#endif /* ESP32C3_RSA_NO_CRT */

  /* Step 3: Basic sanity checks */

  return (rsa_check_context(ctx, is_priv, 1));
}

/****************************************************************************
 * Name: esp32c3_rsa_export_raw
 *
 * Description:
 *   Eexports core parameters of an RSA key in raw big-endian binary format.
 *
 * Input Parameters:
 *   ctx      - The initialized RSA context
 *   N        - The Byte array to store the RSA modulus
 *   NL    - The size of the buffer for the modulus
 *   P        - The Byte array to hold the first prime factor of \p N
 *   PL    - The size of the buffer for the first prime factor
 *   Q        - The Byte array to hold the second prime factor of \p N
 *   QL    - The size of the buffer for the second prime factor
 *   D        - The Byte array to hold the private exponent
 *   DL    - The size of the buffer for the private exponent
 *   E        - The Byte array to hold the public exponent
 *   EL    - The size of the buffer for the public exponent
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_rsa_export_raw(const struct esp32c3_rsa_context_s *ctx,
              unsigned char *N, size_t NL,
              unsigned char *P, size_t PL,
              unsigned char *Q, size_t QL,
              unsigned char *D, size_t DL,
              unsigned char *E, size_t EL)
{
  int ret = 0;
  int is_priv;
  DEBUGASSERT(ctx != NULL);

  /* Check if key is private or public */

  is_priv = esp32c3_mpi_cmp_int(&ctx->N, 0) != 0 &&
            esp32c3_mpi_cmp_int(&ctx->P, 0) != 0 &&
            esp32c3_mpi_cmp_int(&ctx->Q, 0) != 0 &&
            esp32c3_mpi_cmp_int(&ctx->D, 0) != 0 &&
            esp32c3_mpi_cmp_int(&ctx->E, 0) != 0;

  if (!is_priv)
    {
      /* It can't try to export private parameters for a public key */

      if (P != NULL || Q != NULL || D != NULL)
        {
          return (ESP32C3_ERR_RSA_BAD_INPUT_DATA);
        }
    }

  if (N != NULL)
    {
      ESP32C3_MPI_CHK(esp32c3_mpi_write_binary(&ctx->N, N, NL), cleanup);
    }

  if (P != NULL)
    {
      ESP32C3_MPI_CHK(esp32c3_mpi_write_binary(&ctx->P, P, PL), cleanup);
    }

  if (Q != NULL)
    {
      ESP32C3_MPI_CHK(esp32c3_mpi_write_binary(&ctx->Q, Q, QL), cleanup);
    }

  if (D != NULL)
    {
      ESP32C3_MPI_CHK(esp32c3_mpi_write_binary(&ctx->D, D, DL), cleanup);
    }

  if (E != NULL)
    {
      ESP32C3_MPI_CHK(esp32c3_mpi_write_binary(&ctx->E, E, EL), cleanup);
    }

cleanup:

  return ret;
}

/****************************************************************************
 * Name: esp32c3_rsa_export
 *
 * Description:
 *   Exports the core parameters of an RSA key.
 *
 * Input Parameters:
 *   ctx      - The initialized RSA context
 *   N        - The MPI to hold the RSA modulus
 *   P        - The MPI to hold the first prime factor of \p N
 *   Q        - The MPI to hold the second prime factor of \p N
 *   D        - The MPI to hold the private exponent
 *   E        - The MPI to hold the public exponent
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_rsa_export(const struct esp32c3_rsa_context_s *ctx,
                       struct esp32c3_mpi_s *N,
                       struct esp32c3_mpi_s *P,
                       struct esp32c3_mpi_s *Q,
                       struct esp32c3_mpi_s *D,
                       struct esp32c3_mpi_s *E)
{
  int ret;
  int is_priv;
  DEBUGASSERT(ctx != NULL);

  /* Check if key is private or public */

  is_priv =
    esp32c3_mpi_cmp_int(&ctx->N, 0) != 0 &&
    esp32c3_mpi_cmp_int(&ctx->P, 0) != 0 &&
    esp32c3_mpi_cmp_int(&ctx->Q, 0) != 0 &&
    esp32c3_mpi_cmp_int(&ctx->D, 0) != 0 &&
    esp32c3_mpi_cmp_int(&ctx->E, 0) != 0;

  if (!is_priv)
    {
      /* It can't try to export private parameters for a public key */

      if (P != NULL || Q != NULL || D != NULL)
        {
          return (ESP32C3_ERR_RSA_BAD_INPUT_DATA);
        }
    }

  /* Export all requested core parameters. */

  if ((N != NULL && (ret = esp32c3_mpi_copy(N, &ctx->N)) != 0) ||
      (P != NULL && (ret = esp32c3_mpi_copy(P, &ctx->P)) != 0) ||
      (Q != NULL && (ret = esp32c3_mpi_copy(Q, &ctx->Q)) != 0) ||
      (D != NULL && (ret = esp32c3_mpi_copy(D, &ctx->D)) != 0) ||
      (E != NULL && (ret = esp32c3_mpi_copy(E, &ctx->E)) != 0))
    {
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: esp32c3_rsa_export_crt
 *
 * Description:
 *   Exports CRT parameters of a private RSA key.
 *
 * Input Parameters:
 *   ctx      - The initialized RSA context
 *   DP       - The MPI to hold \c D modulo `P-1`
 *   DQ       - The MPI to hold \c D modulo `Q-1`
 *   QP       - The MPI to hold modular inverse of \c Q modulo \c P
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_rsa_export_crt(const struct esp32c3_rsa_context_s *ctx,
                           struct esp32c3_mpi_s *DP,
                           struct esp32c3_mpi_s *DQ,
                           struct esp32c3_mpi_s *QP)
{
  int ret;
  int is_priv;
  DEBUGASSERT(ctx != NULL);

  /* Check if key is private or public */

  is_priv = esp32c3_mpi_cmp_int(&ctx->N, 0) != 0 &&
            esp32c3_mpi_cmp_int(&ctx->P, 0) != 0 &&
            esp32c3_mpi_cmp_int(&ctx->Q, 0) != 0 &&
            esp32c3_mpi_cmp_int(&ctx->D, 0) != 0 &&
            esp32c3_mpi_cmp_int(&ctx->E, 0) != 0;

  if (!is_priv)
    {
      return (ESP32C3_ERR_RSA_BAD_INPUT_DATA);
    }

#if !defined(ESP32C3_RSA_NO_CRT)
  /* Export all requested blinding parameters. */

  if ((DP != NULL && (ret = esp32c3_mpi_copy(DP, &ctx->DP)) != 0) ||
      (DQ != NULL && (ret = esp32c3_mpi_copy(DQ, &ctx->DQ)) != 0) ||
      (QP != NULL && (ret = esp32c3_mpi_copy(QP, &ctx->QP)) != 0))
    {
      return (ESP32C3_ERR_RSA_BAD_INPUT_DATA + ret);
    }
#else
  if ((ret = esp32c3_rsa_deduce_crt(&ctx->P, &ctx->Q, &ctx->D,
                                    DP, DQ, QP)) != 0)
    {
      return (ESP32C3_ERR_RSA_BAD_INPUT_DATA + ret);
    }
#endif

  return OK;
}

/****************************************************************************
 * Name: esp32c3_rsa_init
 *
 * Description:
 *   Initializes an RSA context
 *
 * Input Parameters:
 *   ctx      - The RSA context to initialize
 *   padding  - The padding mode to use
 *   hash_id  - The hash identifier of
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32c3_rsa_init(struct esp32c3_rsa_context_s *ctx,
         int padding,
         int hash_id)
{
  DEBUGASSERT(ctx != NULL);
  DEBUGASSERT(padding == ESP32C3_RSA_PKCS_V15 ||
              padding == ESP32C3_RSA_PKCS_V21);

  memset(ctx, 0, sizeof(struct esp32c3_rsa_context_s));

  esp32c3_rsa_set_padding(ctx, padding, hash_id);
}

/****************************************************************************
 * Name: esp32c3_rsa_set_padding
 *
 * Description:
 *   Sets padding for an already initialized RSA context.
 *
 * Input Parameters:
 *   ctx      - The initialized RSA context to be configured
 *   padding  - The padding mode to use
 *   hash_id  - The hash identifier
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32c3_rsa_set_padding(struct esp32c3_rsa_context_s *ctx,
                             int padding, int hash_id)
{
  DEBUGASSERT(ctx != NULL);
  DEBUGASSERT(padding == ESP32C3_RSA_PKCS_V15 ||
              padding == ESP32C3_RSA_PKCS_V21);

  ctx->padding = padding;
  ctx->hash_id = hash_id;
}

/****************************************************************************
 * Name: esp32c3_rsa_get_len
 *
 * Description:
 *   Exports CRT parameters of a private RSA key.
 *
 * Input Parameters:
 *   ctx      - The initialized RSA context
 *
 * Returned Value:
 *   length of the RSA modulus in Bytes.
 *
 ****************************************************************************/

size_t esp32c3_rsa_get_len(const struct esp32c3_rsa_context_s *ctx)
{
  return (ctx->len);
}

/****************************************************************************
 * Name: esp32c3_rsa_check_pubkey
 *
 * Description:
 *   checks if a context contains at least an RSA public key..
 *
 * Input Parameters:
 *   ctx      - The initialized RSA context to check
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_rsa_check_pubkey(const struct esp32c3_rsa_context_s *ctx)
{
  DEBUGASSERT(ctx != NULL);

  if (rsa_check_context(ctx, 0 /* public */, 0 /* no blinding */) != 0)
    {
      return (ESP32C3_ERR_RSA_KEY_CHECK_FAILED);
    }

  if (esp32c3_mpi_bitlen(&ctx->N) < 128)
    {
      return (ESP32C3_ERR_RSA_KEY_CHECK_FAILED);
    }

  if (esp32c3_mpi_get_bit(&ctx->E, 0) == 0 ||
      esp32c3_mpi_bitlen(&ctx->E)   < 2  ||
      esp32c3_mpi_cmp_mpi(&ctx->E, &ctx->N) >= 0)
    {
      return (ESP32C3_ERR_RSA_KEY_CHECK_FAILED);
    }

  return OK;
}

/****************************************************************************
 * Name: esp32c3_rsa_check_privkey
 *
 * Description:
 *   Checks if a context contains at least an RSA private key
 *   and perform basic consistency checks.
 *
 * Input Parameters:
 *   ctx      - The initialized RSA context to check
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_rsa_check_privkey(const struct esp32c3_rsa_context_s *ctx)
{
  DEBUGASSERT(ctx != NULL);

  if (esp32c3_rsa_check_pubkey(ctx) != 0 ||
      rsa_check_context(ctx, 1 /* private */, 1 /* blinding */) != 0)
    {
      return (ESP32C3_ERR_RSA_KEY_CHECK_FAILED);
    }

  if (esp32c3_rsa_validate_params(&ctx->N, &ctx->P, &ctx->Q,
                                  &ctx->D, &ctx->E, NULL, NULL) != 0)
    {
      return (ESP32C3_ERR_RSA_KEY_CHECK_FAILED);
    }

#if !defined(ESP32C3_RSA_NO_CRT)
  else if (esp32c3_rsa_validate_crt(&ctx->P, &ctx->Q, &ctx->D,
                                    &ctx->DP, &ctx->DQ, &ctx->QP) != 0)
    {
      return (ESP32C3_ERR_RSA_KEY_CHECK_FAILED);
    }
#endif

  return OK;
}

/****************************************************************************
 * Name: esp32c3_rsa_check_pub_priv
 *
 * Description:
 *   Checks a public-private RSA key pair. It checks each of the contexts,
 *   and makes sure they match.
 *
 * Input Parameters:
 *   pub      - The initialized RSA context holding the public key
 *   prv      - The initialized RSA context holding the private key
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_rsa_check_pub_priv(const struct esp32c3_rsa_context_s *pub,
                const struct esp32c3_rsa_context_s *prv)
{
  DEBUGASSERT(pub != NULL);
  DEBUGASSERT(prv != NULL);

  if (esp32c3_rsa_check_pubkey(pub)  != 0 ||
      esp32c3_rsa_check_privkey(prv) != 0)
    {
      return (ESP32C3_ERR_RSA_KEY_CHECK_FAILED);
    }

  if (esp32c3_mpi_cmp_mpi(&pub->N, &prv->N) != 0 ||
      esp32c3_mpi_cmp_mpi(&pub->E, &prv->E) != 0)
    {
      return (ESP32C3_ERR_RSA_KEY_CHECK_FAILED);
    }

  return OK;
}

/****************************************************************************
 * Name: esp32c3_rsa_public
 *
 * Description:
 *   Performs an RSA public key operation.
 *
 * Input Parameters:
 *   ctx      - The initialized RSA context to use
 *   input    - The input buffer
 *   output   - The output buffer
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_rsa_public(struct esp32c3_rsa_context_s *ctx,
        const unsigned char *input,
        unsigned char *output)
{
  int ret;
  size_t olen;
  struct esp32c3_mpi_s T;
  DEBUGASSERT(ctx != NULL);
  DEBUGASSERT(input != NULL);
  DEBUGASSERT(output != NULL);

  if (rsa_check_context(ctx, 0 /* public */, 0 /* no blinding */))
    {
      return (ESP32C3_ERR_RSA_BAD_INPUT_DATA);
    }

  esp32c3_mpi_init(&T);

  ESP32C3_MPI_CHK(esp32c3_mpi_read_binary(&T, input, ctx->len), cleanup);

  if (esp32c3_mpi_cmp_mpi(&T, &ctx->N) >= 0)
    {
      ret = ESP32C3_ERR_MPI_BAD_INPUT_DATA;
      goto cleanup;
    }

  olen = ctx->len;
  ESP32C3_MPI_CHK(esp32c3_mpi_exp_mod(&T, &T, &ctx->E, &ctx->N, &ctx->RN),
                  cleanup);
  ESP32C3_MPI_CHK(esp32c3_mpi_write_binary(&T, output, olen), cleanup);

cleanup:
  esp32c3_mpi_free(&T);

  if (ret != 0)
    {
      return (ESP32C3_ERR_RSA_PUBLIC_FAILED + ret);
    }

  return OK;
}

/****************************************************************************
 * Name: esp32c3_rsa_private
 *
 * Description:
 *   Performs an RSA private key operation.
 *
 * Input Parameters:
 *   ctx      - The initialized RSA context to use
 *   f_rng    - The RNG function
 *   p_rng    - The RNG context to pass to \p f_rng
 *   input    - The input buffer
 *   output   - The output buffer
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_rsa_private(struct esp32c3_rsa_context_s *ctx,
         int (*f_rng)(void *, unsigned char *, size_t),
         void *p_rng,
         const unsigned char *input,
         unsigned char *output)
{
  int ret;
  size_t olen;

  /* Temporary holding the result */

  struct esp32c3_mpi_s T;

  /* Temporaries holding P-1, Q-1 and the exponent blinding factor */

  struct esp32c3_mpi_s P1, Q1, R;

#if !defined(ESP32C3_RSA_NO_CRT)
  /* Temporaries holding the results mod p resp. mod q. */

  struct esp32c3_mpi_s TP, TQ;

  /* Temporaries holding the blinded exponents */

  struct esp32c3_mpi_s dp_blind, dq_blind;

  /* Pointers to actual exponents to be used */

  struct esp32c3_mpi_s *DP = &ctx->DP;
  struct esp32c3_mpi_s *DQ = &ctx->DQ;
#else
  /* Temporary holding the blinded exponent */

  struct esp32c3_mpi_s d_blind;

  /* Pointer to actual exponent to be used */

  struct esp32c3_mpi_s *D = &ctx->D;
#endif /* ESP32C3_RSA_NO_CRT */

  /* Temporaries holding the initial input */

  struct esp32c3_mpi_s I, C;

  DEBUGASSERT(ctx != NULL);
  DEBUGASSERT(input  != NULL);
  DEBUGASSERT(output != NULL);

  if (rsa_check_context(ctx, 1, f_rng != NULL) != 0)
    {
      return (ESP32C3_ERR_RSA_BAD_INPUT_DATA);
    }

  /* MPI Initialization */

  esp32c3_mpi_init(&T);

  esp32c3_mpi_init(&P1);
  esp32c3_mpi_init(&Q1);
  esp32c3_mpi_init(&R);

  if (f_rng != NULL)
    {
#if defined(ESP32C3_RSA_NO_CRT)
      esp32c3_mpi_init(&d_blind);
#else
      esp32c3_mpi_init(&dp_blind);
      esp32c3_mpi_init(&dq_blind);
#endif
    }

#if !defined(ESP32C3_RSA_NO_CRT)
  esp32c3_mpi_init(&TP); esp32c3_mpi_init(&TQ);
#endif

  esp32c3_mpi_init(&I);
  esp32c3_mpi_init(&C);

  /* End of MPI initialization */

  ESP32C3_MPI_CHK(esp32c3_mpi_read_binary(&T, input, ctx->len), cleanup);
  if (esp32c3_mpi_cmp_mpi(&T, &ctx->N) >= 0)
    {
      ret = ESP32C3_ERR_MPI_BAD_INPUT_DATA;
      goto cleanup;
    }

  ESP32C3_MPI_CHK(esp32c3_mpi_copy(&I, &T), cleanup);

  if (f_rng != NULL)
    {
      /* Blinding T = T * VI mod N */

      ESP32C3_MPI_CHK(rsa_prepare_blinding(ctx, f_rng, p_rng), cleanup);
      ESP32C3_MPI_CHK(esp32c3_mpi_mul_mpi(&T, &T, &ctx->VI), cleanup);
      ESP32C3_MPI_CHK(esp32c3_mpi_mod_mpi(&T, &T, &ctx->N), cleanup);

      /* Exponent blinding */

      ESP32C3_MPI_CHK(esp32c3_mpi_sub_int(&P1, &ctx->P, 1), cleanup);
      ESP32C3_MPI_CHK(esp32c3_mpi_sub_int(&Q1, &ctx->Q, 1), cleanup);

#if defined(ESP32C3_RSA_NO_CRT)
      /* d_blind = (P - 1) * (Q - 1) * R + D */

      ESP32C3_MPI_CHK(esp32c3_mpi_fill_random(&R, RSA_EXPONENT_BLINDING,
                      f_rng, p_rng), cleanup);
      ESP32C3_MPI_CHK(esp32c3_mpi_mul_mpi(&d_blind, &P1, &Q1), cleanup);
      ESP32C3_MPI_CHK(esp32c3_mpi_mul_mpi(&d_blind, &d_blind, &R), cleanup);
      ESP32C3_MPI_CHK(esp32c3_mpi_add_mpi(&d_blind, &d_blind, &ctx->D),
                      cleanup);

      D = &d_blind;
#else
      /* dp_blind = (P - 1) * R + DP */

      ESP32C3_MPI_CHK(esp32c3_mpi_fill_random(&R, RSA_EXPONENT_BLINDING,
                      f_rng, p_rng), cleanup);
      ESP32C3_MPI_CHK(esp32c3_mpi_mul_mpi(&dp_blind, &P1, &R), cleanup);
      ESP32C3_MPI_CHK(esp32c3_mpi_add_mpi(&dp_blind, &dp_blind,
                      &ctx->DP), cleanup);

      DP = &dp_blind;

      /* dq_blind = (Q - 1) * R + DQ */

      ESP32C3_MPI_CHK(esp32c3_mpi_fill_random(&R, RSA_EXPONENT_BLINDING,
                      f_rng, p_rng), cleanup);
      ESP32C3_MPI_CHK(esp32c3_mpi_mul_mpi(&dq_blind, &Q1, &R), cleanup);
      ESP32C3_MPI_CHK(esp32c3_mpi_add_mpi(&dq_blind, &dq_blind,
                      &ctx->DQ), cleanup);

      DQ = &dq_blind;
#endif /* ESP32C3_RSA_NO_CRT */
    }

#if defined(ESP32C3_RSA_NO_CRT)
  ESP32C3_MPI_CHK(esp32c3_mpi_exp_mod(&T, &T, D, &ctx->N, &ctx->RN),
                  cleanup);
#else
  /* TP = input ^ dP mod P and TQ = input ^ dQ mod Q */

  ESP32C3_MPI_CHK(esp32c3_mpi_exp_mod(&TP, &T, DP, &ctx->P, &ctx->RP),
                  cleanup);
  ESP32C3_MPI_CHK(esp32c3_mpi_exp_mod(&TQ, &T, DQ, &ctx->Q, &ctx->RQ),
                  cleanup);

  /* T = (TP - TQ) * (Q^-1 mod P) mod P */

  ESP32C3_MPI_CHK(esp32c3_mpi_sub_mpi(&T, &TP, &TQ), cleanup);
  ESP32C3_MPI_CHK(esp32c3_mpi_mul_mpi(&TP, &T, &ctx->QP), cleanup);
  ESP32C3_MPI_CHK(esp32c3_mpi_mod_mpi(&T, &TP, &ctx->P), cleanup);

  /* T = TQ + T * Q */

  ESP32C3_MPI_CHK(esp32c3_mpi_mul_mpi(&TP, &T, &ctx->Q), cleanup);
  ESP32C3_MPI_CHK(esp32c3_mpi_add_mpi(&T, &TQ, &TP), cleanup);
#endif /* ESP32C3_RSA_NO_CRT */

  if (f_rng != NULL)
    {
      /* Unblind T = T * VF mod N */

      ESP32C3_MPI_CHK(esp32c3_mpi_mul_mpi(&T, &T, &ctx->VF), cleanup);
      ESP32C3_MPI_CHK(esp32c3_mpi_mod_mpi(&T, &T, &ctx->N), cleanup);
    }

  /* Verify the result to prevent glitching attacks. */

  ESP32C3_MPI_CHK(esp32c3_mpi_exp_mod(&C, &T, &ctx->E,
                  &ctx->N, &ctx->RN), cleanup);
  if (esp32c3_mpi_cmp_mpi(&C, &I) != 0)
    {
      ret = ESP32C3_ERR_RSA_VERIFY_FAILED;
      goto cleanup;
    }

  olen = ctx->len;
  ESP32C3_MPI_CHK(esp32c3_mpi_write_binary(&T, output, olen), cleanup);

cleanup:
  esp32c3_mpi_free(&P1);
  esp32c3_mpi_free(&Q1);
  esp32c3_mpi_free(&R);

  if (f_rng != NULL)
    {
#if defined(ESP32C3_RSA_NO_CRT)
      esp32c3_mpi_free(&d_blind);
#else
      esp32c3_mpi_free(&dp_blind);
      esp32c3_mpi_free(&dq_blind);
#endif
    }

  esp32c3_mpi_free(&T);

#if !defined(ESP32C3_RSA_NO_CRT)
  esp32c3_mpi_free(&TP); esp32c3_mpi_free(&TQ);
#endif

  esp32c3_mpi_free(&C);
  esp32c3_mpi_free(&I);

  if (ret != 0)
    {
      return (ESP32C3_ERR_RSA_PRIVATE_FAILED + ret);
    }

  return OK;
}

/****************************************************************************
 * Name: esp32c3_rsa_encrypt
 *
 * Description:
 *   Adds the message padding, then performs an RSA operation. It is the
 *   generic wrapper for performing a PKCS#1 encryption operation using the
 *   \p mode from the context.
 *
 * Input Parameters:
 *   ctx      - The initialized RSA context to use
 *   f_rng    - The RNG to use
 *   p_rng    - The RNG context to be passed to \p f_rng
 *   mode     - The mode of operation
 *   ilen     - The length of the plaintext in Bytes
 *   input    - The input data to encrypt
 *   output   - The output buffer
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_rsa_encrypt(struct esp32c3_rsa_context_s *ctx,
             int (*f_rng)(void *, unsigned char *, size_t),
             void *p_rng,
             int mode, size_t ilen,
             const unsigned char *input,
             unsigned char *output)
{
  DEBUGASSERT(ctx != NULL);
  DEBUGASSERT(mode == ESP32C3_RSA_PRIVATE ||
              mode == ESP32C3_RSA_PUBLIC);
  DEBUGASSERT(output != NULL);
  DEBUGASSERT(input != NULL);

  switch (ctx->padding)
    {
#if defined(ESP32C3_PKCS1_V15)
      case ESP32C3_RSA_PKCS_V15:
        return esp32c3_rsa_pkcs1_v15_encrypt(ctx, f_rng, p_rng, mode, ilen,
                                             input, output);
#endif
      default:
        return (ESP32C3_ERR_RSA_INVALID_PADDING);
    }
}

/****************************************************************************
 * Name: esp32c3_rsa_decrypt
 *
 * Description:
 *   Performs an RSA operation, then removes the message padding.
 *
 * Input Parameters:
 *   ctx            - The initialized RSA context to use
 *   f_rng          - The RNG function
 *   p_rng          - The RNG context to be passed to \p f_rng
 *   mode           - The mode of operation
 *   olen           - The point which to store the length of the plaintext
 *   input          - The ciphertext buffer
 *   output         - The buffer used to hold the plaintext
 *   output_max_len - The length in Bytes of the output buffer \p output
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_rsa_decrypt(struct esp32c3_rsa_context_s *ctx,
             int (*f_rng)(void *, unsigned char *, size_t),
             void *p_rng,
             int mode, size_t *olen,
             const unsigned char *input,
             unsigned char *output,
             size_t output_max_len)
{
  DEBUGASSERT(ctx != NULL);
  DEBUGASSERT(mode == ESP32C3_RSA_PRIVATE ||
              mode == ESP32C3_RSA_PUBLIC);
  DEBUGASSERT(output_max_len == 0 || output != NULL);
  DEBUGASSERT(input != NULL);
  DEBUGASSERT(olen != NULL);

  switch (ctx->padding)
    {
#if defined(ESP32C3_PKCS1_V15)
      case ESP32C3_RSA_PKCS_V15:
        return esp32c3_rsa_pkcs1_v15_decrypt(ctx, f_rng, p_rng, mode, olen,
                                             input, output, output_max_len);
#endif
      default:
        return (ESP32C3_ERR_RSA_INVALID_PADDING);
    }
}

/****************************************************************************
 * Name: esp32c3_rsa_copy
 *
 * Description:
 *   Copies the components of an RSA context.
 *
 * Input Parameters:
 *   dst      - The destination context
 *   src      - The source context
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_rsa_copy(struct esp32c3_rsa_context_s *dst,
                     const struct esp32c3_rsa_context_s *src)
{
  int ret;
  DEBUGASSERT(dst != NULL);
  DEBUGASSERT(src != NULL);

  dst->ver = src->ver;
  dst->len = src->len;

  ESP32C3_MPI_CHK(esp32c3_mpi_copy(&dst->N, &src->N), cleanup);
  ESP32C3_MPI_CHK(esp32c3_mpi_copy(&dst->E, &src->E), cleanup);

  ESP32C3_MPI_CHK(esp32c3_mpi_copy(&dst->D, &src->D), cleanup);
  ESP32C3_MPI_CHK(esp32c3_mpi_copy(&dst->P, &src->P), cleanup);
  ESP32C3_MPI_CHK(esp32c3_mpi_copy(&dst->Q, &src->Q), cleanup);

#if !defined(ESP32C3_RSA_NO_CRT)
  ESP32C3_MPI_CHK(esp32c3_mpi_copy(&dst->DP, &src->DP), cleanup);
  ESP32C3_MPI_CHK(esp32c3_mpi_copy(&dst->DQ, &src->DQ), cleanup);
  ESP32C3_MPI_CHK(esp32c3_mpi_copy(&dst->QP, &src->QP), cleanup);
  ESP32C3_MPI_CHK(esp32c3_mpi_copy(&dst->RP, &src->RP), cleanup);
  ESP32C3_MPI_CHK(esp32c3_mpi_copy(&dst->RQ, &src->RQ), cleanup);
#endif

  ESP32C3_MPI_CHK(esp32c3_mpi_copy(&dst->RN, &src->RN), cleanup);

  ESP32C3_MPI_CHK(esp32c3_mpi_copy(&dst->VI, &src->VI), cleanup);
  ESP32C3_MPI_CHK(esp32c3_mpi_copy(&dst->VF, &src->VF), cleanup);

  dst->padding = src->padding;
  dst->hash_id = src->hash_id;

cleanup:
  if (ret != 0)
    {
      esp32c3_rsa_free(dst);
    }

  return ret;
}

/****************************************************************************
 * Name: esp32c3_rsa_free
 *
 * Description:
 *   Frees the components of an RSA key.
 *
 * Input Parameters:
 *   ctx      - The RSA context to free
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32c3_rsa_free(struct esp32c3_rsa_context_s *ctx)
{
  if (ctx == NULL)
    {
      return;
    }

  esp32c3_mpi_free(&ctx->VI);
  esp32c3_mpi_free(&ctx->VF);
  esp32c3_mpi_free(&ctx->RN);
  esp32c3_mpi_free(&ctx->D);
  esp32c3_mpi_free(&ctx->Q);
  esp32c3_mpi_free(&ctx->P);
  esp32c3_mpi_free(&ctx->E);
  esp32c3_mpi_free(&ctx->N);

#if !defined(ESP32C3_RSA_NO_CRT)
  esp32c3_mpi_free(&ctx->RQ);
  esp32c3_mpi_free(&ctx->RP);
  esp32c3_mpi_free(&ctx->QP);
  esp32c3_mpi_free(&ctx->DQ);
  esp32c3_mpi_free(&ctx->DP);
#endif /* ESP32C3_RSA_NO_CRT */
}

#endif

