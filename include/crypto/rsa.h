/****************************************************************************
 * include/crypto/rsa.h
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

#ifndef __INCLUDE_CRYPTO_RSA_H
#define __INCLUDE_CRYPTO_RSA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stddef.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_CRYPTO_RSA_MAX_BITS
#  define RSA_MAX_BITS   CONFIG_CRYPTO_RSA_MAX_BITS
#else
#  define RSA_MAX_BITS   4096
#endif

#define RSA_MAX_BYTES    (RSA_MAX_BITS / 8)

#define RSA_MIN_BITS     256

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* All integers are unsigned big-endian byte strings; leading zero bytes
 * are allowed.  A component that is not available has a NULL pointer or a
 * zero length.  A private operation uses the CRT form when p, q, dp, dq
 * and qp are all present, and falls back to d otherwise.
 */

struct rsa_key_s
{
  FAR const uint8_t *n;
  size_t nlen;
  FAR const uint8_t *e;
  size_t elen;
  FAR const uint8_t *d;
  size_t dlen;
  FAR const uint8_t *p;
  size_t plen;
  FAR const uint8_t *q;
  size_t qlen;
  FAR const uint8_t *dp;
  size_t dplen;
  FAR const uint8_t *dq;
  size_t dqlen;
  FAR const uint8_t *qp;
  size_t qplen;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: rsa_modulus_len
 *
 * Description:
 *   Return the length in bytes of the modulus n, not counting leading
 *   zero bytes.  This is the buffer length rsa_public() and rsa_private()
 *   expect.
 *
 ****************************************************************************/

size_t rsa_modulus_len(FAR const uint8_t *n, size_t nlen);

/****************************************************************************
 * Name: rsa_public
 *
 * Description:
 *   Compute x = x^e mod n in place.  xlen must equal rsa_modulus_len(n)
 *   and x must be lower than n.
 *
 * Returned Value:
 *   0 on success, -EINVAL on a malformed key or input.
 *
 ****************************************************************************/

int rsa_public(FAR uint8_t *x, size_t xlen, FAR const struct rsa_key_s *key);

/****************************************************************************
 * Name: rsa_private
 *
 * Description:
 *   Compute x = x^d mod n in place, through the CRT when the CRT
 *   components are present.  xlen must equal rsa_modulus_len(n) and x
 *   must be lower than n.
 *
 * Returned Value:
 *   0 on success, -EINVAL on a malformed key or input.
 *
 ****************************************************************************/

int rsa_private(FAR uint8_t *x, size_t xlen,
                FAR const struct rsa_key_s *key);

#endif /* __INCLUDE_CRYPTO_RSA_H */
