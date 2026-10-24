/****************************************************************************
 * include/crypto/ecc.h
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2013, Kenneth MacKay All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * SPECIAL, HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ****************************************************************************/

#ifndef __INCLUDE_CRYPTO_ECC_H
#define __INCLUDE_CRYPTO_ECC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Curve selection options. */

#define secp128r1 16
#define secp192r1 24
#define secp256r1 32
#define secp384r1 48

#ifndef ECC_CURVE
#  define ECC_CURVE secp256r1
#endif

#if (ECC_CURVE != secp128r1 && \
     ECC_CURVE != secp192r1 && \
     ECC_CURVE != secp256r1 && \
     ECC_CURVE != secp384r1)
#  error "Must define ECC_CURVE to one of the available curves"
#endif

#define ECC_BYTES ECC_CURVE

#ifdef __cplusplus
extern "C"
{
#endif

/* ecc_make_key() function.
 * Create a public/private key pair.
 *
 * Outputs:
 *   publickey  - Will be filled in with the public key.
 *   privatekey - Will be filled in with the private key.
 *
 * Returns 1 if the key pair was generated successfully,
 * 0 if an error occurred.
 */

int ecc_make_key(uint8_t publickey[ECC_BYTES + 1],
                 uint8_t privatekey[ECC_BYTES]);

int ecc_make_key_uncomp(uint8_t publickey_x[ECC_BYTES],
                        uint8_t publickey_y[ECC_BYTES],
                        uint8_t privatekey[ECC_BYTES]);

/* ecdh_shared_secret() function.
 * Compute a shared secret given your secret key and someone else's
 * public key.
 * Note: It is recommended that you hash the result of ecdh_shared_secret
 * before using it for symmetric encryption or HMAC.
 *
 * Inputs:
 *    publickey  - The public key of the remote party.
 *    privatekey - Your private key.
 *
 * Outputs:
 *    secret - Will be filled in with the shared secret value.
 *
 * Returns 1 if the shared secret was generated successfully,
 * 0 if an error occurred.
 */

int ecdh_shared_secret(const uint8_t publickey[ECC_BYTES + 1],
                       const uint8_t privatekey[ECC_BYTES],
                       uint8_t secret[ECC_BYTES]);

/* ecdsa_sign() function.
 * Generate an ECDSA signature for a given hash value.
 *
 * Usage: Compute a hash of the data you wish to sign (SHA-2 is recommended)
 * and pass it in to this function along with your private key.
 *
 * Inputs:
 *    privatekey - Your private key.
 *    hash       - The message hash to sign.
 *
 * Outputs:
 *    signature  - Will be filled in with the signature value.
 *
 * Returns 1 if the signature generated successfully, 0 if an error occurred.
 */

int ecdsa_sign(const uint8_t privatekey[ECC_BYTES],
               const uint8_t hash[ECC_BYTES],
               uint8_t signature[ECC_BYTES * 2]);

/* ecdsa_verify() function.
 * Verify an ECDSA signature.
 *
 * Usage: Compute the hash of the signed data using the same hash as
 * the signer and pass it to this function along with the signer's
 * public key and the signature values (r and s).
 *
 * Inputs:
 *   publickey - The signer's public key
 *   hash      - The hash of the signed data.
 *   signature - The signature value.
 *
 * Returns 1 if the signature is valid, 0 if it is invalid.
 */

int ecdsa_verify(const uint8_t publickey[ECC_BYTES + 1],
                 const uint8_t hash[ECC_BYTES],
                 const uint8_t signature[ECC_BYTES * 2]);

/* Point operations:
 * Operates on affine points (x, y) with ECC_BYTES per coordinate
 * (big endian).
 */

/* ecc_point_add() function.
 * Add two affine points P and Q: R = P + Q.
 *
 * Inputs:
 *   px, py - Coordinates of point P.
 *   qx, qy - Coordinates of point Q.
 *
 * Outputs:
 *   rx, ry - Coordinates of result point R (may overlap with inputs).
 *
 * Returns 0 on success, 1 if the result is the point at infinity,
 * or a negative error code (e.g. -EINVAL).
 */

int ecc_point_add(uint8_t rx[ECC_BYTES], uint8_t ry[ECC_BYTES],
                  const uint8_t px[ECC_BYTES], const uint8_t py[ECC_BYTES],
                  const uint8_t qx[ECC_BYTES], const uint8_t qy[ECC_BYTES]);

/* ecc_point_mul() function.
 * Multiply an affine point P by a scalar k: R = k * P.
 *
 * Inputs:
 *   px, py - Coordinates of point P.
 *   k      - Scalar multiplier of ECC_BYTES (big endian).
 *
 * Outputs:
 *   rx, ry - Coordinates of result point R (may overlap with inputs).
 *
 * Returns 0 on success, 1 if the result is the point at infinity,
 * or a negative error code (e.g. -EINVAL).
 */

int ecc_point_mul(uint8_t rx[ECC_BYTES], uint8_t ry[ECC_BYTES],
                  const uint8_t px[ECC_BYTES], const uint8_t py[ECC_BYTES],
                  const uint8_t k[ECC_BYTES]);

/* ecc_point_is_on_curve() function.
 * Check whether the given affine point (px, py) lies on the curve.
 *
 * Inputs:
 *   px, py - Coordinates of point P.
 *
 * Returns 1 if the point is on the curve, 0 if not,
 * or a negative error code (e.g. -EINVAL).
 */

int ecc_point_is_on_curve(const uint8_t px[ECC_BYTES],
                          const uint8_t py[ECC_BYTES]);

/* Modular operations:
 * Operates on big-endian integers of ECC_BYTES length.
 */

/* ecc_mod_add() function.
 * Computes res = (a + b) mod m.
 * Inputs and output may overlap.
 *
 * Returns 0 on success, or a negative error code.
 */

int ecc_mod_add(uint8_t res[ECC_BYTES], const uint8_t a[ECC_BYTES],
                const uint8_t b[ECC_BYTES], const uint8_t mod[ECC_BYTES]);

/* ecc_mod_sub() function.
 * Computes res = (a - b) mod m.
 * Inputs and output may overlap.
 *
 * Returns 0 on success, or a negative error code.
 */

int ecc_mod_sub(uint8_t res[ECC_BYTES], const uint8_t a[ECC_BYTES],
                const uint8_t b[ECC_BYTES], const uint8_t mod[ECC_BYTES]);

/* ecc_mod_mul() function.
 * Computes res = (a * b) mod m.
 * Inputs and output may overlap.
 *
 * Returns 0 on success, or a negative error code.
 */

int ecc_mod_mul(uint8_t res[ECC_BYTES], const uint8_t a[ECC_BYTES],
                const uint8_t b[ECC_BYTES], const uint8_t mod[ECC_BYTES]);

/* ecc_mod_exp() function.
 * Computes res = (base ^ exp) mod m.
 * Inputs and output may overlap.
 *
 * Returns 0 on success, or a negative error code.
 */

int ecc_mod_exp(uint8_t res[ECC_BYTES], const uint8_t base[ECC_BYTES],
                const uint8_t exp[ECC_BYTES], const uint8_t mod[ECC_BYTES]);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_CRYPTO_ECC_H */
