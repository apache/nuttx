/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_rsa.h
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

#ifndef __ARCH_RISCV_SRC_ESP32C3_ESP32C3_RSA_H
#define __ARCH_RISCV_SRC_ESP32C3_ESP32C3_RSA_H

#include <nuttx/config.h>
#include <stdint.h>
#include "esp32c3_bignum.h"

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Pre-processor Macros
 ****************************************************************************/

/* RSA Error codes */

#define ESP32C3_ERR_RSA_BAD_INPUT_DATA                    -0x4080  /**< Bad input parameters to function. */
#define ESP32C3_ERR_RSA_INVALID_PADDING                   -0x4100  /**< Input data contains invalid padding and is rejected. */
#define ESP32C3_ERR_RSA_KEY_GEN_FAILED                    -0x4180  /**< Something failed during generation of a key. */
#define ESP32C3_ERR_RSA_KEY_CHECK_FAILED                  -0x4200  /**< Key failed to pass the validity check of the library. */
#define ESP32C3_ERR_RSA_PUBLIC_FAILED                     -0x4280  /**< The public key operation failed. */
#define ESP32C3_ERR_RSA_PRIVATE_FAILED                    -0x4300  /**< The private key operation failed. */
#define ESP32C3_ERR_RSA_VERIFY_FAILED                     -0x4380  /**< The PKCS#1 verification failed. */
#define ESP32C3_ERR_RSA_OUTPUT_TOO_LARGE                  -0x4400  /**< The output buffer for decryption is not large enough. */
#define ESP32C3_ERR_RSA_RNG_FAILED                        -0x4480  /**< The random generator failed to generate non-zeros. */

/* RSA constants */

#define ESP32C3_RSA_PUBLIC      0 /**< Request private key operation. */
#define ESP32C3_RSA_PRIVATE     1 /**< Request public key operation. */

#define ESP32C3_RSA_PKCS_V15    0 /**< Use PKCS#1 v1.5 encoding. */
#define ESP32C3_RSA_PKCS_V21    1 /**< Use PKCS#1 v2.1 encoding. */

#define ESP32C3_RSA_SIGN        1 /**< Identifier for RSA signature operations. */
#define ESP32C3_RSA_CRYPT       2 /**< Identifier for RSA encryption and decryption operations. */

#define ESP32C3_RSA_SALT_LEN_ANY    -1

/****************************************************************************
 * Public Types
 ****************************************************************************/

/**
 * \brief   - The RSA context structure.
 */

struct esp32c3_rsa_context_s
{
    int ver;                    /* Always 0 */
    size_t len;                 /* The size of \p N in Bytes */

    struct esp32c3_mpi_s N;              /* The public modulus */
    struct esp32c3_mpi_s E;              /* The public exponent */

    struct esp32c3_mpi_s D;              /* The private exponent */
    struct esp32c3_mpi_s P;              /* The first prime factor */
    struct esp32c3_mpi_s Q;              /* The second prime factor */

    struct esp32c3_mpi_s DP;             /* <code>D % (P - 1)</code> */
    struct esp32c3_mpi_s DQ;             /* <code>D % (Q - 1)</code> */
    struct esp32c3_mpi_s QP;             /* <code>1 / (Q % P)</code> */

    struct esp32c3_mpi_s RN;             /* cached <code>R^2 mod N</code> */

    struct esp32c3_mpi_s RP;             /* cached <code>R^2 mod P</code> */
    struct esp32c3_mpi_s RQ;             /* cached <code>R^2 mod Q</code> */

    struct esp32c3_mpi_s VI;             /* The cached blinding value */
    struct esp32c3_mpi_s VF;             /* The cached un-blinding value */

    int padding;                /* Selects padding mode */
    int hash_id;                /* Hash identifier */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

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
                       int hash_id);

/****************************************************************************
 * Name: esp32c3_rsa_import
 *
 * Description:
 *   Imports a set of core parameters into an RSA context.
 *
 * Input Parameters:
 *   ctx      - The initialized RSA context to store the parameters in
 *   N        - The RSA modulus
 *   P        - The first prime factor of \p N
 *   Q        - The second prime factor of \p N
 *   D        - The private exponent
 *   E        - The public exponent
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_rsa_import(struct esp32c3_rsa_context_s *ctx,
                        const struct esp32c3_mpi_s *N,
                        const struct esp32c3_mpi_s *P,
                        const struct esp32c3_mpi_s *Q,
                        const struct esp32c3_mpi_s *D,
                        const struct esp32c3_mpi_s *E);

/****************************************************************************
 * Name: esp32c3_rsa_import_raw
 *
 * Description:
 *   Imports core RSA parameters into an RSA context.
 *
 * Input Parameters:
 *   ctx      - The initialized RSA context to store the parameters in
 *   N        - The RSA modulus
 *   NL       - The Byte length of \p N
 *   P        - The first prime factor of \p N
 *   PL       - The Byte length of \p P
 *   Q        - The second prime factor of \p N
 *   QL       - The Byte length of \p Q
 *   D        - The private exponent
 *   DL       - The Byte length of \p D
 *   E        - The public exponent
 *   EL       - The Byte length of \p E
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
                            unsigned char const *E, size_t EL);

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

int esp32c3_rsa_complete(struct esp32c3_rsa_context_s *ctx);

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
                        struct esp32c3_mpi_s *E);

/****************************************************************************
 * Name: esp32c3_rsa_export_raw
 *
 * Description:
 *   Eexports core parameters of an RSA key in raw big-endian binary format.
 *
 * Input Parameters:
 *   ctx      - The initialized RSA context
 *   N        - The Byte array to store the RSA modulus
 *   NL       - The size of the buffer for the modulus
 *   P        - The Byte array to hold the first prime factor of \p N
 *   PL       - The size of the buffer for the first prime factor
 *   Q        - The Byte array to hold the second prime factor of \p N
 *   QL       - The size of the buffer for the second prime factor
 *   D        - The Byte array to hold the private exponent
 *   DL       - The size of the buffer for the private exponent
 *   E        - The Byte array to hold the public exponent
 *   EL       - The size of the buffer for the public exponent
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
                            unsigned char *E, size_t EL);

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
                           struct esp32c3_mpi_s *QP);

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
                             int padding, int hash_id);

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

size_t esp32c3_rsa_get_len(const struct esp32c3_rsa_context_s *ctx);

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

int esp32c3_rsa_check_pubkey(const struct esp32c3_rsa_context_s *ctx);

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

int esp32c3_rsa_check_privkey(const struct esp32c3_rsa_context_s *ctx);

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
                                const struct esp32c3_rsa_context_s *prv);

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
                unsigned char *output);

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
                 unsigned char *output);

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
                       unsigned char *output);

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
                       size_t output_max_len);

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
                     const struct esp32c3_rsa_context_s *src);

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

void esp32c3_rsa_free(struct esp32c3_rsa_context_s *ctx);

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_ESP32C3_ESP32C3_RSA_H */
