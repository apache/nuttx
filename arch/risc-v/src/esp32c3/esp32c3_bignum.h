/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_bignum.h
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

#ifndef __ARCH_RISCV_SRC_ESP32C3_ESP32C3_BIGNUM_H
#define __ARCH_RISCV_SRC_ESP32C3_ESP32C3_BIGNUM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stddef.h>
#include <stdint.h>

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

#define ESP32C3_ERR_MPI_FILE_IO_ERROR           -0x0002  /**< An error occurred while reading from or writing to a file. */
#define ESP32C3_ERR_MPI_BAD_INPUT_DATA          -0x0004  /**< Bad input parameters to function. */
#define ESP32C3_ERR_MPI_INVALID_CHARACTER       -0x0006  /**< There is an invalid character in the digit string. */
#define ESP32C3_ERR_MPI_BUFFER_TOO_SMALL        -0x0008  /**< The buffer is too small to write to. */
#define ESP32C3_ERR_MPI_NEGATIVE_VALUE          -0x000A  /**< The input arguments are negative or result in illegal output. */
#define ESP32C3_ERR_MPI_DIVISION_BY_ZERO        -0x000C  /**< The input argument for division is zero, which is not allowed. */
#define ESP32C3_ERR_MPI_NOT_ACCEPTABLE          -0x000E  /**< The input arguments are not acceptable. */
#define ESP32C3_ERR_MPI_ALLOC_FAILED            -0x0010  /**< Memory allocation failed. */

#define ESP32C3_MPI_CHK(f, a)               \
  do                                        \
    {                                       \
      ret = (f);                            \
      if (ret != 0)                         \
        {                                   \
          goto a;                           \
        }                                   \
    }                                       \
  while(0)

/* Maximum size MPIs are allowed to grow to in number of limbs. */
#define ESP32C3_MPI_MAX_LIMBS               10000

/* Maximum window size used for modular exponentiation */
#define ESP32C3_MPI_WINDOW_SIZE               6

/* Maximum size of MPIs allowed in bits and bytes for user-MPIs. */
#define ESP32C3_MPI_MAX_SIZE                1024

/**< Maximum number of bits for usable MPIs. */
#define ESP32C3_MPI_MAX_BITS                (8 * ESP32C3_MPI_MAX_SIZE)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* MPI structure */

struct esp32c3_mpi_s
{
  int s;                /* Sign: -1 if the mpi is negative, 1 otherwise */
  size_t n;             /* total number of limbs */
  uint32_t *p;          /* pointer to limbs */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp32c3_mpi_init
 *
 * Description:
 *   Initialize an MPI context
 *
 * Input Parameters:
 *   X    - The MPI context to initialize
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32c3_mpi_init(struct esp32c3_mpi_s *X);

/****************************************************************************
 * Name: esp32c3_mpi_free
 *
 * Description:
 *   Frees the components of an MPI context
 *
 * Input Parameters:
 *   X    - The MPI context to be cleared
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32c3_mpi_free(struct esp32c3_mpi_s *X);

/****************************************************************************
 * Name: esp32c3_mpi_grow
 *
 * Description:
 *   Enlarge an MPI to the specified number of limbs
 *
 * Input Parameters:
 *   X     - The MPI context to grow
 *   nblimbs - The target number of limbs
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_grow(struct esp32c3_mpi_s *X, size_t nblimbs);

/****************************************************************************
 * Name: esp32c3_mpi_shrink
 *
 * Description:
 *   Resizes an MPI downwards, keeping at least the specified number of limbs
 *
 * Input Parameters:
 *   X     - The MPI context to shrink
 *   nblimbs - The minimum number of limbs
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_shrink(struct esp32c3_mpi_s *X, size_t nblimbs);

/****************************************************************************
 * Name: esp32c3_mpi_copy
 *
 * Description:
 *   Copy the contents of Y into X
 *
 * Input Parameters:
 *   X     - The destination MPI
 *   Y     - The source MPI
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_copy(struct esp32c3_mpi_s *X,
           const struct esp32c3_mpi_s *Y);

/****************************************************************************
 * Name: esp32c3_mpi_swap
 *
 * Description:
 *   Swap the contents of X and Y
 *
 * Input Parameters:
 *   X     - The first MPI
 *   nblimbs - The second MPI
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32c3_mpi_swap(struct esp32c3_mpi_s *X,
            struct esp32c3_mpi_s *Y);

/****************************************************************************
 * Name: esp32c3_mpi_safe_cond_assign
 *
 * Description:
 *   Perform a safe conditional copy of MPI which doesn't
 *   reveal whether the condition was true or not.
 *
 * Input Parameters:
 *   X     - The MPI to conditionally assign to
 *   Y     - The MPI to be assigned from
 *   assign  - The condition deciding whether perform the assignment or not
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_safe_cond_assign(struct esp32c3_mpi_s *X,
                 const struct esp32c3_mpi_s *Y,
                 unsigned char assign);

/****************************************************************************
 * Name: esp32c3_mpi_safe_cond_swap
 *
 * Description:
 *   Perform a safe conditional swap which doesn't
 *   reveal whether the condition was true or not.
 *
 * Input Parameters:
 *   X     - The first MPI
 *   Y     - The second MPI
 *   swap  - The condition deciding whether to perform the swap or not
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_safe_cond_swap(struct esp32c3_mpi_s *X,
                 struct esp32c3_mpi_s *Y,
                 unsigned char assign);

/****************************************************************************
 * Name: esp32c3_mpi_lset
 *
 * Description:
 *   Set value from integer
 *
 * Input Parameters:
 *   X     - The MPI to set
 *   z     - The value to use
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_lset(struct esp32c3_mpi_s *X, int32_t z);

/****************************************************************************
 * Name: esp32c3_mpi_get_bit
 *
 * Description:
 *   Get a specific bit from an MPI
 *
 * Input Parameters:
 *   X     - The MPI context to query
 *   pos   - Zero-based index of the bit to query
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_get_bit(const struct esp32c3_mpi_s *X, size_t pos);

/****************************************************************************
 * Name: esp32c3_mpi_set_bit
 *
 * Description:
 *   Modify a specific bit in an MPI
 *
 * Input Parameters:
 *   X     - The MPI context to modify
 *   pos   - Zero-based index of the bit to modify
 *   val   - The desired value of bit
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_set_bit(struct esp32c3_mpi_s *X,
            size_t pos, unsigned char val);

/****************************************************************************
 * Name: esp32c3_mpi_lsb
 *
 * Description:
 *   Return the number of bits of value
 *
 * Input Parameters:
 *   X     - The MPI context to query
 *
 * Returned Value:
 *   The number of bits of value.
 *
 ****************************************************************************/

size_t esp32c3_mpi_lsb(const struct esp32c3_mpi_s *X);

/****************************************************************************
 * Name: esp32c3_mpi_bitlen
 *
 * Description:
 *   Return the number of bits up to and including the most
 *   significant bit of value
 *
 * Input Parameters:
 *   X     - The MPI context to query
 *
 * Returned Value:
 *   The number of bits up and including the most significant bit of value.
 *
 ****************************************************************************/

size_t esp32c3_mpi_bitlen(const struct esp32c3_mpi_s *X);

/****************************************************************************
 * Name: esp32c3_mpi_size
 *
 * Description:
 *   Return the total size of an MPI value in bytes
 *
 * Input Parameters:
 *   X     - The MPI context to query
 *
 * Returned Value:
 *   The least number of bytes capable of storing the absolute value.
 *
 ****************************************************************************/

size_t esp32c3_mpi_size(const struct esp32c3_mpi_s *X);

/****************************************************************************
 * Name: esp32c3_mpi_read_string
 *
 * Description:
 *   Import from an ASCII string
 *
 * Input Parameters:
 *   X     - The destination MPI
 *   radix   - The numeric base of the input string
 *   s     - Null-terminated string buffer
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_read_string(struct esp32c3_mpi_s *X,
              int radix, const char *s);

/****************************************************************************
 * Name: esp32c3_mpi_write_string
 *
 * Description:
 *   Export an MPI to an ASCII string
 *
 * Input Parameters:
 *   X     - The source MPI
 *   radix   - The numeric base of the output string
 *   buf   - The buffer to write the string to
 *   buflen  - The available size in Bytes of buf
 *   olen  - The address at which to store the length of the string written
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_write_string(const struct esp32c3_mpi_s *X, int radix,
                char *buf, size_t buflen, size_t *olen);

/****************************************************************************
 * Name: esp32c3_mpi_read_binary
 *
 * Description:
 *   Import an MPI from unsigned big endian binary data
 *
 * Input Parameters:
 *   X     - The destination MPI
 *   buf   - The input buffer
 *   buflen  - The length of the input buffer
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_read_binary(struct esp32c3_mpi_s *X,
              const unsigned char *buf, size_t buflen);

/****************************************************************************
 * Name: esp32c3_mpi_write_binary
 *
 * Description:
 *   Export X into unsigned binary data, big endian
 *
 * Input Parameters:
 *   X     - The source MPI
 *   buf   - The output buffer
 *   buflen  - The length of the output buffer
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_write_binary(const struct esp32c3_mpi_s *X,
               unsigned char *buf, size_t buflen);

/****************************************************************************
 * Name: esp32c3_mpi_shift_l
 *
 * Description:
 *   Perform a left-shift on an MPI: X <<= count
 *
 * Input Parameters:
 *   X     - The MPI to shift
 *   count   - The number of bits to shift by
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_shift_l(struct esp32c3_mpi_s *X, size_t count);

/****************************************************************************
 * Name: esp32c3_mpi_shift_r
 *
 * Description:
 *   Perform a right-shift on an MPI: X >>= count
 *
 * Input Parameters:
 *   X     - The MPI to shift
 *   count   - The number of bits to shift by
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_shift_r(struct esp32c3_mpi_s *X, size_t count);

/****************************************************************************
 * Name: esp32c3_mpi_cmp_abs
 *
 * Description:
 *   Compare the absolute values of two MPIs
 *
 * Input Parameters:
 *   X     - The left-hand MPI
 *   Y     - The right-hand MPI
 *
 * Returned Value:
 *   1 if \p `|X|` is greater than \p `|Y|`.
 *   -1 if \p `|X|` is lesser than \p `|Y|`.
 *   0 if \p `|X|` is equal to \p `|Y|`.
 *
 ****************************************************************************/

int esp32c3_mpi_cmp_abs(const struct esp32c3_mpi_s *X,
            const struct esp32c3_mpi_s *Y);

/****************************************************************************
 * Name: esp32c3_mpi_cmp_mpi
 *
 * Description:
 *   Compare two MPIs.
 *
 * Input Parameters:
 *   X     - The left-hand MPI
 *   Y     - The right-hand MPI
 *
 * Returned Value:
 *   1 if \p `X` is greater than \p `Y`.
 *   -1 if \p `X` is lesser than \p `Y`.
 *   0 if \p `X` is equal to \p `Y`.
 *
 ****************************************************************************/

int esp32c3_mpi_cmp_mpi(const struct esp32c3_mpi_s *X,
            const struct esp32c3_mpi_s *Y);

/****************************************************************************
 * Name: esp32c3_mpi_lt_mpi_ct
 *
 * Description:
 *   Check if an MPI is less than the other in constant time
 *
 * Input Parameters:
 *   X     - The left-hand MPI
 *   Y     - The right-hand MPI
 *   ret   - The result of the comparison:
 *       1 if \p X is less than \p Y.
 *       0 if \p X is greater than or equal to \p Y.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_lt_mpi_ct(const struct esp32c3_mpi_s *X,
              const struct esp32c3_mpi_s *Y,
              unsigned *ret);

/****************************************************************************
 * Name: esp32c3_mpi_cmp_int
 *
 * Description:
 *   Compare an MPI with an integer
 *
 * Input Parameters:
 *   X     - The left-hand MPI
 *   z     - The integer value to compare \p X to
 *
 * Returned Value:
 *   \c 1 if \p X is greater than \p z.
 *   \c -1 if \p X is lesser than \p z.
 *   \c 0 if \p X is equal to \p z.
 *
 ****************************************************************************/

int esp32c3_mpi_cmp_int(const struct esp32c3_mpi_s *X, int32_t z);

/****************************************************************************
 * Name: esp32c3_mpi_add_abs
 *
 * Description:
 *   Perform an unsigned addition of MPIs: X = |A| + |B|
 *
 * Input Parameters:
 *   X     - The left-hand MPI
 *   z     - The integer value to compare \p X to.
 *
 * Returned Value:
 *   \c 1 if \p X is greater than \p z.
 *   \c -1 if \p X is lesser than \p z.
 *   \c 0 if \p X is equal to \p z.
 *
 ****************************************************************************/

int esp32c3_mpi_add_abs(struct esp32c3_mpi_s *X,
            const struct esp32c3_mpi_s *A,
            const struct esp32c3_mpi_s *B);

/****************************************************************************
 * Name: esp32c3_mpi_sub_abs
 *
 * Description:
 *   Perform an unsigned subtraction of MPIs: X = |A| - |B|
 *
 * Input Parameters:
 *   X     - The destination MPI
 *   A     - The minuend
 *   B     - The subtrahend
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_sub_abs(struct esp32c3_mpi_s *X,
            const struct esp32c3_mpi_s *A,
            const struct esp32c3_mpi_s *B);

/****************************************************************************
 * Name: esp32c3_mpi_add_mpi
 *
 * Description:
 *   Perform a signed addition of MPIs: X = A + B
 *
 * Input Parameters:
 *   X     - The destination MPI
 *   A     - The first summand
 *   B     - The second summand
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_add_mpi(struct esp32c3_mpi_s *X,
            const struct esp32c3_mpi_s *A,
            const struct esp32c3_mpi_s *B);

/****************************************************************************
 * Name: esp32c3_mpi_sub_mpi
 *
 * Description:
 *   Perform a signed subtraction of MPIs: X = A - B
 *
 * Input Parameters:
 *   X     - The destination MPI
 *   A     - The minuend
 *   B     - The subtrahend
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_sub_mpi(struct esp32c3_mpi_s *X,
            const struct esp32c3_mpi_s *A,
            const struct esp32c3_mpi_s *B);

/****************************************************************************
 * Name: esp32c3_mpi_add_int
 *
 * Description:
 *   Perform a signed addition of an MPI and an integer: X = A + b
 *
 * Input Parameters:
 *   X     - The destination MPI
 *   A     - The first summand
 *   b     - The second summand
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_add_int(struct esp32c3_mpi_s *X,
            const struct esp32c3_mpi_s *A,
            int32_t b);

/****************************************************************************
 * Name: esp32c3_mpi_sub_int
 *
 * Description:
 *   Perform a signed subtraction of an MPI and an integer: X = A - b
 *
 * Input Parameters:
 *   X     - The destination MPI
 *   A     - The minuend
 *   b     - The subtrahend
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_sub_int(struct esp32c3_mpi_s *X,
            const struct esp32c3_mpi_s *A,
            int32_t b);

/****************************************************************************
 * Name: esp32c3_mpi_mul_mpi
 *
 * Description:
 *   Perform a multiplication of two MPIs: Z = X * Y
 *
 * Input Parameters:
 *   Z    - The destination MPI
 *   X    - The first factor
 *   Y    - The second factor
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_mul_mpi(struct esp32c3_mpi_s *X,
            const struct esp32c3_mpi_s *A,
            const struct esp32c3_mpi_s *B);

/****************************************************************************
 * Name: esp32c3_mpi_mul_int
 *
 * Description:
 *   Perform a multiplication of an MPI with an unsigned integer: X = A * b
 *
 * Input Parameters:
 *   X     - The destination MPI
 *   A     - The first factor
 *   b     - The second factor.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_mul_int(struct esp32c3_mpi_s *X,
            const struct esp32c3_mpi_s *A,
            uint32_t b);

/****************************************************************************
 * Name: esp32c3_mpi_div_mpi
 *
 * Description:
 *   Perform a division with remainder of two MPIs: A = Q * B + R
 *
 * Input Parameters:
 *   Q    - The destination MPI for the quotient
 *   R    - The destination MPI for the remainder value
 *   A    - The dividend
 *   B    - The divisor
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_div_mpi(struct esp32c3_mpi_s *Q,
            struct esp32c3_mpi_s *R,
            const struct esp32c3_mpi_s *A,
            const struct esp32c3_mpi_s *B);

/****************************************************************************
 * Name: esp32c3_mpi_div_int
 *
 * Description:
 *   Perform a division with remainder of an MPI by an integer: A = Q * b + R
 *
 * Input Parameters:
 *   Q    - The destination MPI for the quotient
 *   R    - The destination MPI for the remainder value
 *   A    - The dividend
 *   B    - The divisor
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_div_int(struct esp32c3_mpi_s *Q,
            struct esp32c3_mpi_s *R,
            const struct esp32c3_mpi_s *A,
            int32_t b);

/****************************************************************************
 * Name: esp32c3_mpi_mod_mpi
 *
 * Description:
 *   erform a modular reduction. R = A mod B
 *
 * Input Parameters:
 *   R     - The destination MPI for the residue value
 *   A     - The MPI to compute the residue of
 *   B     - The base of the modular reduction
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_mod_mpi(struct esp32c3_mpi_s *R,
            const struct esp32c3_mpi_s *A,
            const struct esp32c3_mpi_s *B);

/****************************************************************************
 * Name: esp32c3_mpi_mod_int
 *
 * Description:
 *   Perform a modular reduction with respect to an integer: r = A mod b
 *
 * Input Parameters:
 *   r     - The address at which to store the residue
 *   A     - The MPI to compute the residue of
 *   b     - The integer base of the modular reduction
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_mod_int(uint32_t *r,
            const struct esp32c3_mpi_s *A,
            int32_t b);

/****************************************************************************
 * Name: esp32c3_mpi_exp_mod
 *
 * Description:
 *   Perform a sliding-window exponentiation: X = A^E mod N
 *
 * Input Parameters:
 *   X     - The destination MPI
 *   A     - The base of the exponentiation
 *   E     - The exponent MPI
 *   N     - The base for the modular reduction
 *   _RR   - A helper MPI depending solely on \p N which can be used to
 *       speed-up multiple modular exponentiations for the same value
 *       of \p N.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_exp_mod(struct esp32c3_mpi_s *X,
            const struct esp32c3_mpi_s *A,
            const struct esp32c3_mpi_s *E,
            const struct esp32c3_mpi_s *N,
            struct esp32c3_mpi_s *_RR);

/****************************************************************************
 * Name: esp32c3_mpi_gcd
 *
 * Description:
 *   Compute the greatest common divisor: G = gcd(A, B)
 *
 * Input Parameters:
 *   G     - The destination MPI
 *   A     - The first operand
 *   B     - The second operand
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_gcd(struct esp32c3_mpi_s *G,
          const struct esp32c3_mpi_s *A,
          const struct esp32c3_mpi_s *B);

/****************************************************************************
 * Name: esp32c3_mpi_fill_random
 *
 * Description:
 *   Fill an MPI with a number of random bytes
 *
 * Input Parameters:
 *   X    - The destination MPI
 *   size   - The number of random bytes to generate
 *   f_rng  - The RNG function to use. This must not be \c NULL
 *   p_rng  - The RNG parameter to be passed to \p f_rng
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_fill_random(struct esp32c3_mpi_s *X, size_t size,
           int (*f_rng)(void *, unsigned char *, size_t),
           void *p_rng);

/****************************************************************************
 * Name: esp32c3_mpi_inv_mod
 *
 * Description:
 *   Compute the modular inverse: X = A^-1 mod N
 *
 * Input Parameters:
 *   X    - The destination MPI
 *   A    - The MPI to calculate the modular inverse of
 *   N    - The base of the modular inversion
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_inv_mod(struct esp32c3_mpi_s *X,
            const struct esp32c3_mpi_s *A,
            const struct esp32c3_mpi_s *N);

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_ESP32C3_ESP32C3_BIGNUM_H */
