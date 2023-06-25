/****************************************************************************
 * include/crypto/bn.h
 * This is free and unencumbered software released into the public domain.
 * Anyone is free to copy, modify, publish, use, compile, sell, or
 * distribute this software, either in source code form or as a compiled
 * binary, for any purpose, commercial or non-commercial, and by any
 * means.
 * In jurisdictions that recognize copyright laws, the author or authors
 * of this software dedicate any and all copyright interest in the
 * software to the public domain. We make this dedication for the benefit
 * of the public at large and to the detriment of our heirs and
 * successors. We intend this dedication to be an overt act of
 * relinquishment in perpetuity of all present and future rights to this
 * software under copyright law.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * For more information, please refer to <https://unlicense.org>
 ****************************************************************************/

#ifndef __INCLUDE_CRYPTO_BIGNUM_H
#define __INCLUDE_CRYPTO_BIGNUM_H

/* Big number library - arithmetic on multiple-precision unsigned integers.
 *
 * This library is an implementation of arithmetic on arbitrarily large
 * integers.
 *
 * The difference between this and other implementations, is that the data
 * structure
 * has optimal memory utilization (i.e. a 1024 bit integer takes up 128 bytes
 * RAM),
 * and all memory is allocated statically: no dynamic allocation for better
 * or worse.
 *
 * Primary goals are correctness, clarity of code and clean, portable
 * implementation.
 * Secondary goal is a memory footprint small enough to make it suitable for
 * use in
 * embedded applications.
 *
 *
 * The current state is correct functionality and adequate performance.
 * There may well be room for performance-optimizations and improvements.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* This macro defines the word size in bytes of the array that constitues the
 * big-number data structure.
 */

#define WORD_SIZE             1

/* Size of big-numbers in bytes */

#define BN_ARRAY_SIZE         (256 / WORD_SIZE)

/* Data type of array in structure */

#define DTYPE                 uint8_t

/* Data-type larger than DTYPE, for holding intermediate results of
 * calculations
 */

#define DTYPE_TMP             uint32_t

/* bitmask for getting MSB */

#define DTYPE_MSB             ((DTYPE_TMP)(0x80))

/* sprintf format string */

#define SPRINTF_FORMAT_STR    "%.02x"
#define SSCANF_FORMAT_STR     "%2hhx"

/* Max value of integer type */

#define MAX_VAL               ((DTYPE_TMP)0xFF)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Data-holding structure: array of DTYPEs */

struct bn
{
  DTYPE array[BN_ARRAY_SIZE];
};

/* Tokens returned by bignum_cmp() for value comparison */

enum
{
  SMALLER = -1,
  EQUAL = 0,
  LARGER = 1
};

/****************************************************************************
 * Public Functions Prototype
 ****************************************************************************/

/* Initialization functions: */

void bignum_init(FAR struct bn *n);
void bignum_from_int(FAR struct bn *n, DTYPE_TMP i);
int  bignum_to_int(FAR struct bn *n);
void bignum_from_string(FAR struct bn *n, FAR char *str, int nbytes);
void bignum_to_string(FAR struct bn *n, FAR char *str, int maxsize);

/* Basic arithmetic operations: */

/* c = a + b */

void bignum_add(FAR struct bn *a, FAR struct bn *b, FAR struct bn *c);

/* c = a - b */

void bignum_sub(FAR struct bn *a, FAR struct bn *b, FAR struct bn *c);

/* c = a * b */

void bignum_mul(FAR struct bn *a, FAR struct bn *b, FAR struct bn *c);

/* c = a / b */

void bignum_div(FAR struct bn *a, FAR struct bn *b, FAR struct bn *c);

/* c = a % b */

void bignum_mod(FAR struct bn *a, FAR struct bn *b, FAR struct bn *c);

/* c = a / b, d = a % b */

void bignum_divmod(FAR struct bn *a, FAR struct bn *b,
                   FAR struct bn *c, FAR struct bn *d);

/* Bitwise operations: */

/* c = a & b */

void bignum_and(FAR struct bn *a, FAR struct bn *b, FAR struct bn *c);

/* c = a | b */

void bignum_or(FAR struct bn *a, FAR struct bn *b, FAR struct bn *c);

/* c = a ^ b */

void bignum_xor(FAR struct bn *a, FAR struct bn *b, FAR struct bn *c);

/* b = a << nbits */

void bignum_lshift(FAR struct bn *a, FAR struct bn *b, int nbits);

/* b = a >> nbits */

void bignum_rshift(FAR struct bn *a, FAR struct bn *b, int nbits);

/* Special operators and comparison */

/* Compare: returns LARGER, EQUAL or SMALLER */

int  bignum_cmp(FAR struct bn *a, FAR struct bn *b);

/* For comparison with zero */

int  bignum_is_zero(FAR struct bn *n);

/* Increment: add one to n */

void bignum_inc(FAR struct bn *n);

/* Decrement: subtract one from n */

void bignum_dec(FAR struct bn *n);

/* Calculate a^b -- e.g. 2^10 => 1024 */

void bignum_pow(FAR struct bn *a, FAR struct bn *b, FAR struct bn *c);

/* Integer square root -- e.g. isqrt(5) => 2 */

void bignum_isqrt(FAR struct bn *a, FAR struct bn *b);

/* Copy src into dst -- dst := src */

void bignum_assign(FAR struct bn *dst, FAR struct bn *src);

/* CRK_EXP_MOD algorithm */

void pow_mod_faster(FAR struct bn *a, FAR struct bn *b,
                    FAR struct bn *n, FAR struct bn *res);

#endif /* __INCLUDE_CRYPTO_BIGNUM_H */
