/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_bignum.c
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

#ifdef CONFIG_ESP32C3_BIGNUM_ACCELERATOR

#include <stdio.h>
#include <string.h>
#include <malloc.h>
#include <limits.h>
#include <assert.h>
#include <stdlib.h>
#include <sys/param.h>
#include <debug.h>
#include <semaphore.h>

#include "riscv_arch.h"
#include "hardware/esp32c3_rsa.h"
#include "hardware/esp32c3_system.h"

#include "esp32c3_bignum.h"

/****************************************************************************
 * Pre-processor Macros
 ****************************************************************************/

#undef MIN
#undef MAX
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))
#define SOC_RSA_MAX_BIT_LEN (3072)

#define CIL  (sizeof(uint32_t))             /* chars in limb */
#define BIL  (CIL << 3)                     /* bits in limb */
#define BIH  (CIL << 2)                     /* half limb size */

#define MPI_SIZE_T_MAX  ((size_t) -1)       /* SIZE_T_MAX is not standard */

/* Convert between bits/chars and number of limbs
 * Divide first in order to avoid potential overflows
 */

#define BITS_TO_LIMBS(i)  ((i) / BIL + ((i) % BIL != 0))
#define CHARS_TO_LIMBS(i) ((i) / CIL + ((i) % CIL != 0))

/* Get a specific byte, without range checks. */
#define BYTE_BITS         (8)
#define BYTE_CHECKS       (0xff)
#define GET_BYTE(X, i)    (((X)->p[(i) / CIL] >> \
                          (((i) % CIL) * BYTE_BITS)) & BYTE_CHECKS)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static sem_t g_rsa_sem = SEM_INITIALIZER(1);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32c3_mpi_to_mem_block
 *
 * Description:
 *   Copy MPI bignum 'mpi' to hardware memory block.
 *
 * Input Parameters:
 *   mem_base    - The hardware memory block
 *   mpi         - The bignum 'mpi' from the previous calculation
 *   num_words   - The number of words to be represented
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void esp32c3_mpi_to_mem_block(uint32_t mem_base,
                                     const struct esp32c3_mpi_s *mpi,
                                     size_t num_words)
{
  uint32_t *pbase = (uint32_t *)mem_base;
  uint32_t copy_words = MIN(num_words, mpi->n);
  int i;

  /* Copy MPI data to memory block registers */

  memcpy(pbase, mpi->p, copy_words * sizeof(uint32_t));

  /* Zero any remaining memory block data */

  for (i = copy_words; i < num_words; i++)
    {
      pbase[i] = 0;
    }
}

/****************************************************************************
 * Name: esp32c3_mem_block_to_mpi
 *
 * Description:
 *   Read MPI bignum back from hardware memory block.
 *
 * Input Parameters:
 *   x           - The result from the previous calculation
 *   mem_base    - The hardware memory block
 *   num_words   - The number of words to be represented
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void esp32c3_mem_block_to_mpi(struct esp32c3_mpi_s *x,
                                     uint32_t mem_base, int num_words)
{
  int i;

  /* Copy data from memory block registers */

  const size_t REG_WIDTH = sizeof(uint32_t);
  for (i = 0; i < num_words; i++)
    {
      x->p[i] = getreg32(mem_base + (i * REG_WIDTH));
    }

  /* Zero any remaining limbs in the bignum,
   * if the buffer is bigger than num_words
   */

  for (i = num_words; i < x->n; i++)
    {
      x->p[i] = 0;
    }
}

/****************************************************************************
 * Name: esp32c3_mpi_start_op
 *
 * Description:
 *   Begin an RSA operation.
 *
 * Input Parameters:
 *   op_reg   - Specifies which 'START' register to write to.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void esp32c3_mpi_start_op(uint32_t op_reg)
{
  /* Clear interrupt status */

  putreg32(1, RSA_CLEAR_INTERRUPT_REG);

  /* Note: above putreg32 includes a memw, so we know any writes
   * to the memory blocks are also complete.
   */

  putreg32(1, op_reg);
}

/****************************************************************************
 * Name: esp32c3_mpi_wait_op_complete
 *
 * Description:
 *   Wait for an RSA operation to complete.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void esp32c3_mpi_wait_op_complete(void)
{
  while (getreg32(RSA_IDLE_REG) != 1)
    {
    }

  /* clear the interrupt */

  putreg32(1, RSA_CLEAR_INTERRUPT_REG);
}

/****************************************************************************
 * Name: esp32c3_mpi_enable_hardware_hw_op
 *
 * Description:
 *   Enable the MPI hardware and acquire the lock.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void esp32c3_mpi_enable_hardware_hw_op(void)
{
  nxsem_wait(&g_rsa_sem);

  /* Enable RSA hardware */

  modifyreg32(SYSTEM_PERIP_CLK_EN1_REG, 0, SYSTEM_CRYPTO_RSA_CLK_EN);
  modifyreg32(SYSTEM_PERIP_RST_EN1_REG, (SYSTEM_CRYPTO_RSA_RST |
                                         SYSTEM_CRYPTO_DS_RST), 0);

  modifyreg32(SYSTEM_RSA_PD_CTRL_REG, SYSTEM_RSA_MEM_PD, 0);

  while (getreg32(RSA_CLEAN_REG) != 1)
    {
    }
}

/****************************************************************************
 * Name: esp32c3_mpi_disable_hardware_hw_op
 *
 * Description:
 *   Disable the MPI hardware and release the lock.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void esp32c3_mpi_disable_hardware_hw_op(void)
{
  modifyreg32(SYSTEM_RSA_PD_CTRL_REG, 0, SYSTEM_RSA_MEM_PD);

  /* Disable RSA hardware */

  modifyreg32(SYSTEM_PERIP_CLK_EN1_REG, SYSTEM_CRYPTO_RSA_CLK_EN, 0);
  modifyreg32(SYSTEM_PERIP_RST_EN1_REG, 0, SYSTEM_CRYPTO_RSA_RST);

  nxsem_post(&g_rsa_sem);
}

/****************************************************************************
 * Name: esp32c3_mpi_read_result_hw_op
 *
 * Description:
 *   Read out the result from the previous calculation.
 *
 * Input Parameters:
 *   Z          - The result from the previous calculation
 *   z_words    - The number of words to be represented
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void esp32c3_mpi_read_result_hw_op(struct esp32c3_mpi_s *Z,
                                          size_t z_words)
{
  esp32c3_mpi_wait_op_complete();
  esp32c3_mem_block_to_mpi(Z, RSA_MEM_Z_BLOCK_REG, z_words);
}

/****************************************************************************
 * Name: esp32c3_mpi_mul_mpi_hw_op
 *
 * Description:
 *   Starts a (X * Y) calculation in hardware.
 *
 * Input Parameters:
 *   X          - First multiplication argument
 *   Y          - Second multiplication argument
 *   n_words    - The number of words to be represented
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void esp32c3_mpi_mul_mpi_hw_op(const struct esp32c3_mpi_s *X,
                                      const struct esp32c3_mpi_s *Y,
                                      size_t n_words)
{
  /* Copy X (right-extended) & Y (left-extended) to memory block */

  esp32c3_mpi_to_mem_block(RSA_MEM_X_BLOCK_REG, X, n_words);
  esp32c3_mpi_to_mem_block(RSA_MEM_Z_BLOCK_REG + n_words * 4, Y, n_words);

  putreg32(((n_words * 2) - 1), RSA_MODE_REG);
  esp32c3_mpi_start_op(RSA_MULT_START_REG);
}

/****************************************************************************
 * Name: esp32c3_mpi_mult_failover_mod_op
 *
 * Description:
 *   Special-case of (X * Y), where we use hardware montgomery mod
 *   multiplication to calculate result where either A or B are > 2048 bits
 *   so can't use the standard multiplication method.
 *
 * Input Parameters:
 *   X          - First multiplication argument
 *   Y          - Second multiplication argument
 *   num_words  - The number of words to be represented
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void esp32c3_mpi_mult_failover_mod_op(const struct esp32c3_mpi_s *X,
                                             const struct esp32c3_mpi_s *Y,
                                             size_t num_words)
{
  int i;

  /* M = 2^num_words - 1, so block is entirely FF */

  for (i = 0; i < num_words; i++)
    {
      putreg32(UINT32_MAX, RSA_MEM_M_BLOCK_REG + i * 4);
    }

  /* mprime = 1 */

  putreg32(1, RSA_M_PRIME_REG);
  putreg32(num_words - 1, RSA_MODE_REG);

  /* Load X & Y */

  esp32c3_mpi_to_mem_block(RSA_MEM_X_BLOCK_REG, X, num_words);
  esp32c3_mpi_to_mem_block(RSA_MEM_Y_BLOCK_REG, Y, num_words);

  /* rinv = 1, write first word */

  putreg32(1, RSA_MEM_RB_BLOCK_REG);

  /* Zero out rest of the rinv words */

  for (i = 1; i < num_words; i++)
    {
      putreg32(0, RSA_MEM_RB_BLOCK_REG + i * 4);
    }

  esp32c3_mpi_start_op(RSA_MODMULT_START_REG);
}

/****************************************************************************
 * Name: esp32c3_mpi_zeroize
 *
 * Description:
 *   Zero any limbs data.
 *
 * Input Parameters:
 *   v     - The pointer to limbs
 *   n     - The total number of limbs
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void esp32c3_mpi_zeroize(uint32_t *v, size_t n)
{
  memset(v, 0, CIL * n);
}

/****************************************************************************
 * Name: bits_to_words
 *
 * Description:
 *   Convert bit count to 32-bits word count.
 *
 * Input Parameters:
 *   bits  - The number of bit count
 *
 * Returned Value:
 *   Number of words count.
 *
 ****************************************************************************/

static size_t bits_to_words(size_t bits)
{
  return (bits + 31) / 32;
}

/****************************************************************************
 * Name: mpi_sub_hlp
 *
 * Description:
 *   Helper for esp32c3_mpi subtraction
 *
 * Input Parameters:
 *   n       - Number of limbs of \p d and \p s
 *   d       - On input, the left operand, On output, the result operand
 *   s       - The right operand
 *
 * Returned Value:
 *   \c 1 if \p `d < \p s`.
 *   \c 0 if \p `d >= \p s`..
 *
 ****************************************************************************/

static uint32_t mpi_sub_hlp(size_t n,
                            uint32_t *d,
                            const uint32_t *s)
{
  size_t i;
  uint32_t c;
  uint32_t z;

  for (i = c = 0; i < n; i++, s++, d++)
    {
      z   = (*d <  c);
      *d -= c;
      c   = (*d < *s) + z;
      *d -= *s;
    }

  return c;
}

/****************************************************************************
 * Name: mpi_mul_addc
 *
 * Description:
 *   Helper for esp32c3_mpi multiplication
 *
 * Input Parameters:
 *   count   - The count of limbs
 *   c       - The result number of limbs
 *   s       - The target number of limbs
 *   d       - The pointer to limbs
 *   b       - The total number of limbs
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static inline void mpi_mul_addc(uint32_t count, uint32_t *c,
                                uint32_t **s, uint32_t **d, uint32_t b)
{
  uint32_t s0;
  uint32_t s1;
  uint32_t b0;
  uint32_t b1;
  uint32_t r0;
  uint32_t r1;
  uint32_t rx;
  uint32_t ry;

  b0 = (b << BIH) >> BIH;
  b1 = (b >> BIH);

  for (int i = 0; i < count; ++i)
    {
      s0 = (**s << BIH) >> BIH;
      s1 = (**s >> BIH);
      (*s)++;
      rx = s0 * b1;
      r0 = s0 * b0;
      ry = s1 * b0;
      r1 = s1 * b1;
      r1 += (rx >> BIH);
      r1 += (ry >> BIH);
      rx <<= BIH;
      ry <<= BIH;
      r0 += rx;
      r1 += (r0 < rx);
      r0 += ry;
      r1 += (r0 < ry);
      r0 += *c;
      r1 += (r0 < *c);
      r0 += **d;
      r1 += (r0 < **d);
      *c = r1;
      *((*d)++) = r0;
    }
}

/****************************************************************************
 * Name: mpi_mul_hlp
 *
 * Description:
 *   Helper for esp32c3_mpi multiplication
 *
 * Input Parameters:
 *   i       - The MPI context to grow
 *   s       - The target number of limbs
 *   d       - The pointer to limbs
 *   b       - The total number of limbs
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static void mpi_mul_hlp(size_t i, uint32_t *s, uint32_t *d, uint32_t b)
{
  uint32_t c = 0;

  for (; i >= 16; i -= 16)
    {
      mpi_mul_addc(16, &c, &s, &d, b);
    }

  for (; i >= 8; i -= 8)
    {
      mpi_mul_addc(8, &c, &s, &d, b);
    }

  for (; i > 0; i--)
    {
      mpi_mul_addc(1, &c, &s, &d, b);
    }

  do
    {
      *d += c;
      c = (*d < c);
      d++;
    }
  while (c != 0);
}

/****************************************************************************
 * Name: mpi_safe_cond_assign
 *
 * Description:
 *   Conditionally assign dest = src, without leaking information
 *   about whether the assignment was made or not.
 *
 * Input Parameters:
 *   n       - The MPI context to grow
 *   dest    - The MPI to conditionally assign to
 *   src     - The MPI to conditionally assign from
 *   assign  - The condition deciding whether perform the assignment or not
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void mpi_safe_cond_assign(size_t n,
                                 uint32_t *dest,
                                 const uint32_t *src,
                                 unsigned char assign)
{
  size_t i;
  for (i = 0; i < n; i++)
    {
      dest[i] = dest[i] * (1 - assign) + src[i] * assign;
    }
}

/****************************************************************************
 * Name: mpi_montg_init
 *
 * Description:
 *   Fast Montgomery initialization
 *
 * Input Parameters:
 *   X       - The MPI context to grow
 *   nblimbs - The target number of limbs
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static void mpi_montg_init(uint32_t *mm, const struct esp32c3_mpi_s *N)
{
  uint32_t x;
  uint32_t m0 = N->p[0];
  unsigned int i;

  x = m0 + (((m0 + 2) & 4) << 1);

  for (i = BIL; i >= 8; i /= 2)
    {
      x *= (2 - (m0 * x));
    }

  *mm = ~x + 1;
}

/****************************************************************************
 * Name: mpi_montmul
 *
 * Description:
 *   Montgomery multiplication: A = A * B * R^-1 mod N
 *
 * Input Parameters:
 *   A   - One of the numbers to multiply,
 *         It must have at least as many limbs as N
 *         (A->n >= N->n), and any limbs beyond n are ignored.
 *         On successful completion, A contains the result of
 *         the multiplication A * B * R^-1 mod N where
 *         R = (2^CIL)^n.
 *   B   - One of the numbers to multiply, It must be nonzero
 *         and must not have more limbs than N (B->n <= N->n)
 *   N   - The modulo. N must be odd.
 *   mm  - The value calculated by `mpi_montg_init(&mm, N)`.
 *         This is -N^-1 mod 2^CIL.
 *   T   - A bignum for temporary storage.
 *         It must be at least twice the limb size of N plus 2
 *         (T->n >= 2 * (N->n + 1)).
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void mpi_montmul(struct esp32c3_mpi_s *A,
                        const struct esp32c3_mpi_s *B,
                        const struct esp32c3_mpi_s *N,
                        uint32_t mm,
                        const struct esp32c3_mpi_s *T)
{
  size_t i;
  size_t n;
  size_t m;
  uint32_t u0;
  uint32_t u1;
  uint32_t *d;

  memset(T->p, 0, T->n * CIL);

  d = T->p;
  n = N->n;
  m = (B->n < n) ? B->n : n;

  for (i = 0; i < n; i++)
    {
      /* T = (T + u0*B + u1*N) / 2^BIL */

      u0 = A->p[i];
      u1 = (d[0] + u0 * B->p[0]) * mm;

      mpi_mul_hlp(m, B->p, d, u0);
      mpi_mul_hlp(n, N->p, d, u1);

      *d++ = u0;
      d[n + 1] = 0;
    }

  /* At this point, d is either the desired result or the desired result
   * plus N. We now potentially subtract N, avoiding leaking whether the
   * subtraction is performed through side channels.
   */

  /* Copy the n least significant limbs of d to A, so that
   * A = d if d < N (recall that N has n limbs).
   */

  memcpy(A->p, d, n * CIL);

  /* If d >= N then we want to set A to d - N. To prevent timing attacks,
   * do the calculation without using conditional tests.
   */

  /* Set d to d0 + (2^BIL)^n - N where d0 is the current value of d.
   */

  d[n] += 1;
  d[n] -= mpi_sub_hlp(n, d, N->p);
  /* If d0 < N then d < (2^BIL)^n
   * so d[n] == 0 and we want to keep A as it is.
   * If d0 >= N then d >= (2^BIL)^n, and d <= (2^BIL)^n + N < 2 * (2^BIL)^n
   * so d[n] == 1 and we want to set A to the result of the subtraction
   * which is d - (2^BIL)^n, i.e. the n least significant limbs of d.
   * This exactly corresponds to a conditional assignment.
   */

  mpi_safe_cond_assign(n, A->p, d, (unsigned char) d[n]);
}

/****************************************************************************
 * Name: mpi_montred
 *
 * Description:
 *   Montgomery reduction: A = A * R^-1 mod N
 *
 * Input Parameters:
 *   A   - One of the numbers to multiply,
 *         It must have at least as many limbs as N
 *         (A->n >= N->n), and any limbs beyond n are ignored.
 *         On successful completion, A contains the result of
 *         the multiplication A * B * R^-1 mod N where
 *         R = (2^CIL)^n.
 *   N   - The modulo. N must be odd.
 *   mm  - The value calculated by `mpi_montg_init(&mm, N)`.
 *         This is -N^-1 mod 2^CIL.
 *   T   - A bignum for temporary storage.
 *         It must be at least twice the limb size of N plus 2
 *         (T->n >= 2 * (N->n + 1)).
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static void mpi_montred(struct esp32c3_mpi_s *A,
                        const struct esp32c3_mpi_s *N,
                        uint32_t mm,
                        const struct esp32c3_mpi_s *T)
{
  uint32_t z = 1;
  struct esp32c3_mpi_s U;

  U.n = (int) z;
  U.s = (int) z;
  U.p = &z;

  mpi_montmul(A, &U, N, mm, T);
}

/****************************************************************************
 * Name: mpi_mult_mpi_overlong
 *
 * Description:
 *   Deal with the case when X & Y are too long for the hardware unit,
 *   by splitting one operand into two halves.
 *
 * Input Parameters:
 *   Z       - The destination MPI
 *   X       - The first factor
 *   Y       - The second factor
 *   y_words - The number of words to be process
 *   z_words - The number of words to be represented
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static int mpi_mult_mpi_overlong(struct esp32c3_mpi_s *Z,
                                 const struct esp32c3_mpi_s *X,
                                 const struct esp32c3_mpi_s *Y,
                                 size_t y_words, size_t z_words)
{
  int ret = 0;
  struct esp32c3_mpi_s ztemp;

  /* Rather than slicing in two on bits we slice on limbs (32 bit words) */

  const size_t words_slice = y_words / 2;

  /* yp holds lower bits of Y */

  const struct esp32c3_mpi_s yp = {
    .p = Y->p,
    .n = words_slice,
    .s = Y->s
  };

  /* ypp holds upper bits of Y,
   * right shifted (also reuses Y's array contents)
   */

  const struct esp32c3_mpi_s ypp = {
    .p = Y->p + words_slice,
    .n = y_words - words_slice,
    .s = Y->s
  };

  esp32c3_mpi_init(&ztemp);

  /* Get result ztemp = yp * X (need temporary variable ztemp) */

  ESP32C3_MPI_CHK(esp32c3_mpi_mul_mpi(&ztemp, X, &yp), cleanup);

  /* Z = ypp * Y */

  ESP32C3_MPI_CHK(esp32c3_mpi_mul_mpi(Z, X, &ypp), cleanup);

  /* Z = Z << b */

  ESP32C3_MPI_CHK(esp32c3_mpi_shift_l(Z, words_slice * 32), cleanup);

  /* Z += ztemp */

  ESP32C3_MPI_CHK(esp32c3_mpi_add_mpi(Z, Z, &ztemp), cleanup);

cleanup:
  esp32c3_mpi_free(&ztemp);

  return ret;
}

/****************************************************************************
 * Name: mpi_mult_mpi_failover_mod_mult
 *
 * Description:
 *   Where we use hardware montgomery mod multiplication to calculate an
 *   esp32c3_mpi_mult_mpi result where either A or B are > 2048 bits
 *   so can't use the standard multiplication method.
 *
 * Input Parameters:
 *   Z       - The destination MPI
 *   X       - The first factor
 *   Y       - The second factor
 *   z_words - The number of words to be represented
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static int mpi_mult_mpi_failover_mod_mult(struct esp32c3_mpi_s *Z,
                                          const struct esp32c3_mpi_s *X,
                                          const struct esp32c3_mpi_s *Y,
                                          size_t z_words)
{
  int ret;

  esp32c3_mpi_enable_hardware_hw_op();

  esp32c3_mpi_mult_failover_mod_op(X, Y, z_words);
  ESP32C3_MPI_CHK(esp32c3_mpi_grow(Z, z_words), cleanup);
  esp32c3_mpi_read_result_hw_op(Z, z_words);

  Z->s = X->s * Y->s;
cleanup:
  esp32c3_mpi_disable_hardware_hw_op();
  return ret;
}

/****************************************************************************
 * Name: esp32c3_bignum_clz
 *
 * Description:
 *   Count leading zero bits in a given integer
 *
 * Input Parameters:
 *   x       - The MPI context to query
 *
 * Returned Value:
 *   The count leading zero bits in a given integer.
 *
 ****************************************************************************/

static size_t esp32c3_bignum_clz(const uint32_t x)
{
  size_t j;
  uint32_t mask = UINT32_C(1) << (BIL - 1);

  for (j = 0; j < BIL; j++)
    {
      if (x & mask)
        {
          break;
        }

      mask >>= 1;
    }

  return j;
}

/****************************************************************************
 * Name: mpi_get_digit
 *
 * Description:
 *   Convert an ASCII character to digit value
 *
 * Input Parameters:
 *   d       - The destination MPI
 *   radix   - The numeric base of the input character
 *   c       - An ASCII character
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static int mpi_get_digit(uint32_t *d, int radix, char c)
{
  *d = 255;

  if (c >= 0x30 && c <= 0x39)
    {
      *d = c - 0x30;
    }

  if (c >= 0x41 && c <= 0x46)
    {
      *d = c - 0x37;
    }

  if (c >= 0x61 && c <= 0x66)
    {
      *d = c - 0x57;
    }

  if (*d >= (uint32_t) radix)
    {
      return ESP32C3_ERR_MPI_INVALID_CHARACTER;
    }

  return OK;
}

/****************************************************************************
 * Name: mpi_write_hlp
 *
 * Description:
 *   Helper to write the digits high-order first
 *
 * Input Parameters:
 *   X       - The source MPI
 *   radix   - The numeric base of the output string
 *   p       - The buffer to write the string to
 *   buflen  - The available size in Bytes of p
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static int mpi_write_hlp(struct esp32c3_mpi_s *X, int radix,
                         char **p, const size_t buflen)
{
  int ret;
  uint32_t r;
  size_t length = 0;
  char *p_end = *p + buflen;

  do
    {
      if (length >= buflen)
        {
          return ESP32C3_ERR_MPI_BUFFER_TOO_SMALL;
        }

      ESP32C3_MPI_CHK(esp32c3_mpi_mod_int(&r, X, radix), cleanup);
      ESP32C3_MPI_CHK(esp32c3_mpi_div_int(X, NULL, X, radix), cleanup);

      /* Write the residue in the current position, as an ASCII character.
       */

      if (r < 0xa)
        {
          *(--p_end) = (char)('0' + r);
        }
      else
        {
          *(--p_end) = (char)('A' + (r - 0xa));
        }

      length++;
    }
  while (esp32c3_mpi_cmp_int(X, 0) != 0);

  memmove(*p, p_end, length);
  *p += length;

cleanup:

  return ret;
}

/****************************************************************************
 * Name: mpi_uint_bigendian_to_host
 *
 * Description:
 *   Convert a big-endian byte array aligned to the size of uint32_t
 *   into the storage form used by esp32c3_mpi
 *
 * Input Parameters:
 *   X       - The MPI context to convert
 *
 * Returned Value:
 *   The size of uint32_t into the storage form used by esp32c3_mpi.
 *
 ****************************************************************************/

static uint32_t mpi_uint_bigendian_to_host(uint32_t x)
{
  uint8_t i;
  unsigned char *x_ptr;
  uint32_t tmp = 0;

  for (i = 0, x_ptr = (unsigned char *) &x; i < CIL; i++, x_ptr++)
    {
      tmp <<= CHAR_BIT;
      tmp |= (uint32_t) *x_ptr;
    }

  return (tmp);
}

/****************************************************************************
 * Name: mpi_bigendian_to_host
 *
 * Description:
 *   Enlarge an MPI to the specified number of limbs
 *
 * Input Parameters:
 *   p       - The MPI context to grow
 *   limbs   - The target number of limbs
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static void mpi_bigendian_to_host(uint32_t * const p, size_t limbs)
{
  uint32_t *cur_limb_left;
  uint32_t *cur_limb_right;
  if (limbs == 0)
    {
      return ;
    }

  /* Traverse limbs and
   * - adapt byte-order in each limb
   * - swap the limbs themselves.
   * For that, simultaneously traverse the limbs from left to right
   * and from right to left, as long as the left index is not bigger
   * than the right index (it's not a problem if limbs is odd and the
   * indices coincide in the last iteration).
   */

  for (cur_limb_left = p, cur_limb_right = p + (limbs - 1);
     cur_limb_left <= cur_limb_right;
     cur_limb_left++, cur_limb_right--)
    {
      uint32_t tmp;

      /* Note that if cur_limb_left == cur_limb_right,
       * this code effectively swaps the bytes only once.
       */

      tmp = mpi_uint_bigendian_to_host(*cur_limb_left);
      *cur_limb_left = mpi_uint_bigendian_to_host(*cur_limb_right);
      *cur_limb_right = tmp;
    }
}

/****************************************************************************
 * Name: ct_lt_mpi_uint
 *
 * Description:
 *   Decide if an integer is less than the other, without branches.
 *
 * Input Parameters:
 *   X       - First integer
 *   nblimbs - Second integer
 *
 * Returned Value:
 *   1 if \p x is less than \p y, 0 otherwise.
 *
 ****************************************************************************/

static unsigned ct_lt_mpi_uint(const uint32_t x,
                               const uint32_t y)
{
  uint32_t ret;
  uint32_t cond;

  /* Check if the most significant bits (MSB) of the operands are different.
   */

  cond = (x ^ y);

  /* If the MSB are the same then the difference x-y will be negative (and
   * have its MSB set to 1 during conversion to unsigned) if and only if x<y.
   */

  ret = (x - y) & ~cond;

  /* If the MSB are different, then the operand with the MSB of 1 is the
   * bigger. (That is if y has MSB of 1, then x<y is true and it is false if
   * the MSB of y is 0.)
   */

  ret |= y & cond;

  ret = ret >> (BIL - 1);

  return (unsigned) ret;
}

/****************************************************************************
 * Name: esp32c3_bignum_int_div_int
 *
 * Description:
 *   Unsigned integer divide - double uint32_t dividend, u1/u0, and
 *   uint32_t divisor, d
 *
 * Input Parameters:
 *   X       - The MPI context to grow
 *   nblimbs - The target number of limbs
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static uint32_t esp32c3_bignum_int_div_int(uint32_t u1, uint32_t u0,
                                           uint32_t d, uint32_t *r)
{
  const uint32_t radix = (uint32_t) 1 << BIH;
  const uint32_t uint_halfword_mask = ((uint32_t) 1 << BIH) - 1;
  uint32_t d0;
  uint32_t d1;
  uint32_t q0;
  uint32_t q1;
  uint32_t rax;
  uint32_t r0;
  uint32_t quotient;
  uint32_t u0_msw;
  uint32_t u0_lsw;
  size_t s;

  /* Check for overflow */

  if (0 == d || u1 >= d)
    {
      if (r != NULL)
        {
          *r = ~0;
        }

      return (~0);
    }

  /* Algorithm D, Section 4.3.1 - The Art of Computer Programming
   *   Vol. 2 - Seminumerical Algorithms, Knuth
   */

  /* Normalize the divisor, d, and dividend, u0, u1
   */

  s = esp32c3_bignum_clz(d);
  d = d << s;

  u1 = u1 << s;
  u1 |= (u0 >> (BIL - s)) & (-(int32_t)s >> (BIL - 1));
  u0 =  u0 << s;

  d1 = d >> BIH;
  d0 = d & uint_halfword_mask;

  u0_msw = u0 >> BIH;
  u0_lsw = u0 & uint_halfword_mask;

  /* Find the first quotient and remainder
   */

  q1 = u1 / d1;
  r0 = u1 - d1 * q1;

  while (q1 >= radix || (q1 * d0 > radix * r0 + u0_msw))
    {
      q1 -= 1;
      r0 += d1;

      if (r0 >= radix)
        {
          break;
        }
    }

  rax = (u1 * radix) + (u0_msw - q1 * d);
  q0 = rax / d1;
  r0 = rax - q0 * d1;

  while (q0 >= radix || (q0 * d0 > radix * r0 + u0_lsw))
    {
      q0 -= 1;
      r0 += d1;

      if (r0 >= radix)
        {
          break;
        }
    }

  if (r != NULL)
    {
      *r = (rax * radix + u0_lsw - q0 * d) >> s;
    }

  quotient = q1 * radix + q0;

  return quotient;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32c3_mpi_init
 *
 * Description:
 *   Initialize an MPI context
 *
 * Input Parameters:
 *   X      - The MPI context to initialize
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32c3_mpi_init(struct esp32c3_mpi_s *X)
{
  DEBUGASSERT(X != NULL);

  X->s = 1;
  X->n = 0;
  X->p = NULL;
}

/****************************************************************************
 * Name: esp32c3_mpi_free
 *
 * Description:
 *   Frees the components of an MPI context
 *
 * Input Parameters:
 *   X      - The MPI context to be cleared
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32c3_mpi_free(struct esp32c3_mpi_s *X)
{
  if (X == NULL)
    {
      return ;
    }

  if (X->p != NULL)
    {
      esp32c3_mpi_zeroize(X->p, X->n);
      free(X->p);
    }

  X->s = 1;
  X->n = 0;
  X->p = NULL;
}

/****************************************************************************
 * Name: esp32c3_mpi_grow
 *
 * Description:
 *   Enlarge an MPI to the specified number of limbs
 *
 * Input Parameters:
 *   X       - The MPI context to grow
 *   nblimbs - The target number of limbs
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_grow(struct esp32c3_mpi_s *X, size_t nblimbs)
{
  uint32_t *p;
  DEBUGASSERT(X != NULL);

  if (nblimbs > ESP32C3_MPI_MAX_LIMBS)
    {
      return ESP32C3_ERR_MPI_ALLOC_FAILED;
    }

  if (X->n < nblimbs)
    {
      if ((p = (uint32_t *)calloc(nblimbs, CIL)) == NULL)
        {
          return ESP32C3_ERR_MPI_ALLOC_FAILED;
        }

      if (X->p != NULL)
        {
          memcpy(p, X->p, X->n * CIL);
          esp32c3_mpi_zeroize(X->p, X->n);
          free(X->p);
        }

      X->n = nblimbs;
      X->p = p;
    }

  return OK;
}

/****************************************************************************
 * Name: esp32c3_mpi_shrink
 *
 * Description:
 *   Resizes an MPI downwards, keeping at least the specified number of limbs
 *
 * Input Parameters:
 *   X       - The MPI context to shrink
 *   nblimbs - The minimum number of limbs
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_shrink(struct esp32c3_mpi_s *X, size_t nblimbs)
{
  uint32_t *p;
  size_t i;
  DEBUGASSERT(X != NULL);

  if (nblimbs > ESP32C3_MPI_MAX_LIMBS)
    {
      return ESP32C3_ERR_MPI_ALLOC_FAILED;
    }

  /* Actually resize up if there are currently fewer than nblimbs limbs. */

  if (X->n <= nblimbs)
    {
      return (esp32c3_mpi_grow(X, nblimbs));
    }

  /* After this point, then X->n > nblimbs and in particular X->n > 0. */

  for (i = X->n - 1; i > 0; i--)
    {
      if (X->p[i] != 0)
        {
          break;
        }
    }

  i++;

  if (i < nblimbs)
    {
      i = nblimbs;
    }

  if ((p = (uint32_t *)calloc(i, CIL)) == NULL)
    {
      return ESP32C3_ERR_MPI_ALLOC_FAILED;
    }

  if (X->p != NULL)
    {
      memcpy(p, X->p, i * CIL);
      esp32c3_mpi_zeroize(X->p, X->n);
      free(X->p);
    }

  X->n = i;
  X->p = p;

  return OK;
}

/****************************************************************************
 * Name: esp32c3_mpi_copy
 *
 * Description:
 *   Copy the contents of Y into X
 *
 * Input Parameters:
 *   X       - The destination MPI
 *   Y       - The source MPI
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_copy(struct esp32c3_mpi_s *X,
                     const struct esp32c3_mpi_s *Y)
{
  int ret = 0;
  size_t i;
  DEBUGASSERT(X != NULL);
  DEBUGASSERT(Y != NULL);

  if (X == Y)
    {
      return OK;
    }

  if (Y->n == 0)
    {
      esp32c3_mpi_free(X);
      return OK;
    }

  for (i = Y->n - 1; i > 0; i--)
    {
      if (Y->p[i] != 0)
        {
          break;
        }
    }

  i++;

  X->s = Y->s;

  if (X->n < i)
    {
      ESP32C3_MPI_CHK(esp32c3_mpi_grow(X, i), cleanup);
    }
  else
    {
      memset(X->p + i, 0, (X->n - i) * CIL);
    }

  memcpy(X->p, Y->p, i * CIL);

cleanup:

  return ret;
}

/****************************************************************************
 * Name: esp32c3_mpi_swap
 *
 * Description:
 *   Swap the contents of X and Y
 *
 * Input Parameters:
 *   X       - The first MPI
 *   nblimbs - The second MPI
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32c3_mpi_swap(struct esp32c3_mpi_s *X, struct esp32c3_mpi_s *Y)
{
  struct esp32c3_mpi_s T;
  DEBUGASSERT(X != NULL);
  DEBUGASSERT(Y != NULL);

  memcpy(&T,  X, sizeof(struct esp32c3_mpi_s));
  memcpy(X,  Y, sizeof(struct esp32c3_mpi_s));
  memcpy(Y, &T, sizeof(struct esp32c3_mpi_s));
}

/****************************************************************************
 * Name: esp32c3_mpi_safe_cond_assign
 *
 * Description:
 *   Perform a safe conditional copy of MPI which doesn't
 *   reveal whether the condition was true or not.
 *
 * Input Parameters:
 *   X       - The MPI to conditionally assign to
 *   Y       - The MPI to be assigned from
 *   assign  - The condition deciding whether perform the assignment or not
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_safe_cond_assign(struct esp32c3_mpi_s *X,
                                 const struct esp32c3_mpi_s *Y,
                                 unsigned char assign)
{
  int ret = 0;
  size_t i;
  DEBUGASSERT(X != NULL);
  DEBUGASSERT(Y != NULL);

  /* make sure assign is 0 or 1 in a time-constant manner */

  assign = (assign | (unsigned char)-assign) >> 7;

  ESP32C3_MPI_CHK(esp32c3_mpi_grow(X, Y->n), cleanup);

  X->s = X->s * (1 - assign) + Y->s * assign;

  mpi_safe_cond_assign(Y->n, X->p, Y->p, assign);

  for (i = Y->n; i < X->n; i++)
    {
      X->p[i] *= (1 - assign);
    }

cleanup:
  return ret;
}

/****************************************************************************
 * Name: esp32c3_mpi_safe_cond_swap
 *
 * Description:
 *   Perform a safe conditional swap which doesn't
 *   reveal whether the condition was true or not.
 *
 * Input Parameters:
 *   X       - The first MPI
 *   Y       - The second MPI
 *   swap    - The condition deciding whether to perform the swap or not
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_safe_cond_swap(struct esp32c3_mpi_s *X,
                               struct esp32c3_mpi_s *Y,
                               unsigned char swap)
{
  int ret;
  int s;
  size_t i;
  uint32_t tmp;
  DEBUGASSERT(X != NULL);
  DEBUGASSERT(Y != NULL);

  if (X == Y)
    {
      return OK;
    }

  /* make sure swap is 0 or 1 in a time-constant manner */

  swap = (swap | (unsigned char)-swap) >> 7;

  ESP32C3_MPI_CHK(esp32c3_mpi_grow(X, Y->n), cleanup);
  ESP32C3_MPI_CHK(esp32c3_mpi_grow(Y, X->n), cleanup);

  s = X->s;
  X->s = X->s * (1 - swap) + Y->s * swap;
  Y->s = Y->s * (1 - swap) +  s * swap;

  for (i = 0; i < X->n; i++)
    {
      tmp = X->p[i];
      X->p[i] = X->p[i] * (1 - swap) + Y->p[i] * swap;
      Y->p[i] = Y->p[i] * (1 - swap) +   tmp * swap;
    }

cleanup:
  return ret;
}

/****************************************************************************
 * Name: esp32c3_mpi_lset
 *
 * Description:
 *   Set value from integer
 *
 * Input Parameters:
 *   X       - The MPI to set
 *   z       - The value to use
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_lset(struct esp32c3_mpi_s *X, int32_t z)
{
  int ret;
  DEBUGASSERT(X != NULL);

  ESP32C3_MPI_CHK(esp32c3_mpi_grow(X, 1), cleanup);
  memset(X->p, 0, X->n * CIL);

  X->p[0] = (z < 0) ? -z : z;
  X->s  = (z < 0) ? -1 : 1;

cleanup:

  return ret;
}

/****************************************************************************
 * Name: esp32c3_mpi_get_bit
 *
 * Description:
 *   Get a specific bit from an MPI
 *
 * Input Parameters:
 *   X       - The MPI context to query
 *   pos     - Zero-based index of the bit to query
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_get_bit(const struct esp32c3_mpi_s *X, size_t pos)
{
  DEBUGASSERT(X != NULL);

  if (X->n * BIL <= pos)
    {
      return OK;
    }

  return ((X->p[pos / BIL] >> (pos % BIL)) & 0x01);
}

/****************************************************************************
 * Name: esp32c3_mpi_set_bit
 *
 * Description:
 *   Modify a specific bit in an MPI
 *
 * Input Parameters:
 *   X       - The MPI context to modify
 *   pos     - Zero-based index of the bit to modify
 *   val     - The desired value of bit
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_set_bit(struct esp32c3_mpi_s *X,
                        size_t pos, unsigned char val)
{
  int ret = 0;
  size_t off = pos / BIL;
  size_t idx = pos % BIL;
  DEBUGASSERT(X != NULL);

  if (val != 0 && val != 1)
    {
      return ESP32C3_ERR_MPI_BAD_INPUT_DATA;
    }

  if (X->n * BIL <= pos)
    {
      if (val == 0)
        {
          return OK;
        }

      ESP32C3_MPI_CHK(esp32c3_mpi_grow(X, off + 1), cleanup);
    }

  X->p[off] &= ~((uint32_t) 0x01 << idx);
  X->p[off] |= (uint32_t) val << idx;

cleanup:

  return ret;
}

/****************************************************************************
 * Name: esp32c3_mpi_lsb
 *
 * Description:
 *   Return the number of bits of value
 *
 * Input Parameters:
 *   X       - The MPI context to query
 *
 * Returned Value:
 *   The number of bits of value.
 *
 ****************************************************************************/

size_t esp32c3_mpi_lsb(const struct esp32c3_mpi_s *X)
{
  size_t i;
  size_t j;
  size_t count = 0;
  DEBUGASSERT(X != NULL);

  for (i = 0; i < X->n; i++)
    {
      for (j = 0; j < BIL; j++, count++)
        {
          if (((X->p[i] >> j) & 1) != 0)
            {
              return (count);
            }
        }
    }

  return OK;
}

/****************************************************************************
 * Name: esp32c3_mpi_bitlen
 *
 * Description:
 *   Return the number of bits up to and including the most
 *   significant bit of value
 *
 * Input Parameters:
 *   X       - The MPI context to query
 *
 * Returned Value:
 *   The number of bits up and including the most significant bit of value.
 *
 ****************************************************************************/

size_t esp32c3_mpi_bitlen(const struct esp32c3_mpi_s *X)
{
  size_t i;
  size_t j;

  if (X->n == 0)
    {
      return OK;
    }

  for (i = X->n - 1; i > 0; i--)
    {
      if (X->p[i] != 0)
        {
          break;
        }
    }

  j = BIL - esp32c3_bignum_clz(X->p[i]);

  return ((i * BIL) + j);
}

/****************************************************************************
 * Name: esp32c3_mpi_size
 *
 * Description:
 *   Return the total size of an MPI value in bytes
 *
 * Input Parameters:
 *   X       - The MPI context to query
 *
 * Returned Value:
 *   The least number of bytes capable of storing the absolute value.
 *
 ****************************************************************************/

size_t esp32c3_mpi_size(const struct esp32c3_mpi_s *X)
{
  return ((esp32c3_mpi_bitlen(X) + 7) >> 3);
}

/****************************************************************************
 * Name: esp32c3_mpi_read_string
 *
 * Description:
 *   Import from an ASCII string
 *
 * Input Parameters:
 *   X       - The destination MPI
 *   radix   - The numeric base of the input string
 *   s       - Null-terminated string buffer
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_read_string(struct esp32c3_mpi_s *X,
                            int radix, const char *s)
{
  int ret;
  size_t i;
  size_t j;
  size_t slen;
  size_t n;
  uint32_t d;
  struct esp32c3_mpi_s T;
  DEBUGASSERT(X != NULL);
  DEBUGASSERT(s != NULL);

  if (radix < 2 || radix > 16)
    {
      return ESP32C3_ERR_MPI_BAD_INPUT_DATA;
    }

  esp32c3_mpi_init(&T);

  slen = strlen(s);

  if (radix == 16)
    {
      if (slen > MPI_SIZE_T_MAX >> 2)
        {
          return ESP32C3_ERR_MPI_BAD_INPUT_DATA;
        }

      n = BITS_TO_LIMBS(slen << 2);

      ESP32C3_MPI_CHK(esp32c3_mpi_grow(X, n), cleanup);
      ESP32C3_MPI_CHK(esp32c3_mpi_lset(X, 0), cleanup);

      for (i = slen, j = 0; i > 0; i--, j++)
        {
          if (i == 1 && s[i - 1] == '-')
            {
              X->s = -1;
              break;
            }

          ESP32C3_MPI_CHK(mpi_get_digit(&d, radix, s[i - 1]), cleanup);
          X->p[j / (2 * CIL)] |= d << ((j % (2 * CIL)) << 2);
        }
    }
  else
    {
      ESP32C3_MPI_CHK(esp32c3_mpi_lset(X, 0), cleanup);

      for (i = 0; i < slen; i++)
        {
          if (i == 0 && s[i] == '-')
            {
              X->s = -1;
              continue;
            }

          ESP32C3_MPI_CHK(mpi_get_digit(&d, radix, s[i]), cleanup);
          ESP32C3_MPI_CHK(esp32c3_mpi_mul_int(&T, X, radix), cleanup);

          if (X->s == 1)
            {
              ESP32C3_MPI_CHK(esp32c3_mpi_add_int(X, &T, d), cleanup);
            }
          else
            {
              ESP32C3_MPI_CHK(esp32c3_mpi_sub_int(X, &T, d), cleanup);
            }
        }
    }

cleanup:

  esp32c3_mpi_free(&T);

  return ret;
}

/****************************************************************************
 * Name: esp32c3_mpi_write_string
 *
 * Description:
 *   Export an MPI to an ASCII string
 *
 * Input Parameters:
 *   X       - The source MPI
 *   radix   - The numeric base of the output string
 *   buf     - The buffer to write the string to
 *   buflen  - The available size in Bytes of buf
 *   olen    - The address at which to store the length of the string written
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_write_string(const struct esp32c3_mpi_s *X, int radix,
                             char *buf, size_t buflen, size_t *olen)
{
  int ret = 0;
  size_t n;
  char *p;
  struct esp32c3_mpi_s T;
  DEBUGASSERT(X  != NULL);
  DEBUGASSERT(olen != NULL);
  DEBUGASSERT(buflen == 0 || buf != NULL);

  if (radix < 2 || radix > 16)
    {
      return ESP32C3_ERR_MPI_BAD_INPUT_DATA;
    }

  /* Number of bits necessary to present `n`. */

  n = esp32c3_mpi_bitlen(X);
  if (radix >=  4)
    {
      /* Number of 4-adic digits necessary to present
       * `n`. If radix > 4, this might be a strict
       * overapproximation of the number of
       * radix-adic digits needed to present `n`.
       */

      n >>= 1;
    }

  if (radix >= 16)
    {
      /* Number of hexadecimal digits necessary to
       * present `n`.
       */

      n >>= 1;
    }

  /* Terminating null byte */

  n += 1;

  /* Compensate for the divisions above, which round down `n`
   * in case it's not even.
   */

  n += 1;

  /* Potential '-'-sign. */

  n += 1;

  /* Make n even to have enough space for hexadecimal writing,
   * which always uses an even number of hex-digits.
   */

  n += (n & 1);

  if (buflen < n)
    {
      *olen = n;
      return ESP32C3_ERR_MPI_BUFFER_TOO_SMALL;
    }

  p = buf;
  esp32c3_mpi_init(&T);

  if (X->s == -1)
    {
      *p++ = '-';
      buflen--;
    }

  if (radix == 16)
    {
      int c;
      size_t i, j, k;

      for (i = X->n, k = 0; i > 0; i--)
        {
          for (j = CIL; j > 0; j--)
            {
              c = (X->p[i - 1] >> ((j - 1) << 3)) & 0xff;

              if (c == 0 && k == 0 && (i + j) != 2)
                {
                  continue;
                }

              *(p++) = "0123456789ABCDEF" [c / 16];
              *(p++) = "0123456789ABCDEF" [c % 16];
              k = 1;
            }
        }
    }
  else
    {
      ESP32C3_MPI_CHK(esp32c3_mpi_copy(&T, X), cleanup);

      if (T.s == -1)
        {
          T.s = 1;
        }

      ESP32C3_MPI_CHK(mpi_write_hlp(&T, radix, &p, buflen), cleanup);
    }

  *p++ = '\0';
  *olen = p - buf;

cleanup:

  esp32c3_mpi_free(&T);

  return ret;
}

/****************************************************************************
 * Name: esp32c3_mpi_read_binary
 *
 * Description:
 *   Import an MPI from unsigned big endian binary data
 *
 * Input Parameters:
 *   X       - The destination MPI
 *   buf     - The input buffer
 *   buflen  - The length of the input buffer
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_read_binary(struct esp32c3_mpi_s *X,
                            const unsigned char *buf,
                            size_t buflen)
{
  int ret;
  size_t const limbs  = CHARS_TO_LIMBS(buflen);
  size_t const overhead = (limbs * CIL) - buflen;
  unsigned char *XP;

  DEBUGASSERT(X != NULL);
  DEBUGASSERT(buflen == 0 || buf != NULL);

  /* Ensure that target MPI has exactly the necessary number of limbs */

  if (X->n != limbs)
    {
      esp32c3_mpi_free(X);
      esp32c3_mpi_init(X);
      ESP32C3_MPI_CHK(esp32c3_mpi_grow(X, limbs), cleanup);
    }

  ESP32C3_MPI_CHK(esp32c3_mpi_lset(X, 0), cleanup);

  /* Avoid calling `memcpy` with NULL source argument,
   * even if buflen is 0.
   */

  if (buf != NULL)
    {
      XP = (unsigned char *) X->p;
      memcpy(XP + overhead, buf, buflen);

      mpi_bigendian_to_host(X->p, limbs);
    }

cleanup:

  return ret;
}

/****************************************************************************
 * Name: esp32c3_mpi_write_binary
 *
 * Description:
 *   Export X into unsigned binary data, big endian
 *
 * Input Parameters:
 *   X       - The source MPI
 *   buf     - The output buffer
 *   buflen  - The length of the output buffer
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_write_binary(const struct esp32c3_mpi_s *X,
                unsigned char *buf, size_t buflen)
{
  size_t stored_bytes;
  size_t bytes_to_copy;
  unsigned char *p;
  size_t i;

  DEBUGASSERT(X != NULL);
  DEBUGASSERT(buflen == 0 || buf != NULL);

  stored_bytes = X->n * CIL;

  if (stored_bytes < buflen)
    {
      /* There is enough space in the output buffer. Write initial
       * null bytes and record the position at which to start
       * writing the significant bytes. In this case, the execution
       * trace of this function does not depend on the value of the
       * number.
       */

      bytes_to_copy = stored_bytes;
      p = buf + buflen - stored_bytes;
      memset(buf, 0, buflen - stored_bytes);
    }
  else
    {
      /* The output buffer is smaller than the allocated size of X.
       * However X may fit if its leading bytes are zero.
       */

      bytes_to_copy = buflen;
      p = buf;
      for (i = bytes_to_copy; i < stored_bytes; i++)
        {
          if (GET_BYTE(X, i) != 0)
            {
              return ESP32C3_ERR_MPI_BUFFER_TOO_SMALL;
            }
        }
    }

  for (i = 0; i < bytes_to_copy; i++)
    {
      p[bytes_to_copy - i - 1] = GET_BYTE(X, i);
    }

  return OK;
}

/****************************************************************************
 * Name: esp32c3_mpi_shift_l
 *
 * Description:
 *   Perform a left-shift on an MPI: X <<= count
 *
 * Input Parameters:
 *   X       - The MPI to shift
 *   count   - The number of bits to shift by
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_shift_l(struct esp32c3_mpi_s *X, size_t count)
{
  int ret;
  size_t i, v0, t1;
  uint32_t r0 = 0, r1;
  DEBUGASSERT(X != NULL);

  v0 = count / (BIL);
  t1 = count & (BIL - 1);

  i = esp32c3_mpi_bitlen(X) + count;

  if (X->n * BIL < i)
    {
      ESP32C3_MPI_CHK(esp32c3_mpi_grow(X, BITS_TO_LIMBS(i)), cleanup);
    }

  ret = 0;

  /* shift by count / limb_size */

  if (v0 > 0)
    {
      for (i = X->n; i > v0; i--)
        X->p[i - 1] = X->p[i - v0 - 1];

      for (; i > 0; i--)
        X->p[i - 1] = 0;
    }

  /* shift by count % limb_size
   */

  if (t1 > 0)
    {
      for (i = v0; i < X->n; i++)
        {
          r1 = X->p[i] >> (BIL - t1);
          X->p[i] <<= t1;
          X->p[i] |= r0;
          r0 = r1;
        }
    }

cleanup:

  return ret;
}

/****************************************************************************
 * Name: esp32c3_mpi_shift_r
 *
 * Description:
 *   Perform a right-shift on an MPI: X >>= count
 *
 * Input Parameters:
 *   X       - The MPI to shift
 *   count   - The number of bits to shift by
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_shift_r(struct esp32c3_mpi_s *X, size_t count)
{
  size_t i, v0, v1;
  uint32_t r0 = 0, r1;
  DEBUGASSERT(X != NULL);

  v0 = count /  BIL;
  v1 = count & (BIL - 1);

  if (v0 > X->n || (v0 == X->n && v1 > 0))
    {
      return esp32c3_mpi_lset(X, 0);
    }

  /* shift by count / limb_size
   */

  if (v0 > 0)
    {
      for (i = 0; i < X->n - v0; i++)
        {
          X->p[i] = X->p[i + v0];
        }

      for (; i < X->n; i++)
        {
          X->p[i] = 0;
        }
    }

  /* shift by count % limb_size
   */

  if (v1 > 0)
    {
      for (i = X->n; i > 0; i--)
        {
          r1 = X->p[i - 1] << (BIL - v1);
          X->p[i - 1] >>= v1;
          X->p[i - 1] |= r0;
          r0 = r1;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: esp32c3_mpi_cmp_abs
 *
 * Description:
 *   Compare the absolute values of two MPIs
 *
 * Input Parameters:
 *   X       - The left-hand MPI
 *   Y       - The right-hand MPI
 *
 * Returned Value:
 *   1 if \p `|X|` is greater than \p `|Y|`.
 *   -1 if \p `|X|` is lesser than \p `|Y|`.
 *   0 if \p `|X|` is equal to \p `|Y|`.
 *
 ****************************************************************************/

int esp32c3_mpi_cmp_abs(const struct esp32c3_mpi_s *X,
                        const struct esp32c3_mpi_s *Y)
{
  size_t i, j;
  DEBUGASSERT(X != NULL);
  DEBUGASSERT(Y != NULL);

  for (i = X->n; i > 0; i--)
    {
      if (X->p[i - 1] != 0)
        {
          break;
        }
    }

  for (j = Y->n; j > 0; j--)
    {
      if (Y->p[j - 1] != 0)
        {
          break;
        }
    }

  if (i == 0 && j == 0)
    {
      return OK;
    }

  if (i > j)
    {
      return (1);
    }

  if (j > i)
    {
      return (-1);
    }

  for (; i > 0; i--)
    {
      if (X->p[i - 1] > Y->p[i - 1])
        {
          return (1);
        }

      if (X->p[i - 1] < Y->p[i - 1])
        {
          return (-1);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: esp32c3_mpi_cmp_mpi
 *
 * Description:
 *   Compare two MPIs.
 *
 * Input Parameters:
 *   X       - The left-hand MPI
 *   Y       - The right-hand MPI
 *
 * Returned Value:
 *   1 if \p `X` is greater than \p `Y`.
 *   -1 if \p `X` is lesser than \p `Y`.
 *   0 if \p `X` is equal to \p `Y`.
 *
 ****************************************************************************/

int esp32c3_mpi_cmp_mpi(const struct esp32c3_mpi_s *X,
                        const struct esp32c3_mpi_s *Y)
{
  size_t i, j;
  DEBUGASSERT(X != NULL);
  DEBUGASSERT(Y != NULL);

  for (i = X->n; i > 0; i--)
    {
      if (X->p[i - 1] != 0)
        {
          break;
        }
    }

  for (j = Y->n; j > 0; j--)
    {
      if (Y->p[j - 1] != 0)
        {
          break;
        }
    }

  if (i == 0 && j == 0)
    {
      return OK;
    }

  if (i > j)
    {
      return (X->s);
    }

  if (j > i)
    {
      return (-Y->s);
    }

  if (X->s > 0 && Y->s < 0)
    {
      return (1);
    }

  if (Y->s > 0 && X->s < 0)
    {
      return (-1);
    }

  for (; i > 0; i--)
    {
      if (X->p[i - 1] > Y->p[i - 1])
        {
          return (X->s);
        }

      if (X->p[i - 1] < Y->p[i - 1])
        {
          return (-X->s);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: esp32c3_mpi_lt_mpi_ct
 *
 * Description:
 *   Check if an MPI is less than the other in constant time
 *
 * Input Parameters:
 *   X       - The left-hand MPI
 *   Y       - The right-hand MPI
 *   ret     - The result of the comparison:
 *             1 if \p X is less than \p Y.
 *             0 if \p X is greater than or equal to \p Y.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_lt_mpi_ct(const struct esp32c3_mpi_s *X,
                          const struct esp32c3_mpi_s *Y,
                          unsigned *ret)
{
  size_t i;

  /* The value of any of these variables is either 0 or 1 at all times. */

  unsigned cond, done, x_is_negative, y_is_negative;

  DEBUGASSERT(X != NULL);
  DEBUGASSERT(Y != NULL);
  DEBUGASSERT(ret != NULL);

  if (X->n != Y->n)
    {
      return ESP32C3_ERR_MPI_BAD_INPUT_DATA;
    }

  /* Set sign_N to 1 if N >= 0, 0 if N < 0.
   * We know that N->s == 1 if N >= 0 and N->s == -1 if N < 0.
   */

  x_is_negative = (X->s & 2) >> 1;
  y_is_negative = (Y->s & 2) >> 1;

  /* If the signs are different, then the positive operand is the bigger.
   * That is if X is negative (x_is_negative == 1), then X < Y is true and it
   * is false if X is positive (x_is_negative == 0).
   */

  cond = (x_is_negative ^ y_is_negative);
  *ret = cond & x_is_negative;

  /* This is a constant-time function. We might have the result, but we still
   * need to go through the loop. Record if we have the result already.
   */

  done = cond;

  for (i = X->n; i > 0; i--)
    {
      /* If Y->p[i - 1] < X->p[i - 1] then X < Y is true if and only if both
       * X and Y are negative.
       *
       * Again even if we can make a decision, we just mark the result and
       * the fact that we are done and continue looping.
       */

      cond = ct_lt_mpi_uint(Y->p[i - 1], X->p[i - 1]);
      *ret |= cond & (1 - done) & x_is_negative;
      done |= cond;

      /* If X->p[i - 1] < Y->p[i - 1] then X < Y is true if and only if both
       * X and Y are positive.
       *
       * Again even if we can make a decision, we just mark the result and
       * the fact that we are done and continue looping.
       */

      cond = ct_lt_mpi_uint(X->p[i - 1], Y->p[i - 1]);
      *ret |= cond & (1 - done) & (1 - x_is_negative);
      done |= cond;
    }

  return OK;
}

/****************************************************************************
 * Name: esp32c3_mpi_cmp_int
 *
 * Description:
 *   Compare an MPI with an integer
 *
 * Input Parameters:
 *   X       - The left-hand MPI
 *   z       - The integer value to compare \p X to
 *
 * Returned Value:
 *   \c 1 if \p X is greater than \p z.
 *   \c -1 if \p X is lesser than \p z.
 *   \c 0 if \p X is equal to \p z.
 *
 ****************************************************************************/

int esp32c3_mpi_cmp_int(const struct esp32c3_mpi_s *X, int32_t z)
{
  struct esp32c3_mpi_s Y;
  uint32_t p[1];
  DEBUGASSERT(X != NULL);

  *p  = (z < 0) ? -z : z;
  Y.s = (z < 0) ? -1 : 1;
  Y.n = 1;
  Y.p = p;

  return (esp32c3_mpi_cmp_mpi(X, &Y));
}

/****************************************************************************
 * Name: esp32c3_mpi_add_abs
 *
 * Description:
 *   Perform an unsigned addition of MPIs: X = |A| + |B|
 *
 * Input Parameters:
 *   X       - The left-hand MPI
 *   z       - The integer value to compare \p X to.
 *
 * Returned Value:
 *   \c 1 if \p X is greater than \p z.
 *   \c -1 if \p X is lesser than \p z.
 *   \c 0 if \p X is equal to \p z.
 *
 ****************************************************************************/

int esp32c3_mpi_add_abs(struct esp32c3_mpi_s *X,
                        const struct esp32c3_mpi_s *A,
                        const struct esp32c3_mpi_s *B)
{
  int ret;
  size_t i, j;
  uint32_t *o, *p, c, tmp;
  DEBUGASSERT(X != NULL);
  DEBUGASSERT(A != NULL);
  DEBUGASSERT(B != NULL);

  if (X == B)
    {
      const struct esp32c3_mpi_s *T = A; A = X; B = T;
    }

  if (X != A)
    {
      ESP32C3_MPI_CHK(esp32c3_mpi_copy(X, A), cleanup);
    }

  /* X should always be positive as a result of unsigned additions.
   */

  X->s = 1;

  for (j = B->n; j > 0; j--)
    {
      if (B->p[j - 1] != 0)
        {
          break;
        }
    }

  ESP32C3_MPI_CHK(esp32c3_mpi_grow(X, j), cleanup);

  o = B->p; p = X->p; c = 0;

  /* tmp is used because it might happen that p == o
   */

  for (i = 0; i < j; i++, o++, p++)
    {
      tmp = *o;
      *p += c; c = (*p < c);
      *p += tmp; c += (*p < tmp);
    }

  while (c != 0)
    {
      if (i >= X->n)
        {
          ESP32C3_MPI_CHK(esp32c3_mpi_grow(X, i + 1), cleanup);
          p = X->p + i;
        }

      *p += c; c = (*p < c); i++; p++;
    }

cleanup:

  return ret;
}

/****************************************************************************
 * Name: esp32c3_mpi_sub_abs
 *
 * Description:
 *   Perform an unsigned subtraction of MPIs: X = |A| - |B|
 *
 * Input Parameters:
 *   X       - The destination MPI
 *   A       - The minuend
 *   B       - The subtrahend
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_sub_abs(struct esp32c3_mpi_s *X,
                        const struct esp32c3_mpi_s *A,
                        const struct esp32c3_mpi_s *B)
{
  struct esp32c3_mpi_s TB;
  int ret;
  size_t n;
  uint32_t carry;
  DEBUGASSERT(X != NULL);
  DEBUGASSERT(A != NULL);
  DEBUGASSERT(B != NULL);

  esp32c3_mpi_init(&TB);

  if (X == B)
    {
      ESP32C3_MPI_CHK(esp32c3_mpi_copy(&TB, B), cleanup);
      B = &TB;
    }

  if (X != A)
    {
      ESP32C3_MPI_CHK(esp32c3_mpi_copy(X, A), cleanup);
    }

  /* X should always be positive as a result of unsigned subtractions. */

  X->s = 1;

  ret = 0;

  for (n = B->n; n > 0; n--)
    {
      if (B->p[n - 1] != 0)
        {
          break;
        }
    }

  carry = mpi_sub_hlp(n, X->p, B->p);
  if (carry != 0)
    {
      /* Propagate the carry to the first nonzero limb of X. */

      for (; n < X->n && X->p[n] == 0; n++)
        {
          --X->p[n];
        }

      /* If we ran out of space for the carry, it means that the result
       * is negative.
       */

      if (n == X->n)
        {
          ret = ESP32C3_ERR_MPI_NEGATIVE_VALUE;
          goto cleanup;
        }

      --X->p[n];
    }

cleanup:

  esp32c3_mpi_free(&TB);

  return ret;
}

/****************************************************************************
 * Name: esp32c3_mpi_add_mpi
 *
 * Description:
 *   Perform a signed addition of MPIs: X = A + B
 *
 * Input Parameters:
 *   X       - The destination MPI
 *   A       - The first summand
 *   B       - The second summand
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_add_mpi(struct esp32c3_mpi_s *X,
                        const struct esp32c3_mpi_s *A,
                        const struct esp32c3_mpi_s *B)
{
  int ret, s;
  DEBUGASSERT(X != NULL);
  DEBUGASSERT(A != NULL);
  DEBUGASSERT(B != NULL);

  s = A->s;
  if (A->s * B->s < 0)
    {
      if (esp32c3_mpi_cmp_abs(A, B) >= 0)
        {
          ESP32C3_MPI_CHK(esp32c3_mpi_sub_abs(X, A, B), cleanup);
          X->s =  s;
        }
      else
        {
          ESP32C3_MPI_CHK(esp32c3_mpi_sub_abs(X, B, A), cleanup);
          X->s = -s;
        }
    }
  else
    {
      ESP32C3_MPI_CHK(esp32c3_mpi_add_abs(X, A, B), cleanup);
      X->s = s;
    }

cleanup:

  return ret;
}

/****************************************************************************
 * Name: esp32c3_mpi_sub_mpi
 *
 * Description:
 *   Perform a signed subtraction of MPIs: X = A - B
 *
 * Input Parameters:
 *   X       - The destination MPI
 *   A       - The minuend
 *   B       - The subtrahend
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_sub_mpi(struct esp32c3_mpi_s *X,
                        const struct esp32c3_mpi_s *A,
                        const struct esp32c3_mpi_s *B)
{
  int ret, s;
  DEBUGASSERT(X != NULL);
  DEBUGASSERT(A != NULL);
  DEBUGASSERT(B != NULL);

  s = A->s;
  if (A->s * B->s > 0)
    {
      if (esp32c3_mpi_cmp_abs(A, B) >= 0)
        {
          ESP32C3_MPI_CHK(esp32c3_mpi_sub_abs(X, A, B), cleanup);
          X->s =  s;
        }
      else
        {
          ESP32C3_MPI_CHK(esp32c3_mpi_sub_abs(X, B, A), cleanup);
          X->s = -s;
        }
    }
  else
    {
      ESP32C3_MPI_CHK(esp32c3_mpi_add_abs(X, A, B), cleanup);
      X->s = s;
    }

cleanup:

  return ret;
}

/****************************************************************************
 * Name: esp32c3_mpi_add_int
 *
 * Description:
 *   Perform a signed addition of an MPI and an integer: X = A + b
 *
 * Input Parameters:
 *   X       - The destination MPI
 *   A       - The first summand
 *   b       - The second summand
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_add_int(struct esp32c3_mpi_s *X,
                        const struct esp32c3_mpi_s *A,
                        int32_t b)
{
  struct esp32c3_mpi_s _B;
  uint32_t p[1];
  DEBUGASSERT(X != NULL);
  DEBUGASSERT(A != NULL);

  p[0] = (b < 0) ? -b : b;
  _B.s = (b < 0) ? -1 : 1;
  _B.n = 1;
  _B.p = p;

  return (esp32c3_mpi_add_mpi(X, A, &_B));
}

/****************************************************************************
 * Name: esp32c3_mpi_sub_int
 *
 * Description:
 *   Perform a signed subtraction of an MPI and an integer: X = A - b
 *
 * Input Parameters:
 *   X       - The destination MPI
 *   A       - The minuend
 *   b       - The subtrahend
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_sub_int(struct esp32c3_mpi_s *X,
                        const struct esp32c3_mpi_s *A,
                        int32_t b)
{
  struct esp32c3_mpi_s _B;
  uint32_t p[1];
  DEBUGASSERT(X != NULL);
  DEBUGASSERT(A != NULL);

  p[0] = (b < 0) ? -b : b;
  _B.s = (b < 0) ? -1 : 1;
  _B.n = 1;
  _B.p = p;

  return (esp32c3_mpi_sub_mpi(X, A, &_B));
}

/****************************************************************************
 * Name: esp32c3_mpi_mul_mpi
 *
 * Description:
 *   Perform a multiplication of two MPIs: Z = X * Y
 *
 * Input Parameters:
 *   Z      - The destination MPI
 *   X      - The first factor
 *   Y      - The second factor
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_mul_mpi(struct esp32c3_mpi_s *Z,
                        const struct esp32c3_mpi_s *X,
                        const struct esp32c3_mpi_s *Y)
{
  int ret = 0;
  size_t x_bits = esp32c3_mpi_bitlen(X);
  size_t y_bits = esp32c3_mpi_bitlen(Y);
  size_t x_words = bits_to_words(x_bits);
  size_t y_words = bits_to_words(y_bits);
  size_t z_words = bits_to_words(x_bits + y_bits);
  size_t hw_words = MAX(x_words, y_words);

  /* Short-circuit eval if either argument is 0 or 1.

   * This is needed as the mpi modular division
   * argument will sometimes call in here when one
   * argument is too large for the hardware unit, but the other
   * argument is zero or one.
   */

  if (x_bits == 0 || y_bits == 0)
    {
      esp32c3_mpi_lset(Z, 0);
      return 0;
    }

  if (x_bits == 1)
    {
      ret = esp32c3_mpi_copy(Z, Y);
      Z->s *= X->s;
      return ret;
    }

  if (y_bits == 1)
    {
      ret = esp32c3_mpi_copy(Z, X);
      Z->s *= Y->s;
      return ret;
    }

  /* Grow Z to result size early, avoid interim allocations */

  ESP32C3_MPI_CHK(esp32c3_mpi_grow(Z, z_words), cleanup);

  /* If factor is over 2048 bits, we can't use the standard
   * hardware multiplier
   */

  if (hw_words * 32 > SOC_RSA_MAX_BIT_LEN / 2)
    {
      if (z_words * 32 <= SOC_RSA_MAX_BIT_LEN)
        {
          return mpi_mult_mpi_failover_mod_mult(Z, X, Y, z_words);
        }
      else
        {
          /* Still too long for the hardware unit... */

          if (y_words > x_words)
            {
              return mpi_mult_mpi_overlong(Z, X, Y, y_words, z_words);
            }
          else
            {
              return mpi_mult_mpi_overlong(Z, Y, X, x_words, z_words);
            }
        }
    }

  /* Otherwise, we can use the (faster) multiply hardware unit */

  esp32c3_mpi_enable_hardware_hw_op();

  esp32c3_mpi_mul_mpi_hw_op(X, Y, hw_words);
  esp32c3_mpi_read_result_hw_op(Z, z_words);

  esp32c3_mpi_disable_hardware_hw_op();

  Z->s = X->s * Y->s;

cleanup:
  return ret;
}

/****************************************************************************
 * Name: esp32c3_mpi_mul_int
 *
 * Description:
 *   Perform a multiplication of an MPI with an unsigned integer: X = A * b
 *
 * Input Parameters:
 *   X       - The destination MPI
 *   A       - The first factor
 *   b       - The second factor.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_mul_int(struct esp32c3_mpi_s *X,
                        const struct esp32c3_mpi_s *A,
                        uint32_t b)
{
  struct esp32c3_mpi_s _B;
  uint32_t p[1];
  DEBUGASSERT(X != NULL);
  DEBUGASSERT(A != NULL);

  _B.s = 1;
  _B.n = 1;
  _B.p = p;
  p[0] = b;

  return (esp32c3_mpi_mul_mpi(X, A, &_B));
}

/****************************************************************************
 * Name: esp32c3_mpi_div_mpi
 *
 * Description:
 *   Perform a division with remainder of two MPIs: A = Q * B + R
 *
 * Input Parameters:
 *   Q        - The destination MPI for the quotient
 *   R        - The destination MPI for the remainder value
 *   A        - The dividend
 *   B        - The divisor
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_div_mpi(struct esp32c3_mpi_s *Q,
                        struct esp32c3_mpi_s *R,
                        const struct esp32c3_mpi_s *A,
                        const struct esp32c3_mpi_s *B)
{
  int ret;
  size_t i, n, t, k;
  struct esp32c3_mpi_s X, Y, Z, T1, T2;
  DEBUGASSERT(A != NULL);
  DEBUGASSERT(B != NULL);

  if (esp32c3_mpi_cmp_int(B, 0) == 0)
    {
      return ESP32C3_ERR_MPI_DIVISION_BY_ZERO;
    }

  esp32c3_mpi_init(&X);
  esp32c3_mpi_init(&Y);
  esp32c3_mpi_init(&Z);
  esp32c3_mpi_init(&T1);
  esp32c3_mpi_init(&T2);

  if (esp32c3_mpi_cmp_abs(A, B) < 0)
    {
      if (Q != NULL)
        {
          ESP32C3_MPI_CHK(esp32c3_mpi_lset(Q, 0), cleanup);
        }

      if (R != NULL)
        {
          ESP32C3_MPI_CHK(esp32c3_mpi_copy(R, A), cleanup);
        }

      return OK;
    }

  ESP32C3_MPI_CHK(esp32c3_mpi_copy(&X, A), cleanup);
  ESP32C3_MPI_CHK(esp32c3_mpi_copy(&Y, B), cleanup);
  X.s = Y.s = 1;

  ESP32C3_MPI_CHK(esp32c3_mpi_grow(&Z, A->n + 2), cleanup);
  ESP32C3_MPI_CHK(esp32c3_mpi_lset(&Z,  0), cleanup);
  ESP32C3_MPI_CHK(esp32c3_mpi_grow(&T1, 2), cleanup);
  ESP32C3_MPI_CHK(esp32c3_mpi_grow(&T2, 3), cleanup);

  k = esp32c3_mpi_bitlen(&Y) % BIL;
  if (k < BIL - 1)
    {
      k = BIL - 1 - k;
      ESP32C3_MPI_CHK(esp32c3_mpi_shift_l(&X, k), cleanup);
      ESP32C3_MPI_CHK(esp32c3_mpi_shift_l(&Y, k), cleanup);
    }
  else
    {
      k = 0;
    }

  n = X.n - 1;
  t = Y.n - 1;
  ESP32C3_MPI_CHK(esp32c3_mpi_shift_l(&Y, BIL * (n - t)), cleanup);

  while (esp32c3_mpi_cmp_mpi(&X, &Y) >= 0)
    {
      Z.p[n - t]++;
      ESP32C3_MPI_CHK(esp32c3_mpi_sub_mpi(&X, &X, &Y), cleanup);
    }

  ESP32C3_MPI_CHK(esp32c3_mpi_shift_r(&Y, BIL * (n - t)), cleanup);

  for (i = n; i > t ; i--)
    {
      if (X.p[i] >= Y.p[t])
        {
          Z.p[i - t - 1] = ~0;
        }
      else
        {
          Z.p[i - t - 1] = esp32c3_bignum_int_div_int(X.p[i], X.p[i - 1],
                                                      Y.p[t], NULL);
        }

      Z.p[i - t - 1]++;
      do
        {
          Z.p[i - t - 1]--;

          ESP32C3_MPI_CHK(esp32c3_mpi_lset(&T1, 0), cleanup);
          T1.p[0] = (t < 1) ? 0 : Y.p[t - 1];
          T1.p[1] = Y.p[t];
          ESP32C3_MPI_CHK(esp32c3_mpi_mul_int(&T1, &T1, Z.p[i - t - 1]),
                          cleanup);

          ESP32C3_MPI_CHK(esp32c3_mpi_lset(&T2, 0), cleanup);
          T2.p[0] = (i < 2) ? 0 : X.p[i - 2];
          T2.p[1] = (i < 1) ? 0 : X.p[i - 1];
          T2.p[2] = X.p[i];
        }
      while (esp32c3_mpi_cmp_mpi(&T1, &T2) > 0);

      ESP32C3_MPI_CHK(esp32c3_mpi_mul_int(&T1, &Y, Z.p[i - t - 1]), cleanup);
      ESP32C3_MPI_CHK(esp32c3_mpi_shift_l(&T1,  BIL * (i - t - 1)), cleanup);
      ESP32C3_MPI_CHK(esp32c3_mpi_sub_mpi(&X, &X, &T1), cleanup);

      if (esp32c3_mpi_cmp_int(&X, 0) < 0)
        {
          ESP32C3_MPI_CHK(esp32c3_mpi_copy(&T1, &Y), cleanup);
          ESP32C3_MPI_CHK(esp32c3_mpi_shift_l(&T1, BIL * (i - t - 1)),
                          cleanup);
          ESP32C3_MPI_CHK(esp32c3_mpi_add_mpi(&X, &X, &T1), cleanup);
          Z.p[i - t - 1]--;
        }
    }

  if (Q != NULL)
    {
      ESP32C3_MPI_CHK(esp32c3_mpi_copy(Q, &Z), cleanup);
      Q->s = A->s * B->s;
    }

  if (R != NULL)
    {
      ESP32C3_MPI_CHK(esp32c3_mpi_shift_r(&X, k), cleanup);
      X.s = A->s;
      ESP32C3_MPI_CHK(esp32c3_mpi_copy(R, &X), cleanup);

      if (esp32c3_mpi_cmp_int(R, 0) == 0)
        {
          R->s = 1;
        }
    }

cleanup:

  esp32c3_mpi_free(&X); esp32c3_mpi_free(&Y); esp32c3_mpi_free(&Z);
  esp32c3_mpi_free(&T1); esp32c3_mpi_free(&T2);

  return ret;
}

/****************************************************************************
 * Name: esp32c3_mpi_div_int
 *
 * Description:
 *   Perform a division with remainder of an MPI by an integer: A = Q * b + R
 *
 * Input Parameters:
 *   Q        - The destination MPI for the quotient
 *   R        - The destination MPI for the remainder value
 *   A        - The dividend
 *   B        - The divisor
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_div_int(struct esp32c3_mpi_s *Q,
                        struct esp32c3_mpi_s *R,
                        const struct esp32c3_mpi_s *A,
                        int32_t b)
{
  struct esp32c3_mpi_s _B;
  uint32_t p[1];
  DEBUGASSERT(A != NULL);

  p[0] = (b < 0) ? -b : b;
  _B.s = (b < 0) ? -1 : 1;
  _B.n = 1;
  _B.p = p;

  return (esp32c3_mpi_div_mpi(Q, R, A, &_B));
}

/****************************************************************************
 * Name: esp32c3_mpi_mod_mpi
 *
 * Description:
 *   erform a modular reduction. R = A mod B
 *
 * Input Parameters:
 *   R       - The destination MPI for the residue value
 *   A       - The MPI to compute the residue of
 *   B       - The base of the modular reduction
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_mod_mpi(struct esp32c3_mpi_s *R,
                        const struct esp32c3_mpi_s *A,
                        const struct esp32c3_mpi_s *B)
{
  int ret;
  DEBUGASSERT(R != NULL);
  DEBUGASSERT(A != NULL);
  DEBUGASSERT(B != NULL);

  if (esp32c3_mpi_cmp_int(B, 0) < 0)
    {
      return ESP32C3_ERR_MPI_NEGATIVE_VALUE;
    }

  ESP32C3_MPI_CHK(esp32c3_mpi_div_mpi(NULL, R, A, B), cleanup);

  while (esp32c3_mpi_cmp_int(R, 0) < 0)
    {
      ESP32C3_MPI_CHK(esp32c3_mpi_add_mpi(R, R, B), cleanup);
    }

  while (esp32c3_mpi_cmp_mpi(R, B) >= 0)
    {
      ESP32C3_MPI_CHK(esp32c3_mpi_sub_mpi(R, R, B), cleanup);
    }

cleanup:

  return ret;
}

/****************************************************************************
 * Name: esp32c3_mpi_mod_int
 *
 * Description:
 *   Perform a modular reduction with respect to an integer: r = A mod b
 *
 * Input Parameters:
 *   r       - The address at which to store the residue
 *   A       - The MPI to compute the residue of
 *   b       - The integer base of the modular reduction
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_mod_int(uint32_t *r,
                        const struct esp32c3_mpi_s *A, int32_t b)
{
  size_t i;
  uint32_t x, y, z;
  DEBUGASSERT(r != NULL);
  DEBUGASSERT(A != NULL);

  if (b == 0)
    {
      return ESP32C3_ERR_MPI_DIVISION_BY_ZERO;
    }

  if (b < 0)
    {
      return ESP32C3_ERR_MPI_NEGATIVE_VALUE;
    }

  /* handle trivial cases */

  if (b == 1)
    {
      *r = 0;
      return OK;
    }

  if (b == 2)
    {
      *r = A->p[0] & 1;
      return OK;
    }

  /* general case */

  for (i = A->n, y = 0; i > 0; i--)
    {
      x  = A->p[i - 1];
      y  = (y << BIH) | (x >> BIH);
      z  = y / b;
      y -= z * b;

      x <<= BIH;
      y  = (y << BIH) | (x >> BIH);
      z  = y / b;
      y -= z * b;
    }

  /* If A is negative, then the current y represents a negative value.
   * Flipping it to the positive side.
   */

  if (A->s < 0 && y != 0)
    {
      y = b - y;
    }

  *r = y;

  return OK;
}

/****************************************************************************
 * Name: esp32c3_mpi_exp_mod
 *
 * Description:
 *   Perform a sliding-window exponentiation: X = A^E mod N
 *
 * Input Parameters:
 *   X       - The destination MPI
 *   A       - The base of the exponentiation
 *   E       - The exponent MPI
 *   N       - The base for the modular reduction
 *   _RR     - A helper MPI depending solely on \p N which can be used to
 *             speed-up multiple modular exponentiations for the same value
 *             of \p N.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_exp_mod(struct esp32c3_mpi_s *X,
                        const struct esp32c3_mpi_s *A,
                        const struct esp32c3_mpi_s *E,
                        const struct esp32c3_mpi_s *N,
                        struct esp32c3_mpi_s *_RR)
{
  int ret;
  size_t wbits, wsize, one = 1;
  size_t i, j, nblimbs;
  size_t bufsize, nbits;
  uint32_t ei, mm, state;
  struct esp32c3_mpi_s RR, T, W[1 << ESP32C3_MPI_WINDOW_SIZE], apos;
  int neg;

  DEBUGASSERT(X != NULL);
  DEBUGASSERT(A != NULL);
  DEBUGASSERT(E != NULL);
  DEBUGASSERT(N != NULL);

  if (esp32c3_mpi_cmp_int(N, 0) <= 0 || (N->p[0] & 1) == 0)
    {
      return ESP32C3_ERR_MPI_BAD_INPUT_DATA;
    }

  if (esp32c3_mpi_cmp_int(E, 0) < 0)
    {
      return ESP32C3_ERR_MPI_BAD_INPUT_DATA;
    }

  if (esp32c3_mpi_bitlen(E) > ESP32C3_MPI_MAX_BITS ||
      esp32c3_mpi_bitlen(N) > ESP32C3_MPI_MAX_BITS)
    {
      return ESP32C3_ERR_MPI_BAD_INPUT_DATA;
    }

  /* Init temps and window size
   */

  mpi_montg_init(&mm, N);
  esp32c3_mpi_init(&RR); esp32c3_mpi_init(&T);
  esp32c3_mpi_init(&apos);
  memset(W, 0, sizeof(W));

  i = esp32c3_mpi_bitlen(E);

  wsize = (i > 671) ? 6 : (i > 239) ? 5 :
      (i >  79) ? 4 : (i >  23) ? 3 : 1;

#if (ESP32C3_MPI_WINDOW_SIZE < 6)
  if (wsize > ESP32C3_MPI_WINDOW_SIZE)
    {
      wsize = ESP32C3_MPI_WINDOW_SIZE;
    }
#endif

  j = N->n + 1;
  ESP32C3_MPI_CHK(esp32c3_mpi_grow(X, j), cleanup);
  ESP32C3_MPI_CHK(esp32c3_mpi_grow(&W[1],  j), cleanup);
  ESP32C3_MPI_CHK(esp32c3_mpi_grow(&T, j * 2), cleanup);

  /* Compensate for negative A (and correct at the end) */

  neg = (A->s == -1);
  if (neg)
    {
      ESP32C3_MPI_CHK(esp32c3_mpi_copy(&apos, A), cleanup);
      apos.s = 1;
      A = &apos;
    }

  /* If 1st call, pre-compute R^2 mod N */

  if (_RR == NULL || _RR->p == NULL)
    {
      ESP32C3_MPI_CHK(esp32c3_mpi_lset(&RR, 1), cleanup);
      ESP32C3_MPI_CHK(esp32c3_mpi_shift_l(&RR, N->n * 2 * BIL), cleanup);
      ESP32C3_MPI_CHK(esp32c3_mpi_mod_mpi(&RR, &RR, N), cleanup);

      if (_RR != NULL)
        {
          memcpy(_RR, &RR, sizeof(struct esp32c3_mpi_s));
        }
    }
  else
    {
      memcpy(&RR, _RR, sizeof(struct esp32c3_mpi_s));
    }

  /* W[1] = A * R^2 * R^-1 mod N = A * R mod N */

  if (esp32c3_mpi_cmp_mpi(A, N) >= 0)
    {
      ESP32C3_MPI_CHK(esp32c3_mpi_mod_mpi(&W[1], A, N), cleanup);
    }
  else
    {
      ESP32C3_MPI_CHK(esp32c3_mpi_copy(&W[1], A), cleanup);
    }

  mpi_montmul(&W[1], &RR, N, mm, &T);

  /* X = R^2 * R^-1 mod N = R mod N */

  ESP32C3_MPI_CHK(esp32c3_mpi_copy(X, &RR), cleanup);
  mpi_montred(X, N, mm, &T);

  if (wsize > 1)
    {
      /* W[1 << (wsize - 1)] = W[1] ^ (wsize - 1) */

      j =  one << (wsize - 1);

      ESP32C3_MPI_CHK(esp32c3_mpi_grow(&W[j], N->n + 1), cleanup);
      ESP32C3_MPI_CHK(esp32c3_mpi_copy(&W[j], &W[1]), cleanup);

      for (i = 0; i < wsize - 1; i++)
        {
          mpi_montmul(&W[j], &W[j], N, mm, &T);
        }

      /* W[i] = W[i - 1] * W[1] */

      for (i = j + 1; i < (one << wsize); i++)
        {
          ESP32C3_MPI_CHK(esp32c3_mpi_grow(&W[i], N->n + 1), cleanup);
          ESP32C3_MPI_CHK(esp32c3_mpi_copy(&W[i], &W[i - 1]), cleanup);

          mpi_montmul(&W[i], &W[1], N, mm, &T);
        }
    }

  nblimbs = E->n;
  bufsize = 0;
  nbits   = 0;
  wbits   = 0;
  state   = 0;

  while (1)
    {
      if (bufsize == 0)
        {
          if (nblimbs == 0)
            {
              break;
            }

          nblimbs--;

          bufsize = sizeof(uint32_t) << 3;
        }

      bufsize--;

      ei = (E->p[nblimbs] >> bufsize) & 1;

      /* skip leading 0s */

      if (ei == 0 && state == 0)
        {
          continue;
        }

      if (ei == 0 && state == 1)
        {
          /* out of window, square X */

          mpi_montmul(X, X, N, mm, &T);
          continue;
        }

      /* add ei to current window */

      state = 2;

      nbits++;
      wbits |= (ei << (wsize - nbits));

      if (nbits == wsize)
        {
          /* X = X^wsize R^-1 mod N */

          for (i = 0; i < wsize; i++)
            {
              mpi_montmul(X, X, N, mm, &T);
            }

          /* X = X * W[wbits] R^-1 mod N */

          mpi_montmul(X, &W[wbits], N, mm, &T);

          state--;
          nbits = 0;
          wbits = 0;
        }
    }

  /* process the remaining bits */

  for (i = 0; i < nbits; i++)
    {
      mpi_montmul(X, X, N, mm, &T);

      wbits <<= 1;

      if ((wbits & (one << wsize)) != 0)
        mpi_montmul(X, &W[1], N, mm, &T);
    }

  /* X = A^E * R * R^-1 mod N = A^E mod N */

  mpi_montred(X, N, mm, &T);

  if (neg && E->n != 0 && (E->p[0] & 1) != 0)
    {
      X->s = -1;
      ESP32C3_MPI_CHK(esp32c3_mpi_add_mpi(X, N, X), cleanup);
    }

cleanup:

  for (i = (one << (wsize - 1)); i < (one << wsize); i++)
    {
      esp32c3_mpi_free(&W[i]);
    }

  esp32c3_mpi_free(&W[1]);
  esp32c3_mpi_free(&T);
  esp32c3_mpi_free(&apos);

  if (_RR == NULL || _RR->p == NULL)
    {
      esp32c3_mpi_free(&RR);
    }

  return ret;
}

/****************************************************************************
 * Name: esp32c3_mpi_gcd
 *
 * Description:
 *   Compute the greatest common divisor: G = gcd(A, B)
 *
 * Input Parameters:
 *   G       - The destination MPI
 *   A       - The first operand
 *   B       - The second operand
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_gcd(struct esp32c3_mpi_s *G,
                    const struct esp32c3_mpi_s *A,
                    const struct esp32c3_mpi_s *B)
{
  int ret;
  size_t lz, lzt;
  struct esp32c3_mpi_s TG, TA, TB;

  DEBUGASSERT(G != NULL);
  DEBUGASSERT(A != NULL);
  DEBUGASSERT(B != NULL);

  esp32c3_mpi_init(&TG); esp32c3_mpi_init(&TA); esp32c3_mpi_init(&TB);

  ESP32C3_MPI_CHK(esp32c3_mpi_copy(&TA, A), cleanup);
  ESP32C3_MPI_CHK(esp32c3_mpi_copy(&TB, B), cleanup);

  lz = esp32c3_mpi_lsb(&TA);
  lzt = esp32c3_mpi_lsb(&TB);

  if (lzt < lz)
    {
      lz = lzt;
    }

  ESP32C3_MPI_CHK(esp32c3_mpi_shift_r(&TA, lz), cleanup);
  ESP32C3_MPI_CHK(esp32c3_mpi_shift_r(&TB, lz), cleanup);

  TA.s = TB.s = 1;

  while (esp32c3_mpi_cmp_int(&TA, 0) != 0)
    {
      ESP32C3_MPI_CHK(esp32c3_mpi_shift_r(&TA, esp32c3_mpi_lsb(&TA)),
                      cleanup);
      ESP32C3_MPI_CHK(esp32c3_mpi_shift_r(&TB, esp32c3_mpi_lsb(&TB)),
                      cleanup);

      if (esp32c3_mpi_cmp_mpi(&TA, &TB) >= 0)
        {
          ESP32C3_MPI_CHK(esp32c3_mpi_sub_abs(&TA, &TA, &TB), cleanup);
          ESP32C3_MPI_CHK(esp32c3_mpi_shift_r(&TA, 1), cleanup);
        }
      else
        {
          ESP32C3_MPI_CHK(esp32c3_mpi_sub_abs(&TB, &TB, &TA), cleanup);
          ESP32C3_MPI_CHK(esp32c3_mpi_shift_r(&TB, 1), cleanup);
        }
    }

  ESP32C3_MPI_CHK(esp32c3_mpi_shift_l(&TB, lz), cleanup);
  ESP32C3_MPI_CHK(esp32c3_mpi_copy(G, &TB), cleanup);

cleanup:

  esp32c3_mpi_free(&TG); esp32c3_mpi_free(&TA); esp32c3_mpi_free(&TB);

  return ret;
}

/****************************************************************************
 * Name: esp32c3_mpi_fill_random
 *
 * Description:
 *   Fill an MPI with a number of random bytes
 *
 * Input Parameters:
 *   X        - The destination MPI
 *   size     - The number of random bytes to generate
 *   f_rng    - The RNG function to use. This must not be \c NULL
 *   p_rng    - The RNG parameter to be passed to \p f_rng
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_fill_random(struct esp32c3_mpi_s *X, size_t size,
                            int (*f_rng)(void *, unsigned char *, size_t),
                            void *p_rng)
{
  int ret;
  size_t const limbs = CHARS_TO_LIMBS(size);
  size_t const overhead = (limbs * CIL) - size;
  unsigned char *XP;

  DEBUGASSERT(X   != NULL);
  DEBUGASSERT(f_rng != NULL);

  /* Ensure that target MPI has exactly the necessary number of limbs */

  if (X->n != limbs)
    {
      esp32c3_mpi_free(X);
      esp32c3_mpi_init(X);
      ESP32C3_MPI_CHK(esp32c3_mpi_grow(X, limbs), cleanup);
    }

  ESP32C3_MPI_CHK(esp32c3_mpi_lset(X, 0), cleanup);

  XP = (unsigned char *) X->p;
  ESP32C3_MPI_CHK(f_rng(p_rng, XP + overhead, size), cleanup);

  mpi_bigendian_to_host(X->p, limbs);

cleanup:
  return ret;
}

/****************************************************************************
 * Name: esp32c3_mpi_inv_mod
 *
 * Description:
 *   Compute the modular inverse: X = A^-1 mod N
 *
 * Input Parameters:
 *   X        - The destination MPI
 *   A        - The MPI to calculate the modular inverse of
 *   N        - The base of the modular inversion
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32c3_mpi_inv_mod(struct esp32c3_mpi_s *X,
                        const struct esp32c3_mpi_s *A,
                        const struct esp32c3_mpi_s *N)
{
  int ret;
  struct esp32c3_mpi_s G, TA, TU, U1, U2, TB, TV, V1, V2;
  DEBUGASSERT(X != NULL);
  DEBUGASSERT(A != NULL);
  DEBUGASSERT(N != NULL);

  if (esp32c3_mpi_cmp_int(N, 1) <= 0)
    {
      return ESP32C3_ERR_MPI_BAD_INPUT_DATA;
    }

  esp32c3_mpi_init(&TA);
  esp32c3_mpi_init(&TU);
  esp32c3_mpi_init(&U1);
  esp32c3_mpi_init(&U2);
  esp32c3_mpi_init(&G);
  esp32c3_mpi_init(&TB);
  esp32c3_mpi_init(&TV);
  esp32c3_mpi_init(&V1);
  esp32c3_mpi_init(&V2);

  ESP32C3_MPI_CHK(esp32c3_mpi_gcd(&G, A, N), cleanup);

  if (esp32c3_mpi_cmp_int(&G, 1) != 0)
    {
      ret = ESP32C3_ERR_MPI_NOT_ACCEPTABLE;
      goto cleanup;
    }

  ESP32C3_MPI_CHK(esp32c3_mpi_mod_mpi(&TA, A, N), cleanup);
  ESP32C3_MPI_CHK(esp32c3_mpi_copy(&TU, &TA), cleanup);
  ESP32C3_MPI_CHK(esp32c3_mpi_copy(&TB, N), cleanup);
  ESP32C3_MPI_CHK(esp32c3_mpi_copy(&TV, N), cleanup);

  ESP32C3_MPI_CHK(esp32c3_mpi_lset(&U1, 1), cleanup);
  ESP32C3_MPI_CHK(esp32c3_mpi_lset(&U2, 0), cleanup);
  ESP32C3_MPI_CHK(esp32c3_mpi_lset(&V1, 0), cleanup);
  ESP32C3_MPI_CHK(esp32c3_mpi_lset(&V2, 1), cleanup);

  do
    {
      while ((TU.p[0] & 1) == 0)
        {
          ESP32C3_MPI_CHK(esp32c3_mpi_shift_r(&TU, 1), cleanup);

          if ((U1.p[0] & 1) != 0 || (U2.p[0] & 1) != 0)
            {
              ESP32C3_MPI_CHK(esp32c3_mpi_add_mpi(&U1, &U1, &TB), cleanup);
              ESP32C3_MPI_CHK(esp32c3_mpi_sub_mpi(&U2, &U2, &TA), cleanup);
            }

          ESP32C3_MPI_CHK(esp32c3_mpi_shift_r(&U1, 1), cleanup);
          ESP32C3_MPI_CHK(esp32c3_mpi_shift_r(&U2, 1), cleanup);
        }

      while ((TV.p[0] & 1) == 0)
        {
          ESP32C3_MPI_CHK(esp32c3_mpi_shift_r(&TV, 1), cleanup);

          if ((V1.p[0] & 1) != 0 || (V2.p[0] & 1) != 0)
            {
              ESP32C3_MPI_CHK(esp32c3_mpi_add_mpi(&V1, &V1, &TB), cleanup);
              ESP32C3_MPI_CHK(esp32c3_mpi_sub_mpi(&V2, &V2, &TA), cleanup);
            }

          ESP32C3_MPI_CHK(esp32c3_mpi_shift_r(&V1, 1), cleanup);
          ESP32C3_MPI_CHK(esp32c3_mpi_shift_r(&V2, 1), cleanup);
        }

      if (esp32c3_mpi_cmp_mpi(&TU, &TV) >= 0)
        {
          ESP32C3_MPI_CHK(esp32c3_mpi_sub_mpi(&TU, &TU, &TV), cleanup);
          ESP32C3_MPI_CHK(esp32c3_mpi_sub_mpi(&U1, &U1, &V1), cleanup);
          ESP32C3_MPI_CHK(esp32c3_mpi_sub_mpi(&U2, &U2, &V2), cleanup);
        }
      else
        {
          ESP32C3_MPI_CHK(esp32c3_mpi_sub_mpi(&TV, &TV, &TU), cleanup);
          ESP32C3_MPI_CHK(esp32c3_mpi_sub_mpi(&V1, &V1, &U1), cleanup);
          ESP32C3_MPI_CHK(esp32c3_mpi_sub_mpi(&V2, &V2, &U2), cleanup);
        }
    }
  while (esp32c3_mpi_cmp_int(&TU, 0) != 0);

  while (esp32c3_mpi_cmp_int(&V1, 0) < 0)
    {
      ESP32C3_MPI_CHK(esp32c3_mpi_add_mpi(&V1, &V1, N), cleanup);
    }

  while (esp32c3_mpi_cmp_mpi(&V1, N) >= 0)
    {
      ESP32C3_MPI_CHK(esp32c3_mpi_sub_mpi(&V1, &V1, N), cleanup);
    }

  ESP32C3_MPI_CHK(esp32c3_mpi_copy(X, &V1), cleanup);

cleanup:

  esp32c3_mpi_free(&TA);
  esp32c3_mpi_free(&TU);
  esp32c3_mpi_free(&U1);
  esp32c3_mpi_free(&U2);
  esp32c3_mpi_free(&G);
  esp32c3_mpi_free(&TB);
  esp32c3_mpi_free(&TV);
  esp32c3_mpi_free(&V1);
  esp32c3_mpi_free(&V2);

  return ret;
}

#endif /* CONFIG_ESP32C3_BIGNUM_ACCELERATOR */

