/****************************************************************************
 * arch/risc-v/src/erbium/erbium_mcode.c
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

/* ET-Minion raises exception 30 (M-code emulation) for FCVT.L.S,
 * FCVT.LU.S, FCVT.S.L, FCVT.S.LU, FDIV.S and FSQRT.S instead of executing
 * them. This file performs them on the trapped context with RISC-V F
 * semantics and the ET-Minion subnormal rules of its other F instructions:
 * a subnormal input reads as zero and sets fflags bit 31, and a result that
 * is tiny after rounding is flushed to zero with underflow and inexact.
 *
 * Keep this file integer-only: a float, a double or a conversion between
 * floating point and a 64-bit integer would compile to the instructions it
 * emulates, and any FP register use would clobber the interrupted
 * context's FP state, which the trap path does not save.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <arch/irq.h>

#include "riscv_internal.h"
#include "erbium_mcode.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* OP-FP, fmt S, rs2 2 (L) or 3 (LU); the mask keeps funct7, rs2, opcode */

#define FCVT_MASK          0xfff0007f
#define FCVT_L_S           0xc0200053
#define FCVT_LU_S          0xc0300053
#define FCVT_S_L           0xd0200053
#define FCVT_S_LU          0xd0300053
#define FCVT_UNSIGNED      (1u << 20)     /* rs2 bit 0: the LU variants */

/* OP-FP, fmt S: FSQRT.S has rs2 0 and uses FCVT_MASK; FDIV.S keeps funct7 */

#define FSQRT_S            0x58000053
#define FDIV_MASK          0xfe00007f
#define FDIV_S             0x18000053

#define INSN_RD(i)         (((i) >> 7) & 0x1f)
#define INSN_RM(i)         (((i) >> 12) & 0x7)
#define INSN_RS1(i)        (((i) >> 15) & 0x1f)
#define INSN_RS2(i)        (((i) >> 20) & 0x1f)

#define XREG_ZERO          0
#define XREG_GP            3

#define FRM_RNE            0
#define FRM_RTZ            1
#define FRM_RDN            2
#define FRM_RUP            3
#define FRM_RMM            4
#define FRM_DYN            7

#define FFLAG_NX           0x01u
#define FFLAG_UF           0x02u
#define FFLAG_OF           0x04u
#define FFLAG_DZ           0x08u
#define FFLAG_NV           0x10u
#define FFLAG_INPUTDENORM  0x80000000u    /* ET-Minion fflags bit 31 */

/* IEEE 754 binary32. A normal value with biased exponent e and fraction f
 * has magnitude (2^23 + f) * 2^(e - 150); e = 255 encodes Inf and NaN.
 */

#define F32_FRAC_BITS      23
#define F32_BIAS           127
#define F32_EXP_INFNAN     0xff
#define F32_HIDDEN         (UINT64_C(1) << F32_FRAC_BITS)
#define F32_SIGN(x)        (((x) >> 31) != 0)
#define F32_EXP(x)         (((x) >> F32_FRAC_BITS) & 0xff)
#define F32_FRAC(x)        ((x) & 0x7fffff)
#define F32_SIGN_BIT       0x80000000u
#define F32_INF            0x7f800000u
#define F32_QNAN           0x7fc00000u    /* RISC-V canonical NaN */
#define F32_IS_SNAN(x)     (((x) & 0x7fc00000) == 0x7f800000 && \
                            F32_FRAC(x) != 0)

#define I64_MIN_BITS       UINT64_C(0x8000000000000000)
#define I64_MAX_BITS       UINT64_C(0x7fffffffffffffff)

/* The trap frame holds xN in regs[N] for N = 1..31; regs[0] is the EPC */

static_assert(REG_X1 == 1 && REG_X31 == 31, "Unexpected trap frame");

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: erbium_msb64
 *
 * Description:
 *   Find the most significant set bit of a 64-bit value.
 *
 * Input Parameters:
 *   val - The value, which must not be zero.
 *
 * Returned Value:
 *   The bit index, 0 to 63.
 *
 ****************************************************************************/

static unsigned int erbium_msb64(uint64_t val)
{
  unsigned int msb = 0;
  unsigned int step;

  for (step = 32; step != 0; step >>= 1)
    {
      if ((val >> step) != 0)
        {
          val >>= step;
          msb += step;
        }
    }

  return msb;
}

/****************************************************************************
 * Name: erbium_round_up
 *
 * Description:
 *   Decide whether a magnitude truncated to an integer must be incremented
 *   to round it in a RISC-V rounding mode.
 *
 * Input Parameters:
 *   rm   - Rounding mode, FRM_RNE to FRM_RMM.
 *   sign - True if the value is negative.
 *   mag  - The truncated magnitude.
 *   rem  - The discarded bits.
 *   half - The weight of the highest discarded bit.
 *
 * Returned Value:
 *   True if the magnitude must be incremented.
 *
 ****************************************************************************/

static bool erbium_round_up(unsigned int rm, bool sign, uint64_t mag,
                            uint64_t rem, uint64_t half)
{
  switch (rm)
    {
      case FRM_RNE:
        return rem > half || (rem == half && (mag & 1) != 0);

      case FRM_RDN:
        return sign && rem != 0;

      case FRM_RUP:
        return !sign && rem != 0;

      case FRM_RMM:
        return rem >= half;

      default:                                    /* FRM_RTZ */
        return false;
    }
}

/****************************************************************************
 * Name: erbium_x64_invalid
 *
 * Description:
 *   Raise the invalid-operation flag for a float to 64-bit integer
 *   conversion whose result does not fit, and saturate the result.
 *
 * Input Parameters:
 *   sign        - True if the value is negative, false for NaN.
 *   is_unsigned - True for FCVT.LU.S.
 *   fflags      - Accumulated exception flags.
 *
 * Returned Value:
 *   The representable integer closest to the value; the largest one for
 *   NaN.
 *
 ****************************************************************************/

static uint64_t erbium_x64_invalid(bool sign, bool is_unsigned,
                                   uint32_t *fflags)
{
  *fflags |= FFLAG_NV;                      /* NV alone, never with NX */

  if (is_unsigned)
    {
      return sign ? 0 : UINT64_MAX;
    }

  return sign ? I64_MIN_BITS : I64_MAX_BITS;
}

/****************************************************************************
 * Name: erbium_f32_to_x64
 *
 * Description:
 *   Perform FCVT.L.S or FCVT.LU.S.
 *
 * Input Parameters:
 *   bits        - The single-precision source operand.
 *   rm          - Rounding mode, FRM_RNE to FRM_RMM.
 *   is_unsigned - True for FCVT.LU.S.
 *   fflags      - Accumulated exception flags.
 *
 * Returned Value:
 *   The 64-bit integer result.
 *
 ****************************************************************************/

static uint64_t erbium_f32_to_x64(uint32_t bits, unsigned int rm,
                                  bool is_unsigned, uint32_t *fflags)
{
  bool sign = F32_SIGN(bits);
  unsigned int exp = F32_EXP(bits);
  uint64_t sig = F32_FRAC(bits) | F32_HIDDEN;
  uint64_t limit;
  uint64_t mag;
  uint64_t rem = 0;
  unsigned int shift;

  if (exp == 0)
    {
      /* Zero, or a subnormal that ET-Minion reads as zero and flags */

      if (F32_FRAC(bits) != 0)
        {
          *fflags |= FFLAG_INPUTDENORM;
        }

      return 0;
    }

  if (exp == F32_EXP_INFNAN && F32_FRAC(bits) != 0)
    {
      sign = false;                         /* NaN converts like +Inf */
    }

  if (exp >= F32_BIAS + 64)                 /* |x| >= 2^64, Inf or NaN */
    {
      return erbium_x64_invalid(sign, is_unsigned, fflags);
    }

  if (exp >= F32_BIAS + F32_FRAC_BITS)      /* Integral, exact */
    {
      mag = sig << (exp - F32_BIAS - F32_FRAC_BITS);
    }
  else if (exp >= F32_BIAS - 1)             /* 0.5 <= |x| < 2^23 */
    {
      shift = F32_BIAS + F32_FRAC_BITS - exp; /* 1..24 */
      mag = sig >> shift;
      rem = sig & ((UINT64_C(1) << shift) - 1);
      if (erbium_round_up(rm, sign, mag, rem, UINT64_C(1) << (shift - 1)))
        {
          mag++;
        }
    }
  else                                      /* 0 < |x| < 0.5 */
    {
      mag = 0;
      rem = 1;                              /* Nonzero, below one half */
      if (erbium_round_up(rm, sign, 0, rem, 2))
        {
          mag = 1;
        }
    }

  /* The largest magnitude that the destination holds with this sign */

  if (is_unsigned)
    {
      limit = sign ? 0 : UINT64_MAX;
    }
  else
    {
      limit = sign ? I64_MIN_BITS : I64_MAX_BITS;
    }

  if (mag > limit)
    {
      return erbium_x64_invalid(sign, is_unsigned, fflags);
    }

  if (rem != 0)
    {
      *fflags |= FFLAG_NX;
    }

  return sign ? 0 - mag : mag;
}

/****************************************************************************
 * Name: erbium_x64_to_f32
 *
 * Description:
 *   Perform FCVT.S.L or FCVT.S.LU.
 *
 * Input Parameters:
 *   val         - The 64-bit integer source operand.
 *   rm          - Rounding mode, FRM_RNE to FRM_RMM.
 *   is_unsigned - True for FCVT.S.LU.
 *   fflags      - Accumulated exception flags.
 *
 * Returned Value:
 *   The single-precision result.
 *
 ****************************************************************************/

static uint32_t erbium_x64_to_f32(uint64_t val, unsigned int rm,
                                  bool is_unsigned, uint32_t *fflags)
{
  bool sign = !is_unsigned && (int64_t)val < 0;
  uint64_t mag = sign ? 0 - val : val;
  unsigned int msb;
  unsigned int exp;
  unsigned int shift;
  uint64_t sig;
  uint64_t rem;

  if (mag == 0)
    {
      return 0;                             /* +0.0 in every mode */
    }

  msb = erbium_msb64(mag);
  exp = F32_BIAS + msb;

  if (msb <= F32_FRAC_BITS)
    {
      sig = mag << (F32_FRAC_BITS - msb);   /* Exact */
    }
  else
    {
      shift = msb - F32_FRAC_BITS;          /* 1..40 */
      sig = mag >> shift;
      rem = mag & ((UINT64_C(1) << shift) - 1);
      if (rem != 0)
        {
          *fflags |= FFLAG_NX;
          if (erbium_round_up(rm, sign, sig, rem,
                              UINT64_C(1) << (shift - 1)) &&
              ++sig == (F32_HIDDEN << 1))
            {
              sig >>= 1;                    /* Carry into the exponent */
              exp++;
            }
        }
    }

  return (sign ? F32_SIGN_BIT : 0) | (exp << F32_FRAC_BITS) |
         (uint32_t)F32_FRAC(sig);
}

/****************************************************************************
 * Name: erbium_f32_round_pack
 *
 * Description:
 *   Round and pack a single-precision result. 'sig' holds the significand
 *   with its leading one at bit 30 and seven guard bits, and the value is
 *   sig * 2^(exp - 156). A result that is tiny after rounding is flushed to
 *   zero, as ET-Minion does.
 *
 * Input Parameters:
 *   sign   - True if the result is negative.
 *   exp    - Biased exponent minus one.
 *   sig    - Significand, below 2^31, with a sticky bit in bit 0.
 *   rm     - Rounding mode, FRM_RNE to FRM_RMM.
 *   fflags - Accumulated exception flags.
 *
 * Returned Value:
 *   The single-precision result.
 *
 ****************************************************************************/

static uint32_t erbium_f32_round_pack(bool sign, int exp, uint32_t sig,
                                      unsigned int rm, uint32_t *fflags)
{
  uint32_t increment = 0x40;
  uint32_t bits;
  bool tiny;

  if (rm == FRM_RTZ || (rm == FRM_RDN && !sign) || (rm == FRM_RUP && sign))
    {
      increment = 0;
    }
  else if (rm == FRM_RDN || rm == FRM_RUP)
    {
      increment = 0x7f;
    }

  if (exp < 0)
    {
      tiny = exp < -1 || sig + increment < 0x80000000u;
      if (tiny)
        {
          *fflags |= FFLAG_UF | FFLAG_NX;
          return sign ? F32_SIGN_BIT : 0;
        }

      sig = (sig >> 1) | (sig & 1);     /* exp is -1: shift right, jam */
      exp = 0;
    }
  else if (exp > 0xfd || (exp == 0xfd && sig + increment >= 0x80000000u))
    {
      *fflags |= FFLAG_OF | FFLAG_NX;
      return (sign ? F32_SIGN_BIT : 0) | (increment != 0 ? F32_INF :
                                          F32_INF - 1);
    }

  bits = sig & 0x7f;
  if (bits != 0)
    {
      *fflags |= FFLAG_NX;
    }

  sig = (sig + increment) >> 7;
  if (rm == FRM_RNE && bits == 0x40)
    {
      sig &= ~UINT32_C(1);              /* Tie: round to even */
    }

  if (sig == 0)
    {
      exp = 0;
    }

  return (sign ? F32_SIGN_BIT : 0) + ((uint32_t)exp << F32_FRAC_BITS) + sig;
}

/****************************************************************************
 * Name: erbium_f32_nan
 *
 * Description:
 *   Return the canonical NaN, and raise the invalid-operation flag if an
 *   operand is a signaling NaN.
 *
 * Input Parameters:
 *   a, b   - Operands; pass a quiet value for an unused one.
 *   fflags - Accumulated exception flags.
 *
 * Returned Value:
 *   The canonical NaN.
 *
 ****************************************************************************/

static uint32_t erbium_f32_nan(uint32_t a, uint32_t b, uint32_t *fflags)
{
  if (F32_IS_SNAN(a) || F32_IS_SNAN(b))
    {
      *fflags |= FFLAG_NV;
    }

  return F32_QNAN;
}

/****************************************************************************
 * Name: erbium_f32_daz
 *
 * Description:
 *   Read a subnormal input as a zero of the same sign and flag it.
 *
 * Input Parameters:
 *   x      - Operand.
 *   fflags - Accumulated exception flags.
 *
 * Returned Value:
 *   The operand, or a signed zero for a subnormal.
 *
 ****************************************************************************/

static uint32_t erbium_f32_daz(uint32_t x, uint32_t *fflags)
{
  if (F32_EXP(x) == 0 && F32_FRAC(x) != 0)
    {
      *fflags |= FFLAG_INPUTDENORM;
      return x & F32_SIGN_BIT;
    }

  return x;
}

/****************************************************************************
 * Name: erbium_f32_div
 *
 * Description:
 *   Perform FDIV.S.
 *
 * Input Parameters:
 *   a      - Dividend.
 *   b      - Divisor.
 *   rm     - Rounding mode, FRM_RNE to FRM_RMM.
 *   fflags - Accumulated exception flags.
 *
 * Returned Value:
 *   The single-precision quotient.
 *
 ****************************************************************************/

static uint32_t erbium_f32_div(uint32_t a, uint32_t b, unsigned int rm,
                               uint32_t *fflags)
{
  bool sign = F32_SIGN(a) != F32_SIGN(b);
  uint32_t sign_bit = sign ? F32_SIGN_BIT : 0;
  unsigned int exp_a;
  unsigned int exp_b;
  uint64_t dividend;
  uint64_t sig_a;
  uint64_t sig_b;
  uint64_t sig;
  int exp;

  a = erbium_f32_daz(a, fflags);
  b = erbium_f32_daz(b, fflags);
  exp_a = F32_EXP(a);
  exp_b = F32_EXP(b);

  if (exp_a == F32_EXP_INFNAN)
    {
      if (F32_FRAC(a) != 0 ||
          (exp_b == F32_EXP_INFNAN && F32_FRAC(b) != 0))
        {
          return erbium_f32_nan(a, b, fflags);
        }

      if (exp_b == F32_EXP_INFNAN)
        {
          *fflags |= FFLAG_NV;          /* Inf / Inf */
          return F32_QNAN;
        }

      return sign_bit | F32_INF;
    }

  if (exp_b == F32_EXP_INFNAN)
    {
      if (F32_FRAC(b) != 0)
        {
          return erbium_f32_nan(a, b, fflags);
        }

      return sign_bit;                  /* x / Inf */
    }

  if (exp_b == 0)
    {
      if (exp_a == 0)
        {
          *fflags |= FFLAG_NV;          /* 0 / 0 */
          return F32_QNAN;
        }

      *fflags |= FFLAG_DZ;
      return sign_bit | F32_INF;
    }

  if (exp_a == 0)
    {
      return sign_bit;                  /* 0 / x */
    }

  /* The quotient of two significands in [1, 2) lies in (1/2, 2); scale it
   * so that its leading one lands on bit 30.
   */

  exp = (int)exp_a - (int)exp_b + 0x7e;
  sig_a = F32_FRAC(a) | F32_HIDDEN;
  sig_b = F32_FRAC(b) | F32_HIDDEN;
  if (sig_a < sig_b)
    {
      exp--;
      dividend = sig_a << 31;
    }
  else
    {
      dividend = sig_a << 30;
    }

  sig = dividend / sig_b;
  if (sig * sig_b != dividend)
    {
      sig |= 1;                         /* Sticky */
    }

  return erbium_f32_round_pack(sign, exp, (uint32_t)sig, rm, fflags);
}

/****************************************************************************
 * Name: erbium_isqrt64
 *
 * Description:
 *   Integer square root, bit by bit.
 *
 * Input Parameters:
 *   val - Radicand.
 *   rem - Receives val - root^2.
 *
 * Returned Value:
 *   floor(sqrt(val)).
 *
 ****************************************************************************/

static uint64_t erbium_isqrt64(uint64_t val, uint64_t *rem)
{
  uint64_t root = 0;
  uint64_t bit = UINT64_C(1) << 62;

  while (bit > val)
    {
      bit >>= 2;
    }

  while (bit != 0)
    {
      if (val >= root + bit)
        {
          val -= root + bit;
          root = (root >> 1) + bit;
        }
      else
        {
          root >>= 1;
        }

      bit >>= 2;
    }

  *rem = val;
  return root;
}

/****************************************************************************
 * Name: erbium_f32_sqrt
 *
 * Description:
 *   Perform FSQRT.S.
 *
 * Input Parameters:
 *   a      - Operand.
 *   rm     - Rounding mode, FRM_RNE to FRM_RMM.
 *   fflags - Accumulated exception flags.
 *
 * Returned Value:
 *   The single-precision square root.
 *
 ****************************************************************************/

static uint32_t erbium_f32_sqrt(uint32_t a, unsigned int rm,
                                uint32_t *fflags)
{
  unsigned int exp_a = F32_EXP(a);
  unsigned int shift;
  uint64_t radicand;
  uint64_t rem;
  uint64_t sig;

  if (exp_a == F32_EXP_INFNAN)
    {
      if (F32_FRAC(a) != 0)
        {
          return erbium_f32_nan(a, 0, fflags);
        }

      if (!F32_SIGN(a))
        {
          return a;                     /* sqrt(+Inf) */
        }

      *fflags |= FFLAG_NV;
      return F32_QNAN;
    }

  if (exp_a == 0)
    {
      return erbium_f32_daz(a, fflags); /* sqrt(+-0) is +-0 */
    }

  if (F32_SIGN(a))
    {
      *fflags |= FFLAG_NV;
      return F32_QNAN;
    }

  /* |a| = sig_a * 2^(exp_a - 150). Shift sig_a into [2^60, 2^62) by an
   * amount that leaves an even power of two, so that the root has its
   * leading one on bit 30 and the exponent halves exactly.
   */

  shift = (exp_a & 1) != 0 ? 37 : 38;
  radicand = (uint64_t)(F32_FRAC(a) | F32_HIDDEN) << shift;
  sig = erbium_isqrt64(radicand, &rem);
  if (rem != 0)
    {
      sig |= 1;                         /* Sticky */
    }

  return erbium_f32_round_pack(false,
                               ((int)exp_a - 150 - (int)shift) / 2 + 156,
                               (uint32_t)sig, rm, fflags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: erbium_mcode_emulate
 *
 * Description:
 *   Perform the instruction that raised exception 30 (M-code emulation)
 *   on the saved context and advance its EPC. FCVT.L.S, FCVT.LU.S,
 *   FCVT.S.L, FCVT.S.LU, FDIV.S and FSQRT.S are emulated.
 *
 * Input Parameters:
 *   regs - Saved registers of the trapped context.
 *
 * Returned Value:
 *   OK if the instruction was emulated; -EINVAL if it must raise
 *   illegal-instruction instead; -ENOSYS if it is not emulated.
 *
 ****************************************************************************/

int erbium_mcode_emulate(uintreg_t *regs)
{
  uintptr_t epc = regs[REG_EPC];
  uint32_t fflags = 0;
  bool fp_written = false;
  bool is_unsigned;
  unsigned int rd;
  unsigned int rs1;
  unsigned int rm;
  uint32_t insn;

  /* Fetch the instruction. mtval holds it on erbium_emu, but the Erbium
   * documentation does not specify mtval for exception 30. With the C
   * extension a 32-bit instruction may be only 2-byte aligned, and a 16-bit
   * instruction (possible only through minstmask) must not be read past.
   */

  insn = *(FAR volatile const uint16_t *)epc;
  if ((insn & 3) != 3)
    {
      return -ENOSYS;
    }

  insn |= (uint32_t)*(FAR volatile const uint16_t *)(epc + 2) << 16;

  switch (insn & FCVT_MASK)
    {
      case FCVT_L_S:
      case FCVT_LU_S:
      case FCVT_S_L:
      case FCVT_S_LU:
      case FSQRT_S:
        break;

      default:
        if ((insn & FDIV_MASK) != FDIV_S)
          {
            return -ENOSYS;   /* FENCE.I, SFENCE.VMA, packed ops, ... */
          }

        break;
    }

  rd          = INSN_RD(insn);
  rs1         = INSN_RS1(insn);
  rm          = INSN_RM(insn);
  is_unsigned = (insn & FCVT_UNSIGNED) != 0;

  /* Exception 30 outranks the illegal-instruction checks that a native F
   * instruction makes (FPU off, reserved rounding mode). Make them here,
   * before touching any FP register or CSR.
   */

  if ((regs[REG_INT_CTX] & MSTATUS_FS) == 0)
    {
      return -EINVAL;
    }

  if (rm == FRM_DYN)
    {
      rm = READ_CSR(CSR_FRM);
    }

  if (rm > FRM_RMM)
    {
      return -EINVAL;
    }

  if ((insn & FDIV_MASK) == FDIV_S)
    {
      erbium_mcode_setfpr(rd, erbium_f32_div(erbium_mcode_getfpr(rs1),
                                             erbium_mcode_getfpr(
                                               INSN_RS2(insn)),
                                             rm, &fflags));
      fp_written = true;
    }
  else if ((insn & FCVT_MASK) == FSQRT_S)
    {
      erbium_mcode_setfpr(rd, erbium_f32_sqrt(erbium_mcode_getfpr(rs1), rm,
                                              &fflags));
      fp_written = true;
    }
  else if ((insn & FCVT_MASK) == FCVT_L_S ||
           (insn & FCVT_MASK) == FCVT_LU_S)
    {
      uint64_t res;

      if (rd == XREG_GP)
        {
          return -ENOSYS;  /* gp is not restored from the trap frame */
        }

      res = erbium_f32_to_x64(erbium_mcode_getfpr(rs1), rm, is_unsigned,
                              &fflags);
      if (rd != XREG_ZERO)
        {
          regs[rd] = res;  /* Never regs[0], which is the EPC */
        }
    }
  else
    {
      uint64_t src = 0;

      if (rs1 == XREG_GP)
        {
          __asm__ __volatile__("mv %0, gp" : "=r"(src));
        }
      else if (rs1 != XREG_ZERO)
        {
          src = regs[rs1]; /* For sp, the frame holds the pre-trap value */
        }

      erbium_mcode_setfpr(rd, erbium_x64_to_f32(src, rm, is_unsigned,
                                                &fflags));
      fp_written = true;
    }

  if (fflags != 0)
    {
      SET_CSR(CSR_FFLAGS, (uintreg_t)fflags);
    }

  /* The trap return reloads mstatus from the frame. Record the FP state
   * change there, or a lazy FPU context save would skip it.
   */

  if (fp_written || fflags != 0)
    {
      regs[REG_INT_CTX] |= MSTATUS_FS_DIRTY;
    }

  regs[REG_EPC] = epc + 4;  /* None of these has a compressed form */
  return OK;
}
