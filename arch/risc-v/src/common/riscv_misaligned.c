/****************************************************************************
 * arch/risc-v/src/common/riscv_misaligned.c
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

#include <nuttx/irq.h>

#include <assert.h>
#include <debug.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>

#include "riscv_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Instruction mask for load/store */

#define INSN_MASK    0x707f
#define INSN_C_MASK  0xe003

/* Load instruction opcode */

#define INSN_LB      0x0003
#define INSN_LH      0x1003
#define INSN_LW      0x2003
#define INSN_LD      0x3003
#define INSN_LBU     0x4003
#define INSN_LHU     0x5003
#define INSN_LWU     0x6003

#define INSN_C_LW    0x4000
#define INSN_C_LWSP  0x4002
#define INSN_C_LD    0x6000
#define INSN_C_LDSP  0x6002

/* Store instruction opcode */

#define INSN_SB      0x0023
#define INSN_SH      0x1023
#define INSN_SW      0x2023
#define INSN_SD      0x3023

#define INSN_C_SW    0xc000
#define INSN_C_SWSP  0xc002
#define INSN_C_SD    0xe000
#define INSN_C_SDSP  0xe002

/* Float load instruction opcode */

#define INSN_FLW     0x2007
#define INSN_FLD     0x3007

#define INSN_C_FLW   0x6000
#define INSN_C_FLWSP 0x6002
#define INSN_C_FLD   0x2000
#define INSN_C_FLDSP 0x2002

/* Float store instruction opcode */

#define INSN_FSW     0x2027
#define INSN_FSD     0x3027

#define INSN_C_FSW   0xe000
#define INSN_C_FSWSP 0xe002
#define INSN_C_FSD   0xa000
#define INSN_C_FSDSP 0xa002

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* Compressed instruction encoding */

typedef union
{
  uint16_t insn;

  /* Load */

  struct
  {
    uint16_t op     : 2;
    uint16_t rd     : 3;
    uint16_t imm6   : 1; /* IMM bit [6] */
    uint16_t imm2   : 1; /* IMM bit [2] */
    uint16_t rs1    : 3;
    uint16_t imm53  : 3; /* IMM bit [5:3] */
    uint16_t funct3 : 3;
  } lw;

  struct
  {
    uint16_t op     : 2;
    uint16_t imm76  : 2; /* IMM bit [7:6] */
    uint16_t imm42  : 3; /* IMM bit [4:2] */
    uint16_t rd     : 5;
    uint16_t imm5   : 1; /* IMM bit [5] */
    uint16_t funct3 : 3;
  } lwsp;

  struct
  {
    uint16_t op     : 2;
    uint16_t rd     : 3;
    uint16_t imm76  : 2; /* IMM bit [7:6] */
    uint16_t rs1    : 3;
    uint16_t imm53  : 3; /* IMM bit [5:3] */
    uint16_t funct3 : 3;
  } ld;

  struct
  {
    uint16_t op     : 2;
    uint16_t imm86  : 3; /* IMM bit [6:6] */
    uint16_t imm43  : 2; /* IMM bit [4:3] */
    uint16_t rd     : 5;
    uint16_t imm5   : 1; /* IMM bit [5] */
    uint16_t funct3 : 3;
  } ldsp;

  /* Store */

  struct
  {
    uint16_t op     : 2;
    uint16_t rs2    : 3;
    uint16_t imm6   : 1; /* IMM bit [6] */
    uint16_t imm2   : 1; /* IMM bit [2] */
    uint16_t rs1    : 3;
    uint16_t imm53  : 3; /* IMM bit [5:3] */
    uint16_t funct3 : 3;
  } sw;

  struct
  {
    uint16_t op     : 2;
    uint16_t rs2    : 5;
    uint16_t imm76  : 2; /* IMM bit [7:6] */
    uint16_t imm52  : 4; /* IMM bit [5:2] */
    uint16_t funct3 : 3;
  } swsp;

  struct
  {
    uint16_t op     : 2;
    uint16_t rs2    : 3;
    uint16_t imm76  : 2; /* IMM bit [7:6] */
    uint16_t rs1    : 3;
    uint16_t imm53  : 3; /* IMM bit [5:3] */
    uint16_t funct3 : 3;
  } sd;

  struct
  {
    uint16_t op     : 2;
    uint16_t rs2    : 5;
    uint16_t imm86  : 3; /* IMM bit [8:6] */
    uint16_t imm53  : 3; /* IMM bit [5:3] */
    uint16_t funct3 : 3;
  } sdsp;
} riscv_insn_c_t;

/* Normal instruction encoding */

typedef union
{
  uint32_t insn;

  /* Load */

  struct
  {
    uint32_t op     : 7;
    uint32_t rd     : 5;
    uint32_t funct3 : 3;
    uint32_t rs1    : 5;
    uint32_t imm    : 12;
  } l;

  /* Store */

  struct
  {
    uint32_t op     : 7;
    uint32_t imm2   : 5;
    uint32_t funct3 : 3;
    uint32_t rs1    : 5;
    uint32_t rs2    : 5;
    uint32_t imm1   : 7;
  } s;
} riscv_insn_t;

typedef struct
{
  uint8_t *dest;
  uint8_t *src;
  int      len;
  bool     sext;
} riscv_insn_ctx_t;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sext
 *
 * Description:
 *   Sign extension a given bit width data
 *
 ****************************************************************************/

static intptr_t sext(intptr_t v, uint32_t w)
{
  w = 8 * sizeof(intptr_t) - w;
  return v << w >> w;
}

/****************************************************************************
 * Name: decode_insn_compressed
 *
 * Description:
 *   Try to decode a compressed instruction
 *
 ****************************************************************************/

static bool decode_insn_compressed(uintptr_t *regs, riscv_insn_ctx_t *ctx)
{
#ifdef CONFIG_ARCH_RV_ISA_C
  uint16_t in;
  uint32_t imm;
  riscv_insn_c_t insn;

  /* Fetch instruction */

  memcpy(&insn.insn, (void *)regs[REG_EPC], 2);

  in = insn.insn & INSN_C_MASK;

  switch (in)
    {
      /* Always need sign extension for c.lw/c.lwsp */

      case INSN_C_LW:
        imm = insn.lw.imm2 << 2 | insn.lw.imm53 << 3 | insn.lw.imm6 << 6;
        ctx->dest = (uint8_t *)&regs[REG_X8 + insn.lw.rd];
        ctx->src  = (uint8_t *)regs[REG_X8 + insn.lw.rs1] + imm;
        ctx->len  = 4;
        ctx->sext = true;
        break;

      case INSN_C_LWSP:
        imm = insn.lwsp.imm42 << 2 | insn.lwsp.imm5 << 5 |
              insn.lwsp.imm76 << 6;
        ctx->dest = (uint8_t *)&regs[insn.lwsp.rd];
        ctx->src  = (uint8_t *)regs[REG_SP] + imm;
        ctx->len  = 4;
        ctx->sext = true;
        break;

#  ifdef CONFIG_ARCH_RV64
      case INSN_C_LD:
        imm = insn.ld.imm53 << 3 | insn.ld.imm76 << 6;
        ctx->dest = (uint8_t *)&regs[REG_X8 + insn.ld.rd];
        ctx->src  = (uint8_t *)regs[REG_X8 + insn.ld.rs1] + imm;
        ctx->len  = 8;
        break;

      case INSN_C_LDSP:
        imm = insn.ldsp.imm43 << 3 | insn.ldsp.imm5 << 5 |
              insn.ldsp.imm86 << 6;
        ctx->dest = (uint8_t *)&regs[insn.ld.rd];
        ctx->src  = (uint8_t *)regs[REG_SP] + imm;
        ctx->len  = 8;
        break;

      case INSN_C_SD:
        imm = insn.sd.imm53 << 3 | insn.sd.imm76 << 6;
        ctx->dest = (uint8_t *)regs[REG_X8 + insn.sd.rs1] + imm;
        ctx->src  = (uint8_t *)&regs[REG_X8 + insn.sd.rs2];
        ctx->len  = 8;
        break;

      case INSN_C_SDSP:
        imm = insn.sdsp.imm53 << 3 | insn.sdsp.imm86 << 6;
        ctx->dest = (uint8_t *)regs[REG_SP] + imm;
        ctx->src  = (uint8_t *)&regs[insn.sdsp.rs2];
        ctx->len  = 8;
        break;
#  endif

      case INSN_C_SW:
        imm = insn.sw.imm2 << 2 | insn.sw.imm53 << 3 | insn.sw.imm6 << 6;
        ctx->dest = (uint8_t *)regs[REG_X8 + insn.sw.rs1] + imm;
        ctx->src  = (uint8_t *)&regs[REG_X8 + insn.sd.rs2];
        ctx->len  = 4;
        break;

      case INSN_C_SWSP:
        imm = insn.swsp.imm52 << 2 | insn.swsp.imm76 << 6;
        ctx->dest = (uint8_t *)regs[REG_SP] + imm;
        ctx->src  = (uint8_t *)&regs[insn.swsp.rs2];
        ctx->len  = 4;
        break;

#  ifdef CONFIG_ARCH_FPU
#    ifdef CONFIG_ARCH_RV32
      case INSN_C_FLW:

        /* flw share the same encoding layout with lw */

        imm = insn.lw.imm2 << 2 | insn.lw.imm53 << 3 | insn.lw.imm6 << 6;
        ctx->dest = (uint8_t *)&regs[REG_F8 + insn.lw.rd];
        ctx->src  = (uint8_t *)regs[REG_X8 + insn.lw.rs1] + imm;
        ctx->len  = 4;
        break;

      case INSN_C_FLWSP:

        /* flwsp share the same encoding layout with lwsp */

        imm = insn.lwsp.imm42 << 2 | insn.lwsp.imm5 << 5 |
              insn.lwsp.imm76 << 6;
        ctx->dest = (uint8_t *)&regs[REG_F0 + insn.lwsp.rd];
        ctx->src  = (uint8_t *)regs[REG_SP] + imm;
        ctx->len  = 4;
        break;

      case INSN_C_FSW:

        /* fsw share the same encoding layout with sw */

        imm = insn.sw.imm2 << 2 | insn.sw.imm53 << 3 | insn.sw.imm6 << 6;
        ctx->dest = (uint8_t *)regs[REG_X8 + insn.sw.rs1] + imm;
        ctx->src  = (uint8_t *)&regs[REG_F8 + insn.sd.rs2];
        ctx->len  = 4;
        break;

      case INSN_C_FSWSP:

        /* fswsp share the same encoding layout with swsp */

        imm = insn.swsp.imm52 << 2 | insn.swsp.imm76 << 6;
        ctx->dest = (uint8_t *)regs[REG_SP] + imm;
        ctx->src  = (uint8_t *)&regs[REG_F0 + insn.swsp.rs2];
        ctx->len  = 4;
        break;

#    endif
      case INSN_C_FLD:

        /* fld share the same encoding layout with ld */

        imm = insn.ld.imm53 << 3 | insn.ld.imm76 << 6;
        ctx->dest = (uint8_t *)&regs[REG_F8 + insn.ld.rd];
        ctx->src  = (uint8_t *)regs[REG_X8 + insn.ld.rs1] + imm;
        ctx->len  = 8;
        break;

      case INSN_C_FLDSP:

        /* fldsp share the same encoding layout with ldsp */

        imm = insn.ldsp.imm43 << 3 | insn.ldsp.imm5 << 5 |
              insn.ldsp.imm86 << 6;
        ctx->dest = (uint8_t *)&regs[REG_F0 + insn.ld.rd];
        ctx->src  = (uint8_t *)regs[REG_SP] + imm;
        ctx->len  = 8;
        break;

      case INSN_C_FSD:

        /* fsd share the same encoding layout with sd */

        imm = insn.sd.imm53 << 3 | insn.sd.imm76 << 6;
        ctx->dest = (uint8_t *)regs[REG_X8 + insn.sd.rs1] + imm;
        ctx->src  = (uint8_t *)&regs[REG_F8 + insn.sd.rs2];
        ctx->len  = 8;
        break;

      case INSN_C_FSDSP:

        /* fsdsp share the same encoding layout with sdsp */

        imm = insn.sdsp.imm53 << 3 | insn.sdsp.imm86 << 6;
        ctx->dest = (uint8_t *)regs[REG_SP] + imm;
        ctx->src  = (uint8_t *)&regs[REG_F0 + insn.sdsp.rs2];
        ctx->len  = 8;
        break;

#  endif
      default:
        _alert("Compressed: %x\n", insn.insn);
        return false;
    }

  regs[REG_EPC] += 2;

  return true;
#else
  return false;
#endif
}

/****************************************************************************
 * Name: decode_insn
 *
 * Description:
 *   Try to decode a normal encoding
 *
 ****************************************************************************/

static bool decode_insn(uintptr_t *regs, riscv_insn_ctx_t *ctx)
{
  static const uintptr_t x0;
  uint32_t in;
  int32_t imm;
  riscv_insn_t insn;

  /* Fetch instruction */

  memcpy(&insn.insn, (void *)regs[REG_EPC], 4);

  /* Get load/store instruction encoding */

  in = insn.insn & INSN_MASK;

  switch (in)
    {
      case INSN_LH:
      case INSN_LW:

        ctx->sext = true;

#ifdef CONFIG_ARCH_RV64
      case INSN_LD:
#endif

      case INSN_LHU:
      case INSN_LWU:

        /* Load a value from memory to register */

        ctx->dest = (uint8_t *)&regs[insn.l.rd];
        ctx->src = (uint8_t *)regs[insn.l.rs1] +
                   sext(insn.l.imm, 12);

        /* Zero the target register, no effect for sign extension */

        *(uintptr_t *)ctx->dest = 0;

        /* Get data wide bit */

        in &= ~0x4000;
        in >>= 12;
        ctx->len = 1 << in;

        break;

      case INSN_SH:
      case INSN_SW:

#ifdef CONFIG_ARCH_RV64
      case INSN_SD:
#endif
        /* Fetch signed imm */

        imm = sext(insn.s.imm2 | insn.s.imm1 << 5, 12);

        ctx->dest = (uint8_t *)regs[insn.s.rs1] + imm;

        /* If source register is x0, target it to constant register */

        if (insn.s.rs2 == 0)
          {
            ctx->src = (uint8_t *)&x0;
          }
        else
          {
            ctx->src = (uint8_t *)&regs[insn.s.rs2];
          }

        /* Get data wide bit */

        in &= ~0x4000;
        in >>= 12;
        ctx->len = 1 << in;

        break;

#ifdef CONFIG_ARCH_FPU
      case INSN_FLW:
      case INSN_FLD:

        ctx->dest = (uint8_t *)&regs[REG_F0 + insn.l.rd];
        ctx->src = (uint8_t *)regs[insn.l.rs1] +
                   sext(insn.l.imm, 12);

        /* Is instruction flw or fld ? */

        ctx->len = insn.l.funct3 == 0x2 ? 4 : 8;
        break;

      case INSN_FSW:
      case INSN_FSD:

        /* Fetch signed imm */

        imm = sext(insn.s.imm2 | insn.s.imm1 << 5, 12);

        ctx->dest = (uint8_t *)regs[insn.s.rs1] + imm;
        ctx->src = (uint8_t *)&regs[REG_F0 + insn.s.rs2];

        /* Is instruction fsw or fsd ? */

        ctx->len = insn.s.funct3 == 0x2 ? 4 : 8;

        break;
#endif
      default:
        _alert("Uncompressed: %x\n", insn.insn);
        return false;
    }

  /* Adjust EPC */

  regs[REG_EPC] += 4;

  return true;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: riscv_misaligned
 *
 * Description:
 *   This is software interrupt exception handler that handle load/store
 *   address misaligned exception.
 *
 * Input Parameters:
 *   regs - The current exception context
 *
 ****************************************************************************/

int riscv_misaligned(int irq, void *context, void *arg)
{
  bool ret;

  riscv_insn_ctx_t ctx =
    {
      NULL, NULL, 0, false
    };

  /* Try to decode compressed instruction if it is */

  ret = decode_insn_compressed(context, &ctx);

  /* Decode instruction context */

  if (ret == false)
    {
      if (decode_insn(context, &ctx) == false)
        {
          /* Decode failed, we can't handle a invalid instruction */

          PANIC();
        }
    }

  /* Byte copy */

  memcpy(ctx.dest, ctx.src, ctx.len);

  /* Do sign extension on need */

  if (ctx.sext)
    {
      /* Note: Only load instruction need sext, so we can ensure the dest
       * pointed to a register, and we do sext to the read out value of this
       * register context and then write back.
       */

      *(intptr_t *)ctx.dest = sext(*(intptr_t *)ctx.dest, ctx.len * 8);
    }

  return 0;
}
