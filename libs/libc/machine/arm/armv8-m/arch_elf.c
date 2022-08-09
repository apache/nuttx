/****************************************************************************
 * libs/libc/machine/arm/armv8-m/arch_elf.c
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

#include <inttypes.h>
#include <stdlib.h>
#include <errno.h>
#include <debug.h>

#include <arch/elf.h>
#include <nuttx/elf.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_checkarch
 *
 * Description:
 *   Given the ELF header in 'hdr', verify that the ELF file is appropriate
 *   for the current, configured architecture.  Every architecture that uses
 *   the ELF loader must provide this function.
 *
 * Input Parameters:
 *   hdr - The ELF header read from the ELF file.
 *
 * Returned Value:
 *   True if the architecture supports this ELF file.
 *
 ****************************************************************************/

bool up_checkarch(const Elf32_Ehdr *ehdr)
{
  /* Make sure it's an ARM executable */

  if (ehdr->e_machine != EM_ARM)
    {
      berr("ERROR: Not for ARM: e_machine=%04x\n", ehdr->e_machine);
      return false;
    }

  /* Make sure that 32-bit objects are supported */

  if (ehdr->e_ident[EI_CLASS] != ELFCLASS32)
    {
      berr("ERROR: Need 32-bit objects: e_ident[EI_CLASS]=%02x\n",
           ehdr->e_ident[EI_CLASS]);
      return false;
    }

  /* Verify endian-ness */

#ifdef CONFIG_ENDIAN_BIG
  if (ehdr->e_ident[EI_DATA] != ELFDATA2MSB)
#else
  if (ehdr->e_ident[EI_DATA] != ELFDATA2LSB)
#endif
    {
      berr("ERROR: Wrong endian-ness: e_ident[EI_DATA]=%02x\n",
           ehdr->e_ident[EI_DATA]);
      return false;
    }

  /* TODO:  Check ABI here. */

  return true;
}

/****************************************************************************
 * Name: up_relocate and up_relocateadd
 *
 * Description:
 *   Perform an architecture-specific ELF relocation.  Every architecture
 *   that uses the ELF loader must provide this function.
 *
 * Input Parameters:
 *   rel - The relocation type
 *   sym - The ELF symbol structure containing the fully resolved value.
 *         There are a few relocation types for a few architectures that do
 *         not require symbol information.  For those, this value will be
 *         NULL.  Implementations of these functions must be able to handle
 *         that case.
 *   addr - The address that requires the relocation.
 *
 * Returned Value:
 *   Zero (OK) if the relocation was successful.  Otherwise, a negated errno
 *   value indicating the cause of the relocation failure.
 *
 ****************************************************************************/

int up_relocate(const Elf32_Rel *rel, const Elf32_Sym *sym, uintptr_t addr)
{
  int32_t offset;
  uint32_t upper_insn;
  uint32_t lower_insn;
  unsigned int relotype;

  /* All relocations except R_ARM_V4BX depend upon having valid symbol
   * information.
   */

  relotype = ELF32_R_TYPE(rel->r_info);
  if (sym == NULL && relotype != R_ARM_NONE && relotype != R_ARM_V4BX)
    {
      return -EINVAL;
    }

  /* Handle the relocation by relocation type */

  switch (relotype)
    {
    case R_ARM_NONE:
      {
        /* No relocation */
      }
      break;

    case R_ARM_PC24:
    case R_ARM_CALL:
    case R_ARM_JUMP24:
      {
        binfo("Performing PC24 [%" PRId32 "] link at "
              "addr %08lx [%08lx] to sym '%p' st_value=%08lx\n",
              ELF32_R_TYPE(rel->r_info), (long)addr,
              (long)(*(uint32_t *)addr),
              sym, (long)sym->st_value);

        offset = (*(uint32_t *)addr & 0x00ffffff) << 2;
        if (offset & 0x02000000)
          {
            offset -= 0x04000000;
          }

        offset += sym->st_value - addr;
        if (offset & 3 || offset < (int32_t) 0xfe000000 ||
            offset >= (int32_t) 0x02000000)
          {
            berr("ERROR:   ERROR: PC24 [%" PRId32 "] "
                 "relocation out of range, offset=%08lx\n",
                 ELF32_R_TYPE(rel->r_info), offset);

            return -EINVAL;
          }

        offset >>= 2;

        *(uint32_t *)addr &= 0xff000000;
        *(uint32_t *)addr |= offset & 0x00ffffff;
      }
      break;

    case R_ARM_ABS32:
    case R_ARM_TARGET1:  /* New ABI:  TARGET1 always treated as ABS32 */
      {
        binfo("Performing ABS32 link "
              "at addr=%08lx [%08lx] to sym=%p st_value=%08lx\n",
              (long)addr, (long)(*(uint32_t *)addr),
              sym, (long)sym->st_value);

        *(uint32_t *)addr += sym->st_value;
      }
      break;

#ifdef CONFIG_ARMV7M_TARGET2_PREL
    case R_ARM_TARGET2:  /* TARGET2 is a platform-specific relocation: gcc-arm-none-eabi
                          * performs a self relocation */
      {
        binfo("Performing TARGET2 link "
              "at addr=%08lx [%08lx] to sym=%p st_value=%08lx\n",
              (long)addr, (long)(*(uint32_t *)addr),
              sym, (long)sym->st_value);

        *(uint32_t *)addr += sym->st_value - addr;
      }
      break;
#endif

    case R_ARM_THM_CALL:
    case R_ARM_THM_JUMP24:
      {
        uint32_t S;
        uint32_t J1;
        uint32_t J2;

        /* Thumb BL and B.W instructions. Encoding:
         *
         * upper_insn:
         *
         *  1  1  1  1  1  1
         *  5  4  3  2  1  0  9  8  7  6  5  4  3  2  1  0
         * +-------+---+----------------------+-----------+
         * |1  1  1|OP1|  OP2                 |           | 32Bit Instruction
         * +-------+---+-+---+----------------+-----------+
         * |1  1  1| 1  0| S |          imm10             | BL Instruction
         * +-------+-----+---+----------------------------+
         *
         * lower_insn:
         *
         *  1  1  1  1  1  1
         *  5  4  3  2  1  0  9  8  7  6  5  4  3  2  1  0
         * +--+-------------------------------------------+
         * |OP|                                           | 32Bit Instruction
         * +--+-+--+--+--+--------------------------------+
         * |1  1|J1| 1|J2|             imm11              | BL Instruction
         * +----+--+--+--+--------------------------------+
         *
         * The branch target is encoded in these bits:
         *
         *   S     = upper_insn[10]
         *   imm10 = upper_insn[0:9]
         *   imm11 = lower_insn[0:10]
         *   J1    = lower_insn[13]
         *   J2    = lower_insn[11]
         */

        upper_insn = (uint32_t)(*(uint16_t *)addr);
        lower_insn = (uint32_t)(*(uint16_t *)(addr + 2));

        binfo("Performing THM_JUMP24 [%" PRId32 "] link "
              "at addr=%08lx [%04x %04x] to sym=%p st_value=%08lx\n",
              ELF32_R_TYPE(rel->r_info), (long)addr,
              (int)upper_insn, (int)lower_insn,
              sym, (long)sym->st_value);

        /* Extract the 25-bit offset from the 32-bit instruction:
         *
         *   offset[24]    = S
         *   offset[23]    = ~(J1 ^ S)
         *   offset[22]    = ~(J2 ^ S)]
         *   offset[12:21] = imm10
         *   offset[1:11]  = imm11
         *   offset[0]     = 0
         */

        S   = (upper_insn >> 10) & 1;
        J1  = (lower_insn >> 13) & 1;
        J2  = (lower_insn >> 11) & 1;

        offset = (S << 24) |                       /* S -   > offset[24] */
                 ((~(J1 ^ S) & 1) << 23) |         /* J1    -> offset[23] */
                 ((~(J2 ^ S) & 1) << 22) |         /* J2    -> offset[22] */
                 ((upper_insn & 0x03ff) << 12) |   /* imm10 -> offset[12:21] */
                 ((lower_insn & 0x07ff) << 1);     /* imm11 -> offset[1:11] */
                                                   /* 0     -> offset[0] */

        /* Sign extend */

        if (offset & 0x01000000)
          {
            offset -= 0x02000000;
          }

        /* And perform the relocation */

        binfo("  S=%" PRId32 " J1=%" PRId32 " J2=%" PRId32
              " offset=%08" PRIx32 " branch target=%08lx\n",
              S, J1, J2, offset, offset + sym->st_value - addr);

        offset += sym->st_value - addr;

        /* Is this a function symbol?  If so, then the branch target must be
         * an odd Thumb address
         */

        if (ELF32_ST_TYPE(sym->st_info) == STT_FUNC && (offset & 1) == 0)
          {
            berr("ERROR:   ERROR: JUMP24 [%" PRId32 "] "
                 "requires odd offset, offset=%08lx\n",
                 ELF32_R_TYPE(rel->r_info), offset);

            return -EINVAL;
          }

        /* Check the range of the offset */

        if (offset < (int32_t)0xff000000 || offset >= (int32_t)0x01000000)
          {
            berr("ERROR:   ERROR: JUMP24 [%" PRId32 "] "
                 "relocation out of range, branch target=%08lx\n",
                 ELF32_R_TYPE(rel->r_info), offset);

            return -EINVAL;
          }

        /* Now, reconstruct the 32-bit instruction using the new, relocated
         * branch target.
         */

        S  = (offset >> 24) & 1;
        J1 = S ^ (~(offset >> 23) & 1);
        J2 = S ^ (~(offset >> 22) & 1);

        upper_insn = ((upper_insn & 0xf800) | (S << 10) |
                      ((offset >> 12) & 0x03ff));
        *(uint16_t *)addr = (uint16_t)upper_insn;

        lower_insn = ((lower_insn & 0xd000) | (J1 << 13) | (J2 << 11) |
                      ((offset >> 1) & 0x07ff));
        *(uint16_t *)(addr + 2) = (uint16_t)lower_insn;

        binfo("  S=%" PRId32 " J1=%" PRId32 " J2=%" PRId32
              " insn [%04" PRIx32 " %04" PRIx32 "]\n",
              S, J1, J2, upper_insn, lower_insn);
      }
      break;

    case R_ARM_V4BX:
      {
        binfo("Performing V4BX link at addr=%08lx [%08lx]\n",
              (long)addr, (long)(*(uint32_t *)addr));

         /* Preserve only Rm and the condition code */

        *(uint32_t *)addr &= 0xf000000f;

        /* Change instruction to 'mov pc, Rm' */

        *(uint32_t *)addr |= 0x01a0f000;
      }
      break;

    case R_ARM_PREL31:
      {
        binfo("Performing PREL31 link "
              "at addr=%08lx [%08lx] to sym=%p st_value=%08lx\n",
              (long)addr, (long)(*(uint32_t *)addr),
              sym, (long)sym->st_value);

        offset            = *(uint32_t *)addr + sym->st_value - addr;
        *(uint32_t *)addr = offset & 0x7fffffff;
      }
      break;

    case R_ARM_MOVW_ABS_NC:
    case R_ARM_MOVT_ABS:
      {
        binfo("Performing MOVx_ABS [%" PRId32 "] link "
              "at addr=%08lx [%08lx] to sym=%p st_value=%08lx\n",
              ELF32_R_TYPE(rel->r_info), (long)addr,
              (long)(*(uint32_t *)addr),
              sym, (long)sym->st_value);

        offset = *(uint32_t *)addr;
        offset = ((offset & 0xf0000) >> 4) | (offset & 0xfff);

        offset += sym->st_value;
        if (ELF32_R_TYPE(rel->r_info) == R_ARM_MOVT_ABS)
          {
            offset >>= 16;
          }

        *(uint32_t *)addr &= 0xfff0f000;
        *(uint32_t *)addr |= ((offset & 0xf000) << 4) | (offset & 0x0fff);
      }
      break;

    case R_ARM_THM_MOVW_ABS_NC:
    case R_ARM_THM_MOVT_ABS:
      {
        /* Thumb BL and B.W instructions. Encoding:
         *
         * upper_insn:
         *
         *  1  1  1  1  1  1
         *  5  4  3  2  1  0  9  8  7  6  5  4  3  2  1  0
         * +-------+---+-----------------------+----------+
         * |1  1  1|OP1|  OP2                  |          | 32Bit Instruction
         * +-------+---+-+---+-----------------+----------+
         * |1  1  1| 1  0| i | 1 0  1  1  0  0 |  imm4    | MOVT Instruction
         * +-------+-----+---+-----------------+----------+
         *
         * lower_insn:
         *
         *  1  1  1  1  1  1
         *  5  4  3  2  1  0  9  8  7  6  5  4  3  2  1  0
         * +--+-------------------------------------------+
         * |OP|                                           | 32Bit Instruction
         * +--+--------+----------+-----------------------+
         * |0 |  imm3  |    Rd    |        imm8           | MOVT Instruction
         * +--+--------+----------+-----------------------+
         *
         * The 16-bit immediate value is encoded in these bits:
         *
         *   i    = imm16[11]    = upper_insn[10]
         *   imm4 = imm16[12:15] = upper_insn[3:0]
         *   imm3 = imm16[8:10]  = lower_insn[14:12]
         *   imm8 = imm16[0:7]   = lower_insn[7:0]
         */

        upper_insn = (uint32_t)(*(uint16_t *)addr);
        lower_insn = (uint32_t)(*(uint16_t *)(addr + 2));

        binfo("Performing THM_MOVx [%" PRId32 "] link "
              "at addr=%08lx [%04x %04x] to sym=%p st_value=%08lx\n",
              ELF32_R_TYPE(rel->r_info), (long)addr,
              (int)upper_insn, (int)lower_insn,
              sym, (long)sym->st_value);

        /* Extract the 16-bit offset from the 32-bit instruction */

        offset = ((upper_insn & 0x000f) << 12) | /* imm4 -> imm16[8:10] */
                 ((upper_insn & 0x0400) << 1) |  /* i    -> imm16[11] */
                 ((lower_insn & 0x7000) >> 4) |  /* imm3 -> imm16[8:10] */
                  (lower_insn & 0x00ff);         /* imm8 -> imm16[0:7] */

        /* And perform the relocation */

        binfo("  offset=%08lx branch target=%08lx\n",
              (long)offset, offset + sym->st_value);

        offset += sym->st_value;

        /* Update the immediate value in the instruction.
         * For MOVW we want the bottom 16-bits; for MOVT we want
         * the top 16-bits.
         */

        if (ELF32_R_TYPE(rel->r_info) == R_ARM_THM_MOVT_ABS)
          {
            offset >>= 16;
          }

        upper_insn = ((upper_insn & 0xfbf0) | ((offset & 0xf000) >> 12) |
                      ((offset & 0x0800) >> 1));
        *(uint16_t *)addr = (uint16_t)upper_insn;

        lower_insn = ((lower_insn & 0x8f00) | ((offset & 0x0700) << 4) |
                      (offset & 0x00ff));
        *(uint16_t *)(addr + 2) = (uint16_t)lower_insn;

        binfo("  insn [%04x %04x]\n",
             (int)upper_insn, (int)lower_insn);
      }
      break;

    case R_ARM_THM_JUMP11:
      {
        offset = (uint32_t)(*(uint16_t *)addr & 0x7ff) << 1;
        if (offset & 0x0800)
          {
            offset -= 0x1000;
          }

        offset += sym->st_value - addr;

        if (ELF32_ST_TYPE(sym->st_info) == STT_FUNC && (offset & 1) == 0)
          {
            berr("ERROR: JUMP11 [%" PRId32 "] "
                 "requires odd offset, offset=%08lx\n",
                 ELF32_R_TYPE(rel->r_info), offset);

            return -EINVAL;
          }

        /* Check the range of the offset */

        if (offset < (int32_t)0xfffff800 || offset >= (int32_t)0x0800)
          {
            berr("ERROR: JUMP11 [%" PRId32 "] "
                 "relocation out of range, branch target=%08lx\n",
                 ELF32_R_TYPE(rel->r_info), offset);

            return -EINVAL;
          }

        offset >>= 1;

        *(uint16_t *)addr &= 0xf800;
        *(uint16_t *)addr |= offset & 0x7ff;
      }
      break;

    default:
      berr("ERROR: Unsupported relocation: %" PRId32 "\n",
           ELF32_R_TYPE(rel->r_info));
      return -EINVAL;
    }

  return OK;
}

int up_relocateadd(const Elf32_Rela *rel, const Elf32_Sym *sym,
                   uintptr_t addr)
{
  berr("ERROR: RELA relocation not supported\n");
  return -ENOSYS;
}
