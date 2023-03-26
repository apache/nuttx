/****************************************************************************
 * libs/libc/machine/risc-v/arch_elf.c
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
#include <assert.h>

#include <arch/elf.h>
#include <nuttx/elf.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define OPCODE_SW       0x23
#define OPCODE_LUI      0x37

#define RVI_OPCODE_MASK 0x7F

/* ELF32 and ELF64 definitions */

#ifdef CONFIG_LIBC_ARCH_ELF_64BIT
#  define ARCH_ELF_TYP_STR    "64"
#  define ARCH_ELF_CLASS      ELFCLASS64
#  define ARCH_ELF_RELTYPE    ELF64_R_TYPE
#else /* !CONFIG_LIBC_ARCH_ELF_64BIT */
#  define ARCH_ELF_TYP_STR    "32"
#  define ARCH_ELF_CLASS      ELFCLASS32
#  define ARCH_ELF_RELTYPE    ELF32_R_TYPE
#endif /* CONFIG_LIBC_ARCH_ELF_64BIT */

/****************************************************************************
 * Private Data Types
 ****************************************************************************/

struct rname_code_s
{
  const char *name;
  int type;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct rname_code_s _rname_table[] =
{
  {"RELAX", R_RISCV_RELAX},
  {"RISCV_32", R_RISCV_32},
  {"RISCV_64", R_RISCV_64},
  {"PCREL_LO12_I", R_RISCV_PCREL_LO12_I},
  {"PCREL_LO12_S", R_RISCV_PCREL_LO12_S},
  {"PCREL_HI20", R_RISCV_PCREL_HI20},
  {"HI20", R_RISCV_HI20},
  {"LO12_I", R_RISCV_LO12_I},
  {"LO12_S", R_RISCV_LO12_S},
  {"CALL", R_RISCV_CALL},
  {"CALL_PLT", R_RISCV_CALL_PLT},
  {"BRANCH", R_RISCV_BRANCH},
  {"JAL", R_RISCV_JAL},
  {"RVC_JUMP", R_RISCV_RVC_JUMP},
  {"RVC_BRANCH", R_RISCV_RVC_BRANCH},
  {"32_PCREL", R_RISCV_32_PCREL},
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static const char *_get_rname(int type)
{
  int i = 0;

  for (i = 0; i < sizeof(_rname_table) / sizeof(struct rname_code_s); i++)
    {
      if (_rname_table[i].type == type)
        {
          return _rname_table[i].name;
        }
    }

  /* Not found in the table */

  return "?????";
}

/****************************************************************************
 * Name: _get_val, set_val, _add_val
 *
 * Description:
 *   These functions are used when relocating an instruction because we can
 *   not assume the instruction is word-aligned.
 *
 ****************************************************************************/

static uint32_t _get_val(uint16_t *addr)
{
  uint32_t ret;
  ret = *addr | (*(addr + 1)) << 16;
  return ret;
}

static void _set_val(uint16_t *addr, uint32_t val)
{
  *addr       = (val & 0xffff);
  *(addr + 1) = (val >> 16);

  /* NOTE: Ensure relocation before execution */

  asm volatile ("fence.i");
}

static void _add_val(uint16_t *addr, uint32_t val)
{
  uint32_t cur = _get_val(addr);
  _set_val(addr, cur + val);
}

/****************************************************************************
 * Name: _calc_imm
 *
 * Description:
 *   Given offset and obtain imm_hi (20bit) and imm_lo (12bit)
 *
 * Input Parameters:
 *   offset - signed 32bit
 *   imm_hi - signed 20bit
 *   imm_lo - signed 12bit
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void _calc_imm(long offset, long *imm_hi, long *imm_lo)
{
  long lo;
  long hi = offset / 4096;
  long r  = offset % 4096;

  if (2047 < r)
    {
      hi++;
    }
  else if (r < -2048)
    {
      hi--;
    }

  lo = offset - (hi * 4096);

  binfo("offset=%ld: hi=%ld lo=%ld\n",
        offset, hi, lo);

  ASSERT(-2048 <= lo && lo <= 2047);

  *imm_lo = lo;
  *imm_hi = hi;
}

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

bool up_checkarch(const Elf_Ehdr *ehdr)
{
  /* Make sure it's an RISCV executable */

  if (ehdr->e_machine != EM_RISCV)
    {
      berr("ERROR: Not for RISCV: e_machine=%04x\n", ehdr->e_machine);
      return false;
    }

  /* Make sure that current objects are supported */

  if (ehdr->e_ident[EI_CLASS] != ARCH_ELF_CLASS)
    {
      berr("ERROR: Need " ARCH_ELF_TYP_STR "-bit "
           "objects: e_ident[EI_CLASS]=%02x\n",
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

  /* Make sure the entry point address is properly aligned */

  if ((ehdr->e_entry & 1) != 0)
    {
      berr("ERROR: Entry point is not properly aligned: %08lx\n",
           ehdr->e_entry);
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

int up_relocate(const Elf_Rel *rel, const Elf_Sym *sym, uintptr_t addr)
{
  berr("Not implemented\n");
  return -ENOSYS;
}

int up_relocateadd(const Elf_Rela *rel, const Elf_Sym *sym,
                   uintptr_t addr)
{
  long offset;
  unsigned int relotype;

  /* All relocations depend upon having valid symbol information */

  relotype = ARCH_ELF_RELTYPE(rel->r_info);

  if (relotype == R_RISCV_RELAX)
    {
      /* NOTE: RELAX has no symbol, so just return */

      binfo("%s at %08" PRIxPTR " [%08" PRIx32 "]\n",
            _get_rname(relotype),
            addr, _get_val((uint16_t *)addr));

      return OK;
    }

  if (sym == NULL && relotype != R_RISCV_NONE)
    {
      return -EINVAL;
    }

  /* Do relocation based on relocation type */

  switch (relotype)
    {
      case R_RISCV_32:
      case R_RISCV_64:
        {
          binfo("%s at %08" PRIxPTR " [%08" PRIx32 "] "
                "to sym=%p st_value=%08lx\n",
                _get_rname(relotype),
                addr, _get_val((uint16_t *)addr),
                sym, sym->st_value);

          _set_val((uint16_t *)addr,
                   (uint32_t)(sym->st_value + rel->r_addend));
        }
        break;

      case R_RISCV_PCREL_LO12_I:
      case R_RISCV_PCREL_LO12_S:
        {
          binfo("%s at %08" PRIxPTR " [%08" PRIx32 "] "
                "to sym=%p st_value=%08lx\n",
                _get_rname(relotype),
                addr, _get_val((uint16_t *)addr),
                sym, sym->st_value);

          /* NOTE: imm value for mv has been adjusted in previous HI20 */
        }
        break;

      case R_RISCV_PCREL_HI20:
      case R_RISCV_CALL:
      case R_RISCV_CALL_PLT:
        {
          binfo("%s at %08" PRIxPTR " [%08" PRIx32 "] "
                "to sym=%p st_value=%08lx\n",
                _get_rname(relotype),
                addr, _get_val((uint16_t *)addr),
                sym, sym->st_value);

          offset = (long)sym->st_value + (long)rel->r_addend - (long)addr;

          long imm_hi;
          long imm_lo;

          _calc_imm(offset, &imm_hi, &imm_lo);

          /* Adjust auipc (add upper immediate to pc) : 20bit */

          _add_val((uint16_t *)addr, (imm_hi << 12));

          if ((_get_val((uint16_t *)(addr + 4)) & 0x7f) == OPCODE_SW)
            {
              /* Adjust imm for SW : S-type */

              uint32_t val =
                (((int32_t)imm_lo >> 5) << 25) +
                (((int32_t)imm_lo & 0x1f) << 7);

              binfo("imm_lo=%ld (%lx), val=%" PRIx32 "\n",
                    imm_lo, imm_lo, val);

              _add_val((uint16_t *)(addr + 4), val);
            }
          else
            {
              /* Adjust imm for MV(ADDI)/JALR : I-type */

              _add_val((uint16_t *)(addr + 4), ((int32_t)imm_lo << 20));
            }
        }
        break;

      case R_RISCV_BRANCH:
        {
          binfo("%s at %08" PRIxPTR " [%08" PRIx32 "] "
                "to sym=%p st_value=%08lx\n",
                _get_rname(relotype),
                addr, _get_val((uint16_t *)addr),
                sym, sym->st_value);

          /* P.23 Conditinal Branches : B type (imm=12bit) */

          offset = (long)sym->st_value + (long)rel->r_addend - (long)addr;
          uint32_t val = _get_val((uint16_t *)addr) & 0xfe000f80;

          /* NOTE: we assume that a compiler adds an immediate value */

          ASSERT(offset && val);

          binfo("offset for Bx=%ld (0x%lx) (val=0x%08" PRIx32 ") "
                "already set!\n",
                offset, offset, val);
        }
        break;

      case R_RISCV_JAL:
        {
          binfo("%s at %08" PRIxPTR " [%08" PRIx32 "] "
                "to sym=%p st_value=%08lx\n",
                _get_rname(relotype),
                addr, _get_val((uint16_t *)addr),
                sym, sym->st_value);

          /* P.21 Unconditinal Jumps : UJ type (imm=20bit) */

          offset = (long)sym->st_value + (long)rel->r_addend - (long)addr;
          uint32_t val = _get_val((uint16_t *)addr) & 0xfffff000;

          ASSERT(offset && val);

          /* NOTE: we assume that a compiler adds an immediate value */

          binfo("offset for JAL=%ld (0x%lx) (val=0x%08" PRIx32 ") "
                "already set!\n",
                offset, offset, val);
        }
        break;

      case R_RISCV_HI20:
        {
          binfo("%s at %08" PRIxPTR " [%08" PRIx32 "] "
                "to sym=%p st_value=%08lx\n",
                _get_rname(relotype),
                addr, _get_val((uint16_t *)addr),
                sym, sym->st_value);

          /* P.19 LUI */

          offset = (long)sym->st_value + (long)rel->r_addend;
          uint32_t insn = _get_val((uint16_t *)addr);

          ASSERT(OPCODE_LUI == (insn & RVI_OPCODE_MASK));

          long imm_hi;
          long imm_lo;
          _calc_imm(offset, &imm_hi, &imm_lo);
          insn = (insn & 0x00000fff) | (imm_hi << 12);

          _set_val((uint16_t *)addr, insn);
        }
        break;

      case R_RISCV_LO12_I:
        {
          binfo("%s at %08" PRIxPTR " [%08" PRIx32 "] "
                "to sym=%p st_value=%08lx\n",
                _get_rname(relotype),
                addr, _get_val((uint16_t *)addr),
                sym, sym->st_value);

          /* ADDI, FLW, LD, ... : I-type */

          offset = (long)sym->st_value + (long)rel->r_addend;
          uint32_t insn = _get_val((uint16_t *)addr);

          long imm_hi;
          long imm_lo;
          _calc_imm(offset, &imm_hi, &imm_lo);
          insn = (insn & 0x000fffff) | (imm_lo << 20);

          _set_val((uint16_t *)addr, insn);
        }
        break;

      case R_RISCV_LO12_S:
        {
          binfo("%s at %08" PRIxPTR " [%08" PRIx32 "] "
                "to sym=%p st_value=%08lx\n",
                _get_rname(relotype),
                addr, _get_val((uint16_t *)addr),
                sym, sym->st_value);

          /* SW : S-type.
           * not merge with R_RISCV_HI20 since the compiler
           * may not generates these two instructions continuously.
           */

          offset = (long)sym->st_value + (long)rel->r_addend;

          long imm_hi;
          long imm_lo;
          _calc_imm(offset, &imm_hi, &imm_lo);

          uint32_t val =
              (((int32_t)imm_lo >> 5) << 25) +
              (((int32_t)imm_lo & 0x1f) << 7);

          binfo("imm_lo=%ld (%lx), val=%" PRIx32 "\n", imm_lo, imm_lo, val);

          _add_val((uint16_t *)addr, val);
        }
        break;

      case R_RISCV_RVC_JUMP:
        {
          binfo("%s at %08" PRIxPTR " [%08" PRIx32 "] "
                "to sym=%p st_value=%08lx\n",
                _get_rname(relotype),
                addr, _get_val((uint16_t *)addr),
                sym, sym->st_value);

          /* P.111 Table 16.6 : Instruction listings for RVC */

          offset = (long)sym->st_value + (long)rel->r_addend - (long)addr;
          ASSERT(-2048 <= offset && offset <= 2047);

          uint16_t val = (*(uint16_t *)addr) & 0x1ffc;

          binfo("offset for C.J=%ld (0x%lx) (val=0x%04x) already set!\n",
                offset, offset, val);
        }
        break;

      case R_RISCV_RVC_BRANCH:
        {
          binfo("%s at %08" PRIxPTR " [%08" PRIx32 "] "
                "to sym=%p st_value=%08lx\n",
                _get_rname(relotype),
                addr, _get_val((uint16_t *)addr),
                sym, sym->st_value);

          /* P.111 Table 16.6 : Instruction listings for RVC */

          offset = (long)sym->st_value + (long)rel->r_addend - (long)addr;
          ASSERT(-256 <= offset && offset <= 255);

          uint16_t val = (*(uint16_t *)addr) & 0x1c7c;

          /* NOTE: we assume that a compiler adds an immediate value */

          ASSERT(offset && val);

          binfo("offset for C.Bx=%ld (0x%lx) (val=0x%04x) already set!\n",
                offset, offset, val);
        }
        break;
      case R_RISCV_32_PCREL:
        {
          /* P.29 https://github.com/riscv-non-isa/riscv-elf-psabi-doc */

          binfo("%s at %08" PRIxPTR " [%08" PRIx32 "] "
                "to sym=%p st_value=%08lx\n",
                _get_rname(relotype),
                addr, _get_val((uint16_t *)addr),
                sym, sym->st_value);

          addr = (long)sym->st_value + (long)rel->r_addend - (long)addr;
        }
        break;
      case R_RISCV_ADD32:
        {
          *(uint32_t *)addr += (uint32_t)(sym->st_value + rel->r_addend);
        }
        break;
      case R_RISCV_ADD64:
        {
          *(uint64_t *)addr += (uint64_t)(sym->st_value + rel->r_addend);
        }
        break;
      case R_RISCV_SUB32:
        {
          *(uint32_t *)addr -= (uint32_t)(sym->st_value + rel->r_addend);
        }
        break;
      case R_RISCV_SUB64:
        {
          *(uint64_t *)addr -= (uint64_t)(sym->st_value + rel->r_addend);
        }
        break;
      default:
        berr("ERROR: Unsupported relocation: %ld\n",
             ARCH_ELF_RELTYPE(rel->r_info));
        PANIC();
        return -EINVAL;
    }

  return OK;
}
