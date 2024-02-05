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

#include <nuttx/elf.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define OPCODE_AUIPC    0x17
#define OPCODE_LUI      0x37

#define RVI_OPCODE_MASK 0x7F

/* ELF32 and ELF64 definitions */

#ifdef CONFIG_LIBC_ARCH_ELF_64BIT
#  define ARCH_ELF_TYP_STR "64"
#else /* !CONFIG_LIBC_ARCH_ELF_64BIT */
#  define ARCH_ELF_TYP_STR "32"
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
 * Name: _add_hi20
 *
 * Description:
 *   Add PCREL_HI20 relocation offset to the LUT. When a PCREL_LO12_I/_S is
 *   encountered, the corresponding PCREL_HI20 value can be found from it.
 *
 * Input Parameters:
 *   arch_data   - Where the PCREL_HI20 relocations are listed.
 *   hi20_rel    - The PCREL_HI20 relocation entry.
 *   hi20_offset - The corresponding offset value.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void _add_hi20(void *arch_data, uintptr_t hi20_rel,
                      uintptr_t hi20_offset)
{
  arch_elfdata_t *data = (arch_elfdata_t *)arch_data;
  int i;

  /* Try to find a free slot from the list */

  for (i = 0; i < ARCH_ELF_RELCNT; i++)
    {
      struct hi20_rels_s *hi20 = &data->hi20_rels[i];

      if (hi20->hi20_rel == 0)
        {
          hi20->hi20_rel = hi20_rel;
          hi20->hi20_offset = hi20_offset;
          break;
        }
    }
}

/****************************************************************************
 * Name: _find_hi20
 *
 * Description:
 *   Find PCREL_HI20 relocation offset from the LUT. When a PCREL_LO12_I/_S
 *   is encountered, the corresponding PCREL_HI20 value is needed to do the
 *   relocation.
 *
 * Input Parameters:
 *   arch_data   - Where the PCREL_HI20 relocations are listed.
 *   hi20_rel    - The PCREL_HI20 relocation entry.
 *
 * Returned Value:
 *   The corresponding hi20_offset value.
 *
 ****************************************************************************/

static uintptr_t _find_hi20(void *arch_data, uintptr_t hi20_rel)
{
  arch_elfdata_t *data = (arch_elfdata_t *)arch_data;
  int i;

  /* Try to find the hi20 value from the list */

  for (i = 0; i < ARCH_ELF_RELCNT; i++)
    {
      struct hi20_rels_s *hi20 = &data->hi20_rels[i];

      if (hi20->hi20_rel == hi20_rel)
        {
          /* Found it, we can clear the entry now */

          hi20->hi20_rel = 0;
          return hi20->hi20_offset;
        }
    }

  return 0;
}

/****************************************************************************
 * Name: _valid_hi20_imm
 *
 * Description:
 *   Check that any XX_HI20 relocation has a valid upper 20-bit immediate.
 *   Note that this test is not necessary for RV32 targets, the problem is
 *   related to RV64 sign extension.
 *
 * Input Parameters:
 *   imm_hi - The upper immediate value.
 *
 * Returned Value:
 *   true if imm_hi is valid; false otherwise
 *
 ****************************************************************************/

#ifdef CONFIG_LIBC_ARCH_ELF_64BIT
static inline bool _valid_hi20_imm(long imm_hi)
{
  /* 32-bit sign extend imm_hi and compare with the original value */

  long hi   = imm_hi & ((1 << 20) - 1);        /* 32-bit signed value */
  long sign = -((imm_hi >> 19) & 1);           /* 32-bit sign value */
  hi        = ((hi << 12) | sign << 32) >> 12; /* 32-bit sign extend */

  /* If the values do not match, the immediate is invalid */

  return imm_hi == hi;
}
#else
#  define _valid_hi20_imm(imm_hi) 1
#endif

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

  if (ehdr->e_ident[EI_CLASS] != ELF_CLASS)
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

int up_relocate(const Elf_Rel *rel, const Elf_Sym *sym, uintptr_t addr,
                void *arch_data)
{
  berr("Not implemented\n");
  return -ENOSYS;
}

int up_relocateadd(const Elf_Rela *rel, const Elf_Sym *sym,
                   uintptr_t addr, void *arch_data)
{
  long offset;
  unsigned int relotype;

  /* All relocations depend upon having valid symbol information */

  relotype = ELF_R_TYPE(rel->r_info);

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
        {
          long imm_hi;
          long imm_lo;

          binfo("%s at %08" PRIxPTR " [%08" PRIx32 "] "
                "to sym=%p st_value=%08lx\n",
                _get_rname(relotype),
                addr, _get_val((uint16_t *)addr),
                sym, sym->st_value);

          offset = _find_hi20(arch_data, sym->st_value);

          /* Adjust imm for MV(ADDI) / JR (JALR) : I-type */

          _calc_imm(offset, &imm_hi, &imm_lo);

          _add_val((uint16_t *)addr, (int32_t)imm_lo << 20);
        }
        break;

      case R_RISCV_PCREL_LO12_S:
        {
          uint32_t val;
          long imm_hi;
          long imm_lo;

          binfo("%s at %08" PRIxPTR " [%08" PRIx32 "] "
                "to sym=%p st_value=%08lx\n",
                _get_rname(relotype),
                addr, _get_val((uint16_t *)addr),
                sym, sym->st_value);

          offset = _find_hi20(arch_data, sym->st_value);

          /* Adjust imm for SW : S-type */

          _calc_imm(offset, &imm_hi, &imm_lo);

          val = (((int32_t)imm_lo >> 5) << 25) +
                (((int32_t)imm_lo & 0x1f) << 7);

          binfo("imm_lo=%ld (%lx), val=%" PRIx32 "\n", imm_lo, imm_lo, val);

          _add_val((uint16_t *)addr, val);
        }
        break;

      case R_RISCV_PCREL_HI20:
        {
          uint32_t insn;
          long imm_hi;
          long imm_lo;

          binfo("%s at %08" PRIxPTR " [%08" PRIx32 "] "
                "to sym=%p st_value=%08lx\n",
                _get_rname(relotype),
                addr, _get_val((uint16_t *)addr),
                sym, sym->st_value);

          offset = (long)sym->st_value + (long)rel->r_addend - (long)addr;

          insn = _get_val((uint16_t *)addr);
          ASSERT(OPCODE_AUIPC == (insn & RVI_OPCODE_MASK));

          _calc_imm(offset, &imm_hi, &imm_lo);

          if (!_valid_hi20_imm(imm_hi))
            {
              berr("ERROR: %s at %08" PRIxPTR " bad:%08lx\n",
                   _get_rname(relotype), addr, imm_hi << 12);

              return -EINVAL;
            }

          /* Adjust auipc (add upper immediate to pc) : 20bit */

          _add_val((uint16_t *)addr, imm_hi << 12);

          /* Add the hi20 value to the cache */

          _add_hi20(arch_data, addr, offset);
        }
        break;

      case R_RISCV_CALL:
      case R_RISCV_CALL_PLT:
        {
          long imm_hi;
          long imm_lo;

          binfo("%s at %08" PRIxPTR " [%08" PRIx32 "] "
                "to sym=%p st_value=%08lx\n",
                _get_rname(relotype),
                addr, _get_val((uint16_t *)addr),
                sym, sym->st_value);

          offset = (long)sym->st_value + (long)rel->r_addend - (long)addr;

          _calc_imm(offset, &imm_hi, &imm_lo);

          if (!_valid_hi20_imm(imm_hi))
            {
              berr("ERROR: %s at %08" PRIxPTR " bad:%08lx\n",
                   _get_rname(relotype), addr, imm_hi << 12);

              return -EINVAL;
            }

          /* Adjust auipc (add upper immediate to pc) : 20bit */

          _add_val((uint16_t *)addr, imm_hi << 12);

          /* Adjust imm for CALL (JALR) : I-type */

          _add_val((uint16_t *)(addr + 4), (int32_t)imm_lo << 20);
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

          if (!_valid_hi20_imm(imm_hi))
            {
              berr("ERROR: %s at %08" PRIxPTR " bad:%08lx\n",
                   _get_rname(relotype), addr, imm_hi << 12);

              return -EINVAL;
            }

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
      case R_RISCV_SUB16:
        {
          *(uint16_t *)addr -= (uint16_t)(sym->st_value + rel->r_addend);
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
      case R_RISCV_SET16:
        {
          *(uint16_t *)addr = (uint16_t)(sym->st_value + rel->r_addend);
        }
        break;
      default:
        berr("ERROR: Unsupported relocation: %ld\n",
             ELF_R_TYPE(rel->r_info));
        PANIC();
        return -EINVAL;
    }

  return OK;
}
