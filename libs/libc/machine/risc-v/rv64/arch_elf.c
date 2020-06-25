/****************************************************************************
 * libs/libc/machine/risc-v/rv64/arch_elf.c
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

#include <stdlib.h>
#include <errno.h>
#include <debug.h>
#include <assert.h>

#include <arch/elf.h>
#include <nuttx/elf.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define OPCODE_SW  0x23

/****************************************************************************
 * Private Data Types
 ****************************************************************************/

#ifdef CONFIG_DEBUG_BINFMT_INFO
struct rname_code_s
{
  const char *name;
  int type;
};
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_DEBUG_BINFMT_INFO
static struct rname_code_s _rname_table[] =
{
  {"RELAX", R_RISCV_RELAX},
  {"RISCV_64", R_RISCV_64},
  {"PCREL_LO12_I", R_RISCV_PCREL_LO12_I},
  {"PCREL_LO12_S", R_RISCV_PCREL_LO12_S},
  {"PCREL_HI20", R_RISCV_PCREL_HI20},
  {"CALL", R_RISCV_CALL},
  {"BRANCH", R_RISCV_BRANCH},
  {"RVC_JUMP", R_RISCV_RVC_JUMP},
  {"RVC_BRANCH", R_RISCV_RVC_BRANCH},
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_BINFMT_INFO
const char *_get_rname(int type)
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
#endif

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
 *   offset - signed 64bit
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

  binfo("offset=%ld: hi=%ld lo=%ld \n",
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

bool up_checkarch(FAR const Elf64_Ehdr *ehdr)
{
  /* Make sure it's an RISCV executable */

  if (ehdr->e_machine != EM_RISCV)
    {
      berr("ERROR: Not for RISCV: e_machine=%04x\n", ehdr->e_machine);
      return false;
    }

  /* Make sure that 64-bit objects are supported */

  if (ehdr->e_ident[EI_CLASS] != ELFCLASS64)
    {
      berr("ERROR: Need 64-bit objects: e_ident[EI_CLASS]=%02x\n",
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
      berr("ERROR: Entry point is not properly aligned: %08x\n",
           ehdr->e_entry);
    }

  /* TODO:  Check ABI here. */

  return true;
}

/****************************************************************************
 * Name: up_relocate and up_relocateadd
 *
 * Description:
 *   Perform on architecture-specific ELF relocation.  Every architecture
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

int up_relocate(FAR const Elf64_Rel *rel, FAR const Elf64_Sym *sym,
                uintptr_t addr)
{
  berr("Not implemented\n");
  return -ENOSYS;
}

int up_relocateadd(FAR const Elf64_Rela *rel, FAR const Elf64_Sym *sym,
                   uintptr_t addr)
{
  long offset;
  unsigned int relotype;

  /* All relocations depend upon having valid symbol information */

  relotype = ELF64_R_TYPE(rel->r_info);

  if (relotype == R_RISCV_RELAX)
    {
      /* NOTE: RELAX has no symbol, so just return */

      binfo("%s at %08lx [%08x] \n",
            _get_rname(relotype),
            (long)addr, _get_val((uint16_t *)addr));

      return OK;
    }

  if (sym == NULL && relotype != R_RISCV_NONE)
    {
      return -EINVAL;
    }

  /* Do relocation based on relocation type */

  switch (relotype)
    {
      case R_RISCV_64:
        {
          binfo("%s at %08lx [%08x] to sym=%p st_value=%08lx\n",
                _get_rname(relotype),
                (long)addr, _get_val((uint16_t *)addr),
                sym, (long)sym->st_value);

          _set_val((uint16_t *)addr,
                   (uint32_t)(sym->st_value + rel->r_addend));
        }
        break;

      case R_RISCV_PCREL_LO12_I:
      case R_RISCV_PCREL_LO12_S:
        {
          binfo("%s at %08lx [%08x] to sym=%p st_value=%08lx\n",
                _get_rname(relotype),
                (long)addr, _get_val((uint16_t *)addr),
                sym, (long)sym->st_value);

          /* NOTE: imm value for mv has been adjusted in previous HI20 */
        }
        break;

      case R_RISCV_PCREL_HI20:
      case R_RISCV_CALL:
        {
          binfo("%s at %08lx [%08x] to sym=%p st_value=%08lx\n",
                _get_rname(relotype),
                (long)addr, _get_val((uint16_t *)addr),
                sym, (long)sym->st_value);

          offset = (long)sym->st_value - (long)addr;

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

              binfo("imm_lo=%d (%x), val=%x \n", imm_lo, imm_lo, val);

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
          binfo("%s at %08lx [%08x] to sym=%p st_value=%08lx\n",
                _get_rname(relotype),
                (long)addr, _get_val((uint16_t *)addr),
                sym, (long)sym->st_value);

          /* P.23 Conditinal Branches : B type (imm=12bit) */

          offset = (long)sym->st_value - (long)addr;
          uint32_t val = _get_val((uint16_t *)addr) & 0xfe000f80;

          /* NOTE: we assume that a compiler adds an immediate value */

          ASSERT(offset && val);

          binfo("offset for Bx=%ld (0x%x) (val=0x%08x) already set! \n",
                offset, offset, val);
        }
        break;

      case R_RISCV_RVC_JUMP:
        {
          binfo("%s at %08lx [%04x] to sym=%p st_value=%08lx\n",
                _get_rname(relotype),
                (long)addr, _get_val((uint16_t *)addr),
                sym, (long)sym->st_value);

          /* P.111 Table 16.6 : Instruction listings for RVC */

          offset = ((long)sym->st_value - (long)addr);
          ASSERT(-2048 <= offset && offset <= 2047);

          uint16_t val = (*(uint16_t *)addr) & 0x1ffc;

          /* NOTE: we assume that a compiler adds an immediate value */

          ASSERT(offset && val);

          binfo("offset for C.J=%ld (0x%x) (val=0x%04x) already set! \n",
                offset, offset, val);
        }
        break;

      case R_RISCV_RVC_BRANCH:
        {
          binfo("%s at %08lx [%04x] to sym=%p st_value=%08lx\n",
                _get_rname(relotype),
                (long)addr, _get_val((uint16_t *)addr),
                sym, (long)sym->st_value);

          /* P.111 Table 16.6 : Instruction listings for RVC */

          offset = ((long)sym->st_value - (long)addr);
          ASSERT(-256 <= offset && offset <= 255);

          uint16_t val = (*(uint16_t *)addr) & 0x1c7c;

          /* NOTE: we assume that a compiler adds an immediate value */

          ASSERT(offset && val);

          binfo("offset for C.Bx=%ld (0x%x) (val=0x%04x) already set!\n",
                offset, offset, val);
        }
        break;

      default:
        berr("ERROR: Unsupported relocation: %d\n",
             ELF64_R_TYPE(rel->r_info));
        ASSERT(false);
        return -EINVAL;
    }

  return OK;
}
