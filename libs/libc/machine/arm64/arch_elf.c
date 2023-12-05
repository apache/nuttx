/****************************************************************************
 * libs/libc/machine/arm64/arch_elf.c
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
#include <endian.h>

#include <nuttx/compiler.h>
#include <nuttx/bits.h>
#include <nuttx/elf.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* For triggering a fault on purpose (reserved) */

#define FAULT_BRK_IMM           0x100

/* BRK instruction encoding
 * The #imm16 value should be placed at bits[20:5] within BRK ins
 */

#define AARCH64_BREAK_MON       0xd4200000

/* BRK instruction for provoking a fault on purpose
 * Unlike kgdb, #imm16 value with unallocated handler is used for faulting.
 */

#define AARCH64_BREAK_FAULT     (AARCH64_BREAK_MON | (FAULT_BRK_IMM << 5))

#define ADR_IMM_HILOSPLIT       2
#define ADR_IMM_SIZE            (2 * 1024 * 1024)
#define ADR_IMM_LOMASK          ((1 << ADR_IMM_HILOSPLIT) - 1)
#define ADR_IMM_HIMASK          ((ADR_IMM_SIZE >> ADR_IMM_HILOSPLIT) - 1)
#define ADR_IMM_LOSHIFT         29
#define ADR_IMM_HISHIFT         5

#define INSN_SF_BIT             BIT(31)
#define INSN_N_BIT              BIT(22)
#define INSN_LSL_12             BIT(22)

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum reloc_op_e
{
  RELOC_OP_NONE,
  RELOC_OP_ABS,
  RELOC_OP_PREL,
  RELOC_OP_PAGE,
};

enum insn_movw_imm_type_e
{
  INSN_IMM_MOVNZ,
  INSN_IMM_MOVKZ,
};

enum insn_imm_type_e
{
  INSN_IMM_ADR,
  INSN_IMM_26,
  INSN_IMM_19,
  INSN_IMM_16,
  INSN_IMM_14,
  INSN_IMM_12,
  INSN_IMM_N,
  INSN_IMM_MAX
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint32_t
aarch64_insn_encode_immediate(enum insn_imm_type_e type,
                              uint32_t insn, uint64_t imm)
{
  uint32_t immlo;
  uint32_t immhi;
  uint32_t mask;
  int shift;

  if (insn == AARCH64_BREAK_FAULT)
    {
      return AARCH64_BREAK_FAULT;
    }

  switch (type)
    {
      case INSN_IMM_ADR:
        {
          shift = 0;
          immlo = (imm & ADR_IMM_LOMASK) << ADR_IMM_LOSHIFT;
          imm >>= ADR_IMM_HILOSPLIT;
          immhi = (imm & ADR_IMM_HIMASK) << ADR_IMM_HISHIFT;
          imm = immlo | immhi;
          mask = (ADR_IMM_LOMASK << ADR_IMM_LOSHIFT) |
                 (ADR_IMM_HIMASK << ADR_IMM_HISHIFT);
        }
        break;

      case INSN_IMM_26:
        {
          mask = BIT(26) - 1;
          shift = 0;
        }
        break;

      case INSN_IMM_19:
        {
          mask = BIT(19) - 1;
          shift = 5;
        }
        break;

      case INSN_IMM_16:
        {
          mask = BIT(16) - 1;
          shift = 5;
        }
        break;

      case INSN_IMM_14:
        {
          mask = BIT(14) - 1;
          shift = 5;
        }
        break;

      case INSN_IMM_12:
        {
          mask = BIT(12) - 1;
          shift = 10;
        }
        break;

      default:
        {
          berr("unknown immediate encoding %d\n", type);

          return AARCH64_BREAK_FAULT;
        }
    }

  /* Update the immediate field. */

  insn &= ~(mask << shift);
  insn |= (imm & mask) << shift;

  return insn;
}

static uint64_t do_reloc(enum reloc_op_e op,
                         uintptr_t place, uint64_t val)
{
  switch (op)
    {
      case RELOC_OP_ABS:
        return val;
      case RELOC_OP_PREL:
        return val - (uint64_t)place;
      case RELOC_OP_PAGE:
        return (val & ~0xfff) - ((uint64_t)place & ~0xfff);
      case RELOC_OP_NONE:
        return 0;
    }

  return 0;
}

static int reloc_data(enum reloc_op_e op, uintptr_t place,
                      uint64_t val, int len)
{
  int64_t sval = do_reloc(op, place, val);

  /* The ELF psABI for AArch64 documents the 16-bit and 32-bit place
   * relative and absolute relocations as having a range of [-2^15, 2^16)
   * or [-2^31, 2^32), respectively. However, in order to be able to
   * detect overflows reliably, we have to choose whether we interpret
   * such quantities as signed or as unsigned, and stick with it.
   * The way we organize our address space requires a signed
   * interpretation of 32-bit relative references, so let's use that
   * for all R_AARCH64_PRELxx relocations. This means our upper
   * bound for overflow detection should be Sxx_MAX rather than Uxx_MAX.
   */

  switch (len)
    {
      case 16:
        {
          *(int16_t *)place = sval;
          switch (op)
            {
              case RELOC_OP_ABS:
                {
                  if (sval < 0 || sval > UINT16_MAX)
                    {
                      return -ERANGE;
                    }
                }
                break;

              case RELOC_OP_PREL:
                {
                  if (sval < INT16_MIN || sval > INT16_MAX)
                    {
                      return -ERANGE;
                    }
                }
                break;

              default:
                {
                  berr("Invalid 16-bit data relocation (%d)\n", op);
                  return -EINVAL;
                }
            }
        }
        break;

      case 32:
        {
          *(int32_t *)place = sval;
          switch (op)
            {
              case RELOC_OP_ABS:
                {
                  if (sval < 0 || sval > UINT32_MAX)
                    {
                      return -ERANGE;
                    }
                }
                break;

              case RELOC_OP_PREL:
                {
                  if (sval < INT32_MIN || sval > INT32_MAX)
                    {
                      return -ERANGE;
                    }
                }
                break;

              default:
                {
                  berr("Invalid 32-bit data relocation (%d)\n", op);
                  return -EINVAL;
                }
            }
        }
        break;

      case 64:
        {
          *(int64_t *)place = sval;
        }
        break;

      default:
        {
          berr("Invalid length (%d) for data relocation\n", len);
          return -EINVAL;
        }
    }

  return 0;
}

static int reloc_insn_movw(enum reloc_op_e op, uintptr_t place,
                           uint64_t val, int lsb,
                           enum insn_movw_imm_type_e imm_type)
{
  uint32_t insn = htole32(*(uint32_t *)place);
  uint64_t imm;
  int64_t sval;

  sval = do_reloc(op, place, val);
  imm = sval >> lsb;

  if (imm_type == INSN_IMM_MOVNZ)
    {
      /* For signed MOVW relocations, we have to manipulate the
       * instruction encoding depending on whether or not the
       * immediate is less than zero.
       */

      insn &= ~(3 << 29);
      if (sval >= 0)
        {
          /* >=0: Set the instruction to MOVZ (opcode 10b). */

          insn |= 2 << 29;
        }
      else
        {
          /* <0: Set the instruction to MOVN (opcode 00b).
           *     Since we've masked the opcode already, we
           *     don't need to do anything other than
           *     inverting the new immediate field.
           */

          imm = ~imm;
        }
    }

  /* Update the instruction with the new encoding. */

  insn = aarch64_insn_encode_immediate(INSN_IMM_16, insn, imm);
  *(uint32_t *)place = le32toh(insn);

  if (imm > UINT16_MAX)
    {
      return -ERANGE;
    }

  return 0;
}

static int reloc_insn_imm(enum reloc_op_e op, uintptr_t place,
                          uint64_t val, int lsb, int len,
                          enum insn_imm_type_e imm_type)
{
  int64_t sval;
  uint64_t imm;
  uint64_t imm_mask;
  uint32_t insn = le32toh(*(uint32_t *)place);

  /* Calculate the relocation value. */

  sval = do_reloc(op, place, val);
  sval >>= lsb;

  /* Extract the value bits and shift them to bit 0. */

  imm_mask = (BIT(lsb + len) - 1) >> lsb;
  imm = sval & imm_mask;

  /* Update the instruction's immediate field. */

  insn = aarch64_insn_encode_immediate(imm_type, insn, imm);
  *(uint32_t *)place = htole32(insn);

  /* Extract the upper value bits (including the sign bit) and
   * shift them to bit 0.
   */

  sval = (int64_t)(sval & ~(imm_mask >> 1)) >> (len - 1);

  /* Overflow has occurred if the upper bits are not all equal to
   * the sign bit of the value.
   */

  if ((uint64_t)(sval + 1) >= 2)
    {
      return -ERANGE;
    }

  return 0;
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

bool up_checkarch(const Elf64_Ehdr *ehdr)
{
  /* Make sure it's an ARM executable */

  if (ehdr->e_machine != EM_AARCH64)
    {
      berr("ERROR: Not for AARCH64: e_machine=%04x\n", ehdr->e_machine);
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

int up_relocate(const Elf64_Rel *rel, const Elf64_Sym *sym, uintptr_t addr,
                void *arch_data)
{
  berr("ERROR: REL relocation not supported\n");
  return -ENOSYS;
}

int up_relocateadd(const Elf64_Rela *rel, const Elf64_Sym *sym,
                   uintptr_t addr, void *arch_data)
{
  bool overflow_check = true;
  uint64_t val;
  int ret = 0;

  /* addr corresponds to P in the AArch64 ELF document. */

  /* val corresponds to (S + A) in the AArch64 ELF document. */

  val = sym->st_value + rel->r_addend;

  /* Handle the relocation by relocation type */

  switch (ELF64_R_TYPE(rel->r_info))
    {
      case R_AARCH64_NONE:
        {
          /* No relocation */
        }
        break;

      /* Data relocations */

      case R_AARCH64_ABS64:
        {
          overflow_check = false;
          ret = reloc_data(RELOC_OP_ABS, addr, val, 64);
        }
        break;

      case R_AARCH64_ABS32:
        {
          ret = reloc_data(RELOC_OP_ABS, addr, val, 32);
        }
        break;

      case R_AARCH64_ABS16:
        {
          ret = reloc_data(RELOC_OP_ABS, addr, val, 16);
        }
        break;

      case R_AARCH64_PREL64:
        {
          overflow_check = false;
          ret = reloc_data(RELOC_OP_PREL, addr, val, 64);
        }
        break;

      case R_AARCH64_PREL32:
        {
          ret = reloc_data(RELOC_OP_PREL, addr, val, 32);
        }
        break;

      case R_AARCH64_PREL16:
        {
          ret = reloc_data(RELOC_OP_PREL, addr, val, 16);
        }
        break;

      case R_AARCH64_MOVW_UABS_G0_NC:
        {
          overflow_check = false;
        }

        /* fallthrough */

      case R_AARCH64_MOVW_UABS_G0:
        {
          ret = reloc_insn_movw(RELOC_OP_ABS, addr, val, 0,
                                INSN_IMM_MOVKZ);
        }
        break;

      case R_AARCH64_MOVW_UABS_G1_NC:
        {
          overflow_check = false;
        }

        /* fallthrough */

      case R_AARCH64_MOVW_UABS_G1:
        {
          ret = reloc_insn_movw(RELOC_OP_ABS, addr, val, 16,
                                INSN_IMM_MOVKZ);
        }
        break;

      case R_AARCH64_MOVW_UABS_G2_NC:
        {
          overflow_check = false;
        }

        /* fallthrough */

      case R_AARCH64_MOVW_UABS_G2:
        {
          ret = reloc_insn_movw(RELOC_OP_ABS, addr, val, 32,
                                INSN_IMM_MOVKZ);
        }
        break;

      case R_AARCH64_MOVW_UABS_G3:
        {
          /* We're using the top bits so we can't overflow. */

          overflow_check = false;
          ret = reloc_insn_movw(RELOC_OP_ABS, addr, val, 48,
                                INSN_IMM_MOVKZ);
        }
        break;

      case R_AARCH64_MOVW_SABS_G0:
        {
          ret = reloc_insn_movw(RELOC_OP_ABS, addr, val, 0,
                                INSN_IMM_MOVNZ);
        }
        break;

      case R_AARCH64_MOVW_SABS_G1:
        {
          ret = reloc_insn_movw(RELOC_OP_ABS, addr, val, 16,
                                INSN_IMM_MOVNZ);
        }
        break;

      case R_AARCH64_MOVW_SABS_G2:
        {
          ret = reloc_insn_movw(RELOC_OP_ABS, addr, val, 32,
                                INSN_IMM_MOVNZ);
        }
        break;

      case R_AARCH64_MOVW_PREL_G0_NC:
        {
          overflow_check = false;
          ret = reloc_insn_movw(RELOC_OP_PREL, addr, val, 0,
                                INSN_IMM_MOVKZ);
        }
        break;

      case R_AARCH64_MOVW_PREL_G0:
        {
          ret = reloc_insn_movw(RELOC_OP_PREL, addr, val, 0,
                                INSN_IMM_MOVNZ);
        }
        break;

      case R_AARCH64_MOVW_PREL_G1_NC:
        {
          overflow_check = false;
          ret = reloc_insn_movw(RELOC_OP_PREL, addr, val, 16,
                                INSN_IMM_MOVKZ);
        }
        break;

      case R_AARCH64_MOVW_PREL_G1:
        {
          ret = reloc_insn_movw(RELOC_OP_PREL, addr, val, 16,
                                INSN_IMM_MOVNZ);
        }
        break;

      case R_AARCH64_MOVW_PREL_G2_NC:
        {
          overflow_check = false;
          ret = reloc_insn_movw(RELOC_OP_PREL, addr, val, 32,
                                INSN_IMM_MOVKZ);
        }
        break;

      case R_AARCH64_MOVW_PREL_G2:
        {
          ret = reloc_insn_movw(RELOC_OP_PREL, addr, val, 32,
                                INSN_IMM_MOVNZ);
        }
        break;

      case R_AARCH64_MOVW_PREL_G3:
        {
          /* We're using the top bits so we can't overflow. */

          overflow_check = false;
          ret = reloc_insn_movw(RELOC_OP_PREL, addr, val, 48,
                                INSN_IMM_MOVNZ);
        }
        break;

      /* Immediate instruction relocations. */

      case R_AARCH64_LD_PREL_LO19:
        {
          ret = reloc_insn_imm(RELOC_OP_PREL, addr, val, 2, 19,
                               INSN_IMM_19);
        }
        break;

      case R_AARCH64_ADR_PREL_LO21:
        {
          ret = reloc_insn_imm(RELOC_OP_PREL, addr, val, 0, 21,
                               INSN_IMM_ADR);
        }
        break;

      case R_AARCH64_ADR_PREL_PG_HI21_NC:
        {
          overflow_check = false;
        }

        /* fallthrough */

      case R_AARCH64_ADR_PREL_PG_HI21:
        {
          if (((uint64_t)addr & 0xfff) < 0xff8)
            {
              ret = reloc_insn_imm(RELOC_OP_PAGE, addr, val, 12, 21,
                                   INSN_IMM_ADR);
            }
          else
            {
              uint32_t insn;

              /* patch ADRP to ADR if it is in range */

              ret = reloc_insn_imm(RELOC_OP_PREL, addr, val & ~0xfff, 0, 21,
                                   INSN_IMM_ADR);
              if (ret == 0)
                {
                  insn = le32toh(*(uint32_t *)addr);
                  insn &= ~BIT(31);
                  *(uint32_t *)addr = htole32(insn);
                }
              else
                {
                  berr("Out of range for ADR\n");
                  return -EINVAL;
                }
            }
        }
        break;

      case R_AARCH64_ADD_ABS_LO12_NC:
      case R_AARCH64_LDST8_ABS_LO12_NC:
        {
          overflow_check = false;
          ret = reloc_insn_imm(RELOC_OP_ABS, addr, val, 0, 12,
                               INSN_IMM_12);
        }
        break;

      case R_AARCH64_LDST16_ABS_LO12_NC:
        {
          overflow_check = false;
          ret = reloc_insn_imm(RELOC_OP_ABS, addr, val, 1, 11,
                               INSN_IMM_12);
        }
        break;

      case R_AARCH64_LDST32_ABS_LO12_NC:
        {
          overflow_check = false;
          ret = reloc_insn_imm(RELOC_OP_ABS, addr, val, 2, 10,
                               INSN_IMM_12);
        }
        break;

      case R_AARCH64_LDST64_ABS_LO12_NC:
        {
          overflow_check = false;
          ret = reloc_insn_imm(RELOC_OP_ABS, addr, val, 3, 9,
                               INSN_IMM_12);
        }
        break;

      case R_AARCH64_LDST128_ABS_LO12_NC:
        {
          overflow_check = false;
          ret = reloc_insn_imm(RELOC_OP_ABS, addr, val, 4, 8,
                               INSN_IMM_12);
        }
        break;

      case R_AARCH64_TSTBR14:
        {
          ret = reloc_insn_imm(RELOC_OP_PREL, addr, val, 2, 14,
                               INSN_IMM_14);
        }
        break;

      case R_AARCH64_CONDBR19:
        {
          ret = reloc_insn_imm(RELOC_OP_PREL, addr, val, 2, 19,
                               INSN_IMM_19);
        }
        break;

      case R_AARCH64_JUMP26:
      case R_AARCH64_CALL26:
        {
          ret = reloc_insn_imm(RELOC_OP_PREL, addr, val, 2, 26,
                               INSN_IMM_26);
        }
        break;

      default:
        berr("ERROR: Unsupported relocation: %"PRIu64"\n",
             ELF64_R_TYPE(rel->r_info));
        return -EINVAL;
    }

  if (overflow_check && ret == -ERANGE)
    {
      goto overflow;
    }

  return OK;

overflow:
  berr("ERROR: overflow in relocation type %"PRIu64" val %"PRIu64"\n",
       ELF64_R_TYPE(rel->r_info), val);
  return -ENOEXEC;
}
