/****************************************************************************
 * tools/nxflat/ldnxflat.c
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

/****************************************************************************
 * ldnxflat converts the ELF object that
 *
 *   ld -r -d -T binfmt/libnxflat/gnu-nxflat-gotoff.ld -no-check-sections
 *
 * produces into the NXFLAT container that binfmt/libnxflat loads.
 *
 * The file it writes:
 *
 *   struct nxflat_hdr_s                     network byte order
 *   the I-Space image                       h_entry counts from the file
 *   the D-Space image, GOT first            offsets count from its own start
 *   struct nxflat_reloc_s[]                 target byte order
 *
 * I-Space is mapped from the start of the file, so h_entry is biased by
 * the header's size.  An I-Space value left in D-Space is not:
 * nxflat_bindrel32i() adds ispace + sizeof(struct nxflat_hdr_s) itself.
 *
 * A call within I-Space and a GOT-relative reference are resolved here.
 * The rest name an address the loader only knows once it has placed the
 * segments, and become records:  REL32I for a D-Space word holding an
 * I-Space address, REL32D for one holding a D-Space address.  Every GOT
 * entry is a REL32D, which is what makes the GOT work.
 *
 * Provenance.  Written from include/nxflat.h, from what binfmt/libnxflat
 * does with the container, and from gnu-nxflat-gotoff.ld; the relocation
 * arithmetic is that of libs/libc/machine/arm/armv7-m/arch_elf.c, which the
 * ELF loader runs on the target for the same relocations.  Those files and
 * this one are Apache-2.0.
 *
 * The tool of the same name in buildroot is GPL by descent from elf2flt and
 * no part of it is used.  It was run on the same inputs to compare output,
 * which is how the conventions the container does not state -- where the GOT
 * sits, the order of its entries and of the records -- were matched.  Two of
 * its results are deliberately not matched, because they are wrong:  a GOT
 * entry naming a .bss object, and the alignment gap before .bss in h_bssend.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "nxflat_elf.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The container, field for field as include/nxflat.h has it.  That header is
 * not included because it reaches for nuttx/config.h.
 */

#define NXFLAT_MAGIC            "NxFT"
#define NXFLAT_HDR_SIZE         36
#define NXFLAT_RELOC_TYPE_REL32I 0
#define NXFLAT_RELOC_TYPE_REL32D 1
#define NXFLAT_RELOC(t, o)      (((uint32_t)((t) & 3) << 30) | ((o) & 0x3fffffff))

#define DEFAULT_STACK_SIZE      4096

/* The relocations a module carries, computed as arch_elf.c computes them. */

#define R_ARM_ABS32             2
#define R_ARM_REL32             3
#define R_ARM_THM_CALL          10
#define R_ARM_GOTOFF32          24
#define R_ARM_GOT_BREL          26
#define R_ARM_PLT32             27
#define R_ARM_CALL              28
#define R_ARM_JUMP24            29
#define R_ARM_TARGET1           38
#define R_ARM_THM_JUMP24        30
#define R_ARM_PC24              1

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Which segment a section, a symbol or a relocation belongs to. */

enum segment_e
{
  SEG_NONE = 0,
  SEG_TEXT,
  SEG_DATA
};

struct segment_s
{
  uint8_t *img;                /* The image being built */
  uint32_t size;               /* Bytes in it */
};

/* One relocation, as the architecture is handed it.  Addresses count from
 * the start of their segment, which is what the loader adds its bases to.
 */

struct reloc_s
{
  uint32_t type;               /* ELF32_R_TYPE(r_info) */
  uint32_t place;              /* Where the word is */
  enum segment_e pseg;         /* ...and in which segment */
  uint32_t value;              /* Where the symbol resolved to */
  enum segment_e vseg;         /* ...and in which segment */
  uint32_t gotoffset;          /* Its GOT entry, if the type takes one */
  bool isfunc;                 /* The symbol is a function */
  const char *symname;         /* For the diagnostics */
  uint8_t *p;                  /* The word itself */
};

/* What the architecture did with it */

enum reloc_action_e
{
  RELOC_DONE = 0,              /* Resolved; the loader has nothing to do */
  RELOC_RECORD                 /* The word holds an address: record it */
};

/* An architecture supplies what depends on the instruction set: which
 * relocations exist, what they compute, and which reach through the GOT.
 * The rest of this file is common, because a REL32I or REL32D fixup is the
 * addition of a base to a 32-bit word.
 */

struct nxflat_arch_s
{
  const char *name;
  uint16_t machine;                            /* e_machine selects it */
  uint32_t (*entry)(uint32_t value);           /* The entry point address */
  bool (*is_gotref)(uint32_t type);            /* Type needs a GOT entry */
  enum reloc_action_e (*reloc)(struct reloc_s *r);
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void fail(const char *fmt, ...);
static void select_arch(uint16_t machine);
static const char *sym_name(const struct elf32_sym_s *sym);
static uint32_t arm_entry(uint32_t value);
static bool arm_is_gotref(uint32_t type);
static enum reloc_action_e arm_reloc(struct reloc_s *r);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char *g_program;
static const char *g_elf_filename;
static const char *g_out_filename;
static const char *g_entry_name;
static uint32_t    g_stacksize = DEFAULT_STACK_SIZE;
static int         g_verbose;

static uint8_t *g_elf;               /* The whole input file */
static struct elf32_ehdr_s   *g_ehdr;
static struct elf32_shdr_s   *g_shdr;
static struct elf32_sym_s    *g_syms;
static size_t                 g_nsyms;
static const char            *g_strtab;
static const char            *g_shstrtab;
static enum segment_e        *g_segof;   /* Per section */

static const struct nxflat_arch_s *g_arch;

static struct segment_s g_text;
static struct segment_s g_data;
static uint32_t g_bsssize;
static uint32_t g_gotsize;

/* ld -r leaves a tentative definition ("int counter;") SHN_COMMON, with its
 * alignment in st_value and its size in st_size.  Nothing has placed it, so
 * this does, at the end of the bss.  Zero means the symbol is not one.
 */

static uint32_t *g_commonaddr;

/* One GOT entry per symbol that a GOT-relative reference names, in the order
 * the relocations name them.
 */

struct gotent_s
{
  uint32_t symidx;
  uint32_t value;              /* The address the entry will hold */
  enum segment_e seg;          /* Which segment that address is in */
};

static struct gotent_s *g_got;
static uint32_t g_ngot;

/* The relocation records for the loader */

static uint32_t *g_relocs;
static uint32_t  g_nrelocs;

/* The architectures this tool knows.  NXFLAT is tied to none of them, so
 * another one is a table entry and a handler.
 */

static const struct nxflat_arch_s g_arches[] =
{
  {
    "arm",                        /* name */
    EM_ARM,                       /* machine */
    arm_entry,                    /* entry */
    arm_is_gotref,                /* is_gotref */
    arm_reloc                     /* reloc */
  }
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void fail(const char *fmt, ...)
{
  va_list ap;

  fprintf(stderr, "%s: ", g_program);
  va_start(ap, fmt);
  vfprintf(stderr, fmt, ap);
  va_end(ap);
  fprintf(stderr, "\n");
  exit(2);
}

/****************************************************************************
 * Name: read_elf
 *
 * Description:
 *   Read the object and check that it is one this tool can convert.
 *
 ****************************************************************************/

static void read_elf(void)
{
  struct nxflat_elf_s elf;

  nxflat_elf_read(&elf, g_elf_filename, g_program);

  g_elf      = elf.img;
  g_ehdr     = elf.ehdr;
  g_shdr     = elf.shdr;
  g_syms     = elf.syms;
  g_nsyms    = elf.nsyms;
  g_strtab   = elf.strtab;
  g_shstrtab = elf.shstrtab;

  if (g_ehdr->e_type != ET_REL)
    {
      fail("%s is not a relocatable object; it must be linked with ld -r",
           g_elf_filename);
    }

  /* The images are written out as they were read, so a big-endian target
   * would need every patched word swapped.  None exists; refuse it rather
   * than write a wrong file.
   */

  if (!elf.littleendian)
    {
      fail("%s is big endian, which this tool cannot convert",
           g_elf_filename);
    }

  select_arch(g_ehdr->e_machine);
}

/****************************************************************************
 * Name: place_sections
 *
 * Description:
 *   Sort the allocated sections into the two segments and size them.  The
 *   linker script has placed each one within its segment already, both
 *   segments starting at zero -- hence -no-check-sections on the link.
 *
 *   Executable is I-Space; everything else allocated is D-Space, read-only
 *   data included, because the model reaches that through the GOT.
 *
 *   ARM unwind tables are left out.  Their entries are PC-relative from
 *   D-Space into I-Space, which the container cannot express, and nothing
 *   unwinds through a module.
 *
 ****************************************************************************/

static void place_sections(void)
{
  uint32_t dataend = 0;
  uint32_t bssend  = 0;
  size_t i;

  g_segof = calloc(g_ehdr->e_shnum, sizeof(*g_segof));
  if (g_segof == NULL)
    {
      fail("out of memory");
    }

  for (i = 1; i < g_ehdr->e_shnum; i++)
    {
      struct elf32_shdr_s *s = &g_shdr[i];
      uint32_t end = s->sh_addr + s->sh_size;

      if ((s->sh_flags & SHF_ALLOC) == 0 ||
          strncmp(g_shstrtab + s->sh_name, ".ARM.exidx", 10) == 0 ||
          strncmp(g_shstrtab + s->sh_name, ".ARM.extab", 10) == 0)
        {
          continue;
        }

      if ((s->sh_flags & SHF_EXECINSTR) != 0)
        {
          g_segof[i] = SEG_TEXT;
          if (end > g_text.size)
            {
              g_text.size = end;
            }
        }
      else
        {
          g_segof[i] = SEG_DATA;

          if (s->sh_type == SHT_NOBITS)
            {
              if (end > bssend)
                {
                  bssend = end;
                }
            }
          else if (end > dataend)
            {
              dataend = end;
            }
        }
    }

  if (bssend < dataend)
    {
      bssend = dataend;
    }

  g_data.size = dataend;
  g_bsssize   = bssend - dataend;
}

/****************************************************************************
 * Name: sym_segment and sym_value
 *
 * Description:
 *   Where a symbol ends up.  D-Space starts with the GOT, so everything the
 *   linker put there moves up by its size.
 *
 ****************************************************************************/

static enum segment_e sym_segment(const struct elf32_sym_s *sym)
{
  if (sym->st_shndx == SHN_COMMON)
    {
      return SEG_DATA;
    }

  if (sym->st_shndx == SHN_UNDEF || sym->st_shndx >= g_ehdr->e_shnum)
    {
      return SEG_NONE;
    }

  return g_segof[sym->st_shndx];
}

static uint32_t sym_value(const struct elf32_sym_s *sym)
{
  uint32_t value;

  if (sym->st_shndx == SHN_COMMON)
    {
      /* Placed by allocate_common(), which counts from the same zero the
       * linker gave the sections.
       */

      value = g_commonaddr[sym - g_syms];
    }
  else
    {
      value = sym->st_value;

      if (sym->st_shndx < g_ehdr->e_shnum)
        {
          value += g_shdr[sym->st_shndx].sh_addr;
        }
    }

  if (sym_segment(sym) == SEG_DATA)
    {
      value += g_gotsize;
    }

  return value;
}

/****************************************************************************
 * Name: allocate_common
 *
 * Description:
 *   Place every SHN_COMMON symbol at the end of the bss, in symbol table
 *   order, at the alignment it asks for.
 *
 ****************************************************************************/

static void allocate_common(void)
{
  uint32_t next = g_data.size + g_bsssize;
  size_t i;

  g_commonaddr = calloc(g_nsyms, sizeof(*g_commonaddr));
  if (g_commonaddr == NULL)
    {
      fail("out of memory");
    }

  for (i = 0; i < g_nsyms; i++)
    {
      uint32_t align = g_syms[i].st_value;

      if (g_syms[i].st_shndx != SHN_COMMON)
        {
          continue;
        }

      if (align > 1)
        {
          next = (next + align - 1) & ~(align - 1);
        }

      g_commonaddr[i] = next;
      next += g_syms[i].st_size;

      if (g_verbose > 1)
        {
          printf("  common %s at D-Space %08x, %u bytes\n",
                 sym_name(&g_syms[i]), next - g_syms[i].st_size,
                 g_syms[i].st_size);
        }
    }

  g_bsssize = next - g_data.size;
}

static const char *sym_name(const struct elf32_sym_s *sym)
{
  return g_strtab + sym->st_name;
}

/****************************************************************************
 * Name: find_symbol
 ****************************************************************************/

static const struct elf32_sym_s *find_symbol(const char *name)
{
  size_t i;

  for (i = 0; i < g_nsyms; i++)
    {
      if (g_syms[i].st_name != 0 &&
          g_syms[i].st_shndx != SHN_UNDEF &&
          strcmp(sym_name(&g_syms[i]), name) == 0)
        {
          return &g_syms[i];
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: allocate_got
 *
 * Description:
 *   One entry per symbol a GOT-relative reference names, in the order they
 *   are named.  Sized before anything is placed, because the GOT sits at the
 *   start of D-Space and moves the rest up.
 *
 ****************************************************************************/

static uint32_t got_index(uint32_t symidx)
{
  uint32_t i;

  for (i = 0; i < g_ngot; i++)
    {
      if (g_got[i].symidx == symidx)
        {
          return i;
        }
    }

  g_got = realloc(g_got, (g_ngot + 1) * sizeof(*g_got));
  if (g_got == NULL)
    {
      fail("out of memory");
    }

  g_got[g_ngot].symidx = symidx;
  g_got[g_ngot].value  = 0;
  g_got[g_ngot].seg    = SEG_NONE;
  return g_ngot++;
}

static void allocate_got(void)
{
  size_t i;
  size_t j;

  for (i = 1; i < g_ehdr->e_shnum; i++)
    {
      struct elf32_rel_s *rel;
      size_t nrel;

      if (g_shdr[i].sh_type != SHT_REL ||
          g_segof[g_shdr[i].sh_info] == SEG_NONE)
        {
          continue;
        }

      rel  = (struct elf32_rel_s *)(g_elf + g_shdr[i].sh_offset);
      nrel = g_shdr[i].sh_size / sizeof(struct elf32_rel_s);

      for (j = 0; j < nrel; j++)
        {
          if (g_arch->is_gotref(ELF32_R_TYPE(rel[j].r_info)))
            {
              got_index(ELF32_R_SYM(rel[j].r_info));
            }
        }
    }

  g_gotsize = g_ngot * sizeof(uint32_t);
}

/****************************************************************************
 * Name: build_images
 *
 * Description:
 *   Copy each section to the address the linker gave it, above the GOT in
 *   D-Space.
 *
 ****************************************************************************/

static void build_images(void)
{
  size_t i;

  g_data.size += g_gotsize;

  g_text.img = calloc(1, g_text.size ? g_text.size : 1);
  g_data.img = calloc(1, g_data.size ? g_data.size : 1);
  if (g_text.img == NULL || g_data.img == NULL)
    {
      fail("out of memory");
    }

  for (i = 1; i < g_ehdr->e_shnum; i++)
    {
      struct elf32_shdr_s *s = &g_shdr[i];

      if (g_segof[i] == SEG_NONE || s->sh_type == SHT_NOBITS ||
          s->sh_size == 0)
        {
          continue;
        }

      if (g_segof[i] == SEG_TEXT)
        {
          memcpy(g_text.img + s->sh_addr, g_elf + s->sh_offset, s->sh_size);
        }
      else
        {
          memcpy(g_data.img + g_gotsize + s->sh_addr,
                 g_elf + s->sh_offset, s->sh_size);
        }
    }
}

/****************************************************************************
 * Name: add_reloc
 ****************************************************************************/

static void add_reloc(int type, uint32_t offset)
{
  g_relocs = realloc(g_relocs, (g_nrelocs + 1) * sizeof(uint32_t));
  if (g_relocs == NULL)
    {
      fail("out of memory");
    }

  g_relocs[g_nrelocs++] = NXFLAT_RELOC(type, offset);

  if (g_verbose > 1)
    {
      printf("  record %s at D-Space %08x\n",
             type == NXFLAT_RELOC_TYPE_REL32I ? "REL32I" : "REL32D", offset);
    }
}

/****************************************************************************
 * Name: target_word
 *
 * Description:
 *   The word a relocation patches, in whichever image it lives in.
 *
 ****************************************************************************/

static uint8_t *target_word(enum segment_e seg, uint32_t offset)
{
  if (seg == SEG_TEXT)
    {
      if (offset + 4 > g_text.size)
        {
          fail("relocation at I-Space %08x is outside the text", offset);
        }

      return g_text.img + offset;
    }

  if (offset + 4 > g_data.size)
    {
      fail("relocation at D-Space %08x is outside the data", offset);
    }

  return g_data.img + offset;
}

static uint32_t get32(const uint8_t *p)
{
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void put32(uint8_t *p, uint32_t v)
{
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16);
  p[3] = (uint8_t)(v >> 24);
}

static uint32_t get16(const uint8_t *p)
{
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8);
}

static void put16(uint8_t *p, uint32_t v)
{
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
}

/****************************************************************************
 * Name: reloc_thm_call
 *
 * Description:
 *   A Thumb BL or B.W, encoded as arch_elf.c encodes it.  Both ends are in
 *   I-Space, so the loader never sees it.
 *
 ****************************************************************************/

static void reloc_thm_call(uint8_t *p, uint32_t place, uint32_t value,
                           bool isfunc)
{
  uint32_t upper_insn = get16(p);
  uint32_t lower_insn = get16(p + 2);
  int32_t offset;
  uint32_t S;
  uint32_t J1;
  uint32_t J2;

  S  = (upper_insn >> 10) & 1;
  J1 = (lower_insn >> 13) & 1;
  J2 = (lower_insn >> 11) & 1;

  offset = (int32_t)((S << 24) |
                     ((~(J1 ^ S) & 1) << 23) |
                     ((~(J2 ^ S) & 1) << 22) |
                     ((upper_insn & 0x03ff) << 12) |
                     ((lower_insn & 0x07ff) << 1));

  if ((offset & 0x01000000) != 0)
    {
      offset -= 0x02000000;
    }

  offset += (int32_t)value - (int32_t)place;

  if (isfunc && (offset & 1) == 0)
    {
      fail("THM_CALL at %08x needs an odd offset, got %08x", place, offset);
    }

  if (offset < (int32_t)0xff000000 || offset >= (int32_t)0x01000000)
    {
      fail("THM_CALL at %08x is out of range, target %08x", place, offset);
    }

  S  = (offset >> 24) & 1;
  J1 = S ^ (~(offset >> 23) & 1);
  J2 = S ^ (~(offset >> 22) & 1);

  put16(p, (upper_insn & 0xf800) | (S << 10) | ((offset >> 12) & 0x03ff));
  put16(p + 2, (lower_insn & 0xd000) | (J1 << 13) | (J2 << 11) |
               ((offset >> 1) & 0x07ff));
}

/****************************************************************************
 * Name: reloc_call24
 *
 * Description:
 *   An ARM-mode BL, for the boards that build modules without Thumb.
 *
 ****************************************************************************/

static void reloc_call24(uint8_t *p, uint32_t place, uint32_t value)
{
  uint32_t insn = get32(p);
  int32_t offset = (int32_t)((insn & 0x00ffffff) << 2);

  if ((offset & 0x02000000) != 0)
    {
      offset -= 0x04000000;
    }

  offset += (int32_t)value - (int32_t)place;

  if ((offset & 3) != 0 ||
      offset < (int32_t)0xfe000000 || offset >= (int32_t)0x02000000)
    {
      fail("CALL at %08x is out of range, target %08x", place, offset);
    }

  put32(p, (insn & 0xff000000) | ((offset >> 2) & 0x00ffffff));
}

/****************************************************************************
 * Name: arm_entry / arm_is_gotref / arm_reloc
 *
 * Description:
 *   ARM and Thumb-2, with the arithmetic of arch_elf.c.
 *
 ****************************************************************************/

static uint32_t arm_entry(uint32_t value)
{
  /* The loader adds this to the mapped I-Space and enters it, so it is an
   * address, not a function pointer: the Thumb bit comes off.
   */

  return value & ~1u;
}

static bool arm_is_gotref(uint32_t type)
{
  return type == R_ARM_GOT_BREL;
}

static enum reloc_action_e arm_reloc(struct reloc_s *r)
{
  switch (r->type)
    {
      case R_ARM_ABS32:
      case R_ARM_TARGET1:

        /* An address, which only the loader can finish.  The word keeps the
         * segment-relative value, Thumb bit and all, and gains a record.
         */

        if (r->pseg != SEG_DATA)
          {
            fail("ABS32 in I-Space at %08x cannot be relocated at load "
                 "time; only D-Space can", r->place);
          }

        put32(r->p, get32(r->p) + r->value);
        return RELOC_RECORD;

      case R_ARM_REL32:

        /* Both ends must be in one segment for this to survive the segments
         * being placed apart.
         */

        if (r->pseg != r->vseg)
          {
            fail("REL32 at %s %08x reaches %s %08x across the segments; "
                 "build the module with -mno-pic-data-is-text-relative",
                 r->pseg == SEG_TEXT ? "I-Space" : "D-Space", r->place,
                 r->vseg == SEG_TEXT ? "I-Space" : "D-Space", r->value);
          }

        put32(r->p, get32(r->p) + r->value - r->place);
        return RELOC_DONE;

      case R_ARM_GOTOFF32:

        /* An offset from the base the module carries in its PIC register,
         * which is the start of D-Space.
         */

        if (r->vseg != SEG_DATA)
          {
            fail("GOTOFF32 at %08x names %s, which is not in D-Space",
                 r->place, r->symname);
          }

        put32(r->p, get32(r->p) + r->value);
        return RELOC_DONE;

      case R_ARM_GOT_BREL:

        /* The reference holds where the entry is, counted from the start of
         * the GOT.  The entry itself is filled in and recorded by the
         * generic half, because that is the same for every architecture.
         */

        put32(r->p, get32(r->p) + r->gotoffset);
        return RELOC_DONE;

      case R_ARM_THM_CALL:
      case R_ARM_THM_JUMP24:

        if (r->pseg != SEG_TEXT || r->vseg != SEG_TEXT)
          {
            fail("a Thumb branch at %08x leaves I-Space", r->place);
          }

        reloc_thm_call(r->p, r->place, r->value, r->isfunc);
        return RELOC_DONE;

      case R_ARM_PC24:
      case R_ARM_CALL:
      case R_ARM_JUMP24:
      case R_ARM_PLT32:

        if (r->pseg != SEG_TEXT || r->vseg != SEG_TEXT)
          {
            fail("an ARM branch at %08x leaves I-Space", r->place);
          }

        reloc_call24(r->p, r->place, r->value);
        return RELOC_DONE;

      default:
        fail("%s relocation %u at %s %08x is not handled", g_arch->name,
             r->type, r->pseg == SEG_TEXT ? "I-Space" : "D-Space", r->place);
        return RELOC_DONE;
    }
}

/****************************************************************************
 * Name: select_arch
 ****************************************************************************/

static void select_arch(uint16_t machine)
{
  size_t i;

  for (i = 0; i < sizeof(g_arches) / sizeof(g_arches[0]); i++)
    {
      if (g_arches[i].machine == machine)
        {
          g_arch = &g_arches[i];
          return;
        }
    }

  fail("%s is for machine %u, which this tool does not know.  An "
       "architecture needs an entry in g_arches[] naming its relocations",
       g_elf_filename, machine);
}

/****************************************************************************
 * Name: resolve_relocs
 *
 * Description:
 *   Apply what can be applied; record what cannot.
 *
 ****************************************************************************/

static void resolve_relocs(void)
{
  size_t i;
  size_t j;

  for (i = 1; i < g_ehdr->e_shnum; i++)
    {
      enum segment_e tseg;
      struct elf32_rel_s *rel;
      uint32_t tbase;
      size_t nrel;

      if (g_shdr[i].sh_type != SHT_REL)
        {
          continue;
        }

      tseg = g_segof[g_shdr[i].sh_info];
      if (tseg == SEG_NONE)
        {
          continue;
        }

      /* Where the section this relocates sits in its segment.  A D-Space
       * section sits above the GOT.
       */

      tbase = g_shdr[g_shdr[i].sh_info].sh_addr;
      if (tseg == SEG_DATA)
        {
          tbase += g_gotsize;
        }

      rel  = (struct elf32_rel_s *)(g_elf + g_shdr[i].sh_offset);
      nrel = g_shdr[i].sh_size / sizeof(struct elf32_rel_s);

      for (j = 0; j < nrel; j++)
        {
          struct reloc_s r;
          uint32_t type   = ELF32_R_TYPE(rel[j].r_info);
          uint32_t symidx = ELF32_R_SYM(rel[j].r_info);
          const struct elf32_sym_s *sym;
          enum segment_e sseg;
          uint32_t place;
          uint32_t value;
          uint8_t *p;

          if (symidx >= g_nsyms)
            {
              fail("relocation names symbol %u of %zu", symidx, g_nsyms);
            }

          sym   = &g_syms[symidx];
          sseg  = sym_segment(sym);
          value = sym_value(sym);
          place = tbase + rel[j].r_offset;
          p     = target_word(tseg, place);

          if (g_verbose > 1)
            {
              printf("  reloc %2u at %s %08x -> %s %08x %s\n", type,
                     tseg == SEG_TEXT ? "I" : "D", place,
                     sseg == SEG_TEXT ? "I" : sseg == SEG_DATA ? "D" : "?",
                     value, sym_name(sym));
            }

          if (sym->st_shndx == SHN_UNDEF)
            {
              fail("%s is undefined; every import must come from the thunk "
                   "that mknxflat generates", sym_name(sym));
            }

          r.type      = type;
          r.place     = place;
          r.pseg      = tseg;
          r.value     = value;
          r.vseg      = sseg;
          r.isfunc    = ELF32_ST_TYPE(sym->st_info) == STT_FUNC;
          r.symname   = sym_name(sym);
          r.p         = p;
          r.gotoffset = 0;

          if (g_arch->is_gotref(type))
            {
              uint32_t idx = got_index(symidx);

              /* The entry holds the address, which may be a function in
               * I-Space as readily as an object in D-Space, and keeps the
               * Thumb bit or whatever else the symbol carries.
               */

              if (sseg == SEG_NONE)
                {
                  fail("a GOT reference at %08x names %s, which is in "
                       "neither segment", place, sym_name(sym));
                }

              g_got[idx].value = value;
              g_got[idx].seg   = sseg;
              r.gotoffset      = idx * sizeof(uint32_t);
            }

          if (g_arch->reloc(&r) == RELOC_RECORD)
            {
              add_reloc(sseg == SEG_TEXT ? NXFLAT_RELOC_TYPE_REL32I :
                                           NXFLAT_RELOC_TYPE_REL32D, place);
            }
        }
    }

  /* Every GOT entry is a D-Space word holding an address, so each one gets a
   * record: the I-Space kind when it holds a function.
   */

  for (j = 0; j < g_ngot; j++)
    {
      put32(g_data.img + j * sizeof(uint32_t), g_got[j].value);
      add_reloc(g_got[j].seg == SEG_TEXT ? NXFLAT_RELOC_TYPE_REL32I :
                                           NXFLAT_RELOC_TYPE_REL32D,
                j * sizeof(uint32_t));
    }
}

/****************************************************************************
 * Name: put_be32 / write_output
 *
 * Description:
 *   Write the container.  The header is in network order, which
 *   include/nxflat.h states and the loader's NTOHL() calls need; the images
 *   and the records are in the target's.
 *
 ****************************************************************************/

static void put_be32(uint8_t *p, uint32_t v)
{
  p[0] = (uint8_t)(v >> 24);
  p[1] = (uint8_t)(v >> 16);
  p[2] = (uint8_t)(v >> 8);
  p[3] = (uint8_t)v;
}

static void put_be16(uint8_t *p, uint32_t v)
{
  p[0] = (uint8_t)(v >> 8);
  p[1] = (uint8_t)v;
}

static void write_output(void)
{
  const struct elf32_sym_s *entry;
  const struct elf32_sym_s *ibegin;
  const struct elf32_sym_s *iend;
  uint8_t hdr[NXFLAT_HDR_SIZE];
  uint32_t importsymbols = 0;
  uint32_t importcount = 0;
  uint32_t datastart;
  uint32_t dataend;
  FILE *out;

  entry = find_symbol(g_entry_name);
  if (entry == NULL)
    {
      fail("entry point %s is not defined in %s",
           g_entry_name, g_elf_filename);
    }

  if (sym_segment(entry) != SEG_TEXT)
    {
      fail("entry point %s is not in I-Space", g_entry_name);
    }

  /* mknxflat puts the array of imported symbols in D-Space and marks its
   * ends.  A module that imports nothing has neither symbol.
   */

  ibegin = find_symbol("__dynimport_begin");
  iend   = find_symbol("__dynimport_end");

  datastart = NXFLAT_HDR_SIZE + g_text.size;
  dataend   = datastart + g_data.size;

  if (ibegin != NULL && iend != NULL)
    {
      if (sym_segment(ibegin) != SEG_DATA)
        {
          fail("__dynimport_begin is not in D-Space");
        }

      importsymbols = datastart + sym_value(ibegin);
      importcount   = (sym_value(iend) - sym_value(ibegin)) / 8;
    }

  memset(hdr, 0, sizeof(hdr));
  memcpy(hdr, NXFLAT_MAGIC, 4);
  put_be32(hdr + 4,  NXFLAT_HDR_SIZE + g_arch->entry(sym_value(entry)));
  put_be32(hdr + 8,  datastart);
  put_be32(hdr + 12, dataend);
  put_be32(hdr + 16, dataend + g_bsssize);
  put_be32(hdr + 20, g_stacksize);
  put_be32(hdr + 24, dataend);            /* The records follow the data */
  put_be32(hdr + 28, importsymbols);
  put_be16(hdr + 32, g_nrelocs);
  put_be16(hdr + 34, importcount);

  out = fopen(g_out_filename, "wb");
  if (out == NULL)
    {
      fail("%s: cannot create: %s", g_out_filename, strerror(errno));
    }

  if (fwrite(hdr, sizeof(hdr), 1, out) != 1 ||
      (g_text.size != 0 &&
       fwrite(g_text.img, g_text.size, 1, out) != 1) ||
      (g_data.size != 0 &&
       fwrite(g_data.img, g_data.size, 1, out) != 1) ||
      (g_nrelocs != 0 &&
       fwrite(g_relocs, sizeof(uint32_t), g_nrelocs, out) != g_nrelocs))
    {
      fail("%s: cannot write: %s", g_out_filename, strerror(errno));
    }

  fclose(out);

  if (g_verbose != 0)
    {
      printf("%s: entry %s at %08x\n", g_out_filename, g_entry_name,
             NXFLAT_HDR_SIZE + g_arch->entry(sym_value(entry)));
      printf("  I-Space %08x-%08x  %u bytes\n", NXFLAT_HDR_SIZE, datastart,
             g_text.size);
      printf("  D-Space %08x-%08x  %u bytes, of which %u GOT, %u bss\n",
             datastart, dataend, g_data.size, g_gotsize, g_bsssize);
      printf("  %u relocation record(s), %u import(s)\n",
             g_nrelocs, importcount);
    }
}

/****************************************************************************
 * Name: show_usage
 ****************************************************************************/

static void show_usage(void)
{
  fprintf(stderr, "Usage: %s -e <entry> -o <output> [-s <stack>] [-v] "
                  "<elf-file>\n\n", g_program);
  fprintf(stderr, "Convert one ELF object into an NXFLAT module.  The\n");
  fprintf(stderr, "object is what these produce, in this order:\n\n");
  fprintf(stderr, "  ld -r -d -warn-common\n");
  fprintf(stderr, "  mknxflat, assembled and linked in\n");
  fprintf(stderr, "  ld -r -d -warn-common -T gnu-nxflat-gotoff.ld"
                  " -no-check-sections\n\n");
  fprintf(stderr, "  -e <entry>  The symbol the loader enters.  Needed:\n");
  fprintf(stderr, "              a module has no crt0, so no name for it\n");
  fprintf(stderr, "              is conventional.\n");
  fprintf(stderr, "  -o <output> Where to write it.  Needed.\n");
  fprintf(stderr, "  -s <stack>  Stack the module is given, in bytes"
                  " [%d].\n", DEFAULT_STACK_SIZE);
  fprintf(stderr, "  -v          Say what was placed; twice, every"
                  " relocation.\n");
  fprintf(stderr, "\n");
  exit(1);
}

/****************************************************************************
 * Name: parse_args
 ****************************************************************************/

static void parse_args(int argc, char **argv)
{
  int opt;

  g_program = argv[0];

  while ((opt = getopt(argc, argv, "e:o:s:v")) != -1)
    {
      switch (opt)
        {
          case 'e':
            g_entry_name = optarg;
            break;

          case 'o':
            g_out_filename = optarg;
            break;

          case 's':
            g_stacksize = (uint32_t)strtoul(optarg, NULL, 0);
            break;

          case 'v':
            g_verbose++;
            break;

          default:
            show_usage();
            break;
        }
    }

  if (optind != argc - 1)
    {
      fprintf(stderr, "%s: one input file is needed\n", argv[0]);
      show_usage();
    }

  g_elf_filename = argv[optind];

  /* Neither is guessed:  an entry point is whatever its author called it,
   * and every board that builds a module already says so.
   */

  if (g_entry_name == NULL || g_out_filename == NULL)
    {
      fprintf(stderr, "%s: -e and -o are both needed\n", argv[0]);
      show_usage();
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv)
{
  parse_args(argc, argv);

  read_elf();
  place_sections();
  allocate_common();
  allocate_got();
  build_images();
  resolve_relocs();
  write_output();

  return 0;
}
