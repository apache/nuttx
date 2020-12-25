/****************************************************************************
 * include/elf.h
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

#ifndef __INCLUDE_ELF_H
#define __INCLUDE_ELF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#define EI_NIDENT          16     /* Size of e_ident[] */

/* NOTE: elf64.h and elf32.h refer EI_NIDENT defined above */

#ifdef CONFIG_LIBC_ARCH_ELF_64BIT
#  include <elf64.h>
#else
#  include <elf32.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Values for Elf_Ehdr::e_type */

#define ET_NONE           0       /* No file type */
#define ET_REL            1       /* Relocatable file */
#define ET_EXEC           2       /* Executable file */
#define ET_DYN            3       /* Shared object file */
#define ET_CORE           4       /* Core file */
#define ET_LOPROC         0xff00  /* Processor-specific */
#define ET_HIPROC         0xffff  /* Processor-specific */

/* Values for Elf_Ehdr::e_machine (most of this were not included in the
 * original SCO document but have been gleaned from elsewhere).
 */

#define EM_NONE            0      /* No machine */
#define EM_M32             1      /* AT&T WE 32100 */
#define EM_SPARC           2      /* SPARC */
#define EM_386             3      /* Intel 80386 */
#define EM_68K             4      /* Motorola 68000 */
#define EM_88K             5      /* Motorola 88000 */
#define EM_486             6      /* Intel 486+ */
#define EM_860             7      /* Intel 80860 */
#define EM_MIPS            8      /* MIPS R3000 Big-Endian */
#define EM_MIPS_RS4_BE     10     /* MIPS R4000 Big-Endian */
#define EM_PARISC          15     /* HPPA */
#define EM_SPARC32PLUS     18     /* Sun's "v8plus" */
#define EM_PPC             20     /* PowerPC */
#define EM_PPC64           21     /* PowerPC64 */
#define EM_ARM             40     /* ARM */
#define EM_SH              42     /* SuperH */
#define EM_SPARCV9         43     /* SPARC v9 64-bit */
#define EM_H8_300          46
#define EM_IA_64           50     /* HP/Intel IA-64 */
#define EM_X86_64          62     /* AMD x86-64 */
#define EM_S390            22     /* IBM S/390 */
#define EM_CRIS            76     /* Axis Communications 32-bit embedded processor */
#define EM_V850            87     /* NEC v850 */
#define EM_M32R            88     /* Renesas M32R */
#define EM_XTENSA          94     /* Tensilica Xtensa */
#define EM_RISCV           243    /* RISC-V */
#define EM_ALPHA           0x9026
#define EM_CYGNUS_V850     0x9080
#define EM_CYGNUS_M32R     0x9041
#define EM_S390_OLD        0xa390
#define EM_FRV             0x5441

/* Values for Elf_Ehdr::e_version */

#define EV_NONE            0      /* Invalid version */
#define EV_CURRENT         1      /* The current version */

/* Table 2. Ehe ELF identifier */

#define EI_MAG0            0      /* File identification */
#define EI_MAG1            1
#define EI_MAG2            2
#define EI_MAG3            3
#define EI_CLASS           4      /* File class */
#define EI_DATA            5      /* Data encoding */
#define EI_VERSION         6      /* File version */
#define EI_PAD             7      /* Start of padding bytes */

/* EI_NIDENT is defined in "Included Files" section */

#define EI_MAGIC_SIZE      4
#define EI_MAGIC           {0x7f, 'E', 'L', 'F'}

/* Table 3. Values for EI_CLASS */

#define ELFCLASSNONE       0      /* Invalid class */
#define ELFCLASS32         1      /* 32-bit objects */
#define ELFCLASS64         2      /* 64-bit objects */

/* Table 4. Values for EI_DATA */

#define ELFDATANONE        0     /* Invalid data encoding */
#define ELFDATA2LSB        1     /* Least significant byte occupying the lowest address */
#define ELFDATA2MSB        2     /* Most significant byte occupying the lowest address */

/* Table 7: Special Section Indexes */

#define SHN_UNDEF          0
#define SHN_LOPROC         0xff00
#define SHN_HIPROC         0xff1f
#define SHN_ABS            0xfff1
#define SHN_COMMON         0xfff2

/* Figure 4-9: Section Types, sh_type */

#define SHT_NULL           0
#define SHT_PROGBITS       1
#define SHT_SYMTAB         2
#define SHT_STRTAB         3
#define SHT_RELA           4
#define SHT_HASH           5
#define SHT_DYNAMIC        6
#define SHT_NOTE           7
#define SHT_NOBITS         8
#define SHT_REL            9
#define SHT_SHLIB          10
#define SHT_DYNSYM         11
#define SHT_LOPROC         0x70000000
#define SHT_HIPROC         0x7fffffff
#define SHT_LOUSER         0x80000000
#define SHT_HIUSER         0xffffffff

/* Figure 4-11: Section Attribute Flags, sh_flags */

#define SHF_WRITE          1
#define SHF_ALLOC          2
#define SHF_EXECINSTR      4
#define SHF_MASKPROC       0xf0000000

/* Figure 4-16: Symbol Binding, ELF_ST_BIND */

#define STB_LOCAL          0
#define STB_GLOBAL         1
#define STB_WEAK           2
#define STB_LOPROC         13
#define STB_HIPROC         15

/* Figure 4-17: Symbol Types, ELF_ST_TYPE */

#define STT_NOTYPE         0
#define STT_OBJECT         1
#define STT_FUNC           2
#define STT_SECTION        3
#define STT_FILE           4
#define STT_LOPROC         13
#define STT_HIPROC         15

/* Figure 5-2: Segment Types, p_type */

#define PT_NULL            0
#define PT_LOAD            1
#define PT_DYNAMIC         2
#define PT_INTERP          3
#define PT_NOTE            4
#define PT_SHLIB           5
#define PT_PHDR            6
#define PT_LOPROC          0x70000000
#define PT_HIPROC          0x7fffffff

/* Figure 5-3: Segment Flag Bits, p_flags */

#define PF_X               1          /* Execute */
#define PF_W               2          /* Write */
#define PF_R               4          /* Read */
#define PF_MASKPROC        0xf0000000 /* Unspecified */

/* Figure 5-10: Dynamic Array Tags, d_tag */

#define DT_NULL            0          /* d_un=ignored */
#define DT_NEEDED          1          /* d_un=d_val */
#define DT_PLTRELSZ        2          /* d_un=d_val */
#define DT_PLTGOT          3          /* d_un=d_ptr */
#define DT_HASH            4          /* d_un=d_ptr */
#define DT_STRTAB          5          /* d_un=d_ptr */
#define DT_SYMTAB          6          /* d_un=d_ptr */
#define DT_RELA            7          /* d_un=d_ptr */
#define DT_RELASZ          8          /* d_un=d_val */
#define DT_RELAENT         9          /* d_un=d_val */
#define DT_STRSZ           10         /* d_un=d_val */
#define DT_SYMENT          11         /* d_un=d_val */
#define DT_INIT            12         /* d_un=d_ptr */
#define DT_FINI            13         /* d_un=d_ptr */
#define DT_SONAME          14         /* d_un=d_val */
#define DT_RPATH           15         /* d_un=d_val */
#define DT_SYMBOLIC        16         /* d_un=ignored */
#define DT_REL             17         /* d_un=d_ptr */
#define DT_RELSZ           18         /* d_un=d_val */
#define DT_RELENT          19         /* d_un=d_val */
#define DT_PLTREL          20         /* d_un=d_val */
#define DT_DEBUG           21         /* d_un=d_ptr */
#define DT_TEXTREL         22         /* d_un=ignored */
#define DT_JMPREL          23         /* d_un=d_ptr */
#define DT_BINDNOW         24         /* d_un=ignored */
#define DT_LOPROC          0x70000000 /* d_un=unspecified */
#define DT_HIPROC          0x7fffffff /* d_un= unspecified */

#endif /* __INCLUDE_ELF_H */
