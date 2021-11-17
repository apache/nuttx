/****************************************************************************
 * include/nuttx/elf.h
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

#ifndef __INCLUDE_NUTTX_ELF_H
#define __INCLUDE_NUTTX_ELF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <elf.h>
#ifdef CONFIG_ELF_COREDUMP
#include <arch/elf.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ELF_PRARGSZ    (80)  /* Number of chars for args */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef CONFIG_ELF_COREDUMP
typedef struct elf_prpsinfo_s
{
  char           pr_state;    /* Numeric process state */
  char           pr_sname;    /* Char for pr_state */
  char           pr_zomb;     /* Zombie */
  char           pr_nice;     /* Nice val */
  unsigned long  pr_flag;     /* Flags */
  unsigned short pr_uid;
  unsigned short pr_gid;
  int            pr_pid;
  int            pr_ppid;
  int            pr_pgrp;
  int            pr_sid;
  char           pr_fname[16];           /* Filename of executable */
  char           pr_psargs[ELF_PRARGSZ]; /* Initial part of arg list */
} elf_prpsinfo_t;

typedef struct elf_siginfo_s
{
  int            si_signo;    /* Signal number */
  int            si_code;     /* Extra code */
  int            si_errno;    /* Errno */
} elf_siginfo_t;

typedef struct elf_timeval_s
{
  long           tv_sec;      /* Seconds */
  long           tv_usec;     /* Microseconds */
} elf_timeval_t;

typedef struct elf_prstatus_s
{
  elf_siginfo_t  pr_info;     /* Info associated with signal */
  short          pr_cursig;   /* Current signal */
  short          pr_padding;  /* Padding align */
  unsigned long  pr_sigpend;  /* Set of pending signals */
  unsigned long  pr_sighold;  /* Set of held signals */
  int            pr_pid;
  int            pr_ppid;
  int            pr_pgrp;
  int            pr_sid;
  elf_timeval_t  pr_utime;    /* User time */
  elf_timeval_t  pr_stime;    /* System time */
  elf_timeval_t  pr_cutime;   /* Cumulative user time */
  elf_timeval_t  pr_cstime;   /* Cumulative system time */
  elf_gregset_t  pr_regs;
  int            pr_fpvalid;  /* True if math co-processor being used */
} elf_prstatus_t;
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if defined(__cplusplus)
extern "C"
{
#endif

/****************************************************************************
 * Name: up_checkarch
 *
 * Description:
 *   Given the ELF header in 'hdr', verify that the module is appropriate
 *   for the current, configured architecture.  Every architecture that uses
 *   the module loader must provide this function.
 *
 * Input Parameters:
 *   hdr - The ELF header read from the module file.
 *
 * Returned Value:
 *   True if the architecture supports this module file.
 *
 ****************************************************************************/

#ifdef CONFIG_LIBC_ARCH_ELF
bool up_checkarch(FAR const Elf_Ehdr *hdr);
#endif

/****************************************************************************
 * Name: up_relocate and up_relocateadd
 *
 * Description:
 *   Perform on architecture-specific ELF relocation.  Every architecture
 *   that uses the module loader must provide this function.
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

#ifdef CONFIG_LIBC_ARCH_ELF
int up_relocate(FAR const Elf_Rel *rel, FAR const Elf_Sym *sym,
                uintptr_t addr);
int up_relocateadd(FAR const Elf_Rela *rel,
                   FAR const Elf_Sym *sym, uintptr_t addr);
#endif

/****************************************************************************
 * Name: up_init_exidx
 *
 * Description:
 *   Initialize the exception index section.
 *
 * Input Parameters:
 *   address - The exception index section address.
 *   size    - The exception index section size.
 *
 * Returned Value:
 *   Zero (OK) if the initialization was successful. Otherwise, a negated
 *   errno value indicating the cause of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_CXX_EXCEPTION
int up_init_exidx(Elf_Addr address, Elf_Word size);
#endif

#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_ELF_H */
