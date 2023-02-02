/****************************************************************************
 * binfmt/libelf/libelf_coredump.c
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

#include <sys/stat.h>
#include <sys/param.h>

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/elf.h>
#include <nuttx/binfmt/elf.h>
#include <nuttx/binfmt/binfmt.h>
#include <nuttx/sched.h>
#include <sched/sched.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ELF_PAGESIZE    4096

#define ROUNDUP(x, y)   ((x + (y - 1)) / (y)) * (y)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: elf_flush
 *
 * Description:
 *   Flush the out stream
 *
 ****************************************************************************/

static int elf_flush(FAR struct elf_dumpinfo_s *cinfo)
{
  return lib_stream_flush(cinfo->stream);
}

/****************************************************************************
 * Name: elf_emit
 *
 * Description:
 *   Send the dump data to binfmt_outstream_s
 *
 ****************************************************************************/

static int elf_emit(FAR struct elf_dumpinfo_s *cinfo,
                    FAR const void *buf, size_t len)
{
  FAR const uint8_t *ptr = buf;
  size_t total = len;
  int ret;

  while (total > 0)
    {
      ret = lib_stream_puts(cinfo->stream, ptr, total);
      if (ret < 0)
        {
          break;
        }

      total -= ret;
      ptr   += ret;
    }

  return ret < 0 ? ret : len - total;
}

/****************************************************************************
 * Name: elf_emit_align
 *
 * Description:
 *   Align the filled data according to the current offset
 *
 ****************************************************************************/

static int elf_emit_align(FAR struct elf_dumpinfo_s *cinfo)
{
  off_t align = ROUNDUP(cinfo->stream->nput,
                        ELF_PAGESIZE) - cinfo->stream->nput;
  unsigned char null[256];
  off_t total = align;
  off_t ret;

  memset(null, 0, sizeof(null));

  while (total > 0)
    {
      ret = elf_emit(cinfo, null, total > sizeof(null) ?
                                 sizeof(null) : total);
      if (ret <= 0)
        {
          break;
        }

      total -= ret;
    }

  return ret < 0 ? ret : align;
}

/****************************************************************************
 * Name: elf_emit_header
 *
 * Description:
 *   Fill the elf header
 *
 ****************************************************************************/

static int elf_emit_header(FAR struct elf_dumpinfo_s *cinfo,
                           int segs)
{
  Elf_Ehdr ehdr;

  memset(&ehdr, 0, sizeof(ehdr));
  memcpy(ehdr.e_ident, ELFMAG, EI_MAGIC_SIZE);

  ehdr.e_ident[EI_CLASS]   = ELF_CLASS;
  ehdr.e_ident[EI_DATA]    = ELF_DATA;
  ehdr.e_ident[EI_VERSION] = EV_CURRENT;
  ehdr.e_ident[EI_OSABI]   = ELF_OSABI;

  ehdr.e_type              = ET_CORE;
  ehdr.e_machine           = EM_ARCH;
  ehdr.e_version           = EV_CURRENT;
  ehdr.e_phoff             = sizeof(Elf_Ehdr);
  ehdr.e_flags             = EF_FLAG;
  ehdr.e_ehsize            = sizeof(Elf_Ehdr);
  ehdr.e_phentsize         = sizeof(Elf_Phdr);
  ehdr.e_phnum             = segs;

  return elf_emit(cinfo, &ehdr, sizeof(ehdr));
}

/****************************************************************************
 * Name: elf_get_note_size
 *
 * Description:
 *   Calculate the note segment size
 *
 ****************************************************************************/

static int elf_get_note_size(void)
{
  int count = 0;
  int total;
  int i;

  for (i = 0; i < g_npidhash; i++)
    {
      if (g_pidhash[i])
        {
          count++;
        }
    }

  total  = count * (sizeof(Elf_Nhdr) + ROUNDUP(CONFIG_TASK_NAME_SIZE, 8) +
                    sizeof(elf_prstatus_t));
  total += count * (sizeof(Elf_Nhdr) + ROUNDUP(CONFIG_TASK_NAME_SIZE, 8) +
                    sizeof(elf_prpsinfo_t));
  return total;
}

/****************************************************************************
 * Name: elf_emit_note_info
 *
 * Description:
 *   Fill the note segment information
 *
 ****************************************************************************/

static void elf_emit_note_info(FAR struct elf_dumpinfo_s *cinfo)
{
  char name[ROUNDUP(CONFIG_TASK_NAME_SIZE, 8)];
  FAR struct tcb_s *tcb;
  elf_prstatus_t status;
  elf_prpsinfo_t info;
  Elf_Nhdr nhdr;
  int i;
  int j;

  memset(&info,   0x0, sizeof(info));
  memset(&status, 0x0, sizeof(status));

  for (i = 0; i < g_npidhash; i++)
    {
      if (g_pidhash[i] == NULL)
        {
          continue;
        }

      tcb = g_pidhash[i];

      /* Fill Process info */

      nhdr.n_namesz = sizeof(name);
      nhdr.n_descsz = sizeof(info);
      nhdr.n_type   = NT_PRPSINFO;

      elf_emit(cinfo, &nhdr, sizeof(nhdr));

      strlcpy(name, tcb->name, sizeof(name));
      elf_emit(cinfo, name, sizeof(name));

      info.pr_pid   = tcb->pid;
      strlcpy(info.pr_fname, tcb->name, sizeof(info.pr_fname));
      elf_emit(cinfo, &info, sizeof(info));

      /* Fill Process status */

      nhdr.n_descsz = sizeof(status);
      nhdr.n_type   = NT_PRSTATUS;

      elf_emit(cinfo, &nhdr, sizeof(nhdr));
      elf_emit(cinfo, name, sizeof(name));

      status.pr_pid = tcb->pid;

      for (j = 0; j < nitems(status.pr_regs); j++)
        {
          status.pr_regs[j] = *(uintptr_t *)((uint8_t *)tcb +
                                             g_tcbinfo.reg_off.p[j]);
        }

      elf_emit(cinfo, &status, sizeof(status));
    }
}

/****************************************************************************
 * Name: elf_emit_program_header
 *
 * Description:
 *   Fill the program segment header
 *
 ****************************************************************************/

static void elf_emit_program_header(FAR struct elf_dumpinfo_s *cinfo,
                                    int segs)
{
  off_t offset = cinfo->stream->nput + (segs + 1) * sizeof(Elf_Phdr);
  Elf_Phdr phdr;
  int i;

  memset(&phdr, 0, sizeof(Elf_Phdr));

  phdr.p_type   = PT_NOTE;
  phdr.p_offset = offset;
  phdr.p_filesz = elf_get_note_size();
  offset       += phdr.p_filesz;

  elf_emit(cinfo, &phdr, sizeof(phdr));

  /* Write program headers for segments dump */

  for (i = 0; i < segs; i++)
    {
      phdr.p_type   = PT_LOAD;
      phdr.p_offset = ROUNDUP(offset, ELF_PAGESIZE);
      phdr.p_vaddr  = cinfo->regions[i].start;
      phdr.p_paddr  = cinfo->regions[i].start;
      phdr.p_filesz = cinfo->regions[i].end - cinfo->regions[i].start;
      phdr.p_memsz  = phdr.p_filesz;
      phdr.p_flags  = cinfo->regions[i].flags;
      phdr.p_align  = ELF_PAGESIZE;
      offset       += ROUNDUP(phdr.p_memsz, ELF_PAGESIZE);
      elf_emit(cinfo, &phdr, sizeof(phdr));
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: elf_coredump
 *
 * Description:
 *   Generat the core dump stream as ELF structure.
 *
 * Input Parameters:
 *   dumpinfo - elf coredump informations
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int elf_coredump(FAR struct elf_dumpinfo_s *cinfo)
{
  int segs = 0;
  int i;

  /* Check the memory region */

  if (cinfo->regions)
    {
      for (; cinfo->regions[segs].start <
             cinfo->regions[segs].end; segs++);
    }

  if (segs == 0)
    {
      return -EINVAL;
    }

  /* Fill notes section */

  elf_emit_header(cinfo, segs + 1);

  /* Fill all the program information about the process for the
   * notes.  This also sets up the file header.
   */

  elf_emit_program_header(cinfo, segs);

  /* Fill note information */

  elf_emit_note_info(cinfo);

  /* Align to page */

  elf_emit_align(cinfo);

  /* Start dump the memory */

  for (i = 0; i < segs; i++)
    {
      elf_emit(cinfo, (FAR void *)cinfo->regions[i].start,
               cinfo->regions[i].end -
               cinfo->regions[i].start);

      /* Align to page */

      elf_emit_align(cinfo);
    }

  /* Flush the dump */

  return elf_flush(cinfo);
}
