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
#include <nuttx/arch.h>

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

#ifdef PAGESIZE
#  define ELF_PAGESIZE PAGESIZE
#else
#  define ELF_PAGESIZE 1024
#endif

#define PROGRAM_ALIGNMENT 64

#define ROUNDUP(x, y)     ((x + (y - 1)) / (y)) * (y)
#define ROUNDDOWN(x ,y)   (((x) / (y)) * (y))

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint8_t g_running_regs[XCPTCONTEXT_SIZE] aligned_data(16);

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
 * Name: elf_emit_hdr
 *
 * Description:
 *   Fill the elf header
 *
 ****************************************************************************/

static int elf_emit_hdr(FAR struct elf_dumpinfo_s *cinfo,
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
 * Name: elf_get_ntcb
 *
 * Description:
 *   Calculate the note segment size
 *
 ****************************************************************************/

static int elf_get_ntcb(void)
{
  int count = 0;
  int i;

  for (i = 0; i < g_npidhash; i++)
    {
      if (g_pidhash[i] != NULL)
        {
          count++;
        }
    }

  return count;
}

/****************************************************************************
 * Name: elf_get_note_size
 *
 * Description:
 *   Calculate the note segment size
 *
 ****************************************************************************/

static int elf_get_note_size(int stksegs)
{
  int total;

  total  = stksegs * (sizeof(Elf_Nhdr) + ROUNDUP(CONFIG_TASK_NAME_SIZE, 8) +
                     sizeof(elf_prstatus_t));
  total += stksegs * (sizeof(Elf_Nhdr) + ROUNDUP(CONFIG_TASK_NAME_SIZE, 8) +
                     sizeof(elf_prpsinfo_t));
  return total;
}

/****************************************************************************
 * Name: elf_emit_tcb_note
 *
 * Description:
 *   Fill the note segment information from tcb
 *
 ****************************************************************************/

static void elf_emit_tcb_note(FAR struct elf_dumpinfo_s *cinfo,
                              FAR struct tcb_s *tcb)
{
  char name[ROUNDUP(CONFIG_TASK_NAME_SIZE, 8)];
  elf_prstatus_t status;
  elf_prpsinfo_t info;
  FAR uint32_t *regs;
  Elf_Nhdr nhdr;
  int i;

  memset(&info,   0x0, sizeof(info));
  memset(&status, 0x0, sizeof(status));

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

  if (running_task() == tcb)
    {
      if (up_interrupt_context())
        {
          regs = (FAR uint32_t *)CURRENT_REGS;
        }
      else
        {
          up_saveusercontext(g_running_regs);
          regs = (FAR uint32_t *)g_running_regs;
        }
    }
  else
    {
      regs = tcb->xcp.regs;
    }

  if (regs != NULL)
    {
      for (i = 0; i < nitems(status.pr_regs); i++)
        {
          if (g_tcbinfo.reg_off.p[i] == UINT16_MAX)
            {
              continue;
            }
          else
            {
              status.pr_regs[i] =
                *(uintptr_t *)((uint8_t *)regs + g_tcbinfo.reg_off.p[i]);
            }
        }
    }

  elf_emit(cinfo, &status, sizeof(status));
}

/****************************************************************************
 * Name: elf_emit_note
 *
 * Description:
 *   Fill the note segment information
 *
 ****************************************************************************/

static void elf_emit_note(FAR struct elf_dumpinfo_s *cinfo)
{
  int i;

  if (cinfo->pid == INVALID_PROCESS_ID)
    {
      for (i = 0; i < g_npidhash; i++)
        {
          if (g_pidhash[i] != NULL)
            {
              elf_emit_tcb_note(cinfo, g_pidhash[i]);
            }
        }
    }
  else
    {
      elf_emit_tcb_note(cinfo, nxsched_get_tcb(cinfo->pid));
    }
}

/****************************************************************************
 * Name: elf_emit_tcb_stack
 *
 * Description:
 *   Fill the task stack information from tcb
 *
 ****************************************************************************/

static void elf_emit_tcb_stack(FAR struct elf_dumpinfo_s *cinfo,
                               FAR struct tcb_s *tcb)
{
  uintptr_t buf = 0;
  uintptr_t sp;
  size_t len;

  if (running_task() != tcb)
    {
      sp = up_getusrsp(tcb->xcp.regs);

      if (sp > (uintptr_t)tcb->stack_base_ptr &&
          sp < (uintptr_t)tcb->stack_base_ptr + tcb->adj_stack_size)
        {
          len = ((uintptr_t)tcb->stack_base_ptr +
                            tcb->adj_stack_size) - sp;
          buf = sp;
        }
#ifdef CONFIG_STACK_COLORATION
      else
        {
          len = up_check_tcbstack(tcb);
          buf = (uintptr_t)tcb->stack_base_ptr +
                           (tcb->adj_stack_size - len);
        }
#endif
    }

  if (buf == 0)
    {
      buf = (uintptr_t)tcb->stack_alloc_ptr;
      len = tcb->adj_stack_size +
            (tcb->stack_base_ptr - tcb->stack_alloc_ptr);
    }

  sp  = ROUNDDOWN(buf, PROGRAM_ALIGNMENT);
  len = ROUNDUP(len + (buf - sp), PROGRAM_ALIGNMENT);
  buf = sp;

  elf_emit(cinfo, (FAR void *)buf, len);

  /* Align to page */

  elf_emit_align(cinfo);
}

/****************************************************************************
 * Name: elf_emit_stack
 *
 * Description:
 *   Fill the task stack information
 *
 ****************************************************************************/

static void elf_emit_stack(FAR struct elf_dumpinfo_s *cinfo)
{
  int i;

  if (cinfo->pid == INVALID_PROCESS_ID)
    {
      for (i = 0; i < g_npidhash; i++)
        {
          if (g_pidhash[i] != NULL)
            {
              elf_emit_tcb_stack(cinfo, g_pidhash[i]);
            }
        }
    }
  else
    {
      elf_emit_tcb_stack(cinfo, nxsched_get_tcb(cinfo->pid));
    }
}

/****************************************************************************
 * Name: elf_emit_memory
 *
 * Description:
 *   Fill the note segment information
 *
 ****************************************************************************/

static void elf_emit_memory(FAR struct elf_dumpinfo_s *cinfo, int memsegs)
{
  int i;

  for (i = 0; i < memsegs; i++)
    {
      elf_emit(cinfo, (FAR void *)cinfo->regions[i].start,
               cinfo->regions[i].end -
               cinfo->regions[i].start);

      /* Align to page */

      elf_emit_align(cinfo);
    }
}

/****************************************************************************
 * Name: elf_emit_tcb_phdr
 *
 * Description:
 *   Fill the program segment header from tcb
 *
 ****************************************************************************/

static void elf_emit_tcb_phdr(FAR struct elf_dumpinfo_s *cinfo,
                              FAR struct tcb_s *tcb,
                              FAR Elf_Phdr *phdr, off_t *offset)
{
  uintptr_t sp;

  phdr->p_vaddr = 0;

  if (running_task() != tcb)
    {
      sp = up_getusrsp(tcb->xcp.regs);

      if (sp > (uintptr_t)tcb->stack_base_ptr &&
          sp < (uintptr_t)tcb->stack_base_ptr + tcb->adj_stack_size)
        {
          phdr->p_filesz = ((uintptr_t)tcb->stack_base_ptr +
                           tcb->adj_stack_size) - sp;
          phdr->p_vaddr  = sp;
        }
#ifdef CONFIG_STACK_COLORATION
      else
        {
          phdr->p_filesz = up_check_tcbstack(tcb);
          phdr->p_vaddr  = (uintptr_t)tcb->stack_base_ptr +
                           (tcb->adj_stack_size - phdr->p_filesz);
        }
#endif
    }

  if (phdr->p_vaddr == 0)
    {
      phdr->p_vaddr  = (uintptr_t)tcb->stack_alloc_ptr;
      phdr->p_filesz = tcb->adj_stack_size +
                      (tcb->stack_base_ptr - tcb->stack_alloc_ptr);
    }

  sp = ROUNDDOWN(phdr->p_vaddr, PROGRAM_ALIGNMENT);
  phdr->p_filesz = ROUNDUP(phdr->p_filesz +
                           (phdr->p_vaddr - sp), PROGRAM_ALIGNMENT);
  phdr->p_vaddr  = sp;

  phdr->p_type   = PT_LOAD;
  phdr->p_offset = ROUNDUP(*offset, ELF_PAGESIZE);
  phdr->p_paddr  = phdr->p_vaddr;
  phdr->p_memsz  = phdr->p_filesz;
  phdr->p_flags  = PF_X | PF_W | PF_R;
  phdr->p_align  = ELF_PAGESIZE;
  *offset       += ROUNDUP(phdr->p_memsz, ELF_PAGESIZE);

  elf_emit(cinfo, phdr, sizeof(*phdr));
}

/****************************************************************************
 * Name: elf_emit_phdr
 *
 * Description:
 *   Fill the program segment header
 *
 ****************************************************************************/

static void elf_emit_phdr(FAR struct elf_dumpinfo_s *cinfo,
                          int stksegs, int memsegs)
{
  off_t offset = cinfo->stream->nput +
                 (stksegs + memsegs + 1) * sizeof(Elf_Phdr);
  Elf_Phdr phdr;
  int i;

  memset(&phdr, 0, sizeof(Elf_Phdr));

  phdr.p_type   = PT_NOTE;
  phdr.p_offset = offset;
  phdr.p_filesz = elf_get_note_size(stksegs);
  offset       += phdr.p_filesz;

  elf_emit(cinfo, &phdr, sizeof(phdr));

  if (cinfo->pid == INVALID_PROCESS_ID)
    {
      for (i = 0; i < g_npidhash; i++)
        {
          if (g_pidhash[i] != NULL)
            {
              elf_emit_tcb_phdr(cinfo, g_pidhash[i], &phdr, &offset);
            }
        }
    }
  else
    {
      elf_emit_tcb_phdr(cinfo, nxsched_get_tcb(cinfo->pid),
                        &phdr, &offset);
    }

  /* Write program headers for segments dump */

  for (i = 0; i < memsegs; i++)
    {
      phdr.p_type   = PT_LOAD;
      phdr.p_offset = ROUNDUP(offset, ELF_PAGESIZE);
      phdr.p_vaddr  = cinfo->regions[i].start;
      phdr.p_paddr  = phdr.p_vaddr;
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
  irqstate_t flags;
  int memsegs = 0;
  int stksegs;

  flags = enter_critical_section();

  if (cinfo->pid != INVALID_PROCESS_ID)
    {
      if (nxsched_get_tcb(cinfo->pid) == NULL)
        {
          leave_critical_section(flags);
          return -EINVAL;
        }

      stksegs = 1;
    }
  else
    {
      stksegs = elf_get_ntcb();
    }

  /* Check the memory region */

  if (cinfo->regions != NULL)
    {
      for (; cinfo->regions[memsegs].start <
             cinfo->regions[memsegs].end; memsegs++);
    }

  /* Fill notes section */

  elf_emit_hdr(cinfo, stksegs + memsegs + 1);

  /* Fill all the program information about the process for the
   * notes.  This also sets up the file header.
   */

  elf_emit_phdr(cinfo, stksegs, memsegs);

  /* Fill note information */

  elf_emit_note(cinfo);

  /* Align to page */

  elf_emit_align(cinfo);

  /* Dump stack */

  elf_emit_stack(cinfo);

  /* Dump memory segments */

  if (memsegs > 0)
    {
      elf_emit_memory(cinfo, memsegs);
    }

  /* Flush the dump */

  elf_flush(cinfo);

  leave_critical_section(flags);

  return OK;
}
