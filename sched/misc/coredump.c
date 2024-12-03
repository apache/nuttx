/****************************************************************************
 * sched/misc/coredump.c
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
 * Included Files
 ****************************************************************************/

#include <sys/stat.h>
#include <sys/param.h>

#include <syslog.h>
#include <debug.h>

#include <nuttx/coredump.h>
#include <nuttx/elf.h>
#include <nuttx/sched.h>

#include "sched/sched.h"
#include "coredump.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef PAGESIZE
#  define ELF_PAGESIZE    PAGESIZE
#else
#  define ELF_PAGESIZE    1024
#endif

#if defined(CONFIG_BOARD_COREDUMP_BLKDEV) || \
    defined(CONFIG_BOARD_COREDUMP_MTDDEV)
#  define CONFIG_BOARD_COREDUMP_DEV
#endif

#define PROGRAM_ALIGNMENT 64

#define ROUNDUP(x, y)     ((x + (y - 1)) / (y)) * (y)
#define ROUNDDOWN(x ,y)   (((x) / (y)) * (y))

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This struct provides a description of the dump information of
 * memory regions.
 */

struct elf_dumpinfo_s
{
  FAR const struct memory_region_s *regions;
  FAR struct lib_outstream_s *stream;
  pid_t                       pid;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint8_t g_running_regs[XCPTCONTEXT_SIZE] aligned_data(16);

#ifdef CONFIG_BOARD_COREDUMP_COMPRESSION
static struct lib_lzfoutstream_s  g_lzfstream;
#endif

#ifdef CONFIG_BOARD_COREDUMP_SYSLOG
static struct lib_syslogstream_s  g_syslogstream;
#  ifdef CONFIG_BOARD_COREDUMP_BASE64STREAM
static struct lib_base64outstream_s g_base64stream;
#  else
static struct lib_hexdumpstream_s g_hexstream;
#  endif
#endif

#ifdef CONFIG_BOARD_COREDUMP_BLKDEV
static struct lib_blkoutstream_s g_devstream;
#elif defined(CONFIG_BOARD_COREDUMP_MTDDEV)
static struct lib_mtdoutstream_s g_devstream;
#endif

#ifdef CONFIG_BOARD_MEMORY_RANGE
static struct memory_region_s g_memory_region[] =
  {
    CONFIG_BOARD_MEMORY_RANGE
  };
#endif
static const struct memory_region_s *g_regions;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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
  off_t ret = 0;

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
  FAR uintptr_t *regs;
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
          regs = (FAR uintptr_t *)running_regs();
        }
      else
        {
          up_saveusercontext(g_running_regs);
          regs = (FAR uintptr_t *)g_running_regs;
        }
    }
  else
    {
      regs = (FAR uintptr_t *)tcb->xcp.regs;
    }

  if (regs != NULL)
    {
      for (i = 0; i < nitems(status.pr_regs); i++)
        {
          if (g_tcbinfo.reg_off.p[i] != UINT16_MAX)
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
      if (cinfo->regions[i].flags & PF_REGISTER)
        {
          FAR uintptr_t *start = (FAR uintptr_t *)cinfo->regions[i].start;
          FAR uintptr_t *end = (FAR uintptr_t *)cinfo->regions[i].end;
          uintptr_t buf[64];
          size_t offset = 0;

          while (start < end)
            {
              buf[offset++] = *start++;

              if (offset % (sizeof(buf) / sizeof(uintptr_t)) == 0)
                {
                  elf_emit(cinfo, buf, sizeof(buf));
                  offset = 0;
                }
            }

          if (offset != 0)
            {
              elf_emit(cinfo, buf, offset * sizeof(uintptr_t));
            }
        }
      else
        {
          elf_emit(cinfo, (FAR void *)cinfo->regions[i].start,
                   cinfo->regions[i].end - cinfo->regions[i].start);
        }

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

  phdr.p_align  = ELF_PAGESIZE;
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
      offset       += ROUNDUP(phdr.p_memsz, ELF_PAGESIZE);
      elf_emit(cinfo, &phdr, sizeof(phdr));
    }
}

/****************************************************************************
 * Name: coredump_dump_syslog
 *
 * Description:
 *   Put coredump to block device.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_COREDUMP_SYSLOG
static void coredump_dump_syslog(pid_t pid)
{
  FAR void *stream;
  FAR const char *streamname;
  int logmask;

  logmask = setlogmask(LOG_ALERT);

  _alert("Start coredump:\n");

  /* Initialize hex output stream */

  lib_syslogstream(&g_syslogstream, LOG_EMERG);
  stream = &g_syslogstream;
#ifdef CONFIG_BOARD_COREDUMP_BASE64STREAM
  lib_base64outstream(&g_base64stream, stream);
  stream = &g_base64stream;
  streamname = "base64";
#else
  lib_hexdumpstream(&g_hexstream, stream);
  stream = &g_hexstream;
  streamname = "hex";
#endif

#  ifdef CONFIG_BOARD_COREDUMP_COMPRESSION

  /* Initialize LZF compression stream */

  lib_lzfoutstream(&g_lzfstream, stream);
  stream = &g_lzfstream;
#  endif

  /* Do core dump */

  coredump(g_regions, stream, pid);

#  ifdef CONFIG_BOARD_COREDUMP_COMPRESSION
  _alert("Finish coredump (Compression Enabled). %s formatted\n",
         streamname);
#  else
  _alert("Finish coredump. %s formatted\n", streamname);
#  endif

  setlogmask(logmask);
}
#endif

/****************************************************************************
 * Name: coredump_dump_dev
 *
 * Description:
 *   Save coredump to storage device.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_COREDUMP_DEV
static void coredump_dump_dev(pid_t pid)
{
  FAR void *stream = &g_devstream;
  struct coredump_info_s info;
  int ret;

  if (g_devstream.inode == NULL)
    {
      _alert("Coredump device not found\n");
      return;
    }

#ifdef CONFIG_BOARD_COREDUMP_COMPRESSION
  lib_lzfoutstream(&g_lzfstream, stream);
  stream = &g_lzfstream;
#endif
  ret = coredump(g_regions, stream, pid);
  if (ret < 0)
    {
      _alert("Coredump fail %d\n", ret);
      return;
    }

  info.magic = COREDUMP_MAGIC;
  info.size  = g_devstream.common.nput;
  clock_gettime(CLOCK_REALTIME, &info.time);
  uname(&info.name);

  ret = lib_stream_seek(&g_devstream, -(off_t)sizeof(info), SEEK_END);
  if (ret < 0)
    {
      _alert("Coredump info seek fail %d\n", ret);
      return;
    }

  if (info.size > ret)
    {
      _alert("Coredump no enough space for info\n");
      return;
    }

  ret = lib_stream_puts(&g_devstream, &info, sizeof(info));
  if (ret < 0)
    {
      _alert("Coredump information write fail %d\n", ret);
      return;
    }

  /* Flush to ensure outstream write all data to storage device */

  ret = lib_stream_flush(&g_devstream);
  if (ret < 0)
    {
      _alert("Coredump flush fail %d\n", ret);
      return;
    }

  _alert("Finish coredump, write %zu bytes to %s\n",
         info.size, CONFIG_BOARD_COREDUMP_DEVPATH);
}
#endif

/****************************************************************************
 * Name: coredump_set_memory_region
 *
 * Description:
 *   Set do coredump memory region.
 *
 ****************************************************************************/

int coredump_set_memory_region(FAR const struct memory_region_s *region)
{
  /* Not free g_regions, because allow call this fun when crash */

  g_regions = region;
  return 0;
}

/****************************************************************************
 * Name: coredump_add_memory_region
 *
 * Description:
 *   Use coredump to dump the memory of the specified area.
 *
 ****************************************************************************/

int coredump_add_memory_region(FAR const void *ptr, size_t size,
                               uint32_t flags)
{
  FAR struct memory_region_s *region;
  size_t count = 1; /* 1 for end flag */

  if (g_regions != NULL)
    {
      region = (FAR struct memory_region_s *)g_regions;

      while (region->start < region->end)
        {
          if ((uintptr_t)ptr >= region->start &&
              (uintptr_t)ptr + size < region->end)
            {
              /* Already watched */

              return 0;
            }
          else if ((uintptr_t)ptr < region->start &&
                   (uintptr_t)ptr + size >= region->end)
            {
              /* start out of region, end out of region */

              region->start = (uintptr_t)ptr;
              region->end = (uintptr_t)ptr + size;
              return 0;
            }
          else if ((uintptr_t)ptr < region->start &&
                   (uintptr_t)ptr + size >= region->start)
            {
              /* start out of region, end in region */

              region->start = (uintptr_t)ptr;
              return 0;
            }
          else if ((uintptr_t)ptr < region->end &&
                   (uintptr_t)ptr + size >= region->end)
            {
              /* start in region, end out of region */

              region->end = (uintptr_t)ptr + size;
              return 0;
            }

          count++;
          region++;
        }

      /* Need a new region */
    }

  region = lib_malloc(sizeof(struct memory_region_s) * (count + 1));
  if (region == NULL)
    {
      return -ENOMEM;
    }

  memcpy(region, g_regions, sizeof(struct memory_region_s) * count);

  if (g_regions != NULL
#ifdef CONFIG_BOARD_MEMORY_RANGE
    && g_regions != g_memory_region
#endif
    )
    {
      lib_free((FAR void *)g_regions);
    }

  region[count - 1].start = (uintptr_t)ptr;
  region[count - 1].end = (uintptr_t)ptr + size;
  region[count - 1].flags = flags;
  region[count].start = 0;
  region[count].end = 0;
  region[count].flags = 0;

  g_regions = region;
  return 0;
}

/****************************************************************************
 * Name: coredump_initialize
 *
 * Description:
 *   Initialize the coredump facility.  Called once and only from
 *   nx_start_application.
 *
 ****************************************************************************/

int coredump_initialize(void)
{
  int ret = 0;

#ifdef CONFIG_BOARD_MEMORY_RANGE
  g_regions = g_memory_region;
#endif

#ifdef CONFIG_BOARD_COREDUMP_BLKDEV
  ret = lib_blkoutstream_open(&g_devstream,
                              CONFIG_BOARD_COREDUMP_DEVPATH);
#elif defined(CONFIG_BOARD_COREDUMP_MTDDEV)
  ret = lib_mtdoutstream_open(&g_devstream,
                              CONFIG_BOARD_COREDUMP_DEVPATH);
#endif

#ifdef CONFIG_BOARD_COREDUMP_DEV
  if (ret < 0)
    {
      _alert("%s Coredump device not found %d\n",
             CONFIG_BOARD_COREDUMP_DEVPATH, ret);
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: coredump_dump
 *
 * Description:
 *   Do coredump of the task specified by pid.
 *
 * Input Parameters:
 *   pid - The task/thread ID of the thread to dump
 *
 ****************************************************************************/

void coredump_dump(pid_t pid)
{
#ifdef CONFIG_BOARD_COREDUMP_SYSLOG
  coredump_dump_syslog(pid);
#endif

#ifdef CONFIG_BOARD_COREDUMP_DEV
  coredump_dump_dev(pid);
#endif
}

/****************************************************************************
 * Name: coredump
 *
 * Description:
 *   This function for generating core dump stream.
 *
 ****************************************************************************/

int coredump(FAR const struct memory_region_s *regions,
             FAR struct lib_outstream_s *stream,
             pid_t pid)
{
  struct elf_dumpinfo_s cinfo;
  irqstate_t flags;
  int memsegs = 0;
  int stksegs;

  cinfo.regions = regions;
  cinfo.stream  = stream;
  cinfo.pid     = pid;

  flags = enter_critical_section();

  if (cinfo.pid != INVALID_PROCESS_ID)
    {
      if (nxsched_get_tcb(cinfo.pid) == NULL)
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

  if (cinfo.regions != NULL)
    {
      for (; cinfo.regions[memsegs].start <
             cinfo.regions[memsegs].end; memsegs++);
    }

  /* Fill notes section */

  elf_emit_hdr(&cinfo, stksegs + memsegs + 1);

  /* Fill all the program information about the process for the
   * notes.  This also sets up the file header.
   */

  elf_emit_phdr(&cinfo, stksegs, memsegs);

  /* Fill note information */

  elf_emit_note(&cinfo);

  /* Align to page */

  elf_emit_align(&cinfo);

  /* Dump stack */

  elf_emit_stack(&cinfo);

  /* Dump memory segments */

  if (memsegs > 0)
    {
      elf_emit_memory(&cinfo, memsegs);
    }

  /* Flush the dump */

  elf_flush(&cinfo);

  leave_critical_section(flags);

  return OK;
}
