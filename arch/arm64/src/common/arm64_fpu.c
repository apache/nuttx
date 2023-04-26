/***************************************************************************
 * arch/arm64/src/common/arm64_fpu.c
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
 ***************************************************************************/

/***************************************************************************
 * Included Files
 ***************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/stat.h>

#include <inttypes.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <fcntl.h>
#include <stdio.h>
#include <nuttx/sched.h>
#include <nuttx/arch.h>
#include <nuttx/fs/procfs.h>
#include <arch/irq.h>

#include "sched/sched.h"
#include "arm64_arch.h"
#include "arm64_vfork.h"
#include "arm64_internal.h"
#include "arm64_fatal.h"
#include "arm64_fpu.h"

/***************************************************************************
 * Pre-processor Definitions
 ***************************************************************************/

#define FPU_CALLEE_REGS     (8)
#define FPU_PROC_LINELEN    (64 * CONFIG_SMP_NCPUS)

/***************************************************************************
 * Private Types
 ***************************************************************************/

/* This structure describes one open "file" */

#ifdef CONFIG_FS_PROCFS_REGISTER

struct arm64_fpu_procfs_file_s
{
  struct procfs_file_s base; /* Base open file structure */
  unsigned int linesize;     /* Number of valid characters in line[] */

  /* Pre-allocated buffer for formatted lines */

  char line[FPU_PROC_LINELEN];
};
#endif

/***************************************************************************
 * Private Data
 ***************************************************************************/

static struct fpu_reg g_idle_thread_fpu[CONFIG_SMP_NCPUS];
static struct arm64_cpu_fpu_context g_cpu_fpu_ctx[CONFIG_SMP_NCPUS];

#ifdef CONFIG_FS_PROCFS_REGISTER

/* procfs methods */

static int arm64_fpu_procfs_open(struct file *filep, const char *relpath,
                                 int oflags, mode_t mode);
static int arm64_fpu_procfs_close(struct file *filep);
static ssize_t arm64_fpu_procfs_read(struct file *filep, char *buffer,
                                     size_t buflen);
static int arm64_fpu_procfs_stat(const char *relpath, struct stat *buf);

/* See include/nutts/fs/procfs.h
 * We use the old-fashioned kind of initializers so that this will compile
 * with any compiler.
 */

const struct procfs_operations arm64_fpu_procfs_operations =
{
  arm64_fpu_procfs_open,  /* open */
  arm64_fpu_procfs_close, /* close */
  arm64_fpu_procfs_read,  /* read */
  NULL,                   /* write */
  NULL,                   /* dup */
  NULL,                   /* opendir */
  NULL,                   /* closedir */
  NULL,                   /* readdir */
  NULL,                   /* rewinddir */
  arm64_fpu_procfs_stat   /* stat */
};

static const struct procfs_entry_s g_procfs_arm64_fpu =
{
  "fpu",
  &arm64_fpu_procfs_operations
};

#endif

/***************************************************************************
 * Private Functions
 ***************************************************************************/

/* enable FPU access trap */

static void arm64_fpu_access_trap_enable(void)
{
  uint64_t cpacr;

  cpacr = read_sysreg(cpacr_el1);
  cpacr &= ~CPACR_EL1_FPEN_NOTRAP;
  write_sysreg(cpacr, cpacr_el1);

  ARM64_ISB();
}

/* disable FPU access trap */

static void arm64_fpu_access_trap_disable(void)
{
  uint64_t cpacr;

  cpacr = read_sysreg(cpacr_el1);
  cpacr |= CPACR_EL1_FPEN_NOTRAP;
  write_sysreg(cpacr, cpacr_el1);

  ARM64_ISB();
}

#ifdef CONFIG_FS_PROCFS_REGISTER

static int arm64_fpu_procfs_open(struct file *filep, const char *relpath,
                                 int oflags, mode_t mode)
{
  struct arm64_fpu_procfs_file_s *priv;

  uinfo("Open '%s'\n", relpath);

  /* PROCFS is read-only.  Any attempt to open with any kind of write
   * access is not permitted.
   *
   * REVISIT:  Write-able proc files could be quite useful.
   */

  if (((oflags & O_WRONLY) != 0 || (oflags & O_RDONLY) == 0))
    {
      uerr("ERROR: Only O_RDONLY supported\n");
      return -EACCES;
    }

  /* Allocate the open file structure */

  priv = (struct arm64_fpu_procfs_file_s *)kmm_zalloc(
    sizeof(struct arm64_fpu_procfs_file_s));
  if (priv == NULL)
    {
      uerr("ERROR: Failed to allocate file attributes\n");
      return -ENOMEM;
    }

  /* Save the open file structure as the open-specific state in
   * filep->f_priv.
   */

  filep->f_priv = (void *)priv;
  return OK;
}

static int arm64_fpu_procfs_close(struct file *filep)
{
  struct arm64_fpu_procfs_file_s *priv;

  /* Recover our private data from the struct file instance */

  priv = (struct arm64_fpu_procfs_file_s *)filep->f_priv;
  DEBUGASSERT(priv);

  /* Release the file attributes structure */

  kmm_free(priv);
  filep->f_priv = NULL;
  return OK;
}

static ssize_t arm64_fpu_procfs_read(struct file *filep, char *buffer,
                                     size_t buflen)
{
  struct arm64_fpu_procfs_file_s *attr;
  struct arm64_cpu_fpu_context *ctx;
  off_t offset;
  int linesize;
  int ret;
  int i;

  uinfo("buffer=%p buflen=%zu\n", buffer, buflen);

  /* Recover our private data from the struct file instance */

  attr = (struct arm64_fpu_procfs_file_s *)filep->f_priv;
  DEBUGASSERT(attr);

  /* Traverse all FPU context */

  linesize = 0;
  for (i = 0; i < CONFIG_SMP_NCPUS; i++)
    {
      ctx = &g_cpu_fpu_ctx[i];
      linesize += snprintf(attr->line + linesize,
                           FPU_PROC_LINELEN,
                           "CPU%d: save: %d restore: %d "
                           "switch: %d exedepth: %d\n",
                           i, ctx->save_count, ctx->restore_count,
                           ctx->switch_count, ctx->exe_depth_count);
    }

  attr->linesize = linesize;

  /* Transfer the system up time to user receive buffer */

  offset = filep->f_pos;
  ret    = procfs_memcpy(attr->line, attr->linesize,
                         buffer, buflen, &offset);

  /* Update the file offset */

  if (ret > 0)
    {
      filep->f_pos += ret;
    }

  return ret;
}

static int arm64_fpu_procfs_stat(const char *relpath, struct stat *buf)
{
  buf->st_mode    = S_IFREG | S_IROTH | S_IRGRP | S_IRUSR;
  buf->st_size    = 0;
  buf->st_blksize = 0;
  buf->st_blocks  = 0;
  return OK;
}
#endif

/***************************************************************************
 * Public Functions
 ***************************************************************************/

void arm64_init_fpu(struct tcb_s *tcb)
{
  if (tcb->pid < CONFIG_SMP_NCPUS)
    {
      memset(&g_cpu_fpu_ctx[this_cpu()], 0,
             sizeof(struct arm64_cpu_fpu_context));
      g_cpu_fpu_ctx[this_cpu()].idle_thread = tcb;

      tcb->xcp.fpu_regs = (uint64_t *)&g_idle_thread_fpu[this_cpu()];
    }

  memset(tcb->xcp.fpu_regs, 0, sizeof(struct fpu_reg));
}

void arm64_destory_fpu(struct tcb_s *tcb)
{
  struct tcb_s *owner;

  /* save current fpu owner's context */

  owner = g_cpu_fpu_ctx[this_cpu()].fpu_owner;

  if (owner == tcb)
    {
      g_cpu_fpu_ctx[this_cpu()].fpu_owner = NULL;
    }
}

/***************************************************************************
 * Name: arm64_fpu_enter_exception
 *
 * Description:
 *   called at every time get into a exception
 *
 ***************************************************************************/

void arm64_fpu_enter_exception(void)
{
}

void arm64_fpu_exit_exception(void)
{
}

void arm64_fpu_trap(struct regs_context *regs)
{
  struct tcb_s *owner;

  UNUSED(regs);

  /* disable fpu trap access */

  arm64_fpu_access_trap_disable();

  /* save current fpu owner's context */

  owner = g_cpu_fpu_ctx[this_cpu()].fpu_owner;

  if (owner != NULL)
    {
      arm64_fpu_save((struct fpu_reg *)owner->xcp.fpu_regs);
      ARM64_DSB();
      g_cpu_fpu_ctx[this_cpu()].save_count++;
      g_cpu_fpu_ctx[this_cpu()].fpu_owner = NULL;
    }

  if (arch_get_exception_depth() > 1)
    {
      /* if get_exception_depth > 1
       * it means FPU access exception occurred in exception context
       * switch FPU owner to idle thread
       */

      owner = g_cpu_fpu_ctx[this_cpu()].idle_thread;
    }
  else
    {
      owner = (struct tcb_s *)arch_get_current_tcb();
    }

  /* restore our context */

  arm64_fpu_restore((struct fpu_reg *)owner->xcp.fpu_regs);
  g_cpu_fpu_ctx[this_cpu()].restore_count++;

  /* become new owner */

  g_cpu_fpu_ctx[this_cpu()].fpu_owner = owner;
}

void arm64_fpu_context_restore(void)
{
  struct tcb_s *new_tcb = (struct tcb_s *)arch_get_current_tcb();

  arm64_fpu_access_trap_disable();

  /* FPU trap has happened at this task */

  if (new_tcb == g_cpu_fpu_ctx[this_cpu()].fpu_owner)
    {
      arm64_fpu_access_trap_disable();
    }
  else
    {
      arm64_fpu_access_trap_enable();
    }

  g_cpu_fpu_ctx[this_cpu()].switch_count++;
}

#ifdef CONFIG_SMP
void arm64_fpu_context_save(void)
{
  struct tcb_s *tcb = (struct tcb_s *)arch_get_current_tcb();

  if (tcb == g_cpu_fpu_ctx[this_cpu()].fpu_owner)
    {
      arm64_fpu_access_trap_disable();
      arm64_fpu_save((struct fpu_reg *)tcb->xcp.fpu_regs);
      ARM64_DSB();
      g_cpu_fpu_ctx[this_cpu()].save_count++;
      g_cpu_fpu_ctx[this_cpu()].fpu_owner = NULL;
    }
}
#endif

void arm64_fpu_enable(void)
{
  irqstate_t flags = up_irq_save();

  arm64_fpu_access_trap_enable();
  up_irq_restore(flags);
}

void arm64_fpu_disable(void)
{
  irqstate_t flags = up_irq_save();

  arm64_fpu_access_trap_disable();
  up_irq_restore(flags);
}

/***************************************************************************
 * Name: up_fpucmp
 *
 * Description:
 *   Compare FPU areas from thread context.
 *
 * Input Parameters:
 *   saveregs1 - Pointer to the saved FPU registers.
 *   saveregs2 - Pointer to the saved FPU registers.
 *
 * Returned Value:
 *   True if FPU areas compare equal, False otherwise.
 *
 ***************************************************************************/

bool up_fpucmp(const void *saveregs1, const void *saveregs2)
{
  const uint64_t *regs1 = saveregs1 + XCPTCONTEXT_GP_SIZE;
  const uint64_t *regs2 = saveregs2 + XCPTCONTEXT_GP_SIZE;

  /* Only compare callee-saved registers, caller-saved registers do not
   * need to be preserved.
   */

  return memcmp(&regs1[FPU_REG_Q4], &regs2[FPU_REG_Q4],
                8 * FPU_CALLEE_REGS) == 0;
}

/***************************************************************************
 * Name: arm64_fpu_procfs_register
 *
 * Description:
 *   Register the arm64 fpu procfs file system entry
 *
 ***************************************************************************/

#ifdef CONFIG_FS_PROCFS_REGISTER
int arm64_fpu_procfs_register(void)
{
  return procfs_register(&g_procfs_arm64_fpu);
}
#endif
