/****************************************************************************
 * fs/procfs/fs_procfs.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/statfs.h>
#include <sys/stat.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/sched.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/dirent.h>

#include <arch/irq.h>

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && defined(CONFIG_FS_PROCFS)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define STATUS_LINELEN 32

#ifndef MIN
#  define MIN(a,b) ((a < b) ? a : b)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* This enumeration identifies all of the thread attributes that can be
 * accessed via the procfs file system.
 */

enum procfs_attr_e
{
  PROCFS_STATUS = 0,                 /* Task/thread status */
  PROCFS_CMDLINE,                    /* Command line */
};
#define PROCFS_NATTRS 2

/* This structure describes one open "file" */

struct procfs_file_s
{
  pid_t    pid;                      /* Task/thread ID */
  uint8_t  attr;                     /* See enum procfs_attr_e */
  char     line[STATUS_LINELEN];     /* Pre-allocated buffer for formatted lines */
};

/* The generic proc/ pseudo directory structure */

struct procfs_level_s
{
  uint8_t  level;                    /* Directory level.  Currently 0 or 1 */
  uint16_t index;                    /* Index to the next directory entry */
  uint16_t nentries;                 /* Number of directory entries */
};

/* Level 0 is the directory of active tasks */

struct procfs_level0_s
{
  uint8_t  level;                    /* Directory level.  Currently 0 or 1 */
  uint16_t index;                    /* Index to the next directory entry */
  uint16_t nentries;                 /* Number of directory entries */

  pid_t    pid[CONFIG_MAX_TASKS];    /* Snapshot of all active task IDs */
};

/* Level 1 is the directory of task attributes */

struct procfs_level1_s
{
  uint8_t  level;                    /* Directory level.  Currently 0 or 1 */
  uint16_t index;                    /* Index to the next directory entry */
  uint16_t nentries;                 /* Number of directory entries */

  pid_t    pid;                      /* ID of task for attributes */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* Helpers */

static void    procfs_enum(FAR struct tcb_s *tcb, FAR void *arg);
static int     procfs_findattr(FAR const char *attr);
static size_t  procfs_addline(FAR struct procfs_file_s *attr,
                 FAR char *buffer, size_t buflen, size_t linesize,
                 off_t *offset);
static ssize_t procfs_status(FAR struct procfs_file_s *attr,
                 FAR struct tcb_s *tcb, FAR char *buffer, size_t buflen,
                 off_t offset);
static ssize_t procfs_cmdline(FAR struct procfs_file_s *attr,
                 FAR struct tcb_s *tcb, FAR char *buffer, size_t buflen,
                 off_t offset);

/* File system methods */

static int     procfs_open(FAR struct file *filep, FAR const char *relpath,
                 int oflags, mode_t mode);
static int     procfs_close(FAR struct file *filep);
static ssize_t procfs_read(FAR struct file *filep, FAR char *buffer,
                 size_t buflen);
static int     procfs_ioctl(FAR struct file *filep, int cmd,
                 unsigned long arg);

static int     procfs_dup(FAR const struct file *oldp,
                 FAR struct file *newp);

static int     procfs_opendir(FAR struct inode *mountpt, const char *relpath,
                 FAR struct fs_dirent_s *dir);
static int     procfs_closedir(FAR struct inode *mountpt,
                 FAR struct fs_dirent_s *dir);
static int     procfs_readdir(FAR struct inode *mountpt,
                 FAR struct fs_dirent_s *dir);
static int     procfs_rewinddir(FAR struct inode *mountpt,
                 FAR struct fs_dirent_s *dir);

static int     procfs_bind(FAR struct inode *blkdriver,
                 FAR const void *data, FAR void **handle);
static int     procfs_unbind(FAR void *handle, FAR struct inode **blkdriver);
static int     procfs_statfs(FAR struct inode *mountpt,
                 FAR struct statfs *buf);

static int     procfs_stat(FAR struct inode *mountpt,
                 FAR const char *relpath, FAR struct stat *buf);

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/* See fs_mount.c -- this structure is explicitly externed there.
 * We use the old-fashioned kind of initializers so that this will compile
 * with any compiler.
 */

const struct mountpt_operations procfs_operations =
{
  procfs_open,       /* open */
  procfs_close,      /* close */
  procfs_read,       /* read */
  NULL,              /* write */
  NULL,              /* seek */
  procfs_ioctl,      /* ioctl */

  NULL,              /* sync */
  procfs_dup,        /* dup */

  procfs_opendir,    /* opendir */
  procfs_closedir,   /* closedir */
  procfs_readdir,    /* readdir */
  procfs_rewinddir,  /* rewinddir */

  procfs_bind,       /* bind */
  procfs_unbind,     /* unbind */
  procfs_statfs,     /* statfs */

  NULL,              /* unlink */
  NULL,              /* mkdir */
  NULL,              /* rmdir */
  NULL,              /* rename */
  procfs_stat        /* stat */
};

/* This is the list of all attribute strings.  Indexing is with the same
 * values as enum procfs_attr_e.
 */

static const char *g_attrstrings[PROCFS_NATTRS] =
{
  "status",
  "cmdline"
};

static const char *g_statenames[] =
{
  "Invalid",
  "Pending unlock",
  "Ready",
  "Running",
  "Inactive",
  "Semaphore wait",
#ifndef CONFIG_DISABLE_MQUEUE
  "Signal wait",
#endif
#ifndef CONFIG_DISABLE_MQUEUE
  "MQ not empty wait",
  "MQ no full wait"
#endif
};

static const char *g_ttypenames[4] =
{
  "Task",
  "pthread",
  "Kernel thread",
  "--?--"
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: procfs_enum
 ****************************************************************************/

static void procfs_enum(FAR struct tcb_s *tcb, FAR void *arg)
{
  FAR struct procfs_level0_s *dir = (FAR struct procfs_level0_s *)arg;
  int index;

  DEBUGASSERT(dir);

  /* Add the PID to the list */

  index = dir->nentries;
  DEBUGASSERT(index < CONFIG_MAX_TASKS);

  dir->pid[index] = tcb->pid;
  dir->nentries = index + 1;
}

/****************************************************************************
 * Name: procfs_findattr
 ****************************************************************************/

static int procfs_findattr(FAR const char *attr)
{
  int i;

  /* Search every string in g_attrstrings or until a match is found */

  for (i = 0; i < PROCFS_NATTRS; i++)
    {
      if (strcmp(g_attrstrings[i], attr) == 0)
        {
          return i;
        }
    }

  /* Not found */

  return -ENOENT;
}

/****************************************************************************
 * Name: procfs_addline
 ****************************************************************************/

static size_t procfs_addline(FAR struct procfs_file_s *attr,
                             FAR char *buffer, size_t buflen,
                             size_t linesize, off_t *offset)
{
  size_t copysize;
  size_t lnoffset;

  /* Will this line take us past the offset? */

  lnoffset = *offset;
  if (linesize < lnoffset)
    {
      /* No... decrement the offset and return without doing anything */

      *offset -= linesize;
      return 0;
    }

  /* Handle the remaining offset */

  linesize -= lnoffset;
  buffer   += lnoffset;
  *offset   = 0;

  /* Copy the line into the user buffer */

  copysize = MIN(linesize, buflen);
  memcpy(buffer, &attr->line[lnoffset], copysize);
  return copysize;
}

/****************************************************************************
 * Name: procfs_status
 ****************************************************************************/

static ssize_t procfs_status(FAR struct procfs_file_s *attr,
                             FAR struct tcb_s *tcb, FAR char *buffer,
                             size_t buflen, off_t offset)
{
  FAR const char *name;
  size_t remaining;
  size_t linesize;
  size_t copysize;
  size_t totalsize;

  remaining = buflen;
  totalsize = 0;

  /* Show the task name */

#if CONFIG_TASK_NAME_SIZE > 0
  name       = tcb->name;
#else
  name       = "<noname>";
#endif 
  linesize   = snprintf(attr->line, STATUS_LINELEN, "%-12s%s\n",
                        "Name:", name);
  copysize   = procfs_addline(attr, buffer, remaining, linesize, &offset);

  totalsize += copysize;
  buffer    += copysize;
  remaining -= copysize;

  if (totalsize >= buflen)
    {
      return totalsize;
    }

  /* Show the thread type */

  linesize   = snprintf(attr->line, STATUS_LINELEN, "%-12s%s\n", "Type:",
                        g_ttypenames[(tcb->flags & TCB_FLAG_TTYPE_MASK) >>
                        TCB_FLAG_TTYPE_SHIFT]);
  copysize   = procfs_addline(attr, buffer, remaining, linesize, &offset);

  totalsize += copysize;
  buffer    += copysize;
  remaining -= copysize;

  if (totalsize >= buflen)
    {
      return totalsize;
    }

  /* Show the thread state */

  linesize   = snprintf(attr->line, STATUS_LINELEN, "%-12s%s\n", "State:",
                        g_statenames[tcb->task_state]);
  copysize   = procfs_addline(attr, buffer, remaining, linesize, &offset);

  totalsize += copysize;
  buffer    += copysize;
  remaining -= copysize;

  if (totalsize >= buflen)
    {
      return totalsize;
    }

  /* Show the thread priority */

#ifdef CONFIG_PRIORITY_INHERITANCE
  linesize   = snprintf(attr->line, STATUS_LINELEN, "%-12s%d (%d)\n", "Priority:",
                        tcb->sched_priority, tcb->base_priority);
#else
  linesize   = snprintf(attr->line, STATUS_LINELEN, "%-12s%d\n", "Priority:",
                        tcb->sched_priority);
#endif
  copysize   = procfs_addline(attr, buffer, remaining, linesize, &offset);

  totalsize += copysize;
  buffer    += copysize;
  remaining -= copysize;

  if (totalsize >= buflen)
    {
      return totalsize;
    }

  /* Show the scheduler */

  linesize   = snprintf(attr->line, STATUS_LINELEN, "%-12s%s\n", "Scheduler:",
                        tcb->flags & TCB_FLAG_ROUND_ROBIN ? "SCHED_RR" : "SCHED_FIFO");
  copysize   = procfs_addline(attr, buffer, remaining, linesize, &offset);

  totalsize += copysize;
  buffer    += copysize;
  remaining -= copysize;

  if (totalsize >= buflen)
    {
      return totalsize;
    }

  /* Show the signal mast */

#ifndef CONFIG_DISABLE_SIGNALS
  linesize = snprintf(attr->line, STATUS_LINELEN, "%-12s%08x\n", "SigMask:",
                      tcb->sigprocmask);
  copysize = procfs_addline(attr, buffer, remaining, linesize, &offset);

  totalsize += copysize;
#endif

  return totalsize;
}

/****************************************************************************
 * Name: procfs_cmdline
 ****************************************************************************/

static ssize_t procfs_cmdline(FAR struct procfs_file_s *attr,
                             FAR struct tcb_s *tcb, FAR char *buffer,
                             size_t buflen, off_t offset)
{
  FAR struct task_tcb_s *ttcb;
  FAR const char *name;
  FAR char **argv;
  size_t remaining;
  size_t linesize;
  size_t copysize;
  size_t totalsize;

  remaining = buflen;
  totalsize = 0;

  /* Show the task name */

#if CONFIG_TASK_NAME_SIZE > 0
  name       = tcb->name;
#else
  name       = "<noname>";
#endif 
  linesize   = strlen(name);
  memcpy(attr->line, name, linesize);
  copysize   = procfs_addline(attr, buffer, remaining, linesize, &offset);

  totalsize += copysize;
  buffer    += copysize;
  remaining -= copysize;

  if (totalsize >= buflen)
    {
      return totalsize;
    }

#ifndef CONFIG_DISABLE_PTHREAD
  /* Show the pthread argument */

  if ((tcb->flags & TCB_FLAG_TTYPE_MASK) == TCB_FLAG_TTYPE_PTHREAD)
    {
      FAR struct pthread_tcb_s *ptcb = (FAR struct pthread_tcb_s *)tcb;

      linesize   = snprintf(attr->line, STATUS_LINELEN, " 0x%p\n", ptcb->arg);
      copysize   = procfs_addline(attr, buffer, remaining, linesize, &offset);

      totalsize += copysize;
      buffer    += copysize;
      remaining -= copysize;

      return totalsize;
    }
#endif

  /* Show the task argument list (skipping over the name) */

  ttcb = (FAR struct task_tcb_s *)tcb;

  for (argv = ttcb->argv + 1; *argv; argv++)
    {
      linesize   = snprintf(attr->line, STATUS_LINELEN, " %s", *argv);
      copysize   = procfs_addline(attr, buffer, remaining, linesize, &offset);

      totalsize += copysize;
      buffer    += copysize;
      remaining -= copysize;

      if (totalsize >= buflen)
        {
          return totalsize;
        }
    }

  linesize   = snprintf(attr->line, STATUS_LINELEN, "\n");
  copysize   = procfs_addline(attr, buffer, remaining, linesize, &offset);

  totalsize += copysize;
  return totalsize;
}

/****************************************************************************
 * Name: procfs_open
 ****************************************************************************/

static int procfs_open(FAR struct file *filep, FAR const char *relpath,
                      int oflags, mode_t mode)
{
  FAR struct procfs_file_s *attr;
  FAR struct tcb_s *tcb;
  FAR char *ptr;
  irqstate_t flags;
  unsigned long tmp;
  pid_t pid;
  int attrndx;

  fvdbg("Open '%s'\n", relpath);

  /* PROCFS is read-only.  Any attempt to open with any kind of write
   * access is not permitted.
   *
   * REVISIT:  Write-able proc files could be quite useful.
   */

  if ((oflags & O_WRONLY) != 0 || (oflags & O_RDONLY) == 0)
    {
      fdbg("ERROR: Only O_RDONLY supported\n");
      return -EACCES;
    }

  /* The first segment of the relative path should be a task/thread ID */

  ptr = NULL;
  tmp = strtoul(relpath, &ptr, 10);

  if (!ptr || *ptr != '/')
    {
      fdbg("ERROR: Invalid path \"%s\"\n", relpath);
      return -ENOENT;
    }

  /* Skip over the slash */

  ptr++;

  /* A valid PID would be in the range of 0-32767 (0 is reserved for the
   * IDLE thread).
   */

  if (tmp >= 32768)
    {
      fdbg("ERROR: Invalid PID %ld\n", tmp);
      return -ENOENT;
    }

  /* Now verify that a task with this task/thread ID exists */

  pid = (pid_t)tmp;

  flags = irqsave();
  tcb = sched_gettcb(pid);
  irqrestore(flags);

  if (!tcb)
    {
      fdbg("ERROR: PID %d is no longer valid\n", (int)pid);
      return -ENOENT;
    }

  /* The second segment of the relpath should be a well known attribute of
   * the task/thread.
   */

  attrndx = procfs_findattr(ptr);
  if (attrndx < 0)
    {
      fdbg("ERROR: Invalid attribute %s\n", ptr);
      return -ENOENT;
    }

  /* Allocate a container to hold the task and attribute selection */

  attr = (FAR struct procfs_file_s *)kzalloc(sizeof(struct procfs_file_s));
  if (!attr)
    {
      fdbg("ERROR: Failed to allocate file attributes\n");
      return -ENOMEM;
    }

  /* Initialize the file attributes */

  attr->pid  = pid;
  attr->attr = attrndx;

  /* Save the index as the open-specific state in filep->f_priv */

  filep->f_priv = (FAR void *)attr;
  return OK;
}

/****************************************************************************
 * Name: procfs_close
 ****************************************************************************/

static int procfs_close(FAR struct file *filep)
{
  FAR struct procfs_file_s *attr;

  /* Recover our private data from the struct file instance */

  attr = (FAR struct procfs_file_s *)filep->f_priv;
  DEBUGASSERT(attr);

  /* Release the file attributes structure */

  kfree(attr);
  filep->f_priv = NULL;
  return OK;
}

/****************************************************************************
 * Name: procfs_read
 ****************************************************************************/

static ssize_t procfs_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  FAR struct procfs_file_s *attr;
  FAR struct tcb_s *tcb;
  irqstate_t flags;
  ssize_t ret;

  fvdbg("buffer=%p buflen=%d\n", buffer, (int)buflen);

  /* Recover our private data from the struct file instance */

  attr = (FAR struct procfs_file_s *)filep->f_priv;
  DEBUGASSERT(attr);

  /* Verify that the thread is still valid */

  flags = irqsave();
  tcb = sched_gettcb(attr->pid);

  if (!tcb)
    {
      fdbg("ERROR: PID %d is not valid\n", (int)attr->pid);
      irqrestore(flags);
      return -ENODEV;
    }

  /* Provide the requested data */

  switch (attr->attr)
    {
    default:
    case PROCFS_STATUS:  /* Task/thread status */
      ret = procfs_status(attr, tcb, buffer, buflen, filep->f_pos);
      break;
 
    case PROCFS_CMDLINE: /* Command line */
      ret = procfs_cmdline(attr, tcb, buffer, buflen, filep->f_pos);
      break;
    }

  irqrestore(flags);

  /* Update the file offset */

  if (ret > 0)
    {
      filep->f_pos += ret;
    }

  return ret;
}

/****************************************************************************
 * Name: procfs_ioctl
 ****************************************************************************/

static int procfs_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  fvdbg("cmd: %d arg: %08lx\n", cmd, arg);

  /* No IOCTL commands supported */

  return -ENOTTY;
}

/****************************************************************************
 * Name: procfs_dup
 *
 * Description:
 *   Duplicate open file data in the new file structure.
 *
 ****************************************************************************/

static int procfs_dup(FAR const struct file *oldp, FAR struct file *newp)
{
  FAR struct procfs_file_s *oldattr;
  FAR struct procfs_file_s *newattr;

  fvdbg("Dup %p->%p\n", oldp, newp);

  /* Recover our private data from the old struct file instance */

  oldattr = (FAR struct procfs_file_s *)oldp->f_priv;
  DEBUGASSERT(oldattr);

  /* Allocate a new container to hold the task and attribute selection */

  newattr = (FAR struct procfs_file_s *)kzalloc(sizeof(struct procfs_file_s));
  if (!newattr)
    {
      fdbg("ERROR: Failed to allocate file attributes\n");
      return -ENOMEM;
    }

  /* The copy the file attribtes from the old attributes to the new */

  memcpy(newattr, oldattr, sizeof(struct procfs_file_s));

  /* Save the new attributes in the new file structure */

  newp->f_priv = (FAR void *)newattr;
  return OK;
}

/****************************************************************************
 * Name: procfs_opendir
 *
 * Description:
 *   Open a directory for read access
 *
 ****************************************************************************/

static int procfs_opendir(FAR struct inode *mountpt, FAR const char *relpath,
                          FAR struct fs_dirent_s *dir)
{
  FAR struct tcb_s *tcb;
  FAR void *priv = NULL;
  irqstate_t flags;

  fvdbg("relpath: \"%s\"\n", relpath ? relpath : "NULL");
  DEBUGASSERT(mountpt && relpath && dir && !dir->u.procfs);

  /* The relative must be either:
   *
   * ""      - The top level directory of task/thread IDs
   * "<pid>" - The sub-directory of task/thread attributes
   */

  if (!relpath || relpath[0] == '\0')
    {
      FAR struct procfs_level0_s *level0;

      /* The path refers to the top level directory.  Allocate the level0
       * dirent structure.
       */

      level0 = (FAR struct procfs_level0_s *)
         kzalloc(sizeof(struct procfs_level0_s));

      if (!level0)
        {
          fdbg("ERROR: Failed to allocate the level0 directory structure\n");
          return -ENOMEM;
        }

      /* Take a snapshot of all currently active tasks.  Any new tasks
       * added between the opendir() and closedir() call will not be
       * visible.
       *
       * NOTE that interrupts must be disabled throughout the traversal.
       */

      flags = irqsave();
      sched_foreach(procfs_enum, level0);
      irqrestore(flags);

      priv = (FAR void *)level0;
    }
  else
    {
      FAR struct procfs_level1_s *level1;
      unsigned long tmp;
      FAR char *ptr;
      pid_t pid;

      /* Otherwise, the relative path should be a valid task/thread ID */

      ptr = NULL;
      tmp = strtoul(relpath, &ptr, 10);

      if (!ptr || (*ptr != '\0' && strcmp(ptr, "/") != 0))
        {
          /* strtoul failed or there is something in the path after the pid */

          fdbg("ERROR: Invalid path \"%s\"\n", relpath);
          return -ENOENT;
       }

      /* A valid PID would be in the range of 0-32767 (0 is reserved for the
       * IDLE thread).
       */

      if (tmp >= 32768)
        {
          fdbg("ERROR: Invalid PID %ld\n", tmp);
          return -ENOENT;
        }

      /* Now verify that a task with this task/thread ID exists */

      pid = (pid_t)tmp;

      flags = irqsave();
      tcb = sched_gettcb(pid);
      irqrestore(flags);

      if (!tcb)
        {
          fdbg("ERROR: PID %d is not valid\n", (int)pid);
          return -ENOENT;
        }

      /* Was the <pid> the final element of the path? */

      if (*ptr != '\0' && strcmp(ptr, "/") != 0)
        {
          /* There is something in the path after the pid */

          fdbg("ERROR: Invalid path \"%s\"\n", relpath);
          return -ENOENT;
        }

      /* The path refers to the 1st level sbdirectory.  Allocate the level1
       * dirent structure.
       */

      level1 = (FAR struct procfs_level1_s *)
         kzalloc(sizeof(struct procfs_level1_s));

      if (!level1)
        {
          fdbg("ERROR: Failed to allocate the level1 directory structure\n");
          return -ENOMEM;
        }

      level1->level    = 1;
      level1->nentries = PROCFS_NATTRS;
      level1->pid      = pid;

      priv = (FAR void *)level1;
    }

  dir->u.procfs = priv;
  return OK;
}

/****************************************************************************
 * Name: procfs_closedir
 *
 * Description: Close the directory listing
 *
 ****************************************************************************/

static int procfs_closedir(FAR struct inode *mountpt,
                           FAR struct fs_dirent_s *dir)
{
  FAR struct procfs_level_s *priv;

  DEBUGASSERT(mountpt && dir && dir->u.procfs);
  priv = dir->u.procfs;

  if (priv)
    {
      kfree(priv);
    }

  dir->u.procfs = NULL;
  return OK;
}

/****************************************************************************
 * Name: procfs_readdir
 *
 * Description: Read the next directory entry
 *
 ****************************************************************************/

static int procfs_readdir(struct inode *mountpt, struct fs_dirent_s *dir)
{
  FAR struct procfs_level_s *priv;
  FAR struct tcb_s *tcb;
  unsigned int index;
  irqstate_t flags;
  pid_t pid;
  int ret;

  DEBUGASSERT(mountpt && dir && dir->u.procfs);
  priv = dir->u.procfs;

  /* Have we reached the end of the directory */

  index = priv->index;
  if (index >= priv->nentries)
    {
      /* We signal the end of the directory by returning the special
       * error -ENOENT
       */

      fvdbg("Entry %d: End of directory\n", index);
      ret = -ENOENT;
    }

  /* Are tranversing a first level directory of task IDs */

  else if (priv->level == 0)
    {
      FAR struct procfs_level0_s *level0 = (FAR struct procfs_level0_s *)priv;

      /* Verify that the pid still refers to an active task/thread */

      pid = level0->pid[index];

      flags = irqsave();
      tcb = sched_gettcb(pid);
      irqrestore(flags);

      if (!tcb)
        {
          fdbg("ERROR: PID %d is no longer valid\n", (int)pid);
          return -ENOENT;
        }

      /* Save the filename=pid and file type=directory */

      dir->fd_dir.d_type = DTYPE_DIRECTORY;
      snprintf(dir->fd_dir.d_name, NAME_MAX+1, "%d", (int)pid);

      /* Set up the next directory entry offset.  NOTE that we could use the
       * standard f_pos instead of our own private index.
       */

      level0->index = index + 1;
      ret = OK;
    }

  /* No.. We must be tranversing a subdirectory of task attributes */

  else
    {
      FAR struct procfs_level1_s *level1 = (FAR struct procfs_level1_s *)priv;

      DEBUGASSERT(priv->level == 1);

      /* Verify that the pid still refers to an active task/thread */

      pid = level1->pid;

      flags = irqsave();
      tcb = sched_gettcb(pid);
      irqrestore(flags);

      if (!tcb)
        {
          fdbg("ERROR: PID %d is no longer valid\n", (int)pid);
          return -ENOENT;
        }

      /* Save the filename=pid and file type=directory */

      dir->fd_dir.d_type = DTYPE_FILE;
      strncpy(dir->fd_dir.d_name, g_attrstrings[index], NAME_MAX+1);

      /* Set up the next directory entry offset.  NOTE that we could use the
       * standard f_pos instead of our own private index.
       */

      level1->index = index + 1;
      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: procfs_rewindir
 *
 * Description: Reset directory read to the first entry
 *
 ****************************************************************************/

static int procfs_rewinddir(struct inode *mountpt, struct fs_dirent_s *dir)
{
  FAR struct procfs_level_s *priv;

  DEBUGASSERT(mountpt && dir && dir->u.procfs);
  priv = dir->u.procfs;

  priv->index = 0;
  return OK;
}

/****************************************************************************
 * Name: procfs_bind
 *
 * Description: This implements a portion of the mount operation. This
 *  function allocates and initializes the mountpoint private data and
 *  binds the blockdriver inode to the filesystem private data.  The final
 *  binding of the private data (containing the blockdriver) to the
 *  mountpoint is performed by mount().
 *
 ****************************************************************************/

static int procfs_bind(FAR struct inode *blkdriver, const void *data,
                       void **handle)
{
  return OK;
}

/****************************************************************************
 * Name: procfs_unbind
 *
 * Description: This implements the filesystem portion of the umount
 *   operation.
 *
 ****************************************************************************/

static int procfs_unbind(void *handle, FAR struct inode **blkdriver)
{
  return OK;
}

/****************************************************************************
 * Name: procfs_statfs
 *
 * Description: Return filesystem statistics
 *
 ****************************************************************************/

static int procfs_statfs(struct inode *mountpt, struct statfs *buf)
{
  /* Fill in the statfs info */

  memset(buf, 0, sizeof(struct statfs));
  buf->f_type    = PROCFS_MAGIC;
  buf->f_bsize   = 0;
  buf->f_blocks  = 0;
  buf->f_bfree   = 0;
  buf->f_bavail  = 0;
  buf->f_namelen = NAME_MAX;
  return OK;
}

/****************************************************************************
 * Name: procfs_stat
 *
 * Description: Return information about a file or directory
 *
 ****************************************************************************/

static int procfs_stat(struct inode *mountpt, const char *relpath,
                       struct stat *buf)
{
  FAR struct tcb_s *tcb;
  unsigned long tmp;
  FAR char *ptr;
  irqstate_t flags;
  pid_t pid;
  int ret;

  /* Three path forms are accepted:
   *
   * ""      - The relative path refers to the top level directory
   * "<pid>" - If <pid> refers to a currently active task/thread, then it
   *   is a directory
   * "<pid>/<attr>" - If <attr> is a recognized attribute then, then it
   *   is a file.
   */

  if (!relpath || relpath[0] == '\0')
    {
      /* The path refers to the top level directory */
      /* It's a read-only directory */

      buf->st_mode = S_IFDIR|S_IROTH|S_IRGRP|S_IRUSR;

    }
  else
    {
      /* Otherwise, the first segment of the relative path should be a valid
       * task/thread ID
       */

      ptr = NULL;
      tmp = strtoul(relpath, &ptr, 10);

      if (!ptr)
        {
          fdbg("ERROR: Invalid path \"%s\"\n", relpath);
          return -ENOENT;
       }

      /* A valid PID would be in the range of 0-32767 (0 is reserved for the
       * IDLE thread).
       */

      if (tmp >= 32768)
        {
          fdbg("ERROR: Invalid PID %ld\n", tmp);
          return -ENOENT;
        }

      /* Now verify that a task with this task/thread ID exists */

      pid = (pid_t)tmp;

      flags = irqsave();
      tcb = sched_gettcb(pid);
      irqrestore(flags);

      if (!tcb)
        {
          fdbg("ERROR: PID %d is no longer valid\n", (int)pid);
          return -ENOENT;
        }

      /* Was the <pid> the final element of the path? */

      if (*ptr == '\0' || strcmp(ptr, "/") == 0)
        {
          /* Yes ... It's a read-only directory */

          buf->st_mode = S_IFDIR|S_IROTH|S_IRGRP|S_IRUSR;
        }
      else
        {
          /* Otherwise, the second segment of the relpath should be a well
           * known attribute of the task/thread.
           */

          ret = procfs_findattr(ptr);
          if (ret < 0)
            {
              fdbg("ERROR: Invalid attribute %s\n", ptr);
              return -ENOENT;
            }

          /* It's a read-only file name */

          buf->st_mode = S_IFREG|S_IROTH|S_IRGRP|S_IRUSR;
        }
    }

  /* File/directory size, access block size */

  buf->st_size    = 0;
  buf->st_blksize = 0;
  buf->st_blocks  = 0;
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#endif /* !CONFIG_DISABLE_MOUNTPOINT && CONFIG_FS_PROCFS */
