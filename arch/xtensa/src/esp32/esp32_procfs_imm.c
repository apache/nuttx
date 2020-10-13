/****************************************************************************
 * arch/xtensa/src/esp32/esp32_procfs_imm.c
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

#include <sys/types.h>
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
#include <nuttx/fs/procfs.h>
#include <nuttx/fs/dirent.h>

#include <arch/irq.h>

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && defined(CONFIG_FS_PROCFS) && \
     defined(CONFIG_FS_PROCFS_REGISTER) && defined(CONFIG_XTENSA_IMEM_PROCFS)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IMM_LINELEN     64

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This enumeration identifies all of the thread attributes that can be
 * accessed via the procfs file system.
 */

/* This structure describes one open "file" */

struct imm_file_s
{
  struct procfs_file_s  base;  /* Base open file structure */
  unsigned int linesize;       /* Number of valid characters in line[] */
  char line[IMM_LINELEN];      /* Pre-allocated buffer for formatted lines */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* File system methods */

static int     imm_open(FAR struct file *filep, FAR const char *relpath,
                        int oflags, mode_t mode);
static int     imm_close(FAR struct file *filep);
static ssize_t imm_read(FAR struct file *filep, FAR char *buffer,
                        size_t buflen);
static int     imm_dup(FAR const struct file *oldp,
                       FAR struct file *newp);
static int     imm_stat(FAR const char *relpath, FAR struct stat *buf);

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* See include/nutts/fs/procfs.h */

static const struct procfs_operations imm_procfsoperations =
{
  imm_open,       /* open */
  imm_close,      /* close */
  imm_read,       /* read */
  NULL,           /* write */
  imm_dup,        /* dup */
  NULL,           /* opendir */
  NULL,           /* closedir */
  NULL,           /* readdir */
  NULL,           /* rewinddir */
  imm_stat        /* stat */
};

static const struct procfs_entry_s g_procfs_imm =
{
  "imm",
  &imm_procfsoperations
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imm_open
 ****************************************************************************/

static int imm_open(FAR struct file *filep, FAR const char *relpath,
                    int oflags, mode_t mode)
{
  FAR struct imm_file_s *priv;

  finfo("Open '%s'\n", relpath);

  /* PROCFS is read-only.  Any attempt to open with any kind of write
   * access is not permitted.
   *
   * REVISIT:  Write-able proc files could be quite useful.
   */

  if ((oflags & O_WRONLY) != 0 || (oflags & O_RDONLY) == 0)
    {
      ferr("ERROR: Only O_RDONLY supported\n");
      return -EACCES;
    }

  /* "imm" is the only acceptable value for the relpath */

  if (strcmp(relpath, "imm") != 0)
    {
      ferr("ERROR: relpath is '%s'\n", relpath);
      return -ENOENT;
    }

  /* Allocate a container to hold the task and attribute selection */

  priv = (FAR struct imm_file_s *)kmm_zalloc(sizeof(struct imm_file_s));
  if (!priv)
    {
      ferr("ERROR: Failed to allocate file attributes\n");
      return -ENOMEM;
    }

  /* Save the index as the open-specific state in filep->f_priv */

  filep->f_priv = (FAR void *)priv;
  return OK;
}

/****************************************************************************
 * Name: imm_close
 ****************************************************************************/

static int imm_close(FAR struct file *filep)
{
  FAR struct imm_file_s *priv;

  /* Recover our private data from the struct file instance */

  priv = (FAR struct imm_file_s *)filep->f_priv;
  DEBUGASSERT(priv);

  /* Release the file attributes structure */

  kmm_free(priv);
  filep->f_priv = NULL;
  return OK;
}

/****************************************************************************
 * Name: imm_read
 ****************************************************************************/

static ssize_t imm_read(FAR struct file *filep, FAR char *buffer,
                        size_t buflen)
{
  FAR struct imm_file_s *priv;
  size_t linesize;
  size_t copysize;
  size_t remaining;
  size_t totalsize;
  struct mallinfo mem;
  off_t offset = filep->f_pos;

  finfo("buffer=%p buflen=%d\n", buffer, (int)buflen);

  /* Recover our private data from the struct file instance */

  priv = (FAR struct imm_file_s *)filep->f_priv;
  DEBUGASSERT(priv);

  xtensa_imm_mallinfo(&mem);

  remaining = buflen;
  totalsize = 0;

  linesize = snprintf(priv->line,
                    IMM_LINELEN,
                    "             total       used       free    largest\n");
  copysize = procfs_memcpy(priv->line, linesize, buffer, remaining, &offset);
  totalsize += copysize;
  buffer    += copysize;
  remaining -= copysize;

  if (totalsize >= buflen)
    {
      return totalsize;
    }

  linesize = snprintf(priv->line,
                      IMM_LINELEN,
                      "Mem:   %11d%11d%11d%11d\n",
                      mem.arena,
                      mem.uordblks,
                      mem.fordblks,
                      mem.mxordblk);
  copysize = procfs_memcpy(priv->line, linesize, buffer, remaining, &offset);
  totalsize += copysize;

  /* Update the file offset */

  if (totalsize > 0)
    {
      filep->f_pos += totalsize;
    }

  return totalsize;
}

/****************************************************************************
 * Name: imm_dup
 ****************************************************************************/

static int imm_dup(FAR const struct file *oldp, FAR struct file *newp)
{
  FAR struct imm_file_s *oldpriv;
  FAR struct imm_file_s *newpriv;

  finfo("Dup %p->%p\n", oldp, newp);

  /* Recover our private data from the old struct file instance */

  oldpriv = (FAR struct imm_file_s *)oldp->f_priv;
  DEBUGASSERT(oldpriv);

  /* Allocate a new container to hold the task and attribute selection */

  newpriv = (FAR struct imm_file_s *)kmm_zalloc(sizeof(struct imm_file_s));
  if (!newpriv)
    {
      ferr("ERROR: Failed to allocate file attributes\n");
      return -ENOMEM;
    }

  /* The copy the file attributes from the old attributes to the new */

  memcpy(newpriv, oldpriv, sizeof(struct imm_file_s));

  /* Save the new attributes in the new file structure */

  newp->f_priv = (FAR void *)newpriv;
  return OK;
}

/****************************************************************************
 * Name: imm_stat
 ****************************************************************************/

static int imm_stat(const char *relpath, struct stat *buf)
{
  if (strcmp(relpath, "imm") != 0)
    {
      ferr("ERROR: relpath is '%s'\n", relpath);
      return -ENOENT;
    }

  buf->st_mode    = S_IFREG | S_IROTH | S_IRGRP | S_IRUSR;
  buf->st_size    = 0;
  buf->st_blksize = 0;
  buf->st_blocks  = 0;

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imm_procfs_register
 *
 * Description:
 *   Register the internal heap procfs file system entry
 *
 ****************************************************************************/

int imm_procfs_register(void)
{
  return procfs_register(&g_procfs_imm);
}

#endif /* !CONFIG_DISABLE_MOUNTPOINT && CONFIG_FS_PROCFS &&
        * CONFIG_FS_PROCFS_REGISTER && CONFIG_XTENSA_IMEM_PROCFS */
