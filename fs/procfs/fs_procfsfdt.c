/****************************************************************************
 * fs/procfs/fs_procfsfdt.c
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
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <endian.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fdt.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/procfs.h>

#if defined(CONFIG_DEVICE_TREE) && !defined(CONFIG_FS_PROCFS_EXCLUDE_FDT)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes one open "file" */

struct fdt_file_s
{
  struct procfs_file_s base;        /* Base open file structure */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* File system methods */

static int     fdt_open(FAR struct file *filep, FAR const char *relpath,
                        int oflags, mode_t mode);
static int     fdt_close(FAR struct file *filep);
static ssize_t fdt_read(FAR struct file *filep, FAR char *buffer,
                        size_t buflen);
static int     fdt_dup(FAR const struct file *oldp,
                       FAR struct file *newp);
static int     fdt_stat(FAR const char *relpath, FAR struct stat *buf);

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* See fs_mount.c -- this structure is explicitly externed there.
 * We use the old-fashioned kind of initializers so that this will compile
 * with any compiler.
 */

const struct procfs_operations g_fdt_operations =
{
  fdt_open,       /* open */
  fdt_close,      /* close */
  fdt_read,       /* read */
  NULL,           /* write */
  NULL,           /* poll */
  fdt_dup,        /* dup */
  NULL,           /* opendir */
  NULL,           /* closedir */
  NULL,           /* readdir */
  NULL,           /* rewinddir */
  fdt_stat        /* stat */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fdt_open
 ****************************************************************************/

static int fdt_open(FAR struct file *filep, FAR const char *relpath,
                    int oflags, mode_t mode)
{
  FAR struct fdt_file_s *attr;

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

  /* Allocate a container to hold the file attributes */

  attr = kmm_zalloc(sizeof(struct fdt_file_s));
  if (attr == NULL)
    {
      ferr("ERROR: Failed to allocate file attributes\n");
      return -ENOMEM;
    }

  /* Save the attributes as the open-specific state in filep->f_priv */

  filep->f_priv = attr;
  return OK;
}

/****************************************************************************
 * Name: fdt_close
 ****************************************************************************/

static int fdt_close(FAR struct file *filep)
{
  FAR struct fdt_file_s *attr;

  /* Recover our private data from the struct file instance */

  attr = filep->f_priv;
  DEBUGASSERT(attr);

  /* Release the file attributes structure */

  kmm_free(attr);
  filep->f_priv = NULL;
  return OK;
}

/****************************************************************************
 * Name: fdt_read
 ****************************************************************************/

static ssize_t fdt_read(FAR struct file *filep, FAR char *buffer,
                        size_t buflen)
{
  FAR const char *fdt;
  FAR struct fdt_header_s *fdt_header;
  off_t offset;
  ssize_t ret;

  finfo("buffer=%p buflen=%zu\n", buffer, buflen);
  DEBUGASSERT(buffer != NULL && buflen > 0);

  /* Load FDT and parse extents. */

  fdt = fdt_get();
  if (fdt == NULL)
    {
      ferr("FDT cannot be read.\n");
      return -ENOENT;
    }

  /* Transfer the fdt to user receive buffer */

  fdt_header = (struct fdt_header_s *)fdt;
  offset = filep->f_pos;
  ret = procfs_memcpy(fdt, be32toh(fdt_header->totalsize),
                      buffer, buflen, &offset);

  /* Update the file offset */

  if (ret > 0)
    {
      filep->f_pos += ret;
    }

  return ret;
}

/****************************************************************************
 * Name: fdt_dup
 *
 * Description:
 *   Duplicate open file data in the new file structure.
 *
 ****************************************************************************/

static int fdt_dup(FAR const struct file *oldp, FAR struct file *newp)
{
  FAR struct fdt_file_s *oldattr;
  FAR struct fdt_file_s *newattr;

  finfo("Dup %p->%p\n", oldp, newp);

  /* Recover our private data from the old struct file instance */

  oldattr = oldp->f_priv;
  DEBUGASSERT(oldattr);

  /* Allocate a new container to hold the task and attribute selection */

  newattr = kmm_malloc(sizeof(struct fdt_file_s));
  if (newattr == NULL)
    {
      ferr("ERROR: Failed to allocate file attributes\n");
      return -ENOMEM;
    }

  /* The copy the file attributes from the old attributes to the new */

  memcpy(newattr, oldattr, sizeof(struct fdt_file_s));

  /* Save the new attributes in the new file structure */

  newp->f_priv = newattr;
  return OK;
}

/****************************************************************************
 * Name: fdt_stat
 *
 * Description: Return information about a file or directory
 *
 ****************************************************************************/

static int fdt_stat(FAR const char *relpath, FAR struct stat *buf)
{
  /* "fdt" is the name for a read-only file */

  memset(buf, 0, sizeof(struct stat));
  buf->st_mode = S_IFREG | S_IROTH | S_IRGRP | S_IRUSR;
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#endif /* CONFIG_DEVICE_TREE && CONFIG_FS_PROCFS_EXCLUDE_FDT */
