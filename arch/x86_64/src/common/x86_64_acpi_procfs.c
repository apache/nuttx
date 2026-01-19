/****************************************************************************
 * arch/x86_64/src/common/x86_64_acpi_procfs.c
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
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <sys/param.h>

#include <nuttx/nuttx.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/procfs.h>
#include <nuttx/kmalloc.h>
#include <arch/acpi.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes one open "file" */

struct acpi_file_s
{
  struct procfs_file_s base;  /* Base open file structure */
  void *data;                 /* data pointer */
  size_t length;              /* data len */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* File system methods */

static int     acpi_open(struct file *filep, const char *relpath,
                         int oflags, mode_t mode);
static int     acpi_close(struct file *filep);

static ssize_t acpi_read(struct file *filep, char *buffer,
                         size_t buflen);

static int     acpi_opendir(const char *relpath,
                            struct fs_dirent_s **dir);
static int     acpi_closedir(struct fs_dirent_s *dir);
static int     acpi_readdir(struct fs_dirent_s *dir,
                            struct dirent *entry);

static int     acpi_stat(const char *relpath, struct stat *buf);

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* See fs_mount.c -- this structure is explicitly extern'ed there.
 * We use the old-fashioned kind of initializers so that this will compile
 * with any compiler.
 */

const struct procfs_operations g_acpi_operations =
{
  acpi_open,       /* open */
  acpi_close,      /* close */
  acpi_read,       /* read */
  NULL,            /* write */
  NULL,            /* poll */
  NULL,            /* dup */

  acpi_opendir,    /* opendir */
  acpi_closedir,   /* closedir */
  acpi_readdir,    /* readdir */
  NULL,            /* rewinddir */

  acpi_stat        /* stat */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char *g_acpi_files[] =
{
  "FACP",
  "DSDT",
  "APIC",
  "MCFG",
};

#if defined(CONFIG_FS_PROCFS_REGISTER)
static const struct procfs_entry_s g_acpi_procfs =
{
  "acpi",
  &g_acpi_operations,
  PROCFS_DIR_TYPE,
};

static const struct procfs_entry_s g_acpi_file_procfs =
{
  "acpi/**",
  &g_acpi_operations,
  PROCFS_FILE_TYPE,
};
#endif

/****************************************************************************
 * Name: acpi_open
 ****************************************************************************/

static int acpi_open(struct file *filep, const char *relpath,
                     int oflags, mode_t mode)
{
  struct acpi_file_s *acpifile;
  ssize_t len;

  finfo("Open '%s'\n", relpath);

  /* This PROCFS file is read-only.  Any attempt to open with write access
   * is not permitted.
   */

  if ((oflags & O_WRONLY) != 0 || (oflags & O_RDONLY) == 0)
    {
      ferr("ERROR: Only O_RDONLY supported\n");
      return -EACCES;
    }

  relpath += strlen("acpi/");

  /* Allocate a container to hold the file attributes */

  acpifile = kmm_zalloc(sizeof(struct acpi_file_s));
  if (!acpifile)
    {
      ferr("ERROR: Failed to allocate file attributes\n");
      return -ENOMEM;
    }

  len = acpi_table_get(relpath, &acpifile->data);
  acpifile->length = len;
  filep->f_pos     =  0;

  /* Save the attributes as the open-specific state in filep->f_priv */

  filep->f_priv = acpifile;

  return OK;
}

/****************************************************************************
 * Name: acpi_close
 ****************************************************************************/

static int acpi_close(struct file *filep)
{
  struct acpi_file_s *acpifile;

  /* Recover our private data from the struct file instance */

  acpifile = (struct acpi_file_s *)filep->f_priv;

  DEBUGASSERT(acpifile);

  /* Release the file attributes structure */

  kmm_free(acpifile->data);
  kmm_free(acpifile);

  filep->f_priv = NULL;
  return OK;
}

/****************************************************************************
 * Name: acpi_read
 ****************************************************************************/

static ssize_t acpi_read(struct file *filep, char *buffer,
                         size_t buflen)
{
  struct acpi_file_s *acpifile;

  acpifile = (struct acpi_file_s *)filep->f_priv;

  if (filep->f_pos > acpifile->length)
    {
      buflen = 0;
    }
  else if (filep->f_pos + buflen > acpifile->length)
    {
      buflen = acpifile->length - filep->f_pos;
    }

  memcpy(buffer, acpifile->data + filep->f_pos, buflen);
  filep->f_pos += buflen;

  return buflen;
}

/****************************************************************************
 * Name: acpi_opendir
 *
 * Description:
 *   Open a directory for read access
 *
 ****************************************************************************/

static int acpi_opendir(const char *relpath,
                        struct fs_dirent_s **dir)
{
  struct procfs_dir_priv_s *level1;

  finfo("relpath: \"%s\"\n", relpath ? relpath : "NULL");
  DEBUGASSERT(relpath);

  /* Assume that path refers to the 1st level subdirectory.  Allocate the
   * level1 the dirent structure before checking.
   */

  level1 = kmm_zalloc(sizeof(struct procfs_dir_priv_s));
  if (level1 == NULL)
    {
      ferr("ERROR: Failed to allocate the level1 directory structure\n");
      return -ENOMEM;
    }

  /* Initialize base structure components */

  level1->level    = 1;
  level1->nentries = nitems(g_acpi_files);

  *dir = (struct fs_dirent_s *)level1;
  return OK;
}

/****************************************************************************
 * Name: acpi_closedir
 *
 * Description: Close the directory listing
 *
 ****************************************************************************/

static int acpi_closedir(struct fs_dirent_s *dir)
{
  DEBUGASSERT(dir);
  kmm_free(dir);
  return OK;
}

/****************************************************************************
 * Name: acpi_readdir
 *
 * Description: Read the next directory entry
 *
 ****************************************************************************/

static int acpi_readdir(struct fs_dirent_s *dir,
                        struct dirent *entry)
{
  struct procfs_dir_priv_s *level1;
  int index;
  int fpos;

  DEBUGASSERT(dir);

  level1 = (struct procfs_dir_priv_s *)dir;

  index = level1->index;
  if (index >= level1->nentries)
    {
      /* We signal the end of the directory by returning the special
       * error -ENOENT
       */

      finfo("Entry %d: End of directory\n", index);
      return -ENOENT;
    }

  fpos   = index % nitems(g_acpi_files);

  entry->d_type = DTYPE_FILE;
  snprintf(entry->d_name, NAME_MAX + 1, "%s",
           g_acpi_files[fpos]);

  level1->index++;
  return OK;
}

/****************************************************************************
 * Name: acpi_stat
 *
 * Description: Return information about a file or directory
 *
 ****************************************************************************/

static int acpi_stat(const char *relpath, struct stat *buf)
{
  memset(buf, 0, sizeof(struct stat));

  if (strcmp(relpath, "acpi") == 0 || strcmp(relpath, "acpi/") == 0)
    {
      buf->st_mode = S_IFDIR | S_IROTH | S_IRGRP | S_IRUSR;
    }
  else
    {
      int ret = acpi_table_get(relpath + sizeof("acpi"), NULL);
      if (ret < 0)
        {
          return ret;
        }

      buf->st_mode = S_IFREG | S_IROTH | S_IRGRP | S_IRUSR;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: acpi_procfs_register
 *
 * Description:
 *   Register the acpi_procfs  procfs file system entry
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure
 *
 ****************************************************************************/
#if defined(CONFIG_FS_PROCFS_REGISTER)
int acpi_procfs_register(void)
{
  int ret;

  ret = procfs_register(&g_acpi_procfs);
  if (ret >= 0)
    {
      ret = procfs_register(&g_acpi_file_procfs);
    }

  DEBUGASSERT(ret >= 0);
  return ret;
}
#endif

