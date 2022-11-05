/****************************************************************************
 * fs/procfs/fs_procfs.c
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
#include <sys/statfs.h>
#include <sys/stat.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <fnmatch.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/sched.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/procfs.h>

#include "mount/mount.h"

/****************************************************************************
 * External Definitions
 ****************************************************************************/

extern const struct procfs_operations proc_operations;
extern const struct procfs_operations pm_operations;
extern const struct procfs_operations irq_operations;
extern const struct procfs_operations cpuload_operations;
extern const struct procfs_operations critmon_operations;
extern const struct procfs_operations meminfo_operations;
extern const struct procfs_operations memdump_operations;
extern const struct procfs_operations mempool_operations;
extern const struct procfs_operations iobinfo_operations;
extern const struct procfs_operations module_operations;
extern const struct procfs_operations uptime_operations;
extern const struct procfs_operations version_operations;
extern const struct procfs_operations tcbinfo_operations;

/* This is not good.  These are implemented in other sub-systems.  Having to
 * deal with them here is not a good coupling. What is really needed is a
 * run-time procfs registration system vs. a build time, fixed procfs
 * configuration.
 */

extern const struct procfs_operations net_procfsoperations;
extern const struct procfs_operations net_procfs_routeoperations;
extern const struct procfs_operations part_procfsoperations;
extern const struct procfs_operations mount_procfsoperations;
extern const struct procfs_operations smartfs_procfsoperations;

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Table of all known / pre-registered procfs handlers / participants. */

#ifdef CONFIG_FS_PROCFS_REGISTER
static const struct procfs_entry_s g_base_entries[] =
#else
static const struct procfs_entry_s g_procfs_entries[] =
#endif
{
#ifndef CONFIG_FS_PROCFS_EXCLUDE_PROCESS
  { "[0-9]*/**",     &proc_operations,            PROCFS_UNKOWN_TYPE },
  { "[0-9]*",        &proc_operations,            PROCFS_DIR_TYPE    },
#endif

#if defined(CONFIG_SCHED_CPULOAD) && !defined(CONFIG_FS_PROCFS_EXCLUDE_CPULOAD)
  { "cpuload",       &cpuload_operations,         PROCFS_FILE_TYPE   },
#endif

#ifdef CONFIG_SCHED_CRITMONITOR
  { "critmon",       &critmon_operations,         PROCFS_FILE_TYPE   },
#endif

#ifdef CONFIG_SCHED_IRQMONITOR
  { "irqs",          &irq_operations,             PROCFS_FILE_TYPE   },
#endif

#ifndef CONFIG_FS_PROCFS_EXCLUDE_MEMINFO
  { "meminfo",       &meminfo_operations,         PROCFS_FILE_TYPE   },
#  ifndef CONFIG_FS_PROCFS_EXCLUDE_MEMDUMP
  { "memdump",       &memdump_operations,         PROCFS_FILE_TYPE   },
#  endif
#endif

#ifndef CONFIG_FS_PROCFS_EXCLUDE_MEMPOOL
  { "mempool",       &mempool_operations,         PROCFS_FILE_TYPE   },
#endif

#if defined(CONFIG_MM_IOB) && !defined(CONFIG_FS_PROCFS_EXCLUDE_IOBINFO)
  { "iobinfo",       &iobinfo_operations,         PROCFS_FILE_TYPE   },
#endif

#if defined(CONFIG_MODULE) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MODULE)
  { "modules",       &module_operations,          PROCFS_FILE_TYPE   },
#endif

#ifndef CONFIG_FS_PROCFS_EXCLUDE_BLOCKS
  { "fs/blocks",     &mount_procfsoperations,     PROCFS_FILE_TYPE   },
#endif

#ifndef CONFIG_FS_PROCFS_EXCLUDE_MOUNT
  { "fs/mount",      &mount_procfsoperations,     PROCFS_FILE_TYPE   },
#endif

#ifndef CONFIG_FS_PROCFS_EXCLUDE_USAGE
  { "fs/usage",      &mount_procfsoperations,     PROCFS_FILE_TYPE   },
#endif

#if defined(CONFIG_FS_SMARTFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_SMARTFS)
  { "fs/smartfs**",  &smartfs_procfsoperations,   PROCFS_UNKOWN_TYPE },
#endif

#if defined(CONFIG_NET) && !defined(CONFIG_FS_PROCFS_EXCLUDE_NET)
  { "net",           &net_procfsoperations,       PROCFS_DIR_TYPE    },
#  if defined(CONFIG_NET_ROUTE) && !defined(CONFIG_FS_PROCFS_EXCLUDE_ROUTE)
  { "net/route",     &net_procfs_routeoperations, PROCFS_DIR_TYPE    },
  { "net/route/**",  &net_procfs_routeoperations, PROCFS_UNKOWN_TYPE },
#  endif
  { "net/**",        &net_procfsoperations,       PROCFS_UNKOWN_TYPE },
#endif

#if defined(CONFIG_MTD_PARTITION) && !defined(CONFIG_FS_PROCFS_EXCLUDE_PARTITIONS)
  { "partitions",    &part_procfsoperations,      PROCFS_FILE_TYPE   },
#endif

#if defined(CONFIG_PM) && defined(CONFIG_PM_PROCFS)
  { "pm",            &pm_operations,              PROCFS_DIR_TYPE    },
  { "pm/**",         &pm_operations,              PROCFS_UNKOWN_TYPE },
#endif

#ifndef CONFIG_FS_PROCFS_EXCLUDE_PROCESS
  { "self",          &proc_operations,            PROCFS_DIR_TYPE    },
  { "self/**",       &proc_operations,            PROCFS_UNKOWN_TYPE },
#endif

#if !defined(CONFIG_FS_PROCFS_EXCLUDE_UPTIME)
  { "uptime",        &uptime_operations,          PROCFS_FILE_TYPE   },
#endif

#if !defined(CONFIG_FS_PROCFS_EXCLUDE_VERSION)
  { "version",       &version_operations,         PROCFS_FILE_TYPE   },
#endif

#if defined(CONFIG_DEBUG_TCBINFO) && !defined(CONFIG_FS_PROCFS_EXCLUDE_TCBINFO)
  { "tcbinfo",       &tcbinfo_operations,         PROCFS_FILE_TYPE   },
#endif
};

#ifdef CONFIG_FS_PROCFS_REGISTER
static const uint8_t g_base_entrycount = sizeof(g_base_entries) /
                                         sizeof(struct procfs_entry_s);

static FAR struct procfs_entry_s *g_procfs_entries;
static uint8_t g_procfs_entrycount;
#else
static const uint8_t g_procfs_entrycount = sizeof(g_procfs_entries) /
                                           sizeof(struct procfs_entry_s);
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* File system methods */

static int     procfs_open(FAR struct file *filep, FAR const char *relpath,
                 int oflags, mode_t mode);
static int     procfs_close(FAR struct file *filep);
static ssize_t procfs_read(FAR struct file *filep, FAR char *buffer,
                 size_t buflen);
static ssize_t procfs_write(FAR struct file *filep, FAR const char *buffer,
                 size_t buflen);
static int     procfs_ioctl(FAR struct file *filep, int cmd,
                 unsigned long arg);

static int     procfs_dup(FAR const struct file *oldp,
                 FAR struct file *newp);
static int     procfs_fstat(FAR const struct file *filep,
                 FAR struct stat *buf);

static int     procfs_opendir(FAR struct inode *mountpt,
                 FAR const char *relpath, FAR struct fs_dirent_s **dir);
static int     procfs_closedir(FAR struct inode *mountpt,
                 FAR struct fs_dirent_s *dir);
static int     procfs_readdir(FAR struct inode *mountpt,
                 FAR struct fs_dirent_s *dir, FAR struct dirent *entry);
static int     procfs_rewinddir(FAR struct inode *mountpt,
                 FAR struct fs_dirent_s *dir);

static int     procfs_bind(FAR struct inode *blkdriver,
                 FAR const void *data, FAR void **handle);
static int     procfs_unbind(FAR void *handle, FAR struct inode **blkdriver,
                 unsigned int flags);
static int     procfs_statfs(FAR struct inode *mountpt,
                 FAR struct statfs *buf);

static int     procfs_stat(FAR struct inode *mountpt,
                 FAR const char *relpath, FAR struct stat *buf);

/* Initialization */

#ifdef CONFIG_FS_PROCFS_REGISTER
static int     procfs_initialize(void);
#endif

/****************************************************************************
 * Public Data
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
  procfs_write,      /* write */
  NULL,              /* seek */
  procfs_ioctl,      /* ioctl */

  NULL,              /* sync */
  procfs_dup,        /* dup */
  procfs_fstat,      /* fstat */
  NULL,              /* fchstat */
  NULL,              /* truncate */

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
  procfs_stat,       /* stat */
  NULL               /* chstat */
};

/* Level 0 contains the directory of active tasks in addition to other
 * statically registered entries with custom handlers.  This structure
 * contains a snapshot of the active tasks when the directory is first
 * opened.
 */

struct procfs_level0_s
{
  struct procfs_dir_priv_s base;         /* Base struct for ProcFS dir */

  /* Our private data */

  uint8_t lastlen;                       /* length of last reported static dir */
  pid_t pid[CONFIG_FS_PROCFS_MAX_TASKS]; /* Snapshot of all active task IDs */
  FAR const char *lastread;              /* Pointer to last static dir read */
};

/* Level 1 is an internal virtual directory (such as /proc/fs) which
 * will contain one or more additional static entries based on the
 * configuration.
 */

struct procfs_level1_s
{
  struct procfs_dir_priv_s base;     /* Base struct for ProcFS dir */

  /* Our private data */

  uint8_t lastlen;                   /* length of last reported static dir */
  uint8_t subdirlen;                 /* Length of the subdir search */
  uint16_t firstindex;               /* Index of 1st entry matching this subdir */
  FAR const char *lastread;          /* Pointer to last static dir read */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifndef CONFIG_FS_PROCFS_EXCLUDE_PROCESS

/****************************************************************************
 * Name: procfs_enum
 ****************************************************************************/

static void procfs_enum(FAR struct tcb_s *tcb, FAR void *arg)
{
  FAR struct procfs_level0_s *dir = (FAR struct procfs_level0_s *)arg;
  int index;

  DEBUGASSERT(dir);

  /* Add the PID to the list */

  index = dir->base.nentries;
  if (index >= CONFIG_FS_PROCFS_MAX_TASKS)
    {
      return;
    }

  dir->pid[index] = tcb->pid;
  dir->base.nentries = index + 1;
}
#endif

/****************************************************************************
 * Name: procfs_open
 ****************************************************************************/

static int procfs_open(FAR struct file *filep, FAR const char *relpath,
                       int oflags, mode_t mode)
{
  int x;
  int ret = -ENOENT;

  finfo("Open '%s'\n", relpath);

  /* Perform the stat based on the procfs_entry operations */

  for (x = 0; x < g_procfs_entrycount; x++)
    {
      /* Test if the path matches this entry's specification */

      if (fnmatch(g_procfs_entries[x].pathpattern, relpath, 0) == 0)
        {
          /* Match found!  Stat using this procfs entry */

          DEBUGASSERT(g_procfs_entries[x].ops &&
                      g_procfs_entries[x].ops->open);

          ret = g_procfs_entries[x].ops->open(filep, relpath, oflags, mode);
          if (ret == OK)
            {
              DEBUGASSERT(filep->f_priv);

              ((FAR struct procfs_file_s *)filep->f_priv)->procfsentry =
                                    &g_procfs_entries[x];
              break;
            }
        }
    }

  return ret;
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

  kmm_free(attr);
  filep->f_priv = NULL;
  return OK;
}

/****************************************************************************
 * Name: procfs_read
 ****************************************************************************/

static ssize_t procfs_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  FAR struct procfs_file_s *handler;

  finfo("buffer=%p buflen=%d\n", buffer, (int)buflen);

  /* Recover our private data from the struct file instance */

  handler = (FAR struct procfs_file_s *)filep->f_priv;
  DEBUGASSERT(handler);

  /* Call the handler's read routine */

  return handler->procfsentry->ops->read(filep, buffer, buflen);
}

/****************************************************************************
 * Name: procfs_write
 ****************************************************************************/

static ssize_t procfs_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen)
{
  FAR struct procfs_file_s *handler;

  finfo("buffer=%p buflen=%d\n", buffer, (int)buflen);

  /* Recover our private data from the struct file instance */

  handler = (FAR struct procfs_file_s *)filep->f_priv;
  DEBUGASSERT(handler);

  /* Call the handler's read routine */

  if (handler->procfsentry->ops->write)
    {
      return handler->procfsentry->ops->write(filep, buffer, buflen);
    }

  return 0;
}

/****************************************************************************
 * Name: procfs_ioctl
 ****************************************************************************/

static int procfs_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  finfo("cmd: %d arg: %08lx\n", cmd, arg);

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

  finfo("Dup %p->%p\n", oldp, newp);

  /* Recover our private data from the old struct file instance */

  oldattr = (FAR struct procfs_file_s *)oldp->f_priv;
  DEBUGASSERT(oldattr);

  /* Allow lower-level handler do the dup to get it's extra data */

  return oldattr->procfsentry->ops->dup(oldp, newp);
}

/****************************************************************************
 * Name: procfs_fstat
 *
 * Description:
 *   Obtain information about an open file associated with the file
 *   descriptor 'fd', and will write it to the area pointed to by 'buf'.
 *
 ****************************************************************************/

static int procfs_fstat(FAR const struct file *filep, FAR struct stat *buf)
{
  FAR struct procfs_file_s *handler;

  finfo("buf=%p\n", buf);

  /* Recover our private data from the struct file instance */

  handler = (FAR struct procfs_file_s *)filep->f_priv;
  DEBUGASSERT(handler);

  /* The procfs file system contains only directory and data file entries.
   * Since the file has been opened, we know that this is a data file and,
   * at a minimum, readable.
   */

  memset(buf, 0, sizeof(struct stat));
  buf->st_mode = S_IFREG | S_IROTH | S_IRGRP | S_IRUSR;

  /* If the write method is provided, then let's also claim that the file is
   * writable.
   */

  if (handler->procfsentry->ops->write != NULL)
    {
      buf->st_mode |= S_IWOTH | S_IWGRP | S_IWUSR;
    }

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
                          FAR struct fs_dirent_s **dir)
{
  FAR struct procfs_level0_s *level0;

  finfo("relpath: \"%s\"\n", relpath ? relpath : "NULL");
  DEBUGASSERT(mountpt && dir && relpath);

  /* The relative must be either:
   *
   * ""      - The top level directory of task/thread IDs
   * "<pid>" - The sub-directory of task/thread attributes
   */

  if (!relpath || relpath[0] == '\0')
    {
      /* The path refers to the top level directory.  Allocate the level0
       * dirent structure.
       */

      level0 = (FAR struct procfs_level0_s *)
         kmm_zalloc(sizeof(struct procfs_level0_s));
      if (!level0)
        {
          ferr("ERROR: Failed to allocate the level0 directory structure\n");
          return -ENOMEM;
        }

      /* Take a snapshot of all currently active tasks.  Any new tasks
       * added between the opendir() and closedir() call will not be
       * visible.
       *
       * NOTE that interrupts must be disabled throughout the traversal.
       */

#ifndef CONFIG_FS_PROCFS_EXCLUDE_PROCESS
      nxsched_foreach(procfs_enum, level0);
#else
      level0->base.index = 0;
      level0->base.nentries = 0;
#endif

      /* Initialize lastread entries */

      level0->lastread = "";
      level0->lastlen = 0;
      level0->base.procfsentry = NULL;

      *dir = (FAR struct fs_dirent_s *)level0;
    }
  else
    {
      int x;
      int ret;
      int len = strlen(relpath);

      /* Search the static array of procfs_entries */

      for (x = 0; x < g_procfs_entrycount; x++)
        {
          /* Test if the path matches this entry's specification */

          if (fnmatch(g_procfs_entries[x].pathpattern, relpath, 0) == 0)
            {
              /* Match found!  Call the handler's opendir routine.  If
               * successful, this opendir routine will create an entry
               * derived from struct procfs_dir_priv_s as dir.
               */

              DEBUGASSERT(g_procfs_entries[x].ops != NULL &&
                          g_procfs_entries[x].ops->opendir != NULL);

              ret = g_procfs_entries[x].ops->opendir(relpath, dir);
              if (ret == OK)
                {
                  FAR struct procfs_dir_priv_s *dirpriv;

                  DEBUGASSERT(*dir);

                  /* Set the procfs_entry handler */

                  dirpriv = (FAR struct procfs_dir_priv_s *)(*dir);
                  dirpriv->procfsentry = &g_procfs_entries[x];
                }

              return ret;
            }

          /* Test for a sub-string match (e.g. "ls /proc/fs") */

          else if (strncmp(g_procfs_entries[x].pathpattern, relpath,
                           len) == 0)
            {
              FAR struct procfs_level1_s *level1;

              /* Doing an intermediate directory search */

              /* The path refers to the top level directory.  Allocate
               * the level1 dirent structure.
               */

              level1 = (FAR struct procfs_level1_s *)
                 kmm_zalloc(sizeof(struct procfs_level1_s));
              if (!level1)
                {
                  ferr("ERROR: Failed to allocate the level0 directory "
                       "structure\n");
                  return -ENOMEM;
                }

              level1->base.level = 1;
              level1->base.index = x;
              level1->firstindex = x;
              level1->subdirlen = len;
              level1->lastread = "";
              level1->lastlen = 0;
              level1->base.procfsentry = NULL;

              *dir = (FAR struct fs_dirent_s *)level1;
              break;
            }
        }

      if (x == g_procfs_entrycount)
        {
          return -ENOENT;
        }
    }

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
  DEBUGASSERT(mountpt && dir);
  kmm_free(dir);
  return OK;
}

/****************************************************************************
 * Name: procfs_readdir
 *
 * Description: Read the next directory entry
 *
 ****************************************************************************/

static int procfs_readdir(FAR struct inode *mountpt,
                          FAR struct fs_dirent_s *dir,
                          FAR struct dirent *entry)
{
  FAR const struct procfs_entry_s *pentry = NULL;
  FAR struct procfs_dir_priv_s *priv;
  FAR struct procfs_level0_s *level0;
  FAR const char *name = NULL;
  unsigned int index;
  int ret = -ENOENT;

  DEBUGASSERT(mountpt && dir);
  priv = (FAR struct procfs_dir_priv_s *)dir;

  /* Are we reading the 1st directory level with dynamic PID and static
   * entries?
   */

  if (priv->level == 0)
    {
      level0 = (FAR struct procfs_level0_s *)priv;

      /* Have we reached the end of the PID information */

      index = priv->index;
      if (index >= priv->nentries)
        {
          index -= priv->nentries;

          /* We must report the next static entry ... no more PID entries.
           * skip any entries with wildcards in the first segment of the
           * directory name.
           */

          while (index < g_procfs_entrycount)
            {
              pentry = &g_procfs_entries[index];
              name = pentry->pathpattern;

              while (*name != '/' && *name != '\0')
                {
                  if (*name == '*' || *name == '[' || *name == '?')
                    {
                      /* Wildcard found.  Skip this entry */

                      index++;
                      name = NULL;
                      break;
                    }

                  name++;
                }

              /* Test if we skipped this entry */

              if (name != NULL)
                {
                  /* This entry is okay to report. Test if it has a
                   * duplicate first level name as the one we just reported.
                   * This could happen in the event of procfs_entry_s such
                   * as:
                   *
                   *    fs/smartfs
                   *    fs/nfs
                   *    fs/nxffs
                   */

                  name = g_procfs_entries[index].pathpattern;
                  if (!level0->lastlen ||
                      strncmp(name, level0->lastread, level0->lastlen) != 0)
                    {
                      /* Not a duplicate, return the first segment of this
                       * entry
                       */

                      break;
                    }
                  else
                    {
                      /* Skip this entry ... duplicate 1st level name found */

                      index++;
                    }
                }
            }

          /* Test if we are at the end of the directory */

          if (index < g_procfs_entrycount)
            {
              /* Report the next static entry */

              level0->lastlen = strcspn(name, "/");
              level0->lastread = name;
              strlcpy(entry->d_name, name, level0->lastlen + 1);

              /* If the entry is a directory type OR if the reported name is
               * only a sub-string of the entry (meaning that it contains
               * '/'), then report this entry as a directory.
               */

              if (pentry->type == PROCFS_DIR_TYPE ||
                  level0->lastlen != strlen(name))
                {
                  entry->d_type = DTYPE_DIRECTORY;
                }
              else
                {
                  entry->d_type = DTYPE_FILE;
                }

              /* Advance to next entry for the next read */

              priv->index = priv->nentries + index;
              ret = OK;
            }
        }
#ifndef CONFIG_FS_PROCFS_EXCLUDE_PROCESS
      else
        {
          /* Verify that the pid still refers to an active task/thread */

          pid_t pid = level0->pid[index];
          FAR struct tcb_s *tcb = nxsched_get_tcb(pid);
          if (!tcb)
            {
              ferr("ERROR: PID %d is no longer valid\n", (int)pid);
              return -ENOENT;
            }

          /* Save the filename=pid and file type=directory */

          entry->d_type = DTYPE_DIRECTORY;
          procfs_snprintf(entry->d_name, NAME_MAX + 1, "%d", (int)pid);

          /* Set up the next directory entry offset.  NOTE that we could use
           * the standard f_pos instead of our own private index.
           */

          level0->base.index = index + 1;
          ret = OK;
        }
#endif /* CONFIG_FS_PROCFS_EXCLUDE_PROCESS */
    }

  /* Are we reading an intermediate subdirectory? */

  else if (priv->level > 0 && priv->procfsentry == NULL)
    {
      FAR struct procfs_level1_s *level1;

      level1 = (FAR struct procfs_level1_s *)priv;

      /* Test if this entry matches.  We assume all entries of the same
       * subdirectory are listed in order in the procfs_entry array.
       */

      if (level1->base.index < g_procfs_entrycount &&
          level1->firstindex < g_procfs_entrycount &&
          strncmp(g_procfs_entries[level1->base.index].pathpattern,
                  g_procfs_entries[level1->firstindex].pathpattern,
                  level1->subdirlen) == 0)
        {
          /* This entry matches.  Report the subdir entry */

          name = &g_procfs_entries[level1->base.index].
                    pathpattern[level1->subdirlen + 1];
          level1->lastlen = strcspn(name, "/");
          level1->lastread = name;
          strlcpy(entry->d_name, name, level1->lastlen);

          /* Some of the search entries contain '**' wildcards.  When we
           * report the entry name, we must remove this wildcard search
           * specifier.
           */

          while (entry->d_name[level1->lastlen - 1] == '*')
            {
              level1->lastlen--;
            }

          entry->d_name[level1->lastlen] = '\0';

          if (name[level1->lastlen] == '/')
            {
              entry->d_type = DTYPE_DIRECTORY;
            }
          else
            {
              entry->d_type = DTYPE_FILE;
            }

          level1->base.index++;
          ret = OK;
        }
    }
  else
    {
      /* We are performing a directory search of one of the subdirectories
       * and we must let the handler perform the read.
       */

      DEBUGASSERT(priv->procfsentry && priv->procfsentry->ops->readdir);
      ret = priv->procfsentry->ops->readdir(dir, entry);
    }

  return ret;
}

/****************************************************************************
 * Name: procfs_rewindir
 *
 * Description: Reset directory read to the first entry
 *
 ****************************************************************************/

static int procfs_rewinddir(FAR struct inode *mountpt,
                            FAR struct fs_dirent_s *dir)
{
  FAR struct procfs_dir_priv_s *priv;

  DEBUGASSERT(mountpt && dir);
  priv = (FAR struct procfs_dir_priv_s *)dir;

  if (priv->level > 0 && priv->procfsentry == NULL)
    {
      priv->index = ((FAR struct procfs_level1_s *)priv)->firstindex;
    }
  else
    {
      priv->index = 0;
    }

  return OK;
}

/****************************************************************************
 * Name: procfs_bind
 *
 * Description: This implements a portion of the mount operation. This
 *  function allocates and initializes the mountpoint private data and
 *  binds the block driver inode to the filesystem private data.  The final
 *  binding of the private data (containing the block driver) to the
 *  mountpoint is performed by mount().
 *
 ****************************************************************************/

static int procfs_bind(FAR struct inode *blkdriver, FAR const void *data,
                       FAR void **handle)
{
#ifdef CONFIG_FS_PROCFS_REGISTER
  /* Make sure that we are properly initialized */

  procfs_initialize();
#endif

  return OK;
}

/****************************************************************************
 * Name: procfs_unbind
 *
 * Description: This implements the filesystem portion of the umount
 *   operation.
 *
 ****************************************************************************/

static int procfs_unbind(FAR void *handle, FAR struct inode **blkdriver,
                         unsigned int flags)
{
  return OK;
}

/****************************************************************************
 * Name: procfs_statfs
 *
 * Description: Return filesystem statistics
 *
 ****************************************************************************/

static int procfs_statfs(FAR struct inode *mountpt, FAR struct statfs *buf)
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

static int procfs_stat(FAR struct inode *mountpt, FAR const char *relpath,
                       FAR struct stat *buf)
{
  int ret = -ENOENT;

  /* Three path forms are accepted:
   *
   * ""      - The relative path refers to the top level directory
   * "<pid>" - If <pid> refers to a currently active task/thread, then it
   *   is a directory
   * "<pid>/<attr>" - If <attr> is a recognized attribute then, then it
   *   is a file.
   */

  memset(buf, 0, sizeof(struct stat));
  if (!relpath || relpath[0] == '\0')
    {
      /* The path refers to the top level directory.
       * It's a read-only directory.
       */

      buf->st_mode = S_IFDIR | S_IROTH | S_IRGRP | S_IRUSR;
      ret = OK;
    }
  else
    {
      int x;
      int len = strlen(relpath);

      /* Perform the stat based on the procfs_entry operations */

      for (x = 0; x < g_procfs_entrycount; x++)
        {
          /* Test if the path matches this entry's specification */

          if (fnmatch(g_procfs_entries[x].pathpattern, relpath, 0) == 0)
            {
              /* Match found!  Stat using this procfs entry */

              DEBUGASSERT(g_procfs_entries[x].ops &&
                          g_procfs_entries[x].ops->stat);

              return g_procfs_entries[x].ops->stat(relpath, buf);
            }

          /* Test for an internal subdirectory stat */

          else if (strncmp(g_procfs_entries[x].pathpattern, relpath,
                           len) == 0)
            {
              /* It's an internal subdirectory */

              buf->st_mode = S_IFDIR | S_IROTH | S_IRGRP | S_IRUSR;
              ret = OK;
              break;
            }
        }
    }

  return ret;
}

/****************************************************************************
 * Name: procfs_initialize
 *
 * Description:
 *   Configure the initial set of entries in the procfs file system.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure
 *
 ****************************************************************************/

#ifdef CONFIG_FS_PROCFS_REGISTER
int procfs_initialize(void)
{
  /* Are we already initialized? */

  if (g_procfs_entries == NULL)
    {
      /* No.. allocate a modifiable list of entries */

      g_procfs_entries = (FAR struct procfs_entry_s *)
        kmm_malloc(sizeof(g_base_entries));
      if (g_procfs_entries == NULL)
        {
          return -ENOMEM;
        }

      /* And copy the fixed entries into the allocated array */

      memcpy(g_procfs_entries, g_base_entries, sizeof(g_base_entries));
      g_procfs_entrycount = g_base_entrycount;
    }

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: procfs_register
 *
 * Description:
 *   Add a new entry to the procfs file system.
 *
 *   NOTE: This function should be called *prior* to mounting the procfs
 *   file system to prevent concurrency problems with the modification of
 *   the procfs data set while it is in use.
 *
 * Input Parameters:
 *   entry - Describes the entry to be registered.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure
 *
 ****************************************************************************/

#ifdef CONFIG_FS_PROCFS_REGISTER
int procfs_register(FAR const struct procfs_entry_s *entry)
{
  FAR struct procfs_entry_s *newtable;
  unsigned int newcount;
  size_t newsize;
  int ret = -ENOMEM;

  /* Make sure that we are properly initialized */

  procfs_initialize();

  /* realloc the table of procfs entries.
   *
   * REVISIT:  This reallocation may free memory previously used for the
   * procfs entry table.  If that table were actively in use, then that
   * could cause procfs logic to use a stale memory pointer!  We avoid that
   * problem by requiring that the procfs file be unmounted when the new
   * entry is added.  That requirement, however, is not enforced explicitly.
   *
   * Locking the scheduler as done below is insufficient.  As would be just
   * marking the entries as volatile.
   */

  newcount = g_procfs_entrycount + 1;
  newsize  = newcount * sizeof(struct procfs_entry_s);

  sched_lock();

  newtable = (FAR struct procfs_entry_s *)
    kmm_realloc(g_procfs_entries, newsize);
  if (newtable != NULL)
    {
      /* Copy the new entry at the end of the reallocated table */

      memcpy(&newtable[g_procfs_entrycount], entry,
             sizeof(struct procfs_entry_s));

      /* Instantiate the reallocated table */

      g_procfs_entries    = newtable;
      g_procfs_entrycount = newcount;
      ret = OK;
    }

  sched_unlock();
  return ret;
}
#endif
