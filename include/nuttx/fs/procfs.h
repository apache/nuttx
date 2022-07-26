/****************************************************************************
 * include/nuttx/fs/procfs.h
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

#ifndef __INCLUDE_NUTTX_FS_PROCFS_H
#define __INCLUDE_NUTTX_FS_PROCFS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/fs.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Data entry declaration prototypes ****************************************/

/* Procfs operations are a subset of the mountpt_operations */

struct procfs_operations
{
  /* The procfs open method differs from the driver open method
   * because it receives (1) the inode that contains the procfs
   * private data, (2) the relative path into the procfs, and (3)
   * information to manage privileges.
   */

  int     (*open)(FAR struct file *filep, FAR const char *relpath,
                  int oflags, mode_t mode);

  /* The following methods must be identical in signature and position
   * because the struct file_operations and struct mountp_operations are
   * treated like unions.
   */

  int     (*close)(FAR struct file *filep);
  ssize_t (*read)(FAR struct file *filep,
                  FAR char *buffer,
                  size_t buflen);
  ssize_t (*write)(FAR struct file *filep,
                   FAR const char *buffer,
                   size_t buflen);

  /* The two structures need not be common after this point. The following
   * are extended methods needed to deal with the unique needs of mounted
   * file systems.
   *
   * Additional open-file-specific procfs operations:
   */

  int     (*dup)(FAR const struct file *oldp,
                 FAR struct file *newp);

  /* Directory operations */

  int     (*opendir)(FAR const char *relpath,
                     FAR struct fs_dirent_s *dir);
  int     (*closedir)(FAR struct fs_dirent_s *dir);
  int     (*readdir)(FAR struct fs_dirent_s *dir, FAR struct dirent *entry);
  int     (*rewinddir)(FAR struct fs_dirent_s *dir);

  /* Operations on paths */

  int     (*stat)(FAR const char *relpath, FAR struct stat *buf);
};

/* Procfs handler prototypes ************************************************/

/* These are the types of entries that may appear in the procfs: */

enum procfs_entry_e
{
  PROCFS_UNKOWN_TYPE = 0, /* Unknown type */
  PROCFS_FILE_TYPE,       /* File type */
  PROCFS_DIR_TYPE,        /* Directory type */
};

/* This is a procfs entry that each handler should provide to supply
 * specific operations for file and directory handling.
 */

struct procfs_entry_s
{
  FAR const char *pathpattern;
  FAR const struct procfs_operations *ops;
  uint8_t type;
};

/* Specifies the common elements for an open file in the procfs
 * file system.  This structure should be sub-classed by handlers
 * to add their own specific data elements to the context.
 */

struct procfs_file_s
{
  FAR const struct procfs_entry_s *procfsentry;
};

/* The generic proc/ pseudo directory structure */

struct procfs_dir_priv_s
{
  uint8_t level;                                /* Directory level.  Currently 0 or 1 */
  uint16_t index;                               /* Index to the next directory entry */
  uint16_t nentries;                            /* Number of directory entries */
  FAR const struct procfs_entry_s *procfsentry; /* Pointer to procfs handler entry */
};

/* An entry for procfs_register_meminfo */

struct mm_heap_s;
struct procfs_meminfo_entry_s
{
  FAR const char *name;
  FAR struct mm_heap_s *heap;
  struct procfs_meminfo_entry_s *next;
#if CONFIG_MM_BACKTRACE >= 0

  /* This is dynamic control flag whether to turn on backtrace in the heap,
   * you can set it by /proc/memdump.
   */

  bool backtrace;
#endif
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: procfs_memcpy
 *
 * Description:
 *   procfs/ file data may be read by the user with different user buffer
 *   sizes to receive the data.  If the amount of data to be returned is
 *   large or if the callers receive buffer is small, then multiple read
 *   operations will be required.
 *
 *   If multiple read operations are required, then each read operation will
 *   be identical accept that file position (f_pos) will be incremented with
 *   each read:  f_pos must be incremented by the read method after each
 *   read operation to provide the 'offset' for the next read.
 *
 *   procfs_memcpy() is a helper function.  Each read() method should
 *   provide data in a local data buffer ('src' and 'srclen').  This
 *   will transfer the data to the user receive buffer ('dest' and
 *   'destlen'), respecting both (1) the size of the destination buffer so
 *   that it will write beyond the user receiver and (1) the file position,
 *   'offset'.
 *
 *   This function will skip over data until the under of bytes specified
 *   by 'offset' have been skipped.  Then it will transfer data from the
 *   the procfs/ 'src' buffer into the user receive buffer.  No more than
 *   'destlen' bytes will be transferred.
 *
 * Input Parameters:
 *   src     - The address of the intermediate procfs/ buffer containing the
 *             data to be returned.
 *   srclen  - The number of bytes of data in the 'src' buffer
 *   dest    - The address of the user's receive buffer.
 *   destlen - The size (in bytes) of the user's receive buffer.
 *   offset  - On input, this is the number of bytes to skip before returning
 *             data;  If bytes were skipped, this offset will be decremented.
 *             Data will not be transferred until this offset decrements to
 *             zero.
 *
 * Returned Value:
 *   The number of bytes actually transferred into the user's receive buffer.
 *
 ****************************************************************************/

size_t procfs_memcpy(FAR const char *src, size_t srclen,
                     FAR char *dest, size_t destlen,
                     off_t *offset);

/****************************************************************************
 * Name: procfs_snprintf
 *
 * Description:
 *   This function is same with snprintf, except return values.
 *   If buf has no enough space and output was truncated due to size limit,
 *   snprintf:        return formatted string len.
 *   procfs_snprintf: return string len which has written to buf.
 *
 * Input Parameters:
 *   Same with snprintf
 *
 * Returned Value:
 *   See Description.
 *
 ****************************************************************************/

int procfs_snprintf(FAR char *buf, size_t size,
                    FAR const IPTR char *format, ...) printflike(3, 4);

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
int procfs_register(FAR const struct procfs_entry_s *entry);
#endif

/****************************************************************************
 * Name: procfs_register_meminfo
 *
 * Description:
 *   Add a new meminfo entry to the procfs file system.
 *
 * Input Parameters:
 *   entry - Describes the entry to be registered.
 *
 ****************************************************************************/

void procfs_register_meminfo(FAR struct procfs_meminfo_entry_s *entry);

/****************************************************************************
 * Name: procfs_unregister_meminfo
 *
 * Description:
 *   Remove a meminfo entry from the procfs file system.
 *
 * Input Parameters:
 *   entry - Describes the entry to be unregistered.
 *
 ****************************************************************************/

void procfs_unregister_meminfo(FAR struct procfs_meminfo_entry_s *entry);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_FS_PROCFS_H */
