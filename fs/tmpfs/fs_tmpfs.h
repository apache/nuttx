/****************************************************************************
 * fs/tmpfs/fs_tmpfs.h
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

#ifndef __FS_TMPFS_FS_TMPFS_H
#define __FS_TMPFS_FS_TMPFS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/fs/fs.h>
#include <nuttx/mutex.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Bit definitions for file object flags */

#define TFO_FLAG_UNLINKED (1 << 0)  /* Bit 0: File is unlinked */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* TMPFS memory object types */

enum tmpfs_objtype_e
{
  TMPFS_DIRECTORY = 0,  /* Directory */
  TMPFS_REGULAR         /* Regular file */
};

/* Values returned by tmpfs_foreach() */

enum tmpfs_foreach_e
{
  TMPFS_CONTINUE = 0,    /* Continue enumeration */
  TMPFS_HALT,            /* Stop enumeration */
  TMPFS_DELETED,         /* Object and directory entry deleted */
  TMPFS_UNLINKED         /* Only the directory entry was deleted */
};

/* The form of one directory entry */

struct tmpfs_dirent_s
{
  FAR struct tmpfs_object_s *tde_object;
  FAR char *tde_name;
};

/* The generic form of a TMPFS memory object */

struct tmpfs_object_s
{
  rmutex_t to_lock;

  size_t   to_alloc;     /* Allocated size of the memory object */
  uint8_t  to_type;      /* See enum tmpfs_objtype_e */
  uint8_t  to_refs;      /* Reference count */
};

/* The form of a directory memory object */

struct tmpfs_directory_s
{
  /* First fields must match common TMPFS object layout */

  rmutex_t tdo_lock;

  size_t   tdo_alloc;    /* Allocated size of the directory object */
  uint8_t  tdo_type;     /* See enum tmpfs_objtype_e */
  uint8_t  tdo_refs;     /* Reference count */

  /* Remaining fields are unique to a directory object */

  uint16_t tdo_nentries; /* Number of directory entries */
  FAR struct tmpfs_dirent_s *tdo_entry;
};

#define SIZEOF_TMPFS_DIRECTORY(n) ((n) * sizeof(struct tmpfs_dirent_s))

/* The form of a regular file memory object
 *
 * NOTE that in this very simplified implementation, there is no per-open
 * state.  The file memory object also serves as the open file object,
 * saving an allocation.  This has the negative side effect that no per-
 * open state can be retained (such as open flags).
 */

struct tmpfs_file_s
{
  /* First fields must match common TMPFS object layout */

  rmutex_t tfo_lock;

  size_t   tfo_alloc;    /* Allocated size of the file object */
  uint8_t  tfo_type;     /* See enum tmpfs_objtype_e */
  uint8_t  tfo_refs;     /* Reference count */

  /* Remaining fields are unique to a directory object */

  uint8_t       tfo_flags; /* See TFO_FLAG_* definitions */
  size_t        tfo_size;  /* Valid file size */
  FAR uint8_t  *tfo_data;  /* File data starts here */
};

/* This structure represents one instance of a TMPFS file system */

struct tmpfs_s
{
  /* The root directory */

  FAR struct tmpfs_dirent_s tfs_root;
  rmutex_t tfs_lock;
};

/* This is the type used the tmpfs_statfs_callout to accumulate memory
 * usage
 */

struct tmpfs_statfs_s
{
  size_t tsf_alloc;      /* Total memory allocated */
  size_t tsf_avail;      /* Total memory available */
  off_t  tsf_files;      /* Total file nodes in the file system */
  off_t  tsf_ffree;      /* Free directory nodes in the file system */
};

/* This is the type of the for tmpfs_foreach callback */

typedef int (*tmpfs_foreach_t)(FAR struct tmpfs_directory_s *tdo,
                               unsigned int index, FAR void *arg);

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

EXTERN const struct mountpt_operations g_tmpfs_operations;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __FS_TMPFS_FS_TMPFS_H */
