/****************************************************************************
 * fs/tmpfs/fs_tmpfs.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#ifndef __FS_TMPFS_FS_TMPFS_H
#define __FS_TMPFS_FS_TMPFS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <semaphore.h>

#include <nuttx/fs/fs.h>

#ifndef CONFIG_DISABLE_MOUNTPOINT

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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

/* The generic form of a TMPFS memory object */

struct tmpfs_object_s
{
  size_t   to_alloc;     /* Allocated size of the memory object */
  uint8_t  to_type;      /* See enum tmpfs_objtype_e */
  uint8_t  to_refs;      /* Reference count */
  sem_t    to_exclsem;   /* Supports exclusive access to the object */
};

/* The form of one directory entry */

struct tmpfs_dirent_s
{
  FAR struct tmpfs_object_s *rde_object;
  FAR char *rde_name;
};

/* The form of a directory memory object */

struct tmpfs_directory_s
{
  /* First fields must match common TMPFS object layout */

  size_t   tdo_alloc;    /* Allocated size of the directory object */
  uint8_t  tdo_type;     /* See enum tmpfs_objtype_e */
  uint8_t  tdo_refs;     /* Reference count */
  sem_t    tdo_exclsem;  /* Supports exclusive access to the directory */

  /* Remaining fields are unique to a directory object */

  uint16_t tdo_nentries; /* Number of directory entries */
  struct tmpfs_dirent_s tdo_entry[1];
};

#define SIZEOF_TMPFS_DIRECTORY(n) \
  (sizeof(struct tmpfs_directory_s) + ((n) - 1)*sizeof(struct tmpfs_dirent_s))

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

  size_t   tfo_alloc;    /* Allocated size of the file object */
  uint8_t  tfo_type;     /* See enum tmpfs_objtype_e */
  uint8_t  tfo_refs;     /* Reference count */
  sem_t    tfo_exclsem;  /* Supports exclusive access to the file */

  /* Remaining fields are unique to a directory object */

  uint8_t  tfo_flags;    /* See TFO_FLAG_* definitions */
  size_t   tfo_size;     /* Valid file size */
  uint8_t  tfo_data[1];  /* File data starts here */
};

#define SIZEOF_TMPFS_FILE(n) (sizeof(struct tmpfs_file_s) + (n) - 1)

/* This structure represents one instance of a TMPFS file system */

struct tmpfs_s
{
  /* The root directory */

  FAR struct tmpfs_directory_s *r_root;

  sem_t r_exclsem;       /* Supports exclusive access to the file system */
};

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

EXTERN const struct mountpt_operations tmpfs_operations;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __FS_TMPFS_FS_TMPFS_H */
