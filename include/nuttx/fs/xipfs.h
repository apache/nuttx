/****************************************************************************
 * include/nuttx/fs/xipfs.h
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

#ifndef __INCLUDE_NUTTX_FS_XIPFS_H
#define __INCLUDE_NUTTX_FS_XIPFS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Maximum length of one path component, not including the NUL terminator.
 * Depth comes from the directory an entry belongs to rather than from its
 * name, so this bounds a component and not a whole path -- which is what
 * statfs reports it as, in f_namelen.
 */

#define XIPFS_NAME_MAX      31

/* Longest path reported back to an application, separators included.  A path
 * is bounded only by the depth of the tree, so this is a reporting limit and
 * not a filesystem one: a path longer than this is truncated from the front,
 * which keeps the part that identifies the file.
 */

#define XIPFS_PATH_MAX      127

/* Reason codes returned in struct xipfs_defrag_result_s.reason.
 *
 * The distinction that matters to the caller is transient (PINS, RAM,
 * BUDGET -- retrying later may help) versus permanent (FULL -- it will
 * not).  The caller normally asked because an allocation returned
 * -ENOSPC, so largest_free_run tells it directly whether a retry of that
 * allocation can now succeed.
 */

#define XIPFS_DEFRAG_DONE         0  /* Nothing left to compact           */
#define XIPFS_DEFRAG_BLOCKED_PINS 1  /* Blocked by a live XIP mapping     */
#define XIPFS_DEFRAG_BLOCKED_RAM  2  /* Transient resource shortage       */
#define XIPFS_DEFRAG_TIME_BUDGET  3  /* Ran out of the caller's budget    */
#define XIPFS_DEFRAG_ERROR        4  /* Media error; stopped cleanly      */
#define XIPFS_DEFRAG_BLOCKED_OPEN 5  /* Blocked by a merely open file     */

/* BLOCKED_PINS and BLOCKED_OPEN are deliberately distinct because the
 * caller resolves them differently: a pinned extent needs a module to be
 * unloaded, whereas an open one only needs a descriptor to be closed.  Issue
 * the ioctl on a descriptor for the mountpoint directory to avoid the
 * self-inflicted case: a descriptor for a file inside the volume holds that
 * file open, and the pass then reports BLOCKED_OPEN for the caller's own
 * file.
 */

/* Fault-injection mode carried in the XIPFSIOC_FAULTINJECT argument.
 *
 * CLEAN models a power loss that stops the failing write or erase before it
 * perturbs the medium at all, leaving the target exactly as it was.  TORN
 * models the harder real case: an interrupted NOR program leaves the first
 * half of a page written and the rest unprogrammed, and an interrupted erase
 * leaves the first half of a sector erased and the rest holding old
 * contents.  A torn generation is what forces the mount-time CRC to do real
 * work -- reject a half-formed generation rather than trust it -- which the
 * clean model, where an operation either happens whole or not at all, never
 * exercises.
 */

#define XIPFS_FAULT_CLEAN 0
#define XIPFS_FAULT_TORN  1

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Outcome of one xipfs defragmentation pass.  Returned by the
 * XIPFSIOC_DEFRAG ioctl.
 */

struct xipfs_defrag_result_s
{
  size_t   largest_free_run;   /* Bytes in the largest contiguous free run */
  uint32_t blocks_reclaimed;   /* Erase blocks coalesced into free space   */
  uint32_t blocks_pinned;      /* Blocks skipped because pin count > 0     */
  uint32_t extents_moved;      /* Number of atomic relocations performed   */
  int      reason;             /* One of XIPFS_DEFRAG_*                    */
};

/* Argument to the XIPFSIOC_DEFRAG ioctl */

struct xipfs_defrag_arg_s
{
  uint32_t max_ms;                          /* 0 means "no time budget" */
  struct xipfs_defrag_result_s result;      /* Filled in on return      */
};

/* One entry returned by the XIPFSIOC_LISTPINNED ioctl.  Without this,
 * "blocked by pins" is a dead end the caller cannot resolve; with it the
 * application can unload an idle module and re-trigger defrag.
 */

struct xipfs_pinned_entry_s
{
  char     path[XIPFS_PATH_MAX + 1];  /* Relative to the mountpoint */
  uint32_t start_block;
  uint32_t nblocks;
  uint32_t pincount;
};

/* Argument to the XIPFSIOC_LISTPINNED ioctl */

struct xipfs_pinned_arg_s
{
  FAR struct xipfs_pinned_entry_s *entries; /* Caller supplied array   */
  size_t   nentries;                        /* Capacity of that array  */
  size_t   count;                           /* OUT: entries filled in  */
};

/* Argument to the XIPFSIOC_EXTENTINFO ioctl.  Reports the physical
 * placement of the file behind the file descriptor, which is what the
 * contiguity invariant tests assert against.
 */

struct xipfs_extent_info_s
{
  uint32_t start_block;
  uint32_t nblocks;
  uint32_t erasesize;
  uint32_t size;
  uint32_t pincount;
  uint32_t data_start;    /* First block of the data region                */
  uint32_t data_nblocks;  /* Length of the data region                     */
  uintptr_t xipaddr;      /* Direct flash address, or 0 if not XIP capable */
};

/* Argument to the XIPFSIOC_FAULTINJECT ioctl */

struct xipfs_fault_s
{
  int32_t count;    /* Operations to allow before failing; negative disables */
  uint8_t mode;     /* XIPFS_FAULT_CLEAN or XIPFS_FAULT_TORN                 */
};

#endif /* __INCLUDE_NUTTX_FS_XIPFS_H */
