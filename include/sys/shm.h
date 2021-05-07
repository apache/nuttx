/****************************************************************************
 * include/sys/shm.h
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

#ifndef __INCLUDE_SYS_SHM_H
#define __INCLUDE_SYS_SHM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ipc.h>
#include <time.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Definitions required by POSIX */

#define SHM_RDONLY 0x01 /* Attach read-only (else read-write) */
#define SHM_RND    0x02 /* Round attach address to SHMLBA */

/* Segment low boundary address multiple */

#ifdef CONFIG_SHM_SHMLBA
#  define SHMLBA CONFIG_SHM_SHMLBA
#else
#  define SHMLBA 0x0
#endif

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* Unsigned integer used for the number of current attaches that must be
 * able to store values at least as large as a type unsigned short.
 */

typedef unsigned short shmatt_t;

struct shmid_ds
{
  struct ipc_perm shm_perm;   /* Operation permission structure */
  size_t          shm_segsz;  /* Size of segment in bytes */
  pid_t           shm_lpid;   /* Process ID of last shared memory operation */
  pid_t           shm_cpid;   /* Process ID of creator */
  shmatt_t        shm_nattch; /* Number of current attaches */
  time_t          shm_atime;  /* Time of last shmat() */
  time_t          shm_dtime;  /* Time of last shmdt() */
  time_t          shm_ctime;  /* Time of last change by shmctl() */
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

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

FAR void *shmat(int shmid, FAR const void *shmaddr, int shmflg);
int shmctl(int shmid, int cmd, FAR struct shmid_ds *buf);
int shmdt(FAR const void *shmaddr);
int shmget(key_t key, size_t size, int shmflg);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_SYS_SHM_H */
