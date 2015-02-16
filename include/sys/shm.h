/****************************************************************************
 * include/sys/shm.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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
