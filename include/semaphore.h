/****************************************************************************
 * include/semaphore.h
 *
 *   Copyright (C) 2007-2009, 2012-2013 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_SEMAPHORE_H
#define __INCLUDE_SEMAPHORE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <limits.h>
#include <time.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Value returned by sem_open() in the event of a failure. */

#define SEM_FAILED ((FAR sem_t *)NULL)

/* Bit definitions for the struct sem_s flags field */

#define PRIOINHERIT_FLAGS_DISABLE (1 << 0)  /* Bit 0: Priority inheritance
                                             * is disabled for this semaphore. */

/****************************************************************************
 * Public Type Declarations
 ****************************************************************************/

/* This structure contains information about the holder of a semaphore */

#ifdef CONFIG_PRIORITY_INHERITANCE
struct tcb_s; /* Forward reference */
struct semholder_s
{
#if CONFIG_SEM_PREALLOCHOLDERS > 0
  struct semholder_s *flink;     /* Implements singly linked list */
#endif
  FAR struct tcb_s *htcb;        /* Holder TCB */
  int16_t counts;                /* Number of counts owned by this holder */
};

#if CONFIG_SEM_PREALLOCHOLDERS > 0
#  define SEMHOLDER_INITIALIZER {NULL, NULL, 0}
#else
#  define SEMHOLDER_INITIALIZER {NULL, 0}
#endif
#endif /* CONFIG_PRIORITY_INHERITANCE */

/* This is the generic semaphore structure. */

struct sem_s
{
  volatile int16_t semcount;     /* >0 -> Num counts available */
                                 /* <0 -> Num tasks waiting for semaphore */

  /* If priority inheritance is enabled, then we have to keep track of which
   * tasks hold references to the semaphore.
   */

#ifdef CONFIG_PRIORITY_INHERITANCE
  uint8_t flags;                 /* See PRIOINHERIT_FLAGS_* definitions */
# if CONFIG_SEM_PREALLOCHOLDERS > 0
  FAR struct semholder_s *hhead; /* List of holders of semaphore counts */
# else
  struct semholder_s holder[2];  /* Slot for old and new holder */
# endif
#endif
};

typedef struct sem_s sem_t;

/* Initializers */

#ifdef CONFIG_PRIORITY_INHERITANCE
# if CONFIG_SEM_PREALLOCHOLDERS > 0
#  define SEM_INITIALIZER(c) \
    {(c), 0, NULL}               /* semcount, flags, hhead */
# else
#  define SEM_INITIALIZER(c) \
    {(c), 0, {SEMHOLDER_INITIALIZER, SEMHOLDER_INITIALIZER}} /* semcount, flags, holder[2] */
# endif
#else
#  define SEM_INITIALIZER(c) \
    {(c)}                        /* semcount */
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Counting Semaphore Interfaces (based on POSIX APIs) */

int        sem_init(FAR sem_t *sem, int pshared, unsigned int value);
int        sem_destroy(FAR sem_t *sem);
int        sem_wait(FAR sem_t *sem);
int        sem_timedwait(FAR sem_t *sem, FAR const struct timespec *abstime);
int        sem_clockwait(FAR sem_t *sem, clockid_t clockid,
                         FAR const struct timespec *abstime);
int        sem_trywait(FAR sem_t *sem);
int        sem_post(FAR sem_t *sem);
int        sem_getvalue(FAR sem_t *sem, FAR int *sval);

#ifdef CONFIG_FS_NAMED_SEMAPHORES
FAR sem_t *sem_open(FAR const char *name, int oflag, ...);
int        sem_close(FAR sem_t *sem);
int        sem_unlink(FAR const char *name);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_SEMAPHORE_H */
