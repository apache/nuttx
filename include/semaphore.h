/************************************************************
 * semaphore.h
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
 ************************************************************/

#ifndef __SEMAPHORE_H
#define __SEMAPHORE_H

/************************************************************
 * Included Files
 ************************************************************/

#include <sys/types.h>

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/************************************************************
 * Definitions
 ************************************************************/

/* The maximum value that a semaphore may have. */

#define SEM_MAX_VALUE 0x7fff /* Max value POSIX counting semaphore */

/* The maximum number of semaphores that a task may have */

#define SEM_NSEMS_MAX 0x7fffffff

/************************************************************
 * Public Type Declarations
 ************************************************************/

/* This is the generic semaphore structure. */

struct sem_s
{
  sint16 semcount;              /* >0 -> Num counts available */
                                /* <0 -> Num tasks waiting for semaphore */
};
typedef struct sem_s sem_t;

/************************************************************
 * Public Variables
 ************************************************************/

/************************************************************
 * Public Function Prototypes
 ************************************************************/

/* Counting Semaphore Interfaces (based on POSIX APIs) */

EXTERN int        sem_init(sem_t *sem, int pshared, unsigned int value);
EXTERN int        sem_destroy(sem_t *sem);
EXTERN FAR sem_t *sem_open(const char *name, int oflag, ...);
EXTERN int        sem_close(FAR sem_t *sem);
EXTERN int        sem_unlink(const char *name);
EXTERN int        sem_wait(sem_t *sem);
EXTERN int        sem_trywait(sem_t *sem);
EXTERN int        sem_post(sem_t *sem);
EXTERN int        sem_getvalue(sem_t *sem, int *sval);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __SEMAPHORE_H */

