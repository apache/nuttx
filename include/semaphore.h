/****************************************************************************
 * include/semaphore.h
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

/* Values for protocol attribute */

#define SEM_PRIO_NONE             0
#define SEM_PRIO_INHERIT          1
#define SEM_PRIO_PROTECT          2

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
struct sem_s;

struct semholder_s
{
#if CONFIG_SEM_PREALLOCHOLDERS > 0
  FAR struct semholder_s *flink;  /* List of semaphore's holder            */
#endif
  FAR struct semholder_s *tlink;  /* List of task held semaphores          */
  FAR struct sem_s *sem;          /* Ths corresponding semaphore           */
  FAR struct tcb_s *htcb;         /* Ths corresponding TCB                 */
  int16_t counts;                 /* Number of counts owned by this holder */
};

#if CONFIG_SEM_PREALLOCHOLDERS > 0
#  define SEMHOLDER_INITIALIZER   {NULL, NULL, NULL, NULL, 0}
#  define INITIALIZE_SEMHOLDER(h) \
    do { \
      (h)->flink  = NULL; \
      (h)->tlink  = NULL; \
      (h)->sem    = NULL; \
      (h)->htcb   = NULL; \
      (h)->counts = 0; \
    } while (0)
#else
#  define SEMHOLDER_INITIALIZER   {NULL, NULL, NULL, 0}
#  define INITIALIZE_SEMHOLDER(h) \
    do { \
      (h)->tlink  = NULL; \
      (h)->sem    = NULL; \
      (h)->htcb   = NULL; \
      (h)->counts = 0; \
    } while (0)
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

/****************************************************************************
 * Name: sem_setprotocol
 *
 * Description:
 *    Set semaphore protocol attribute.
 *
 *    One particularly important use of this function is when a semaphore
 *    is used for inter-task communication like:
 *
 *      TASK A                 TASK B
 *      sem_init(sem, 0, 0);
 *      sem_wait(sem);
 *                             sem_post(sem);
 *      Awakens as holder
 *
 *    In this case priority inheritance can interfere with the operation of
 *    the semaphore.  The problem is that when TASK A is restarted it is a
 *    holder of the semaphore.  However, it never calls sem_post(sem) so it
 *    becomes *permanently* a holder of the semaphore and may have its
 *    priority boosted when any other task tries to acquire the semaphore.
 *
 *    The fix is to call sem_setprotocol(SEM_PRIO_NONE) immediately after
 *    the sem_init() call so that there will be no priority inheritance
 *    operations on this semaphore.
 *
 * Input Parameters:
 *    sem      - A pointer to the semaphore whose attributes are to be
 *               modified
 *    protocol - The new protocol to use
 *
 * Returned Value:
 *   This function is exposed as a non-standard application interface.  It
 *   returns zero (OK) if successful.  Otherwise, -1 (ERROR) is returned and
 *   the errno value is set appropriately.
 *
 ****************************************************************************/

int sem_setprotocol(FAR sem_t *sem, int protocol);

/****************************************************************************
 * Name: sem_getprotocol
 *
 * Description:
 *    Return the value of the semaphore protocol attribute.
 *
 * Input Parameters:
 *    sem      - A pointer to the semaphore whose attributes are to be
 *               queried.
 *    protocol - The user provided location in which to store the protocol
 *               value.
 *
 * Returned Value:
 *   This function is exposed as a non-standard application interface.  It
 *   returns zero (OK) if successful.  Otherwise, -1 (ERROR) is returned and
 *   the errno value is set appropriately.
 *
 ****************************************************************************/

int sem_getprotocol(FAR sem_t *sem, FAR int *protocol);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_SEMAPHORE_H */
