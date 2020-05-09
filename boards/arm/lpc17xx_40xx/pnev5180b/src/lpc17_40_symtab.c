/*****************************************************************************
 * boards/arm/lpc17xx_40xx/pnev5180b/src/lpc17_40_symtab.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Michael Jung <mijung@gmx.net>
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
 *****************************************************************************/

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <nuttx/compiler.h>
#include <nuttx/binfmt/symtab.h>

/*****************************************************************************
 * Public Data
 *****************************************************************************/

extern void *exit;
extern void *fflush;
extern void *fopen;
extern void *fprintf;
extern void *getpid;
extern void *kill;
extern void *memset;
extern void *printf;
extern void *pthread_attr_init;
extern void *pthread_create;
extern void *pthread_exit;
extern void *pthread_join;
extern void *pthread_mutex_init;
extern void *pthread_mutex_lock;
extern void *pthread_mutex_unlock;
extern void *puts;
extern void *nxsched_get_streams;
extern void *sem_destroy;
extern void *sem_init;
extern void *sem_post;
extern void *sem_wait;
extern void *sigaction;
extern void *sigemptyset;
extern void *sigqueue;
extern void *sleep;
extern void *strcmp;
extern void *task_create;
extern void *usleep;

const struct symtab_s lpc17_40_exports[] =
{
  {"exit", &exit},
  {"fflush", &fflush},
  {"fopen", &fopen},
  {"fprintf", &fprintf},
  {"getpid", &getpid},
  {"kill", &kill},
  {"memset", &memset},
  {"printf", &printf},
  {"pthread_attr_init", &pthread_attr_init},
  {"pthread_create", &pthread_create},
  {"pthread_exit", &pthread_exit},
  {"pthread_join", &pthread_join},
  {"pthread_mutex_init", &pthread_mutex_init},
  {"pthread_mutex_lock", &pthread_mutex_lock},
  {"pthread_mutex_unlock", &pthread_mutex_unlock},
  {"puts", &puts},
  {"nxsched_get_streams", &nxsched_get_streams},
  {"sem_destroy", &sem_destroy},
  {"sem_init", &sem_init},
  {"sem_post", &sem_post},
  {"sem_wait", &sem_wait},
  {"sigaction", &sigaction},
  {"sigemptyset", &sigemptyset},
  {"sigqueue", &sigqueue},
  {"sleep", &sleep},
  {"strcmp", &strcmp},
  {"task_create", &task_create},
  {"usleep", &usleep},
};

const int lpc17_40_nexports =
  sizeof(lpc17_40_exports) / sizeof(struct symtab_s);

/*****************************************************************************
 * Public Functions
 *****************************************************************************/
