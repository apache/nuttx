/****************************************************************************
 * boards/arm/lpc17xx_40xx/pnev5180b/src/lpc17_40_symtab.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>
#include <nuttx/binfmt/symtab.h>

/****************************************************************************
 * Public Data
 ****************************************************************************/

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
extern void *lib_get_streams;
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
  {"lib_get_streams", &lib_get_streams},
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

/****************************************************************************
 * Public Functions
 ****************************************************************************/
