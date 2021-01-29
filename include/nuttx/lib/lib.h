/****************************************************************************
 * include/nuttx/lib/lib.h
 * Non-standard, internal APIs available in lib/.
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

#ifndef __INCLUDE_NUTTX_LIB_LIB_H
#define __INCLUDE_NUTTX_LIB_LIB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/fs.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

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

/* Hook for library initialization.  No is needed now, however */

#define lib_initialize()

/* Functions contained in lib_streams.c *************************************/

#ifdef CONFIG_FILE_STREAM
struct task_group_s;
void lib_stream_initialize(FAR struct task_group_s *group);
void lib_stream_release(FAR struct task_group_s *group);
#endif

/* Functions defined in lib_filesem.c ***************************************/

#ifdef CONFIG_STDIO_DISABLE_BUFFERING
#  define lib_sem_initialize(s)
#  define lib_take_semaphore(s)
#  define lib_give_semaphore(s)
#else
void lib_sem_initialize(FAR struct file_struct *stream);
void lib_take_semaphore(FAR struct file_struct *stream);
void lib_give_semaphore(FAR struct file_struct *stream);
#endif

/* Functions defined in lib_srand.c *****************************************/

unsigned long nrand(unsigned long limit);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __INCLUDE_NUTTX_LIB_LIB_H */
