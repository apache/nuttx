/****************************************************************************
 * arch/arm/include/cxd56xx/crashdump.h
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

#ifndef __ARCH_ARM_INCLUDE_CXD56XX_CRASHDUMP_H
#define __ARCH_ARM_INCLUDE_CXD56XX_CRASHDUMP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/irq.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* Configuration ************************************************************/

/* The following guides in the amount of the user and interrupt stack
 * data we can save. The amount of storage left will dictate the actual
 * number of entries of the user stack data saved.
 */

#define CRASHLOG_SIZE 1024
#if CONFIG_ARCH_INTERRUPTSTACK <= 3
#  define NUMBER_STACKS 1
#else
#  define NUMBER_STACKS 2
#endif
#define CRASHLOG_LEFTOVER  (CRASHLOG_SIZE - sizeof(info_t))
#define CONFIG_ISTACK_SIZE (CRASHLOG_LEFTOVER / NUMBER_STACKS / \
                            sizeof(stack_word_t))
#define CONFIG_USTACK_SIZE (CRASHLOG_LEFTOVER / NUMBER_STACKS / \
                            sizeof(stack_word_t))

#define ARRAYSIZE(a) (sizeof((a))/sizeof(a[0]))

/* For Assert keep this much of the file name */

#define MAX_FILE_PATH_LENGTH 40

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* Used for stack frame storage */

typedef uint32_t stack_word_t;

/* Stack related data */

typedef struct
{
  uint32_t sp;
  uint32_t top;
  uint32_t size;
} _stack_t;

typedef struct
{
  _stack_t user;
#if CONFIG_ARCH_INTERRUPTSTACK > 3
  _stack_t interrupt;
#endif
} stack_t;

/* Flags to identify what is in the dump */

typedef enum
{
  REGS_PRESENT          = 0x01,
  USERSTACK_PRESENT     = 0x02,
  INTSTACK_PRESENT      = 0x04,
  INVALID_USERSTACK_PTR = 0x20,
  INVALID_INTSTACK_PTR  = 0x40,
} fault_flags_t;

typedef struct
{
  struct timespec ts;                   /* timestamp */
  fault_flags_t flags;                  /* What is in the dump */
  uintptr_t     current_regs;           /* Used to validate the dump */
  int           lineno;                 /* __LINE__ to up_assert */
  pid_t         pid;                    /* Process ID */
  uint32_t      regs[XCPTCONTEXT_REGS]; /* Interrupt register save area */
  stack_t       stacks;                 /* Stack info */
#if CONFIG_TASK_NAME_SIZE > 0
  char          name[CONFIG_TASK_NAME_SIZE + 1]; /* Task name (with NULL
                                                  * terminator) */
#endif
  char          filename[MAX_FILE_PATH_LENGTH];  /* the Last of chars in
                                                  * __FILE__ to up_assert */
} info_t;

typedef struct
{
  info_t    info;
#if CONFIG_ARCH_INTERRUPTSTACK > 3
  stack_word_t istack[CONFIG_ISTACK_SIZE];
#endif
  stack_word_t ustack[CONFIG_USTACK_SIZE];
} fullcontext_t;

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

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_INCLUDE_CXD56XX_CRASHDUMP_H */
