/****************************************************************************
 * arch/include/CXD56XX/crashdump.h
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
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

#ifndef __ARCH_ARM_INCLUDE_CXD56XX_CRASHDUMP_H
#define __ARCH_ARM_INCLUDE_CXD56XX_CRASHDUMP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/irq.h>

/****************************************************************************
 * Pre-processor Definitions
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
  int           pid;                    /* Process ID */
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
