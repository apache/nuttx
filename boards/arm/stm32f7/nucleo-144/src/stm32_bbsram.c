/****************************************************************************
 * boards/arm/stm32f7/nucleo-144/src/stm32_bbsram.c
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

#include <nuttx/config.h>

#include <sys/ioctl.h>

#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <syslog.h>

#include <sys/param.h>

#include <nuttx/fs/fs.h>

#include "arm_internal.h"
#include "stm32_bbsram.h"

#include "nucleo-144.h"

#ifdef CONFIG_STM32F7_BBSRAM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#define FREEZE_STR(s)          #s
#define STRINGIFY(s)           FREEZE_STR(s)
#define HARDFAULT_FILENO       3
#define HARDFAULT_PATH         BBSRAM_PATH""STRINGIFY(HARDFAULT_FILENO)
#define HARDFAULT_REBOOT_      FILENO 0
#define HARDFAULT_REBOOT_PATH  BBSRAM_PATH""STRINGIFY(HARDFAULT_REBOOT_FILENO)

#define BBSRAM_SIZE_FN0        (sizeof(int))
#define BBSRAM_SIZE_FN1        384
#define BBSRAM_SIZE_FN2        384
#define BBSRAM_SIZE_FN3 -      1

/* The following guides in the amount of the user and interrupt stack
 * data we can save. The amount of storage left will dictate the actual
 * number of entries of the user stack data saved. If it is too big
 * It will be truncated by the call to stm32_bbsram_savepanic
 */
#define BBSRAM_HEADER_SIZE     20 /* This is an assumption */
#define BBSRAM_USED            ((4*BBSRAM_HEADER_SIZE)+ \
                                (BBSRAM_SIZE_FN0+BBSRAM_SIZE_FN1+ \
                                 BBSRAM_SIZE_FN2))
#define BBSRAM_REAMINING       (STM32F7_BBSRAM_SIZE-BBSRAM_USED)
#if CONFIG_ARCH_INTERRUPTSTACK <= 3
#  define BBSRAM_NUMBER_STACKS 1
#else
#  define BBSRAM_NUMBER_STACKS 2
#endif
#define BBSRAM_FIXED_ELEMENTS_SIZE (sizeof(info_t))
#define BBSRAM_LEFTOVER            (BBSRAM_REAMINING-\
                                    BBSRAM_FIXED_ELEMENTS_SIZE)

#define CONFIG_ISTACK_SIZE (BBSRAM_LEFTOVER/BBSRAM_NUMBER_STACKS/ \
                            sizeof(stack_word_t))
#define CONFIG_USTACK_SIZE (BBSRAM_LEFTOVER/BBSRAM_NUMBER_STACKS/ \
                            sizeof(stack_word_t))

/* The path to the Battery Backed up SRAM */

#define BBSRAM_PATH "/fs/bbr"

/* The sizes of the files to create (-1) use rest of BBSRAM memory */

#define BSRAM_FILE_SIZES \
{ \
  BBSRAM_SIZE_FN0, \
  BBSRAM_SIZE_FN1, \
  BBSRAM_SIZE_FN2, \
  BBSRAM_SIZE_FN3, \
  0 \
}

/* For Assert keep this much of the file name */

#define MAX_FILE_PATH_LENGTH 40

#define HEADER_TIME_FMT      "%Y-%m-%d-%H:%M:%S"
#define HEADER_TIME_FMT_NUM  (2+ 0+ 0+ 0+ 0+ 0)
#define HEADER_TIME_FMT_LEN  (((nitems(HEADER_TIME_FMT)-1) + \
                                HEADER_TIME_FMT_NUM))

/****************************************************************************
 * Private Data
 ****************************************************************************/

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

/* Not Used for reference only */

typedef struct
{
  uint32_t r0;
  uint32_t r1;
  uint32_t r2;
  uint32_t r3;
  uint32_t r4;
  uint32_t r5;
  uint32_t r6;
  uint32_t r7;
  uint32_t r8;
  uint32_t r9;
  uint32_t r10;
  uint32_t r11;
  uint32_t r12;
  uint32_t sp;
  uint32_t lr;
  uint32_t pc;
  uint32_t xpsr;
  uint32_t d0;
  uint32_t d1;
  uint32_t d2;
  uint32_t d3;
  uint32_t d4;
  uint32_t d5;
  uint32_t d6;
  uint32_t d7;
  uint32_t d8;
  uint32_t d9;
  uint32_t d10;
  uint32_t d11;
  uint32_t d12;
  uint32_t d13;
  uint32_t d14;
  uint32_t d15;
  uint32_t fpscr;
  uint32_t sp_main;
  uint32_t sp_process;
  uint32_t apsr;
  uint32_t ipsr;
  uint32_t epsr;
  uint32_t primask;
  uint32_t basepri;
  uint32_t faultmask;
  uint32_t control;
  uint32_t s0;
  uint32_t s1;
  uint32_t s2;
  uint32_t s3;
  uint32_t s4;
  uint32_t s5;
  uint32_t s6;
  uint32_t s7;
  uint32_t s8;
  uint32_t s9;
  uint32_t s10;
  uint32_t s11;
  uint32_t s12;
  uint32_t s13;
  uint32_t s14;
  uint32_t s15;
  uint32_t s16;
  uint32_t s17;
  uint32_t s18;
  uint32_t s19;
  uint32_t s20;
  uint32_t s21;
  uint32_t s22;
  uint32_t s23;
  uint32_t s24;
  uint32_t s25;
  uint32_t s26;
  uint32_t s27;
  uint32_t s28;
  uint32_t s29;
  uint32_t s30;
  uint32_t s31;
} proc_regs_t;

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
  info_t    info;                  /* The info */
#if CONFIG_ARCH_INTERRUPTSTACK > 3
  /* The amount of stack data is compile time
   * sized backed on what is left after the
   * other BBSRAM files are defined
   * The order is such that only the
   * ustack should be truncated
   */
  stack_word_t istack[CONFIG_USTACK_SIZE];
#endif
  stack_word_t ustack[CONFIG_ISTACK_SIZE];
} fullcontext_t;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint8_t g_sdata[STM32F7_BBSRAM_SIZE];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hardfault_get_desc
 ****************************************************************************/

static int hardfault_get_desc(struct bbsramd_s *desc)
{
  struct file filestruct;
  int ret;

  ret = file_open(&filestruct, HARDFAULT_PATH, O_RDONLY);
  if (ret < 0)
    {
      syslog(LOG_INFO, "stm32 bbsram: Failed to open Fault Log file [%s] "
             "(%d)\n", HARDFAULT_PATH, ret);
    }
  else
    {
      ret = file_ioctl(&filestruct, STM32F7_BBSRAM_GETDESC_IOCTL,
                       (unsigned long)((uintptr_t)desc));
      file_close(&filestruct);

      if (ret < 0)
        {
          syslog(LOG_INFO, "stm32 bbsram: Failed to get Fault Log "
              "descriptor (%d)\n", ret);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: copy_reverse
 ****************************************************************************/

#if defined(CONFIG_STM32F7_SAVE_CRASHDUMP)
static void copy_reverse(stack_word_t *dest, stack_word_t *src, int size)
{
  while (size--)
    {
      *dest++ = *src--;
    }
}
#endif /* CONFIG_STM32F7_SAVE_CRASHDUMP */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bbsram_int
 ****************************************************************************/

int stm32_bbsram_int(void)
{
  int filesizes[CONFIG_STM32F7_BBSRAM_FILES + 1] = BSRAM_FILE_SIZES;
  char buf[HEADER_TIME_FMT_LEN + 1];
  struct bbsramd_s desc;
  int rv;
  int state;
  struct tm tt;
  time_t time_sec;

  /* Using Battery Backed Up SRAM */

  stm32_bbsraminitialize(BBSRAM_PATH, filesizes);

#if defined(CONFIG_STM32F7_SAVE_CRASHDUMP)
  /* Panic Logging in Battery Backed Up Files */

  /* Do we have an hard fault in BBSRAM? */

  rv = hardfault_get_desc(&desc);
  if (rv >= OK)
    {
      syslog(LOG_EMERG, "There is a hard fault logged.\n");
      state = (desc.lastwrite.tv_sec || desc.lastwrite.tv_nsec) ?  OK : 1;

      syslog(LOG_INFO, "Fault Log info File No %d Length %d flags:0x%02x "
          "state:%d\n", (unsigned int)desc.fileno, (unsigned int)desc.len,
          (unsigned int)desc.flags, state);

      if (state == OK)
        {
          time_sec = desc.lastwrite.tv_sec + (desc.lastwrite.tv_nsec / 1e9);
          gmtime_r(&time_sec, &tt);
          strftime(buf, HEADER_TIME_FMT_LEN , HEADER_TIME_FMT , &tt);

          syslog(LOG_INFO, "Fault Logged on %s - Valid\n", buf);
        }

      rv = nx_unlink(HARDFAULT_PATH);
      if (rv < 0)
        {
          syslog(LOG_INFO, "stm32 bbsram: Failed to unlink Fault Log file"
                 " [%s] (%d)\n", HARDFAULT_PATH, rv);
        }
    }
#endif /* CONFIG_STM32F7_SAVE_CRASHDUMP */

  return rv;
}

/****************************************************************************
 * Name: board_crashdump
 ****************************************************************************/

#if defined(CONFIG_STM32F7_SAVE_CRASHDUMP)
void board_crashdump(uintptr_t sp, struct tcb_s *tcb,
                     const char *filename, int lineno,
                     const char *msg)
{
  fullcontext_t *pdump = (fullcontext_t *)&g_sdata;
  int rv;

  enter_critical_section();

  /* Zero out everything */

  memset(pdump, 0, sizeof(fullcontext_t));

  /* Save Info */

  pdump->info.lineno = lineno;

  if (filename)
    {
      int offset = 0;
      unsigned int len = strlen((char *)filename) + 1;

      if (len > sizeof(pdump->info.filename))
        {
          offset = len - sizeof(pdump->info.filename);
        }

      strlcpy(pdump->info.filename, (char *)&filename[offset],
              sizeof(pdump->info.filename));
    }

  /* Save the value of the pointer for current_regs as debugging info.
   * It should be NULL in case of an ASSERT and will aid in cross
   * checking the validity of system memory at the time of the
   * fault.
   */

  pdump->info.current_regs = (uintptr_t) CURRENT_REGS;

  /* Save Context */

#if CONFIG_TASK_NAME_SIZE > 0
  strlcpy(pdump->info.name, tcb->name, sizeof(pdump->info.name));
#endif

  pdump->info.pid = tcb->pid;

  /* If  current_regs is not NULL then we are in an interrupt context
   * and the user context is in current_regs else we are running in
   * the users context
   */

  if (CURRENT_REGS)
    {
      pdump->info.stacks.interrupt.sp = sp;
      pdump->info.flags |= (REGS_PRESENT | USERSTACK_PRESENT | \
                            INTSTACK_PRESENT);
      memcpy(pdump->info.regs, (void *)CURRENT_REGS,
             sizeof(pdump->info.regs));
      pdump->info.stacks.user.sp = pdump->info.regs[REG_R13];
    }
  else
    {
      /* users context */

      pdump->info.flags |= USERSTACK_PRESENT;
      pdump->info.stacks.user.sp = sp;
    }

  pdump->info.stacks.user.top = (uint32_t)tcb->stack_base_ptr +
                                          tcb->adj_stack_size;
  pdump->info.stacks.user.size = (uint32_t)tcb->adj_stack_size;

#if CONFIG_ARCH_INTERRUPTSTACK > 3
  /* Get the limits on the interrupt stack memory */

  pdump->info.stacks.interrupt.top = (uint32_t)g_intstacktop;
  pdump->info.stacks.interrupt.size = (CONFIG_ARCH_INTERRUPTSTACK & ~3);

  /* If In interrupt Context save the interrupt stack data centered
   * about the interrupt stack pointer
   */

  if ((pdump->info.flags & INTSTACK_PRESENT) != 0)
    {
      stack_word_t *ps = (stack_word_t *) pdump->info.stacks.interrupt.sp;
      copy_reverse(pdump->istack, &ps[nitems(pdump->istack) / 2],
                   nitems(pdump->istack));
    }

  /* Is it Invalid? */

  if (!(pdump->info.stacks.interrupt.sp <=
        pdump->info.stacks.interrupt.top &&
        pdump->info.stacks.interrupt.sp > pdump->info.stacks.interrupt.top -
          pdump->info.stacks.interrupt.size))
    {
      pdump->info.flags |= INVALID_INTSTACK_PTR;
    }

#endif
  /* If In interrupt context or User save the user stack data centered
   * about the user stack pointer
   */

  if ((pdump->info.flags & USERSTACK_PRESENT) != 0)
    {
      stack_word_t *ps = (stack_word_t *) pdump->info.stacks.user.sp;
      copy_reverse(pdump->ustack, &ps[nitems(pdump->ustack) / 2],
                   nitems(pdump->ustack));
    }

  /* Is it Invalid? */

  if (!(pdump->info.stacks.user.sp <= pdump->info.stacks.user.top &&
        pdump->info.stacks.user.sp > pdump->info.stacks.user.top -
          pdump->info.stacks.user.size))
    {
      pdump->info.flags |= INVALID_USERSTACK_PTR;
    }

  rv = stm32_bbsram_savepanic(HARDFAULT_FILENO, (uint8_t *)pdump,
                              sizeof(fullcontext_t));

  /* Test if memory got wiped because of using _sdata */

  if (rv == -ENXIO)
    {
      char *dead = "Memory wiped - dump not saved!";

      while (*dead)
        {
          arm_lowputc(*dead++);
        }
    }
  else if (rv == -ENOSPC)
    {
      /* hard fault again */

      arm_lowputc('!');
    }
}
#endif /* CONFIG_STM32F7_SAVE_CRASHDUMP */

#endif /* CONFIG_STM32_BBSRAM */
