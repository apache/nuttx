/************************************************************
 * main.c
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

/************************************************************
 * Compilation Switches
 ************************************************************/

/************************************************************
 * Included Files
 ************************************************************/

#include <nuttx/os_external.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <sched.h>
#include "ostest.h"

/************************************************************
 * Definitions
 ************************************************************/

#define PRIORITY   100
#define STACKSIZE 8192
#define NARGS        4

/************************************************************
 * Private Data
 ************************************************************/

static const char arg1[] = "Arg1";
static const char arg2[] = "Arg2";
static const char arg3[] = "Arg3";
static const char arg4[] = "Arg4";

#if CONFIG_NFILE_DESCRIPTORS > 0
static const char write_data1[] = "stdio_test: write fd=1\n";
static const char write_data2[] = "stdio_test: write fd=2\n";
#endif

#ifdef SDCC
/* I am not yet certain why SDCC does not like the following
 * initializer.  It involves some issues with 2- vs 3-byte
 * pointer types.
 */

static const char *g_argv[NARGS+1];
#else
static const char *g_argv[NARGS+1] = { arg1, arg2, arg3, arg4, NULL };
#endif

/************************************************************
 * Private Functions
 ************************************************************/

/************************************************************
 * Name: user_main
 ************************************************************/

static int user_main(int argc, char *argv[])
{
  int i;

  printf("user_main: Started with argc=%d\n", argc);

  /* Verify passed arguments */

  if (argc != NARGS + 1)
    {
      printf("user_main: Error expected argc=%d got argc=%d\n",
             NARGS+1, argc);
    }

  for (i = 0; i <= NARGS; i++)
    {
      printf("user_main: argv[%d]=\"%s\"\n", i, argv[i]);
    }

  for (i = 1; i <= NARGS; i++)
    {
      if (strcmp(argv[i], g_argv[i-1]) != 0)
        {
          printf("user_main: ERROR argv[%d]:  Expected \"%s\" found \"%s\"\n",
                 i, g_argv[i-1], argv[i]);
        }
    }

#if CONFIG_NFILE_DESCRIPTORS > 0
  /* Checkout /dev/null */

  dev_null();
#endif

#ifndef CONFIG_DISABLE_PTHREAD
  /* Verify pthreads and pthread mutex */

  mutex_test();
#endif

#ifndef CONFIG_DISABLE_PTHREAD
  /* Verify pthread cancellation */

  cancel_test();
#endif

#ifndef CONFIG_DISABLE_PTHREAD
  /* Verify pthreads and semaphores */

  sem_test();
#endif

#ifndef CONFIG_DISABLE_PTHREAD
  /* Verify pthreads and condition variables */

  cond_test();
#endif

#if !defined(CONFIG_DISABLE_SIGNALS) && !defined(CONFIG_DISABLE_PTHREAD)
  /* Verify pthreads and condition variable timed waits */

  timedwait_test();
#endif

#if !defined(CONFIG_DISABLE_MQUEUE) && !defined(CONFIG_DISABLE_PTHREAD)
  /* Verify pthreads and message queues */

  mqueue_test();
#endif

#ifndef CONFIG_DISABLE_SIGNALS
  /* Verify signal handlers */

  sighand_test();
#endif

#if !defined(CONFIG_DISABLE_PTHREAD) && CONFIG_RR_INTERVAL > 0
  /* Verify round robin scheduling */

  rr_test();
#endif

  printf("user_main: Exitting\n");
  return 0;
}

/************************************************************
 * Name: stdio_test
 ************************************************************/

static void stdio_test(void)
{
  /* Verify that we can communicate */

#if CONFIG_NFILE_DESCRIPTORS > 0
  write(1, write_data1, sizeof(write_data1)-1);
#endif
  printf("stdio_test: Standard I/O Check: printf\n");

#if CONFIG_NFILE_DESCRIPTORS > 1
  write(2, write_data2, sizeof(write_data2)-1);
#endif
#if CONFIG_NFILE_STREAMS > 0
  fprintf(stderr, "stdio_test: Standard I/O Check: fprintf to stderr\n");
#endif
}

/************************************************************
 * Public Functions
 ************************************************************/

/************************************************************
 * user_initialize
 ************************************************************/

void user_initialize(void)
{
  /* stub */
}

/************************************************************
 * user_start
 ************************************************************/

int user_start(int argc, char *argv[])
{
  int result;

  /* Verify that stdio works first */

  stdio_test();

#ifdef SDCC
  /* I am not yet certain why SDCC does not like the initilizer.
   * It involves some issues with 2- vs 3-byte pointer types.
   */

  g_argv[0] = arg1;
  g_argv[1] = arg2;
  g_argv[2] = arg3;
  g_argv[3] = arg4;
  g_argv[4] = NULL;
#endif

  /* Verify that we can spawn a new task */

#ifndef CONFIG_CUSTOM_STACK
  result = task_create("ostest", PRIORITY, STACKSIZE, user_main, g_argv);
#else
  result = task_create("ostest", PRIORITY, user_main, g_argv);
#endif
  if (result == ERROR)
    {
      printf("user_start: ERROR Failed to start user_main\n");
    }
  else
    {
      printf("user_start: Started user_main at PID=%d\n", result);
    }

  return 0;
}
