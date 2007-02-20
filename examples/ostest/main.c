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
#include <string.h>
#include <sched.h>
#include "ostest.h"

/************************************************************
 * Definitions
 ************************************************************/

#define PRIORITY   100
#define STACKSIZE 8192
#define NARGS        4

#define ARG1      "Arg1"
#define ARG2      "Arg2"
#define ARG3      "Arg3"
#define ARG4      "Arg4"

/************************************************************
 * Private Data
 ************************************************************/

static char write_data1[] = "Standard I/O Check: write fd=1\n";
static char write_data2[] = "Standard I/O Check: write fd=2\n";
static char *args[NARGS] = { ARG1, ARG2, ARG3, ARG4 };

/************************************************************
 * Private Functions
 ************************************************************/

static int user_main(int argc, char *argv[])
{
  int i;

  printf("user_main: Started with argc=%d\n", argc);

  /* Verify passed arguments */

  if (argc != NARGS + 1)
    {
      fprintf(stderr, "user_main: Error expected argc=%d got argc=%d\n",
              NARGS+1, argc);
    }

  for (i = 0; i <= NARGS; i++)
    {
      printf("user_main: argv[%d]=\"%s\"\n", i, argv[i]);
    }

  for (i = 1; i <= NARGS; i++)
    {
      if (strcmp(argv[i], args[i-1]) != 0)
        {
          fprintf(stderr, "user_main: ERROR argv[%d]:  Expected \"%s\" found \"%s\"\n",
                  i, argv[i], args[i-1]);
        }
    }

  /* Checkout /dev/null */

  dev_null();

  /* Verify pthreads and pthread mutex */

  mutex_test();

  /* Verify pthread cancellation */

  cancel_test();

  /* Verify pthreads and semaphores */

  sem_test();

  /* Verify pthreads and condition variables */

  cond_test();

  /* Verify pthreads and condition variable timed waits */

  timedwait_test();

  /* Verify pthreads and message queues */

  mqueue_test();

  /* Verify signal handlers */

  sighand_test();
  return 0;
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

int user_start(int parm1, int parm2, int parm3, int parm4)
{
  int result;

  /* Verify that we can communicate */

  write(1, write_data1, sizeof(write_data1));
  printf("user_start: Standard I/O Check: printf\n");
  write(2, write_data2, sizeof(write_data2));
  fprintf(stderr, "user_start: Standard I/O Check: fprintf to stderr\n");

  /* Verify that we can spawn a new task */

  result = task_create("ostest", PRIORITY, STACKSIZE, user_main,
                       ARG1, ARG2, ARG3, ARG4);
  if (result == ERROR)
    {
      fprintf(stderr, "user_start: Failed to start user_main\n");
    }
  else
    {
       printf("user_start: Started user_main at PID=%d\n", result);
    }
  return 0;
}
