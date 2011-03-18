/****************************************************************************
 * examples/nsh/nsh_main.c
 *
 *   Copyright (C) 2007-2011 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/stat.h>
#include <stdint.h>
#include <sched.h>

#include <apps/nsh.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_BUILTIN_APPS
#  warning "This example requires CONFIG_BUILTIN_APPS..."
#  warning "  You must have an appconfig file in your config directory to use this example"
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: user_initialize
 ****************************************************************************/

/* In order to support user_initialize if CONFIG_PAGING is defined, this
 * function (and only this function) would need to get moved to the locked
 * text region.
 */

#ifndef CONFIG_PAGING
void user_initialize(void)
{
  /* stub */
}
#endif

/****************************************************************************
 * Name: user_start
 ****************************************************************************/

int user_start(int argc, char *argv[])
{
  int mid_priority;
#if defined(CONFIG_EXAMPLES_NSH_CONSOLE) && defined(CONFIG_EXAMPLES_NSH_TELNET)
  int ret;
#endif

  /* Initialize the NSH library */

  nsh_initialize();

  /* Set the priority of this task to something in the middle so that 'nice'
   * can both raise and lower the priority.
   */

  mid_priority = (sched_get_priority_max(SCHED_NSH) + sched_get_priority_min(SCHED_NSH)) >> 1;
    {
      struct sched_param param;

      param.sched_priority = mid_priority;
      (void)sched_setscheduler(0, SCHED_NSH, &param);
    }

  /* If both the console and telnet are selected as front-ends, then run
   * the telnet front end on another thread.
   */

#if defined(CONFIG_EXAMPLES_NSH_CONSOLE) && defined(CONFIG_EXAMPLES_NSH_TELNET)
# ifndef CONFIG_CUSTOM_STACK
  ret = task_create("nsh_telnetmain", mid_priority, CONFIG_EXAMPLES_NSH_STACKSIZE,
                    nsh_telnetmain, NULL);
# else
  ret = task_create("nsh_telnetmain", mid_priority, nsh_telnetmain, NULL);
# endif
  if (ret < 0)
   {
     fprintf(stderr, g_fmtcmdfailed, "user_start", "task_create", NSH_ERRNO);
   }

  /* If only the telnet front-end is selected, run it on this thread */

#elif defined(CONFIG_EXAMPLES_NSH_TELNET)
  return nsh_telnetmain(0, NULL);
#endif

/* If the serial console front end is selected, then run it on this thread */

#ifdef CONFIG_EXAMPLES_NSH_CONSOLE
  return nsh_consolemain(0, NULL);
#endif
}
