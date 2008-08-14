/****************************************************************************
 * examples/nsh/nsh_serial.h
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include "nsh.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct serial_s
{
  int         ss_refs;    /* Reference counts on the intance */
  char        ss_line[CONFIG_EXAMPLES_NSH_LINELEN];
};

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
 * Name: nsh_allocstruct
 ****************************************************************************/

static inline FAR struct serial_s *nsh_allocstruct(void)
{
  struct serial_s *pstate = (struct serial_s *)malloc(sizeof(struct serial_s));
  if (pstate)
    {
      pstate->ss_refs = 1;
    }
  return pstate;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nsh_main
 ****************************************************************************/

int nsh_main(void)
{
  FAR struct serial_s *pstate = nsh_allocstruct();

  printf("NuttShell (NSH)\n");
  fflush(stdout);

  for (;;)
    {
      /* Display the prompt string */

      fputs(g_nshprompt, stdout);
      fflush(stdout);

      /* Get the next line of input */

      if (fgets(pstate->ss_line, CONFIG_EXAMPLES_NSH_LINELEN, stdin))
        {
          /* Parse process the command */

          (void)nsh_parse(pstate, pstate->ss_line);
          fflush(stdout);
        }
    }
}

/****************************************************************************
 * Name: nsh_output
 *
 * Description:
 *   Print a string to stdout.
 *
 ****************************************************************************/

int nsh_output(FAR void *handle, const char *fmt, ...)
{
  va_list ap;
  int     ret;
 
  va_start(ap, fmt);
  ret = vfprintf(stdout, fmt, ap);
  va_end(ap);
 
  return ret;
}

/****************************************************************************
 * Name: nsh_linebuffer
 *
 * Description:
 *   Return a reference to the current line buffer
 *
 ****************************************************************************/

FAR char *nsh_linebuffer(FAR void *handle)
{
  FAR struct serial_s *pstate = (FAR struct serial_s *)handle;
  return pstate->ss_line;
}

/****************************************************************************
 * Name: nsh_clone
 *
 * Description:
 *   Make an independent copy of the handle
 *
 ****************************************************************************/

FAR void *nsh_clone(FAR void *handle)
{
  return nsh_allocstruct();
}

/****************************************************************************
 * Name: nsh_addref
 *
 * Description:
 *   Increment the reference count on the handle.
 *
 ****************************************************************************/

void nsh_addref(FAR void *handle)
{
  FAR struct serial_s *pstate = (FAR struct serial_s *)handle;
  pstate->ss_refs++;
}

/****************************************************************************
 * Name: nsh_release
 *
 * Description:
 *   Decrement the reference count on the handle, releasing it when the count
 *   decrements to zero.
 *
 ****************************************************************************/

void nsh_release(FAR void *handle)
{
  FAR struct serial_s *pstate = (FAR struct serial_s *)handle;
  if (pstate->ss_refs > 1)
    {
      pstate->ss_refs--;
    }
  else
    {
      free(handle);
    }
}

/****************************************************************************
 * Name: cmd_exit
 *
 * Description:
 *   Exit the shell task
 *
 ****************************************************************************/

void cmd_exit(void *handle, int argc, char **argv)
{
  exit(0);
}
