/****************************************************************************
 * nsh_envcmds.c
 *
 *   Copyright (C) 2007, 2008 Gregory Nutt. All rights reserved.
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
#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "nsh.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

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
 * Name: cmd_echo
 ****************************************************************************/

void cmd_echo(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  int i;

  /* echo each argument, separated by a space as it must have been on the
   * command line
   */

  for (i = 1; i < argc; i++)
    {
      /* Check for references to environment variables */

#ifndef CONFIG_DISABLE_ENVIRON
      if (argv[i][0] == '$')
        {
          char *value = getenv(argv[i]+1);
          if (value)
            {
              nsh_output(vtbl, "%s ", value);
            }
        }
      else
#endif
        {
          nsh_output(vtbl, "%s ", argv[i]);
        }
    }
  nsh_output(vtbl, "\n");
}

/****************************************************************************
 * Name: cmd_set
 ****************************************************************************/

#ifndef CONFIG_DISABLE_ENVIRON
void cmd_set(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  if (setenv(argv[1], argv[2], TRUE) < 0)
    {
      nsh_output(vtbl, g_fmtcmdfailed, argv[0], "setenv", NSH_ERRNO);
    }
}
#endif

/****************************************************************************
 * Name: cmd_unset
 ****************************************************************************/

#ifndef CONFIG_DISABLE_ENVIRON
void cmd_unset(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  if (unsetenv(argv[1]) < 0)
    {
      nsh_output(vtbl, g_fmtcmdfailed, argv[0], "unsetenv", NSH_ERRNO);
    }
}
#endif
