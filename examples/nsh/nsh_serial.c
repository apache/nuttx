/****************************************************************************
 * examples/nsh/nsh_serial.c
 *
 *   Copyright (C) 2007-2009, 2011 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdarg.h>
#include <errno.h>
#include <debug.h>

#include "nsh.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct serial_s
{
  struct nsh_vtbl_s ss_vtbl;
  int    ss_fd;      /* Re-direct file descriptor */
  FILE  *ss_stream;  /* Re-direct stream */
  char   ss_line[CONFIG_EXAMPLES_NSH_LINELEN];
};

struct serialsave_s
{
  int    ss_fd;      /* Re-direct file descriptor */
  FILE  *ss_stream;  /* Re-direct stream */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifndef CONFIG_EXAMPLES_NSH_DISABLEBG
static FAR struct nsh_vtbl_s *nsh_consoleclone(FAR struct nsh_vtbl_s *vtbl);
static void nsh_consolerelease(FAR struct nsh_vtbl_s *vtbl);
#endif
static ssize_t nsh_consolewrite(FAR struct nsh_vtbl_s *vtbl, FAR const void *buffer, size_t nbytes);
static int nsh_consoleoutput(FAR struct nsh_vtbl_s *vtbl, const char *fmt, ...);
static FAR char *nsh_consolelinebuffer(FAR struct nsh_vtbl_s *vtbl);
static void nsh_consoleredirect(FAR struct nsh_vtbl_s *vtbl, int fd, FAR uint8_t *save);
static void nsh_consoleundirect(FAR struct nsh_vtbl_s *vtbl, FAR uint8_t *save);
static void nsh_consoleexit(FAR struct nsh_vtbl_s *vtbl);

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
  struct serial_s *pstate = (struct serial_s *)zalloc(sizeof(struct serial_s));
  if (pstate)
    {
#ifndef CONFIG_EXAMPLES_NSH_DISABLEBG
      pstate->ss_vtbl.clone      = nsh_consoleclone;
      pstate->ss_vtbl.release    = nsh_consolerelease;
#endif
      pstate->ss_vtbl.write      = nsh_consolewrite;
      pstate->ss_vtbl.output     = nsh_consoleoutput;
      pstate->ss_vtbl.linebuffer = nsh_consolelinebuffer;
      pstate->ss_vtbl.redirect   = nsh_consoleredirect;
      pstate->ss_vtbl.undirect   = nsh_consoleundirect;
      pstate->ss_vtbl.exit       = nsh_consoleexit;

      pstate->ss_fd              = 1;
      pstate->ss_stream          = stdout;
    }
  return pstate;
}

/****************************************************************************
 * Name: nsh_openifnotopen
 ****************************************************************************/

static int nsh_openifnotopen(struct serial_s *pstate)
{
  /* The stream is open in a lazy fashion.  This is done because the file
   * descriptor may be opened on a different task than the stream.
   */

  if (!pstate->ss_stream)
    {
      pstate->ss_stream = fdopen(pstate->ss_fd, "w");
      if (!pstate->ss_stream)
        {
          return ERROR;
        }
    }
  return 0;
}

/****************************************************************************
 * Name: nsh_closeifnotclosed
 ****************************************************************************/

static void nsh_closeifnotclosed(struct serial_s *pstate)
{
  if (pstate->ss_stream == stdout)
    {
      fflush(stdout);
      pstate->ss_fd = 1;
    }
  else
    {
      if (pstate->ss_stream)
        {
          fflush(pstate->ss_stream);
          fclose(pstate->ss_stream);
        }
      else if (pstate->ss_fd >= 0 && pstate->ss_fd != 1)
        {
          close(pstate->ss_fd);
        }

      pstate->ss_fd     = -1;
      pstate->ss_stream = NULL;
    }
}

/****************************************************************************
 * Name: nsh_consolewrite
 *
 * Description:
 *   write a buffer to the remote shell window.
 *
 *   Currently only used by cat.
 *
 ****************************************************************************/

static ssize_t nsh_consolewrite(FAR struct nsh_vtbl_s *vtbl, FAR const void *buffer, size_t nbytes)
{
  FAR struct serial_s *pstate = (FAR struct serial_s *)vtbl;
  ssize_t ret;

  /* The stream is open in a lazy fashion.  This is done because the file
   * descriptor may be opened on a different task than the stream.  The
   * actual open will then occur with the first output from the new task.
   */

  if (nsh_openifnotopen(pstate) != 0)
   {
     return (ssize_t)ERROR;
   }

  /* Write the data to the output stream */

  ret = fwrite(buffer, 1, nbytes, pstate->ss_stream);
  if (ret < 0)
    {
      dbg("[%d] Failed to send buffer: %d\n", pstate->ss_fd, errno);
    }
  return ret;
}

/****************************************************************************
 * Name: nsh_consoleoutput
 *
 * Description:
 *   Print a string to the currently selected stream.
 *
 ****************************************************************************/

static int nsh_consoleoutput(FAR struct nsh_vtbl_s *vtbl, const char *fmt, ...)
{
  FAR struct serial_s *pstate = (FAR struct serial_s *)vtbl;
  va_list ap;
  int     ret;

  /* The stream is open in a lazy fashion.  This is done because the file
   * descriptor may be opened on a different task than the stream.  The
   * actual open will then occur with the first output from the new task.
   */

  if (nsh_openifnotopen(pstate) != 0)
   {
     return ERROR;
   }
 
  va_start(ap, fmt);
  ret = vfprintf(pstate->ss_stream, fmt, ap);
  va_end(ap);
 
  return ret;
}

/****************************************************************************
 * Name: nsh_consolelinebuffer
 *
 * Description:
 *   Return a reference to the current line buffer
 *
 ****************************************************************************/

static FAR char *nsh_consolelinebuffer(FAR struct nsh_vtbl_s *vtbl)
{
  FAR struct serial_s *pstate = (FAR struct serial_s *)vtbl;
  return pstate->ss_line;
}

/****************************************************************************
 * Name: nsh_consoleclone
 *
 * Description:
 *   Make an independent copy of the vtbl
 *
 ****************************************************************************/

#ifndef CONFIG_EXAMPLES_NSH_DISABLEBG
static FAR struct nsh_vtbl_s *nsh_consoleclone(FAR struct nsh_vtbl_s *vtbl)
{
  FAR struct serial_s *pstate = (FAR struct serial_s *)vtbl;
  FAR struct serial_s *pclone = nsh_allocstruct();

  if (pclone->ss_fd == 1)
    {
      pclone->ss_fd     = 1;
      pclone->ss_stream = stdout;
    }
  else
    {
      pclone->ss_fd     = pstate->ss_fd;
      pclone->ss_stream = NULL;
    }
  return &pclone->ss_vtbl;
}
#endif

/****************************************************************************
 * Name: nsh_consolerelease
 *
 * Description:
 *   Release the cloned instance
 *
 ****************************************************************************/

#ifndef CONFIG_EXAMPLES_NSH_DISABLEBG
static void nsh_consolerelease(FAR struct nsh_vtbl_s *vtbl)
{
  FAR struct serial_s *pstate = (FAR struct serial_s *)vtbl;

  nsh_closeifnotclosed(pstate);
  free(vtbl);
}
#endif

/****************************************************************************
 * Name: nsh_consoleredirect
 *
 * Description:
 *   Set up for redirected output.  This function is called from nsh_parse()
 *   in two different contexts:
 *
 *   1) Redirected background commands of the form:  command > xyz.text &
 *
 *      In this case:
 *      - vtbl: A newly allocated and initialized instance created by
 *        nsh_consoleclone,
 *      - fd:- The file descriptor of the redirected output
 *      - save: NULL
 *
 *      nsh_consolerelease() will perform the clean-up when the clone is
 *      destroyed.
 *        
 *   2) Redirected foreground commands of the form:  command > xyz.txt
 *
 *      In this case:
 *      - vtbl: The current state structure,
 *      - fd: The file descriptor of the redirected output
 *      - save: Where to save the re-directed registers.
 *
 *      nsh_consoleundirect() will perform the clean-up after the redirected
 *      command completes.
 *
 ****************************************************************************/

static void nsh_consoleredirect(FAR struct nsh_vtbl_s *vtbl, int fd, FAR uint8_t *save)
{
  FAR struct serial_s     *pstate = (FAR struct serial_s *)vtbl;
  FAR struct serialsave_s *ssave  = (FAR struct serialsave_s *)save;

  /* Case 1: Redirected foreground commands */

  if (ssave)
    {
      /* pstate->ss_stream and ss_fd refer refer to the
       * currently opened output stream.  If the console is open, flush
       * any pending output.
       */

      if (pstate->ss_stream)
        {
          fflush(pstate->ss_stream);          
        }

      /* Save the current fd and stream values.  These will be restored
       * when nsh_consoleundirect() is called.
       */

      ssave->ss_fd     = pstate->ss_fd;
      ssave->ss_stream = pstate->ss_stream;
    }
  else
    {
      /* nsh_consoleclone() set pstate->ss_fd and ss_stream to refer
       * to standard out.  We just want to leave these alone and overwrite
       * them with the fd for the re-directed stream.
       */
    }

  /* In either case, set the fd of the new, re-directed output and nullify
   * the output stream (it will be fdopen'ed if it is used).
   */

  pstate->ss_fd     = fd;
  pstate->ss_stream = NULL;
}

/****************************************************************************
 * Name: nsh_consoleundirect
 *
 * Description:
 *   Set up for redirected output
 *
 ****************************************************************************/

static void nsh_consoleundirect(FAR struct nsh_vtbl_s *vtbl, FAR uint8_t *save)
{
  FAR struct serial_s *pstate = (FAR struct serial_s *)vtbl;
  FAR struct serialsave_s *ssave  = (FAR struct serialsave_s *)save;

  nsh_closeifnotclosed(pstate);
  pstate->ss_fd     = ssave->ss_fd;
  pstate->ss_stream = ssave->ss_stream;
}

/****************************************************************************
 * Name: nsh_consoleexit
 *
 * Description:
 *   Exit the shell task
 *
 ****************************************************************************/

static void nsh_consoleexit(FAR struct nsh_vtbl_s *vtbl)
{
  exit(0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nsh_main
 ****************************************************************************/

int nsh_consolemain(int argc, char *argv[])
{
  FAR struct serial_s *pstate = nsh_allocstruct();

  /* Present a greeting */

  printf(g_nshgreeting);
  fflush(pstate->ss_stream);

  /* Execute the startup script */

#ifdef CONFIG_EXAMPLES_NSH_ROMFSETC
  (void)nsh_script(&pstate->ss_vtbl, "init", NSH_INITPATH);
#endif

  /* Then enter the command line parsing loop */

  for (;;)
    {
      /* Display the prompt string */

      fputs(g_nshprompt, pstate->ss_stream);
      fflush(pstate->ss_stream);

      /* Get the next line of input */

      if (fgets(pstate->ss_line, CONFIG_EXAMPLES_NSH_LINELEN, stdin))
        {
          /* Parse process the command */

          (void)nsh_parse(&pstate->ss_vtbl, pstate->ss_line);
          fflush(pstate->ss_stream);
        }
    }
  return OK;
}
