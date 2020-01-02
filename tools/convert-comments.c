/****************************************************************************
 * tools/convert-comments.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>

/****************************************************************************
 * Preprocessor Definitions
 ****************************************************************************/

#define LINESIZE 1024  /* Big so that we don't have to bother with range checks */

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char g_lineA[LINESIZE + 3];
static char g_lineB[LINESIZE + 3];
static char g_iobuffer[LINESIZE];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv)
{
  char *g_line0;
  char *g_line1;
  char *ptr;
  char *tmpptr;
  FILE *instream;
  FILE *outstream;
  unsigned long lineno = 0;
  unsigned int indent;
  unsigned int lastindent;
  unsigned int nextindent;
  unsigned int tmpindent;
  bool iscomment;
  bool wascomment;
  bool willbecomment;
  bool isblank;
  bool wasblank;
  bool willbeblank;
  bool willbevalid;
  int ret = 1;

  if (argc != 3)
    {
      fprintf(stderr, "ERROR:  Two arguments expected\n");
      return 1;
    }

  /* Open the source file read-only */

  instream = fopen(argv[1], "r");
  if (instream == NULL)
    {
      fprintf(stderr, "ERROR:  Failed to open %s for reading\n", argv[1]);
      return 1;
    }

  /* Open the destination file write-only */

  outstream = fopen(argv[2], "w");
  if (outstream == NULL)
    {
      fprintf(stderr, "ERROR:  Failed to open %s for reading\n", argv[2]);
      goto errout_with_instream;
    }

  /* Prime the pump */

  g_line0    = g_lineA;
  g_line1    = g_lineB;
  wasblank   = true;
  wascomment = false;
  lastindent = 0;

  /* Read line n + 1 (for n = 0) */

  if (fgets(g_line1, LINESIZE, instream) == NULL)
    {
      fprintf(stderr, "ERROR:  File is empty\n");
      goto errout_with_outstream;
    }

  /* Skip over leading spaces in line n + 1 */

  for (ptr = g_line1, nextindent = 0;
       *ptr != '\0' && *ptr != '\n' && isspace(*ptr);
       ptr++, nextindent++)
    {
    }

  /* Check for a blank line */

  if (*ptr == '\0' || *ptr == '\n')
    {
      ptr           = g_line1;
      ptr[0]        = '\n';
      ptr[1]        = '\0';
      willbeblank   = true;
      willbecomment = false;
      nextindent    = 0;
    }
  else
    {
      /* Check for a C++ style comment at this indentation in line n + 1 */

      willbeblank   = false;
      willbecomment = strncmp(ptr, "//", 2) == 0;
    }

  /* Loop for each line in the file */

  do
    {
      /* Swap line n and line n + 1 */

      ptr     = g_line0;
      g_line0 = g_line1; /* New line n */
      g_line1 = ptr;     /* Will be new line n + 1 */

      /* Read the new line n + 1 */

      willbevalid = (fgets(g_line1, LINESIZE, instream) != NULL);

      /* Update info for line 0 */

      lineno++;
      indent    = nextindent;
      isblank   = willbeblank;
      iscomment = willbecomment;

      if (willbevalid)
        {
          /* Strip trailing spaces and carriage returns from line n + 1 */

          tmpptr = strchr(g_line1, '\r');
          if (tmpptr != NULL)
            {
              *tmpptr++ = '\n';
              *tmpptr++ = '\0';
            }
          else
            {
              int len = strlen(ptr);
              if (len > 0)
                {
                  for (ptr += len - 1; isspace(*ptr); ptr--)
                    {
                      ptr[0] = '\n';
                      ptr[1] = '\0';
                    }
                }
            }

          /* Skip over leading spaces in line n + 1 */

          for (ptr = g_line1, nextindent = 0;
               *ptr != '\0' && *ptr != '\n' && isspace(*ptr);
               ptr++, nextindent++)
            {
            }

          /* Check for a blank line */

          if (*ptr == '\0' || *ptr == '\n')
            {
              ptr           = g_line1;
              ptr[0]        = '\n';
              ptr[1]        = '\0';
              willbeblank   = true;
              willbecomment = false;
              nextindent    = 0;
            }
          else
            {
              /* Check for a C++ style comment at this indentation in line n + 1 */

              willbeblank   = false;
              willbecomment = strncmp(ptr, "//", 2) == 0;
            }

          /* Check for a C++ style comment at this indentation in line n + 1 */

          willbecomment = strncmp(ptr, "//", 2) == 0;
        }
      else
        {
          willbeblank   = true;
          willbecomment = false;
          nextindent    = 0;
        }

      /* If current line is not a comment line, then check for a C++ style comment at the
       * end of the line.
       */

      if (!iscomment)
        {
          /* Skip over leading spaces in line n + 1 */

          for (ptr = g_line0 + indent, tmpindent = indent;
               *ptr != '\0' && *ptr != '\n';
               ptr++, tmpindent++)
            {
              if (ptr[0] == '/' && ptr[1] == '/')
                {
                  indent     = tmpindent;
                  iscomment  = true;
                  wascomment = false;
                  break;
                }
            }
        }

      printf("*****************************************************************************\n");
      printf("* %5lu. %s\n", lineno, g_line0);
      printf("*  indent: last=%u curr=%u next=%u\n",
              lastindent, indent, nextindent);
      printf("* comment: last=%u curr=%u next=%u\n",
              wascomment, iscomment, willbecomment);
      printf("*   blank: last=%u curr=%u next=%u\n",
              wasblank, isblank, willbeblank);
      printf("*****************************************************************************\n");

      /* Does line n start with a comment */

      ptr = g_line0 + indent;
      if (iscomment)
        {
          char *endptr;

          /* Get a pointer for the first non-blank character following the
           * comment.
           */

          for (endptr = &ptr[2];
               *endptr != '\0' && *endptr != '\n' && isspace(*endptr);
               endptr++)
            {
            }

          /* Check for a comment only line that is was not preceded by a
           * comment.
           */

          if ((*endptr == '\n' || *endptr == '\0') && !wascomment)
            {
              /* Avoid double spacing */

              if (!wasblank)
                {
                  /* Output a blank line  */

                  fputc('\n', outstream);
                }

              /* Reset to indicate a blank line */

              indent    = 0;
              iscomment = false;
              isblank   = true;
            }

          /* Did line n - 1 start with a comment? */

          else if (wascomment)
            {
              /* Yes.. Change it to a C-style block comment continuation */

              ptr[0] = ' ';
              ptr[1] = '*';

              /* Write the modified line to the output */

              fputs(g_line0, outstream);
            }
          else
            {
              /* No.. Change it to a C-style opening comment. */

              ptr[1] = '*';

              if (!willbecomment)
                {
                  int len;

                  /* Remove final linefeed */

                  len = strlen(ptr);
                  if (len > 0 && ptr[len - 1] == '\n')
                    {
                      len--;
                    }

                  /* Close the single line C comment */

                  ptr += len;
                  *ptr++ = ' ';
                  *ptr++ = '*';
                  *ptr++ = '/';
                  *ptr++ = '\n';
                  *ptr++ = '\0';

                  iscomment = false;

                  /* Write the modified line to the output */

                  fputs(g_line0, outstream);

                  /* Closing comment must be followed by a blank line */

                  if (!willbeblank)
                    {
                      /* Output a blank line */

                      fputc('\n', outstream);
                    }
                }
              else
                {
                  /* Write the modified line to the output */

                  fputs(g_line0, outstream);
                }
            }
        }
      else if (wascomment)
        {
          /* Line n is not a comment, but line n - 1 was.  Output a closing on a
           * newline at the same indentation.
           */

          memset(g_iobuffer, ' ', LINESIZE);
          ptr = g_iobuffer + lastindent + 1;
          *ptr++ = '*';
          *ptr++ = '/';
          *ptr++ = '\n';
          *ptr++ = '\0';

          /* Write the closing line to the output */

          fputs(g_iobuffer, outstream);

          /* Closing comment must be followed by a blank line */

          if (!isblank)
            {
              /* Output a blank line */

              fputc('\n', outstream);
            }

          /* Write the noncomment line to the output */

          fputs(g_line0, outstream);
        }
      else
        {
          /* Write the noncomment line to the output */

          fputs(g_line0, outstream);
        }

      wascomment = iscomment;
      wasblank   = isblank;
      lastindent = indent;
    }
  while (willbevalid);

  ret = 0;

errout_with_outstream:
  fclose(outstream);

errout_with_instream:
  fclose(instream);
  return ret;
}
