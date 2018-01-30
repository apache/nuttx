/****************************************************************************
 * tools/logparser.c
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

#define _GNU_SOURCE 1
#include <stdbool.h>
#include <stdlib.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LINESIZE  (PATH_MAX > 4096 ? PATH_MAX : 4096)

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum parse_state_e
{
  STATE_IDLE = 0,   /* Finished last, ready to start next */
  STATE_HEADER,     /* Started next, parsing header */
  STATE_ENDHEADER,  /* Finished the header looking for the start of the body */
  STATE_BODY,       /* Parsing the body */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char g_line[LINESIZE+1];
static unsigned long g_lineno;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Skip over any spaces */

static char *skip_space(char *ptr)
{
  while (*ptr && isspace((int)*ptr)) ptr++;
  return ptr;
}

/* Find the end of a variable string */

static void trim_end(char *startptr, char *endptr)
{
  while (endptr != startptr && isspace((int)*endptr))
    {
      *endptr = '\0';
      endptr--;
    }
}

/* check for a blank line */

static bool blank_line(void)
{
  char *ptr = g_line;

  while (*ptr != '\0')
    {
      if (!isspace((int)*ptr))
        {
          return false;
        }

      ptr++;
    }

  return true;
}

/* Convert git log date string */

static char *parse_date(char *ptr)
{
  char *next;
  char *endptr;
  char *alloc;
  unsigned int mon;
  unsigned long date;
  unsigned long year;

  /* Form: DOW MON DD HH:MM:SS YYYY GMT
   * Eg.   Mon Jan 29 07:17:17 2018 -0600
   */

  /* Skip the day of the week */

  next = strchr(ptr, ' ');
  if (next == NULL)
    {
      fprintf(stderr, "ERROR line %ld: Did not find day of the week\n", g_lineno);
      exit(EXIT_FAILURE);
    }

  ptr = skip_space(next);

  /* Get the month */

  next = strchr(ptr, ' ');
  if (next == NULL)
    {
      fprintf(stderr, "ERROR line %ld: Did not find month\n", g_lineno);
      exit(EXIT_FAILURE);
    }

  endptr = next;
  next   = skip_space(next);
  trim_end(ptr, endptr);

  if (strncmp(ptr, "Jan", 3) == 0)
    {
      mon = 1;
    }
  else if (strncmp(ptr, "Feb", 3) == 0)
    {
      mon = 2;
    }
  else if (strncmp(ptr, "Mar", 3) == 0)
    {
      mon = 3;
    }
  else if (strncmp(ptr, "Apr", 3) == 0)
    {
      mon = 4;
    }
  else if (strncmp(ptr, "May", 3) == 0)
    {
      mon = 5;
    }
  else if (strncmp(ptr, "Jun", 3) == 0)
    {
      mon = 6;
    }
  else if (strncmp(ptr, "Jul", 3) == 0)
    {
      mon = 7;
    }
  else if (strncmp(ptr, "Aug", 3) == 0)
    {
      mon = 8;
    }
  else if (strncmp(ptr, "Sep", 3) == 0)
    {
      mon = 9;
    }
  else if (strncmp(ptr, "Oct", 3) == 0)
    {
      mon = 10;
    }
  else if (strncmp(ptr, "Nov", 3) == 0)
    {
      mon = 11;
    }
  else if (strncmp(ptr, "Dec", 3) == 0)
    {
      mon = 12;
    }
  else
    {
      fprintf(stderr, "ERROR line %ld: Unrecognized month %s\n", g_lineno, ptr);
      exit(EXIT_FAILURE);
    }

  /* Get the day of the month */

  ptr  = next;
  next = strchr(ptr, ' ');
  if (next == NULL)
    {
      fprintf(stderr, "ERROR line %ld: Did not find day of the month\n", g_lineno);
      exit(EXIT_FAILURE);
    }

  endptr = next;
  next   = skip_space(next);
  trim_end(ptr, endptr);

  date = strtoul(ptr, &endptr, 10);
  if (*endptr != '\0' || date > 31)
    {
      fprintf(stderr, "ERROR line %ld: Invalid date\n", g_lineno);
      exit(EXIT_FAILURE);
    }

  /* Skip over the time */

  ptr = next;
  next = strchr(ptr, ' ');
  if (next == NULL)
    {
      fprintf(stderr, "ERROR line %ld: Did not find day of the month\n", g_lineno);
      exit(EXIT_FAILURE);
    }

  /* Get the year */

  ptr = skip_space(next);
  next = strchr(ptr, ' ');
  if (next == NULL)
    {
      fprintf(stderr, "ERROR line %ld: Did not find year\n", g_lineno);
      exit(EXIT_FAILURE);
    }

  trim_end(ptr, next);
  year = strtoul(ptr, &endptr, 10);
  if (*endptr != '\0' || year < 2007 || year >= 3000)
    {
      fprintf(stderr, "ERROR line %ld: Invalid year: %s\n", g_lineno, ptr);
      exit(EXIT_FAILURE);
    }

  /* Then create the final date string */

 asprintf(&alloc, "(%04lu-%02u-%02lu)", year, mon, date);
 if (alloc == NULL)
   {
      fprintf(stderr, "ERROR line %ld: asprintf failed\n", g_lineno);
      exit(EXIT_FAILURE);
   }

 return alloc;
}

/* Parse the entire file */

static void parse_file(FILE *stream)
{
  enum parse_state_e state;
  bool lastblank;
  bool firstline;
  bool consumed;
  bool merge;
  char *name;
  char *date;
  char *ptr;

  /* Loop until the entire file has been parsed. */

  g_lineno  = 0;
  state     = STATE_IDLE;
  name      = NULL;
  date      = NULL;
  consumed  = true;
  firstline = true;
  merge     = false;

  for (; ; )
    {
      /* Read the next line from the file (unless it was not consumed on the
       * previous pass through the loop.
       */

      if (consumed)
        {
          g_line[LINESIZE] = '\0';
          if (!fgets(g_line, LINESIZE, stream))
            {
              /* Check if the body was terminated with the end of file */

              if (state == STATE_BODY)
                {
                /* End of body.  Add author, date, and final newline */

                printf("  From %s %s.\n", name, date);
                free(name);
                free(date);
                }
              else if (state != STATE_IDLE)
                {
                   fprintf(stderr, "ERROR line %ld: Unexpected EOF in state %d\n",
                           g_lineno, state);
                   exit(EXIT_FAILURE);
                }

              return;
            }

          g_lineno++;
          consumed = false;
        }

      ptr = g_line;

      /* Process the line depending upon the state of the parser */

      switch (state)
        {
          case STATE_IDLE:       /* Finished last, ready to start next */
            if (blank_line())
              {
                consumed = true;
                break;
              }

            if (isspace(g_line[0]))
              {
                fprintf(stderr, "ERROR line %lu: Unexpected whitespace in state %d\n",
                        g_lineno, state);
                exit(EXIT_FAILURE);
              }

            /* Change state and fall through */

            state = STATE_HEADER;

          case STATE_HEADER:     /* Started next, parsing header */
            if (!isspace(g_line[0]))
              {
                if (strncmp(g_line, "commit ", 7) == 0)
                  {
                    /* Skip commit line */
                  }
                else if (strncmp(g_line, "Merge: ", 6) == 0)
                  {
                    /* Skip merges */

                    merge = true;
                  }
                else if (strncmp(g_line, "Author: ", 8) == 0)
                  {
                    char *endptr;

                    /* Extract the name */

                    ptr += 7;
                    ptr = skip_space(ptr);
                    endptr = strchr(ptr, '<');
                    if (endptr == NULL)
                      {
                        fprintf(stderr, "ERROR line %lu: No email address after name, state %d\n",
                                g_lineno, state);
                        exit(EXIT_FAILURE);
                      }

                    if (name != NULL)
                      {
                        fprintf(stderr, "ERROR line %lu: Duplicate name, state %d\n",
                                g_lineno, state);
                        exit(EXIT_FAILURE);
                      }

                    /* Copy and save the name string */

                    trim_end(ptr, endptr - 1);
                    name = strdup(ptr);
                    if (name == NULL)
                      {
                        fprintf(stderr, "ERROR line %lu: Failed to duplicate name, state %d\n",
                                g_lineno, state);
                        exit(EXIT_FAILURE);
                      }
                  }
                else if (strncmp(g_line, "Date: ", 6) ==0)
                  {
                    if (date != NULL)
                      {
                        fprintf(stderr, "ERROR line %lu: Duplicate date, state %d\n",
                                g_lineno, state);
                        exit(EXIT_FAILURE);
                      }

                    /* Extract the date */

                    ptr += 6;
                    ptr  = skip_space(ptr);
                    date = parse_date(ptr);
                  }
                else
                  {
                    fprintf(stderr, "ERROR line %lu: Unrecognized header line in state %d\n",
                            g_lineno, state);
                    exit(EXIT_FAILURE);
                  }

                consumed = true;
                break;
              }

            /* Change state and fall through */

            if (name == NULL || date == NULL)
              {
                fprintf(stderr, "ERROR line %lu: name or date not found in header. State %d\n",
                        g_lineno, state);
                exit(EXIT_FAILURE);
              }

            state = STATE_ENDHEADER;

          case STATE_ENDHEADER:  /* Finished the header looking for the start of the body */
            if (blank_line())
              {
                consumed = true;
                break;
              }

            if (!isspace(g_line[0]))
              {
                fprintf(stderr, "ERROR line %lu: Unexpected in state %d\n",
                        g_lineno, state);
                exit(EXIT_FAILURE);
              }

            /* Change state and fall through */

            state     = STATE_BODY;
            lastblank = false;

          case STATE_BODY:       /* Parsing the body */
            if (blank_line())
              {
                lastblank = true;
                consumed  = true;
                break;
              }

            if (isspace(g_line[0]))
              {
                char *endptr;
                char *tmp;

                /* Remove the newline from the end */

                ptr = skip_space(ptr);
                endptr = &ptr[strlen(ptr) - 1];
                trim_end(ptr, endptr);

                /* Change leading "* " to "- " */

                tmp = ptr;
                if (ptr[0] == '*' && ptr[1] == ' ')
                  {
                    *ptr = '-';
                    tmp = ptr + 2;
                  }

                /* Skip over certain crap body lines added by GIT; Skip over
                 * merge entries altogether.
                 */

                if (strncmp(tmp, "Merged in ", 10) != 0 &&
                    strncmp(tmp, "Approved-by: ", 13) != 0 &&
                    strncmp(tmp, "Signed-off-by: ", 15) != 0 &&
                    !merge)
                  {
                    /* Is this the first paragraph in the body? */

                    if (firstline)
                      {
                        printf("\t* %s", ptr);
                      }
                    else
                      {
                        /* This paragraph is not the first, was it separated
                         * from the previous paragraph by a blank line?
                         */

                        if (lastblank)
                          {
                            putchar('\n');
                          }

                        printf("\n\t  %s", ptr);
                      }

                    firstline = false;
                    lastblank = false;
                  }

                consumed  = true;
                break;
              }

            /* End of body.  Add author, date, and final newline */

            if (!merge)
              {
                printf("  From %s %s.\n", name, date);
              }

            /* Revert to IDLE state */

            free(name);
            name  = NULL;
            free(date);
            date  = NULL;

            state = STATE_IDLE;
            firstline = true;
            merge = false;
            break;

          default:
            fprintf(stderr, "ERROR line %lu: Bad state %d\n", g_lineno, state);
            exit(EXIT_FAILURE);
        }
   }
}

static void show_usage(const char *progname)
{
  fprintf(stderr, "USAGE: %s <abs path to git log file>\n", progname);
  exit(EXIT_FAILURE);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv, char **envp)
{
  FILE *stream;

  if (argc != 2)
    {
      fprintf(stderr, "Unexpected number of arguments\n");
      show_usage(argv[0]);
    }

  stream = fopen(argv[1], "r");
  if (!stream)
    {
      fprintf(stderr, "open %s failed: %s\n", argv[1], strerror(errno));
      return EXIT_FAILURE;
    }

  parse_file(stream);
  fclose(stream);
  return EXIT_SUCCESS;
}