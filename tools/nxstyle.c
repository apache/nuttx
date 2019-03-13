/****************************************************************************
 * tools/nxstyle.c
 *
 *   Copyright (C) 2015, 2018-2019 Gregory Nutt. All rights reserved.
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
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN  ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <ctype.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LINE_SIZE    512

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void show_usage(char *progname, int exitcode)
{
  fprintf(stderr, "Usage:  %s [-m <maxline>] <filename>\n", progname);
  fprintf(stderr, "        %s -h\n", progname);
  exit(exitcode);
}

static void check_spaces_left(char *line, int lineno, int ndx)
{
  /* Unary operator should generally be preceded by a space but make also
   * follow a left parenthesis at the beginning of a parthentical list or
   * expression or follow a right parentheses in the case of a cast.
   */

  if (ndx > 0 && line[ndx - 1] != ' ' && line[ndx - 1] != '(' && line[ndx - 1] != ')')
    {
      fprintf(stderr,
              "Operator/assignment must be preceded with whitespace at line %d:%d\n",
              lineno, ndx);
    }
}

static void check_spaces_leftright(char *line, int lineno, int ndx1, int ndx2)
{
  if (ndx1 > 0 && line[ndx1 - 1] != ' ')
    {
      fprintf(stderr,
              "Operator/assignment must be preceded with whitespace at line %d:%d\n",
              lineno, ndx1);
    }

  if (line[ndx2 + 1] != '\0' && line[ndx2 + 1] != '\n' && line[ndx2 + 1] != ' ')
    {
      fprintf(stderr,
              "Operator/assignment must be followed with whitespace at line %d:%d\n",
              lineno, ndx2);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv, char **envp)
{
  FILE *instream;       /* File input stream */
  char line[LINE_SIZE]; /* The current line being examined */
  char *filename;       /* Name of the file to open */
  char *lptr;           /* Temporary pointer into line[] */
  bool btabs;           /* True: TAB characters found on the line */
  bool bfunctions;      /* True: In private or public functions */
  bool bstatm;          /* True: This line is beginning of a statement */
  bool bfor;            /* True: This line is beginning of a 'for' statement */
  bool bswitch;         /* True: Within a switch statement */
  bool bstring;         /* True: Within a string */
  bool bquote;          /* True: Backslash quoted character next */
  bool bblank;          /* Used to verify block comment termintor */
  bool ppline;          /* True: The next line the continuation of a precessor command */
  int lineno;           /* Current line number */
  int indent;           /* Indentation level */
  int ncomment;         /* Comment nesting level on this line */
  int prevncomment;     /* Comment nesting level on the previous line */
  int bnest;            /* Brace nesting level on this line */
  int prevbnest;        /* Brace nesting level on the previous line */
  int dnest;            /* Data declaration nesting level on this line */
  int prevdnest;        /* Data declaration nesting level on the previous line */
  int pnest;            /* Parenthesis nesting level on this line */
  int comment_lineno;   /* Line on which the last one line comment was closed */
  int blank_lineno;     /* Line number of the last blank line */
  int noblank_lineno;   /* A blank line is not needed after this line */
  int lbrace_lineno;    /* Line number of last left brace */
  int linelen;          /* Length of the line */
  int maxline;          /* Lines longer that this generate warnings */
  int n;
  int i;

  maxline  = 78;
  filename = argv[1];

  /* Usage:  nxstyle [-m <maxline>] <filename>
   *         nxstyle -h
   */

  if (argc == 4)
    {
      if (strcmp(argv[1], "-m") != 0)
        {
          fprintf(stderr, "Unrecognized argument\n");
          show_usage(argv[0], 1);
        }

      maxline = atoi(argv[2]);
      if (maxline < 1)
        {
          fprintf(stderr, "Bad value for <maxline>\n");
          show_usage(argv[0], 1);
        }

      filename = argv[3];
    }
  else if (argc == 2)
    {
      if (strcmp(argv[1], "-h") == 0)
        {
          show_usage(argv[0], 0);
        }
    }
  else
    {
      fprintf(stderr, "Invalid number of arguments\n");
      show_usage(argv[0], 1);
    }

  instream = fopen(filename, "r");
  if (!instream)
    {
      fprintf(stderr, "Failed to open %s\n", argv[1]);
      return 1;
    }

  btabs          = false; /* True: TAB characters found on the line */
  bfunctions     = false; /* True: In private or public functions */
  bswitch        = false; /* True: Within a switch statement */
  bstring        = false; /* True: Within a string */
  ppline         = false; /* True: Continuation of a pre-processor line */
  lineno         = 0;     /* Current line number */
  ncomment       = 0;     /* Comment nesting level on this line */
  bnest          = 0;     /* Brace nesting level on this line */
  dnest          = 0;     /* Data declaration nesting level on this line */
  pnest          = 0;     /* Parenthesis nesting level on this line */
  comment_lineno = -1;    /* Line on which the last one line comment was closed */
  blank_lineno   = -1;    /* Line number of the last blank line */
  noblank_lineno = -1;    /* A blank line is not needed after this line */
  lbrace_lineno  = -1;    /* Line number of last left brace */

  /* Process each line in the input stream */

  while (fgets(line, LINE_SIZE, instream))
    {
      lineno++;
      indent       = 0;
      prevbnest    = bnest;    /* Brace nesting level on the previous line */
      prevdnest    = dnest;    /* Data declaration nesting level on the previous line */
      prevncomment = ncomment; /* Comment nesting level on the previous line */
      bstatm       = false;    /* True: This line is beginning of a statement */
      bfor         = false;    /* REVISIT: Implies for() is all on one line */

      /* Check for a blank line */

      for (n = 0; line[n] != '\n' && isspace((int)line[n]); n++)
        {
        }

      if (line[n] == '\n')
        {
          if (n > 0)
            {
              fprintf(stderr, "Blank line contains whitespace at line %d\n",
                      lineno);
            }

          if (lineno == 1)
            {
              fprintf(stderr,  "File begins with a blank line\n");
            }
          else if (lineno == blank_lineno + 1)
            {
              fprintf(stderr,  "Too many blank lines at line %d\n", lineno);
            }
          else if (lineno == lbrace_lineno + 1)
            {
              fprintf(stderr, "Blank line follows left brace at line %d\n",
                      lineno);
            }

          blank_lineno = lineno;
          continue;
        }
      else /* This line is non-blank */
        {
          /* Check for a missing blank line after a comment */

          if (lineno == comment_lineno + 1)
            {
              /* No blank line should be present if the current line contains
               * a right brace, a pre-processor line, the start of another
               * comment.
               *
               * REVISIT: Generates a false alarm if the current line is also
               * a comment.  Generally it is acceptable for one comment to
               * follow another with no space separation.
               */

              if (line[n] != '}' /* && line[n] != '#' */)
                {
                  fprintf(stderr,
                          "Missing blank line after comment found at line %d\n",
                          comment_lineno);
                }
            }

          /* Files must begin with a comment (the file header).
           * REVISIT:  Logically, this belongs in the STEP 2 operations
           * below.
           */

          if (lineno == 1 && (line[n] != '/' || line[n + 1] != '*'))
            {
              fprintf(stderr,
                      "Missing file header comment block at line 1\n");
            }
        }

      /* STEP 1: Find the indentation level and the start of real stuff on
       * the line.
       */

      for (n = 0; line[n] != '\n' && isspace((int)line[n]); n++)
        {
          switch (line[n])
            {
            case ' ':
              {
                indent++;
              }
              break;

            case '\t':
              {
                if (!btabs)
                  {
                    fprintf(stderr, "TABs found.  First at line %d:%d\n", lineno, n);
                    btabs = true;
                  }

                indent = (indent + 4) & ~3;
              }
              break;

            default:
              {
                fprintf(stderr,
                        "Unexpected white space character %02x found at line %d:%d\n",
                        line[n], lineno, n);
              }
              break;
            }
        }

      /* STEP 2: Detect some certain start of line conditions */
      /* Skip over pre-processor lines (or continuations of pre-processor
       * lines as indicated by ppline)
       */

      if (line[indent] == '#' || ppline)
        {
          int len;

          /* Suppress error for comment following conditional compilation */

          noblank_lineno = lineno;

          /* Check if the next line will be a continuation of the pre-
           * processor command.
           */

          len = strlen(&line[indent]) + indent - 1;
          if (line[len] == '\n')
            {
              len--;
            }

          ppline = (line[len] == '\\');
          continue;
        }

      /* Check for a single line comment */

      linelen = strlen(line);
      if (linelen >= 5)      /* Minimum is slash, star, star, slash, newline */
        {
          lptr = strstr(line, "*/");
          if (line[indent] == '/' && line[indent +1] == '*' &&
              lptr - line == linelen - 3)
            {
              /* Check if there should be a blank line before the comment */

              if (lineno > 1 &&
                  comment_lineno != lineno - 1 &&
                  blank_lineno   != lineno - 1 &&
                  noblank_lineno != lineno - 1)
                {
                  /* TODO:  This generates a false alarm if preceded by a label. */

                  fprintf(stderr,
                          "Missing blank line before comment found at line %d\n",
                          lineno);
                }

              comment_lineno = lineno;
            }
        }

      /* Check for the comment block indicating the beginning of functions. */

      if (!bfunctions && ncomment > 0 &&
          (strcmp(line, " * Private Functions\n") == 0 ||
           strcmp(line, " * Public Functions\n") == 0))
        {
          //fprintf(stderr, "Functions begin at line %d:%d\n", lineno, n);
          bfunctions = true;
        }

      /* Check for some kind of declaration.
       * REVISIT: The following logic fails for any non-standard types.
       * REVISIT: Terminator after keyword might not be a space.  Might be
       * a newline, for example.  struct and unions are often unnamed, for
       * example.
       */

      else if (strncmp(&line[indent], "auto ", 5) == 0 ||
               strncmp(&line[indent], "bool ", 5) == 0 ||
               strncmp(&line[indent], "char ", 5) == 0 ||
               strncmp(&line[indent], "CODE ", 5) == 0 ||
               strncmp(&line[indent], "const ", 6) == 0 ||
               strncmp(&line[indent], "double ", 7) == 0 ||
//             strncmp(&line[indent], "struct ", 7) == 0 ||
               strncmp(&line[indent], "struct", 6) == 0 ||      /* May be unnamed */
               strncmp(&line[indent], "enum ", 5) == 0 ||
               strncmp(&line[indent], "extern ", 7) == 0 ||
               strncmp(&line[indent], "EXTERN ", 7) == 0 ||
               strncmp(&line[indent], "FAR ", 4) == 0 ||
               strncmp(&line[indent], "float ", 6) == 0 ||
               strncmp(&line[indent], "int ", 4) == 0 ||
               strncmp(&line[indent], "int16_t ", 8) == 0 ||
               strncmp(&line[indent], "int32_t ", 8) == 0 ||
               strncmp(&line[indent], "long ", 5) == 0 ||
               strncmp(&line[indent], "off_t ", 6) == 0 ||
               strncmp(&line[indent], "register ", 9) == 0 ||
               strncmp(&line[indent], "short ", 6) == 0 ||
               strncmp(&line[indent], "signed ", 7) == 0 ||
               strncmp(&line[indent], "size_t ", 7) == 0 ||
               strncmp(&line[indent], "ssize_t ", 8) == 0 ||
               strncmp(&line[indent], "static ", 7) == 0 ||
               strncmp(&line[indent], "time_t ", 7) == 0 ||
               strncmp(&line[indent], "typedef ", 8) == 0 ||
               strncmp(&line[indent], "uint8_t ", 8) == 0 ||
               strncmp(&line[indent], "uint16_t ", 9) == 0 ||
               strncmp(&line[indent], "uint32_t ", 9) == 0 ||
//             strncmp(&line[indent], "union ", 6) == 0 ||
               strncmp(&line[indent], "union", 5) == 0 ||      /* May be unnamed */
               strncmp(&line[indent], "unsigned ", 9) == 0 ||
               strncmp(&line[indent], "void ", 5) == 0 ||
               strncmp(&line[indent], "volatile ", 9) == 0)
        {
          /* REVISIT: Also picks up function return types */
          /* REVISIT: Logic problem for nested data/function declarations */

          if ((!bfunctions || bnest > 0) && dnest == 0)
            {
              dnest = 1;
            }

          /* Check for multiple definitions of variables on the line.
           * Ignores declarations within parentheses which are probably
           * formal parameters.
           */

          if (pnest == 0)
            {
              int tmppnest;

              /* Note, we have not yet parsed each character on the line so
               * a comma have have been be preceded by '(' on the same line.
               * We will have parse up to any comma to see if that is the
               * case.
               */

              for (i = indent, tmppnest = 0;
                   line[i] != '\n' && line[i] != '\0';
                   i++)
                {
                  if (tmppnest == 0 && line[i] == ',')
                    {
                      fprintf(stderr,
                              "Multiple data definitions on line %d\n",
                              lineno);
                      break;
                    }
                  else if (line[i] == '(')
                    {
                      tmppnest++;
                    }
                  else if (line[i] == ')')
                    {
                      if (tmppnest < 1)
                        {
                          /* We should catch this later */

                          break;
                        }

                      tmppnest--;
                    }
                  else if (line[i] == ';')
                    {
                      /* Break out if the semicolon terminates the
                       * declaration is found.  Avoids processing any
                       * righthand comments in most cases.
                       */

                      break;
                    }
                }
            }
        }

      /* Check for a keyword indicating the beginning of a statement.
       * REVISIT:  This, obviously, will not detect statements that do not
       * begin with a C keyword (such as assignment statements).
       */

      else if (strncmp(&line[indent], "break ", 6) == 0 ||
               strncmp(&line[indent], "case ", 5) == 0 ||
//             strncmp(&line[indent], "case ", 5) == 0 ||    /* Part of switch */
               strncmp(&line[indent], "continue ", 9) == 0 ||
//             strncmp(&line[indent], "default ", 8) == 0 || /* Part of switch */
               strncmp(&line[indent], "do ", 3) == 0 ||
               strncmp(&line[indent], "else ", 5) == 0 ||
               strncmp(&line[indent], "goto ", 5) == 0 ||
               strncmp(&line[indent], "if ", 3) == 0 ||
               strncmp(&line[indent], "return ", 7) == 0 ||
//             strncmp(&line[indent], "switch ", 7) == 0 ||  /* Doesn't follow pattern */
               strncmp(&line[indent], "while ", 6) == 0)
        {
          bstatm = true;
        }

      /* Spacing works a little differently for and switch statements */

      else if (strncmp(&line[indent], "for ", 4) == 0)
        {
          bfor   = true;
          bstatm = true;
        }
      else if (strncmp(&line[indent], "switch ", 7) == 0)
        {
          bswitch = true;
        }

      /* Also check for C keywords with missing white space */

      else if (strncmp(&line[indent], "do(", 3) == 0 ||
               strncmp(&line[indent], "if(", 3) == 0 ||
               strncmp(&line[indent], "while(", 6) == 0)
        {
          fprintf(stderr, "Missing whitespace after keyword at line %d:%d\n", lineno, n);
          bstatm = true;
        }
      else if (strncmp(&line[indent], "for(", 4) == 0)
        {
          fprintf(stderr, "Missing whitespace after keyword at line %d:%d\n", lineno, n);
          bfor   = true;
          bstatm = true;
        }
      else if (strncmp(&line[indent], "switch(", 7) == 0)
        {
          fprintf(stderr, "Missing whitespace after keyword at line %d:%d\n",
                  lineno, n);
          bswitch = true;
        }

      /* STEP 3: Parse each character on the line */

      bquote = false;   /* True: Backslash quoted character next */
      bblank = true;    /* Used to verify block comment termintor */

      for (; line[n] != '\n' && line[n] != '\0'; n++)
        {
          /* Skip over indentifiers */

          if (ncomment == 0 && !bstring && (line[n] == '_' || isalpha(line[n])))
            {
              bool have_upper = false;
              bool have_lower = false;
              int ident_index = n;

              /* Parse over the identifier.  Check if it contains mixed upper-
               * and lower-case characters.
               */

              do
                {
                  have_upper |= isupper(line[n]);

                  /* The coding standard provides for some exceptions of lower
                   * case characters in pre-processor strings:
                   *
                   *   IPv[4|6]    as an IP version number
                   *   ICMPv6      as an ICMP version number
                   *   IGMPv2      as an IGMP version number
                   *   [0-9]p[0-9] as a decimal point
                   *   d[0-9]      as a divisor
                   */

                   if (!have_lower && islower(line[n]))
                     {
                       switch (line[n])
                       {
                         /* A sequence containing 'v' may occur at the
                          * beginning of the identifier.
                          */

                         case 'v':
                           if (n > 1 &&
                               line[n - 2] == 'I' &&
                               line[n - 1] == 'P' &&
                               (line[n + 1] == '4' ||
                                line[n + 1] == '6'))
                             {
                             }
                           else if (n > 3 &&
                                    line[n - 4] == 'I' &&
                                    line[n - 3] == 'C' &&
                                    line[n - 2] == 'M' &&
                                    line[n - 1] == 'P' &&
                                    line[n + 1] == '6')
                             {
                             }
                           else if (n > 3 &&
                                    line[n - 4] == 'I' &&
                                    line[n - 3] == 'G' &&
                                    line[n - 2] == 'M' &&
                                    line[n - 1] == 'P' &&
                                    line[n + 1] == '2')
                             {
                             }
                           else
                             {
                               have_lower = true;
                             }
                           break;

                         /* Sequences containing 'p' or 'd' must have been
                          * preceded by upper case characters.
                          */

                         case 'p':
                           if (!have_upper || n < 1 ||
                               !isdigit(line[n - 1]) ||
                               !isdigit(line[n + 1]))
                             {
                               have_lower = true;
                             }
                             break;

                         case 'd':
                           if (!have_upper || !isdigit(line[n + 1]))
                             {
                               have_lower = true;
                             }
                             break;

                         default:
                           have_lower = true;
                           break;
                       }
                     }

                  n++;
                }
              while (line[n] == '_' || isalnum(line[n]));

              /* Check for mixed upper and lower case */

              if (have_upper && have_lower)
                {
                  /* Special case hex constants.  These will look like
                   * identifiers starting with 'x' or 'X' but preceded
                   * with '0'
                   */

                  if (ident_index < 1 ||
                      (line[ident_index] != 'x' && line[ident_index] != 'X') ||
                      line[ident_index - 1] != '0')
                    {
                      fprintf(stderr,
                              "Mixed case identifier found at line %d:%d\n",
                              lineno, ident_index);
                    }
                  else if (have_upper)
                    {
                      fprintf(stderr,
                              "Upper case hex constant found at line %d:%d\n",
                              lineno, ident_index);
                    }
                }

              /* Check if the identifier is the last thing on the line */

              if (line[n] == '\n' || line[n] == '\0')
                {
                  break;
                }
            }

          /* Handle comments */

          if (line[n] == '/' && !bstring)
            {
              /* Check for start of a C comment */

              if (line[n + 1] == '*')
                {
                  if (line[n + 2] == '\n')
                    {
                      fprintf(stderr, "C comment on separate line at %d:%d\n",
                              lineno, n);
                    }
                  else if (line[n + 2] != ' ' && line[n + 2] != '*')
                    {
                      fprintf(stderr,
                              "Missing space after opening C comment at line %d:%d\n",
                              lineno, n);
                    }

                  ncomment++;
                  n++;
                  continue;
                }

              /* Check for end of a C comment */

              else if (n > 0 && line[n - 1] == '*')
                {
                  if (n < 2)
                    {
                      fprintf(stderr, "Closing C comment not indented at line %d:%d\n",
                              lineno, n);
                    }
                  else if (line[n - 2] != ' ' && line[n - 2] != '*')
                    {
                      fprintf(stderr,
                              "Missing space before closing C comment at line %d:%d\n",
                              lineno, n);
                    }

                  /* Check for block comments that are not on a separate line.
                   * This would be the case if we are we are within a comment
                   * that did not start on this line and the current line is
                   * not blank up to the point where the comment was closed.
                   */

                  if (prevncomment > 0 && !bblank)
                    {
                      fprintf(stderr,
                              "Block comment terminator must be on a separate line at line %d:%d\n",
                              lineno, n);
                    }

#if 0
                  /* REVISIT: Generates false alarms when portions of an
                   * expression are commented out within the expression.
                   */

                  if (line[n + 1] != '\n')
                    {
                      fprintf(stderr,
                              "Garbage on line after C comment at line %d:%d\n",
                              lineno, n);
                    }
#endif

                  /* Handle nested comments */

                  if (ncomment > 0)
                    {
                      /* Remember the line number of the line containing the
                       * closing of the outermost comment.
                       */

                      if (--ncomment == 0)
                        {
#if 0
                          /* REVISIT: causes false alarms when comment appears to
                           * the right of a statement.
                           */

                          comment_lineno = lineno;
#endif
                        }
                    }
                  else
                    {
                      ncomment = 0;
                      fprintf(stderr,
                              "Closing without opening comment at line %d:%d\n",
                              lineno, n);
                    }

                  n++;
                  continue;
                }

              /* Check for C++ style comments
               * NOTE: Gives false alarms on URLs (http://...) embedded
               * inside of comments.
               */

              else if (line[n + 1] == '/')
                {
                  fprintf(stderr, "C++ style comment at %d:%d\n",
                          lineno, n);
                  n++;
                  continue;
                }
            }

          /* Check if the line is blank so far.  This is only used to
           * to verify the the closing of a block comment is on a separate
           * line.  So we also need to treat '*' as a 'blank'.
           */

          if (!isblank(line[n]) && line[n] != '*')
            {
              bblank = false;
            }

          /* Check for a string... ignore if we are in the middle of a
           * comment.
           */

          if (ncomment == 0)
            {
              /* Backslash quoted charater */

              if (line[n] == '\\')
                {
                  bquote = true;
                  n++;
                }

              /* Check for quoated characters: \" in string */

              if (line[n] == '"' && !bquote)
                {
                  bstring = !bstring;
                }

              bquote = false;
            }

          /* The reset of the line is only examined of we are not in a comment
           * or a string.
           *
           * REVISIT: Should still check for whitespace at the end of the
           * line.
           */

          if (ncomment == 0 && !bstring)
            {
              switch (line[n])
                {
                /* Handle logic nested with curly braces */

                case '{':
                  {
                    if (n > indent)
                      {
                        if (dnest == 0)
                          {
                            fprintf(stderr,
                                    "Left bracket not on separate line at %d:%d\n",
                                    lineno, n);
                          }
                      }
                    else if (line[n + 1] != '\n')
                      {
                        if (dnest == 0)
                          {
                            fprintf(stderr,
                                    "Garbage follows left bracket at line %d:%d\n",
                                    lineno, n);
                          }
                      }

                    bnest++;
                    if (dnest > 0)
                      {
                        dnest++;
                      }

                    /* Suppress error for comment following a left brace */

                    noblank_lineno = lineno;
                    lbrace_lineno  = lineno;
                  }
                  break;

                case '}':
                  {
                   /* Decrement the brace nesting level */

                   if (bnest < 1)
                     {
                       fprintf(stderr, "Unmatched right brace at line %d:%d\n", lineno, n);
                     }
                   else
                     {
                       bnest--;
                       if (bnest < 1)
                         {
                           bnest = 0;
                           bswitch = false;
                         }
                     }

                    /* Decrement the declaration nesting level */

                    if (dnest < 3)
                      {
                        dnest = 0;
                      }
                    else
                      {
                        dnest--;
                      }

                    /* The right brace should be on a separate line */

                    if (n > indent)
                      {
                        if (dnest == 0)
                          {
                            fprintf(stderr,
                                    "Right bracket not on separate line at %d:%d\n",
                                   lineno, n);
                          }
                      }

                    /* Check for garbage following the left brace */

                    if (line[n + 1] != '\n' &&
                        line[n + 1] != ',' &&
                        line[n + 1] != ';')
                      {
                        /* One case where there may be garbage after the right
                         * bracket is, for example, when declaring a until or
                         * structure variable using an un-named union or
                         * structure.
                         */

                        if (prevdnest <= 0 || dnest > 0)
                          {
                            /* REVISIT:  Generates false alarms on named structures
                             * that are fields of other structures or unions.
                             */

                            fprintf(stderr,
                                    "Garbage follows right bracket at line %d:%d\n",
                                    lineno, n);
                          }
                      }

                    /* The right brace should not be preceded with a a blank line */


                    if (lineno == blank_lineno + 1)
                      {
                        fprintf(stderr,
                                "Blank line precedes right brace at line %d\n",
                                lineno);
                      }
                  }
                  break;

                /* Handle logic with parenthese */

                case '(':
                  {
                    /* Increase the parenthetical nesting level */

                    pnest++;

                   /* Check for inappropriate space around parentheses */

                    if (line[n + 1] == ' ' /* && !bfor */)
                      {
                        fprintf(stderr,
                                "Space follows left parenthesis at line %d:%d\n",
                                lineno, n);
                      }
                  }
                  break;

                case ')':
                  {
                    /* Decrease the parenthetical nesting level */

                    if (pnest < 1)
                     {
                       fprintf(stderr, "Unmatched right parentheses at line %d:%d\n",
                               lineno, n);
                       pnest = 0;
                     }
                   else
                     {
                       pnest--;
                     }

                    /* Allow ')' as first thing on the line (n == indent)
                     * Allow "for (xx; xx; )" (bfor == true)
                     */

                    if (n > 0 && n != indent && line[n - 1] == ' ' && !bfor)
                      {
                        fprintf(stderr,
                                "Space precedes right parenthesis at line %d:%d\n",
                                lineno, n);
                      }
                  }
                  break;

                /* Check for inappropriate space around square brackets */

                case '[':
                  {
                    if (line[n + 1] == ' ')
                      {
                        fprintf(stderr,
                                "Space follows left bracket at line %d:%d\n",
                                lineno, n);
                      }
                  }
                  break;

                case ']':
                  {
                    if (n > 0 && line[n - 1] == ' ')
                      {
                        fprintf(stderr,
                                "Space precedes right bracket at line %d:%d\n",
                                lineno, n);
                      }
                  }
                  break;

                /* Semi-colon may terminate a declaration */

                case ';':
                  {
                    if (!isspace((int)line[n + 1]))
                      {
                        fprintf(stderr, "Missing whitespace after semicolon at line %d:%d\n",
                                lineno, n);
                      }

                    /* Semicolon terminates a declaration/definition if there
                     * was no left curly brace (i.e., dnest is only 1).
                     */

                    if (dnest == 1)
                      {
                        dnest = 0;
                      }
                  }
                  break;

                /* Semi-colon may terminate a declaration */

                case ',':
                  {
                    if (!isspace((int)line[n + 1]))
                      {
                        fprintf(stderr, "Missing whitespace after comma at line %d:%d\n",
                                lineno, n);
                      }
                  }
                  break;

                case '\r':
                  {
                    fprintf(stderr,
                            "Carriage return detected at line %d:%d\n",
                            lineno, n);
                  }
                  break;

                /* Skip over character constants */

                case '\'':
                  {
                    int endndx = n + 2;

                    if (line[n + 1] != '\n' && line[n + 1] != '\0')
                      {
                        if (line[n + 1] == '\\')
                          {
                            for (;
                                 line[endndx] != '\n' &&
                                 line[endndx] != '\0' &&
                                 line[endndx] != '\'';
                                 endndx++);
                          }

                        n = endndx + 1;
                      }
                  }
                  break;

                /* Check for space around various operators */

                case '-':
                  /* ->, -- */

                  if (line[n + 1] == '>' || line[n + 1] == '-')
                    {
                      n++;
                    }

                  /* -= */

                  else if (line[n + 1] == '=')
                    {
                      check_spaces_leftright(line, lineno, n, n + 1);
                      n++;
                    }
                  else
                    {
                      /* '-' may function as a unary operator and snuggle
                       * on the left.
                       */

                      check_spaces_left(line, lineno, n);
                    }

                  break;

                case '+':
                  /* ++ */

                  if (line[n + 1] == '+')
                    {
                      n++;
                    }

                  /* += */

                  else if (line[n + 1] == '=')
                    {
                      check_spaces_leftright(line, lineno, n, n + 1);
                      n++;
                    }
                  else
                    {
                      /* '+' may function as a unary operator and snuggle
                       * on the left.
                       */

                      check_spaces_left(line, lineno, n);
                    }

                  break;

                case '&':

                  /* &<variable> OR &(<expression>) */

                  if (isalpha((int)line[n + 1]) || line[n + 1] == '_' ||
                      line[n + 1] == '(')
                    {
                    }

                  /* &&, &= */

                  else if (line[n + 1] == '=' || line[n + 1] == '&')
                    {
                      check_spaces_leftright(line, lineno, n, n + 1);
                      n++;
                    }
                  else
                    {
                      check_spaces_leftright(line, lineno, n, n);
                    }

                  break;

                case '/':
                  /* C comment terminator*/

                  if (line[n - 1] == '*')
                    {
                      n++;
                    }

                    /* C++-style comment */

                  else if (line[n + 1] == '/')
                    {
                      fprintf(stderr, "C++ style comment on at %d:%d\n",
                              lineno, n);
                      n++;
                    }

                  /* /= */

                  else if (line[n + 1] == '=')
                    {
                      check_spaces_leftright(line, lineno, n, n + 1);
                      n++;
                    }

                  /* Division operator */

                  else
                    {
                      check_spaces_leftright(line, lineno, n, n);
                    }

                  break;

                case '*':
                  /* *\/, ** */

                  if (line[n] == '*' &&
                      (line[n + 1] == '/' ||
                       line[n + 1] == '*'))
                    {
                     n++;
                     break;
                    }

                  /* *<variable>, *(<expression>) */

                  else if (isalpha((int)line[n + 1]) ||
                           line[n + 1] == '_' ||
                           line[n + 1] == '(')
                    {
                      break;
                    }

                  /* (<type> *) */

                  else if (line[n + 1] == ')')
                    {
                      /* REVISIT: This gives false alarms on syntax like *--ptr */

                      if (line[n - 1] != ' ')
                        {
                          fprintf(stderr,
                                  "Operator/assignment must be preceded with whitespace at line %d:%d\n",
                                  lineno, n);
                        }

                      break;
                    }

                  /* *= */

                  else if (line[n + 1] == '=')
                    {
                      check_spaces_leftright(line, lineno, n, n + 1);
                      n++;
                    }
                  else
                    {
                      /* A single '*' may be an binary operator, but
                       * it could also be a unary operator when used to deference
                       * a pointer.
                       */

                      check_spaces_left(line, lineno, n);
                    }

                  break;

                case '%':
                  /* %= */

                  if (line[n + 1] == '=')
                    {
                      check_spaces_leftright(line, lineno, n, n + 1);
                      n++;
                    }
                  else
                    {
                      check_spaces_leftright(line, lineno, n, n);
                    }

                  break;

                case '<':
                  /* <=, <<, <<= */

                  if (line[n + 1] == '=')
                    {
                      check_spaces_leftright(line, lineno, n, n + 1);
                      n++;
                    }
                  else if (line[n + 1] == '<')
                    {
                      if (line[n + 2] == '=')
                        {
                          check_spaces_leftright(line, lineno, n, n + 2);
                          n += 2;
                        }
                      else
                        {
                          check_spaces_leftright(line, lineno, n, n + 1);
                          n++;
                        }
                    }
                  else
                    {
                      check_spaces_leftright(line, lineno, n, n);
                    }

                  break;

                case '>':
                  /* >=, >>, >>= */

                  if (line[n + 1] == '=')
                    {
                      check_spaces_leftright(line, lineno, n, n + 1);
                      n++;
                    }
                  else if (line[n + 1] == '>')
                    {
                      if (line[n + 2] == '=')
                        {
                          check_spaces_leftright(line, lineno, n, n + 2);
                          n += 2;
                        }
                      else
                        {
                          check_spaces_leftright(line, lineno, n, n + 1);
                          n++;
                        }
                    }
                  else
                    {
                      check_spaces_leftright(line, lineno, n, n);
                    }

                  break;

                case '|':
                  /* |=, || */

                  if (line[n + 1] == '=')
                    {
                      check_spaces_leftright(line, lineno, n, n + 1);
                      n++;
                    }
                  else if (line[n + 1] == '|')
                    {
                      check_spaces_leftright(line, lineno, n, n + 1);
                      n++;
                    }
                  else
                    {
                      check_spaces_leftright(line, lineno, n, n);
                    }

                  break;

                case '^':
                  /* ^= */

                  if (line[n + 1] == '=')
                    {
                      check_spaces_leftright(line, lineno, n, n + 1);
                      n++;
                    }
                  else
                    {
                      check_spaces_leftright(line, lineno, n, n);
                    }

                  break;

                case '=':
                  /* == */

                  if (line[n + 1] == '=')
                    {
                      check_spaces_leftright(line, lineno, n, n + 1);
                      n++;
                    }
                  else
                    {
                      check_spaces_leftright(line, lineno, n, n);
                    }

                  break;

                case '~':
                  check_spaces_left(line, lineno, n);
                  break;

                case '!':
                  /* != */

                  if (line[n + 1] == '=')
                    {
                      check_spaces_leftright(line, lineno, n, n + 1);
                      n++;
                    }

                  /* !! */

                  else if (line[n + 1] == '!')
                    {
                      check_spaces_left(line, lineno, n);
                      n++;
                    }
                  else
                    {
                      check_spaces_left(line, lineno, n);
                    }

                  break;

                default:
                  break;
                }
            }
        }

      /* Loop terminates when NUL or newline character found */

      if (line[n] == '\n')
        {
          /* Check for space at the end of the line */

          if (n > 1 && isspace((int)line[n - 1]))
            {
              fprintf(stderr,
                      "Dangling whitespace at the end of line %d:%d\n",
                      lineno, n);
            }

          /* Check for long lines */

          if (n > maxline)
            {
              fprintf(stderr,
                      "Long line found at %d:%d\n",
                      lineno, n);
            }
        }

      /* STEP 4: Check alignment */

      /* Within a comment block, we need only check on the alignment of the
       * comment.
       */

      if ((ncomment > 0 || prevncomment > 0) && !bstring)
        {
          if (indent == 0 && line[0] != '/')
            {
              fprintf(stderr, "No indentation line %d:%d\n",
                      lineno, indent);
            }
          else if (indent == 1 && line[0] == ' ' && line[1] == '*')
            {
              /* Good indentation */
            }
          else if (indent > 0 && line[indent] == '\n')
            {
              fprintf(stderr, "Whitespace on blank line at line %d:%d\n",
                      lineno, indent);
            }
          else if (indent > 0 && indent < 2)
            {
              if (bnest > 0)
                {
                  fprintf(stderr, "Insufficient indentation line %d:%d\n",
                          lineno, indent);
                }
              else
                {
                  fprintf(stderr, "Expected indentation line %d:%d\n",
                          lineno, indent);
                }
            }
          else if (indent > 0 && !bswitch)
            {
              if (line[indent] == '/')
                {
                  if ((indent & 3) != 2)
                    {
                      fprintf(stderr,
                              "Bad comment alignment at line %d:%d\n",
                              lineno, indent);
                    }

                  /* REVISIT:  This screws up in cases where there is C code,
                   * followed by a comment that continues on the next line.
                   */

                  else if (line[indent+1] != '*')
                    {
                      fprintf(stderr,
                              "Missing asterisk in comment at line %d:%d\n",
                              lineno, indent);
                    }
                }
              else if (line[indent] == '*')
                {
                  /* REVISIT: Generates false alarms on comments at the end of
                   * the line if there is nothing preceding (such as the aligned
                   * comments with a structure field definition).  So disabled for
                   * comments before beginning of function definitions.
                   */

                  if ((indent & 3) != 3 && bfunctions && dnest == 0)
                    {
                      fprintf(stderr,
                              "Bad comment block alignment at line %d:%d\n",
                              lineno, indent);
                    }

                  if (line[indent+1] != ' ' &&
                      line[indent+1] != '\n' &&
                      line[indent+1] != '/')
                    {
                      fprintf(stderr,
                              "Invalid character after asterisk in comment block at line %d:%d\n",
                              lineno, indent);
                    }
                }

              /* If this is not the line containing the comment start, then this
               * line should begin with '*'
               */

              else if (prevncomment > 0)
                {
                  fprintf(stderr, "Missing asterisk in comment block at line %d:%d\n",
                          lineno, indent);
                }
            }
        }

      /* Check for various alignment outside of the comment block */

      else if ((ncomment > 0 || prevncomment > 0) && !bstring)
        {
          if (indent == 0 && strchr("\n#{}", line[0]) == NULL)
            {
               /* Ignore if we are at global scope */

               if (prevbnest > 0)
                {
                  bool blabel = false;

                  if (isalpha((int)line[indent]))
                    {
                      for (i = indent + 1; isalnum((int)line[i]) || line[i] == '_'; i++);
                      blabel = (line[i] == ':');
                    }

                  if (!blabel)
                    {
                      fprintf(stderr, "No indentation line %d:%d\n",
                              lineno, indent);
                    }
                }
            }
          else if (indent == 1 && line[0] == ' ' && line[1] == '*')
            {
              /* Good indentation */
            }
          else if (indent > 0 && line[indent] == '\n')
            {
              fprintf(stderr, "Whitespace on blank line at line %d:%d\n",
                      lineno, indent);
            }
          else if (indent > 0 && indent < 2)
            {
              fprintf(stderr, "Insufficient indentation line %d:%d\n",
                      lineno, indent);
            }
          else if (line[indent] == '{')
            {
              /* REVISIT:  False alarms in data initializers and switch statements */

              if ((indent & 3) != 0 && !bswitch && dnest == 0)
                {
                  fprintf(stderr, "Bad left brace alignment at line %d:%d\n",
                          lineno, indent);
                }
            }
          else if (line[indent] == '}')
            {
              /* REVISIT:  False alarms in data initializers and switch statements */

              if ((indent & 3) != 0 && !bswitch && prevdnest == 0)
                {
                  fprintf(stderr, "Bad right brace alignment at line %d:%d\n",
                          lineno, indent);
                }
            }
          else if (indent > 0)
            {
              /* REVISIT: Generates false alarms when a statement continues on
               * the next line.  The bstatm check limits to lines beginnnig with
               * C keywords.
               * REVISIT:  The bstatm check will not detect statements that
               * do not begin with a C keyword (such as assignement statements).
               * REVISIT: Generates false alarms on comments at the end of
               * the line if there is nothing preceding (such as the aligned
               * comments with a structure field definition).  So disabled for
               * comments before beginning of function definitions.
               */

              if ((bstatm ||                              /* Begins with C keyword */
                  (line[indent] == '/' && bfunctions)) && /* Comment in functions */
                  !bswitch &&                             /* Not in a switch */
                  dnest == 0)                             /* Not a data definition */
                {
                  if ((indent & 3) != 2)
                    {
                      fprintf(stderr, "Bad alignment at line %d:%d\n",
                              lineno, indent);
                    }
                }

              /* Crazy cases.  There should be no small odd alignements
               * outside of comment/string.  Odd alignments are possible
               * on continued lines, but not if they are small.
               */

              else if (indent == 1 || indent == 3)
                {
                  fprintf(stderr, "Small odd alignment at line %d:%d\n",
                          lineno, indent);
                }
            }
        }
    }

  if (ncomment > 0 || bstring)
    {
      fprintf(stderr, "In a comment/string at end of file\n");
    }

  fclose(instream);
  return 0;
}
