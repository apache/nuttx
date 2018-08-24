/****************************************************************************
 * tools/nxstyle.c
 *
 *   Copyright (C) 2015, 2018 Gregory Nutt. All rights reserved.
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

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LINE_SIZE    512

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv, char **envp)
{
  FILE *instream;
  char line[LINE_SIZE];
  char *lptr;
  bool btabs;
  bool bfunctions;
  bool bstatm;
  bool bfor;
  bool bswitch;
  bool bstring;
  bool bquote;
  int lineno;
  int indent;
  int prevnest;
  int ncomment;
  int nnest;
  int declnest;
  int prevdeclnest;
  int prevncomment;
  int n;
  int i;
  int comment_lineno;
  int blank_lineno;
  int noblank_lineno;
  int linelen;

  instream = fopen(argv[1], "r");
  if (!instream)
    {
      fprintf(stderr, "Failed to open %s\n", argv[1]);
      return 1;
    }

  btabs          = false;
  bfunctions     = false;
  bswitch        = false;
  bstring        = false;
  lineno         = 0;
  ncomment       = 0;
  nnest          = 0;
  declnest       = 0;
  prevdeclnest   = 0;
  prevncomment   = 0;
  comment_lineno = -1;   /* Line on which the last one line comment was closed */
  blank_lineno   = -1;   /* Line number of the last blank line */
  noblank_lineno = -1;   /* A blank line is not needed after this line */

  while (fgets(line, LINE_SIZE, instream))
    {
      lineno++;
      indent       = 0;
      prevnest     = nnest;
      prevdeclnest = declnest;
      prevncomment = ncomment;
      bstatm       = false;
      bfor         = false;  /* REVISIT: Implies for() is all on one line */

      /* Check for a blank line */

      if (line[0] == '\n')
        {
          if (lineno == blank_lineno + 1)
            {
              fprintf(stderr,  "Too many blank lines at line %d\n", lineno);
            }

          blank_lineno = lineno;
        }
      else /* this line is non-blank */
        {
          if (lineno == comment_lineno + 1)
            {
              /* TODO:  This generates a false alarm if the current line
               * contains a right brace or a pre-processor line.  No blank line
               * should be present in those cases.
               */

              fprintf(stderr,
                      "Missing blank line after comment line. Found at line %d\n",
                      comment_lineno);
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
                fprintf(stderr, "Unexpected white space character %02x found at line %d:%d\n", line[n], lineno, n);
              }
              break;
            }
        }

      /* STEP 2: Detect some certain start of line conditions */
      /* Skip over pre-processor lines */

      if (line[indent] == '#')
        {
          /* Suppress error for comment following conditional compilation */

          noblank_lineno = lineno;
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
              if (comment_lineno != lineno - 1 &&
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

          if ((!bfunctions || nnest > 0) && declnest == 0)
            {
              declnest = 1;
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
          fprintf(stderr, "Missing whitespace after keyword at line %d:%d\n", lineno, n);
          bswitch = true;
        }

      /* STEP 3: Parse each character on the line */

      bquote = false;
      for (; line[n] != '\n' && line[n] != '\0'; n++)
        {
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

                  if (ncomment > 0)
                    {
                      ncomment--;
                    }
                  else
                    {
                      ncomment = 0;
                      fprintf(stderr,
                              "Closing without opening comment at line %d:%d\n",
                              lineno, n);
                    }
                }

              /* Check for C++ style comments
               * NOTE: Gives false alarms on URLs (http://...) embedded
               * inside of comments.
               */

              else if (line[n + 1] == '/')
                {
                  fprintf(stderr, "C++ style comment on at %d:%d\n",
                          lineno, n);
                  n++;
                  continue;
                }
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

          /* The reset of the line is only examined of we are in a comment
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
                        if (declnest == 0)
                          {
                            fprintf(stderr,
                                    "Left bracket not on separate line at %d:%d\n",
                                    lineno, n);
                          }
                      }
                    else if (line[n + 1] != '\n')
                      {
                        if (declnest == 0)
                          {
                            fprintf(stderr,
                                    "Garbage follows left bracket at line %d:%d\n",
                                    lineno, n);
                          }
                      }

                    nnest++;
                    if (declnest > 0)
                      {
                        declnest++;
                      }

                    /* Suppress error for comment following a left brace */

                    noblank_lineno = lineno;
                  }
                  break;

                case '}':
                  {
                   if (nnest < 1)
                     {
                       fprintf(stderr, "Unmatched right brace at line %d:%d\n", lineno, n);
                     }
                   else
                     {
                       nnest--;
                       if (nnest < 1)
                         {
                           nnest = 0;
                           bswitch = false;
                         }
                     }

                    if (declnest < 3)
                      {
                        declnest = 0;
                      }
                    else
                      {
                        declnest--;
                      }

                    if (n > indent)
                      {
                        if (declnest == 0)
                          {
                            fprintf(stderr,
                                    "Right bracket not on separate line at %d:%d\n",
                                   lineno, n);
                          }
                      }
                    else if (line[n + 1] != '\n' &&
                             line[n + 1] != ',' &&
                             line[n + 1] != ';')
                      {
                        /* One case where there may be garbage after the right
                         * bracket is, for example, when declaring a until or
                         * structure variable using an un-named union or
                         * structure.
                         */

                        if (prevdeclnest <= 0 || declnest > 0)
                          {
                            fprintf(stderr,
                                    "Garbage follows right bracket at line %d:%d\n",
                                    lineno, n);
                          }
                      }
                  }
                  break;

                /* Check for inappropriate space around parentheses */

                case '(':
                  {
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
                     * was no left curly brace (i.e., declnest is only 1).
                     */

                    if (declnest == 1)
                      {
                        declnest = 0;
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

                /* Check for space at the end of the line */

                case '\n':
                  {
                    if (n > 0 && isspace((int)line[n - 1]))
                      {
                        fprintf(stderr,
                                "Dangling whitespace at the end of line %d:%d\n",
                                lineno, n);
                      }
                  }
                  break;

                /* Check for space around various operators */

                case '-':
                  /* -> */

                  if (line[n + 1] == '>')
                    {
                      n++;
                      break;
                    }

                case '+':
                  /* ++, -- */

                  if (line[n + 1] == line[n])
                    {
                      n++;
                      break;
                    }

                case '&':
                  /* && */

                  if (line[n] == '&' && line[n + 1] == line[n])
                    {
                      int curr;
                      int next;

                      curr = n;
                      n++;
                      next = n + 1;

                      if (line[curr-1] != ' ')
                        {
                          fprintf(stderr,
                                  "Operator/assignment must be preceded with whitespace at line %d:%d\n",
                                  lineno, curr);
                        }

                      if (line[next] != ' ' && line[next] != '\n')
                        {
                          fprintf(stderr,
                                  "Operator/assignment needs whitespace separation at line %d:%d\n",
                                  lineno, curr);
                        }

                      break;
                    }

                  /* &<variable> OR &(<expression>)*/

                  else if (isalpha((int)line[n + 1]) || line[n + 1] == '_' || line[n + 1] == '(')
                    {
                      break;
                    }

                case '/':
                  {
                    if (line[n] == '/')
                      {
                         if (line[n - 1] == '*')
                           {
                             n++;
                             break;
                           }
                         else if (line[n + 1] == '/')
                          {
                            fprintf(stderr, "C++ style comment on at %d:%d\n",
                                    lineno, n);
                             n++;
                             break;
                          }
                      }
                  }

                case '*':
                  {
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
                  }

                case '%':
                  {
                    if (isalnum((int)line[n + 1]))
                      {
                        break;
                      }
                  }

                case '<':
                case '>':
                case '|':
                case '^':
                case '=':
                  {
                    int curr;
                    int next;

                    curr = n;
                    if (line[curr-1] != ' ')
                      {
                        fprintf(stderr,
                                "Operator/assignment must be preceded with whitespace at line %d:%d\n",
                                lineno, curr);
                      }

                    next = n + 1;

                    /* <<, >>, <<=, >>= */

                    if (line[curr] == '>' || line[curr] == '<')
                      {
                        if (line[next] == line[curr])
                          {
                            next++;
                            n++;
                          }

                        if (line[next] == '=')
                          {
                            next++;
                            n++;
                          }
                      }
                    else if (line[next] == '=' || line[next] == line[n])
                      {
                        next++;
                        n++;
                      }

                    /* REVISIT: This gives false alarms on syntax like *--ptr */

                    if (line[curr] != '-' && line[next] != ' ' && line[next] != '\n')
                      {
                        fprintf(stderr,
                                "Operator/assignment needs whitespace separation at line %d:%d\n",
                                lineno, curr);
                      }
                  }
                  break;

                case '~':
                case '!':
                  {
                    int curr;
                    int next;

                    curr = n;
                    next = n + 1;
                    if (line[next] == '=' || line[next] == line[n])
                      {
                        next++;
                        n++;

                        if (line[next] != ' ' && line[next] != '\n')
                          {
                            fprintf(stderr,
                                    "Operator/assignment needs whitespace separation at line %d:%d\n",
                                    lineno, curr);
                          }
                      }

                    if (line[curr-1] != ' ' && line[curr-1] != '(')
                      {
                        fprintf(stderr,
                                "Operator/assignment must be preceded with whitespace at line %d:%d\n",
                                lineno, curr);
                      }
                  }
                  break;

                default:
                  break;
                }
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
              fprintf(stderr, "Insufficient indentation line %d:%d\n",
                      lineno, indent);
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

                  if ((indent & 3) != 3 && bfunctions && declnest == 0)
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

               if (prevnest > 0)
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

              if ((indent & 3) != 0 && !bswitch && declnest == 0)
                {
                  fprintf(stderr, "Bad left brace alignment at line %d:%d\n",
                          lineno, indent);
                }
            }
          else if (line[indent] == '}')
            {
              /* REVISIT:  False alarms in data initializers and switch statements */

              if ((indent & 3) != 0 && !bswitch && prevdeclnest == 0)
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
                  declnest == 0)                          /* Not a data definition */
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
