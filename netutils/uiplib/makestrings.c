/****************************************************************************
 * net/recv.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * Based loosely on a uIP perl script by Adam Dunkels
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <errno.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum eaction { GENSRC=0, SRCLIST=1 };

 /****************************************************************************
 * Private Data
 ****************************************************************************/

static const char *g_progname;
static const char *g_stringfile;
static enum eaction g_action;

static char g_line[1024];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: parse_stringfile_line
 ****************************************************************************/
 
void unescape_value(char *pvalue)
{
  const char *pin;
  char *pout;
  
  pin = pvalue;
  pout = pvalue;
  while (*pin)
    {
      if (*pin == '\\')
        {
          pin++;
          if ( *pin >= 0 && *pin <= 0)
            {
              char *pend;
              unsigned long val = strtoul(pin, &pend, 0);
              if (pend != pin)
                {
                  *pout++ = (char)val;
                  pin = pend;
                }
              else
                {
                  *pout++ = '\\';
                  *pout++ = *pin++;
                }
            }
          else
            {
              switch (*pin)
                {
                  case 'a':
                    *pout++ = 0x07;
                    break;
                  case 'b':
                    *pout++ = 0x08;
                    break;
                  case 't':
                    *pout++ = 0x09;
                    break;
                  case 'n':
                    *pout++ = 0x0a;
                    break;
                  case 'v':
                    *pout++ = 0x0b;
                  case 'f':
                    *pout++ = 0x0c;
                    break;
                  case 'r':
                    *pout++ = 0x0d;
                    break;
                  default:
                    *pout++ = *pin;
                    break;
                }
              pin++;
            }
        }
      else
        {
          *pout++ = *pin++;
        }
    }
  *pout = '\0';
}

/****************************************************************************
 * Function: parse_stringfile_line
 ****************************************************************************/
 
int parse_stringfile_line(const char **ppname, const char **ppvalue)
{
  char *ptmp;
  char *pname;
  char *pvalue;
  
  pname = g_line;
  ptmp  = strchr(g_line, ' ');
  if (ptmp)
    {
      *ptmp++ = '\0';
      pvalue = strchr(ptmp, '"');
      if (pvalue)
        {
          pvalue++;
          ptmp = strchr(pvalue, '"');
          if (ptmp)
            {
              *ptmp = '\0';
              unescape_value(pvalue);
              if (ppname)
                {
                  *ppname = pname;
                }
              if (ppvalue)
                {
                  *ppvalue = pvalue;
                }
              return 0;
            }
        }
    }
  return 1;
}

/****************************************************************************
 * Function: open_stringfile
 ****************************************************************************/
 
FILE *open_stringfile(void)
{
  FILE *stream = fopen(g_stringfile, "r");
  if (!stream)
    {
      fprintf(stderr, "Failed to open %s for reading: %s\n", g_stringfile, strerror(errno));
    }
  return stream;
}

/****************************************************************************
 * Function: open_outfile
 ****************************************************************************/
 
FILE *open_outfile(const char *filename)
{
  FILE *stream = fopen(filename, "w");
  if (!stream)
    {
      fprintf(stderr, "Failed to open %s for writing: %s\n", filename, strerror(errno));
    }
  return stream;
}

/****************************************************************************
 * Function: generate_sourcefile_list
 ****************************************************************************/
 
int generate_sourcefile_list(void)
{
  int ret = 1;
  FILE *stream;
  
  if (( stream = open_stringfile()))
    {
      ret = 0;
      while (fgets(g_line, 1024, stream) && !ret)
        {
          const char *pname;
          ret = parse_stringfile_line(&pname, NULL);
          if (!ret)
            {
              printf("%s.c ", pname);
            }
        }
      fclose(stream);
    }
  return ret;
}

/****************************************************************************
 * Function: generate_sourcefiles
 ****************************************************************************/
 
int generate_sourcefiles(void)
{
  FILE *instream;
  FILE *hstream;
  FILE *cstream;
  const char *pname;
  const char *pvalue;
  char buffer[512];
  int len;
  int ndx;
  int ret = 1;
  
  if (( instream = open_stringfile()))
    {
      snprintf(buffer, 512, "%s.h", g_stringfile);
      hstream = open_outfile(buffer);
      if (hstream)
        {
          fprintf(hstream, "#ifndef __NETUTIL_STRINGS\n#define __NETUTIL_STRINGS\n\n");
          
          ret = 0;
          while (fgets(g_line, 1024, instream) && !ret)
            {
              ret = parse_stringfile_line(&pname, &pvalue);
              if (!ret)
                {
                  snprintf(buffer, 512, "%s.c", pname);
                  cstream = open_outfile(buffer);
                  if (cstream)
                    {
                      len = strlen(pvalue);
                      fprintf(cstream, "const char %s[%d] = {", pname, len);
                      for (ndx = 0; ndx < len; ndx++)
                        {
                          if (ndx > 0)
                            {
                              fprintf(cstream, ", ");
                            }
                          fprintf(cstream, "0x%02x", pvalue[ndx]);
                        }
                      fprintf(cstream, "}\n");
                      fclose(cstream);
                    }
                  fprintf(hstream, "extern const char %s[%d];\n", pname, len);
                }
            }
            fprintf(hstream, "\n#endif /* __NETUTIL_STRINGS */\n");
            fclose(hstream);
        }
      fclose(instream);
    }
  return ret;
}

/****************************************************************************
 * Function: show_usage
 ****************************************************************************/

static void show_usage( void )
{
   fprintf(stderr, "USAGE: %s [OPTIONS] <string-file>\n\n", g_progname );
   fprintf(stderr, "Where [OPTIONS] include:\n");
   fprintf(stderr, "\t-s: Output string source file list on stdout");
   exit(1);
}

/****************************************************************************
 * Function: show_usage
 ****************************************************************************/

static void parse_commandline( int argc, char **argv )
{
  int option;
  g_progname = argv[0];
  while ((option =  getopt(argc, argv, ":s")) >= 0)
    {
      switch (option)
        {
          case 's': 
            g_action = SRCLIST;
            break;
            
          case ':':
            fprintf(stderr, "Missing command argument\n");
            show_usage();
            break;
            
          case '?':
            option = optopt;
          default:
            fprintf(stderr, "Unrecognized option: %c\n", option);
            show_usage();
            break;
        }
    }
    
  if (optind < argc)
    {
      g_stringfile = argv[optind];
      optind++;
    }
  else
    {
      fprintf(stderr, "Missing <string-file> path\n");
      show_usage();
    }
    
  if (optind < argc)
    {
      fprintf(stderr, "Garbage on command line after <string-file>\n");
      show_usage();
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: main
 ****************************************************************************/

int main(int argc, char **argv, char **envp)
{
  int ret = 0;
  parse_commandline(argc, argv);
  switch (g_action)
    {
        case GENSRC:
          ret = generate_sourcefiles();
          break;
        case SRCLIST:
          ret = generate_sourcefile_list();
          break;
    }
  return ret;
}

