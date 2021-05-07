/****************************************************************************
 * tools/mksymtab.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include "csvparser.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAX_HEADER_FILES 500
#define SYMTAB_NAME      "g_symtab"
#define NSYMBOLS_NAME    "g_nsymbols"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char *g_hdrfiles[MAX_HEADER_FILES];
static int nhdrfiles;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void show_usage(const char *progname)
{
  fprintf(stderr,
    "USAGE:\n");
  fprintf(stderr,
    "%s [-d] <cvs-file> <symtab-file> [<symtab-name> [<nsymbols-name>]]\n\n",
    progname);
  fprintf(stderr,
    "Where:\n\n");
  fprintf(stderr,
    " <cvs-file>      : The path to the input CSV file (required)\n");
  fprintf(stderr,
    " <symtab-file>   : The path to the output symbol table file\n");
  fprintf(stderr,
    "(required)\n");
  fprintf(stderr,
    " <symtab-name>   : Optional name for the symbol table variable\n");
  fprintf(stderr,
    "                   Default: \"%s\"\n", SYMTAB_NAME);
  fprintf(stderr,
    " <nsymbols-name> : Optional name for the symbol table variable\n");
  fprintf(stderr,
    "                   Default: \"%s\"\n", NSYMBOLS_NAME);
  fprintf(stderr,
    "  -d              : Enable debug output\n");
  exit(EXIT_FAILURE);
}

static bool check_hdrfile(const char *hdrfile)
{
  int i;

  for (i = 0; i < nhdrfiles; i++)
    {
      if (strcmp(g_hdrfiles[i], hdrfile) == 0)
        {
          return true;
        }
    }

  return false;
}

static void add_hdrfile(const char *hdrfile)
{
  if (hdrfile && strlen(hdrfile) > 0)
    {
      if (!check_hdrfile(hdrfile))
        {
          if (nhdrfiles > MAX_HEADER_FILES)
            {
              fprintf(stderr,
                "ERROR:  Too man header files. Increase MAX_HEADER_FILES\n");
              exit(EXIT_FAILURE);
            }

          g_hdrfiles[nhdrfiles] = strdup(hdrfile);
          nhdrfiles++;
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv, char **envp)
{
  char *csvpath;
  char *sympath;
  char *symtab;
  char *nsymbols;
  char *nextterm;
  char *finalterm;
  char *ptr;
  bool cond;
  FILE *instream;
  FILE *outstream;
  int ch;
  int i;

  /* Parse command line options */

  symtab   = SYMTAB_NAME;
  nsymbols = NSYMBOLS_NAME;
  g_debug  = false;

  while ((ch = getopt(argc, argv, ":d")) > 0)
    {
      switch (ch)
        {
          case 'd' :
            g_debug = true;
            break;

          case '?' :
            fprintf(stderr, "Unrecognized option: %c\n", optopt);
            show_usage(argv[0]);

          case ':' :
            fprintf(stderr, "Missing option argument, option: %c\n", optopt);
            show_usage(argv[0]);

          default:
            fprintf(stderr, "Unexpected option: %c\n", ch);
            show_usage(argv[0]);
        }
    }

  if (optind >= argc)
    {
       fprintf(stderr, "Missing <cvs-file> and <symtab-file>\n");
       show_usage(argv[0]);
    }

  csvpath = argv[optind];
  optind++;

  if (optind >= argc)
    {
       fprintf(stderr, "Missing <symtab-file>\n");
       show_usage(argv[0]);
    }

  sympath = argv[optind];
  optind++;

  if (optind < argc)
    {
       symtab = argv[optind];
       optind++;
    }

  if (optind < argc)
    {
       nsymbols = argv[optind];
       optind++;
    }

  if (optind < argc)
    {
       fprintf(stderr, "Unexpected garbage at the end of the line\n");
       show_usage(argv[0]);
    }

  /* Open the CSV file for reading */

  instream = fopen(csvpath, "r");
  if (!instream)
    {
      fprintf(stderr, "open %s failed: %s\n", csvpath, strerror(errno));
      exit(EXIT_FAILURE);
    }

  /* Open the Symbol table file for writing */

  outstream = fopen(sympath, "w");
  if (!outstream)
    {
      fprintf(stderr, "open %s failed: %s\n", csvpath, strerror(errno));
      exit(EXIT_FAILURE);
    }

  /* Get all of the header files that we need to include */

  while ((ptr = read_line(instream)) != NULL)
    {
      /* Parse the line from the CVS file */

      int nargs = parse_csvline(ptr);
      if (nargs < PARM1_INDEX)
        {
          fprintf(stderr, "Only %d arguments found: %s\n", nargs, g_line);
          exit(EXIT_FAILURE);
        }

      /* Add the header file to the list of header files we need to include */

      add_hdrfile(g_parm[HEADER_INDEX]);
    }

  /* Back to the beginning */

  rewind(instream);

  /* Output up-front file boilerplate */

  fprintf(outstream,
     "/* %s: Auto-generated symbol table.  Do not edit */\n\n", sympath);
  fprintf(outstream, "#include <nuttx/config.h>\n");
  fprintf(outstream, "#include <nuttx/compiler.h>\n");
  fprintf(outstream, "#include <nuttx/symtab.h>\n\n");

  /* Output all of the require header files */

  for (i = 0; i < nhdrfiles; i++)
    {
      fprintf(outstream, "#include <%s>\n", g_hdrfiles[i]);
    }

  /* Now the symbol table itself */

  fprintf(outstream, "\nconst struct symtab_s %s[] =\n", symtab);
  fprintf(outstream, "{\n");

  /* Parse each line in the CVS file */

  nextterm  = "";
  finalterm = "";

  while ((ptr = read_line(instream)) != NULL)
    {
      /* Parse the line from the CVS file */

      int nargs = parse_csvline(ptr);
      if (nargs < PARM1_INDEX)
        {
          fprintf(stderr, "Only %d arguments found: %s\n", nargs, g_line);
          exit(EXIT_FAILURE);
        }

      /* Output any conditional compilation */

      cond = (g_parm[COND_INDEX] && strlen(g_parm[COND_INDEX]) > 0);
      if (cond)
        {
          fprintf(outstream, "%s#if %s\n", nextterm, g_parm[COND_INDEX]);
          nextterm  = "";
        }

      /* Output the symbol table entry */

      fprintf(outstream, "%s  { \"%s\", (FAR const void *)%s }",
              nextterm, g_parm[NAME_INDEX], g_parm[NAME_INDEX]);

      if (cond)
        {
          nextterm  = ",\n#endif\n";
          finalterm = "\n#endif\n";
        }
      else
        {
          nextterm  = ",\n";
          finalterm = "\n";
        }
    }

  fprintf(outstream, "%s};\n\n", finalterm);
  fprintf(outstream,
    "#define NSYMBOLS (sizeof(%s) / sizeof (struct symtab_s))\n", symtab);
  fprintf(outstream, "int %s = NSYMBOLS;\n", nsymbols);

  /* Close the CSV and symbol table files and exit */

  fclose(instream);
  fclose(outstream);
  return EXIT_SUCCESS;
}
