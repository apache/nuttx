/****************************************************************************
 * libs/libc/unistd/lib_getopt.c
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

#include <nuttx/config.h>

#include <stdbool.h>
#include <getopt.h>
#include <string.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

FAR char *optarg; /* Optional argument following option */
int opterr = 0;   /* Print error message */
int optind = 1;   /* Index into argv */
int optopt = '?'; /* unrecognized option character */

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR char         *g_optptr       = NULL;
static FAR char * const *g_argv         = NULL;
static int               g_argc         = 0;
static bool              g_binitialized = false;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getopt
 *
 * Description:
 *   getopt() parses command-line arguments.  Its arguments argc and argv
 *   are the argument count and array as passed to the main() function on
 *   program invocation.  An element of argv that starts with '-' is an
 *   option element. The characters of this element (aside from the initial
 *   '-') are option characters. If getopt() is called repeatedly, it
 *   returns successively each of the option characters from each of the
 *   option elements.
 *
 *   If getopt() finds another option character, it returns that character,
 *   updating the external variable optind and a static variable nextchar so
 *   that the next call to getopt() can resume the scan with the following
 *   option character or argv-element.
 *
 *   If there are no more option characters, getopt() returns -1. Then optind
 *   is the index in argv of the first argv-element that is not an option.
 *
 *   The 'optstring' argument is a string containing the legitimate option
 *   characters. If such a character is followed by a colon, this indicates
 *   that the option requires an argument.  If an argument is required for an
 *   option so getopt() places a pointer to the following text in the same
 *   argv-element, or the text of the following argv-element, in optarg.
 *
 *   NOTES:
 *   1. opterr is not supported and this implementation of getopt() never
 *      printfs error messages.
 *   2. getopt is NOT threadsafe!
 *   3. This version of getopt() does not reset global variables until
 *      -1 is returned.  As a result, your command line parsing loops
 *      must call getopt() repeatedly and continue to parse if other
 *      errors are returned ('?' or ':') until getopt() finally returns -1.
 *     (You can also set optind to -1 to force a reset).
 *
 * Returned Value:
 *   If an option was successfully found, then getopt() returns the option
 *   character. If all command-line options have been parsed, then getopt()
 *   returns -1.  If getopt() encounters an option character that was not
 *   in optstring, then '?' is returned. If getopt() encounters an option
 *   with a missing argument, then the return value depends on the first
 *   character in optstring: if it is ':', then ':' is returned; otherwise
 *   '?' is returned.
 *
 ****************************************************************************/

int getopt(int argc, FAR char * const argv[], FAR const char *optstring)
{
  /* Were new argc or argv passed in?  This detects misuse of getopt() by
   * applications that break out of the getopt() loop before getop() returns
   * -1.
   */

  if (argc != g_argc || argv != g_argv)
    {
      /* Yes, clear the internal state */

      g_binitialized = false;
      g_argc         = argc;
      g_argv         = argv;
    }

  /* Verify input parameters. */

  if (argv != NULL && optstring != NULL)
    {
      FAR char *optchar;
      int noarg_ret = '?';

      /* The initial value of optind is 1.  If getopt() is called again in
       * the program, optind must be reset to some value <= 1.
       */

      if (optind < 1 || !g_binitialized)
        {
          optarg         = NULL;
          optind         = 1;     /* Skip over the program name */
          optopt         = '?';
          g_optptr       = NULL;  /* Start at the beginning of the first argument */
          g_binitialized = true;  /* Now we are initialized */
        }

      /* If the first character of opstring s ':', then ':' is in the event
       * of a missing argument. Otherwise '?' is returned.
       */

      if (*optstring == ':')
        {
           noarg_ret = ':';
           optstring++;
        }

      /* Are we resuming in the middle, or at the end of a string of
       * arguments? g_optptr == NULL means that we are started at the
       * beginning of argv[optind]; *g_optptr == \0 means that we are
       * starting at the beginning of optind+1
       */

      while (!g_optptr || !*g_optptr)
        {
          /* We need to start at the beginning of the next argv. Check if we
           * need to increment optind
           */

          if (g_optptr)
            {
              /* Yes.. Increment it and check for the case where where we
               * have processed everything in the argv[] array.
               */

              optind++;
            }

          /* Check for the end of the argument list */

          g_optptr = argv[optind];
          if (!g_optptr)
            {
              /* There are no more arguments, we are finished */

              g_binitialized = false;
              return ERROR;
            }

          /* We are starting at the beginning of argv[optind].  In this case,
           * the first character must be '-'
           */

          if (*g_optptr != '-')
            {
              /* The argument does not start with '-', we are finished */

              g_binitialized = false;
              return ERROR;
            }

          /* Skip over the '-' */

          g_optptr++;
        }

      /* Special case handling of "-" and "-:" */

      if (!*g_optptr)
        {
          optopt = '\0'; /* We'll fix up g_optptr the next time we are called */
          return '?';
        }

      /* Handle the case of "-:" */

      if (*g_optptr == ':')
        {
          optopt = ':';
          g_optptr++;
          return '?';
        }

      /* g_optptr now points at the next option and it is not something
       * crazy. check if the option is in the list of valid options.
       */

      optchar = strchr(optstring, *g_optptr);
      if (!optchar)
        {
          /* No this character is not in the list of valid options */

          optopt = *g_optptr;
          g_optptr++;
          return '?';
        }

      /* Yes, the character is in the list of valid options.  Does it have an
       * required argument?
       */

      if (optchar[1] != ':')
        {
          /* No, no arguments. Just return the character that we found */

          g_optptr++;
          return *optchar;
        }

      /* Yes, it has a required argument.  Is the required argument
       * immediately after the command in this same argument?
       */

      if (g_optptr[1] != '\0')
        {
          /* Yes, return a pointer into the current argument */

          optarg = &g_optptr[1];
          optind++;
          g_optptr = NULL;
          return *optchar;
        }

      /* No.. is the optional argument the next argument in argv[] ? */

      if (argv[optind + 1] && *argv[optind + 1] != '-')
        {
          /* Yes.. return that */

          optarg = argv[optind + 1];
          optind += 2;
          g_optptr = NULL;
          return *optchar;
        }

      /* No argument was supplied */

      g_optptr = NULL;
      optarg   = NULL;
      optopt   = *optchar;
      optind++;
      return noarg_ret;
    }

  /* Restore the initial, uninitialized state */

  g_binitialized = false;
  return ERROR;
}

int getopt_long(int argc, FAR char *const argv[],
                FAR const char *shortopts,
                FAR const struct option *longopts,
                FAR int *longind)
{
  return getopt(argc, argv, shortopts);
}
