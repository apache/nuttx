/****************************************************************************
 * libs/libc/unistd/lib_getopt_long.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <assert.h>
#include <stdbool.h>
#include <string.h>

#include "unistd.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: compare_long_option
 *
 * Description:
 *   Compare a command argument with a long option, handling the cases:
 *
 *   --option:          Any argument is in the next argv entry
 *   --option=argument: The argument is in the same argv entry
 *
 ****************************************************************************/

static int compare_long_option(FAR const char *cmdarg,
                               FAR const char *optname,
                               FAR const char **argument)
{
  int result;

  *argument = NULL;

  for (; ; )
    {
      int rawchar = *cmdarg++;
      int optchar = *optname++;
      int cmdchar;

      /* The command line option may terminate with either '\0' or '='. */

      cmdchar = rawchar;
      if (cmdchar == '=')
        {
          cmdchar = '\0';
        }

      /* Perform the comparison */

      result = cmdchar - optchar;
      if (result != 0 || cmdchar == '\0')
        {
          /* If the '=' is the real terminator, then return the argument
           * that follows the '='
           */

          if (rawchar == '=')
            {
              *argument = cmdarg;
            }

          break;
        }
    }

  return result;
}

/****************************************************************************
 * Name: getopt_long_option
 *
 * Description:
 *   Handle one long option
 *
 ****************************************************************************/

int getopt_long_option(FAR struct getopt_s *go,
                       FAR char * const argv[],
                       FAR const struct option *longopts,
                       FAR int *longindex)
{
  int ndx;
  int ret;

  /* The option list may not be NULL in this context */

  if (longopts == NULL)
    {
      goto errout;
    }

  /* Search the list of long options for a matching name.
   * The last element of the option array must be filled with zeroes.
   */

  for (ndx = 0; longopts[ndx].name != NULL; ndx++)
    {
      FAR char *terminator = NULL;

      if (compare_long_option(go->go_optptr, longopts[ndx].name,
                              (FAR const char **)&terminator) == 0)
        {
          /* Found the option with the matching name.  Does it have an
           * argument provided in the same argv entry like
           * --option=argument?
           */

          if (terminator != NULL)
            {
              /* Skip over the option + argument */

              go->go_optptr = NULL;
              go->go_optind++;

              switch (longopts[ndx].has_arg)
                {
                  case no_argument:

                    /* But no argument is expected! */

                    go->go_optarg = NULL;
                    return '?';

                  case optional_argument:
                  case required_argument:

                    /* Return the required argument */

                    go->go_optarg  = terminator;
                    break;

                  default:
                    goto errout;
                }
            }
          else
            {
              FAR char *next;

              /* Does the option have a required argument in the next argv
               * entry?  An optional argument?
               */

              switch (longopts[ndx].has_arg)
                {
                  case no_argument:
                  /* No, no arguments. Just return the argument that we
                   * found.
                   */

                    go->go_optptr = NULL;
                    go->go_optind++;
                    break;

                  case optional_argument:
                  /* Check if there is a following argument and if that
                   * following argument is another option.
                   */

                    next = argv[go->go_optind + 1];
                    if (next == NULL || next[0] == '-')
                      {
                        go->go_optptr = NULL;
                        go->go_optarg = NULL;
                        go->go_optind++;
                        break;
                      }

                    /* Fall through and treat as a required option */

                  case required_argument:

                    /* Verify that the required argument is present */

                    next = argv[go->go_optind + 1];
                    if (next == NULL || next[0] == '-')
                      {
                        go->go_optptr = NULL;
                        go->go_optarg = NULL;
                        go->go_optind++;
                        return '?';
                      }

                    /* Return the required argument */

                    go->go_optptr  = NULL;
                    go->go_optarg  = next;
                    go->go_optind += 2;
                    break;

                  default:
                    goto errout;
                }
            }

          /* Setup return value.
           *
           * 'val' is the value to return on success, or to load into the
           * variable pointed to by flag.
           *
           * 'flag' specifies how results are returned for a long option. If
           * flag is NULL, then getopt_long() returns val.  Otherwise,
           * getopt_long() returns 0, and flag points to a variable which is
           * set to val if the option is found, but left unchanged if the
           * option is not found.
           */

          if (longopts[ndx].flag != NULL)
            {
              *(longopts[ndx].flag) = longopts[ndx].val;
              ret = OK;
            }
          else
            {
              ret = longopts[ndx].val;
            }

          /* If longindex is not NULL, it points to a variable which is
           * set to the index of the long option relative to longopts.
           */

          if (longindex != NULL)
            {
              *longindex = ndx;
            }

          return ret;
        }
    }

  /* This option is not in the list of valid options */

  go->go_optopt = *go->go_optptr;
  return '?';

errout:

  /* Restore the initial, uninitialized state */

  go->go_binitialized = false;
  return ERROR;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getopt_long
 *
 * Description:
 *   The getopt_long() function works like getopt() except that it also
 *   accepts long options, started with two dashes. (If the program accepts
 *   only long options, then optstring should be specified as an empty
 *   string (""), not NULL.) Long option names may be abbreviated if the
 *   abbreviation is unique or is an exact match for some defined option
 *
 ****************************************************************************/

int getopt_long(int argc, FAR char * const argv[],
                FAR const char *optstring,
                FAR const struct option *longopts,
                FAR int *longindex)
{
  return getopt_common(argc, argv, optstring, longopts, longindex,
                       GETOPT_LONG_MODE);
}

/****************************************************************************
 * Name: getopt_long_only
 *
 * Description:
 *   getopt_long_only() is like getopt_long(), but '-' as well as "--" can
 *   indicate a long option. If an option that starts with '-' (not "--")
 *   doesn't match a long option, but does match a short option, it is
 *   parsed as a short option instead.
 *
 ****************************************************************************/

int getopt_long_only(int argc, FAR char * const argv[],
                     FAR const char *optstring,
                     FAR const struct option *longopts,
                     FAR int *longindex)
{
  return getopt_common(argc, argv, optstring, longopts, longindex,
                       GETOPT_LONG_ONLY_MODE);
}
