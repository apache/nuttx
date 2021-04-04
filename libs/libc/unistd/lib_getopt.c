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

#include "unistd.h"

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
 *      (You can also set optind to -1 to force a reset).
 *   4. Standard getopt() permutes the contents of argv as it scans, so that
 *      eventually all the nonoptions are at the end.  This implementation
 *      does not do this.
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
  return getopt_common(argc, argv, optstring, NULL, NULL, GETOPT_MODE);
}
