/****************************************************************************
 * include/getopt.h
 *
 *   Copyright (C) 2018 Pinecone Inc. All rights reserved.
 *   Author:  Xiang Xiao <xiaoxiang@pinecone.net>
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
 * AUTHOR: Gregory Pietsch
 * CREATED Thu Jan 09 22:37:00 1997
 *
 * COPYRIGHT NOTICE AND DISCLAIMER:
 *
 * Copyright (C) 1997 Gregory Pietsch
 *
 * This file and the accompanying getopt.c implementation file are hereby
 * placed in the public domain without restrictions.  Just give the author
 * credit, don't claim you wrote it or prevent anyone else from using it.
 *
 * Gregory Pietsch's current e-mail address:
 * gpietsch@comcast.net
 ***************************************************************************/

/* This is a glibc-extension header file. */

#ifndef __INCLUDE_GETOPT_H
#define __INCLUDE_GETOPT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Names for the values of the `has_arg' field of `struct option'. */

#define no_argument             0
#define required_argument       1
#define optional_argument       2

#define NO_ARG                  no_argument
#define REQUIRED_ARG            required_argument
#define OPTIONAL_ARG            optional_argument

/* The GETOPT_DATA_INITIALIZER macro is used to initialize a statically-
 * allocated variable of type struct getopt_data.
 */

#define GETOPT_DATA_INITIALIZER {0, 0, 1, 0, 0, 0, 0}

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* Used by getopt (obviously NOT thread safe!).  These variables cannot be
 * accessed directly by an external NXFLAT module.  In that case, accessor
 * functions must be used.
 */

#ifndef __NXFLAT__

/* For communication from `getopt' to the caller.
 * When `getopt' finds an option that takes an argument,
 * the argument value is returned here.
 * Also, when `ordering' is RETURN_IN_ORDER,
 * each non-option ARGV-element is returned here.
 */

EXTERN FAR char *optarg;

/* Index in ARGV of the next element to be scanned.
 * This is used for communication to and from the caller
 * and for communication between successive calls to `getopt'.
 *
 * On entry to `getopt', zero means this is the first call; initialize.
 *
 * When `getopt' returns -1, this is the index of the first of the
 * non-option elements that the caller should itself scan.
 *
 * Otherwise, `optind' communicates from one call to the next
 * how much of ARGV has been scanned so far.
 */

EXTERN int optind;

/* Callers store zero here to inhibit the error message `getopt' prints
 * for unrecognized options.
 */

EXTERN int opterr;

/* Set to an option character which was unrecognized. */

EXTERN int optopt;
#else
#  define optarg  (*(getoptargp()))
#  define optind  (*(getoptindp()))
#  define optopt  (*(getoptoptp()))
#endif

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* Describe the long-named options requested by the application.
 * The LONG_OPTIONS argument to getopt_long or getopt_long_only is a vector
 * of `struct option' terminated by an element containing a name which is
 * zero.
 *
 * The field `has_arg' is:
 * no_argument       (or 0) if the option does not take an argument,
 * required_argument (or 1) if the option requires an argument,
 * optional_argument (or 2) if the option takes an optional argument.
 *
 * If the field `flag' is not NULL, it points to a variable that is set
 * to the value given in the field `val' when the option is found, but
 * left unchanged if the option is not found.
 *
 * To have a long-named option do something other than set an `int' to
 * a compiled-in constant, such as set a value from `optarg', set the
 * option's `flag' field to zero and its `val' field to a nonzero
 * value (the equivalent single-letter option character, if there is
 * one).  For long options that have a zero `flag' field, `getopt'
 * returns the contents of the `val' field.
 */

struct option
{
  FAR const char *name; /* The name of the long option */
  int has_arg;          /* One of the above macros */
  FAR int *flag;        /* Determines if getopt_long() returns a
                         * value for a long option; if it is
                         * non-NULL, 0 is returned as a function
                         * value and the value of val is stored in
                         * the area pointed to by flag.  Otherwise,
                         * val is returned.
                         */
  int val;              /* Determines the value to return if flag is
                         * NULL.
                         */
};

/* The getopt_data structure is for reentrancy. Its members are similar to
 * the externally-defined variables.
 */

typedef struct getopt_data
{
  FAR char *optarg;
  int optind, opterr, optopt, optwhere;
  int permute_from, num_nonopts;
} getopt_data;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int getopt(int argc, FAR char *const argv[], FAR const char *optstring);

int getopt_long(int argc, FAR char *const argv[], FAR const char *shortopts,
                FAR const struct option *longopts, FAR int *longind);

int getopt_long_only(int argc, FAR char *const argv[], FAR const char *shortopts,
                     FAR const struct option *longopts, FAR int *longind);

int getopt_r(int argc, FAR char *const argv[], FAR const char *optstring,
             FAR struct getopt_data *data);

int getopt_long_r(int argc, FAR char *const argv[], FAR const char *shortopts,
                  FAR const struct option *longopts, FAR int *longind,
                  FAR struct getopt_data *data);

int getopt_long_only_r(int argc, FAR char *const argv[], FAR const char *shortopts,
                       FAR const struct option *longopts, FAR int *longind,
                       FAR struct getopt_data *data);

/* Accessor functions intended for use only by external NXFLAT
 * modules.  The global variables optarg, optind, and optopt cannot
 * be referenced directly from external modules.
 */

FAR char **getoptargp(void);  /* Optional argument following option */
FAR int   *getoptindp(void);  /* Index into argv */
FAR int   *getoptoptp(void);  /* Unrecognized option character */

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_GETOPT_H */
