/****************************************************************************
 * libs/libc/stdio/lib_perror.c
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

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* POSIX requires that perror provide its output on stderr.  This option may
 * be defined, however, to provide perror output that is serialized with
 * other stdout messages.
 */

#ifdef CONFIG_LIBC_PERROR_STDOUT
#  define PERROR_STREAM stdout
#  define PERROR_FILENO STDOUT_FILENO
#else
#  define PERROR_STREAM stderr
#  define PERROR_FILENO STDERR_FILENO
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: perror
 ****************************************************************************/

void perror(FAR const char *s)
{
  /* If strerror() is not enabled, then just print the error number */

#ifdef CONFIG_LIBC_STRERROR
# ifdef CONFIG_FILE_STREAM
  fprintf(PERROR_STREAM, "%s: %s\n", s, strerror(get_errno()));
# else
  dprintf(PERROR_FILENO, "%s: %s\n", s, strerror(get_errno()));
# endif
#else
# ifdef CONFIG_FILE_STREAM
  fprintf(PERROR_STREAM, "%s: Error %d\n", s, get_errno());
# else
  dprintf(PERROR_FILENO, "%s: Error %d\n", s, get_errno());
# endif
#endif
}
