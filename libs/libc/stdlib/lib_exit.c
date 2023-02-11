/****************************************************************************
 * libs/libc/stdlib/lib_exit.c
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

#include <nuttx/atexit.h>
#include <nuttx/compiler.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern FAR void *__dso_handle weak_data;
FAR void *__dso_handle = &__dso_handle;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: exit
 *
 * Description:
 *    The exit() function causes normal process termination and the
 *    least significant byte of status (i.e., status & 0xFF) is
 *    returned to the parent (see wait(2)).
 *
 *    All functions registered with atexit(3) and on_exit(3) are
 *    called, in the reverse order of their registration.  (It is
 *    possible for one of these functions to use atexit(3) or
 *    on_exit(3) to register an additional function to be executed
 *    during exit processing; the new registration is added to the
 *    front of the list of functions that remain to be called.)  If one
 *    of these functions does not return (e.g., it calls _exit(2), or
 *    kills itself with a signal), then none of the remaining functions
 *    is called, and further exit processing (in particular, flushing
 *    of stdio(3) streams) is abandoned.  If a function has been
 *    registered multiple times using atexit(3) or on_exit(3), then it
 *    is called as many times as it was registered.
 *
 *    All open stdio(3) streams are flushed and closed.  Files created
 *    by tmpfile(3) are removed.
 *
 *    The C standard specifies two constants, EXIT_SUCCESS and
 *    EXIT_FAILURE, that may be passed to exit() to indicate successful
 *    or unsuccessful termination, respectively.
 *
 * Input Parameters:
 *   status - Exit status code
 *
 * Returned Value:
 *   Does not return.
 *
 ****************************************************************************/

void exit(int status)
{
  /* Run the registered exit functions */

  atexit_call_exitfuncs(status, false);

#ifdef CONFIG_FILE_STREAM
  /* Flush all streams */

  fflush(NULL);
#endif

  /* Then perform the exit */

  _exit(status);
}

/****************************************************************************
 * Name: quick_exit
 *
 * Description:
 *    The quick_exit() function exits the program quickly calling any cleanup
 *    functions registered with at_quick_exit(3) but not any C++ destructors
 *    or cleanup code registered with atexit(3).  The stdio(3) file buffers
 *    are not flushed.
 *
 * Input Parameters:
 *   status - Exit status code
 *
 * Returned Value:
 *   Does not return.
 *
 ****************************************************************************/

void quick_exit(int status)
{
  /* Run the registered exit functions */

  atexit_call_exitfuncs(status, true);

  /* Then perform the exit */

  _exit(status);
}

/****************************************************************************
 * Name: _Exit
 *
 * Description:
 *    The _Exit() functions shall not call functions registered with atexit()
 *    nor any registered signal handlers. Open streams shall not be flushed.
 *    Whether open streams are closed (without flushing) is implementation
 *    defined.
 *
 * Input Parameters:
 *   status - Exit status code
 *
 * Returned Value:
 *   Does not return.
 *
 ****************************************************************************/

void _Exit(int status)
{
  _exit(status);
}
