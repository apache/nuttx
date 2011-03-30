/************************************************************
 * lib/unistd/lib_sem.c
 *
 *   Copyright (C) 2007, 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 ************************************************************/

/************************************************************
 * Included Files
 ************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_STDIO_BUFFERED_IO

#include <sys/types.h>
#include <unistd.h>
#include <semaphore.h>
#include <errno.h>
#include <assert.h>

#include "lib_internal.h"

/************************************************************
 * Pre-processor Definitions
 ************************************************************/

/************************************************************
 * Private Data
 ************************************************************/

/************************************************************
 * Global Functions
 ************************************************************/

/************************************************************
 * lib_sem_initialize
 ************************************************************/

void lib_sem_initialize(FILE *stream)
{
  /* Initialize the LIB semaphore to one (to support one-at-
   * a-time access to private data sets.
   */

  (void)sem_init(&stream->sem, 0, 1);

  stream->holder = -1;
  stream->counts = 0;
}

/************************************************************
 * lib_take_semaphore
 ************************************************************/

void lib_take_semaphore(FILE *stream)
{
  pid_t my_pid = getpid();

  /* Do I already have the semaphore? */

  if (stream->holder == my_pid)
    {
      /* Yes, just increment the number of references that I have */

      stream->counts++;
    }
  else
    {
      /* Take the semaphore (perhaps waiting) */

      while (sem_wait(&stream->sem) != 0)
	{
	  /* The only case that an error should occr here is if
	   * the wait was awakened by a signal.
	   */

	  ASSERT(*get_errno_ptr() == EINTR);
	}

      /* We have it.  Stake the claim and return */

      stream->holder = my_pid;
      stream->counts = 1;
    }
}

/************************************************************
 * lib_give_semaphore
 ************************************************************/

void lib_give_semaphore(FILE *stream)
{
  pid_t my_pid = getpid();

  /* I better be holding at least one count on the semaphore */

  ASSERT(stream->holder == my_pid);

  /* Do I hold multiple references to the semphore */

  if (stream->counts > 1)
    {
      /* Yes, just release one count and return */

      stream->counts--;
    }
  else
    {
      /* Nope, this is the last reference I have */

      stream->holder = -1;
      stream->counts = 0;
      ASSERT(sem_post(&stream->sem) == 0);
    }
}
#endif /* CONFIG_STDIO_BUFFERED_IO */
