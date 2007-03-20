/************************************************************
 * lib_libfwrite.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
 * Compilation Switches
 ************************************************************/

/************************************************************
 * Included Files
 ************************************************************/

#include <nuttx/config.h>  /* for CONFIG_STDIO_BUFFER_SIZE */
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include "lib_internal.h"

/************************************************************
 * Definitions
 ************************************************************/

/************************************************************
 * Private Type Declarations
 ************************************************************/

/************************************************************
 * Private Function Prototypes
 ************************************************************/

/************************************************************
 * Global Constant Data
 ************************************************************/

/************************************************************
 * Global Variables
 ************************************************************/

/************************************************************
 * Private Constant Data
 ************************************************************/

/************************************************************
 * Private Variables
 ************************************************************/

/************************************************************
 * Global Functions
 ************************************************************/

/************************************************************
 * lib_fwrite
 ************************************************************/

#if CONFIG_NFILE_STREAMS > 0

ssize_t lib_fwrite(const void *ptr, size_t count, FILE *stream)
#if CONFIG_STDIO_BUFFER_SIZE > 0
{
  const unsigned char *start = ptr;
  const unsigned char *src   = ptr;
  ssize_t bytes_written;
  unsigned char *dest;

  /* Make sure that writing to this stream is allowed */

  if ((!stream) || ((stream->fs_oflags & O_WROK) == 0))
    {
      *get_errno_ptr() = EBADF;
      bytes_written = -1;
    }
  else
    {
      /* Loop until all of the bytes have been buffered */

      lib_take_semaphore(stream);
      while (count > 0)
	{
	  /* Determine the number of bytes left in the buffer */

	  size_t gulp_size = stream->fs_bufend - stream->fs_bufpos;

	  /* Will the user data fit into the amount of buffer space
	   * that we have left?
	   */

	  if (gulp_size > count)
	    {
	      /* Yes, clip the gulp to the size of the user data */

	      gulp_size = count;
	    }

	  /* Adjust the number of bytes remaining to be transferred
	   * on the next pass through the loop (might be zero).
	   */

	  count -= gulp_size;

	  /* Transfer the data into the buffer */

	  for (dest = stream->fs_bufpos; gulp_size > 0; gulp_size--)
	    {
	      *dest++ = *src++;
	    }
	  stream->fs_bufpos = dest;

	  /* Is the buffer full? */

	  if (dest >= stream->fs_bufend)
	    {
	      /* flush the buffered data to the IO stream */

	      int bytes_buffered = fflush(stream);
	      if (bytes_buffered < 0)
		{
		  bytes_written = bytes_buffered;
		  goto err_out;
		}
	    }
	}
  
      bytes_written = src - start;

    err_out:
      lib_give_semaphore(stream);
    }
  return bytes_written;
}
#else
{
  return write(stream->fs_filedes, ptr, count);
}
#endif /* CONFIG_STDIO_BUFFER_SIZE */

#endif /* CONFIG_NFILE_STREAMS */
