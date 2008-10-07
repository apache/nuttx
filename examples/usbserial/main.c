/****************************************************************************
 * examples/usbserial/main.c
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <errno.h>
#include <nuttx/usbdev.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef CONFIG_DEBUG
#    define message(...) lib_lowprintf(__VA_ARGS__)
#  else
#    define message(...) printf(__VA_ARGS__)
#  endif
#else
#  ifdef CONFIG_DEBUG
#    define message lib_lowprintf
#  else
#    define message printf
#  endif
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * user_initialize
 ****************************************************************************/

#ifndef CONFIG_HAVE_WEAKFUNCTIONS
void user_initialize(void)
{
  /* Stub that must be provided only if the toolchain does not support weak
   * functions.
   */
}
#endif

/****************************************************************************
 * user_start
 ****************************************************************************/

int user_start(int argc, char *argv[])
{
  FILE *stream;
  int ret;

  /* Initialize the USB serial driver */

  message("user_start: Registering USB serial driver\n");
  ret = usbdev_serialinitialize(0);
  if (ret < 0)
    {
      message("user_start: ERROR: Failed to create the USB serial device: %d\n", -ret);
      return 1;
    }
  message("user_start: Successfully registered the serial driver\n");

  /* Open the USB serial device for output */

  do
    {
      message("user_start: Opening USB serial driver\n");
      stream = fopen("/dev/ttyUSB0", "w");
      if (!stream)
        {
          int errcode = errno;
          message("user_start: ERROR: Failed to open /dev/ttyUSB0: %d\n", errcode);

          /* ENOTCONN means that the USB device is not yet connected */

          if (errcode = ENOTCONN)
            {
              message("user_start:        Not connected. Wait and try again.\n");
              sleep(5);
            }
          else
            {
              /* Give up on other errors */

              message("user_start:        Aborting\n");
              return 2;
            }
        }
    }
  while (!stream);
  message("user_start: Successfully opened the serial driver\n");

  /* Then say hello -- forever */

  for (;;)
    {
      message("user_start: Saying hello\n");
      ret = fprintf(stream, "Hello, World!!\n");
      if (ret < 0)
        {
          message("user_start: ERROR: fprintf failed: %d\n", errno);
          fclose(stream);
          return 3;
        }

      ret = fflush(stream);
      if (ret < 0)
        {
          message("user_start: ERROR: fflush failed: %d\n", errno);
          fclose(stream);
          return 4;
        }

      message("user_start: Waiting\n");
      sleep(5);
    }

  /* Won't get here, but if we did this what we would have to do */

  fclose(stream);
  return 0;
}

