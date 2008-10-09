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

#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

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

#define IOBUFFER_SIZE 256

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char g_shortmsg[] = "Hello, World!!\n";
static const char g_longmsg[] =
  "The Spanish Armada a Speech by Queen Elizabeth I of England\n"
  "Addressed to the English army at Tilbury Fort - 1588\n"
  "My loving people, we have been persuaded by some, that are careful of our "
  "safety, to take heed how we commit ourselves to armed multitudes, for fear "
  "of treachery; but I assure you, I do not desire to live to distrust my "
  "faithful and loving people.\n"
  "Let tyrants fear; I have always so behaved myself that, under God, I have "
  "placed my chiefest strength and safeguard in the loyal hearts and good will "
  "of my subjects. And therefore I am come amongst you at this time, not as for "
  "my recreation or sport, but being resolved, in the midst and heat of the "
  "battle, to live or die amongst you all; to lay down, for my God, and for "
  "my kingdom, and for my people, my honour and my blood, even the dust.\n"
  "I know I have but the body of a weak and feeble woman; but I have the heart "
  "of a king, and of a king of England, too; and think foul scorn that Parma "
  "or Spain, or any prince of Europe, should dare to invade the borders of my "
  "realms: to which, rather than any dishonour should grow by me, I myself will "
  "take up arms; I myself will be your general, judge, and rewarder of every "
  "one of your virtues in the field.\n"
  "I know already, by your forwardness, that you have deserved rewards and "
  "crowns; and we do assure you, on the word of a prince, they shall be duly "
  "paid you. In the mean my lieutenant general shall be in my stead, than whom "
  "never prince commanded a more noble and worthy subject; not doubting by "
  "your obedience to my general, by your concord in the camp, and by your "
  "valour in the field, we shall shortly have a famous victory over the enemies "
  "of my God, of my kingdom, and of my people.\n";
static char g_iobuffer[IOBUFFER_SIZE];

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
  int infd;
  int outfd;
  int count;
  ssize_t nbytes;
  int ret;
  int i;

  /* Initialize the USB serial driver */

  message("user_start: Registering USB serial driver\n");
  ret = usbdev_serialinitialize(0);
  if (ret < 0)
    {
      message("user_start: ERROR: Failed to create the USB serial device: %d\n", -ret);
      return 1;
    }
  message("user_start: Successfully registered the serial driver\n");

  /* Open the USB serial device for writing (blocking) */

  do
    {
      message("user_start: Opening USB serial driver\n");
      outfd = open("/dev/ttyUSB0", O_WRONLY);
      if (outfd < 0)
        {
          int errcode = errno;
          message("user_start: ERROR: Failed to open /dev/ttyUSB0 for writing: %d\n", errcode);

          /* ENOTCONN means that the USB device is not yet connected */

          if (errcode == ENOTCONN)
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
  while (outfd < 0);

  /* Open the USB serial device for reading (non-blocking) */

  infd = open("/dev/ttyUSB0", O_RDONLY|O_NONBLOCK);
  if (infd < 0)
    {
      message("user_start: ERROR: Failed to open /dev/ttyUSB0 for reading: %d\n", errno);
      close(outfd);
      return 3;
    }

  message("user_start: Successfully opened the serial driver\n");

  /* Then say hello -- forever */

  count = 0;
  for (;;)
    {
      if (count < 8)
        {
          message("user_start: Saying hello\n");
          nbytes = write(outfd, g_shortmsg, sizeof(g_shortmsg));
          count++;
        }
      else
        {
          message("user_start: Reciting QEI's speech of 1588\n");
          nbytes = write(outfd, g_longmsg, sizeof(g_longmsg));
          count = 0;
        }

      if (nbytes < 0)
        {
          message("user_start: ERROR: write failed: %d\n", errno);
          close(infd);
          close(outfd);
          return 4;
        }

      /* Poll for incoming messages */

      message("user_start: Waiting\n");
      for (i = 0; i < 5; i++)
        {
          nbytes = read(infd, g_iobuffer, IOBUFFER_SIZE);
          if (nbytes < 0)
            {
              int errorcode = errno;
              if (errorcode != EAGAIN)
                {
                  message("user_start: ERROR: read failed: %d\n", errno);
                  close(infd);
                  close(outfd);
                  return 6;
                }
            }
          else
            {
              message("user_start: Received %d bytes:\n", nbytes);
              if (nbytes > 0)
                {
                  message("user_start: ");
                  (void)fwrite(g_iobuffer, 1, nbytes, stdout);
                }
            }
          sleep(1);
        }
    }

  /* Won't get here, but if we did this what we would have to do */

  close(infd);
  close(outfd);
  return 0;
}

