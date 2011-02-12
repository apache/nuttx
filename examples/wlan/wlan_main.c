/****************************************************************************
 * examples/wlan/wlan_main.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <spudmonkey@racsa.co.cr>
 *            Rafael Noronha <rafael@pdsolucoes.com.br>
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
#include <sched.h>
#include <errno.h>

#include <nuttx/usb/usbhost.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

/* Sanity checking */

#ifndef CONFIG_USBHOST
#  error "CONFIG_USBHOST is not defined"
#endif

#ifdef CONFIG_USBHOST_BULK_DISABLE
#  error "Bulk endpoints are disabled (CONFIG_USBHOST_BULK_DISABLE)"
#endif

#ifndef CONFIG_NFILE_DESCRIPTORS
#  error "CONFIG_NFILE_DESCRIPTORS > 0 needed"
#endif

/* Provide some default values for other configuration settings */

#ifndef CONFIG_EXAMPLES_WLAN_DEFPRIO
#  define CONFIG_EXAMPLES_WLAN_DEFPRIO 50
#endif

#ifndef CONFIG_EXAMPLES_WLAN_STACKSIZE
#  define CONFIG_EXAMPLES_WLAN_STACKSIZE 1024
#endif

#ifndef CONFIG_EXAMPLES_WLAN_DEVNAME
#  define CONFIG_EXAMPLES_WLAN_DEVNAME "/dev/wlana"
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct usbhost_driver_s *g_drvr;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wlan_waiter
 *
 * Description:
 *   Wait for USB devices to be connected.
 *
 ****************************************************************************/

static int wlan_waiter(int argc, char *argv[])
{
  bool connected = false;
  int ret;

  printf("wlan_waiter: Running\n");
  for (;;)
    {
      /* Wait for the device to change state */

      ret = DRVR_WAIT(g_drvr, connected);
      DEBUGASSERT(ret == OK);

      connected = !connected;
      printf("wlan_waiter: %s\n", connected ? "connected" : "disconnected");

      /* Did we just become connected? */

      if (connected)
        {
          /* Yes.. enumerate the newly connected device */

          (void)DRVR_ENUMERATE(g_drvr);
        }
    }

  /* Keep the compiler from complaining */

  return 0;
}

/****************************************************************************
 * Name: user_initialize
 ****************************************************************************/

void user_initialize(void)
{
}

/****************************************************************************
 * Name: user_start
 ****************************************************************************/

int user_start(int argc, char *argv[])
{
  char buffer[256];
  pid_t pid;
  ssize_t nbytes;
  int fd;
  int ret;

  /* First, register all of the USB host Wireless LAN drivers */

  printf("user_start: Register drivers\n");
  ret = usbhost_wlaninit();
  if (ret != OK)
    {
      printf("user_start: Failed to register the WLAN driver\n");
    }

  /* Then get an instance of the USB host interface */

  printf("user_start: Initialize USB host WLAN driver\n");
  g_drvr = usbhost_initialize(0);
  if (g_drvr)
    {
      /* Start a thread to handle device connection. */

      printf("user_start: Start hidkbd_waiter\n");

#ifndef CONFIG_CUSTOM_STACK
      pid = task_create("usbhost", CONFIG_EXAMPLES_WLAN_DEFPRIO,
                        CONFIG_EXAMPLES_WLAN_STACKSIZE,
                        (main_t)wlan_waiter, (const char **)NULL);
#else
      pid = task_create("usbhost", CONFIG_EXAMPLES_WLAN_DEFPRIO,
                        (main_t)wlan_waiter, (const char **)NULL);
#endif

      /* Now just sleep.  Eventually logic here will open the WLAN device and
       * perform the device test.
       */

      for (;;)
        {
          /* Open the WLAN device.  Loop until the device is successfully
           * opened.
           */

          do
            {
              printf("Opening device %s\n", CONFIG_EXAMPLES_WLAN_DEVNAME);
              fd = open(CONFIG_EXAMPLES_WLAN_DEVNAME, O_RDONLY);
              if (fd < 0)
                {
                   printf("Failed: %d\n", errno);
                   fflush(stdout);
                   sleep(3);
                }
            }
          while (fd < 0);

          printf("Device %s opened\n", CONFIG_EXAMPLES_WLAN_DEVNAME);
          fflush(stdout);

          /* Loop until there is a read failure */

          do
            {
              /* Read a buffer of data */

              nbytes = read(fd, buffer, 256);
              if (nbytes > 0)
                {
                  /* On success, echo the buffer to stdout */

                  (void)write(1, buffer, nbytes);
                }
            }
          while (nbytes >= 0);

          printf("Closing device %s: %d\n", CONFIG_EXAMPLES_WLAN_DEVNAME, (int)nbytes);
          fflush(stdout);
          close(fd);
        }
    }
  return 0;
}
