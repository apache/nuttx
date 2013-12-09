/****************************************************************************
 * configs/sama5d3x-ek/src/sam_ov2640.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <stdlib.h>
#include <debug.h>

#include <nuttx/fb.h>
#include <nuttx/nx/nx.h>

#include "up_arch.h"

#include "sam_periphclks.h"
#include "sam_lcd.h"
#include "sam_pck.h"
#include "sama5d3x-ek.h"

#if defined(CONFIG_SAMA5_ISI) && defined(CONFIG_SAMA5_OV2640_DEMO)

/****************************************************************************
 * Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The connection handler */

static NXHANDLE g_hnx = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ov2640_lcd_initialize
 ****************************************************************************/

#ifndef CONFIG_NX_MULTIUSER
static inline int ov2640_lcd_initialize(void)
{
  FAR NX_DRIVERTYPE *dev;
  int ret;

  /* Initialize the frame buffer device */

  gvdbg("Initializing framebuffer\n");
  ret = up_fbinitialize();
  if (ret < 0)
    {
      gdbg("ERROR: up_fbinitialize failed: %d\n", -ret);
      return EXIT_FAILURE;
    }

  dev = up_fbgetvplane(0);
  if (!dev)
    {
      gdbg("ERROR: up_fbgetvplane failed\n");
      return EXIT_FAILURE;
    }

  /* Then open NX */

  gvdbg("Open NX\n");
  g_hnx = nx_open(dev);
  if (!g_hnx)
    {
      gdbg("ERROR: nx_open failed: %d\n", errno);
      return EXIT_FAILURE;
    }

  return EXIT_SUCCESS;
}
#endif

#ifdef CONFIG_NX_MULTIUSER
static inline int ov2640_lcd_initialize(void)
{
  struct sched_param param;
  pthread_t thread;
  pid_t servrid;
  int ret;

  /* Set the client task priority */

  param.sched_priority = CONFIG_EXAMPLES_NX_CLIENTPRIO;
  ret = sched_setparam(0, &param);
  if (ret < 0)
    {
      gdbg("ERROR: sched_setparam failed: %d\n" , ret);
      return EXIT_FAILURE;
    }

  /* Start the server task */

  gvdbg("Starting nx_servertask task\n");
  servrid = task_create("NX Server", CONFIG_EXAMPLES_NX_SERVERPRIO,
                        CONFIG_EXAMPLES_NX_STACKSIZE, nx_servertask, NULL);
  if (servrid < 0)
    {
      gdbg("ERROR: Failed to create nx_servertask task: %d\n", errno);
      return EXIT_FAILURE;
    }

  /* Wait a bit to let the server get started */

  sleep(1);

  /* Connect to the server */

  g_hnx = nx_connect();
  if (g_hnx)
    {
       pthread_attr_t attr;

       /* Start a separate thread to listen for server events.  This is probably
        * the least efficient way to do this, but it makes this example flow more
        * smoothly.
        */

       (void)pthread_attr_init(&attr);
       param.sched_priority = CONFIG_EXAMPLES_NX_LISTENERPRIO;
       (void)pthread_attr_setschedparam(&attr, &param);
       (void)pthread_attr_setstacksize(&attr, CONFIG_EXAMPLES_NX_STACKSIZE);

       ret = pthread_create(&thread, &attr, nx_listenerthread, NULL);
       if (ret != 0)
         {
            printf("pthread_create failed: %d\n", ret);
            return EXIT_FAILURE;
         }

       /* Don't return until we are connected to the server */

       while (!g_connected)
         {
           /* Wait for the listener thread to wake us up when we really
            * are connected.
            */

           (void)sem_wait(&g_semevent);
         }
    }
  else
    {
      gdbg("ERROR: nx_connect failed: %d\n", errno);
      return EXIT_FAILURE;
    }

  return EXIT_SUCCESS;
}
#endif

/****************************************************************************
 * Name: ov2640_camera_initialize
 ****************************************************************************/

static inline int ov2640_camera_initialize(void)
{
  /* Enable clocking to the ISI peripheral */

  sam_isi_enableclk();

#warning Missing Logic
  return EXIT_FAILURE;
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: ov2640_main
 *
 * Description:
 *   Entry point for the OV2640 Camera Demo
 *
 ************************************************************************************/

int ov2640_main(int argc, char *argv[])
{
  int ret;

  /* First, initialize the display */

  ret = ov2640_lcd_initialize();
  if (ret != EXIT_SUCCESS)
    {
      gdbg("ERROR: ov2640_lcd_initialize failed\n");
      return  EXIT_FAILURE;
    }

  /* Then, initialize the camera */

  ret = ov2640_camera_initialize();
  if (ret != EXIT_SUCCESS)
    {
      gdbg("ERROR: ov2640_camera_initialize failed\n");
      goto  errout_with_nx;
    }

  return EXIT_SUCCESS;

errout_with_nx:
#ifdef CONFIG_NX_MULTIUSER
  /* Disconnect from the server */

  gvdbg("Disconnect from the server\n");
  nx_disconnect(g_hnx);
#else
  /* Close the server */

  gvdbg("Close NX\n");
  nx_close(g_hnx);
#endif
  return EXIT_FAILURE;
}

#endif /* CONFIG_SAMA5_ISI && CONFIG_SAMA5_OV2640_DEMO */
