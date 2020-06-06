/****************************************************************************
 * boards/sim/sim/sim/src/sim_touchscreen.c
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

#include <sys/types.h>
#include <sys/boardctl.h>
#include <stdint.h>
#include <stdbool.h>
#include <sched.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/sched.h>
#include <nuttx/board.h>
#include <nuttx/video/fb.h>
#include <nuttx/input/touchscreen.h>
#include <nuttx/semaphore.h>
#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxglib.h>

#ifdef CONFIG_VNCSERVER
#  include <nuttx/video/vnc.h>
#endif

#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Pick a background color */

#ifndef CONFIG_EXAMPLES_TOUCHSCREEN_BGCOLOR
#  define CONFIG_EXAMPLES_TOUCHSCREEN_BGCOLOR 0x007b68ee
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sim_touchscreen_s
{
  NXHANDLE hnx;
  bool connected;
  sem_t eventsem;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct sim_touchscreen_s g_simtc =
{
  NULL,               /* hnx */
  false,              /* connected */
  SEM_INITIALIZER(0), /* eventsem */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_listener
 ****************************************************************************/

static FAR void *sim_listener(FAR void *arg)
{
  int ret;

  /* Process events forever */

  for (; ; )
    {
      /* Handle the next event.  If we were configured blocking, then
       * we will stay right here until the next event is received.  Since
       * we have dedicated a while thread to servicing events, it would
       * be most natural to also select CONFIG_NX_BLOCKING -- if not, the
       * following would be a tight infinite loop (unless we added addition
       * logic with nx_eventnotify and sigwait to pace it).
       */

      ret = nx_eventhandler(g_simtc.hnx);
      if (ret < 0)
        {
          /* An error occurred... assume that we have lost connection with
           * the server.
           */

          fwarn("WARNING: Lost server connection: %d\n", errno);
          break;
        }

      /* If we received a message, we must be connected */

      if (!g_simtc.connected)
        {
          g_simtc.connected = true;
          nxsem_post(&g_simtc.eventsem);
          ginfo("Connected\n");
        }
    }

  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_tsc_setup
 *
 * Description:
 *   This function is called by board-bringup logic to configure the
 *   touchscreen device.  This function will register the driver as
 *   /dev/inputN where N is the minor device number.
 *
 * Input Parameters:
 *   minor   - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int sim_tsc_setup(int minor)
{
  struct sched_param param;
  nxgl_mxpixel_t color;
  pthread_t thread;
  int ret;

  /* Set the client task priority */

  param.sched_priority = CONFIG_SIM_CLIENTPRIO;
  ret = nxsched_set_param(0, &param);
  if (ret < 0)
    {
      gerr("ERROR: nxsched_set_param failed: %d\n" , ret);
      return ret;
    }

  /* Start the NX server kernel thread */

  ret = boardctl(BOARDIOC_NX_START, 0);
  if (ret < 0)
    {
      gerr("ERROR: Failed to start the NX server: %d\n", errno);
      return ret;
    }

  /* Connect to the server */

  g_simtc.hnx = nx_connect();
  if (g_simtc.hnx)
    {
       pthread_attr_t attr;

#ifdef CONFIG_VNCSERVER
      /* Setup the VNC server to support keyboard/mouse inputs */

      ret = vnc_default_fbinitialize(0, g_simtc.hnx);
      if (ret < 0)
        {
          ginfo("ERROR: vnc_default_fbinitialize failed: %d\n", ret);
          nx_disconnect(g_simtc.hnx);
          return ret;
        }
#endif

      /* Start a separate thread to listen for server events.
       * This is probably the least efficient way to do this,
       * but it makes this example flow more smoothly.
       */

      pthread_attr_init(&attr);
      param.sched_priority = CONFIG_SIM_LISTENERPRIO;
      pthread_attr_setschedparam(&attr, &param);
      pthread_attr_setstacksize(&attr, CONFIG_SIM_LISTENER_STACKSIZE);

      ret = pthread_create(&thread, &attr, sim_listener, NULL);
      if (ret != 0)
        {
          gerr("ERROR: pthread_create failed: %d\n", ret);
          return -ret;
        }

      /* Don't return until we are connected to the server */

      while (!g_simtc.connected)
        {
          /* Wait for the listener thread to wake us up when we really
           * are connected.
           */

           nxsem_wait(&g_simtc.eventsem);
        }
    }
  else
    {
      gerr("ERROR: nx_connect failed: %d\n", errno);
      return ERROR;
    }

  /* Set the background to the configured background color */

  iinfo("Set background color=%d\n", CONFIG_EXAMPLES_TOUCHSCREEN_BGCOLOR);

  color = CONFIG_EXAMPLES_TOUCHSCREEN_BGCOLOR;
  ret = nx_setbgcolor(g_simtc.hnx, &color);
  if (ret < 0)
    {
      ierr("ERROR: nx_setbgcolor failed: %d\n", ret);
      goto errout_with_nx;
    }

  /* Finally, initialize the touchscreen simulation on the X window */

  ret = sim_tsc_initialize(minor);
  if (ret < 0)
    {
      ierr("ERROR: sim_tsc_initialize failed: %d\n", ret);
      goto errout_with_nx;
    }

  return OK;

errout_with_nx:
  nx_disconnect(g_simtc.hnx);
  goto errout;

errout:
  return ret;
}
