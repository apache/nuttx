/****************************************************************************
 * drivers/usbhost/usbhost_waiter.c
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

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <stdio.h>

#include <nuttx/kthread.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbhost.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_waiter
 *
 * Description:
 *   Wait for USB devices to be connected.
 *
 ****************************************************************************/

static int usbhost_waiter(int argc, FAR char *argv[])
{
  FAR struct usbhost_connection_s *conn =
    (FAR struct usbhost_connection_s *)((uintptr_t)strtoul(argv[1],
                                                           NULL, 16));
  FAR struct usbhost_hubport_s *hport;

  uinfo("Running %p\n", conn);
  for (; ; )
    {
      /* Wait for the device to change state */

      DEBUGVERIFY(CONN_WAIT(conn, &hport));
      uinfo("%s\n", hport->connected ? "connected" : "disconnected");

      /* Did we just become connected? */

      if (hport->connected)
        {
          /* Yes.. enumerate the newly connected device */

          CONN_ENUMERATE(conn, hport);
        }
    }

  /* Keep the compiler from complaining */

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_waiter_initialize
 *
 * Description:
 *   Initialize the USB host waiter. This function will start a thread that
 *   will monitor for device connection/disconnection events.
 *
 ****************************************************************************/

int usbhost_waiter_initialize(FAR struct usbhost_connection_s *conn)
{
  FAR char *argv[2];
  char      arg1[32];

  /* Start a thread to handle device connection. */

  snprintf(arg1, 16, "%p", conn);
  argv[0] = arg1;
  argv[1] = NULL;
  return kthread_create("usbhost",
                        CONFIG_USBHOST_WAITER_PRIO,
                        CONFIG_USBHOST_WAITER_STACKSIZE,
                        usbhost_waiter, argv);
}
