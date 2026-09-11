/****************************************************************************
 * boards/arm/rk3506/luckfox-lyra-amp/src/rk3506_bringup.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <stdint.h>
#include <stdio.h>
#include <fcntl.h>
#include <syslog.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>
#include <nuttx/kthread.h>
#include <nuttx/serial/serial.h>

#ifdef CONFIG_SENSORS_FAKESENSOR
#  include <nuttx/uorb.h>
#  include <nuttx/sensors/fakesensor.h>
#endif

#include "chip.h"
#include "rk3506.h"

/* Console uart_dev_t accessor exported by drivers/serial/uart_16550.c. */

FAR uart_dev_t *u16550_consoledev(void);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Poll-driven console RX worker.
 *
 * UART4 RX is wired to GIC SPI IRQ70. On this AMP SoC the GIC distributor is
 * shared with Linux, so IRQ70 cannot be delivered to CPU2 reliably (Linux
 * owns/reconfigures the distributor and the SPI enable/target for IRQ70 does
 * not stick). Polling the UART bypasses the GIC entirely and is reliable;
 * TX and the per-CPU PPI timer are unaffected.
 */

static int rk3506_rxpoll(int argc, char *argv[])
{
  FAR uart_dev_t *con = u16550_consoledev();

  /* Disable IRQ70 so the (unreliable) RX interrupt never races this poller
   * for the same RBR byte. Give NSH a moment to open the console first.
   */

  usleep(200000);
  up_disable_irq(70);

  /* Poll the Line Status Register; when data is ready, push it up through
   * the normal serial upper-half via uart_recvchars() - exactly what the RX
   * ISR would have done.
   *
   *   LSR @ 0xff0e0000 + (5 << 2), Data-Ready = bit0
   */

  for (; ; )
    {
      uint32_t lsr = *(volatile uint32_t *)(0xff0e0000 + (5 << 2));

      if (lsr & 0x01)
        {
          uart_recvchars(con);
        }
      else
        {
          usleep(2000);
        }
    }

  return 0;
}

#ifdef CONFIG_SENSORS_FAKESENSOR
/* Register a demo accelerometer backed by a small CSV that fakesensor
 * replays in a loop. This brings up a live sensor node at
 * /dev/uorb/sensor_accel0 with no real hardware, so the full uORB pipeline
 * (sensor framework -> uORB -> read/poll) can be exercised on the board:
 *
 *   uorb_listener sensor_accel        # watch the live stream
 *   cat /dev/uorb/sensor_accel0       # raw binary samples
 *
 * The CSV format is fakesensor's own: first line "interval:<ms>", second
 * line the header, then rows of samples. EOF wraps back to the top, so a
 * handful of rows produces an endless stream.
 */

static void rk3506_fakesensor_setup(void)
{
  static const char csv[] =
    "interval:100\n"   /* 10 Hz */
    "x,y,z\n"
    "0.10,0.00,9.81\n"
    "0.20,0.05,9.79\n"
    "0.05,-0.05,9.82\n"
    "-0.10,0.10,9.80\n";

  const char *path = CONFIG_LIBC_TMPDIR "/accel0.csv";
  struct file f;
  int ret;

  ret = file_open(&f, path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: fakesensor CSV open %s: %d\n", path, ret);
      return;
    }

  file_write(&f, csv, sizeof(csv) - 1);
  file_close(&f);

  /* type, csv path, devno=0, batch buffer = 8 samples */

  ret = fakesensor_init(SENSOR_TYPE_ACCELEROMETER, path, 0, 8);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: fakesensor_init: %d\n", ret);
      return;
    }

  syslog(LOG_INFO, "fakesensor accel0 -> /dev/uorb/sensor_accel0\n");
}
#endif /* CONFIG_SENSORS_FAKESENSOR */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rk3506_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

int rk3506_bringup(void)
{
  int ret;

  UNUSED(ret);

  /* Defer the rest of bring-up (and the subsequent one-shot NSH banner) a
   * few seconds. The console UART4 is observed through a board-powered CH340
   * USB-serial adapter that re-enumerates on the host for ~2-3s after every
   * board reset. Without this delay the NSH banner is emitted while the host
   * port is still gone and is lost. This also exercises the timer tick.
   */

  sleep(3);

  /* Spawn the poll-driven console RX worker (see rk3506_rxpoll above:
   * interrupt-driven RX is unreliable on the AMP-shared GIC).
   */

  kthread_create("rxpoll", 100, 2048, rk3506_rxpoll, NULL);

#ifdef CONFIG_FS_TMPFS
  /* Mount the tmpfs file system */

  ret = nx_mount(NULL, CONFIG_LIBC_TMPDIR, "tmpfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount tmpfs at %s: %d\n",
             CONFIG_LIBC_TMPDIR, ret);
    }
#endif

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_FAKESENSOR
  /* Register a demo accelerometer (CSV-backed) on /dev/uorb/sensor_accel0. */

  rk3506_fakesensor_setup();
#endif

  return OK;
}
