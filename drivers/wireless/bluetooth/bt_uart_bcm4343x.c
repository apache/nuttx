/****************************************************************************
 * drivers/wireless/bluetooth/bt_uart_bcm4343x.c
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

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/semaphore.h>
#include <nuttx/serial/tioctl.h>
#include <termios.h>

#include "bt_uart.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HCIUART_DEFAULT_SPEED 2000000
#define HCIUART_LOW_SPEED     115200

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint8_t g_hcd_patchram_command  = 0x2e;
static const uint8_t g_hcd_launch_command    = 0x4e;
static const uint8_t g_hcd_write_command     = 0x4c;
static const uint8_t g_hcd_command_byte2     = 0xfc;

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern const long int g_bt_firmware_len;
extern const uint8_t g_bt_firmware_hcd[];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hciuartCB
 *
 * Callback from hcirx indicating that there are data to read.
 *
 * Input Parameters:
 *   lower - an instance of the lower half driver interface
 *   param - pointer to the semaphore to indicate data readiness
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void hciuart_cb(FAR const struct btuart_lowerhalf_s *lower,
                       FAR void *param)
{
  FAR sem_t *rxsem = (FAR sem_t *)param;

  /* Data has arrived - post to release the semaphore */

  nxsem_post(rxsem);
}

/****************************************************************************
 * Name: uartwriteconf
 *
 * Write to HCI UART and get confirmation back
 *
 * Input Parameters:
 *   lower - an instance of the lower half driver interface
 *   rxsem - pointer to the semaphore to indicate data readiness
 *   dout  - pointer to data to send
 *   doutl - length of out data
 *   cmp   - pointer to comparison data on inbound side
 *   maxl  - length of data to receive on inbound side
 *
 * Returned Value:
 *   0 or error code.
 *
 ****************************************************************************/

static int uartwriteconf(FAR const struct btuart_lowerhalf_s *lower,
                         FAR sem_t *rxsem,
                         FAR const uint8_t *dout, uint32_t doutl,
                         FAR const uint8_t *cmp, uint32_t maxl)
{
  int ret;
  int gotlen = 0;
  FAR uint8_t *din;

  DEBUGASSERT(lower != NULL);

  lower->rxdrain(lower);

  ret = lower->write(lower, dout, doutl);
  if (ret < 0)
    {
      wlerr("Failed to write\n");
      goto exit_uartwriteconf_nofree;
    }

  if ((cmp == NULL) || (maxl == 0))
    {
      ret = OK;
      goto exit_uartwriteconf_nofree;
    }

  /* We've been asked to check the response too ... */

  din = kmm_malloc(maxl);
  while (gotlen < maxl)
    {
      ret = nxsem_tickwait_uninterruptible(rxsem, MSEC2TICK(100));
      if (ret < 0)
        {
          /* We didn't receive enough message, so fall out */

          wlerr("Response timed out: %d\n", ret);
          goto exit_uartwriteconf;
        }

      ret = lower->read(lower, &din[gotlen], maxl - gotlen);
      if (ret < 0)
        {
          wlerr("Couldn't read: %d\n", ret);
          ret = -ECOMM;
          goto exit_uartwriteconf;
        }

      gotlen += ret;
    }

  ret = ((memcmp(din, cmp, maxl) == 0) ? 0 : -ECOMM);

exit_uartwriteconf:
  kmm_free(din);
exit_uartwriteconf_nofree:
  return ret;
}

/****************************************************************************
 * Name: set_baudrate
 *
 * Set baudrate to be used in future for UART comms. In case of error the
 * current baudrate is not changed.
 *
 * Input Parameters:
 *   lower - an instance of the lower half driver interface
 *   rxsem - pointer to the semaphore to indicate data readiness
 *   targetspeed  - new baudrate to be used
 *
 * Returned Value:
 *   0 or error code.
 *
 ****************************************************************************/

static int set_baudrate(FAR const struct btuart_lowerhalf_s *lower,
                        FAR sem_t *rxsem, int targetspeed)
{
  int ret;
  uint8_t baudrate_cmd[] =
    {
      0x01, 0x18, g_hcd_command_byte2, 0x06, 0x00, 0x00,
      targetspeed & 0xff, (targetspeed >> 8) & 0xff,
      (targetspeed >> 16) & 0xff, (targetspeed >> 24) & 0xff
    };

  const uint8_t baudrate_conf[] =
    {
      0x04, 0x0e, 0x04, 0x01, 0x18, g_hcd_command_byte2, 0x00
    };

  ret = uartwriteconf(lower, rxsem, baudrate_cmd, sizeof(baudrate_cmd),
                       baudrate_conf, sizeof(baudrate_conf));

  if (ret == OK)
    {
      ret = lower->setbaud(lower, targetspeed);
    }

  return ret;
}

/****************************************************************************
 * Name: load_bcm4343x_firmware
 *
 * Attempt to load firmware into target device.
 *
 * Input Parameters:
 *   lower - an instance of the lower half driver interface
 *
 * Returned Value:
 *   0 or error code.
 *
 ****************************************************************************/

static int load_bcm4343x_firmware(FAR const struct btuart_lowerhalf_s *lower)
{
  FAR uint8_t *rp = (FAR uint8_t *)g_bt_firmware_hcd;
  int ret = OK;
  sem_t rxsem;
  uint8_t command;
  uint8_t txlen;
  uint8_t istx = 1;
  uint32_t blockattempts;

  /* Various responses to upload commands */

  const uint8_t command_resp[] =
    {
      0x04, 0x0e, 0x04, 0x01, g_hcd_write_command, g_hcd_command_byte2,
      0x00
    };

  const uint8_t launch_resp[] =
    {
      0x04, 0x0e, 0x04, 0x01, g_hcd_launch_command, g_hcd_command_byte2,
      0x00
    };

  const uint8_t download_resp[] =
    {
      0x04, 0x0e, 0x04, 0x01, g_hcd_patchram_command, g_hcd_command_byte2,
      0x00
    };

  /* Command to switch the chip into download mode */

  const uint8_t enter_download_mode[] =
    {
      0x01, g_hcd_patchram_command, g_hcd_command_byte2,
      0x00
    };

  /* Let's temporarily connect to the hci uart rx callback so we can get
   * data.
   */

  lower->rxattach(lower, hciuart_cb, &rxsem);
  lower->rxenable(lower, true);

  nxsem_init(&rxsem, 0, 0);
  nxsem_set_protocol(&rxsem, SEM_PRIO_NONE);

  /* It is possible this could fail if modem is already at high speed, so we
   * can safely ignore error return value.
   */

  lower->setbaud(lower, HCIUART_LOW_SPEED);
  set_baudrate(lower, &rxsem, HCIUART_DEFAULT_SPEED);

  /* New baudrate is established, prepare to receive firmware */

  ret = uartwriteconf(lower, &rxsem,
                      enter_download_mode, sizeof(enter_download_mode),
                      download_resp, sizeof(download_resp));
  if (ret != OK)
    {
      wlerr("Failed to enter download mode\n");
      ret = -ECOMM;
      goto load_bcm4343x_firmware_finished;
    }

  /* ...so now we can spin, pushing firmware into the chip */

  while (rp < (g_bt_firmware_hcd + g_bt_firmware_len))
    {
      command = rp[0];
      txlen = rp[2];

      if (command == g_hcd_launch_command)
        {
          break;
        }

      /* Try a few times for each block, just in case */

      blockattempts = 0;
      do
        {
          lower->write(lower, &istx, 1);
          ret = uartwriteconf(lower, &rxsem, rp, 3 + txlen,
                              command_resp, sizeof(command_resp));
          if (ret)
            {
              wlwarn("block offset %x upload failed, retrying\n",
                            rp - g_bt_firmware_hcd);
            }
        }
      while ((ret != 0) && (++blockattempts < 3));

      if (ret != 0)
        {
          wlerr("block upload repeatedly failed, aborting\n");
          goto load_bcm4343x_firmware_finished;
        }

      /* The +3 in here is the header bytes from the source file */

      rp += 3 + txlen;
    }

  /* To have gotten here we must've uploaded correctly, or barfed out */

  if (command == g_hcd_launch_command)
    {
      lower->write(lower, &istx, 1);
      ret = uartwriteconf(lower, &rxsem, rp, 3 + txlen,
                          launch_resp, sizeof(launch_resp));
      if (ret != 0)
        {
          wlerr("failed to launch firmware\n");
          goto load_bcm4343x_firmware_finished;
        }

      /* Give everything time to start up */

      nxsig_usleep(1000000);

      /* Once the firmware has booted it goes back to low speed,
       * so kick it up again
       */

      lower->setbaud(lower, HCIUART_LOW_SPEED);

      ret = set_baudrate(lower, &rxsem, HCIUART_DEFAULT_SPEED);
    }
  else
    {
      ret = -ECOMM;
    }

  load_bcm4343x_firmware_finished:
  lower->rxenable(lower, false);
  lower->rxattach(lower, NULL, NULL);

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: btuart_register
 *
 *   Create the UART-based Bluetooth device and register it with the
 *   Bluetooth stack.
 *
 * Input Parameters:
 *   lower - an instance of the lower half driver interface
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.
 *
 ****************************************************************************/

int btuart_register(FAR const struct btuart_lowerhalf_s *lower)
{
  FAR struct btuart_upperhalf_s *upper;
  int ret;

  wlinfo("lower %p\n", lower);

  if (lower == NULL)
    {
      wlerr("ERROR: btuart lower half is NULL\n");
      return -ENODEV;
    }

  /* Allocate a new instance of the upper half driver state structure */

  upper = (FAR struct btuart_upperhalf_s *)
      kmm_zalloc(sizeof(struct btuart_upperhalf_s));

  if (upper == NULL)
    {
      wlerr("ERROR: Failed to allocate upper-half state\n");
      return -ENOMEM;
    }

  /* Initialize the upper half driver state */

  upper->dev.head_reserve = H4_HEADER_SIZE;
  upper->dev.open = btuart_open;
  upper->dev.send = btuart_send;
  upper->lower = lower;

  /* Load firmware */

  ret = load_bcm4343x_firmware(lower);
  if (ret < 0)
    {
      wlerr("ERROR: Firmware error\n");
      kmm_free(upper);
      return -EINVAL;
    }

  /* And register the driver with the network and the Bluetooth stack. */

  ret = bt_netdev_register(&upper->dev);
  if (ret < 0)
    {
      wlerr("ERROR: bt_netdev_register failed: %d\n", ret);
      kmm_free(upper);
    }

  return ret;
}
