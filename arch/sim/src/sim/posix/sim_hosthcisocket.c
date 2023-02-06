/****************************************************************************
 * arch/sim/src/sim/posix/sim_hosthcisocket.c
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

#include <sys/types.h>
#include <sys/uio.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include "sim_internal.h"
#include "sim_hostusrsock.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BTPROTO_HCI 1

#define HCI_CHANNEL_RAW   0
#define HCI_CHANNEL_USER  1

#define HCIDEVDOWN  0x400448ca

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sockaddr_hci
{
  sa_family_t     hci_family;
  unsigned short  hci_dev;
  unsigned short  hci_channel;
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: host_bthcisock_avail
 *
 * Description:
 *   Monitor the host user channel to see if I/O is possible on socket.
 *
 * Input Parameters:
 *   fd: Host Bluetooth socket fd
 *
 * Returned Value:
 *   TRUE is returned on I/O available
 *
 ****************************************************************************/

int host_bthcisock_avail(int fd)
{
  struct timeval tv;
  fd_set fdset;

  /* We can't do anything if we failed to open the user channel */

  if (fd < 0)
    {
      return 0;
    }

  /* Wait for data on the user channel (or a timeout) */

  tv.tv_sec  = 0;
  tv.tv_usec = 0;

  FD_ZERO(&fdset);
  FD_SET(fd, &fdset);

  return select(fd + 1, &fdset, NULL, NULL, &tv) > 0;
}

/****************************************************************************
 * Name: host_bthcisock_send
 *
 * Description:
 *   Send a Bluetooth packet out via the host user socket.
 *
 * Input Parameters:
 *   fd  : Host Bluetooth socket fd
 *   data: Pointer to the HCI packet
 *   len : Length of packet
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.
 *
 ****************************************************************************/

int host_bthcisock_send(int fd, const void *data, size_t len)
{
  while (write(fd, data, len) < 0)
    {
      if (errno == EAGAIN || errno == EINTR)
        {
          continue;
        }

      return -1;
    }

  return 0;
}

/****************************************************************************
 * Name: host_bthcisock_receive
 *
 * Description:
 *   Read from the Host HCI socket interface.
 *
 * Input Parameters:
 *   fd  : Host Bluetooth socket fd
 *   data: Pointer to store HCI packet
 *   len : Maximum length of packet
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.
 *
 ****************************************************************************/

int host_bthcisock_receive(int fd, void *data, size_t len)
{
  int err;

  while ((err = read(fd, data, len)) < 0 && (errno == EINTR));

  if (err <= 0)
    {
      /* Both an empty read and an error are "error" conditions */

      return -1;
    }

  /* Return the number of bytes written to data */

  return err;
}

/****************************************************************************
 * Name: host_bthcisock_open
 *
 * Description:
 *   Open a User Channel HCI socket on the Host for the given device.
 *   This will also disconnect the device from existing management. It can
 *   still be monitored using an HCI monitor socket.
 *
 * Input Parameters:
 *   dev_idx: This is the device index to be connected to.  HCI0 would be 0.
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.
 *
 ****************************************************************************/

int host_bthcisock_open(int dev_idx)
{
  int err;
  struct sockaddr_hci addr;
  int fd = socket(PF_BLUETOOTH, SOCK_RAW | SOCK_CLOEXEC | SOCK_NONBLOCK,
                  BTPROTO_HCI);
  if (fd < 0)
    {
      return fd;
    }

  /* We must bring the device down before binding to user channel */

  err = ioctl(fd, HCIDEVDOWN, dev_idx);
  if (err < 0)
    {
      return err;
    }

  memset(&addr, 0, sizeof(addr));
  addr.hci_family = AF_BLUETOOTH;
  addr.hci_dev = dev_idx;
  addr.hci_channel = HCI_CHANNEL_USER;

  err = bind(fd, (struct sockaddr *) &addr, sizeof(addr));
  if (err < 0)
    {
      close(fd);
      return err;
    }

  return fd;
}

/****************************************************************************
 * Name: host_bthcisock_close
 *
 * Description:
 *   Close a User Channel HCI socket on the Host for the given device idx.
 *
 * Input Parameters:
 *   fd: The resources associated with the open user channel are freed.
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.
 *
 ****************************************************************************/

int host_bthcisock_close(int fd)
{
  return close(fd);
}
