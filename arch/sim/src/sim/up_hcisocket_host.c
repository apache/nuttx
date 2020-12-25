/****************************************************************************
 * arch/sim/src/sim/up_hcisocket.c
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

#include "up_internal.h"
#include "up_hcisocket_host.h"

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
 * Name: bthcisock_host_send
 *
 * Description:
 *   Send a Bluetooth packet out via the host user socket.
 *
 * Input Parameters:
 *   fd: Host Bluetooth socket fd
 *   pkt_type: Packet type as known to the Linux Bluetooth stack
 *   data: Pointer to the HCI packet
 *   len: Length of packet
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.
 *
 ****************************************************************************/

int bthcisock_host_send(int fd, uint8_t pkt_type, uint8_t *data, size_t len)
{
  struct iovec iv[2];

  iv[0].iov_base = &pkt_type;
  iv[0].iov_len = 1;
  iv[1].iov_base = data;
  iv[1].iov_len = len;

  while (writev(fd, iv, 2) < 0)
    {
      if (errno == EAGAIN || errno == EINTR)
        continue;
      return -1;
    }

  return 0;
}

/****************************************************************************
 * Name: bthcisock_host_read
 *
 * Description:
 *   Read from the Host HCI socket interface.
 *
 * Input Parameters:
 *   fd: Host Bluetooth socket fd
 *   data: Pointer to store HCI packet
 *   len: Maximum length of packet
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.
 *
 ****************************************************************************/

int bthcisock_host_read(int fd, uint8_t *type, void *buf, size_t len)
{
  int err;
  struct iovec iv[2];

  iv[0].iov_base = type;
  iv[0].iov_len = 1;
  iv[1].iov_base = buf;
  iv[1].iov_len = len;

  while ((err = readv(fd, iv, 2)) < 0 && (errno == EINTR));

  if (err <= 0)
    {
      /* Both an empty read and an error are "error" conditions */

      return -1;
    }

  /* Return the number of bytes written to buf so remove the header byte */

  return (err - 1);
}

/****************************************************************************
 * Name: bthcisock_host_open
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

int bthcisock_host_open(int dev_idx)
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

  err = ioctl(fd, HCIDEVDOWN, 0);
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
