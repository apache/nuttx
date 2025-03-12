/****************************************************************************
 * arch/sim/src/sim/posix/sim_hostcan.c
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

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <syslog.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "sim_hostcan.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: host_can_init
 ****************************************************************************/

int host_can_init(struct sim_can_s *can, int devidx)
{
  struct sockaddr_can addr;
  struct ifreq        ifr;
  int                 enable_canfd = 1;
  int                 ret;

  /* Get socket */

  can->fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (can->fd < 0)
    {
      return -errno;
    }

  /* Get SocketCAN interface */

  snprintf(ifr.ifr_name, IFNAMSIZ, "can%d", devidx);
  ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);
  if (!ifr.ifr_ifindex)
    {
      return -EINVAL;
    }

  memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  /* Switch to CAN FD mode */

  ret = setsockopt(can->fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd,
                   sizeof(enable_canfd));
  if (ret < 0)
    {
      return -errno;
    }

  /* Bind socket */

  ret = bind(can->fd, (struct sockaddr *)&addr, sizeof(addr));
  if (ret < 0)
    {
      return -errno;
    }

  return 0;
}

/****************************************************************************
 * Name: host_can_read
 *
 * Description:
 *   Read data from host CAN interface.
 *
 * Assumption:
 *   NuttX canfd_frame is byte compatible with host canfd_frame
 *
 ****************************************************************************/

int host_can_read(struct sim_can_s *can, void *frame)
{
  if (!can->ifup || can->fd < 0)
    {
      return -EIO;
    }

  return read(can->fd, frame, sizeof(struct canfd_frame));
}

/****************************************************************************
 * Name: host_can_send
 *
 * Description:
 *   Send data on host CAN interface.
 *
 * Assumption:
 *   NuttX canfd_frame is byte compatible with host canfd_frame
 *
 ****************************************************************************/

int host_can_send(struct sim_can_s *can, void *frame, size_t len)
{
  if (!can->ifup || can->fd < 0)
    {
      return -EIO;
    }

  if (len != sizeof(struct canfd_frame) && len != sizeof(struct can_frame))
    {
      return -EINVAL;
    }

  return write(can->fd, frame, len);
}

/****************************************************************************
 * Name: host_can_ifup
 ****************************************************************************/

int host_can_ifup(struct sim_can_s *can)
{
  can->ifup = true;
  return 0;
}

/****************************************************************************
 * Name: host_can_ifdown
 ****************************************************************************/

int host_can_ifdown(struct sim_can_s *can)
{
  can->ifup = false;
  return 0;
}

/****************************************************************************
 * Name: host_can_avail
 ****************************************************************************/

bool host_can_avail(struct sim_can_s *can)
{
  struct timeval tv;
  fd_set fdset;

  if (!can->ifup || can->fd < 0)
    {
      return false;
    }

  /* Wait for data on the user channel (or a timeout) */

  tv.tv_sec  = 0;
  tv.tv_usec = 0;

  FD_ZERO(&fdset);
  FD_SET(can->fd, &fdset);

  return select(can->fd + 1, &fdset, NULL, NULL, &tv) > 0;
}
