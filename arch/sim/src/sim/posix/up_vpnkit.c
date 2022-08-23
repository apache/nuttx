/****************************************************************************
 * arch/sim/src/sim/posix/up_vpnkit.c
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
#include <sys/un.h>

/* XXX broken api: mixing nuttx and host in_addr_t */

#include <netinet/in.h>

#include <poll.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <syslog.h>
#include <unistd.h>

#include "up_internal.h"
#include "vpnkit/protocol.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ERROR(fmt, ...) \
        syslog(LOG_ERR, "up_vpnkit: " fmt "\n", ##__VA_ARGS__)
#define INFO(fmt, ...) \
        syslog(LOG_ERR, "up_vpnkit: " fmt "\n", ##__VA_ARGS__)
#define DEBUG(fmt, ...)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char *g_vpnkit_socket_path = CONFIG_SIM_NETDEV_VPNKIT_PATH;
static struct vif_info g_vifinfo;
static int g_vpnkit_fd = -1;
static bool g_connect_warned;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int vpnkit_connect(void)
{
  struct sockaddr_un sun;
  int fd;

  if (g_vpnkit_fd != -1)
    {
      return 0;
    }

  fd = socket(AF_UNIX, SOCK_STREAM, 0);
  if (fd == -1)
    {
      ERROR("failed to create a socket");
      return -1;
    }

  memset(&sun, 0, sizeof(sun));
  sun.sun_family = AF_UNIX;
  strncpy(sun.sun_path, g_vpnkit_socket_path, sizeof(sun.sun_path) - 1);

  if (connect(fd, (const struct sockaddr *)&sun, sizeof(sun)) == -1)
    {
      if (!g_connect_warned)
        {
          ERROR("failed to connect to the vpnkit socket %s",
                g_vpnkit_socket_path);
          g_connect_warned = true;
        }

      close(fd);
      return -1;
    }

  if (negotiate(fd, &g_vifinfo))
    {
      ERROR("failed to negotiate with vpnkit");
      close(fd);
      return -1;
    }

  INFO("Successfully negotiated with vpnkit");
  g_vpnkit_fd = fd;
  g_connect_warned = false;
  netdriver_setmacaddr(g_vifinfo.mac);
  return 0;
}

static void vpnkit_disconnect(void)
{
  if (g_vpnkit_fd == -1)
    {
      return;
    }

  INFO("disconnecting from vpnkit");
  close(g_vpnkit_fd);
  g_vpnkit_fd = -1;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vpnkit_init
 *
 ****************************************************************************/

void vpnkit_init(void *priv,
                 void (*tx_done_intr_cb)(void *priv),
                 void (*rx_ready_intr_cb)(void *priv))
{
  /* TODO: support emulation of TX done and RX ready interrupts */

  vpnkit_connect();
}

/****************************************************************************
 * Name: vpnkit_avail
 *
 ****************************************************************************/

int vpnkit_avail(void)
{
  struct pollfd pfd;
  int ret;

  if (vpnkit_connect())
    {
      return 0;
    }

  memset(&pfd, 0, sizeof(pfd));
  pfd.fd = g_vpnkit_fd;
  pfd.events = POLLIN;
  ret = poll(&pfd, 1, 0);
  if (ret == -1)
    {
      ERROR("poll failed on vpnkit socket");
      vpnkit_disconnect();
      return 0;
    }
  else if (ret != 0)
    {
      DEBUG("vpnkit_avail is returning 1");
    }

  return ret != 0;
}

/****************************************************************************
 * Name: vpnkit_read
 *
 ****************************************************************************/

unsigned int vpnkit_read(unsigned char *buf, unsigned int buflen)
{
  uint8_t header[2]; /* 16-bit payload length in little endian */
  size_t packet_len;
  ssize_t ret;
  DEBUG("vpnkit_read called");

  if (vpnkit_connect())
    {
      return 0;
    }

  ret = really_read(g_vpnkit_fd, header, sizeof(header));
  if (ret == -1)
    {
      ERROR("failed to read packet header");
      vpnkit_disconnect();
      return 0;
    }

  packet_len = (header[1] << 8) | header[0];
  if (packet_len > buflen)
    {
      INFO("packet larger (%zu) than buffer size (%u)", packet_len, buflen);

      /* XXX it's better to drop this particular packet it. */

      vpnkit_disconnect();
      return 0;
    }

  ret = really_read(g_vpnkit_fd, buf, packet_len);
  if (ret == -1)
    {
      ERROR("failed to read packet");
      vpnkit_disconnect();
      return 0;
    }

  DEBUG("a packet received (size %zu)", packet_len);
  return packet_len;
}

/****************************************************************************
 * Name: vpnkit_send
 *
 ****************************************************************************/

void vpnkit_send(unsigned char *buf, unsigned int buflen)
{
  uint8_t header[2]; /* 16-bit payload length in little endian */
  ssize_t ret;

  DEBUG("vpnkit_send called");
  if (vpnkit_connect())
    {
      return;
    }

  header[0] = buflen & 0xff;
  header[1] = (buflen >> 8) & 0xff;
  ret = really_write(g_vpnkit_fd, header, sizeof(header));
  if (ret == -1)
    {
      ERROR("failed to write packet header");
      vpnkit_disconnect();
      return;
    }

  ret = really_write(g_vpnkit_fd, buf, buflen);
  if (ret == -1)
    {
      ERROR("failed to write packet payload");
      vpnkit_disconnect();
      return;
    }

  DEBUG("a packet sent (size %u)", buflen);
}
