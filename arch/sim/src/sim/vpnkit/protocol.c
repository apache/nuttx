/****************************************************************************
 * arch/sim/src/sim/vpnkit/protocol.c
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
 * This file is derivative from vpnkit.
 *
 * Copyright 2013-2016 Docker, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/socket.h>
#include <sys/types.h>
#include <sys/uio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <syslog.h>

#include "protocol.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ERROR(fmt, ...) \
        syslog(LOG_ERR, "up_vpnkit: " fmt "\n", ##__VA_ARGS__)

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Version 0 of the protocol used this */

char expected_hello_old[5] =
{
  'V', 'M', 'N', 'E', 'T'
};

/* Version 1 and later of the protocol used this */

char expected_hello[5] =
{
  'V', 'M', 'N', '3', 'T'
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int really_read(int fd, uint8_t *buffer, size_t total)
{
  size_t remaining = total;
  ssize_t n;

  while (remaining > 0)
    {
      n = read(fd, buffer, remaining);
      if (n == 0)
        {
          ERROR("EOF reading from socket: closing\n");
          goto err;
        }

      if (n < 0)
        {
          ERROR("Failure reading from socket: closing: %s",
                strerror(errno));
          goto err;
        }

      remaining -= (size_t)n;
      buffer = buffer + n;
    }

  return 0;
err:
  /* On error: stop reading from the socket and trigger a clean
   * shutdown
   */

  shutdown(fd, SHUT_RD);
  return -1;
}

int really_write(int fd, uint8_t *buffer, size_t total)
{
  size_t remaining = total;
  ssize_t n;

  while (remaining > 0)
    {
      n = write(fd, buffer, remaining);
      if (n == 0)
        {
          ERROR("EOF writing to socket: closing");
          goto err;
        }

      if (n < 0)
        {
          ERROR("Failure writing to socket: closing: %s",
                strerror(errno));
          goto err;
        }

      remaining -= (size_t) n;
      buffer = buffer + n;
    }

  return 0;
err:

  /* On error: stop listening to the socket */

  shutdown(fd, SHUT_WR);
  return -1;
}

struct init_message *create_init_message()
{
  struct init_message *m;

  m = malloc(sizeof(struct init_message));
  if (!m)
    {
      return NULL;
    }

  bzero(m, sizeof(struct init_message));
  memcpy(&m->hello[0], &expected_hello[0], sizeof(m->hello));
  m->version = CURRENT_VERSION;
  memset(&m->commit[0], 0, sizeof(m->commit));

  return m;
}

char *print_init_message(struct init_message *m)
{
  char tmp[41];

  memcpy(&tmp[0], &m->commit[0], 40);
  tmp[40] = '\000';
  char *buffer;
  int n;

  buffer = malloc(80);
  if (!buffer)
    {
      return NULL;
    }

  n = snprintf(buffer, 80, "version %d, commit %s", m->version, tmp);
  if (n < 0)
    {
      perror("Failed to format init_message");
      exit(1);
    }

  return buffer;
}

int read_init_message(int fd, struct init_message *ci)
{
  int res;

  bzero(ci, sizeof(struct init_message));

  res = really_read(fd, (uint8_t *)&ci->hello[0], sizeof(ci->hello));
  if (res  == -1)
    {
      ERROR("Failed to read hello from client");
      return -1;
    }

  res = memcmp(&ci->hello[0],
               &expected_hello_old[0], sizeof(expected_hello_old));
  if (res == 0)
    {
      ci->version = 0;
      return 0;
    }

  res = memcmp(&ci->hello[0],
               &expected_hello[0], sizeof(expected_hello));
  if (res != 0)
    {
      ERROR("Failed to read header magic from client");
      return -1;
    }

  res = really_read(fd, (uint8_t *)&ci->version, sizeof(ci->version));
  if (res == -1)
    {
      ERROR("Failed to read header version from client");
      return -1;
    }

  res = really_read(fd, (uint8_t *)&ci->commit[0], sizeof(ci->commit));
  if (res == -1)
    {
      ERROR("Failed to read header hash from client");
      return -1;
    }

  return 0;
}

int write_init_message(int fd, struct init_message *ci)
{
  int res;

  res = really_write(fd, (uint8_t *)&ci->hello[0], sizeof(ci->hello));
  if (res == -1)
    {
      ERROR("Failed to write hello to client");
      return -1;
    }

  if (ci->version > 0)
    {
      res = really_write(fd, (uint8_t *)&ci->version,
                         sizeof(ci->version));
      if (res == -1)
        {
          ERROR("Failed to write version to client");
          return -1;
        }

      res = really_write(fd, (uint8_t *)&ci->commit[0],
                         sizeof(ci->commit));
      if (res == -1)
        {
          ERROR("Failed to write header hash to client");
          return -1;
        }
    }

  return 0;
}

int read_vif_response(int fd, struct vif_info *vif)
{
  struct msg_response msg;

  if (really_read(fd, (uint8_t *)&msg, sizeof(msg)) == -1)
    {
      ERROR("Client failed to read server response");
      return -1;
    }

  switch (msg.response_type)
    {
      case rt_vif:
          memcpy((uint8_t *)vif, (uint8_t *)&msg.vif, sizeof(*vif));
          return 0;
      case rt_disconnect:
          ERROR("Server disconnected: %*s", msg.disconnect.len,
                msg.disconnect.msg);
          return -1;
      default:
          ERROR("Unknown response type from server");
          return -1;
    }
}

int write_command(int fd, enum command *c)
{
  uint8_t command = *c;

  if (really_write(fd, (uint8_t *)&command, sizeof(command)) == -1)
    {
      ERROR("Failed to write command to client");
      return -1;
    }

  return 0;
}

int write_ethernet_args(int fd, struct ethernet_args *args)
{
  uint8_t buffer[40];
  memset(&buffer[0], 0, sizeof(buffer));
  memcpy(&buffer[0], (uint8_t *)&args->uuid_string[0], 36);

  if (really_write(fd, (uint8_t *)&buffer, sizeof(buffer)) == -1)
    {
      ERROR("Failed to write ethernet args to client");
      return -1;
    }

  return 0;
}
