/****************************************************************************
 * arch/sim/src/sim/posix/vpnkit/sim_negotiate.c
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>

#include "sim_protocol.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ERROR(fmt, ...) \
        syslog(LOG_ERR, "sim_vpnkit: " fmt "\n", ##__VA_ARGS__)
#define INFO(fmt, ...) \
        syslog(LOG_ERR, "sim_vpnkit: " fmt "\n", ##__VA_ARGS__)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Negotiate a vmnet connection, returns 0 on success and 1 on error. */

int negotiate(int fd, struct vif_info *vif)
{
  enum command command = ethernet;
  struct init_message *me;
  struct ethernet_args args;
  struct init_message you;
  char *txt;

  me = create_init_message();
  if (!me)
    {
      goto err;
    }

  if (write_init_message(fd, me) == -1)
    {
      goto err;
    }

  if (read_init_message(fd, &you) == -1)
    {
      goto err;
    }

  if (me->version != you.version)
    {
      ERROR("Server did not accept our protocol version "
            "(client: %d, server: %d)", me->version, you.version);
      goto err;
    }

  txt = print_init_message(&you);
  if (!txt)
    {
      goto err;
    }

  INFO("Server reports %s", txt);
  free(txt);

  if (write_command(fd, &command) == -1)
    {
      goto err;
    }

  /* We don't need a uuid */

  memset(&args.uuid_string[0], 0, sizeof(args.uuid_string));
  if (write_ethernet_args(fd, &args) == -1)
    {
      goto err;
    }

  if (read_vif_response(fd, vif) == -1)
    {
      goto err;
    }

  return 0;
err:
  ERROR("Failed to negotiate vmnet connection");
  return 1;
}
