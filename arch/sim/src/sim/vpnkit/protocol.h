/****************************************************************************
 * arch/sim/src/sim/vpnkit/protocol.h
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

#ifndef _ARCH_SIM_SRC_SIM_VPNKIT_PROTOCOL_H_
#define _ARCH_SIM_SRC_SIM_VPNKIT_PROTOCOL_H_

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <errno.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* This should be bumped whenever we add something (like a feature or a
 * bugfix) and we wish the UI to be able to detect when to trigger a
 * reinstall.
 */

#define CURRENT_VERSION 22

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Client -> Server init_message */

/* Server -> Client init_message */

struct init_message
{
  char hello[5];
  uint8_t _padding[3];
  uint32_t version;
  char commit[40]; /* git sha of the compiled commit */
};

/* Client -> Server command */

enum command
{
  ethernet = 1,
};

/* Server -> Client response */

enum response_type
{
  rt_vif = 1,
  rt_disconnect = 2,
};

/* Client -> Server command arguments */

struct ethernet_args
{
  char uuid_string[36];
};

/* Server -> Client: details of a vif */

struct vif_info
{
  uint16_t mtu;
  uint16_t max_packet_size;
  uint8_t mac[6];
}
__attribute__((packed));

/* Server -> Client: disconnect w/reason */

struct disconnect_reason
{
  uint8_t len;
  char msg[256];
}
__attribute__((packed));

struct msg_response
{
  uint8_t response_type;
  union
  {
    struct vif_info vif;
    struct disconnect_reason disconnect;
  };
}
__attribute__((packed));

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern char expected_hello[5];
extern char expected_hello_old[5];

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int negotiate(int fd, struct vif_info *vif);

extern struct init_message *create_init_message(void);
extern int read_init_message(int fd, struct init_message *ci);
extern int write_init_message(int fd, struct init_message *ci);
extern char *print_init_message(struct init_message *m);

extern int write_command(int fd, enum command *c);

extern int write_ethernet_args(int fd, struct ethernet_args *args);

extern int read_vif_response(int fd, struct vif_info *vif);

extern int really_read(int fd, uint8_t *buffer, size_t total);
extern int really_write(int fd, uint8_t *buffer, size_t total);

#endif /* _ARCH_SIM_SRC_SIM_VPNKIT_PROTOCOL_H_ */
