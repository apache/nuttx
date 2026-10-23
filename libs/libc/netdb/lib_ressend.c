/****************************************************************************
 * libs/libc/netdb/lib_ressend.c
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

#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <resolv.h>

#include <arpa/nameser.h>
#include <sys/socket.h>

#include "netdb/lib_dns.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct res_send_s
{
  FAR const unsigned char *msg;   /* The query message */
  int msglen;                     /* The size of the query */
  FAR unsigned char *answer;      /* The buffer for the answer */
  int anslen;                     /* The size of the answer buffer */
  int result;                     /* Size of the answer or negated errno */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: res_send_callback
 *
 * Description:
 *   Send the query to one name server and wait for the answer.
 *
 * Returned Value:
 *   One (1) if an answer was received, zero (0) to try the next name
 *   server.
 *
 ****************************************************************************/

static int res_send_callback(FAR void *arg, FAR struct sockaddr *addr,
                             socklen_t addrlen)
{
  FAR struct res_send_s *rs = arg;
  ssize_t nrecv;
  int retries;
  int rcode;
  int sd;

  for (retries = 0; retries < CONFIG_NETDB_DNSCLIENT_RETRIES; retries++)
    {
      sd = dns_bind(addr->sa_family, false, retries);
      if (sd < 0)
        {
          rs->result = sd;
          return 0;
        }

      if (connect(sd, addr, addrlen) < 0 ||
          send(sd, rs->msg, rs->msglen, 0) < 0)
        {
          rs->result = -get_errno();
          close(sd);
          return 0;
        }

      /* Wait for the answer with the ID of the query.  Ignore other
       * messages.  A timeout sends the query again.
       */

      for (; ; )
        {
          nrecv = recv(sd, rs->answer, rs->anslen, 0);
          if (nrecv < 0)
            {
              rs->result = -get_errno();
              break;
            }

          if (nrecv >= NS_HFIXEDSZ &&
              (rs->answer[2] & 0x80) != 0 &&
              memcmp(rs->answer, rs->msg, 2) == 0)
            {
              break;
            }
        }

      close(sd);

      if (nrecv >= 0)
        {
          rs->result = nrecv;

          /* After a server failure or a refusal, try the next server */

          rcode = rs->answer[3] & 0x0f;
          return rcode == ns_r_servfail || rcode == ns_r_refused ? 0 : 1;
        }

      if (rs->result != -EAGAIN)
        {
          break;
        }
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: res_send
 *
 * Description:
 *   Send the DNS message msg to the configured name servers over UDP and
 *   copy the answer into the buffer answer.  A server that fails or
 *   refuses the query is skipped.  An answer that does not fit the buffer
 *   is truncated.
 *
 * Input Parameters:
 *   msg    - The query message, for example from res_mkquery().
 *   msglen - The size of the query message.
 *   answer - The buffer that receives the answer.
 *   anslen - The size of answer.
 *
 * Returned Value:
 *   The size of the answer on success.  On failure, -1 is returned and
 *   errno is set.
 *
 ****************************************************************************/

int res_send(FAR const unsigned char *msg, int msglen,
             FAR unsigned char *answer, int anslen)
{
  struct res_send_s rs;
  int ret;

  if (msg == NULL || msglen < NS_HFIXEDSZ || answer == NULL ||
      anslen < NS_HFIXEDSZ)
    {
      set_errno(EINVAL);
      return -1;
    }

  rs.msg    = msg;
  rs.msglen = msglen;
  rs.answer = answer;
  rs.anslen = anslen;
  rs.result = -EADDRNOTAVAIL;

  ret = dns_foreach_nameserver(res_send_callback, &rs);
  if (ret < 0)
    {
      rs.result = ret;
    }

  if (rs.result < 0)
    {
      set_errno(-rs.result);
      return -1;
    }

  return rs.result;
}
