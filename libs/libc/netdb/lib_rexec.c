/****************************************************************************
 * libs/libc/netdb/lib_rexec.c
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

#include <netinet/in.h>

#include <errno.h>
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rexec_af
 *
 * Description:
 *   The rexec() function works over IPv4 (AF_INET). By contrast,
 *   the rexec_af() function provides an extra argument, af, that allows
 *   the caller to select the protocol. This argument can be specified
 *   as AF_INET, AF_INET6, or AF_UNSPEC (to allow the implementation to
 *   select the protocol).
 *
 * Input Parameters:
 *
 * Returned Value:
 *   OK on success; -1 on failure with the errno variable set appropriately.
 *
 ****************************************************************************/

int rexec_af(FAR char **ahost, int inport, FAR const char *user,
             FAR const char *passwd, FAR const char *cmd,
             FAR int *fd2p, sa_family_t af)
{
  FAR struct addrinfo *res;
  struct addrinfo hints;
  char port_str[12];
  int sock;
  int ret;

  if (!cmd || !ahost)
    {
      set_errno(EINVAL);
      return -1;
    }

  snprintf(port_str, sizeof(port_str), "%d", NTOHS(inport));

  memset(&hints, 0, sizeof(hints));
  hints.ai_family = af;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_flags = AI_CANONNAME;

  ret = getaddrinfo(*ahost, port_str, &hints, &res);
  if (ret < 0)
    {
      return -1;
    }

  if (res->ai_canonname)
    {
      *ahost = strdup(res->ai_canonname);
      if (*ahost == NULL)
        {
          set_errno(ENOMEM);
          goto addr_out;
        }
    }
  else
    {
      *ahost = NULL;
      set_errno(ENOENT);
      goto addr_out;
    }

  sock = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
  if (sock < 0)
    {
      goto sock_out;
    }

  if (connect(sock, res->ai_addr, res->ai_addrlen) < 0)
    {
      goto conn_out;
    }

  /* ignore second connection(fd2p always is NULL) */

  write(sock, "", 1);

  /* Send username */

  if (user)
    {
      ret = write(sock, user, strlen(user) + 1);
    }
  else
    {
      ret = write(sock, "", 1);
    }

  if (ret < 0)
    {
      goto conn_out;
    }

  /* Send passwd */

  if (passwd)
    {
      ret = write(sock, passwd, strlen(passwd) + 1);
    }
  else
    {
      ret = write(sock, "", 1);
    }

  if (ret < 0)
    {
      goto conn_out;
    }

  /* Send command */

  ret = write(sock, cmd, strlen(cmd) + 1);
  if (ret < 0)
    {
      goto conn_out;
    }

  freeaddrinfo(res);
  return sock;

conn_out:
  close(sock);
sock_out:
  free(*ahost);
addr_out:
  freeaddrinfo(res);
  return -1;
}

/****************************************************************************
 * Name: rexec
 *
 * Description:
 *   The rexec() function looks up the host *ahost using gethostbyname(),
 *   returning -1 if the host does not exist. Otherwise, *ahost is set to
 *   standard name of the host. If a username and password are both
 *   specified, then these are used to authenticate to the foreign host;
 *   otherwise the environment and then the .netrc file in user's home
 *   directory are searched for appropiate information. If all that fails,
 *   the user is prompted for the information.
 *
 *   The port inport specifies which well-known DARPA Internet port to
 *   use for the  connection; the call getservbyname("exec", "tcp") will
 *   return a pointer to a structure that contains the necessary port.
 *   The protocol for connection is described in  detail in rexecd(8).
 *
 *   If the connection succeeds, a socket in the Internet domain of type
 *   SOCK_STREAM is returned to the caller, and given to the remote
 *   command as stdin and stdout. If fd2p is nonzero, then an auxiliary
 *   channel to a control process will be setup, and a file descriptor for
 *   it will be placed in *fd2p. The control process will return diagnostic
 *   output from the command on this channel, and will also
 *   accept bytes on this channel as being UNIX signal numbers, to be
 *   forwarded to the process group of the command. The diagnostic
 *   information returned does not include remote authorization failure,
 *   as the secondary connection is set up after authorization has been
 *   verified. If fd2p is 0, then the stderr (unit2 of the remote command)
 *   will be made the same as the stdout and no provision is made for
 *   sending arbitrary signals to the remote process, although you may
 *   be able to get its  attention by using out-of-band data.
 *
 *   The rexec() function works over IPv4 (AF_INET).
 *
 * Input Parameters:
 *
 * Returned Value:
 *   OK on success; -1 on failure with the errno variable set appropriately.
 *
 ****************************************************************************/

int rexec(FAR char **ahost, int inport, FAR const char *user,
          FAR const char *passwd, FAR const char *cmd, FAR int *fd2p)
{
  return rexec_af(ahost, inport, user, passwd, cmd, fd2p, AF_INET);
}
