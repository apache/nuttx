/****************************************************************************
 * fs/nfs/rpc_clnt.c
 *
 *   Copyright (C) 2012-2013, 2018 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2012 Jose Pablo Rojas Vargas. All rights reserved.
 *   Author: Jose Pablo Rojas Vargas <jrojas@nx-engineering.com>
 *           Gregory Nutt <gnutt@nuttx.org>
 *
 * Leveraged from OpenBSD:
 *
 *   Copyright (c) 2004 The Regents of the University of Michigan.
 *   All rights reserved.
 *
 *   Copyright (c) 2004 Weston Andros Adamson <muzzle@umich.edu>.
 *   Copyright (c) 2004 Marius Aamodt Eriksen <marius@umich.edu>.
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *   Copyright (c) 1989, 1991, 1993, 1995 The Regents of the University of
 *   California.  All rights reserved.
 *
 * This code is derived from software contributed to Berkeley by Rick Macklem at
 * The University of Guelph.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer. 2.
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution. 3. All advertising
 * materials mentioning features or use of this software must display the
 * following acknowledgement: This product includes software developed by the
 * University of California, Berkeley and its contributors. 4. Neither the
 * name of the University nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific
 * prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/socket.h>
#include <sys/time.h>
#include <queue.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include <string.h>
#include <debug.h>

#include "xdr_subs.h"
#include "nfs_proto.h"
#include "rpc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Increment RPC statistics */

#ifdef CONFIG_NFS_STATISTICS
#  define rpc_statistics(n) do { rpcstats.n++; } while (0)
#else
#  define rpc_statistics(n)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Global RPC statistics */

#ifdef CONFIG_NFS_STATISTICS
struct rpcstats
{
  int rpcretries;
  int rpcrequests;
  int rpctimeouts;
  int rpcinvalid;
};
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Static data, mostly RPC constants in XDR form */

static uint32_t rpc_reply;
static uint32_t rpc_call;
static uint32_t rpc_vers;
static uint32_t rpc_auth_null;
static uint32_t rpc_auth_unix;

/* Global statics for all client instances.  Cleared by NuttX on boot-up. */

#ifdef CONFIG_NFS_STATISTICS
static struct rpcstats rpcstats;
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int rpcclnt_socket(FAR struct rpcclnt *rpc, in_port_t rport);
static int rpcclnt_send(FAR struct rpcclnt *rpc,
                        FAR void *call, int reqlen);
static int rpcclnt_receive(FAR struct rpcclnt *rpc,
                           FAR void *reply, size_t resplen);
static int rpcclnt_reply(FAR struct rpcclnt *rpc, uint32_t xid,
                         FAR void *reply, size_t resplen);
static void rpcclnt_fmtheader(FAR struct rpc_call_header *ch,
                              uint32_t xid, int procid, int prog, int vers);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rpcclnt_socket
 *
 * Description:
 *   Close(old), create, bind and connect the socket.
 *
 * Returned Value:
 *   Returns zero on success or a (negative) errno value on failure.
 *
 ****************************************************************************/

static int rpcclnt_socket(FAR struct rpcclnt *rpc, in_port_t rport)
{
  struct sockaddr_storage raddr;
  struct sockaddr_storage laddr;
  FAR in_port_t *lport;
  in_port_t port = 1024;
  struct timeval tv;
  socklen_t addrlen;
  int error;

  /* Close the old socket */

  psock_close(&rpc->rc_so);

  /* Prepare the socket address */

  memcpy(&raddr, rpc->rc_name, sizeof(raddr));

  laddr.ss_family = raddr.ss_family;
  memset(laddr.ss_data, 0, sizeof(laddr.ss_data));

  if (raddr.ss_family == AF_INET6)
    {
      FAR struct sockaddr_in6 *sin;

      addrlen = sizeof(struct sockaddr_in6);
      if (rport != 0)
        {
          sin = (FAR struct sockaddr_in6 *)&raddr;
          sin->sin6_port = htons(rport);
        }

      sin = (FAR struct sockaddr_in6 *)&laddr;
      lport = &sin->sin6_port;
    }
  else
    {
      FAR struct sockaddr_in *sin;

      addrlen = sizeof(struct sockaddr_in);
      if (rport != 0)
        {
          sin = (FAR struct sockaddr_in *)&raddr;
          sin->sin_port = htons(rport);
        }

      sin = (FAR struct sockaddr_in *)&laddr;
      lport = &sin->sin_port;
    }

  /* Create the socket */

  error = psock_socket(raddr.ss_family, rpc->rc_sotype, 0, &rpc->rc_so);
  if (error < 0)
    {
      ferr("ERROR: psock_socket failed: %d", error);
      return error;
    }

  /* Always set receive timeout to detect server crash and reconnect.
   * Otherwise, we can get stuck in psock_receive forever.
   */

  tv.tv_sec  = rpc->rc_timeo / 10;
  tv.tv_usec = (rpc->rc_timeo % 10) * 100000;

  error = psock_setsockopt(&rpc->rc_so, SOL_SOCKET, SO_RCVTIMEO,
                          (FAR const void *)&tv, sizeof(tv));
  if (error < 0)
    {
      ferr("ERROR: psock_setsockopt failed: %d\n", error);
      goto bad;
    }

  /* Some servers require that the client port be a reserved port
   * number. We always allocate a reserved port, as this prevents
   * filehandle disclosure through UDP port capture.
   */

  do
    {
      *lport = htons(--port);
      error = psock_bind(&rpc->rc_so, (FAR struct sockaddr *)&laddr, addrlen);
      if (error < 0)
        {
          ferr("ERROR: psock_bind failed: %d\n", error);
        }
    }
  while (error == -EADDRINUSE && port >= 512);

  if (error)
    {
      ferr("ERROR: psock_bind failed: %d\n", error);
      goto bad;
    }

  /* Protocols that do not require connections could be optionally left
   * unconnected.  That would allow servers to reply from a port other than
   * the NFS_PORT.
   */

  error = psock_connect(&rpc->rc_so, (FAR struct sockaddr *)&raddr, addrlen);
  if (error < 0)
    {
      ferr("ERROR: psock_connect to PMAP port failed: %d", error);
      goto bad;
    }

  return OK;

bad:
  psock_close(&rpc->rc_so);
  return error;
}

/****************************************************************************
 * Name: rpcclnt_send
 *
 * Description:
 *   This is the nfs send routine.
 *
 * Returned Value:
 *   Returns zero on success or a (negative) errno value on failure.
 *
 ****************************************************************************/

static int rpcclnt_send(FAR struct rpcclnt *rpc,
                        FAR void *call, int reqlen)
{
  uint32_t mark;
  int ret = OK;

  /* Send the record marking(RM) for stream only */

  if (rpc->rc_sotype == SOCK_STREAM)
    {
      mark = txdr_unsigned(0x80000000 | reqlen);
      ret = psock_send(&rpc->rc_so, &mark, sizeof(mark), 0);
      if (ret < 0)
        {
          ferr("ERROR: psock_send mark failed: %d\n", ret);
          return ret;
        }
    }

  /* Send the call message
   *
   * On success, psock_send returns the number of bytes sent;
   * On failure, it returns a negated errno value.
   */

  ret = psock_send(&rpc->rc_so, call, reqlen, 0);
  if (ret < 0)
    {
      ferr("ERROR: psock_send request failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: rpcclnt_receive
 *
 * Description:
 *   Receive a Sun RPC Request/Reply.
 *
 ****************************************************************************/

static int rpcclnt_receive(FAR struct rpcclnt *rpc,
                           FAR void *reply, size_t resplen)
{
  uint32_t mark;
  int error = 0;

  /* Receive the record marking(RM) for stream only */

  if (rpc->rc_sotype == SOCK_STREAM)
    {
      error = psock_recv(&rpc->rc_so, &mark, sizeof(mark), 0);
      if (error < 0)
        {
          ferr("ERROR: psock_recv mark failed: %d\n", error);
          return error;
        }

      /* Limit the receive length to the marked value */

      mark = fxdr_unsigned(uint32_t, mark);
      if (!(mark & 0x80000000))
        {
          return -ENOSYS;
        }

      mark &= 0x7fffffff;
      if (mark > resplen)
        {
          return -E2BIG;
        }

      resplen = mark;
    }

  error = psock_recv(&rpc->rc_so, reply, resplen, 0);
  if (error < 0)
    {
      ferr("ERROR: psock_recv response failed: %d\n", error);
      return error;
    }

  return OK;
}

/****************************************************************************
 * Name: rpcclnt_reply
 *
 * Description:
 *   Received the RPC reply on the socket.
 *
 ****************************************************************************/

static int rpcclnt_reply(FAR struct rpcclnt *rpc, uint32_t xid,
                         FAR void *reply, size_t resplen)
{
  int error;

retry:
  /* Get the next RPC reply from the socket */

  error = rpcclnt_receive(rpc, reply, resplen);
  if (error != 0)
    {
      ferr("ERROR: rpcclnt_receive returned: %d\n", error);
    }

  /* Get the xid and check that it is an RPC replysvr */

  else
    {
      FAR struct rpc_reply_header *replyheader =
        (FAR struct rpc_reply_header *)reply;

      if (replyheader->rp_direction != rpc_reply)
        {
          ferr("ERROR: Different RPC REPLY returned\n");
          rpc_statistics(rpcinvalid);
          error = -EPROTO;
        }
      else if (replyheader->rp_xid != txdr_unsigned(xid))
        {
          ferr("ERROR: Different RPC XID returned\n");
          rpc_statistics(rpcinvalid);
          goto retry;
        }
    }

  return error;
}

/****************************************************************************
 * Name: rpcclnt_fmtheader
 *
 * Description:
 *   Format the common part of the call header
 *
 ****************************************************************************/

static void rpcclnt_fmtheader(FAR struct rpc_call_header *ch,
                              uint32_t xid, int prog, int vers, int procid)
{
  /* Format the call header */

  ch->rp_xid            = txdr_unsigned(xid);
  ch->rp_direction      = rpc_call;
  ch->rp_rpcvers        = rpc_vers;
  ch->rp_prog           = txdr_unsigned(prog);
  ch->rp_vers           = txdr_unsigned(vers);
  ch->rp_proc           = txdr_unsigned(procid);

  /* rpc_auth part (auth_unix) */

  ch->rpc_auth.authtype = rpc_auth_unix;
  ch->rpc_auth.authlen  = txdr_unsigned(sizeof(ch->rpc_unix));

  ch->rpc_unix.stamp    = 0;
  ch->rpc_unix.hostname = 0;
  ch->rpc_unix.uid      = 0;
  ch->rpc_unix.gid      = 0;
  ch->rpc_unix.gidlist  = 0;

  /* rpc_verf part (auth_null) */

  ch->rpc_verf.authtype  = rpc_auth_null;
  ch->rpc_verf.authlen   = 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rpcclnt_init
 *
 * Description:
 *   Initialize the RPC client
 *
 ****************************************************************************/

void rpcclnt_init(void)
{
  /* RPC constants how about actually using more than one of these! */

  rpc_reply = txdr_unsigned(RPC_REPLY);
  rpc_vers = txdr_unsigned(RPC_VER2);
  rpc_call = txdr_unsigned(RPC_CALL);
  rpc_auth_null = txdr_unsigned(RPCAUTH_NULL);
  rpc_auth_unix = txdr_unsigned(RPCAUTH_UNIX);

  finfo("RPC initialized\n");
}

/****************************************************************************
 * Name: rpcclnt_connect
 *
 * Description:
 *   Initialize sockets for a new RPC connection.  We do not free the
 *   sockaddr if an error occurs.
 *
 ****************************************************************************/

int rpcclnt_connect(FAR struct rpcclnt *rpc)
{
  int error;
  int prot;

  union
  {
    struct rpc_call_pmap  sdata;
    struct rpc_call_mount mountd;
  } request;

  union
  {
    struct rpc_reply_pmap  rdata;
    struct rpc_reply_mount mdata;
  } response;

  finfo("Connecting\n");

  /* Create the socket */

  error = rpcclnt_socket(rpc, 0);
  if (error < 0)
    {
      ferr("ERROR: rpcclnt_socket failed: %d", error);
      return error;
    }

  prot = rpc->rc_sotype == SOCK_DGRAM ? IPPROTO_UDP : IPPROTO_TCP;

  /* Do the RPC to get a dynamic bounding with the server using ppmap.
   * Get port number for MOUNTD.
   */

  request.sdata.pmap.prog = txdr_unsigned(RPCPROG_MNT);
  request.sdata.pmap.vers = txdr_unsigned(RPCMNT_VER3);
  request.sdata.pmap.prot = txdr_unsigned(prot);
  request.sdata.pmap.port = 0;

  error = rpcclnt_request(rpc, PMAPPROC_GETPORT, PMAPPROG, PMAPVERS,
                          (FAR void *)&request.sdata, sizeof(struct call_args_pmap),
                          (FAR void *)&response.rdata, sizeof(struct rpc_reply_pmap));
  if (error != 0)
    {
      ferr("ERROR: rpcclnt_request failed: %d\n", error);
      goto bad;
    }

  error = rpcclnt_socket(rpc, fxdr_unsigned(uint32_t, response.rdata.pmap.port));
  if (error < 0)
    {
      ferr("ERROR: rpcclnt_socket MOUNTD port failed: %d\n", error);
      goto bad;
    }

  /* Do RPC to mountd. */

  strncpy(request.mountd.mount.rpath, rpc->rc_path, 90);
  request.mountd.mount.len = txdr_unsigned(sizeof(request.mountd.mount.rpath));

  error = rpcclnt_request(rpc, RPCMNT_MOUNT, RPCPROG_MNT, RPCMNT_VER3,
                          (FAR void *)&request.mountd,
                          sizeof(struct call_args_mount),
                          (FAR void *)&response.mdata,
                          sizeof(struct rpc_reply_mount));
  if (error != 0)
    {
      ferr("ERROR: rpcclnt_request failed: %d\n", error);
      goto bad;
    }

  error = -fxdr_unsigned(uint32_t, response.mdata.mount.status);
  if (error != 0)
    {
      ferr("ERROR: Bad mount status: %d\n", error);
      goto bad;
    }

  rpc->rc_fhsize = fxdr_unsigned(uint32_t, response.mdata.mount.fhandle.length);
  memcpy(&rpc->rc_fh, &response.mdata.mount.fhandle.handle, rpc->rc_fhsize);

  /* Do the RPC to get a dynamic bounding with the server using PMAP.
   * NFS port in the socket.
   */

  error = rpcclnt_socket(rpc, 0);
  if (error < 0)
    {
      ferr("ERROR: rpcclnt_socket PMAP port failed: %d\n", error);
      goto bad;
    }

  request.sdata.pmap.prog = txdr_unsigned(NFS_PROG);
  request.sdata.pmap.vers = txdr_unsigned(NFS_VER3);
  request.sdata.pmap.prot = txdr_unsigned(prot);
  request.sdata.pmap.port = 0;

  error = rpcclnt_request(rpc, PMAPPROC_GETPORT, PMAPPROG, PMAPVERS,
                          (FAR void *)&request.sdata,
                          sizeof(struct call_args_pmap),
                          (FAR void *)&response.rdata,
                          sizeof(struct rpc_reply_pmap));
  if (error != 0)
    {
      ferr("ERROR: rpcclnt_request failed: %d\n", error);
      goto bad;
    }

  error = rpcclnt_socket(rpc, fxdr_unsigned(uint32_t, response.rdata.pmap.port));
  if (error < 0)
    {
      ferr("ERROR: rpcclnt_socket NFS port returns %d\n", error);
      goto bad;
    }

  return OK;

bad:
  psock_close(&rpc->rc_so);
  return error;
}

/****************************************************************************
 * Name: rpcclnt_disconnect
 *
 * Description:
 *   Disconnect from the NFS server.
 *
 ****************************************************************************/

void rpcclnt_disconnect(FAR struct rpcclnt *rpc)
{
  union
  {
    struct rpc_call_pmap   sdata;
    struct rpc_call_umount mountd;
  } request;

  union
  {
    struct rpc_reply_pmap   rdata;
    struct rpc_reply_umount mdata;
  } response;

  int error;
  int prot;

  error = rpcclnt_socket(rpc, 0);
  if (error < 0)
    {
      ferr("ERROR: rpcclnt_socket failed: %d\n", error);
      goto bad;
    }

  prot = rpc->rc_sotype == SOCK_DGRAM ? IPPROTO_UDP : IPPROTO_TCP;

  request.sdata.pmap.prog = txdr_unsigned(RPCPROG_MNT);
  request.sdata.pmap.vers = txdr_unsigned(RPCMNT_VER3);
  request.sdata.pmap.prot = txdr_unsigned(prot);
  request.sdata.pmap.port = 0;

  error = rpcclnt_request(rpc, PMAPPROC_GETPORT, PMAPPROG, PMAPVERS,
                          (FAR void *)&request.sdata,
                          sizeof(struct call_args_pmap),
                          (FAR void *)&response.rdata,
                          sizeof(struct rpc_reply_pmap));
  if (error != 0)
    {
      ferr("ERROR: rpcclnt_request failed: %d\n", error);
      goto bad;
    }

  error = rpcclnt_socket(rpc, fxdr_unsigned(uint32_t, response.rdata.pmap.port));
  if (error < 0)
    {
      ferr("ERROR: rpcclnt_socket failed: %d\n", error);
      goto bad;
    }

  /* Do RPC to umountd. */

  strncpy(request.mountd.umount.rpath, rpc->rc_path, 90);
  request.mountd.umount.len = txdr_unsigned(sizeof(request.mountd.umount.rpath));

  error = rpcclnt_request(rpc, RPCMNT_UMOUNT, RPCPROG_MNT, RPCMNT_VER3,
                          (FAR void *)&request.mountd,
                          sizeof(struct call_args_umount),
                          (FAR void *)&response.mdata,
                          sizeof(struct rpc_reply_umount));
  if (error != 0)
    {
      ferr("ERROR: rpcclnt_request failed: %d\n", error);
      goto bad;
    }

bad:
  psock_close(&rpc->rc_so);
}

/****************************************************************************
 * Name: rpcclnt_request
 *
 * Description:
 *   Perform the RPC request.  Logic formats the RPC CALL message and calls
 *   rpcclnt_send to send the RPC CALL message.  It then calls rpcclnt_reply()
 *   to get the response.  It may attempt to re-send the CALL message on
 *   certain errors.
 *
 *   On successful receipt, it verifies the RPC level of the returned values.
 *   (There may still be be NFS layer errors that will be detected by calling
 *   logic).
 *
 ****************************************************************************/

int rpcclnt_request(FAR struct rpcclnt *rpc, int procnum, int prog,
                    int version, FAR void *request, size_t reqlen,
                    FAR void *response, size_t resplen)
{
  FAR struct rpc_reply_header *replymsg;
  uint32_t tmp;
  uint32_t xid;
  int retries = 0;
  int error = 0;

  /* Get a new (non-zero) xid */

  xid = ++rpc->rc_xid;

  /* Initialize the RPC header fields */

  rpcclnt_fmtheader((FAR struct rpc_call_header *)request,
                    xid, prog, version, procnum);

  /* Get the full size of the message (the size of variable data plus the size of
   * the messages header).
   */

  reqlen += sizeof(struct rpc_call_header);

  /* Send the RPC call messages and receive the RPC response.  A limited
   * number of re-tries will be attempted, but only for the case of response
   * timeouts.
   */

  for (; ; )
    {
      /* Do the client side RPC. */

      rpc_statistics(rpcrequests);

      /* Send the RPC CALL message */

      error = rpcclnt_send(rpc, request, reqlen);
      if (error != OK)
        {
          finfo("ERROR rpcclnt_send failed: %d\n", error);
        }

      /* Wait for the reply from our send */

      else
        {
          error = rpcclnt_reply(rpc, xid, response, resplen);
          if (error != OK)
            {
              finfo("ERROR rpcclnt_reply failed: %d\n", error);
            }
        }

      /* If we failed because of a timeout, then try sending the CALL
       * message again.
       */

      if (error != -EAGAIN && error != -ETIMEDOUT)
        {
          break;
        }

      rpc_statistics(rpctimeouts);
      if (++retries >= rpc->rc_retry)
        {
          break;
        }
      rpc_statistics(rpcretries);
    }

  if (error != OK)
    {
      ferr("ERROR: RPC failed: %d\n", error);
      return error;
    }

  /* Break down the RPC header and check if it is OK */

  replymsg = (FAR struct rpc_reply_header *)response;

  tmp = fxdr_unsigned(uint32_t, replymsg->type);
  if (tmp != RPC_MSGACCEPTED)
    {
      return -EOPNOTSUPP;
    }

  tmp = fxdr_unsigned(uint32_t, replymsg->status);
  if (tmp == RPC_SUCCESS)
    {
      finfo("RPC_SUCCESS\n");
    }
  else
    {
      ferr("ERROR: Unsupported RPC type: %d\n", tmp);
      return -EOPNOTSUPP;
    }

  return OK;
}
