/****************************************************************************
 * netuils/tftp/tftpc_get.c
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, TFTP_DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Compilation Switches
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/stat.h>

#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <net/uip/uipopt.h>
#include <net/uip/uip.h>
#include <net/uip/tftp.h>

#include "tftpc_internal.h"

#if defined(CONFIG_NET) && defined(CONFIG_NET_UDP) && CONFIG_NFILE_DESCRIPTORS > 0

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define TFTP_RETRIES 3

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tftp_write
 ****************************************************************************/

static inline ssize_t tftp_write(int fd, const ubyte *buf, size_t len)
{
  size_t left = len;
  ssize_t nbyteswritten;

  while (left > 0)
    {
      /* Write the data... repeating the write in the event that it was
       * interrupted by a signal.
       */

      do
        {
          nbyteswritten = write(fd, buf, left);
        }
      while (nbyteswritten < 0 && errno == EINTR);

      /* Check for non-EINTR errors */

      if (nbyteswritten < 0)
        {
          ndbg("write failed: %d\n", errno);
          return ERROR;
        }

      /* Handle partial writes */

      left -= nbyteswritten;
      buf  += nbyteswritten;
    }
  return len;
}

/****************************************************************************
 * Name: tftp_parsedatapacket
 ****************************************************************************/

static inline int tftp_parsedatapacket(const ubyte *packet,
                                       uint16 *opcode, uint16 *blockno)
{
  *opcode = (uint16)packet[0] << 8 | (uint16)packet[1];
  if (*opcode == TFTP_DATA)
    {
       *blockno = (uint16)packet[2] << 8 | (uint16)packet[3];
       return OK;
    }
#if defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_NET)
  else if (*opcode == TFTP_ERR)
    {
      (void)tftp_parseerrpacket(packet);
    }
#endif
  return ERROR;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tftpget
 *
 * Input Parameters:
 *   remote - The name of the file on the TFTP server.
 *   local  - Path to the location on a mounted filesystem where the file
 *            will be stored.
 *   addr   - The IP address of the server in network order
 *   binary - TRUE:  Perform binary ('octect') transfer
 *            FALSE: Perform text ('netascii') transfer
 *
 ****************************************************************************/

int tftpget(const char *remote, const char *local, in_addr_t addr, boolean binary)
{
  struct sockaddr_in server;  /* The address of the TFTP server */
  struct sockaddr_in from;    /* The address the last UDP message recv'd from */
  ubyte *packet;              /* Allocated memory to hold one packet */
  uint16 blockno = 0;         /* The current transfer block number */
  uint16 opcode;              /* Received opcode */
  uint16 rblockno;            /* Received block number */
  int len;                    /* Generic length */
  int sd;                     /* Socket descriptor for socket I/O */
  int fd;                     /* File descriptor for file I/O */
  int retry;                  /* Retry counter */
  int nbytesrecvd;            /* The number of bytes received in the packet */
  int ndatabytes;             /* The number of data bytes received */
  int result = ERROR;         /* Assume failure */
  int ret;                    /* Generic return status */

#if CONFIG_NETUTILS_TFTP_ACKPACKETS > 1
  uint16 lastacked = 0;       /* The last block number that was ACK'ed */
  int ablockno;               /* Number of un-ACKed packets */
#endif

  /* Allocate the buffer to used for socket/disk I/O */

  packet = (ubyte*)zalloc(TFTP_IOBUFSIZE);
  if (!packet)
    {
      ndbg("packet memory allocation failure\n");
      errno = ENOMEM;
      goto errout;
    }

  /* Open the file for writing */

  fd = open(local, O_WRONLY|O_CREAT|O_TRUNC, 0666);
  if (fd < 0)
    {
      ndbg("open failed: %d\n", errno);
      goto errout_with_packet;
   }

  /* Initialize a UDP socket and setup the server addresss */

  sd = tftp_sockinit(&server, addr);
  if (sd < 0)
    {
      goto errout_with_fd;
    }

  /* Send the read request using the well-known port number */

  len = tftp_mkreqpacket(packet, TFTP_RRQ, remote, binary);
  ret = tftp_sendto(sd, packet, len, &server);
  if (ret != len)
    {
      goto errout_with_sd;
    }

  /* Subsequent sendto will use the port number selected by the TFTP
   * server.  Setting the server port to zero indicates that we have
   * not yet received the server port number.
   */

  server.sin_port = 0;

  /* Then enter the transfer loop.  Loop until the entire file has
   * been received or until an error occurs.
   */

#if CONFIG_NETUTILS_TFTP_ACKPACKETS > 1
  ablockno = CONFIG_NETUTILS_TFTP_ACKPACKETS-1;
#endif
  do
    {
      /* Increment the TFTP block number for the next transfer */

      blockno++;

      /* Send the next block if the file within a loop.  We will
       * retry up to TFTP_RETRIES times before giving up on the
       * transfer.
       */

      for (retry = 0; retry < TFTP_RETRIES; retry++)
        {
          /* Get the next packet from the server */

          nbytesrecvd = tftp_recvfrom(sd, packet, TFTP_IOBUFSIZE, &from);

          /* Check if anything valid was received */

          if (nbytesrecvd >= 0)
            {
              /* Verify the sender address and port number */

              if (server.sin_addr.s_addr != from.sin_addr.s_addr)
                {
                  nvdbg("Invalid address in DATA\n");
                  retry--;
                  continue;
                }

              if (server.sin_port && server.sin_port != from.sin_port)
                {
                  nvdbg("Invalid port in DATA\n");
                  len = tftp_mkerrpacket(packet, TFTP_ERR_UNKID, TFTP_ERRST_UNKID);
                  ret = tftp_sendto(sd, packet, len, &from);
                  retry--;
                  continue;
                }

              /* Parse the incoming DATA packet */

              if (nbytesrecvd < TFTP_DATAHEADERSIZE ||
                  tftp_parsedatapacket(packet, &opcode, &rblockno) != OK ||
                  blockno != rblockno)
                {
                  nvdbg("Parse failure\n");
                  if (opcode > TFTP_MAXRFC1350)
                    {
                      len = tftp_mkerrpacket(packet, TFTP_ERR_ILLEGALOP, TFTP_ERRST_ILLEGALOP);
                      ret = tftp_sendto(sd, packet, len, &from);
                    }
                  continue;
                }

              /* Replace the server port to the one in the good data response */

              if (!server.sin_port)
                {
                  server.sin_port = from.sin_port;
                }

              /* Then break out of the loop */

              break;
            }
        }

      /* Did we exhaust all of the retries? */

      if (retry == TFTP_RETRIES)
        {
          nvdbg("Retry limit exceeded\n");
          goto errout_with_sd;
        }

      /* Write the received data chunk to the file */

      ndatabytes = nbytesrecvd - TFTP_DATAHEADERSIZE;
      if (tftp_write(fd, packet + TFTP_DATAHEADERSIZE, ndatabytes) < 0)
        {
          goto errout_with_sd;
        }

      /* Send the acknowledgment if we have reach the configured block count */

#if CONFIG_NETUTILS_TFTP_ACKPACKETS > 1
      ablockno++;
      if (ablockno == CONFIG_NETUTILS_TFTP_ACKPACKETS)
#endif
        {
          len = tftp_mkackpacket(packet, blockno);
          ret = tftp_sendto(sd, packet, len, &server);
          if (ret != len)
            {
              goto errout_with_sd;
            }
#if CONFIG_NETUTILS_TFTP_ACKPACKETS > 1
          lastacked = blockno;
#endif
          nvdbg("ACK blockno %d\n", blockno);
        }
    }
  while (ndatabytes >= TFTP_DATASIZE);

  /* The final packet of the transfer will be a partial packet
   *
   * If the final packet(s) were not ACK'ed, then we will ACK them here
   */

#if CONFIG_NETUTILS_TFTP_ACKPACKETS > 1
  if (ndatabytes < TFTP_DATASIZE && blockno != lastacked)
    {
      len = tftp_mkackpacket(packet, blockno);
      ret = tftp_sendto(sd, packet, len, &server);
      if (ret != len)
        {
          goto errout_with_sd;
        }
      nvdbg("ACK blockno %d\n", blockno);
    }
#endif

  /* Return success */

  result = OK;

errout_with_sd:
  close(sd);
errout_with_fd:
  close(fd);
errout_with_packet:
  free(packet);
errout:
  return result;
}

#endif /* CONFIG_NET && CONFIG_NET_UDP && CONFIG_NFILE_DESCRIPTORS > 0 */
