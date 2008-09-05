/****************************************************************************
 * netutils/tftoc/tftpc_internal.h
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
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __NETUTILS_TFTP_TFTPC_INTERNAL_H
#define __NETUTILS_TFTP_TFTPC_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <net/uip/uipopt.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Verify TFTP configuration settings ***************************************/
/* The settings beginning with CONFIG_NETUTILS_TFTP_* can all be set in the
 * NuttX configuration file.  If they are are defined in the configuration
 * then default values are assigned here.
 */

/* Number of packets before ACK is returned */

#ifndef CONFIG_NETUTILS_TFTP_ACKPACKETS
#  define CONFIG_NETUTILS_TFTP_ACKPACKETS 1
#endif

#define TFTP_MAXACKPACKETS 16
#if CONFIG_NETUTILS_TFTP_ACKPACKETS > TFTP_MAXACKPACKETS
#  error "CONFIG_NETUTILS_TFTP_ACKPACKETS exceeds maximum"
#endif

/* The TFTP port number (usually 69) */

#ifndef CONFIG_NETUTILS_TFTP_PORT
#  define CONFIG_NETUTILS_TFTP_PORT 69
#endif

/* recvfrom timeout in deci-seconds */

#ifndef CONFIG_NETUTILS_TFTP_TIMEOUT
#  define CONFIG_NETUTILS_TFTP_TIMEOUT 10 /* One second */
#endif

#define TFTP_ACKHEADERSIZE  4
#define TFTP_ERRHEADERSIZE  4
#define TFTP_DATAHEADERSIZE 4

/* The maximum size for TFTP data is determined by the configured uIP packet
 * size (but cannot exceed 512 + sizeof(TFTP_DATA header).
 */

#define TFTP_DATAHEADERSIZE 4
#define TFTP_MAXPACKETSIZE  (TFTP_DATAHEADERSIZE+512)

#if UIP_UDP_MSS < TFTP_MAXPACKETSIZE
#  define TFTP_PACKETSIZE   UIP_UDP_MSS
#  warning "uIP MSS is too small for TFTP"
#else
#  define TFTP_PACKETSIZE   TFTP_MAXPACKETSIZE
#endif

#define TFTP_DATASIZE      (TFTP_PACKETSIZE-TFTP_DATAHEADERSIZE)
#define TFTP_IOBUFSIZE     (TFTP_PACKETSIZE+8)

/* TFTP Opcodes *************************************************************/

#define TFTP_RRQ  1  /* Read Request          RFC 1350, RFC 2090 */
#define TFTP_WRQ  2  /* Write Request         RFC 1350 */
#define TFTP_DATA 3  /* Data chunk            RFC 1350 */
#define TFTP_ACK  4  /* Acknowledgement       RFC 1350 */
#define TFTP_ERR  5  /* Error Message         RFC 1350 */
#define TFTP_OACK 6  /* Option acknowledgment RFC 2347 */

#define TFTP_MAXRFC1350 5

/* TFTP Error Codes *********************************************************/

/* Error codes */

#define TFTP_ERR_NONE         0  /* No error */
#define TFTP_ERR_NOSUCHFILE   1  /* File not found */
#define TFTP_ERR_ACCESS       2  /* Access violation */
#define TFTP_ERR_FULL         3  /* Disk full or allocation exceeded */
#define TFTP_ERR_ILLEGALOP    4  /* Illegal TFTP operation */
#define TFTP_ERR_UNKID        5  /* Unknown transfer ID */
#define TFTP_ERR_EXISTS       6  /* File already exists */
#define TFTP_ERR_UNKUSER      7  /* No such user */
#define TFTP_ERR_NEGOTIATE    8  /* Terminate transfer due to option negotiation */

/* Error strings */

#define TFTP_ERR_STNOSUCHFILE "File not found"
#define TFTP_ERRST_ACCESS     "Access violation"
#define TFTP_ERRST_FULL       "Disk full or allocation exceeded"
#define TFTP_ERRST_ILLEGALOP  "Illegal TFTP operation"
#define TFTP_ERRST_UNKID      "Unknown transfer ID"
#define TFTP_ERRST_EXISTS     "File already exists"
#define TFTP_ERRST_UNKUSER    "No such user"
#define TFTP_ERRST_NEGOTIATE  "Terminate transfer due to option negotiation"

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#if CONFIG_DEBUG
extern const char g_tftpcallfailed[];
extern const char g_tftpcalltimedout[];
extern const char g_tftpnomemory[];
extern const char g_tftptoomanyretries[];
extern const char g_tftpaddress[];
extern const char g_tftpport[];
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Defined in tftp_packet.c *************************************************/

extern int tftp_sockinit(struct sockaddr_in *server, in_addr_t addr);
extern int tftp_mkreqpacket(ubyte *buffer, int opcode, const char *path, boolean binary);
extern int tftp_mkackpacket(ubyte *buffer, uint16 blockno);
extern int tftp_mkerrpacket(ubyte *buffer, uint16 errorcode, const char *errormsg);
#if CONFIG_DEBUG
extern int tftp_parseerrpacket(const ubyte *packet);
#endif

extern ssize_t tftp_recvfrom(int sd, void *buf, size_t len, struct sockaddr_in *from);
extern ssize_t tftp_sendto(int sd, const void *buf, size_t len, struct sockaddr_in *to);

#endif /* __NETUTILS_TFTP_TFTPC_INTERNAL_H */
