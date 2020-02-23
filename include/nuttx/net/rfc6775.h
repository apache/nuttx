/****************************************************************************
 * include/nuttx/net/rfc6775.h
 * Definitions for 6LoWPAN Neighbor Discovery
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Reference: RFC6775, Neighbor Discovery Optimization for IPv6 over Low-
 *            Power Wireless Personal Area Networks (6LoWPANs), Internet
 *            Engineering Task Force (IETF), November 2012
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

#ifndef __INCLUDE_NUTTX_NET_RFC6775_H
#define __INCLUDE_NUTTX_NET_RFC6775_H

/* Acronyms (in addition to those defined in context below:
 *
 * 6LN  - 6LoWPAN Node
 * 6LR  - 6LoWPAN Router
 * 6LBR - 6LoWPAN Border Router
 */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* 4.2 Bit definitions for 6CO resccid field */

#define RESCCID_CID_SHIFT             (0)       /* Bits 0-3: Context identifier */
#define RESCCID_CID_MASK              (7 << RESCCID_CID_SHIFT)
#define RESCCID_C                     (1 << 4)  /* Bit 4: Compression Flag */
                                                /* Bits 5-7: Reserved */

/* 9 Protocol Constants */

/* 6LBR Constants: */

#define MIN_CONTEXT_CHANGE_DELAY      300       /* Seconds */

/* 6LR Constants: */

#define MAX_RTR_ADVERTISEMENTS        3         /* Transmissions */
#define MIN_DELAY_BETWEEN_RAS         10        /* Seconds */
#define MAX_RA_DELAY_TIME             2         /* Seconds */
#define TENTATIVE_NCE_LIFETIME        20        /* Seconds */

/* Router Constants: */

#define MULTIHOP_HOPLIMI              64

/* Host Constants: */

#define RTR_SOLICITATION_INTERVAL     10        /* Seconds */
#define MAX_RTR_SOLICITATIONS         3         /* Transmissions */
#define MAX_RTR_SOLICITATION_INTERVAL 60        /* Seconds */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Table 1. Values for status field */

enum sixlowpan_status_e
{
  SIXLOWPAN_SUCCESS   = 0,  /* Success */
  SIXLOWPAN_DUPLICATE = 1,  /* Duplicate address */
  SIXLOWPAN_FULL      = 2,  /* Neighbor cache full */
  SIXLOWPAN_ALLOCATED = 3   /* 3-255: Allocated using standard action (RFC5226 */
};

/* 4.1 Source Link-Layer Address Option (SLLAO) */

struct sixlowpan_sllao_s
{
  uint8_t type;        /* Byte 0:      Type = 33 */
  uint8_t length;      /* Byte 1:      Length in units of 8 bytes = 2 */
  uint8_t status;      /* Byte 2:      Status of NS message.  See enum sixlowpan_status_e */
  uint8_t reserved[3]; /* Bytes 3-5:   Reserved */
  uint8_t lifetime[2]; /* Bytes 6-7:   Registration lifetime */
  uint8_t eui64[8];    /* Bytes 8-15:  EUI-64 identifier assigned to interface */
};

/* 4.2 6LoWPAN Context Option (6CO) */

struct sixlowpan_6co_s
{
  uint8_t type;        /* Byte 0:      Type = 34 */
  uint8_t length;      /* Byte 1:      Length in units of 8 bytes = 2 or 3 */
  uint8_t ctxlen;      /* Byte 2:      Context length = 0-128 bits */
  uint8_t resccid;     /* Byte 3:      See RESCCID_* bit definitions */
  uint8_t reserved[2]; /* Bytes 4-6:   Reserved */
  uint8_t lifetime[2]; /* Bytes 6-7:   Valid lifetime */
  uint8_t prefix[1];   /* Bytes 8-15 or 8-23: Context Prefix */
};

#define SIXLOWPAN_6CO_LEN(b)      ((b > 64) ? 16 : 8)
#define SIZEOF_SIXLOWPAN_6CO_S(b) (sizeof(struct sixlowpan_6co_s) + CMPV6_6CO_LEN(b) - 1)

/* 4.3 Authoritative Border Router Option (ABRO) */

struct sixlowpan_abro_s
{
  uint8_t type;        /* Byte 0:      Type = 35 */
  uint8_t length;      /* Byte 1:      Length in units of 8 bytes = 3 */
  uint8_t verlo[2];    /* Bytes 2-3:   Version low */
  uint8_t verhi[2];    /* Bytes 4-5:   Version high */
  uint8_t lifetime[2]; /* Bytes 6-7:   Valid lifetime */
  uint8_t ipv6[16];    /* Bytes 8-23:  IPv6 address of the 6LBR origin of version */
};

/* 4.4 Duplicate Address Messages (DAD for both DAR and DAC) */

struct sixlowpan_dad_s
{
  uint8_t type;        /* Byte 0:      Type = 157 (DAR) or 158 (DAC) */
  uint8_t code;        /* Byte 1:      Code */
  uint8_t chksum[2];   /* Bytes 2-3:   Checksum */
  uint8_t status;      /* Byte 4:      Status of DAR.  See enum sixlowpan_status_e */
  uint8_t reserved;    /* Byte 5:      Reserved */
  uint8_t lifetime[2]; /* Bytes 6-7:   Registration lifetime */
  uint8_t eui64[8];    /* Bytes 8-15:  EUI-64 identifier registered address */
  uint8_t ipv6[16];    /* Bytes 16-31: Registered address */
};

#endif /* __INCLUDE_NUTTX_NET_RFC6775_H */
