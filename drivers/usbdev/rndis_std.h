/****************************************************************************
 * drivers/usbdev/rndis.c
 *
 *   Copyright (C) 2011-2017 Gregory Nutt. All rights reserved.
 *   Authors: Sakari Kapanen <sakari.m.kapanen@gmail.com>,
 *            Petteri Aimonen <jpa@git.mail.kapsi.fi>
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


#ifndef __NUTTX_DRIVERS_USBDEV_RNDIS_STD_H
#define __NUTTX_DRIVERS_USBDEV_RNDIS_STD_H

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/* Definitions for Microsoft RNDIS protocol.
 * Documentation:
 * https://docs.microsoft.com/en-us/windows-hardware/drivers/network/remote-ndis--rndis-2
 * https://winprotocoldoc.blob.core.windows.net/productionwindowsarchives/WinArchive/[MS-RNDIS].pdf
 */

#define RNDIS_MAJOR_VERSION                  1
#define RNDIS_MINOR_VERSION                  0

#define RNDIS_DEVICEFLAGS                    0x00000001
#define RNDIS_MEDIUM_802_3                   0x00000000

/* Message types */

#define RNDIS_MSG_COMPLETE                   0x80000000
#define RNDIS_PACKET_MSG                     0x00000001
#define RNDIS_INITIALIZE_MSG                 0x00000002
#define RNDIS_HALT_MSG                       0x00000003
#define RNDIS_QUERY_MSG                      0x00000004
#define RNDIS_SET_MSG                        0x00000005
#define RNDIS_RESET_MSG                      0x00000006
#define RNDIS_INDICATE_MSG                   0x00000007
#define RNDIS_KEEPALIVE_MSG                  0x00000008

/* Status codes */

#define RNDIS_STATUS_SUCCESS                 0x00000000
#define RNDIS_STATUS_FAILURE                 0xC0000001
#define RNDIS_STATUS_NOT_SUPPORTED           0xC00000BB

/* ObjectIDs for query commands */

#define RNDIS_OID_GEN_SUPPORTED_LIST         0x00010101
#define RNDIS_OID_GEN_HARDWARE_STATUS        0x00010102
#define RNDIS_OID_GEN_MEDIA_SUPPORTED        0x00010103
#define RNDIS_OID_GEN_MEDIA_IN_USE           0x00010104
#define RNDIS_OID_GEN_MAXIMUM_FRAME_SIZE     0x00010106
#define RNDIS_OID_GEN_LINK_SPEED             0x00010107
#define RNDIS_OID_GEN_TRANSMIT_BLOCK_SIZE    0x0001010A
#define RNDIS_OID_GEN_RECEIVE_BLOCK_SIZE     0x0001010B
#define RNDIS_OID_GEN_VENDOR_ID              0x0001010C
#define RNDIS_OID_GEN_VENDOR_DESCRIPTION     0x0001010D
#define RNDIS_OID_GEN_VENDOR_DRIVER_VERSION  0x00010116
#define RNDIS_OID_GEN_CURRENT_PACKET_FILTER  0x0001010E
#define RNDIS_OID_GEN_MAXIMUM_TOTAL_SIZE     0x00010111
#define RNDIS_OID_GEN_MEDIA_CONNECT_STATUS   0x00010114
#define RNDIS_OID_GEN_PHYSICAL_MEDIUM        0x00010202
#define RNDIS_OID_GEN_XMIT_OK                0x00020101
#define RNDIS_OID_GEN_RCV_OK                 0x00020102
#define RNDIS_OID_GEN_XMIT_ERROR             0x00020103
#define RNDIS_OID_GEN_RCV_ERROR              0x00020104
#define RNDIS_OID_GEN_RCV_NO_BUFFER          0x00020105
#define RNDIS_OID_802_3_PERMANENT_ADDRESS    0x01010101
#define RNDIS_OID_802_3_CURRENT_ADDRESS      0x01010102
#define RNDIS_OID_802_3_MULTICAST_LIST       0x01010103
#define RNDIS_OID_802_3_MAXIMUM_LIST_SIZE    0x01010104
#define RNDIS_OID_802_3_MAC_OPTIONS          0x01010105
#define RNDIS_OID_802_3_RCV_ERROR_ALIGNMENT  0x01020101
#define RNDIS_OID_802_3_XMIT_ONE_COLLISION   0x01020102
#define RNDIS_OID_802_3_XMIT_MORE_COLLISION  0x01020103

#define RNDIS_SEND_ENCAPSULATED_COMMAND 0x00
#define RNDIS_GET_ENCAPSULATED_RESPONSE 0x01

#define RNDIS_NOTIFICATION_RESPONSE_AVAILABLE 0x01

/****************************************************************************
 * Public types
 ****************************************************************************/

struct rndis_notification
{
  uint32_t notification;  /* Notification */
  uint32_t reserved;      /* Reserved */
};

struct rndis_command_header
{
  uint32_t msgtype;       /* MessageType */
  uint32_t msglen;        /* MessageLength */
  uint32_t reqid;         /* RequestID */
};

struct rndis_response_header
{
  uint32_t msgtype;       /* MessageType */
  uint32_t msglen;        /* MessageLength */
  uint32_t reqid;         /* RequestID */
  uint32_t status;        /* Status */
};

struct rndis_initialize_msg
{
  struct rndis_command_header hdr;
  uint32_t major;         /* MajorVersion */
  uint32_t minor;         /* MinorVersion */
  uint32_t xfrsize;       /* MaxTransferSize */
};

struct rndis_initialize_cmplt
{
  struct rndis_response_header hdr;
  uint32_t major;         /* MajorVersion */
  uint32_t minor;         /* MinorVersion */
  uint32_t devflags;      /* DeviceFlags */
  uint32_t medium;        /* Medium */
  uint32_t pktperxfer;    /* MaxPacketsPerTransfer */
  uint32_t xfrsize;       /* MaxTransferSize */
  uint32_t pktalign;      /* PacketAlignmentFactor */
  uint32_t reserved1;     /* Reserved1 */
  uint32_t reserved2;     /* Reserved2 */
};

struct rndis_query_msg
{
  struct rndis_command_header hdr;
  uint32_t objid;         /* ObjectID */
  uint32_t buflen;        /* InformationBufferLength */
  uint32_t bufoffset;     /* InformationBufferOffset */
  uint32_t reserved;      /* Reserved */
  uint32_t buffer[];      /* Buffer */
};

struct rndis_query_cmplt
{
  struct rndis_response_header hdr;
  uint32_t buflen;        /* InformationBufferLength */
  uint32_t bufoffset;     /* InformationBufferOffset */
  uint32_t buffer[];      /* Buffer */
};

struct rndis_set_msg
{
  struct rndis_command_header hdr;
  uint32_t objid;         /* ObjectID */
  uint32_t buflen;        /* InformationBufferLength */
  uint32_t bufoffset;     /* InformationBufferOffset */
  uint32_t reserved;      /* Reserved */
  uint32_t buffer[];      /* Buffer */
};

struct rndis_reset_cmplt
{
  struct rndis_response_header hdr;
  uint32_t addreset;      /* AddressingReset */
};

struct rndis_indicate_msg
{
  uint32_t msgtype;       /* MessageType */
  uint32_t msglen;        /* MessageLength */
  uint32_t status;        /* Status */
  uint32_t buflen;        /* StatusBufferLength */
  uint32_t bufoffset;     /* StatusBufferOffset */
  uint32_t buffer[];      /* Buffer */
};

struct rndis_packet_msg
{
  uint32_t msgtype;      /* MessageType */
  uint32_t msglen;       /* MessageLength */
  uint32_t dataoffset;   /* DataOffset */
  uint32_t datalen;      /* DataLength */
  uint32_t ooboffset;    /* OutOfBandDataOffset */
  uint32_t ooblen;       /* OutOfBandDataLength */
  uint32_t noobelem;     /* NumOutOfBandDataElements */
  uint32_t infooffset;   /* PerPacketInfoOffset */
  uint32_t infolen;      /* PerPacketInfoLength */
};

#endif
