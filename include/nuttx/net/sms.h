/****************************************************************************
 * include/nuttx/net/sms.h
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

#ifndef __INCLUDE_NUTTX_NET_SMS_H
#define __INCLUDE_NUTTX_NET_SMS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SMS_MAX_ADDRLEN   16   /* 16 characters in USC2 format: 32byte */
#define SMS_MAX_DATALEN   670  /* 670 characters in USC2 format: 1340byte */

#define SMS_MSG_TYPE_DELIVER       0
#define SMS_MSG_TYPE_STATUS_REPORT 1

#define SMS_STATUS_SUCCESS 0
#define SMS_STATUS_FAILED  1
#define SMS_STATUS_PENDING 2

#define SMS_CHSET_UCS2     0
#define SMS_CHSET_GSM7     1
#define SMS_CHSET_BINARY   2

#define SMS_CONCATENATE_MAX 10

/* Nature of Address Indicator */

#define SMS_NAI_UNKNOWN              0x0 /* Unknown */
#define SMS_NAI_INTERNATIONAL        0x1 /* International number */
#define SMS_NAI_NATIONAL             0x2 /* National number */
#define SMS_NAI_NETWORK_SPEC         0x3 /* Network specific number */
#define SMS_NAI_SUBSCRIBER           0x4 /* Subscriber number */
#define SMS_NAI_ALPANUMERIC          0x5 /* Alphanumeric */
#define SMS_NAI_ABBREVIATED          0x6 /* Abbreviated number */
#define SMS_NAI_RESERVED             0x7 /* Reserved for extension */

/* Numbering Plan Indicator */

#define SMS_NPI_UNKNOWN              0x0 /* Unknown */
#define SMS_NPI_ISDN                 0x1 /* ISDN/telephone numbering plan */
#define SMS_NPI_DATA                 0x3 /* Data numbering plan */
#define SMS_NPI_TELEX                0x4 /* Telex numbering plan */
#define SMS_NPI_SERVICE_CENTRE_SPEC  0x5 /* Service Centre Specific plan 1) */
#define SMS_NPI_SERVICE_CENTRE_SPEC2 0x6 /* Service Centre Specific plan 1) */
#define SMS_NPI_NATIONAL             0x8 /* National numbering plan */
#define SMS_NPI_PRIVATE              0x9 /* Private numbering plan */
#define SMS_NPI_ERMES                0xa /* ERMES numbering plan */
#define SMS_NPI_RESERVED             0xf /* Reserved for extension */

#define SMS_NAI_SHIFT(a)             ((a) << 4)
#define SMS_SET_TOA(a, p)            (SMS_NAI_SHIFT(a) | (p))

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct sms_timestamp_s
{
  unsigned char year;   /* Years (0-99) */
  unsigned char mon;    /* Month (1-12) */
  unsigned char mday;   /* Day of the month (1-31) */
  unsigned char hour;   /* Hours (0-23) */
  unsigned char min;    /* Minutes (0-59) */
  unsigned char sec;    /* Seconds (0-59) */
  signed char tz;       /* Time zone (-24-24) */
};

/* This is the data structure used when sending SMS */

struct sms_send_msg_header_s
{
  unsigned char destaddrlen;
  unsigned char reserve;
  unsigned short datalen;
  unsigned short destaddr[SMS_MAX_ADDRLEN];
};

struct sms_send_msg_s
{
  struct sms_send_msg_header_s header;

  unsigned short data[0]; /* payload data in USC2 format */
};

/* This is the data structure used when receiving SMS */

struct sms_recv_msg_header_s
{
  unsigned char msgtype;
  struct sms_timestamp_s send_time;
  unsigned char srcaddrlen;
  unsigned char reserve;
  unsigned short datalen;
  unsigned short srcaddr[SMS_MAX_ADDRLEN];
};

struct sms_deliver_msg_s
{
  struct sms_recv_msg_header_s header;

  unsigned short data[0]; /* payload data in USC2 format */
};

struct sms_deliver_msg_max_s
{
  struct sms_recv_msg_header_s header;

  unsigned short data[SMS_MAX_DATALEN]; /* payload data in USC2 format */
};

struct sms_status_report_s
{
  unsigned char refid;
  unsigned char status;
  unsigned char reserve;
  struct sms_timestamp_s discharge_time;
};

struct sms_status_report_msg_s
{
  struct sms_recv_msg_header_s header;

  struct sms_status_report_s status_report;
};

struct sms_refids_s
{
  unsigned char nrefid;
  unsigned char refid[SMS_CONCATENATE_MAX];
};

struct sms_sc_addr_s
{
  unsigned char toa;
  unsigned char addrlen;
  unsigned short addr[SMS_MAX_ADDRLEN];
};

#endif /* __INCLUDE_NUTTX_NET_SMS_H */
