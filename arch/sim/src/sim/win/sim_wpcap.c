/****************************************************************************
 * arch/sim/src/sim/win/sim_wpcap.c
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
 *
 * Based on code from uIP which also has a BSD-like license:
 *
 *   Copyright (c) 2007, Swedish Institute of Computer Science.
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#define WIN32_LEAN_AND_MEAN
#define _WIN32_WINNT 0x0501
#include <windows.h>
#include <winsock2.h>
#include <iphlpapi.h>

#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <syslog.h>

#include "sim_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_EXAMPLES_WEBSERVER_DHCPC
#  define WCAP_IPADDR (10 << 24 | 0 << 16 | 0 << 8 | 1)
#else
#  define WCAP_IPADDR (0)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This is normally prototyped in sim_internal.h.  However, sim_internal.h
 * cannot be included by this file do to collisions between BSD networking
 * definitions and Windows network definitions.
 */

void sim_netdriver_setmacaddr(int devidx, unsigned char *macaddr);

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct pcap;

struct pcap_if
{
  struct pcap_if *next;
  char *name;
  char *description;
  struct pcap_addr
  {
    struct pcap_addr *next;
    struct sockaddr *addr;
    struct sockaddr *netmask;
    struct sockaddr *broadaddr;
    struct sockaddr *dstaddr;
  }
  *addresses;
  DWORD flags;
};

struct pcap_pkthdr
{
  struct timeval ts;
  DWORD caplen;
  DWORD len;
};

/* DLL function types (for casting) */

typedef int (*pcap_findalldevs_t)(struct pcap_if **, char *);
typedef struct pcap *(*pcap_open_live_t)(char *, int, int, int, char *);
typedef int (*pcap_next_ex_t)(struct pcap *, struct pcap_pkthdr **,
                              unsigned char **);
typedef int (*pcap_sendpacket_t)(struct pcap *, unsigned char *, int);

/****************************************************************************
 * Private Data
 ****************************************************************************/

HMODULE wpcap;
static struct pcap *pcap;

static pcap_findalldevs_t pcap_findalldevs;
static pcap_open_live_t   pcap_open_live;
static pcap_next_ex_t     pcap_next_ex;
static pcap_sendpacket_t  pcap_sendpacket;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void error_exit(char *message)
{
  syslog(LOG_INFO, "error_exit: %s\n", message);
  exit(EXIT_FAILURE);
}

static void init_pcap(struct in_addr addr)
{
  struct pcap_if *interfaces;
  char error[256];

  if (pcap_findalldevs(&interfaces, error) == -1)
    {
      error_exit(error);
    }

  while (interfaces != NULL)
    {
      syslog(LOG_INFO, "init_pcap: found interface: %s\n",
              interfaces->description);

      if (interfaces->addresses != NULL &&
          interfaces->addresses->addr != NULL &&
          interfaces->addresses->addr->sa_family == AF_INET)
        {
          struct in_addr interface_addr;
          interface_addr =
            ((struct sockaddr_in *)interfaces->addresses->addr)->sin_addr;
          syslog(LOG_INFO, "init_pcap: with address: %s\n",
                 inet_ntoa(interface_addr));

          if (interface_addr.s_addr == addr.s_addr)
            {
              break;
            }
        }

      interfaces = interfaces->next;
    }

  if (interfaces == NULL)
    {
      error_exit("No interface found with IP address");
    }

  pcap = pcap_open_live(interfaces->name,
                        CONFIG_NET_ETH_PKTSIZE, 0, -1, error);
  if (pcap == NULL)
    {
      error_exit(error);
    }
}

static void set_ethaddr(struct in_addr addr)
{
  PIP_ADAPTER_ADDRESSES adapters;
  ULONG size = 0;

  if (GetAdaptersAddresses(AF_INET, GAA_FLAG_SKIP_ANYCAST |
                           GAA_FLAG_SKIP_MULTICAST |
                           GAA_FLAG_SKIP_DNS_SERVER,
                           NULL, NULL, &size) != ERROR_BUFFER_OVERFLOW)
    {
      error_exit("error on access to adapter list size");
    }

  adapters = alloca(size);

  if (GetAdaptersAddresses(AF_INET, GAA_FLAG_SKIP_ANYCAST |
                           GAA_FLAG_SKIP_MULTICAST |
                           GAA_FLAG_SKIP_DNS_SERVER,
                           NULL, adapters, &size) != ERROR_SUCCESS)
    {
      error_exit("error on access to adapter list");
    }

  while (adapters != NULL)
    {
      char buffer[256];
      WideCharToMultiByte(CP_ACP, 0, adapters->Description, -1,
                          buffer, sizeof(buffer), NULL, NULL);
      syslog(LOG_INFO, "set_ethaddr: found adapter: %s\n", buffer);

      if (adapters->FirstUnicastAddress != NULL &&
          adapters->FirstUnicastAddress->Address.lpSockaddr != NULL &&
          adapters->FirstUnicastAddress->Address.lpSockaddr->sa_family ==
          AF_INET)
        {
          struct in_addr adapter_addr;
          adapter_addr =
            ((struct sockaddr_in *)adapters->FirstUnicastAddress->Address.
             lpSockaddr)->sin_addr;
          syslog(LOG_INFO, "set_ethaddr: with address: %s\n",
                 inet_ntoa(adapter_addr));

          if (adapter_addr.s_addr == addr.s_addr)
            {
              if (adapters->PhysicalAddressLength != 6)
                {
                  error_exit("ip addr does not belong to an ethernet card");
                }

              syslog(LOG_INFO, "set_ethaddr:%02X-%02X-%02X-%02X-%02X-%02X\n",
                 adapters->PhysicalAddress[0], adapters->PhysicalAddress[1],
                 adapters->PhysicalAddress[2], adapters->PhysicalAddress[3],
                 adapters->PhysicalAddress[4], adapters->PhysicalAddress[5]);

              sim_netdriver_setmacaddr(0, adapters->PhysicalAddress);
              break;
            }
        }

      adapters = adapters->Next;
    }

  if (adapters == NULL)
    {
      error_exit("No adaptor found with IP address");
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void sim_wpcap_init(void *priv,
                void (*tx_done_intr_cb)(void *priv),
                void (*rx_ready_intr_cb)(void *priv))
{
  struct in_addr addr;
  FARPROC dlladdr;

  /* TODO: support emulation of TX done and RX ready interrupts */

  addr.s_addr = HTONL(WCAP_IPADDR);
  syslog(LOG_INFO, "sim_wpcap_init: IP address: %s\n", inet_ntoa(addr));

  wpcap = LoadLibrary("wpcap.dll");
  dlladdr = GetProcAddress(wpcap, "pcap_findalldevs");
  pcap_findalldevs = (pcap_findalldevs_t)dlladdr;

  dlladdr = GetProcAddress(wpcap, "pcap_open_live");
  pcap_open_live = (pcap_open_live_t)dlladdr;

  dlladdr = GetProcAddress(wpcap, "pcap_next_ex");
  pcap_next_ex = (pcap_next_ex_t)dlladdr;

  dlladdr = GetProcAddress(wpcap, "pcap_sendpacket");
  pcap_sendpacket = (pcap_sendpacket_t)dlladdr;

  if (pcap_findalldevs == NULL || pcap_open_live == NULL ||
      pcap_next_ex == NULL || pcap_sendpacket == NULL)
    {
      error_exit("error on access to winpcap library");
    }

  init_pcap(addr);
  set_ethaddr(addr);
}

unsigned int sim_wpcap_read(unsigned char *buf, unsigned int buflen)
{
  struct pcap_pkthdr *packet_header;
  unsigned char *packet;

  switch (pcap_next_ex(pcap, &packet_header, &packet))
    {
    case -1:
      error_exit("error on read");
    case 0:
      return 0;
    }

  if (packet_header->caplen > buflen)
    {
      return 0;
    }

  memcpy(buf, packet, packet_header->caplen);
  return packet_header->caplen;
}

void sim_wpcap_send(unsigned char *buf, unsigned int buflen)
{
  if (pcap_sendpacket(pcap, buf, buflen) == -1)
    {
      error_exit("error on send");
    }
}
