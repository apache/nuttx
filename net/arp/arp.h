/****************************************************************************
 * net/arp/arp.h
 *
 *   Copyright (C) 2014-2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __NET_ARP_ARP_H
#define __NET_ARP_ARP_H

/* The Address Resolution Protocol ARP is used for mapping between IP
 * addresses and link level addresses such as the Ethernet MAC
 * addresses. ARP uses broadcast queries to ask for the link level
 * address of a known IP address and the host which is configured with
 * the IP address for which the query was meant, will respond with its
 * link level address.
 *
 * Note: This ARP implementation only supports Ethernet.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <semaphore.h>

#include <netinet/in.h>

#include <nuttx/net/netdev.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifndef CONFIG_DEBUG
#  undef CONFIG_NET_ARP_DUMP
#endif

#ifndef CONFIG_ARP_SEND_MAXTRIES
#  define CONFIG_ARP_SEND_MAXTRIES 5
#endif

#ifndef CONFIG_ARP_SEND_DELAYMSEC
#  define CONFIG_ARP_SEND_DELAYMSEC 20
#endif

/* ARP Definitions **********************************************************/

#define ARP_REQUEST    1
#define ARP_REPLY      2

#define ARP_HWTYPE_ETH 1

#define RASIZE         4  /* Size of ROUTER ALERT */

/* Allocate a new ARP data callback */

#define arp_callback_alloc(conn)   devif_callback_alloc(&(conn)->list)
#define arp_callback_free(conn,cb) devif_callback_free(cb, &(conn)->list)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* ARP header -- Size 28 bytes */

struct arp_hdr_s
{
  uint16_t ah_hwtype;        /* 16-bit Hardware type (Ethernet=0x001) */
  uint16_t ah_protocol;      /* 16-bit Protocol type (IP=0x0800) */
  uint8_t  ah_hwlen;         /*  8-bit Hardware address size (6) */
  uint8_t  ah_protolen;      /*  8-bit Procotol address size (4) */
  uint16_t ah_opcode;        /* 16-bit Operation */
  uint8_t  ah_shwaddr[6];    /* 48-bit Sender hardware address */
  uint16_t ah_sipaddr[2];    /* 32-bit Sender IP adress */
  uint8_t  ah_dhwaddr[6];    /* 48-bit Target hardware address */
  uint16_t ah_dipaddr[2];    /* 32-bit Target IP address */
};

/* IP header -- Size 20 or 24 bytes */

struct arp_iphdr_s
{
  uint8_t  eh_vhl;           /*  8-bit Version (4) and header length (5 or 6) */
  uint8_t  eh_tos;           /*  8-bit Type of service (e.g., 6=TCP) */
  uint8_t  eh_len[2];        /* 16-bit Total length */
  uint8_t  eh_ipid[2];       /* 16-bit Identification */
  uint8_t  eh_ipoffset[2];   /* 16-bit IP flags + fragment offset */
  uint8_t  eh_ttl;           /*  8-bit Time to Live */
  uint8_t  eh_proto;         /*  8-bit Protocol */
  uint16_t eh_ipchksum;      /* 16-bit Header checksum */
  uint16_t eh_srcipaddr[2];  /* 32-bit Source IP address */
  uint16_t eh_destipaddr[2]; /* 32-bit Destination IP address */
  uint16_t eh_ipoption[2];   /* (optional) */
};

#ifdef CONFIG_NET_ARP_SEND
/* This structure holds the state of the send operation until it can be
 * operated upon from the interrupt level.
 */

struct arp_send_s
{
  FAR struct devif_callback_s *snd_cb; /* Reference to callback instance */
  sem_t     snd_sem;                   /* Used to wake up the waiting thread */
  uint8_t   snd_retries;               /* Retry count */
  volatile bool snd_sent;              /* True: if request sent */
#ifdef CONFIG_NETDEV_MULTINIC
  uint8_t   snd_ifname[IFNAMSIZ];      /* Interface name */
#endif
  in_addr_t snd_ipaddr;                /* The IP address to be queried */
};
#endif

#ifdef CONFIG_NET_ARP_SEND
/* For symmetry with other protocols, a "connection" structure is
 * provided.  But it is a singleton for the case of ARP packet transfers.
 */

struct arp_conn_s
{
  FAR struct devif_callback_s *list;   /* ARP callbacks */
};
#endif

#ifdef CONFIG_NET_ARP_SEND
/* Used to notify a thread waiting for a particular ARP response */

struct arp_notify_s
{
  FAR struct arp_notify_s *nt_flink;   /* Supports singly linked list */
  in_addr_t nt_ipaddr;                 /* Waited for IP address in the mapping */
  sem_t     nt_sem;                    /* Will wake up the waiter */
  int       nt_result;                 /* The result of the wait */
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_NET_ARP_SEND
/* This is the singleton "connection" structure */

extern struct arp_conn_s g_arp_conn;
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

 #ifdef CONFIG_NET_ARP
/****************************************************************************
 * Name: arp_reset
 *
 * Description:
 *   Re-initialize the ARP table.
 *
 ****************************************************************************/

void arp_reset(void);

/****************************************************************************
 * Function: arp_timer_initialize
 *
 * Description:
 *   Initialized the 10 second timer that is need by the ARP logic in order
 *   to age ARP address associations
 *
 * Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called once at system initialization time
 *
 ****************************************************************************/

void arp_timer_initialize(void);

/****************************************************************************
 * Name: arp_timer
 *
 * Description:
 *   This function performs periodic timer processing in the ARP module
 *   and should be called at regular intervals.  The recommended interval
 *   is 10 seconds between the calls.  It is responsible for flushing old
 *   entries in the ARP table.
 *
 ****************************************************************************/

void arp_timer(void);

/****************************************************************************
 * Name: arp_format
 *
 * Description:
 *   Format an ARP packet.
 *
 * Input Parameters:
 *   dev    - Device structure
 *   ipaddr - Target IP address (32-bit)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

struct net_driver_s; /* Forward reference */
void arp_format(FAR struct net_driver_s *dev, in_addr_t ipaddr);

/****************************************************************************
 * Function: arp_send
 *
 * Description:
 *   The arp_send() call may be to send an ARP request to resolve an IPv4
 *   address.  This function first checks if the IPv4 address is already in
 *   the ARP table.  If so, then it returns success immediately.
 *
 *   If the requested IPv4 address in not in the ARP table, then this function
 *   will send an ARP request, delay, then check if the IP address is now in
 *   the ARP table.  It will repeat this sequence until either (1) the IP
 *   address mapping is now in the ARP table, or (2) a configurable number
 *   of timeouts occur without receiving the ARP replay.
 *
 * Parameters:
 *   ipaddr   The IP address to be queried.
 *
 * Returned Value:
 *   Zero (OK) is returned on success and the IP address mapping can now be
 *   found in the ARP table.  On error a negated errno value is returned:
 *
 *     -ETIMEDOUT:    The number or retry counts has been exceed.
 *     -EHOSTUNREACH: Could not find a route to the host
 *
 * Assumptions:
 *   This function is called from the normal tasking context.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ARP_SEND
int arp_send(in_addr_t ipaddr);
#else
#  define arp_send(i) (0)
#endif

/****************************************************************************
 * Function: arp_poll
 *
 * Description:
 *   Poll all pending transfer for ARP requests to send.
 *
 * Assumptions:
 *   This function is called from the MAC device driver indirectly through
 *   devif_poll() and devif_timer() and may be called from the timer
 *   interrupt/watchdog handler level.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ARP_SEND
int arp_poll(FAR struct net_driver_s *dev, devif_poll_callback_t callback);
#else
#  define arp_poll(d,c) (0)
#endif

/****************************************************************************
 * Function: arp_wait_setup
 *
 * Description:
 *   Called BEFORE an ARP request is sent.  This function sets up the ARP
 *   response timeout before the the ARP request is sent so that there is
 *   no race condition when arp_wait() is called.
 *
 * Assumptions:
 *   This function is called from ARP send and executes in the normal
 *   tasking environment.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ARP_SEND
void arp_wait_setup(in_addr_t ipaddr, FAR struct arp_notify_s *notify);
#else
#  define arp_wait_setup(i,n)
#endif

/****************************************************************************
 * Function: arp_wait_cancel
 *
 * Description:
 *   Cancel any wait set after arp_wait_setup is called but before arm_wait()
 *   is called (arp_wait() will automatically cancel the wait).
 *
 * Assumptions:
 *   This function may execute in the interrupt context when called from
 *   arp_wait().
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ARP_SEND
int arp_wait_cancel(FAR struct arp_notify_s *notify);
#else
#  define arp_wait_cancel(n) (0)
#endif

/****************************************************************************
 * Function: arp_wait
 *
 * Description:
 *   Called each time that a ARP request is sent.  This function will sleep
 *   until either: (1) the matching ARP response is received, or (2) a
 *   timeout occurs.
 *
 * Assumptions:
 *   This function is called from ARP send and mut execute with the network
 *   un-locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ARP_SEND
struct timespec;
int arp_wait(FAR struct arp_notify_s *notify, FAR struct timespec *timeout);
#else
#  define arp_wait(n,t) (0)
#endif

/****************************************************************************
 * Function: arp_notify
 *
 * Description:
 *   Called each time that a ARP response is received in order to wake-up
 *   any threads that may be waiting for this particular ARP response.
 *
 * Assumptions:
 *   This function is called from the MAC device driver indirectly through
 *   arp_arpin() and will execute with the network locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ARP_SEND
void arp_notify(in_addr_t ipaddr);
#else
#  define arp_notify(i)
#endif

/****************************************************************************
 * Name: arp_find
 *
 * Description:
 *   Find the ARP entry corresponding to this IP address.
 *
 * Input parameters:
 *   ipaddr - Refers to an IP address in network order
 *
 * Assumptions
 *   Interrupts are disabled; Returned value will become unstable when
 *   interrupts are re-enabled or if any other uIP APIs are called.
 *
 ****************************************************************************/

FAR struct arp_entry *arp_find(in_addr_t ipaddr);

/****************************************************************************
 * Name: arp_delete
 *
 * Description:
 *   Remove an IP association from the ARP table
 *
 * Input parameters:
 *   ipaddr - Refers to an IP address in network order
 *
 * Assumptions
 *   Interrupts are disabled to assure exclusive access to the ARP table
 *   (and because arp_find() is called).
 *
 ****************************************************************************/

#define arp_delete(ipaddr) \
{ \
  struct arp_entry *tabptr = arp_find(ipaddr); \
  if (tabptr) \
    { \
      tabptr->at_ipaddr = 0; \
    } \
}

/****************************************************************************
 * Name: arp_update
 *
 * Description:
 *   Add the IP/HW address mapping to the ARP table -OR- change the IP
 *   address of an existing association.
 *
 * Input parameters:
 *   pipaddr - Refers to an IP address uint16_t[2] in network order
 *   ethaddr - Refers to a HW address uint8_t[IFHWADDRLEN]
 *
 * Assumptions
 *   Interrupts are disabled to assure exclusive access to the ARP table.
 *
 ****************************************************************************/

void arp_update(FAR uint16_t *pipaddr, FAR uint8_t *ethaddr);

/****************************************************************************
 * Name: arp_dump
 *
 * Description:
 *   Dump the contents of an ARP packet to the SYSLOG device
 *
 * Input Parameters:
 *   arp - A reference to the ARP header to be dumped.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ARP_DUMP
void arp_dump(FAR struct arp_hdr_s *arp);
#else
#  define arp_dump(arp)
#endif

#else /* CONFIG_NET_ARP */

/* If ARP is disabled, stub out all ARP interfaces */

#  define arp_reset()
#  define arp_timer_initialize()
#  define arp_timer()
#  define arp_format(d,i);
#  define arp_send(i) (0)
#  define arp_poll(d,c) (0)
#  define arp_wait_setup(i,n)
#  define arp_wait_cancel(n) (0)
#  define arp_wait(n,t) (0)
#  define arp_notify(i)
#  define arp_find(i) (NULL)
#  define arp_delete(i)
#  define arp_update(i,m);
#  define arp_dump(arp)

#endif /* CONFIG_NET_ARP */
#endif /* __NET_ARP_ARP_H */
