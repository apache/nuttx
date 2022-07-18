/****************************************************************************
 * include/net/if.h
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

#ifndef __INCLUDE_NET_IF_H
#define __INCLUDE_NET_IF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <signal.h>
#include <sys/socket.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If CONFIG_NETDEV_IFINDEX is enabled then there is limit to the number of
 * devices that can be registered due to the nature of some static data.
 */

#define MAX_IFINDEX        32

/* Sizing parameters */

#define IFNAMSIZ           16  /* Older naming standard */
#define IF_NAMESIZE        16  /* Newer naming standard */
#define IFHWADDRLEN        6

/* Interface flag bits */

#define IFF_DOWN           (1 << 0)  /* Interface is down */
#define IFF_UP             (1 << 1)  /* Interface is up */
#define IFF_RUNNING        (1 << 2)  /* Carrier is available */
#define IFF_IPv6           (1 << 3)  /* Configured for IPv6 packet (vs ARP or IPv4) */
#define IFF_BOUND          (1 << 4)  /* Bound to a socket */
#define IFF_LOOPBACK       (1 << 5)  /* Is a loopback net */
#define IFF_POINTOPOINT    (1 << 6)  /* Is point-to-point link */
#define IFF_NOARP          (1 << 7)  /* ARP is not required for this packet */
#define IFF_MULTICAST      (1 << 12) /* Supports multicast. */
#define IFF_BROADCAST      (1 << 13) /* Broadcast address valid. */

/* Interface flag helpers */

#define IFF_SET_UP(f)          do { (f) |= IFF_UP; } while (0)
#define IFF_SET_RUNNING(f)     do { (f) |= IFF_RUNNING; } while (0)
#define IFF_SET_BOUND(f)       do { (f) |= IFF_BOUND; } while (0)
#define IFF_SET_NOARP(f)       do { (f) |= IFF_NOARP; } while (0)
#define IFF_SET_LOOPBACK(f)    do { (f) |= IFF_LOOPBACK; } while (0)
#define IFF_SET_POINTOPOINT(f) do { (f) |= IFF_POINTOPOINT; } while (0)
#define IFF_SET_MULTICAST(f)   do { (f) |= IFF_MULTICAST; } while (0)
#define IFF_SET_BROADCAST(f)   do { (f) |= IFF_BROADCAST; } while (0)

#define IFF_CLR_UP(f)          do { (f) &= ~IFF_UP; } while (0)
#define IFF_CLR_RUNNING(f)     do { (f) &= ~IFF_RUNNING; } while (0)
#define IFF_CLR_BOUND(f)       do { (f) &= ~IFF_BOUND; } while (0)
#define IFF_CLR_NOARP(f)       do { (f) &= ~IFF_NOARP; } while (0)
#define IFF_CLR_LOOPBACK(f)    do { (f) &= ~IFF_LOOPBACK; } while (0)
#define IFF_CLR_POINTOPOINT(f) do { (f) &= ~IFF_POINTOPOINT; } while (0)
#define IFF_CLR_MULTICAST(f)   do { (f) &= ~IFF_MULTICAST; } while (0)
#define IFF_CLR_BROADCAST(f)   do { (f) &= ~IFF_BROADCAST; } while (0)

#define IFF_IS_UP(f)          (((f) & IFF_UP) != 0)
#define IFF_IS_RUNNING(f)     (((f) & IFF_RUNNING) != 0)
#define IFF_IS_BOUND(f)       (((f) & IFF_BOUND) != 0)
#define IFF_IS_NOARP(f)       (((f) & IFF_NOARP) != 0)
#define IFF_IS_LOOPBACK(f)    (((f) & IFF_LOOPBACK) != 0)
#define IFF_IS_POINTOPOINT(f) (((f) & IFF_POINTOPOINT) != 0)
#define IFF_IS_MULTICAST(f)   (((f) & IFF_MULTICAST) != 0)
#define IFF_IS_BROADCAST(f)   (((f) & IFF_BROADCAST) != 0)

/* We only need to manage the IPv6 bit if both IPv6 and IPv4 are supported.
 * Otherwise, we can save a few bytes by ignoring it.
 */

#if defined(CONFIG_NET_IPv4) && defined(CONFIG_NET_IPv6)
#  define IFF_SET_IPv6(f)  do { (f) |= IFF_IPv6; } while (0)
#  define IFF_CLR_IPv6(f)  do { (f) &= ~IFF_IPv6; } while (0)
#  define IFF_IS_IPv6(f)   (((f) & IFF_IPv6) != 0)

#  define IFF_SET_IPv4(f)  IFF_CLR_IPv6(f)
#  define IFF_CLR_IPv4(f)  IFF_SET_IPv6(f)
#  define IFF_IS_IPv4(f)   (!IFF_IS_IPv6(f))

#elif defined(CONFIG_NET_IPv6)
#  define IFF_SET_IPv6(f)
#  define IFF_CLR_IPv6(f)
#  define IFF_IS_IPv6(f)   (1)

#  define IFF_SET_IPv4(f)
#  define IFF_CLR_IPv4(f)
#  define IFF_IS_IPv4(f)   (0)

#else /* if defined(CONFIG_NET_IPv4) */
#  define IFF_SET_IPv6(f)
#  define IFF_CLR_IPv6(f)
#  define IFF_IS_IPv6(f)   (0)

#  define IFF_SET_IPv4(f)
#  define IFF_CLR_IPv4(f)
#  define IFF_IS_IPv4(f)   (1)
#endif

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

struct if_nameindex
{
  unsigned int if_index; /* 1, 2, ... */
  FAR char *if_name;     /* null terminated name: "eth0", ... */
};

/* Structure passed with the SIOCMIINOTIFY ioctl command to enable
 * notification of of PHY state changes.
 */

struct mii_ioctl_notify_s
{
  pid_t pid;             /* PID of the task to receive the signal.  Zero means "this task" */
  struct sigevent event; /* Describe the way a task is to be notified */
};

/* Structure passed to read from or write to the MII/PHY management
 * interface via the SIOCxMIIREG ioctl commands.
 */

struct mii_ioctl_data_s
{
  uint16_t phy_id;      /* PHY device address */
  uint16_t reg_num;     /* PHY register address */
  uint16_t val_in;      /* PHY input data */
  uint16_t val_out;     /* PHY output data */
};

/* Structure passed to get or set the CAN bitrate
 * SIOCxCANBITRATE ioctl commands.
 */

struct can_ioctl_data_s
{
  uint16_t arbi_bitrate; /* Classic CAN / Arbitration phase bitrate kbit/s */
  uint16_t arbi_samplep; /* Classic CAN / Arbitration phase input % */
  uint16_t data_bitrate; /* Data phase bitrate kbit/s */
  uint16_t data_samplep; /* Data phase sample point % */
};

/* Structure passed to add or remove hardware-level CAN ID filters
 * SIOCxCANSTDFILTER / SIOCxCANEXTFILTER ioctl commands.
 */

struct can_ioctl_filter_s
{
  uint32_t fid1;  /* 11- or 29-bit ID (context dependent).  For dual match or
                   * for the lower address in a range of addresses  */
  uint32_t fid2;  /* 11- or 29-bit ID.  For dual match, address mask or for
                   * upper address in address range  */
  uint8_t  ftype; /* See CAN_FILTER_* definitions */
  uint8_t  fprio; /* See CAN_MSGPRIO_* definitions */
};

/* There are two forms of the I/F request structure.
 * One for IPv6 and one for IPv4.
 * Notice that they are (and must be) cast compatible and really different
 * only in the size of the structure allocation.
 *
 * This is the I/F request that should be used with IPv6.
 */

struct lifreq
{
  char                         lifr_name[IFNAMSIZ];  /* Network device name (e.g. "eth0") */
  int16_t                      lifr_ifindex;         /* Interface index */
  union
  {
    struct sockaddr_storage    lifru_addr;           /* IP Address */
    struct sockaddr_storage    lifru_dstaddr;        /* P-to-P Address */
    struct sockaddr_storage    lifru_broadaddr;      /* Broadcast address */
    struct sockaddr_storage    lifru_netmask;        /* Netmask */
    struct sockaddr            lifru_hwaddr;         /* MAC address */
    int                        lifru_count;          /* Number of devices */
    int                        lifru_mtu;            /* MTU size */
    uint32_t                   lifru_flags;          /* Interface flags */
    struct mii_ioctl_notify_s  llfru_mii_notify;     /* PHY event notification */
    struct mii_ioctl_data_s    lifru_mii_data;       /* MII request data */
    struct can_ioctl_data_s    lifru_can_data;       /* CAN bitrate request data */
    struct can_ioctl_filter_s  lifru_can_filter;     /* CAN filter request data */
  } lifr_ifru;
};

#define lifr_addr             lifr_ifru.lifru_addr             /* IP address */
#define lifr_dstaddr          lifr_ifru.lifru_dstaddr          /* P-to-P Address */
#define lifr_broadaddr        lifr_ifru.lifru_broadaddr        /* Broadcast address */
#define lifr_netmask          lifr_ifru.lifru_netmask          /* Interface net mask */
#define lifr_hwaddr           lifr_ifru.lifru_hwaddr           /* MAC address */
#define lifr_mtu              lifr_ifru.lifru_mtu              /* MTU */
#define lifr_count            lifr_ifru.lifru_count            /* Number of devices */
#define lifr_flags            lifr_ifru.lifru_flags            /* interface flags */
#define lifr_mii_notify_pid   lifr_ifru.llfru_mii_notify.pid   /* PID to be notified */
#define lifr_mii_notify_event lifr_ifru.llfru_mii_notify.event /* Describes notification */
#define lifr_mii_phy_id       lifr_ifru.lifru_mii_data.phy_id  /* PHY device address */
#define lifr_mii_reg_num      lifr_ifru.lifru_mii_data.reg_num /* PHY register address */
#define lifr_mii_val_in       lifr_ifru.lifru_mii_data.val_in  /* PHY input data */
#define lifr_mii_val_out      lifr_ifru.lifru_mii_data.val_out /* PHY output data */

/* Used only with the SIOCGLIFCONF IOCTL command */

struct lifconf
{
  size_t                      lifc_len;              /* Size of buffer */
  union
  {
    FAR char                 *lifcu_buf;             /* Buffer address */
    FAR struct lifreq        *lifcu_req;             /* Array of ifreq structures */
  } lifc_ifcu;
};

#define lifc_buf              lifc_ifcu.lifcu_buf    /* Buffer address */
#define lifc_req              lifc_ifcu.lifcu_req    /* Array of ifreq structures */

/* This is the I/F request that should be used with IPv4. */

struct ifreq
{
  char                         ifr_name[IFNAMSIZ];  /* Network device name (e.g. "eth0") */
  int16_t                      ifr_ifindex;         /* Interface index */
  union
  {
    struct sockaddr            ifru_addr;           /* IP Address */
    struct sockaddr            ifru_dstaddr;        /* P-to-P Address */
    struct sockaddr            ifru_broadaddr;      /* Broadcast address */
    struct sockaddr            ifru_netmask;        /* Netmask */
    struct sockaddr            ifru_hwaddr;         /* MAC address */
    int                        ifru_count;          /* Number of devices */
    int                        ifru_mtu;            /* MTU size */
    uint32_t                   ifru_flags;          /* Interface flags */
    struct mii_ioctl_notify_s  ifru_mii_notify;     /* PHY event notification */
    struct mii_ioctl_data_s    ifru_mii_data;       /* MII request data */
    struct can_ioctl_data_s    ifru_can_data;       /* CAN bitrate request data */
    struct can_ioctl_filter_s  ifru_can_filter;     /* CAN filter request data */
  } ifr_ifru;
};

#define ifr_addr              ifr_ifru.ifru_addr             /* IP address */
#define ifr_dstaddr           ifr_ifru.ifru_dstaddr          /* P-to-P Address */
#define ifr_broadaddr         ifr_ifru.ifru_broadaddr        /* Broadcast address */
#define ifr_netmask           ifr_ifru.ifru_netmask          /* Interface net mask */
#define ifr_hwaddr            ifr_ifru.ifru_hwaddr           /* MAC address */
#define ifr_mtu               ifr_ifru.ifru_mtu              /* MTU */
#define ifr_count             ifr_ifru.ifru_count            /* Number of devices */
#define ifr_flags             ifr_ifru.ifru_flags            /* interface flags */
#define ifr_mii_notify_pid    ifr_ifru.ifru_mii_notify.pid   /* PID to be notified */
#define ifr_mii_notify_event  ifr_ifru.ifru_mii_notify.event /* Describes notification */
#define ifr_mii_phy_id        ifr_ifru.ifru_mii_data.phy_id  /* PHY device address */
#define ifr_mii_reg_num       ifr_ifru.ifru_mii_data.reg_num /* PHY register address */
#define ifr_mii_val_in        ifr_ifru.ifru_mii_data.val_in  /* PHY input data */
#define ifr_mii_val_out       ifr_ifru.ifru_mii_data.val_out /* PHY output data */

/* Used only with the SIOCGIFCONF IOCTL command */

struct ifconf
{
  size_t                      ifc_len;                  /* Size of buffer */
  union
  {
    FAR char                 *ifcu_buf;                 /* Buffer address */
    FAR struct ifreq         *ifcu_req;                 /* Array of ifreq structures */
  } ifc_ifcu;
};

#define ifc_buf              ifc_ifcu.ifcu_buf          /* Buffer address */
#define ifc_req              ifc_ifcu.ifcu_req          /* Array of ifreq structures */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: if_nametoindex
 *
 * Description:
 *   The if_nametoindex() function returns the interface index corresponding
 *  to name ifname.
 *
 * Input Parameters:
 *   ifname - The interface name
 *
 * Returned Value:
 *   The corresponding index if ifname is the name of an interface;
 *   otherwise, zero.
 *
 ****************************************************************************/

unsigned int if_nametoindex(FAR const char *ifname);

/****************************************************************************
 * Name: if_indextoname
 *
 * Description:
 *   The if_indextoname() function maps an interface index to its
 *   corresponding name.
 *
 * Input Parameters:
 *   ifname  - Points to a buffer of at least IF_NAMESIZE bytes.
 *             if_indextoname() will place in this buffer the name
 *              of the interface with index ifindex.
 *
 * Returned Value:
 *   If ifindex is an interface index, then the function will return the
 *   value supplied by ifname.
 *   Otherwise, the function returns a NULL pointer and sets errno to
 *   indicate the error.
 *
 ****************************************************************************/

FAR char *if_indextoname(unsigned int ifindex, FAR char *ifname);

/****************************************************************************
 * Name: if_nameindex
 *
 * Description:
 *   The if_nameindex() function returns an array of if_nameindex structures,
 *   each containing information about one of the network interfaces on the
 *   local system. The if_nameindex structure contains at least the following
 *   entries:
 *         unsigned int if_index;
 *         FAR char     *if_name;
 *   The if_index field contains the interface index. The if_name field
 *   points to the null-terminated interface name.  The end of the array
 *   is indicated by entry with if_index set to zero and if_name set to NULL.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success, if_nameindex() returns pointer to the array; on error, NULL
 *   is returned, and errno is set to indicate the error.
 *
 ****************************************************************************/

FAR struct if_nameindex *if_nameindex(void);

/****************************************************************************
 * Name: if_freenameindex
 *
 * Description:
 *   The if_freenameindex() function free the data structure returned by
 *   if_nameindex().
 *
 * Input Parameters:
 *   ifn - The data structure to free
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void if_freenameindex(FAR struct if_nameindex *ifn);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NET_IF_H */
