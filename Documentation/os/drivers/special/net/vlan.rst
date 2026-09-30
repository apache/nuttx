===================
Vlan Device Drivers
===================

-  ``include/nuttx/net/vlan.h``. All structures and APIs
   needed to work with Vlan drivers are provided in this
   header file.
-  we also create VLAN devices for managing VLAN, which will become
   interfaces like ``eth0.58``
-  QinQ is also supported, we can create VLAN devices above another VLAN
   devices, like ``eth0.100.101`` (or even ``eth0.1.2.3.4``, also supported
   on Linux).
-  Supporting ADD_VLAN_CMD and DEL_VLAN_CMD of SIOCSIFVLAN.
-  We add default PCP because some of our apps may not want to set
   PCP manually
-  ``include/nuttx/net/ethernet.h``. Some definitions for 802.1Q VLAN

   .. code-block:: c

      #define VLAN_PRIO_MASK   0xe000 /* Priority Code Point */
      #define VLAN_PRIO_SHIFT  13
      #define VLAN_CFI_MASK    0x1000 /* Canonical Format Indicator / Drop Eligible Indicator */
      #define VLAN_VID_MASK    0x0fff /* VLAN Identifier */
      #define VLAN_N_VID       4096

-  **Driver**: ``drivers/net/vlan.c``

Configuration Options
=====================

``CONFIG_NET_VLAN``
  Enable 802.1Q VLAN interface support.
``CONFIG_NET_VLAN_COUNT``
  Maximum number of VLAN interfaces per physical Ethernet interface.

Usage
=====

.. code-block:: c

  #include <nuttx/net/vlan.h>
  #include "netutils/netlib.h"

  /* Create a VLAN interface (eth0.100, VLAN ID 100) */

  FAR const char *ifname = "eth0";
  uint16_t vlanid = 100;
  uint8_t priority = 0; /* Default PCP */

  int sockfd = socket(NET_SOCK_FAMILY, NET_SOCK_TYPE, NET_SOCK_PROTOCOL);

  if (sockfd >= 0)
   {
      struct vlan_ioctl_args ifv;

      strncpy(ifv.vlan_devname, ifname, sizeof(ifv.device1));
      ifv.u.VID  = vlanid;
      ifv.vlan_qos = priority;
      ifv.cmd = ADD_VLAN_CMD;
      if (ioctl(sockfd, SIOCSIFVLAN, (unsigned long)&ifv) < 0)
        {
          /* Handle error */
        }
      close(sockfd);
   }

  /* Enable the VLAN interface */

  netdev_ifup("eth0.100");

.. code-block:: c

  #include <nuttx/net/vlan.h>
  #include "netutils/netlib.h"

  /* Delete a VLAN interface (eth0.100, VLAN ID 100) */

  FAR const char *ifname = "eth0.100";

  int sockfd = socket(NET_SOCK_FAMILY, NET_SOCK_TYPE, NET_SOCK_PROTOCOL);

  if (sockfd >= 0)
   {
      struct vlan_ioctl_args ifv;

      strncpy(ifv.vlan_devname, ifname, sizeof(ifv.device1));
      ifv.cmd = DEL_VLAN_CMD;
      if (ioctl(sockfd, SIOCSIFVLAN, (unsigned long)&ifv) < 0)
        {
          /* Handle error */
        }
      close(sockfd);
   }