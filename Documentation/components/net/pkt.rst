===========================
"Raw" packet socket support
===========================

Packet sockets (:c:macro:`AF_PACKET`) allow receiving and transmitting frames
without a transport protocol in between. Frames received are copied into a
packet socket tap before they enter the network. Data written into a packet
socket will bypass the network altogether and be placed in the transmission
buffer of the network interface driver.

Configuration Options
=====================

``CONFIG_NET_PKT_PREALLOC_CONNS``
  Number of preallocated packet connections (all tasks).
``CONFIG_NET_PKT_ALLOC_CONNS``
  Dynamic memory allocations for packet connections.
``CONFIG_NET_PKT_MAX_CONNS``
  Maximum number of packet connections.
``NET_PKT_WRITE_BUFFERS``
  Use write buffers for packet sockets, support SOCK_NONBLOCK mode.

Usage
=====

.. code-block:: c

  struct sockaddr_ll addr;
  uint8_t buffer[BUFSIZE];
  int sd = socket(AF_PACKET, SOCK_RAW, 0); /* Create a Raw packet socket */

  addr.sll_family = AF_PACKET;
  addr.sll_ifindex = if_nametoindex("eth0");
  addr.sll_protocol = htons(ETH_P_ALL);
  bind(sd, (FAR struct sockaddr *)&addr, sizeof(addr)); /* Bind to device */

  recv(sd, buffer, sizeof(buffer), 0); /* read(sd, buffer, sizeof(buffer)); */
  send(sd, buffer, sizeof(buffer), 0); /* write(sd, buffer, sizeof(buffer)); */

  close(sd); /* Close the socket */

.. code-block:: c

  struct sockaddr_ll addr;
  uint8_t buffer[BUFSIZE];
  int sd = socket(AF_PACKET, SOCK_DGRAM, 0); /* Create a Dgram packet socket */

  addr.sll_family = AF_PACKET;
  addr.sll_ifindex = if_nametoindex("eth0");
  addr.sll_protocol = htons(ETH_P_IP);
  bind(sd, (FAR struct sockaddr *)&addr, sizeof(addr)); /* Bind to device */

  recv(sd, buffer, sizeof(buffer), 0); /* read(sd, buffer, sizeof(buffer)); */

  memset(addr.sll_addr, 0xff, sizeof(addr.sll_addr)); /* Destination MAC address */
  addr.sll_halen = ETH_ALEN;
  sendto(sd, buffer, sizeof(buffer), 0, /* SOCK_DGRAM can not use write() */
         (struct sockaddr *)&addr, sizeof(addr));

  close(sd); /* Close the socket */
