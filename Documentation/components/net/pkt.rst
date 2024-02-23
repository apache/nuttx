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

Usage
=====

.. code-block:: c

  struct sockaddr_ll addr;
  uint8_t buffer[BUFSIZE];
  int sd = socket(AF_PACKET, SOCK_RAW, 0); /* Create a packet socket */

  addr.sll_family = AF_PACKET;
  addr.sll_ifindex = if_nametoindex("eth0");
  bind(sd, (FAR struct sockaddr *)&addr, sizeof(addr)); /* Bind to device */

  recv(sd, buffer, sizeof(buffer), 0); /* read(sd, buffer, sizeof(buffer)); */
  send(sd, buffer, sizeof(buffer), 0); /* write(sd, buffer, sizeof(buffer)); */

  close(sd); /* Close the socket */
