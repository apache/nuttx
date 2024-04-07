.. _netdriver:

===============
Network Drivers
===============

The NuttX network driver is split into two parts:

#. An "upper half", generic driver that provides the common network
   interface to application level code, and
#. A "lower half", platform-specific driver that implements the
   low-level timer controls to implement the network functionality.

Files supporting network driver can be found in the following locations:

-  **Interface Definition**. The header file for the NuttX network
   driver resides at ``include/nuttx/net/netdev_lowerhalf.h``. This
   header file includes the interface between the "upper half" and
   "lower half" drivers.
-  **"Upper Half" Driver**. The generic, "upper half" network driver
   resides at ``drivers/net/netdev_upperhalf.c``.
-  **"Lower Half" Drivers**. Platform-specific network drivers reside
   in ``arch/<architecture>/src/<hardware>`` or ``drivers/net``
   directory for the specific processor ``<architecture>`` and for
   the specific ``<chip>`` network peripheral devices.

**Special Note**: Not all network drivers are implemented with this
architecture.  Known lower-half drivers:
``arch/sim/src/sim/sim_netdriver.c``, ``drivers/virtio/virtio-net.c``

How to change full network driver into lower-half one
=====================================================

We have many network drivers that are implemented as full network drivers
with ``include/nuttx/net/netdev.h``, we can change them into lower-half
drivers to remove the common code (which is already in upper-half driver).
Here is a guide to do so:

1.  Change ``struct net_driver_s`` to ``struct netdev_lowerhalf_s`` in
    the network driver structure.  If you really need to touch some fields
    inside ``struct net_driver_s`` like MAC address, you can access them
    through ``struct netdev_lowerhalf_s::netdev``.
2.  Change the function names called in the network driver file to the names
    with prefix ``netdev_lower_``, e.g. ``netdev_lower_register`` and
    ``netdev_lower_carrier_on``.
3.  Change the core functions called by work queue like ``txpoll`` as
    ``transmit`` and ``receive`` in the ``netdev_ops_s`` structure.  You may
    need to change ``memcpy`` for ``d_buf`` into ``netpkt_copyin`` and
    ``netpkt_copyout``.

    -  Note that the ``receive`` function just need to return the received
       packet instead of calling functions like ``ipv4_input`` or doing reply.
       The upper-half will call ``receive`` to get all packets until it
       returns ``NULL`` and send these packets into the network stack.
    -  Also remember to call ``netpkt_free`` for the transmitted packets.

4.  Remove work queues related to send and receive, and replace them
    with calling ``netdev_lower_txdone`` and ``netdev_lower_rxready``.
    Then the upper-half driver will call ``transmit`` and ``receive`` to
    send/get packets.
5.  Remove any buffer related to ``d_buf``, and make sure ``d_buf`` is not
    used in the lower-half driver.
6.  Remove ``txavail`` function, the upper-half driver will call ``transmit``
    when it has packets to send.
7.  Remove the statistics macros like ``NETDEV_TXPACKETS``, ``NETDEV_TXDONE``,
    ``NETDEV_RXPACKETS`` or ``NETDEV_RXDROPPED``, these macros are well
    handled in upper-half.  But you may still keep macros like
    ``NETDEV_TXTIMEOUTS`` and ``NETDEV_RXERRORS`` because the upper-half
    cannot know whether these error happens.
8.  Find a suitable ``quota`` for the driver, and set it in the driver
    initialization function.  The quota is the maximum number of buffers
    that the driver can hold at the same time.  For example, if the TX quota
    is set to 5, it means that if the driver has 5 unreleased packets
    (``netpkt_free``), the upper-half will not call ``transmit`` until they
    are released.

    -  Note: An exception is that if the net stack is replying for RX packet,
       this replied packet will always be put into ``transmit``, which may
       exceed the TX quota temporarily.

"Lower Half" Example
====================

.. code-block:: c

  struct <chip>_priv_s
  {
    /* This holds the information visible to the NuttX network */

    struct netdev_lowerhalf_s dev;

    ...
  };

  static const struct netdev_ops_s g_ops =
  {
    .ifup     = <chip>_ifup,
    .ifdown   = <chip>_ifdown,
    .transmit = <chip>_transmit,
    .receive  = <chip>_receive,
    .addmac   = <chip>_addmac,
    .rmmac    = <chip>_rmmac,
    .ioctl    = <chip>_ioctl
  };

  /* The Wi-Fi driver registration function can be implemented as follows,
   * where <chip> refers to the chip name.  netdev_lower_register() is the
   * network device interface provided by upper-half drivers to register
   * network device drivers.
   */

  int <chip>_netdev_init(FAR struct <chip>_priv_s *priv)
  {
      FAR struct netdev_lowerhalf_s *dev = &priv->dev;

      dev->ops = &g_ops;

      /* The maximum number of buffers that the driver can hold
       * at the same time.  For example, if the TX quota is set to 5, it
       * means that if the driver has 5 unreleased packets (netpkt_free),
       * the upper layer will not call transmit until they are released.
       * After the rx quota is used up and no new buffer can be allocated
       * (netpkt_alloc), it needs to notify the upper layer
       * (netdev_lower_rxready) and restore the quota by submitting buffer
       * back through receive function.
       * If the driver processes each packet individually (without
       * accumulating multiple packets before sending/receiving), it can be
       * set to 1.
       */

      dev->quota[NETPKT_TX] = 1;
      dev->quota[NETPKT_RX] = 1;

      return netdev_lower_register(dev, NET_LL_ETHERNET);
  }

  /* The transmit function can be implemented as follows, where <chip>
   * refers to the chip name.
   */

  static int <chip>_transmit(FAR struct netdev_lowerhalf_s *dev,
                             FAR netpkt_t *pkt)
  {
    FAR struct <chip>_priv_s *priv = (FAR struct <chip>_priv_s *)dev;
    unsigned int len = netpkt_getdatalen(dev, pkt);

  #if you want to do offloading
    if (!netpkt_is_fragmented(pkt))
      {
        /* Contiguous memory, just use data pointer */

        FAR uint8_t *databuf = netpkt_getdata(dev, pkt);
        FAR uint8_t *devbuf  = databuf - sizeof(struct <chip>_txhead_s);

        /* Do Transmit.  Note: `databuf` points to the L2 data, and there is
         * a reserved memory with size of `CONFIG_NET_LL_GUARDSIZE` before
         * databuf to be used for driver header, drivers can just fill data
         * there (`devbuf`) and start the transmission.
         */

        ...
      }
    else
  #endif
      {
        /* Copyout the L2 data and transmit. */

        uint8_t devbuf[1600];
        netpkt_copyout(dev, devbuf, pkt, len, 0);

        /* Do Transmit */

        ...
      }

    return OK;
  }

  static void <chip>_txdone_interrupt(FAR struct <chip>_priv_s *priv)
  {
    FAR struct netdev_lowerhalf_s *dev = &priv->dev;

    /* Perform some processing in the driver (if necessary) */

    ...

    /* Free the buffer and notify the upper layer */

    netpkt_free(dev, pkt, NETPKT_TX);
    netdev_lower_txdone(dev);
  }

  /* The receive function can be implemented as follows, where <chip>
   * refers to the chip name.
   */

  static void <chip>_rxready_interrupt(FAR struct <chip>_priv_s *priv)
  {
    FAR struct netdev_lowerhalf_s *dev = &priv->dev;
    netdev_lower_rxready(dev);
  }

  static FAR netpkt_t *<chip>_receive(FAR struct netdev_lowerhalf_s *dev)
  {
    /* It is also possible to allocate the pkt and receive the data in
     * advance, and then call rxready and return pkt through receive
     */

    FAR netpkt_t *pkt = netpkt_alloc(dev, NETPKT_RX);

    if (pkt)
      {
  #if NETPKT_BUFLEN > 15xx && you want to do offloading
        /* Write directly to the buffer inside pkt, len corresponds to the
         * length of L2 data (need the NETPKT_BUFLEN to be large enough to
         * hold the data).  The `<chip>_rxhead_s` is the driver header before
         * the actual data (maybe you don't have).
         */

        len = receive_data_into(netpkt_getbase(pkt));
        netpkt_resetreserved(&priv->dev, pkt, sizeof(struct <chip>_rxhead_s));
        netpkt_setdatalen(&priv->dev, pkt, len);
  #else
        uint8_t devbuf[1600];

        /* Copy from src, len corresponds to the length of L2 data, you can
         * always use this method to receive data.  The `<chip>_rxhead_s` is
         * the driver header before the actual data (maybe you don't have).
         */

        len = receive_data_into(devbuf);
        netpkt_copyin(dev, pkt, devbuf + sizeof(struct <chip>_rxhead_s), len, 0);
  #endif
      }

    return pkt;
  }
