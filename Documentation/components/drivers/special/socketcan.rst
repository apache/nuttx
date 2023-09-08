========================
SocketCAN Device Drivers
========================

  -  ``include/nuttx/net/netdev.h``. All structures and APIs
     needed to work with drivers are provided in this header file.
     The structure struct net_driver_s defines the interface and is
     passed to the network via netdev_register().

  -  ``include/nuttx/can.h``. CAN & CAN FD frame data
     structures.

  -  ``int netdev_register(FAR struct net_driver_s *dev, enum net_lltype_e lltype)'``.
     Each driver registers itself by calling netdev_register().

  -  ``Include/nuttx/net/can.h``. contains lookup tables for CAN
     dlc to CAN FD len sizes named

     .. code-block:: c

       extern const uint8_t can_dlc_to_len[16];
       extern const uint8_t len_to_can_dlc[65];

  -  **Initialization sequence is as follows**.

     #. xxx_netinitialize(void) is called on startup of NuttX in this
        function you call your own init function to initialize your
        CAN driver
     #. In your own init function you create the net_driver_s
        structure set required init values and register the required
        callbacks for SocketCAN
     #. Then you ensure that the CAN interface is in down mode
        (usually done by calling the d_ifdown function)
     #. Register the net_driver_s using netdev_register

  -  **Receive sequence is as follows**.

     #. Device generates interrupt
     #. Process this interrupt in your interrupt handler
     #. When a new CAN frame has been received you process this
        frame
     #. When the CAN frame is a normal CAN frame you allocate the
        can_frame struct, when it's a CAN FD frame you allocate a
        canfd_frame struct (note you can of course preallocate and
        just use the pointer).
     #. Copy the frame from the driver to the struct you've
        allocated in the previous step.
     #. Point the net_driver_s d_buf pointer to the allocated
        can_frame
     #. Call the ``can_input(FAR struct net_driver_s *dev)``
        function ``include/nuttx/net/can.h``

  -  **Transmit sequence is as follows**.

     #. Socket layer executes d_txavail callback
     #. An example of the txavail function can be found in
        ``arch/arm/src/s32k1xx/s32k1xx_flexcan.c``
     #. An example of the txpoll function can be found in
        ``arch/arm/src/s32k1xx/s32k1xx_flexcan.c``
     #. In your ``transmit(struct driver_s *priv)`` function you
        check the length of ``net_driver_s.d_len`` whether it
        matches the size of a ``struct can_frame`` or
        ``struct canfd_frame`` then you cast the content of the
        ``net_driver_s.d_buf`` pointer to the correct CAN frame
        struct

