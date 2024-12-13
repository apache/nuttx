=========
SocketCAN
=========

SocketCAN Device Drivers
========================

- ``include/nuttx/net/netdev.h``. All structures and APIs
  needed to work with drivers are provided in this header file.
  The structure struct net_driver_s defines the interface and is
  passed to the network via ``netdev_register()``.

- ``include/nuttx/can.h``. CAN & CAN FD frame data structures and dlc to len size

  .. code-block:: c

     uint8_t can_bytes2dlc(uint8_t nbytes);
     uint8_t can_dlc2bytes(uint8_t dlc);

- ``int netdev_register(FAR struct net_driver_s *dev, enum net_lltype_e lltype)'``.
  Each driver registers itself by calling ``netdev_register()``.

- ``Include/nuttx/net/can.h``. contains lookup tables for CAN
  dlc to CAN FD len sizes named

- **Initialization sequence is as follows**.

  #. ``xxx_netinitialize(void)`` is called on startup of NuttX in this
     function you call your own init function to initialize your
     CAN driver
  #. In your own init function you create the net_driver_s
     structure set required init values and register the required
     callbacks for SocketCAN
  #. Then you ensure that the CAN interface is in down mode
     (usually done by calling the d_ifdown function)
  #. Register the net_driver_s using netdev_register

- **Receive sequence is as follows**.

  #. Device generates interrupt
  #. Process this interrupt in your interrupt handler
  #. When a new CAN frame has been received you process this frame
  #. When the CAN frame is a normal CAN frame you allocate the
     can_frame struct, when it's a CAN FD frame you allocate a
     canfd_frame struct (note you can of course preallocate and
     just use the pointer).
  #. Copy the frame from the driver to the struct you've
     allocated in the previous step.
  #. Point the net_driver_s d_buf pointer to the allocated can_frame
  #. Call the ``can_input(FAR struct net_driver_s *dev)``
     function ``include/nuttx/net/can.h``

- **Transmit sequence is as follows**.

  #. Socket layer executes d_txavail callback
  #. An example of the txavail function can be found in
     ``arch/arm/src/s32k1xx/s32k1xx_flexcan.c``
  #. An example of the txpoll function can be found in
     ``arch/arm/src/s32k1xx/s32k1xx_flexcan.c``
  #. In your ``transmit(struct driver_s *priv)`` function you
     check the length of ``net_driver_s.d_len`` whether it
     matches the size of a ``struct can_frame`` or
     ``struct canfd_frame`` then you cast the content of the
     ``net_driver_s.d_buf`` pointer to the correct CAN frame struct

SocketCAN protocol stack
========================

SocketCAN is a CAN protocol stack implementation based on the BSD socket API,
providing a more standardized and flexible CAN communication interface.
SocketCAN uses the network protocol stack framework, allowing applications
to use standard socket system calls (such as ``socket()``, ``bind()``,
``send()``, ``recv()``, etc.) for CAN communication.

Architecture
------------

The SocketCAN implementation follows the standard network layer hierarchy:

#. **Application Layer Interface**: Uses standard socket API
   (AF_CAN address family)
#. **Protocol Layer**: CAN protocol processing (located in ``net/can/``)
#. **Device Layer**: CAN network device drivers

File Locations
--------------

Files supporting SocketCAN can be found in the following locations:

-  **Protocol Implementation**: The SocketCAN protocol stack is located in
   the ``net/can/`` directory
-  **Header File**: ``include/nuttx/net/can.h``
-  **Main Modules**:

   - ``can_conn.c`` - Connection management
   - ``can_sockif.c`` - Socket interface implementation
   - ``can_sendmsg.c`` - Message transmission
   - ``can_sendmsg_buffered.c`` - Message transmission with buffering
   - ``can_recvmsg.c`` - Message reception
   - ``can_poll.c`` - Polling support
   - ``can_callback.c`` - Callback handling

Configuration Options
---------------------

To enable SocketCAN, configure the following options:

-  ``CONFIG_NET_CAN`` - Enable SocketCAN support
-  ``CONFIG_NET_CAN_NOTIFIER`` - Enable CAN notifier (optional)
-  ``CONFIG_NET_CAN_NBUFFERS`` - Number of CAN buffers
-  ``CONFIG_NET_RECV_BUFSIZE`` - Receive buffer size

Usage Notes
-----------

SocketCAN uses the standard socket programming model:

.. code-block:: c

   /* Create CAN socket */
   int sock = socket(AF_CAN, SOCK_RAW, CAN_RAW);

   /* Bind to CAN interface */
   struct sockaddr_can addr;
   addr.can_family = AF_CAN;
   addr.can_ifindex = if_nametoindex("can0");
   bind(sock, (struct sockaddr *)&addr, sizeof(addr));

   /* Send CAN frame */
   struct can_frame frame;
   frame.can_id = 0x123;
   frame.can_dlc = 8;
   /* Fill data */
   send(sock, &frame, sizeof(frame), 0);

   /* Receive CAN frame */
   recv(sock, &frame, sizeof(frame), 0);

Features
--------

-  **Standard Socket API**: Uses familiar socket programming interface
-  **Filtering Support**: Set CAN ID filters via socket options
-  **Non-blocking I/O**: Supports non-blocking mode and polling
-  **Multiple Connections**: Supports multiple sockets accessing the same CAN bus simultaneously
-  **Read Buffering**: Supports data read buffering to prevent data loss
-  **CAN FD Support**: Supports CAN FD frames if enabled in configuration
-  **Extensible**: Easy to extend for additional CAN protocols
