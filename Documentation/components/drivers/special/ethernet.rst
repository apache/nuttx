=======================
Ethernet Device Drivers
=======================

-  ``include/nuttx/net/netdev.h``. All structures and APIs
   needed to work with Ethernet drivers are provided in this
   header file. The structure ``struct net_driver_s`` defines the
   interface and is passed to the network via
   ``netdev_register()``.

-  ``int netdev_register(FAR struct net_driver_s *dev, enum net_lltype_e lltype);``.
   Each Ethernet driver registers itself by calling
   ``netdev_register()``.

-  **Examples**: ``drivers/net/dm90x0.c``,
   ``arch/drivers/arm/src/c5471/c5471_ethernet.c``,
   ``arch/z80/src/ez80/ez80_emac.c``, etc.
