=====================
USB Host-Side Drivers
=====================

-  **USB host controller driver** abstracts the host controller
   device in the target chip. Each USB host controller driver
   must implement an instance of ``struct usbhost_driver_s`` and
   ``struct usbhost_connection_s`` defined in
   ``include/nuttx/usb/usbhost.h``.

   -  ``struct usbhost_driver_s`` provides the interface between 
      the USB host driver and the USB host class driver.

   -  ``struct usbhost_connection_s`` provides the interface between
      the USB host driver and platform-specific connection management
      and device enumeration logic.


   **Examples**: ``arch/arm/src/lpc17xx_40xx/lpc17_40_usbhost.c``,
   ``arch/arm/src/stm32/stm32_otgfshost.c``,
   ``arch/arm/src/sama5/sam_ohci.c``, and
   ``arch/arm/src/sama5/sam_ehci.c``.

-  **USB host class driver** abstracts USB peripherals conected to
   the USB host controller. Each USB host class driver must implement
   an instance of ``struct usbhost_class_s`` defined also in
   ``include/nuttx/usb/usbhost.h``.

   **Examples**: ``drivers/usbhost/usbhost_storage.c``

-  **USB Host Class Driver Registry**. The NuttX USB host
   infrastructure includes a *registry*. During its
   initialization, each USB host class driver must call the
   interface, ``usbhost_registerclass()`` in order to add its
   interface to the registry. Later, when a USB device is
   connected, the USB host controller will look up the USB host
   class driver that is needed to support the connected device in
   this registry.

   **Examples**: ``drivers/usbhost/usbhost_registry.c``,
   ``drivers/usbhost/usbhost_registerclass.c``, and
   ``drivers/usbhost/usbhost_findclass.c``,

-  **Detection and Enumeration of Connected Devices**. Each USB
   host device controller supports two methods that are used to
   detect and enumeration newly connected devices (and also detect
   disconnected devices):

   -  ``int (*wait)(FAR struct usbhost_connection_s *drvr, FAR const bool *connected);``

      Wait for a device to be connected or disconnected.

   -  ``int (*enumerate)(FAR struct usbhost_connection_s *drvr, int rhpndx);``

      Enumerate the device connected to a root hub port. As part
      of this enumeration process, the driver will (1) get the
      device's configuration descriptor, (2) extract the class ID
      info from the configuration descriptor, (3) call
      ``usbhost_findclass(``) to find the class that supports this
      device, (4) call the ``create()`` method on the
      ``struct usbhost_registry_s interface`` to get a class
      instance, and finally (5) call the ``connect()`` method of
      the ``struct usbhost_class_s`` interface. After that, the
      class is in charge of the sequence of operations.

-  **Binding USB Host-Side Drivers**. USB host-side controller
   drivers are not normally directly accessed by user code, but
   are usually bound to another, higher level USB host class
   driver. The class driver exports the standard NuttX device
   interface so that the connected USB device can be accessed just
   as with other, similar, on-board devices. For example, the USB
   host mass storage class driver
   (``drivers/usbhost/usbhost_storage.c``) will register a
   standard, NuttX block driver interface (like ``/dev/sda``) that
   can be used to mount a file system just as with any other other
   block driver instance. In general, the binding sequence is:

   #. Each USB host class driver includes an initialization entry
      point that is called from the application at initialization
      time. This driver calls ``usbhost_registerclass()`` during
      this initialization in order to makes itself available in
      the event the device that it supports is connected.

      **Examples**: The function ``usbhost_msc_initialize()`` in
      the file ``drivers/usbhost/usbhost_storage.c``

   #. Each application must include a *waiter* thread that
      (1) calls the USB host controller driver's ``wait()`` to
      detect the connection of a device, and then (2) call the USB
      host controller driver's ``enumerate`` method to bind the
      registered USB host class driver to the USB host controller
      driver.

      **Examples**: The function ``nsh_waiter()`` in the file
      ``boards/arm/lpc17xx_40xx/olimex-lpc1766stk/src/lpc17_40_appinit.c``.

   #. As part of its operation during the binding operation, the
      USB host class driver will register an instances of a
      standard NuttX driver under the ``/dev`` directory. To
      repeat the above example, the USB host mass storage class
      driver (``drivers/usbhost/usbhost_storage.c``) will register
      a standard, NuttX block driver interface (like ``/dev/sda``)
      that can be used to mount a file system just as with any
      other other block driver instance.

      **Examples**: See the call to ``register_blockdriver()`` in
      the function ``usbhost_initvolume()`` in the file
      ``drivers/usbhost/usbhost_storage.c``.
