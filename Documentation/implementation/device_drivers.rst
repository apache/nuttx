==============
Device Drivers
==============

Standard Device Drivers
=======================

Device drivers should be implemented in the RTOS and used by applications.
Drivers provide access to device functionality for applications. This is a
necessary part of the modular RTOS design. In the NuttX directory structure,
share-able device drivers reside under ``drivers/`` and custom drivers reside in
the board-specific directories at ``nuttx/boards/<arch>/<chip>/<board>/src`` or
``nuttx/boards/<arch>/<chip>/drivers`` that are built into the RTOS.

Bus Drivers
===========

There a many things that get called drivers in OS; NuttX makes a distinction
between device drivers and bus drivers. For example, SPI, PCI, PCMCIA, USB,
Ethernet, etc. are buses and not devices. You will never find a device driver
for a bus in the NuttX architecture.

In most devices architectures, devices reside on a bus. A bus is a transport
layer that connects the device residing on the bus to a device driver.
The bus is managed by a bus driver. The device driver uses the facilities of
the bus driver transport layer to interact with the device.

Consider SPI. SPI is a bus. It provides a serial bus to which many devices may
be connected. An SPI device resides on the SPI bus in the sense that is shares
the same MISO, MOSI, and clock lines with other devices on the SPI bus (but in
SPI, it will have its own dedicated chip select discrete).

Although we typically use the same term driver to refer to both bus drivers and
device drivers, there is one big, fundamental difference: applications interact
only with devices drivers and never with bus drivers. Applications never talk
directly to PCI, PCMCIA, USB, Ethernet, nor with I2C, SPI, or GPIOs. Applications
interface through device drivers that use PCI, PCMCIA, USB, Ethernet, I2C, or SPI.
Bus drivers only exist to support the communication between the device driver and
the device on the bus.

Back to SPI... There will never be an application accessible interface to SPI.
If your application were to use SPI directly, then you would have have embedded
a device driver in your application and would have violated the RTOS functional
partition.

Test Drivers
============

It would be possible to provide character driver, such as SPI driver, that could
perform bus level accesses on behalf of an application. There are not many cases
where this would be acceptable, however. One possibility would be to support
support testing of bus drivers.
There is, an example for I2S here: ``drivers/audio/i2schar.c`` with a test case
here ``apps/examples/i2schar``. I2S is, of course, very similar to SPI.
This interface exists only for testing purposes and would probably not be
possible to build any meaningless application with it.

The I2C Tool
------------

Of course, like most rules, there are lots of violations. I2C is another bus and
the the I2C "driver" is another transport similar in many ways to SPI. For I2C,
there is an application at ``apps/system/i2c`` alled the "I2C tool" that will allow
you access I2C devices from the command line. This is not really just a test tool
and not a real part of an application.

And there is a fundamental flaw in the I2C tool: it uses NuttX internal interfaces
and violates the functional partitioning. NuttX has three build mode: (1) A flat
build where there is no enforcement of RTOS boundaries. In that flat build,
the I2C tool works fine. And (2) a kernel build mode and (3) a protected build mode.
In bothof these latter cases, the OS interfaces are strictly enforced. In the kernel
pand protected build modes, the I2C tool is not available because it cannot access
those NuttX internal interfaces.

User Space Drivers
==================

Above, it was stated that if your application were to use a bus directly, then you
would have have embedded a device driver in your application and would violate
the RTOS functional partition. Such device built into user applications are
referred to as user space drivers in some contexts. There is no plan or intent
to support user space drivers in NuttX.

Communication Devices
=====================

What about interface like CAN and UARTs? Why are those exposed as drivers when
SPI and I2C are not?

Semantics are difficult. The general principles that are maintain in
the RTOS are clear, but sometimes applying principles in a black and white way
is not easy in a world with shades of grey. (And if the principles get in the
way of good design then the principles should change).

In the case of true buses that support generic devices, the principle
is a good one. But there are grey areas too.

CAN seems similar to Ethernet. Both are network interfaces of sorts. You
wouldn't interface directly with Ethernet driver because you need to go
through a network stack of some type. The OSI model prevents it.

UARTs are communication devices. There is no RS-232 bus with devices connected
to it. Rather there are peers on the bus that you communicate with. This does not
preclude a UART from being used as a low level transfer for a device driver
(as with the driver for a wireless modules). Nor does it preclude a stack layer
like Modbus from being inserted in the path.

CAN differs from Ethernet in that it really is a direct peer-to-peer
communication, more like a UART. Although you can support a stack like CANOPen
on CAN. Currently CAN can be used as a simple character device, or as a network
interface using SocketCAN.

Communication devices support a fundamental peer-to-peer model. CAN and UARTs
are basically serial interfaces. But so are SPI, I2C, and USB. But those latter
serial interfaces clearly have a host/device, master/slave model associated with
them. It make perfectly good sense to think of them as buses that support device
interfaces.

I/O Expander
============

An I/O expander is device that interfaces with the MCU, usually via I2C, and
provides additional discrete inputs and outputs. The same rules apply:

* **GPIOS are Board-Specific**. Nothing in the system should now about GPIOs
  except for board specific logic. GPIOs can change from board-toboard. They
  can come and go. They can be replaced by GPIO expanders. Your (portable)
  application should not have any knowledge about how any discrete I/O is
  implemented on the board. There will never be GPIO drivers as a part of
  the NuttX architecture.

* **Common Drivers are Board-Independent**. Nor should common drivers
  (like those in ``drivers/``) know anything about GPIOs. In ALL cases,
  the board specific implementation in the board directories creates
  a "lower half" driver and binds that "lower half" driver with an common
  "upper half" driver to initialize the driver. Only the board logic has
  any kind of GPIO knowledge; not the application and not the common
  "upper half driver".

* **I2C and SPI Drivers are Internal Bus Drivers**. Similarly I2C and SPI
  drivers are not accessible to applications. These are NOT device drivers
  but are bus drivers. They should not be accessed directly by applications.
  Rather, again, the board-specific logic generates a "lower half" driver
  that provides a common I2C or SPI interface and binds that with
  an "upper half" driver to initialize the driver.

None of those rules change if you use an I/O expander, things just get
more convoluted.

Example Architecture
--------------------

Consider this case for some ``<board>``:

#. A discrete joystick is implemented as set of buttons: UP, DOWN, LEFT, RIGHT,
   and CENTER. The state of each the buttons is sensed as a GPIO input.

#. The GPIO button inputs go to I2C I/O expander at say,
   ``drivers/ioexpander/myexpander.c``, and finally to

#. The discrete joystick driver "upper half" driver (``drivers/input/djoystick.c``).

Implementation Details
----------------------

These should be implemented in the following, flexible, portable, layered architecture:

#. In the end, the application would interact only with a joystick driver
   interface via standard open/close/read/ioctl operations. It would receive
   pjoystick information as described in ``include/nuttx/input/djoystick.h.``

#. The discrete joystick driver would have been initialized by logic in some
   file like ``boards/<arch>/xyz/<board>/src/xyz_djoystick.c`` when the system
   was initialized. ``zyz_joystick.c`` would have created instance of
   the ``struct djoy_lowerhalf_s`` "lower half" interface as described in
   ``nuttx/include/nuttx/input/djoystick.h`` and would have passed that
   interface instance to the ``drivers/input/djoystick.c`` "upper half" driver
   to initialize it.

#. As part of the creation of the ``struct djoy_lowerhalf_s`` "lower half"
   interface instance, logic in ``xyz_djoystick.c`` would have done the following:
   It would have created an I2C driver instance by called MCU specific I2C initialization
   logic then passed this I2C driver instance to the I/O expander initialization interface
   in ``drivers/ioexpander/myexpander.c`` to create the I/O expander interface instance.

   Note that the I/O expander interface should NOT be a normal character driver.
   It should NOT be accessed via open/close/read/write/ioctl. Rather, it should return
   an instance of a some ``struct ioexpander_s`` interface. That I/O expander interface
   would be described in ``nuttx/include/ioexpander/ioexpander.h``. It is an internal
   operating system interface and would never be available to application logic.

   After receiving the I/O expander interface instance, the "lower half" discrete
   joystick interface would retain this internally as private data. Nothing in the
   system other than this "lower half" discrete joystick driver needs to know how
   the joystick is connected on board.

#. After creating the "upper half" discrete joystick interface interface,
   the "lower half" discrete joystick interface would enable interrupts from
   the I/O expander device.

#. When a key is pressed, the "lower half" discrete joystick driver would receive
   an interrupt from the I/O expander. It would then interact with the I/O driver
   to obtain the current discrete button depressions. The I/O expander driver would
   interact with I2C to obtain those button settings. Then the discrete joystick
   interface callback will be called, providing the discrete joystick "upper half"
   driver with the joystick input.

#. The "upper half" discrete joystick character driver would then return the encoded
   joystick input to the application in response to a ``read()`` from application code.
