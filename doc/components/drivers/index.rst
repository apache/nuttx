==============
Device Drivers
==============
    
NuttX supports a variety of device drivers including:

  -  *Character* Device Drivers,
  -  *Block* Device Drivers, and
  -  Other *Specialized* Drivers.

These different device driver types are discussed in the following
paragraphs. Note: device driver support depends on the
*in-memory*, *pseudo* file system that is enabled by default.

Character Device Drivers
************************

Character device drivers have these properties:

-  ``include/nuttx/fs/fs.h``. All structures and APIs needed
   to work with character drivers are provided in this header
   file.

-  ``struct file_operations``. Each character device driver
   must implement an instance of ``struct file_operations``. That
   structure defines a call table with the following methods:

-  ``int register_driver(const char *path, const struct file_operations *fops, mode_t mode, void *priv);``.
   Each character driver registers itself by calling
   ``register_driver()``, passing it the ``path`` where it will
   appear in the `pseudo-file-system <#NxFileSystem>`__ and it's
   initialized instance of ``struct file_operations``.

-  **User Access**. After it has been registered, the character
   driver can be accessed by user code using the standard `driver
   operations <NuttxUserGuide.html#driveroperations>`__ including
   ``open()``, ``close()``, ``read()``, ``write()``, etc.

-  **Specialized Character Drivers**. Within the common character
   driver framework, there are different specific varieties of
   *specialized* character drivers. The unique requirements of the
   underlying device hardware often mandates some customization of
   the character driver. These customizations tend to take the
   form of:

   -  Device-specific ``ioctl()`` commands used to performed
      specialized operations on the device. These ``ioctl()`` will
      be documented in header files under ``include/nuttx`` that
      detail the specific device interface.
   -  Specialized I/O formats. Some devices will require that
      ``read()`` and/or ``write()`` operations use data conforming
      to a specific format, rather than a plain stream of bytes.
      These specialized I/O formats will be documented in header
      files under ``include/nuttx`` that detail the specific
      device interface. The typical representation of the I/O
      format will be a C structure definition.

   The specialized character drivers support by NuttX are
   documented in the following paragraphs.

-  **Examples**: ``drivers/dev_null.c``, ``drivers/fifo.c``,
   ``drivers/serial.c``, etc.

Serial Device Drivers
=====================

-  ``include/nuttx/serial/serial.h``. All structures and APIs
   needed to work with serial drivers are provided in this header
   file.

-  ``struct uart_ops_s``. Each serial device driver must
   implement an instance of ``struct uart_ops_s``. That structure
   defines a call table with the following methods:

-  ``int uart_register(FAR const char *path, FAR uart_dev_t *dev);``.
   A serial driver may register itself by calling
   ``uart_register()``, passing it the ``path`` where it will
   appear in the `pseudo-file-system <#NxFileSystem>`__ and it's
   initialized instance of ``struct uart_ops_s``. By convention,
   serial device drivers are registered at paths like
   ``/dev/ttyS0``, ``/dev/ttyS1``, etc. See the
   ``uart_register()`` implementation in ``drivers/serial.c``.

-  **User Access**. Serial drivers are, ultimately, normal
   `character drivers <#chardrivers>`__ and are accessed as other
   character drivers.

-  **Examples**: ``arch/arm/src/stm32/stm32_serial.c``,
   ``arch/arm/src/lpc214x/lpc214x_serial.c``,
   ``arch/z16/src/z16f/z16f_serial.c``, etc.

Touchscreen Device Drivers
==========================

NuttX supports a two-part touchscreen driver architecture.

#. An "upper half", generic driver that provides the common
   touchscreen interface to application level code, and
#. A "lower half", platform-specific driver that implements the
   low-level touchscreen controls to implement the touchscreen
   functionality.

Files supporting the touchscreen controller (TSC) driver can be
found in the following locations:

-  **Interface Definition**. The header files for NuttX
   touchscreen drivers reside in the
   ``include/nuttx/include/input`` directory. The interface
   between the touchscreen controller "upper half" and "lower
   half" drivers are *not* common, but vary from
   controller-to-controller. Because of this, each touchscreen
   driver has its own unique header file that describes the "upper
   half"/"lower half" interface in that directory. The application
   level interface to each touchscreen driver, on the other hand,
   *is* the same for each touchscreen driver and is described
   ``include/nuttx/include/input/touchscreen.h``. The touchscreen
   driver uses a standard character driver framework but read
   operations return specially formatted data.
-  **"Upper Half" Driver**. The controller-specific, "upper half"
   touchscreen drivers reside in the directory ``drivers/input``.
-  **"Lower Half" Drivers**. Platform-specific touchscreen drivers
   reside in either: (1) The
   ``arch/``\ *<architecture>*\ ``/src/``\ *<hardware>* directory
   for the processor architectures that have build in touchscreen
   controllers or (2) the
   ``boards/``\ *<arch>*\ ``/``\ *<chip>*\ ``/``\ *<board>*\ ``/src/``
   directory for boards that use an external touchscreen
   controller chip.

Analog (ADC/DAC) Drivers
========================

The NuttX analog drivers are split into two parts:

#. An "upper half", generic driver that provides the common analog
   interface to application level code, and
#. A "lower half", platform-specific driver that implements the
   low-level controls to implement the analog functionality.

-  General header files for the NuttX analog drivers reside in
   ``include/nuttx/analog/``. These header files includes both the
   application level interface to the analog driver as well as the
   interface between the "upper half" and "lower half" drivers.
-  Common analog logic and share-able analog drivers reside in the
   ``drivers/analog/``.
-  Platform-specific drivers reside in
   ``arch/``\ *<architecture>*\ ``/src/``\ *<hardware>* directory
   for the specific processor *<architecture>* and for the
   specific *<chip>* analog peripheral devices.

ADC Drivers
-----------

-  ``include/nuttx/analog/adc.h``. All structures and APIs needed
   to work with ADC drivers are provided in this header file. This
   header file includes:

   #. Structures and interface descriptions needed to develop a
      low-level, architecture-specific, ADC driver.
   #. To register the ADC driver with a common ADC character
      driver.
   #. Interfaces needed for interfacing user programs with the
      common ADC character driver.

-  ``drivers/analog/adc.c``. The implementation of the common ADC
   character driver.

DAC Drivers
-----------

-  ``include/nuttx/analog/dac.h``. All structures and APIs needed
   to work with DAC drivers are provided in this header file. This
   header file includes:

   #. Structures and interface descriptions needed to develop a
      low-level, architecture-specific, DAC driver.
   #. To register the DAC driver with a common DAC character
      driver.
   #. Interfaces needed for interfacing user programs with the
      common DAC character driver.

-  ``drivers/analog/dac.c``. The implementation of the common DAC
   character driver.

PWM Drivers
===========

For the purposes of this driver, a PWM device is any device that
generates periodic output pulses of controlled frequency and pulse
width. Such a device might be used, for example, to perform
pulse-width modulated output or frequency/pulse-count modulated
output (such as might be needed to control a stepper motor).

The NuttX PWM driver is split into two parts:

#. An "upper half", generic driver that provides the common PWM
   interface to application level code, and
#. A "lower half", platform-specific driver that implements the
   low-level timer controls to implement the PWM functionality.

Files supporting PWM can be found in the following locations:

-  **Interface Definition**. The header file for the NuttX PWM
   driver reside at ``include/nuttx/timers/pwm.h``. This header
   file includes both the application level interface to the PWM
   driver as well as the interface between the "upper half" and
   "lower half" drivers. The PWM module uses a standard character
   driver framework. However, since the PWM driver is a devices
   control interface and not a data transfer interface, the
   majority of the functionality available to the application is
   implemented in driver ioctl calls.
-  **"Upper Half" Driver**. The generic, "upper half" PWM driver
   resides at ``drivers/pwm.c``.
-  **"Lower Half" Drivers**. Platform-specific PWM drivers reside
   in ``arch/``\ *<architecture>*\ ``/src/``\ *<hardware>*
   directory for the specific processor *<architecture>* and for
   the specific *<chip>* PWM peripheral devices.

CAN Drivers
===========

NuttX supports only a very low-level CAN driver. This driver
supports only the data exchange and does not include any
high-level CAN protocol. The NuttX CAN driver is split into two
parts:

#. An "upper half", generic driver that provides the common CAN
   interface to application level code, and
#. A "lower half", platform-specific driver that implements the
   low-level timer controls to implement the CAN functionality.

Files supporting CAN can be found in the following locations:

-  **Interface Definition**. The header file for the NuttX CAN
   driver resides at ``include/nuttx/can/can.h``. This header file
   includes both the application level interface to the CAN driver
   as well as the interface between the "upper half" and "lower
   half" drivers. The CAN module uses a standard character driver
   framework.
-  **"Upper Half" Driver**. The generic, "upper half" CAN driver
   resides at ``drivers/can.c``.
-  **"Lower Half" Drivers**. Platform-specific CAN drivers reside
   in ``arch/``\ *<architecture>*\ ``/src/``\ *<hardware>*
   directory for the specific processor *<architecture>* and for
   the specific *<chip>* CAN peripheral devices.

**Usage Note**: When reading from the CAN driver multiple messages
may be returned, depending on (1) the size the returned can
messages, and (2) the size of the buffer provided to receive CAN
messages. *Never assume that a single message will be returned*...
if you do this, *you will lose CAN data* under conditions where
your read buffer can hold more than one small message. Below is an
example about how you should think of the CAN read operation:

Quadrature Encoder Drivers
==========================

NuttX supports a low-level, two-part Quadrature Encoder driver.

#. An "upper half", generic driver that provides the common
   Quadrature Encoder interface to application level code, and
#. A "lower half", platform-specific driver that implements the
   low-level timer controls to implement the Quadrature Encoder
   functionality.

Files supporting the Quadrature Encoder can be found in the
following locations:

-  **Interface Definition**. The header file for the NuttX
   Quadrature Encoder driver reside at
   ``include/nuttx/sensors/qencoder.h``. This header file includes
   both the application level interface to the Quadrature Encoder
   driver as well as the interface between the "upper half" and
   "lower half" drivers. The Quadrature Encoder module uses a
   standard character driver framework.
-  **"Upper Half" Driver**. The generic, "upper half" Quadrature
   Encoder driver resides at ``drivers/sensors/qencoder.c``.
-  **"Lower Half" Drivers**. Platform-specific Quadrature Encoder
   drivers reside in
   ``arch/``\ *<architecture>*\ ``/src/``\ *<hardware>* directory
   for the specific processor *<architecture>* and for the
   specific *<chip>* Quadrature Encoder peripheral devices.

Timer Drivers
=============

NuttX supports a low-level, two-part timer driver.

#. An "upper half", generic driver that provides the common timer
   interface to application level code, and
#. A "lower half", platform-specific driver that implements the
   low-level timer controls to implement the timer functionality.

Files supporting the timer driver can be found in the following
locations:

-  **Interface Definition**. The header file for the NuttX timer
   driver reside at ``include/nuttx/timers/timer.h``. This header
   file includes both the application level interface to the timer
   driver as well as the interface between the "upper half" and
   "lower half" drivers. The timer driver uses a standard
   character driver framework.
-  **"Upper Half" Driver**. The generic, "upper half" timer driver
   resides at ``drivers/timers/timer.c``.
-  **"Lower Half" Drivers**. Platform-specific timer drivers
   reside in ``arch/``\ *<architecture>*\ ``/src/``\ *<hardware>*
   directory for the specific processor *<architecture>* and for
   the specific *<chip>* timer peripheral devices.

RTC Drivers
===========

NuttX supports a low-level, two-part RealTime Clock (RTC) driver.

#. An "upper half", generic driver that provides the common RTC
   interface to application level code, and
#. A "lower half", platform-specific driver that implements the
   low-level timer controls to implement the RTC functionality.

Files supporting the RTC driver can be found in the following
locations:

-  **Interface Definition**. The header file for the NuttX RTC
   driver reside at ``include/nuttx/timers/rtc.h``. This header
   file includes both the application level interface to the RTC
   driver as well as the interface between the "upper half" and
   "lower half" drivers. The RTC driver uses a standard character
   driver framework.
-  **"Upper Half" Driver**. The generic, "upper half" RTC driver
   resides at ``drivers/timers/rtc.c``.
-  **"Lower Half" Drivers**. Platform-specific RTC drivers reside
   in ``arch/``\ *<architecture>*\ ``/src/``\ *<hardware>*
   directory for the specific processor *<architecture>* and for
   the specific *<chip>* RTC peripheral devices.

Watchdog Timer Drivers
======================

NuttX supports a low-level, two-part watchdog timer driver.

#. An "upper half", generic driver that provides the common
   watchdog timer interface to application level code, and
#. A "lower half", platform-specific driver that implements the
   low-level timer controls to implement the watchdog timer
   functionality.

Files supporting the watchdog timer driver can be found in the
following locations:

-  **Interface Definition**. The header file for the NuttX
   watchdog timer driver reside at
   ``include/nuttx/timers/watchdog.h``. This header file includes
   both the application level interface to the watchdog timer
   driver as well as the interface between the "upper half" and
   "lower half" drivers. The watchdog timer driver uses a standard
   character driver framework.
-  **"Upper Half" Driver**. The generic, "upper half" watchdog
   timer driver resides at ``drivers/timers/watchdog.c``.
-  **"Lower Half" Drivers**. Platform-specific watchdog timer
   drivers reside in
   ``arch/``\ *<architecture>*\ ``/src/``\ *<hardware>* directory
   for the specific processor *<architecture>* and for the
   specific *<chip>* watchdog timer peripheral devices.

Keyboard/Keypad Drivers
=======================

**Keypads vs. Keyboards** Keyboards and keypads are really the
same devices for NuttX. A keypad is thought of as simply a
keyboard with fewer keys.

**Special Commands**. In NuttX, a keyboard/keypad driver is simply
a character driver that may have an (optional) encoding/decoding
layer on the data returned by the character driver. A keyboard may
return simple text data (alphabetic, numeric, and punctuation) or
control characters (enter, control-C, etc.) when a key is pressed.
We can think about this the "normal" keyboard data stream.
However, in addition, most keyboards support actions that cannot
be represented as text or control data. Such actions include
things like cursor controls (home, up arrow, page down, etc.),
editing functions (insert, delete, etc.), volume controls, (mute,
volume up, etc.) and other special functions. In this case, some
special encoding may be required to multiplex the normal text data
and special command key press data streams.

**Key Press and Release Events** Sometimes the time that a key is
released is needed by applications as well. Thus, in addition to
normal and special key press events, it may also be necessary to
encode normal and special key release events.

**Encoding/Decoding** Layer. An optional encoding/decoding layer
can be used with the basic character driver to encode the keyboard
events into the text data stream. The function interfaces that
comprise that encoding/decoding layer are defined in the header
file ``include/nuttx/input/kbd_code.h``. These functions provide
an matched set of (a) driver encoding interfaces, and (b)
application decoding interfaces.

#. **Driver Encoding Interfaces**. These are interfaces used by
   the keyboard/keypad driver to encode keyboard events and data.

   -  ``kbd_press()``

      **Function Prototype:**

      **Description:**

      **Input Parameters:**

      -  ``ch``: The character to be added to the output stream.
      -  ``stream``: An instance of ``lib_outstream_s`` to perform
         the actual low-level put operation.

      **Returned Value:**

   -  ``kbd_release()``

      **Function Prototype:**

      **Description:**

      **Input Parameters:**

      -  ``ch``: The character associated with the key that was
         released.
      -  ``stream``: An instance of ``lib_outstream_s`` to perform
         the actual low-level put operation.

      **Returned Value:**

   -  ``kbd_specpress()``

      **Function Prototype:**

      **Description:**

      **Input Parameters:**

      -  ``keycode``: The command to be added to the output
         stream. The enumeration ``enum kbd_keycode_e keycode``
         identifies all commands known to the system.
      -  ``stream``: An instance of ``lib_outstream_s`` to perform
         the actual low-level put operation.

      **Returned Value:**

   -  ``kbd_specrel()``

      **Function Prototype:**

      **Description:**

      **Input Parameters:**

      -  ``keycode``: The command to be added to the output
         stream. The enumeration ``enum kbd_keycode_e keycode``
         identifies all commands known to the system.
      -  ``stream``: An instance of ``lib_outstream_s`` to perform
         the actual low-level put operation.

      **Returned Value:**

#. **Application Decoding Interfaces**. These are user interfaces
   to decode the values returned by the keyboard/keypad driver.

   -  ``kbd_decode()``

      **Function Prototype:**

      **Description:**

      **Input Parameters:**

      -  ``stream``: An instance of ``lib_instream_s`` to perform
         the actual low-level get operation.
      -  ``pch``: The location to save the returned value. This
         may be either a normal, character code or a special
         command (i.e., a value from ``enum kbd_getstate_s``.
      -  ``state``: A user provided buffer to support parsing.
         This structure should be cleared the first time that
         ``kbd_decode()`` is called.

      **Returned Value:**

      -  ``KBD_PRESS`` (0)**: Indicates the successful receipt
         of normal, keyboard data. This corresponds to a keypress
         event. The returned value in ``pch`` is a simple byte of
         text or control data.
      -  ``KBD_RELEASE`` (1)**: Indicates a key release event.
         The returned value in ``pch`` is the byte of text or
         control data corresponding to the released key.
      -  ``KBD_SPECPRESS`` (2)**: Indicates the successful
         receipt of a special keyboard command. The returned value
         in ``pch`` is a value from ``enum kbd_getstate_s``.
      -  ``KBD_SPECREL`` (3)**: Indicates a special command key
         release event. The returned value in ``pch`` is a value
         from ``enum kbd_getstate_s``.
      -  ``KBD_ERROR`` (``EOF``)**: An error has getting the
         next character (reported by the ``stream``). Normally
         indicates the end of file.

**I/O Streams**. Notice the use of the abstract I/O streams in
these interfaces. These stream interfaces are defined in
``include/nuttx/streams.h``.

Block Device Drivers
********************

Block device drivers have these properties:

-  ``include/nuttx/fs/fs.h``. All structures and APIs needed
   to work with block drivers are provided in this header file.

-  ``struct block_operations``. Each block device driver must
   implement an instance of ``struct block_operations``. That
   structure defines a call table with the following methods:

-  ``int register_blockdriver(const char *path, const struct block_operations *bops, mode_t mode, void *priv);``.
   Each block driver registers itself by calling
   ``register_blockdriver()``, passing it the ``path`` where it
   will appear in the `pseudo-file-system <#NxFileSystem>`__ and
   it's initialized instance of ``struct block_operations``.

-  **User Access**. Users do not normally access block drivers
   directly, rather, they access block drivers indirectly through
   the ``mount()`` API. The ``mount()`` API binds a block driver
   instance with a file system and with a mountpoint. Then the
   user may use the block driver to access the file system on the
   underlying media. *Example*: See the ``cmd_mount()``
   implementation in ``apps/nshlib/nsh_fscmds.c``.

-  **Accessing a Character Driver as a Block Device**. See the
   loop device at ``drivers/loop.c``. *Example*: See the
   ``cmd_losetup()`` implementation in
   ``apps/nshlib/nsh_fscmds.c``.

-  **Accessing a Block Driver as Character Device**. See the
   Block-to-Character (BCH) conversion logic in ``drivers/bch/``.
   *Example*: See the ``cmd_dd()`` implementation in
   ``apps/nshlib/nsh_ddcmd.c``.

-  **Examples**. ``drivers/loop.c``,
   ``drivers/mmcsd/mmcsd_spi.c``, ``drivers/ramdisk.c``, etc.

Specialized Device Drivers
**************************

All device drivers that are accessible to application logic are
either: (1) Character device drivers that can be accessed via the
standard driver operations (``open()``, ``close()``, ``read()``,
``write()``, etc.), or (2) block drivers that can be accessing
only as part of mounting a file system or other special use cases
as described in the preceding paragraph.

In addition to this, there are also specialized "drivers" that can
be used only within the OS logic itself and are not accessible to
application logic. These specialized drivers are discussed in the
following paragraphs.

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

SPI Device Drivers
==================

-  ``include/nuttx/spi/spi.h``. All structures and APIs needed
   to work with SPI drivers are provided in this header file.

-  ``struct spi_ops_s``. Each SPI device driver must implement
   an instance of ``struct spi_ops_s``. That structure defines a
   call table with the following methods:

-  **Binding SPI Drivers**. SPI drivers are not normally directly
   accessed by user code, but are usually bound to another, higher
   level device driver. See for example,
   ``int mmcsd_spislotinitialize(int minor, int slotno, FAR struct spi_dev_s *spi)``
   in ``drivers/mmcsd/mmcsd_spi.c``. In general, the binding
   sequence is:

   #. Get an instance of ``struct spi_dev_s`` from the
      hardware-specific SPI device driver, and
   #. Provide that instance to the initialization method of the
      higher level device driver.

-  **Examples**: ``drivers/loop.c``,
   ``drivers/mmcsd/mmcsd_spi.c``, ``drivers/ramdisk.c``, etc.

I2C Device Drivers
==================

-  ``include/nuttx/i2c/i2c.h``. All structures and APIs needed
   to work with I2C drivers are provided in this header file.

-  ``struct i2c_ops_s``. Each I2C device driver must implement
   an instance of ``struct i2c_ops_s``. That structure defines a
   call table with the following methods:

-  **Binding I2C Drivers**. I2C drivers are not normally directly
   accessed by user code, but are usually bound to another, higher
   level device driver. In general, the binding sequence is:

   #. Get an instance of ``struct i2c_master_s`` from the
      hardware-specific I2C device driver, and
   #. Provide that instance to the initialization method of the
      higher level device driver.

-  **Examples**: ``arch/z80/src/ez80/ez80_i2c.c``,
   ``arch/z80/src/z8/z8_i2c.c``, etc.

Frame Buffer Drivers
====================

-  ``include/nuttx/video/fb.h``. All structures and APIs
   needed to work with frame buffer drivers are provided in this
   header file.

-  ``struct fb_vtable_s``. Each frame buffer device driver
   must implement an instance of ``struct fb_vtable_s``. That
   structure defines a call table with the following methods:

   Get information about the video controller configuration and
   the configuration of each color plane.

   The following are provided only if the video hardware supports
   RGB color mapping:

   The following are provided only if the video hardware supports
   a hardware cursor:

-  **Binding Frame Buffer Drivers**. Frame buffer drivers are not
   normally directly accessed by user code, but are usually bound
   to another, higher level device driver. In general, the binding
   sequence is:

   #. Get an instance of ``struct fb_vtable_s`` from the
      hardware-specific frame buffer device driver, and
   #. Provide that instance to the initialization method of the
      higher level device driver.

-  **Examples**: ``arch/sim/src/up_framebuffer.c``. See also the
   usage of the frame buffer driver in the ``graphics/``
   directory.

LCD Drivers
===========

-  ``include/nuttx/lcd/lcd.h``. Structures and APIs needed to
   work with LCD drivers are provided in this header file. This
   header file also depends on some of the same definitions used
   for the frame buffer driver as provided in
   ``include/nuttx/video/fb.h``.

-  ``struct lcd_dev_s``. Each LCD device driver must implement
   an instance of ``struct lcd_dev_s``. That structure defines a
   call table with the following methods:

   Get information about the LCD video controller configuration
   and the configuration of each LCD color plane.

   The following are provided only if the video hardware supports
   RGB color mapping:

   The following are provided only if the video hardware supports
   a hardware cursor:

   Get the LCD panel power status (0: full off -
   ``CONFIG_LCD_MAXPOWER``: full on). On backlit LCDs, this
   setting may correspond to the backlight setting.

   Enable/disable LCD panel power (0: full off -
   ``CONFIG_LCD_MAXPOWER``: full on). On backlit LCDs, this
   setting may correspond to the backlight setting.

   Get the current contrast setting (0-CONFIG_LCD_MAXCONTRAST) \*/

   Set LCD panel contrast (0-CONFIG_LCD_MAXCONTRAST)

-  **Binding LCD Drivers**. LCD drivers are not normally directly
   accessed by user code, but are usually bound to another, higher
   level device driver. In general, the binding sequence is:

   #. Get an instance of ``struct lcd_dev_s`` from the
      hardware-specific LCD device driver, and
   #. Provide that instance to the initialization method of the
      higher level device driver.

-  **Examples**: ``drivers/lcd/p14201.c``,
   ``boards/arm/sam34/sam3u-ek/src/up_lcd.c.`` See also the usage
   of the LCD driver in the ``graphics/`` directory.

Memory Technology Device Drivers
================================

-  ``include/nuttx/mtd/mtd.h``. All structures and APIs needed
   to work with MTD drivers are provided in this header file.

-  ``struct mtd_dev_s``. Each MTD device driver must implement
   an instance of ``struct mtd_dev_s``. That structure defines a
   call table with the following methods:

   Erase the specified erase blocks (units are erase blocks):

   Read/write from the specified read/write blocks:

   Some devices may support byte oriented reads (optional). Most
   MTD devices are inherently block oriented so byte-oriented
   accesses are not supported. It is recommended that low-level
   drivers not support read() if it requires buffering.

   Some devices may also support byte oriented writes (optional).
   Most MTD devices are inherently block oriented so byte-oriented
   accesses are not supported. It is recommended that low-level
   drivers not support read() if it requires buffering. This
   interface is only available if ``CONFIG_MTD_BYTE_WRITE`` is
   defined.

   Support other, less frequently used commands:

   -  ``MTDIOC_GEOMETRY``: Get MTD geometry
   -  ``MTDIOC_XIPBASE:``: Convert block to physical address for
      eXecute-In-Place
   -  ``MTDIOC_BULKERASE``: Erase the entire device

   is provided via a single ``ioctl`` method (see
   ``include/nuttx/fs/ioctl.h``):

-  **Binding MTD Drivers**. MTD drivers are not normally directly
   accessed by user code, but are usually bound to another, higher
   level device driver. In general, the binding sequence is:

   #. Get an instance of ``struct mtd_dev_s`` from the
      hardware-specific MTD device driver, and
   #. Provide that instance to the initialization method of the
      higher level device driver.

-  **Examples**: ``drivers/mtd/m25px.c`` and ``drivers/mtd/ftl.c``

SDIO Device Drivers
===================

-  ``include/nuttx/sdio.h``. All structures and APIs needed to
   work with SDIO drivers are provided in this header file.

-  ``struct sdio_dev_s``. Each SDIO device driver must
   implement an instance of ``struct sdio_dev_s``. That structure
   defines a call table with the following methods:

   Mutual exclusion:

   Initialization/setup:

   Command/Status/Data Transfer:

   Event/Callback support:

   DMA support:

-  **Binding SDIO Drivers**. SDIO drivers are not normally
   directly accessed by user code, but are usually bound to
   another, higher level device driver. In general, the binding
   sequence is:

   #. Get an instance of ``struct sdio_dev_s`` from the
      hardware-specific SDIO device driver, and
   #. Provide that instance to the initialization method of the
      higher level device driver.

-  **Examples**: ``arch/arm/src/stm32/stm32_sdio.c`` and
   ``drivers/mmcsd/mmcsd_sdio.c``

USB Host-Side Drivers
=====================

-  ``include/nuttx/usb/usbhost.h``. All structures and APIs
   needed to work with USB host-side drivers are provided in this
   header file.

-  ``struct usbhost_driver_s`` and
   ``struct usbhost_connection_s``. Each USB host controller
   driver must implement an instance of
   ``struct usbhost_driver_s`` and
   ``struct usbhost_connection_s``: ``struct usbhost_driver_s``
   provides the interface between the USB host driver and the USB
   class driver; ``struct usbhost_connection_s`` provides the
   interface between the USB host driver and platform-specific
   connection management and device enumeration logic. These
   structures are defined in ``include/nuttx/usb/usbhost.h``.

   **Examples**: ``arch/arm/src/lpc17xx_40xx/lpc17_40_usbhost.c``,
   ``arch/arm/src/stm32/stm32_otgfshost.c``,
   ``arch/arm/src/sama5/sam_ohci.c``, and
   ``arch/arm/src/sama5/sam_ehci.c``.

-  ``struct usbhost_class_s``. Each USB host class driver must
   implement an instance of ``struct usbhost_class_s``. This
   structure is also defined in ``include/nuttx/usb/usbhost.h``.

   **Examples**: ``drivers/usbhost/usbhost_storage.c``

-  **USB Host Class Driver Registry**. The NuttX USB host
   infrastructure includes a *registry*. During its
   initialization, each USB host class driver must call the
   interface, ``usbhost_registerclass()`` in order add its
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

   #. Each application must include a *waiter* thread thread that
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

USB Device-Side Drivers
=======================

-  ``include/nuttx/usb/usbdev.h``. All structures and APIs
   needed to work with USB device-side drivers are provided in
   this header file.

-  ``include/nuttx/usb/usbdev_trace.h``. Declarations needed
   to work with the NuttX USB device driver trace capability. That
   USB trace capability is detailed in `separate
   document <UsbTrace.html>`__.

-  ``struct usbdev_s``. Each USB device controller driver must
   implement an instance of ``struct usbdev_s``. This structure is
   defined in ``include/nuttx/usb/usbdev.h``.

   **Examples**: ``arch/arm/src/dm320/dm320_usbdev.c``,
   ``arch/arm/src/lpc17xx_40xx/lpc17_40_usbdev.c``,
   ``arch/arm/src/lpc214x/lpc214x_usbdev.c``,
   ``arch/arm/src/lpc313x/lpc313x_usbdev.c``, and
   ``arch/arm/src/stm32/stm32_usbdev.c``.

-  ``struct usbdevclass_driver_s``. Each USB device class
   driver must implement an instance of
   ``struct usbdevclass_driver_s``. This structure is also defined
   in ``include/nuttx/usb/usbdev.h``.

   **Examples**: ``drivers/usbdev/pl2303.c`` and
   ``drivers/usbdev/usbmsc.c``

-  **Binding USB Device-Side Drivers**. USB device-side controller
   drivers are not normally directly accessed by user code, but
   are usually bound to another, higher level USB device class
   driver. The class driver is then configured to export the USB
   device functionality. In general, the binding sequence is:

   #. Each USB device class driver includes an initialization
      entry point that is called from the application at
      initialization time.

      **Examples**: The function ``usbdev_serialinitialize()`` in
      the file ``drivers/usbdev/pl2303.c`` and the function
      in the file ``drivers/usbdev/usbmsc.c``

   #. These initialization functions called the driver API,
      ``usbdev_register()``. This driver function will *bind* the
      USB class driver to the USB device controller driver,
      completing the initialization.

