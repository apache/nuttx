================
One Wire Drivers
================

---------------------------------------------------------------
Interfacing multiple devices of the same family on the same bus
---------------------------------------------------------------

1Wire is a bus and thus the driver should allow to specify the device the application
should interface. Also the driver should allow to scan the whole bus for available devices.
The proper way should involve a background thread continuously scanning the bus and possibly
registering them in a filesystem.

But this would probably too much of an overkill, and thus a much simpler solution is to
call a scanning ``ioctl``, obtain available devices, select the one we want and tell that
to the driver.

.. c:macro:: ONEWIREIOC_GETFAMILYROMS

    An ``ioctl`` call that returns all scanned devices' ROMs of the same family
    (the family depends on the driver). All of the records are stored
    to the ``struct onewire_availroms_s`` object (supplied by the application).
    Usage: ``ioctl(fd, ONEWIREIOC_GETFAMILYROMS, (unsigned long *)&query)``

.. c:struct:: onewire_availroms_s
.. code-block:: c

    struct onewire_availroms_s
    {
        uint64_t *roms;
        int maxroms;
        int actual;
    };

The number of maximum records the driver can provide is specified by the user
in the ``maxroms`` field. The user must also specify the ``uint64_t`` buffer
into the ``roms`` field where the driver can store the scanned devices.
The number of scanned devices is set in the ``actual`` field by the driver.

.. c:macro:: ONEWIREIOC_SETROM

Set the ROM of the device the driver should interface. Please note the
ROM argument is a pointer to a ``uint64_t`` variable, and not a ``uint64_t``
value, the reason being ``arg`` in ``ioctl`` is ``unsigned long``, which
is 32bits on many architectures.
Usage: ``ioctl(fd, ONEWIREIOC_SETROM, (unsigned long *)&romcode)``.

-----------------------------------------
Maxim/Analog Devices DS2XXX EEPROM driver
-----------------------------------------

This driver can be used to interface the following EEPROMS with a scratchpad
(also specified in ``enum ds2xxx_eeproms_e`` in ``include/nuttx/1wire/1wire_ds2xxx.h``):

- DS2430: 32 bytes, 8 byte scratchpad,
- DS2431: 128 bytes, 8 byte scratchpad,
- DS2432: 128 bytes, 8 byte scratchpad,
- DS2433: 512 bytes, 32 byte scratchpad,
- DS28E04: 512 bytes, 32 byte scratchpad,
- DS28E07: 128 bytes, 8 byte scratchpad,
- DS28EC20: 2560 bytes, 32 byte scratchpad.

Each driver's instance can interface only one EEPROM type. If you want to interface
multiple eeproms on the same bus, you need to have two drivers (e.g. two ``/dev`` files).

Currently, only basic read/write operations on the EEPROMs are implemented.
Special ioctl calls locking respective pages (or possibly any other EEPROM features)
are still not yet implemented.
As the EEPROM driver is character based, you can move around the EEPROM using ``lseek``.
Scratchpad unaligned reads and writes are possible.

Driver's API
============

.. c:enum:: ds2xxx_eeproms_e
.. code-block:: c

    enum ds2xxx_eeproms_e
    {
        EEPROM_DS2430 = 0,
        EEPROM_DS2431,
        EEPROM_DS2432,
        EEPROM_DS2433,
        EEPROM_DS28E04,
        EEPROM_DS28E07,
        EEPROM_DS28EC20,
        EEPROM_DS_COUNT
    };

The enum of all supported EEPROMS (besides ``EEPROM_DS_COUNT``).

.. c:function:: int ds2xxx_initialize(FAR struct onewire_dev_s *dev, enum ds2xxx_eeproms_e devtype, FAR char *devname)

    Bind a ``onewire_dev_s`` struct to this driver, capable of interfacing
    DS2XXX 1Wire EEPROMs. The user must specify the device type
    and also the name of the device (e.g. ``/dev/ds2xxx``).

    :param dev: a pointer to the lowerhalf struct
    :param devtype: the type of EEPROMs to be interfaced
    :param devname: the name of the registered file

    :return: 0 on success and a registered driver, negated errno on failure

Example usage
-------------

Registering a driver (STM32 BSP, DS2431 memory):

.. code-block:: c

    struct onewire_dev_s *lwhalf;
    lwhalf = stm32_1wireinitialize(0);
    ds2xxx_initialize(lwhalf, EEPROM_DS2431, "/dev/ds2431");

Application usage (suppose all calls are successful):

.. code-block:: c

    /* Write to a specific EEPROM */

    int fd = open("/dev/ds2xxx", O_RDWR);
    struct onewire_availroms_s query;
    uint64_t romarr[8];
    query.roms = romarr;
    query.maxroms = 8;
    ioctl(fd, ONEWIREIOC_GETFAMILYROMS, (unsigned long *)&query);

    /* Suppose the driver returns 3 in query.actual. We want the last
     * device to be accessed.
     */

    ioctl(fd, ONEWIREIOC_SETROM, (unsigned long *)&query.roms[query.actual - 1]);

    lseek(fd, 10, SEEK_SET);
    write(fd, "HELLO", 5);
