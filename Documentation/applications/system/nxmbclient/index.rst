===================================
``nxmbclient`` NxModbus Client Tool
===================================

The ``nxmbclient`` command-line tool provides a convenient way to perform Modbus
client (master) operations from the NuttX shell. It supports RTU, ASCII, and TCP
transports with full command-line configuration.

Supported Commands
==================

Read Operations:

- ``read-coils ADDR COUNT`` - Read coil status (FC01)
- ``read-discrete ADDR COUNT`` - Read discrete input status (FC02)
- ``read-input ADDR COUNT`` - Read input registers (FC04)
- ``read-holding ADDR COUNT`` - Read holding registers (FC03)

Write Operations:

- ``write-coil ADDR VALUE`` - Write single coil (FC05)
- ``write-holding ADDR VALUE`` - Write single holding register (FC06)
- ``write-coils ADDR VALUE...`` - Write multiple coils (FC15)
- ``write-holdings ADDR VALUE...`` - Write multiple holding registers (FC16)

Command-Line Options
====================

Transport Selection (required):

- ``-t TYPE`` - Transport type: ``rtu``, ``ascii``, or ``tcp``

Serial Transport Options (RTU/ASCII):

- ``-d DEVICE`` - Serial device path (e.g., ``/dev/ttyS1``)
- ``-b BAUD`` - Baud rate (default: 115200)
- ``-p PARITY`` - Parity: ``none``, ``even``, or ``odd`` (default: none)

TCP Transport Options:

- ``-h HOST`` - TCP host address (e.g., ``192.168.1.100``)
- ``-P PORT`` - TCP port (default: 502)

Modbus Options:

- ``-u UNIT`` - Unit ID / slave address (default: 1)
- ``-T TIMEOUT`` - Timeout in milliseconds (default: 1000)
- ``--poll MS`` - Polling interval in milliseconds (0 = one-shot mode)

Usage Examples
==============

RTU Client - Read Holding Registers::

    nsh> nxmbclient -t rtu -d /dev/ttyS1 -b 115200 read-holding 0 10
    Read 10 holding registers from address 0:
    [0]: 0x0000
    [1]: 0x0064
    [2]: 0x00C8
    ...

TCP Client - Write Single Register::

    nsh> nxmbclient -t tcp -h 192.168.1.100 -P 502 write-holding 0 1234
    Wrote holding register at address 0: 1234

RTU Client - Continuous Polling::

    nsh> nxmbclient -t rtu -d /dev/ttyS1 --poll 1000 read-holding 0 5
    Read 5 holding registers from address 0:
    [0]: 0x0000
    [1]: 0x0064
    [2]: 0x00C8
    [3]: 0x012C
    [4]: 0x0190
    
    (repeats every 1000ms until Ctrl+C)

ASCII Client - Write Multiple Coils::

    nsh> nxmbclient -t ascii -d /dev/ttyS1 -b 9600 -p even write-coils 0 1 0 1 1
    Wrote 5 coils starting at address 0

Configuration
=============

Enable the tool in your NuttX configuration::

    CONFIG_SYSTEM_NXMBCLIENT=y
    CONFIG_NXMODBUS=y
    CONFIG_NXMODBUS_RTU=y      # For RTU support
    CONFIG_NXMODBUS_ASCII=y    # For ASCII support
    CONFIG_NXMODBUS_TCP=y      # For TCP support

Kconfig Options:

- ``CONFIG_SYSTEM_NXMBCLIENT`` - Enable the nxmbclient tool
- ``CONFIG_NXMBCLIENT_PROGNAME`` - Program name (default: "nxmbclient")
- ``CONFIG_NXMBCLIENT_PRIORITY`` - Task priority (default: 100)
- ``CONFIG_NXMBCLIENT_STACKSIZE`` - Stack size (default: DEFAULT_TASK_STACKSIZE)

See Also
========

- :doc:`/applications/industry/nxmodbus/index` - NxModbus protocol stack
- :doc:`/applications/examples/nxmbserver/index` - NxModbus server example
