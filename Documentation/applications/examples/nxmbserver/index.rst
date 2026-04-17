======================================
``nxmbserver`` NxModbus Server Example
======================================

The ``nxmbserver`` example demonstrates how to create a Modbus server (slave)
using the NxModbus protocol stack. It supports RTU, ASCII, and TCP transports
with simulated register data.

Register Map
============

The example server provides the following simulated registers:

Coils (read/write, FC01/FC05/FC15):

- Address range: 1-100
- Initial state: All zeros
- Writable via FC05 (write single) and FC15 (write multiple)

Discrete Inputs (read-only, FC02):

- Address range: 1-100
- Initial state: All zeros
- Read-only (cannot be modified by client)

Input Registers (read-only, FC04):

- Address range: 1-100
- Initial values: ``register[i] = i * 10`` (register 1 = 10, register 2 = 20,
  register 10 = 100)

Holding Registers (read/write, FC03/FC06/FC16/FC23):

- Address range: 1-100
- Initial values: ``register[i] = i * 100`` (register 1 = 100, register 2 = 200,
  register 10 = 1000)
- Writable via FC06 (write single), FC16 (write multiple), FC23 (read/write)

Command-Line Options
====================

Transport Selection (required):

- ``-t TYPE`` - Transport type: ``rtu``, ``ascii``, or ``tcp``

Serial Transport Options (RTU/ASCII):

- ``-d DEVICE`` - Serial device path (e.g., ``/dev/ttyS1``)
- ``-b BAUD`` - Baud rate (default: 115200)
- ``-p PARITY`` - Parity: ``none``, ``even``, or ``odd`` (default: none)

TCP Transport Options:

- ``-a ADDR`` - Bind address (default: 0.0.0.0 - all interfaces)
- ``-P PORT`` - TCP port (default: 502)

Modbus Options:

- ``-u UNIT`` - Unit ID / slave address (default: 1)

Usage Examples
==============

RTU Server on Serial Port::

    nsh> nxmbserver -t rtu -d /dev/ttyS1 -b 115200
    Starting Modbus RTU server on /dev/ttyS1 (baud=115200, unit=1)
    Server running. Press Ctrl+C to stop.
    Register map:
      Coils:          1-100 (read/write)
      Discrete:       1-100 (read-only)
      Input regs:     1-100 (read-only, value=addr*10)
      Holding regs:   1-100 (read/write, initial=addr*100)

TCP Server on Default Port::

    nsh> nxmbserver -t tcp -P 502
    Starting Modbus TCP server on port 502 (unit=1)
    Server running. Press Ctrl+C to stop.
    Register map:
      Coils:          1-100 (read/write)
      Discrete:       1-100 (read-only)
      Input regs:     1-100 (read-only, value=addr*10)
      Holding regs:   1-100 (read/write, initial=addr*100)

ASCII Server with Even Parity::

    nsh> nxmbserver -t ascii -d /dev/ttyS1 -b 9600 -p even -u 5
    Starting Modbus ASCII server on /dev/ttyS1 (baud=9600, unit=5)
    Server running. Press Ctrl+C to stop.
    ...

Testing with nxmbclient
=======================

The server can be tested using the ``nxmbclient`` tool:

Read Input Registers (initial values)::

    nsh> nxmbclient -t tcp -h 127.0.0.1 read-input 1 5
    Read 5 input registers from address 1:
    [1]: 10
    [2]: 20
    [3]: 30
    [4]: 40
    [5]: 50

Read Holding Registers (initial values)::

    nsh> nxmbclient -t tcp -h 127.0.0.1 read-holding 1 5
    Read 5 holding registers from address 1:
    [1]: 100
    [2]: 200
    [3]: 300
    [4]: 400
    [5]: 500

Write and Read Back Holding Register::

    nsh> nxmbclient -t tcp -h 127.0.0.1 write-holding 10 9999
    Wrote holding register at address 10: 9999
    
    nsh> nxmbclient -t tcp -h 127.0.0.1 read-holding 10 1
    Read 1 holding register from address 10:
    [10]: 9999

Configuration
=============

Enable the example in your NuttX configuration::

    CONFIG_EXAMPLES_NXMBSERVER=y
    CONFIG_INDUSTRY_NXMODBUS=y
    CONFIG_NXMODBUS_RTU=y      # For RTU support
    CONFIG_NXMODBUS_ASCII=y    # For ASCII support
    CONFIG_NXMODBUS_TCP=y      # For TCP support

Kconfig Options:

- ``CONFIG_EXAMPLES_NXMBSERVER`` - Enable the nxmbserver example
- ``CONFIG_EXAMPLES_NXMBSERVER_PROGNAME`` - Program name (default: "nxmbserver")
- ``CONFIG_EXAMPLES_NXMBSERVER_PRIORITY`` - Task priority (default: 100)
- ``CONFIG_EXAMPLES_NXMBSERVER_STACKSIZE`` - Stack size (default: DEFAULT_TASK_STACKSIZE)

See Also
========

- :doc:`/applications/industry/nxmodbus/index` - NxModbus protocol stack
- :doc:`/applications/system/nxmbclient/index` - NxModbus client tool
