=========================
``nxmodbus`` NuttX Modbus
=========================

NxModbus is a lightweight Modbus protocol stack implementation for NuttX RTOS.
It provides client and server functionality for RTU, ASCII, and TCP transports
with a clean, callback-based API.

NxModbus follows the Modbus Organization's modern terminology (adopted September 2020):

- **Client**: Device that initiates requests (formerly "Master")
- **Server**: Device that processes requests and returns responses (formerly "Slave")
- **Device**: Remote Modbus device (in exception codes)

This aligns with current industry standards and matches other modern protocols
(HTTP, MQTT, OPC-UA).

NxModbus uses **0-based addressing**. Addresses passed to application
callbacks match the Modbus wire protocol directly: address 0 on the wire is
address 0 in the callback. This is consistent with libmodbus, pymodbus, and
the Modbus Application Protocol specification.

This differs from FreeModBus, which adds 1 to wire addresses before passing
them to callbacks (PLC register numbering convention). If migrating from
FreeModBus, remove any ``addr - 1`` adjustments in your callback functions.

Features
========

- **Multiple Transports**: RTU (serial), ASCII (serial), TCP, and raw ADU
- **Client/Server Modes**: Full support for both roles
- **Callback Architecture**: No internal data storage, application controls all data
- **Conditional Termios**: Supports systems with and without termios
- **Thread-Safe**: Mutex-protected context operations
- **Multi-Instance**: Supports concurrent instances with independent configuration
- **Extensible**: Custom function code registration via ``nxmb_register_custom_fc()``

Supported Function Codes
========================

NxModbus implements the following Modbus function codes:

Bit Access:

- **FC01** (0x01): Read Coils
- **FC02** (0x02): Read Discrete Inputs
- **FC05** (0x05): Write Single Coil
- **FC15** (0x0F): Write Multiple Coils

Register Access:

- **FC03** (0x03): Read Holding Registers
- **FC04** (0x04): Read Input Registers
- **FC06** (0x06): Write Single Holding Register
- **FC16** (0x10): Write Multiple Holding Registers
- **FC23** (0x17): Read/Write Multiple Holding Registers

Diagnostics and Identification:

- **FC08** (0x08): Diagnostics (sub-function 0x0000 Return Query Data)
- **FC17** (0x11): Report Server ID

Each server-side function code handler can be individually enabled or disabled
via Kconfig (all enabled by default). Disabled handlers respond with
Illegal Function exception. Client-side request functions are always available
when ``CONFIG_NXMODBUS_CLIENT`` is enabled. Custom function codes can be
registered via ``nxmb_register_custom_fc()`` when ``CONFIG_NXMODBUS_CUSTOM_FC``
is enabled.

Transport Modes
===============

- RTU (``CONFIG_NXMODBUS_RTU``):
  Binary encoding over serial with CRC16 error checking. Uses T1.5 and T3.5
  character timing for frame delimiting. Supports configurable baud rate and
  parity.

- ASCII (``CONFIG_NXMODBUS_ASCII``):
  Hex-encoded text over serial with LRC error checking. Frames are delimited
  by ':' start and CR/LF end markers. Character timeout is configurable via
  ``CONFIG_NXMODBUS_ASCII_TIMEOUT_SEC``.

- TCP (``CONFIG_NXMODBUS_TCP``):
  Modbus frames encapsulated in TCP/IP using the MBAP (Modbus Application
  Protocol) header. Default port 502. Supports both server (listening) and
  client (connecting) modes. Server mode supports multiple simultaneous
  client connections (configurable via ``CONFIG_NXMODBUS_TCP_MAX_CLIENTS``,
  default 1, up to 8).
  Requires ``CONFIG_NET_TCP``.

- Raw ADU (``CONFIG_NXMODBUS_RAW_ADU``):
  Application-provided callbacks for transmitting and receiving raw Modbus
  frames. Enables custom transport backends such as TLS, CAN, BLE, or MQTT.

Architecture
============

NxModbus uses a three-layer architecture:

1. **Core Layer** (``core/``): Protocol logic, function handlers, exception handling
2. **Transport Layer** (``transport/``): RTU, ASCII, TCP, and raw ADU implementations
3. **Public API** (``include/nxmodbus/``): Application interface

The transport layer uses a function pointer-based abstraction (``nxmb_transport_ops_s``)
allowing runtime selection of transport mode.

Configuration Options
=====================

NxModbus configuration is integrated into the NuttX Kconfig system:

Role Selection:

- ``CONFIG_NXMODBUS_SERVER`` – Enable Modbus server support
- ``CONFIG_NXMODBUS_CLIENT`` – Enable Modbus master (client) support

Transport Selection:

- ``CONFIG_NXMODBUS_RTU`` – Enable Modbus RTU (serial) transport
- ``CONFIG_NXMODBUS_ASCII`` – Enable Modbus ASCII (serial) transport
- ``CONFIG_NXMODBUS_TCP`` – Enable Modbus TCP transport (requires ``CONFIG_NET_TCP``)
- ``CONFIG_NXMODBUS_RAW_ADU`` – Enable raw ADU transport (custom backends)

Function Code Selection (server-side, all enabled by default):

- ``CONFIG_NXMODBUS_FUNC_READ_COILS`` – FC01 Read Coils
- ``CONFIG_NXMODBUS_FUNC_READ_DISCRETE`` – FC02 Read Discrete Inputs
- ``CONFIG_NXMODBUS_FUNC_READ_HOLDING`` – FC03 Read Holding Registers
- ``CONFIG_NXMODBUS_FUNC_READ_INPUT`` – FC04 Read Input Registers
- ``CONFIG_NXMODBUS_FUNC_WRITE_COIL`` – FC05 Write Single Coil
- ``CONFIG_NXMODBUS_FUNC_WRITE_HOLDING`` – FC06 Write Single Holding Register
- ``CONFIG_NXMODBUS_FUNC_DIAGNOSTICS`` – FC08 Diagnostics
- ``CONFIG_NXMODBUS_FUNC_WRITE_COILS`` – FC15 Write Multiple Coils
- ``CONFIG_NXMODBUS_FUNC_WRITE_HOLDINGS`` – FC16 Write Multiple Holding Registers
- ``CONFIG_NXMODBUS_FUNC_REPORT_SERVER_ID`` – FC17 Report Server ID
- ``CONFIG_NXMODBUS_FUNC_READWRITE_HOLDINGS`` – FC23 Read/Write Multiple Holding Registers

Instance and Buffer Configuration:

- ``CONFIG_NXMODBUS_MAX_INSTANCES`` – Maximum concurrent Modbus instances (1-16, default: 1)
- ``CONFIG_NXMODBUS_BUFFER_SIZE`` – ADU buffer size in bytes (64-256, default: 256)

FC17 Report Server ID:

- ``CONFIG_NXMODBUS_REP_SERVER_ID_BUF`` – Buffer size for Report Server ID
  response data (4-253, default: 32). Holds server ID, run indicator, and
  optional additional data configured via ``nxmb_set_server_id()``.

Timeouts:

- ``CONFIG_NXMODBUS_CLIENT_TIMEOUT_MS`` – Default client response timeout
  in milliseconds (100-60000, default: 1000). Can be overridden at runtime
  via ``nxmb_set_timeout()``.
- ``CONFIG_NXMODBUS_TCP_MAX_CLIENTS`` – Maximum simultaneous TCP client
  connections per server instance (1-8, default: 1).
- ``CONFIG_NXMODBUS_TCP_TIMEOUT_SEC`` – TCP idle connection timeout in
  seconds (1-3600, default: 60). Connections with no activity beyond this
  period are closed.
- ``CONFIG_NXMODBUS_RTU_IDLE_TIMEOUT_MS`` – RTU inter-frame idle timeout
  in milliseconds (1-1000, default: 50). Fallback ``select()`` timeout when
  waiting for a new frame. T3.5 character timing is still used for frame
  delimiting during active reception.
- ``CONFIG_NXMODBUS_ASCII_TIMEOUT_SEC`` – Character timeout for ASCII mode
  in seconds (1-60, default: 1)

Serial Configuration (RTU/ASCII):

- ``CONFIG_SERIAL_TERMIOS`` – Enable termios-based serial configuration

  If disabled, serial ports are used in raw mode without baud rate or parity
  configuration. This is useful for systems without termios support or when
  using pre-configured serial devices.

Extensibility:

- ``CONFIG_NXMODBUS_CUSTOM_FC`` – Enable custom function code handler registration

Termios Support
===============

NxModbus conditionally uses termios for serial port configuration based on
``CONFIG_SERIAL_TERMIOS``:

With termios (``CONFIG_SERIAL_TERMIOS=y``):

- Full baud rate configuration (9600, 19200, 38400, 57600, 115200)
- Parity configuration (none, even, odd)
- Automatic raw mode setup (no echo, no canonical mode)
- Original settings saved and restored on cleanup

Without termios (``CONFIG_SERIAL_TERMIOS=n``):

- Serial device opened in raw mode
- No baud rate or parity configuration
- Device must be pre-configured or support raw I/O
- Useful for:

  - Systems without termios support
  - Pre-configured serial backends
  - Custom serial drivers
  - Non-standard serial devices

API Reference
=============

Instance Management:

- ``nxmb_create()`` – Create and initialize an NxModbus instance
- ``nxmb_destroy()`` – Destroy an instance and release resources
- ``nxmb_enable()`` – Enable transport and start processing
- ``nxmb_disable()`` – Stop processing and release transport

Server Configuration:

- ``nxmb_set_callbacks()`` – Register application callbacks for data model access
- ``nxmb_set_server_id()`` – Configure FC17 Report Server ID response data
- ``nxmb_poll()`` – Execute one server-side polling iteration

Client Functions (require ``CONFIG_NXMODBUS_CLIENT``):

- ``nxmb_read_coils()`` – Read coils (FC01)
- ``nxmb_read_discrete()`` – Read discrete inputs (FC02)
- ``nxmb_read_holding()`` – Read holding registers (FC03)
- ``nxmb_read_input()`` – Read input registers (FC04)
- ``nxmb_write_coil()`` – Write single coil (FC05)
- ``nxmb_write_holding()`` – Write single holding register (FC06)
- ``nxmb_write_coils()`` – Write multiple coils (FC15)
- ``nxmb_write_holdings()`` – Write multiple holding registers (FC16)
- ``nxmb_readwrite_holdings()`` – Read/write multiple holding registers (FC23)
- ``nxmb_set_timeout()`` – Set client-side response timeout

Extensibility (require ``CONFIG_NXMODBUS_CUSTOM_FC``):

- ``nxmb_register_custom_fc()`` – Register a custom function code handler

Exception Codes
===============

NxModbus implements standard Modbus exception codes:

- ``NXMB_EX_NONE`` (0x00): No exception
- ``NXMB_EX_ILLEGAL_FUNCTION`` (0x01): Function code not supported
- ``NXMB_EX_ILLEGAL_DATA_ADDRESS`` (0x02): Register address out of range
- ``NXMB_EX_ILLEGAL_DATA_VALUE`` (0x03): Invalid data value
- ``NXMB_EX_DEVICE_FAILURE`` (0x04): Device failure
- ``NXMB_EX_ACKNOWLEDGE`` (0x05): Request acknowledged (long operation)
- ``NXMB_EX_DEVICE_BUSY`` (0x06): Device busy
- ``NXMB_EX_MEMORY_PARITY_ERROR`` (0x08): Memory parity error
- ``NXMB_EX_GATEWAY_PATH_FAILED`` (0x0A): Gateway path unavailable
- ``NXMB_EX_GATEWAY_TGT_FAILED`` (0x0B): Gateway target failed to respond

Comparison with FreeModBus
==========================

NxModbus is an alternative to FreeModBus with several improvements:

- Modern client/server terminology
- Cleaner API with opaque handles
- Multi-instance support
- Conditional termios support
- Thread-safe by design
- No global state
- Raw ADU transport for custom backends (TLS, CAN, BLE, MQTT)
- Extensible function code table via ``nxmb_register_custom_fc()``
