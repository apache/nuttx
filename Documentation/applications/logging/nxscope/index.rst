===============
NxScope Library
===============

NxScope provides real-time data logging by buffering samples into up to 255
virtual channels (vector, point, or text data) and streaming them through
custom protocols and interfaces. It supports remote control via commands,
timestamping, sample rate dividers, and critical non-buffered channels.

Use Cases
=========

- Real-time sensor data streaming and visualization.
- Control system telemetry (PID tuning, state monitoring).
- High-frequency signal capture (ADC).
- Remote debugging and live variable tracking.

Configuration
=============

- ``CONFIG_LOGGING_NXSCOPE_INTF_SERIAL`` – Serial port interface
- ``CONFIG_LOGGING_NXSCOPE_INTF_UDP`` – UDP interface (over Ethernet)
- ``CONFIG_LOGGING_NXSCOPE_INTF_DUMMY`` – Dummy interface (for debug)
- ``CONFIG_LOGGING_NXSCOPE_PROTO_SER`` – Default serial protocol (CRC-16)
- ``CONFIG_LOGGING_NXSCOPE_DIVIDER`` – Sample rate divider support
- ``CONFIG_LOGGING_NXSCOPE_ACKFRAMES`` – ACK frames for set requests
- ``CONFIG_LOGGING_NXSCOPE_USERTYPES`` – Support for user-defined frames
- ``CONFIG_LOGGING_NXSCOPE_CRICHANNELS`` – Support for non-buffered channels

Implementation
==============

- Serial interface: ``apps/logging/nxscope/nxscope_iser.c``
- UDP interface: ``apps/logging/nxscope/nxscope_iudp.c``
- Dummy interface: ``apps/logging/nxscope/nxscope_idummy.c``
- Default serial protocol: ``apps/logging/nxscope/nxscope_pser.c``

Serial Protocol
---------------

The default serial protocol (``nxscope_pser.c``) uses a simple frame format
with a header, data payload, and a CRC-16 footer.

Frame Format
~~~~~~~~~~~~

+----------+-----------+-----------+------------+-------------+
| SOF (1B) | Len (2B)  | ID (1B)   | Data (nB)  | CRC-16 (2B) |
+==========+===========+===========+============+=============+
| ``0x55`` | u16 (LE)  | u8        | ...        | u16 (BE)    |
+----------+-----------+-----------+------------+-------------+

- **SOF**: Start of Frame, always ``0x55``.
- **Len**: Total frame length (including header and footer) in little-endian.
- **ID**: Frame type identifier (see ``enum nxscope_hdr_id_e``).
- **Data**: Variable length data payload.
- **CRC-16**: CRC-16-XMODEM (polynomial ``0x1021``) of the header and data.

Frame Types
~~~~~~~~~~~

The following frame types are defined in ``enum nxscope_hdr_id_e``:

+----------------------------+----+--------------------------------------------+
| Type                       | ID | Description                                |
+============================+====+============================================+
| ``NXSCOPE_HDRID_STREAM``   | 1  | Real-time stream data                      |
+----------------------------+----+--------------------------------------------+
| ``NXSCOPE_HDRID_CMNINFO``  | 2  | Get common info (chmax, flags, padding)    |
+----------------------------+----+--------------------------------------------+
| ``NXSCOPE_HDRID_CHINFO``   | 3  | Get channel info (name, type, etc.)        |
+----------------------------+----+--------------------------------------------+
| ``NXSCOPE_HDRID_ACK``      | 4  | ACK/NACK response for set requests         |
+----------------------------+----+--------------------------------------------+
| ``NXSCOPE_HDRID_START``    | 5  | Start or stop the data stream              |
+----------------------------+----+--------------------------------------------+
| ``NXSCOPE_HDRID_ENABLE``   | 6  | Enable or disable specific channels        |
+----------------------------+----+--------------------------------------------+
| ``NXSCOPE_HDRID_DIV``      | 7  | Set samples divider for channels           |
+----------------------------+----+--------------------------------------------+
| ``NXSCOPE_HDRID_USER``     | 8  | User defined frames                        |
+----------------------------+----+--------------------------------------------+

Examples
========

The following applications in NuttX demonstrate the use of the NxScope library:

1. :doc:`../../examples/nxscope/index` - Basic streaming example.

2. :doc:`../../industry/foc/index` - Real-time FOC (Field Oriented Control)
   telemetry.

3. :doc:`../../system/adcscope/index` - ADC data visualization.

4. :doc:`../../system/sensorscope/index` - Sensor data streaming.

Supported Boards
================

Several board configurations in NuttX use the NxScope library:

- **nrf52832-dk:nxscope_rtt**: Uses Segger RTT as the transport interface.
  This allows for high-speed data streaming without using a physical UART
  or USB cable (requires a J-Link debugger).

- **nrf52832-dk:nxscope_uart**: Uses a physical UART interface with a high
  baud rate (1Mbps) for data streaming.

- **stm32f4discovery:nxscope_cdcacm**: Uses USB CDC/ACM for the transport
  interface, providing a standard serial connection over USB.

- **qemu-intel64:jumbo**: Uses the UDP interface for high-speed data streaming
  over an emulated Ethernet connection.

- **thingy53:sensors_cpuapp**: Uses USB CDC/ACM with the :doc:`sensorscope
  <../../system/sensorscope/index>` application to stream data from
  on-board sensors.

- **sim:nxscope**: A configuration for the NuttX simulator that allows for
  easy testing and development of NxScope-related features on a host PC.

- **nucleo-c071rb:adcscope**: Uses UART and ST-LINK VCOM with the
  :doc:`adcscope <../../system/adcscope/index>` application to stream
  data from ADC channels.

External Tools
==============

- `Nxslib <https://github.com/railab/nxslib>`_ - Python client library.

- `Nxscli <https://github.com/railab/nxscli>`_ - Core command-line interface.

- `Nxscli-mpl <https://github.com/railab/nxscli-mpl>`_ - Matplotlib extension
  for nxscli.

- `Nxscli-pqg <https://github.com/railab/nxscli-pqg>`_ - PyQtGraph extension
  for nxscli.

Nxscli Quick Start
------------------

The `Nxscli <https://github.com/railab/nxscli>`_ is a Python tool used to
interface with NxScope-enabled devices. It uses a plugin-based architecture
to capture, store, and visualize data.

.. note::
   This is a quick command reference only. For a detailed description of all
   supported features, plugins, and configuration options, please visit the
   official repositories.

.. note::
   A graphical user interface (GUI) for ``nxscli`` is currently under
   development and will be available soon.

1. **Installation**:

   .. code-block:: bash

      # Core tool
      pip install nxscli
      
      # Matplotlib extension (optional)
      pip install nxscli-mpl
      
      # PyQtGraph extension (optional)
      pip install nxscli-pqg

2. **Interface selection**:

   Select the interface to connect to your device:

   .. code-block:: bash

      # Serial port
      nxscli serial <serial-port> ...
      
      # Segger RTT
      nxscli rtt <rtt-target> <rtt-buffer-index> <rtt-buffer-size>...
      
      # UDP (Ethernet)
      nxscli udp <target-ip> <target-port> ...

      # Simulated/Dummy interface
      nxscli dummy ...

3. **Device information**:

   To display information about the connected NxScope device and its
   available channels:

   .. code-block:: bash

      # Serial interface
      nxscli serial /dev/ttyACM0 pdevinfo

4. **Data capture**:

   Configure channels and capture samples:

   .. code-block:: bash

      # Print samples from channels 0 and 1 via serial
      nxscli serial /dev/ttyACM0 chan 0,1 pprinter

5. **Visualization**:

   NxScope supports live visualization using Matplotlib or PyQtGraph:

   .. code-block:: bash

      # Live plot from serial using Matplotlib
      nxscli serial /dev/ttyACM0 chan 0,1 m_live

      # Live plot from serial using PyQtGraph
      nxscli serial /dev/ttyACM0 chan 0,1 q_live

6. **Data streaming**:

   To stream data over UDP (e.g., to PlotJuggler):

   .. code-block:: bash

      nxscli serial /dev/ttyACM0 chan 0,1 pudp 0
