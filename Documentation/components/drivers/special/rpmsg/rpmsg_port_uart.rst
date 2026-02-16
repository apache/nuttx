RPMsg Port UART
===============

Overview
--------

RPMsg Port UART is a transport layer implementation that enables RPMsg
communication between two SoCs connected via UART. This is useful when
shared memory is not available between chips, such as two separate chips
on the same PCB.

Hardware Requirements
---------------------

The UART interface requires four signals with hardware flow control support:

::

    ┌─────────────┐               ┌─────────────┐
    │   SoC A     │               │   SoC B     │
    │             │  TX ────────> │             │
    │    UART     │  RX <──────── │    UART     │
    │             │ RTS ────────> │             │
    │             │ CTS <──────── │             │
    └─────────────┘               └─────────────┘

Requirements:

- Hardware flow control (RTS/CTS) must be supported to prevent data loss
- RX pin should support wake-up for low power mode
- Data corruption during wake-up is not allowed, but data loss is acceptable
  (handled by software)

Software Architecture
---------------------

::

    ┌────────────────────────────────────────┐
    │           RPMsg Framework              │
    ├────────────────────────────────────────┤
    │              RPMsg Port                │
    │   (Buffer Management, Name Service)    │
    ├────────────────────────────────────────┤
    │           RPMsg Port UART              │
    │   (Escape Coding, Connection Protocol, │
    │   Data Transfer, Low Power Support)    │
    ├────────────────────────────────────────┤
    │           UART Driver                  │
    └────────────────────────────────────────┘

Protocol Details
----------------

Escape Coding
~~~~~~~~~~~~~

Special characters (0x70-0x7f) are used to distinguish commands from data:

.. code-block:: c

   #define RPMSG_PORT_UART_START      0x7f  /* Frame start */
   #define RPMSG_PORT_UART_END        0x70  /* Frame end */
   #define RPMSG_PORT_UART_ESCAPE     0x7c  /* Escape character */
   #define RPMSG_PORT_UART_CONNREQ    0x7e  /* Connection request */
   #define RPMSG_PORT_UART_CONNACK    0x7d  /* Connection acknowledge */
   #define RPMSG_PORT_UART_ESCAPE_MASK 0x20 /* Escape XOR mask */

Encoding example:

::

   Original:  [0x01, 0x7f, 0x02, 0x70, 0x03]
   Encoded:   [START] [0x01] [ESC] [0x5f] [0x02] [ESC] [0x50] [0x03] [END]
              [0x7f]  [0x01] [0x7c][0x5f] [0x02] [0x7c][0x50] [0x03] [0x70]

When a byte falls in the 0x70-0x7f range, it is escaped by sending the
ESCAPE character followed by the original byte XORed with 0x20.

Connection Protocol
~~~~~~~~~~~~~~~~~~~

The connection protocol supports:

- Either side can start first during system boot
- Reconnection after either side restarts

::

   Normal startup:
   ┌────────┐                      ┌────────┐
   │ CPU A  │                      │ CPU B  │
   └────────┘                      └────────┘
       │                               │
       │ ──── CONNREQ (0x7e) ────────> │
       │                               │ Initialize resources
       │ <──── CONNACK (0x7d) ──────── │
       │ Initialize resources          │
       │                               │
       │      Connection established   │

   Restart scenario (CPU B restarts):
   ┌────────┐                      ┌────────┐
   │ CPU A  │                      │ CPU B  │
   │(running)│                     │(restart)│
   └────────┘                      └────────┘
       │                               │
       │ <──── CONNREQ (0x7e) ──────── │
       │ Reinitialize resources        │
       │ ──── CONNACK (0x7d) ────────> │
       │                               │ Initialize resources
       │      Connection established   │

Data Frame Format
~~~~~~~~~~~~~~~~~

::

   ┌───────┬─────────────────────────────────┬───────┐
   │ START │   Header + Payload (escaped)    │  END  │
   │ 0x7f  │                                 │ 0x70  │
   └───────┴─────────────────────────────────┴───────┘

   Header structure (rpmsg_port_header_s):
   ┌────────┬────────┬────────┬────────┬──────────────┐
   │  crc   │  cmd   │ avail  │  len   │   payload    │
   │ 2 byte │ 2 byte │ 2 byte │ 2 byte │   N bytes    │
   └────────┴────────┴────────┴────────┴──────────────┘

- crc: CRC16 checksum (optional, enabled via CONFIG_RPMSG_PORT_UART_CRC)
- cmd: Command field (unused for data frames, set to 0)
- avail: Available space (unused for data frames, set to 0)
- len: Total length including header
- payload: Actual RPMsg data

Low Power Support
-----------------

RPMsg Port UART implements low power support without additional GPIOs using
special wake/sleep commands:

.. code-block:: c

   #define RPMSG_PORT_UART_STAYWAKE1     0x79  /* Wake request 1 */
   #define RPMSG_PORT_UART_STAYWAKEACK1  0x78  /* Wake acknowledge 1 */
   #define RPMSG_PORT_UART_STAYWAKE2     0x75  /* Wake request 2 */
   #define RPMSG_PORT_UART_STAYWAKEACK2  0x74  /* Wake acknowledge 2 */
   #define RPMSG_PORT_UART_RELAXWAKE     0x77  /* Allow sleep */

Wake/Sleep Flow
~~~~~~~~~~~~~~~

::

   ┌────────┐                           ┌────────┐
   │ CPU A  │                           │ CPU B  │
   │(sender)│                           │(asleep)│
   └────────┘                           └────────┘
       │                                    │
       │ Acquire TX PM lock                 │
       │ ──── STAYWAKE1 ──────────────────> │
       │                                    │ Acquire RX PM lock
       │ <──── STAYWAKEACK1 ─────────────── │
       │                                    │
       │ ════ Data Transfer ══════════════> │
       │                                    │
       │ ──── RELAXWAKE ──────────────────> │
       │ Release TX PM lock                 │ Release RX PM lock
       │                                    │

Ping-Pong Wake Mechanism
~~~~~~~~~~~~~~~~~~~~~~~~

Two sets of wake commands (STAYWAKE1/ACK1 and STAYWAKE2/ACK2) are used
alternately to prevent false wake acknowledgments:

::

   First transfer:  STAYWAKE1 → STAYWAKEACK1 → Data → RELAXWAKE
   Second transfer: STAYWAKE2 → STAYWAKEACK2 → Data → RELAXWAKE
   Third transfer:  STAYWAKE1 → STAYWAKEACK1 → Data → RELAXWAKE
   ...

This prevents the sender from mistakenly accepting a delayed ACK from a
previous transfer as confirmation for the current transfer.

Source Files
------------

- ``nuttx/drivers/rpmsg/rpmsg_port_uart.c``
