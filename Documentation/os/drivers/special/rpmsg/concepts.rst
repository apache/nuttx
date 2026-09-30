RPMsg Core Concepts
===================

Overview
--------

Remote Processor Messaging (RPMsg) is a lightweight messaging framework for
inter-processor communication (IPC) in Asymmetric Multiprocessing (AMP) systems.
It enables cores running different OSes (Linux, RTOS) to exchange data efficiently.

Application Scenarios
---------------------

Heterogeneous AMP (Big-Little Cores)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

::

    ┌─────────────────┐                    ┌─────────────────┐
    │   Big Core      │      SPI           │  Little Core A  │
    │   (Linux)       │◄──────────────────►│    (RTOS)       │
    └─────────────────┘                    └────────┬────────┘
                                                    │
                                                    │ VirtIO
                                                    │ (Shared Memory)
                                                    ▼
                                           ┌─────────────────┐
                                           │  Little Core B  │
                                           │    (RTOS)       │
                                           └─────────────────┘

- Big Core ↔ Little Core A: RPMsg over SPI (cross-chip)
- Little Core A ↔ Little Core B: RPMsg over VirtIO (on-chip)

Homogeneous AMP (Peer Cores)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

::

    ┌──────────┐     VirtIO      ┌──────────┐
    │  Core 0  │◄───────────────►│  Core 1  │
    │  (RTOS)  │                 │  (RTOS)  │
    └────┬─────┘                 └────┬─────┘
         │                            │
         │         VirtIO             │
         └────────────┬───────────────┘
                      │
                      ▼
               ┌──────────┐
               │  Core 2  │
               │  (RTOS)  │
               └──────────┘

All cores communicate via shared memory-based VirtIO.

Layered Architecture
--------------------

::

    ┌─────────────────────────────────────────────────────────┐
    │                    Services Layer                       │
    │   ┌─────────────────┐       ┌─────────────────┐         │
    │   │  RPMsg Socket   │       │    RPMsg FS     │         │
    │   │ (BSD Socket API)│       │   (VFS Access)  │         │
    │   └─────────────────┘       └─────────────────┘         │
    ├─────────────────────────────────────────────────────────┤
    │                    Framework Layer                      │
    │   ┌──────────┐  ┌──────────┐  ┌──────────────────┐      │
    │   │ Endpoint │  │ Channel  │  │ Service Discovery│      │
    │   │   Mgmt   │  │   Mgmt   │  │    & Routing     │      │
    │   └──────────┘  └──────────┘  └──────────────────┘      │
    ├─────────────────────────────────────────────────────────┤
    │                    Transport Layer                      │
    │   ┌───────┐  ┌───────┐  ┌───────┐  ┌─────────────┐      │
    │   │ Rptun │  │ UART  │  │  SPI  │  │   Router    │      │
    │   │VirtIO │  │       │  │       │  │  (Logical)  │      │
    │   └───────┘  └───────┘  └───────┘  └─────────────┘      │
    ├─────────────────────────────────────────────────────────┤
    │                    Physical Layer                       │
    │   ┌────────────┐  ┌────────────┐  ┌────────────┐        │
    │   │Shared Mem  │  │ UART HW    │  │  SPI HW    │        │
    │   │ + Interrupt│  │ Controller │  │ Controller │        │
    │   └────────────┘  └────────────┘  └────────────┘        │
    └─────────────────────────────────────────────────────────┘

Layer Descriptions
~~~~~~~~~~~~~~~~~~

1. **Services Layer**: High-level APIs for applications

   - RPMsg Socket: BSD Socket-like interface for stream communication
   - RPMsg FS: VFS interface for file-like resource access

2. **Framework Layer**: Core RPMsg functionality

   - Endpoint/Channel lifecycle management
   - Name/address-based service discovery
   - VFS character device registration

3. **Transport Layer**: Message transmission implementations

   - Rptun/VirtIO: High-performance shared memory (recommended)
   - UART: Low-speed cross-chip communication
   - SPI: Medium-speed cross-chip communication
   - Router: Logical routing across domains

4. **Physical Layer**: Hardware interaction

   - Shared memory configuration
   - DMA controller management
   - Hardware interrupt handling

Message Encapsulation
---------------------

::

    Application Data
    ┌─────────────────────────────────────┐
    │           Payload                   │
    └─────────────────────────────────────┘
                    │
                    ▼ Framework Layer adds header
    ┌───────────────┬─────────────────────┐
    │  RPMsg Header │      Payload        │
    │ (src,dst,len) │                     │
    └───────────────┴─────────────────────┘
                    │
                    ▼ Transport Layer adds header
    ┌───────┬───────────────┬─────────────┐
    │VirtIO │  RPMsg Header │   Payload   │
    │Header │               │             │
    └───────┴───────────────┴─────────────┘

Each layer adds its own header for routing and processing.

Workflow
--------

Channel Establishment
~~~~~~~~~~~~~~~~~~~~~

**Name-based Matching (Dynamic Address)**::

    Core A                              Core B
       │                                   │
       │ create_ept(name="svc")            │ create_ept(name="svc")
       │ src=ANY, dst=ANY                  │ src=ANY, dst=ANY
       ▼                                   ▼
    ┌──────┐  1. NS Announce           ┌──────┐
    │ Ept  │──────────────────────────►│ Ept  │
    │  A   │                           │  B   │
    │      │  2. NS Response           │      │
    │      │◄──────────────────────────│      │
    │      │                           │      │
    │      │  3. Address Allocated     │      │
    │      │◄─────────────────────────►│      │
    └──────┘                           └──────┘
       │                                   │
       └───────── Channel Ready ───────────┘

**Address-based Matching (Static Address)**::

    Core A                              Core B
       │                                   │
       │ create_ept(name="svc")            │ create_ept(name="svc")
       │ src=0x100, dst=0x200              │ src=0x200, dst=0x100
       ▼                                   ▼
    ┌──────┐                           ┌──────┐
    │ Ept  │◄─────────────────────────►│ Ept  │
    │  A   │     Direct Connection     │  B   │
    └──────┘                           └──────┘
       │                                   │
       └───────── Channel Ready ───────────┘

Sending Messages
~~~~~~~~~~~~~~~~

**Standard Send**::

    App: rpmsg_send(ept, data, len)
         │
         ▼
    ┌─────────────┐
    │  Copy data  │  ← Memory copy occurs
    │  to buffer  │
    └──────┬──────┘
           │
           ▼
    Send to remote

**Zero-Copy Send** (Recommended for large data)::

    App: buf = rpmsg_get_tx_payload_buffer(ept)
         │
         ▼
    ┌─────────────┐
    │ Write data  │  ← No copy, direct write
    │ to buffer   │
    └──────┬──────┘
           │
    App: rpmsg_send_nocopy(ept, buf, len)
           │
           ▼
    Send to remote

Receiving Messages
~~~~~~~~~~~~~~~~~~

::

    Sender Core                         Receiver Core
    ┌─────────┐                         ┌─────────┐
    │   App   │                         │   App   │
    └────┬────┘                         └────▲────┘
         │ rpmsg_send()                      │ callback()
         ▼                                   │
    ┌─────────┐                         ┌────┴────┐
    │Framework│                         │Framework│
    └────┬────┘                         └────▲────┘
         │                                   │ dispatch
         ▼                                   │
    ┌─────────┐                         ┌────┴────┐
    │Transport│                         │RX Thread│
    └────┬────┘                         └────▲────┘
         │                                   │ wake up
         ▼                                   │
    ┌─────────┐    Shared Memory       ┌────┴────┐
    │Physical │ ─────────────────────► │   ISR   │
    └─────────┘    + Interrupt         └─────────┘

RX Thread Processing Model
~~~~~~~~~~~~~~~~~~~~~~~~~~

::

    ┌─────────────────────────────────────────────────────┐
    │                    RX Thread                         │
    │                                                      │
    │   while (true) {                                     │
    │       msg = get_message_from_vring();               │
    │       ept = find_endpoint(msg->dst);                │
    │       ept->callback(msg);  ← Serial execution       │
    │   }                                                  │
    │                                                      │
    └─────────────────────────────────────────────────────┘

    Messages from same remote core → Same RX thread → FIFO order

Key Design Considerations
-------------------------

FIFO Order Guarantee
~~~~~~~~~~~~~~~~~~~~

Messages within a single link are processed in strict FIFO order.

Callback Blocking Risk
~~~~~~~~~~~~~~~~~~~~~~

::

    ┌─────────────────────────────────────────────────────┐
    │  WARNING: Blocking callbacks affect ALL messages    │
    │                                                      │
    │  RX Thread processes messages serially:             │
    │                                                      │
    │  [Msg1] → [Msg2] → [Msg3] → [Msg4]                  │
    │     │                                                │
    │     └─► If callback blocks here,                    │
    │         Msg2, Msg3, Msg4 are delayed!               │
    └─────────────────────────────────────────────────────┘

**Best Practices**:

- Keep callbacks short and fast
- Offload heavy work to worker threads
- Use multiple channels for isolation
- Implement priority scheduling if needed

Transport Layer Comparison
--------------------------

::

    ┌────────────┬─────────────┬───────────┬──────────────┐
    │ Transport  │   Medium    │ Bandwidth │   Use Case   │
    ├────────────┼─────────────┼───────────┼──────────────┤
    │ Rptun      │ Shared Mem  │   High    │ On-chip IPC  │
    │ VirtIO     │ + Interrupt │           │ (Preferred)  │
    ├────────────┼─────────────┼───────────┼──────────────┤
    │ UART       │ UART HW     │   Low     │ Cross-chip   │
    │            │             │           │ (Simple)     │
    ├────────────┼─────────────┼───────────┼──────────────┤
    │ SPI        │ SPI HW      │  Medium   │ Cross-chip   │
    │            │             │           │ (Faster)     │
    ├────────────┼─────────────┼───────────┼──────────────┤
    │ Router     │ Logical     │   N/A     │ Cross-domain │
    │            │             │           │ Routing      │
    └────────────┴─────────────┴───────────┴──────────────┘

