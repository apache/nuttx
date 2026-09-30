RPMsg Port Transport Layer
==========================

Overview
--------

RPMsg Port is an abstract transport layer in the RPMsg framework, designed to
support cross-SoC communication. It serves as an intermediate layer that can
accommodate various physical transport backends such as SPI and UART.

Architecture
~~~~~~~~~~~~

::

    ┌──────────────────────────────────────────────────────────────────┐
    │                       RPMsg Framework Layer                      │
    │                               ↓ ops                              │
    ├──────────────────────────────────────────────────────────────────┤
    │                       RPMsg Transport Layer                      │
    │  ┌────────────────────────────────────────────────────────────┐  │
    │  │                         RPMsg Port                         │  │
    │  │       ┌─────────────────┐        ┌─────────────────┐       │  │
    │  │       │   RPMsg UART    │        │    RPMsg SPI    │       │  │
    │  │       │      UART       │        │   SPI + GPIO    │       │  │
    │  │       └─────────────────┘        └─────────────────┘       │  │
    │  └────────────────────────────────────────────────────────────┘  │
    ├──────────────────────────────────────────────────────────────────┤
    │                       RPMsg Physical Layer                       │
    │           ┌──────────────┐            ┌──────────────┐           │
    │           │ UART Hardware│            │ SPI + GPIO   │           │
    │           │  Controller  │            │  Hardware    │           │
    │           └──────────────┘            └──────────────┘           │
    └──────────────────────────────────────────────────────────────────┘

As an abstraction layer, RPMsg Port extracts common transport functionality
and provides three main features:

1. **Buffer Management**: Unified buffer management mechanism for underlying
   transport layers
2. **RPMsg Framework Integration**: Helps underlying transport layers
   interface with the RPMsg framework, including:

   - Receiving data from RPMsg framework and passing to RPMsg Port SPI/UART
     for actual transmission
   - Dispatching received data from RPMsg Port SPI/UART to the framework
   - Basic Name Service implementation

3. **Physical Transport Abstraction**: Provides a consistent interface for
   different physical transport layers (SPI, UART, etc.)

Main Features
-------------

Buffer Management
~~~~~~~~~~~~~~~~~

During initialization, RPMsg Port creates separate memory pools for TX and RX
operations. Initially, all buffers are attached to their respective free lists.
During data transmission and reception, buffers flow between free and ready
lists.

::

    TX Queue                              RX Queue
    ┌─────────────────────────┐          ┌─────────────────────────┐
    │                         │          │                         │
    │  ┌───────────────────┐  │          │  ┌───────────────────┐  │
    │  │     Free List     │  │          │  │     Free List     │  │
    │  └───────────────────┘  │          │  └───────────────────┘  │
    │           ↑↓            │          │           ↑↓            │
    │  ┌───────────────────┐  │          │  ┌───────────────────┐  │
    │  │    Ready List     │  │          │  │    Ready List     │  │
    │  └───────────────────┘  │          │  └───────────────────┘  │
    │                         │          │                         │
    └─────────────────────────┘          └─────────────────────────┘

Buffer Layout
~~~~~~~~~~~~~

RPMsg Port managed buffers reserve a header space for underlying physical
transport layers. The RPMsg Port layer uses the space after the header.

::

                                ┌─────── RPMsg Port uses ───────┐
                                │                               │
    buffer: ┌───────────────────┬───────────────────────────────┐
            │ rpmsg port header │            data               │
            └───────────────────┴───────────────────────────────┘
            └────────────── RPMsg Port SPI/UART uses ───────────┘

Data Transmission
~~~~~~~~~~~~~~~~~

When RPMsg Services need to send data:

1. Get a buffer from TX free list
2. Fill data into the buffer
3. Add buffer to TX ready list
4. Notify RPMsg Port SPI/UART to fetch and send data
5. After transmission, return buffer to TX free list

::

  ┌────────────── RPMsg Services
  │               ▲
  │ 2. Add ready  │ 1. Get free buffer
  │               │
  │   ┌───────────│─────────────────────────────────────────────┐
  │   │           │           TX Queue                          │
  │   │           │                                             │
  │   │  ┌───────────────────────────────────────────────────┐  │
  │   │  │                    Free List                      │<─────┐
  │   │  └───────────────────────────────────────────────────┘  │   │
  │   │                                                         │   │
  │   │  ┌───────────────────────────────────────────────────┐  │   │
  └─────>│                   Ready List                      │  │   │
      │  └───────────────────────────────────────────────────┘  │   │
      │           │                                             │   │
      └───────────┼─────────────────────────────────────────────┘   │
                  │                                                 │
                  │ 3. Get ready buffer                             │
                  ▼                                                 │
                  RPMsg Port SPI/UART ──────────5. Free─────────────┘
                            │
                            │ 4. Send data
                            ▼
                 ┌──────────┴──────────┐
                 │                     │
           ┌──────────┐            ┌────────┐
           │ SPI+GPIO │            │  UART  │
           └──────────┘            └────────┘

Data Reception
~~~~~~~~~~~~~~

When RPMsg Port SPI/UART receives data:

1. Get a buffer from RX free list
2. Receive data into the buffer
3. Add buffer to RX ready list
4. Notify Port layer to process data
5. After processing, return buffer to RX free list

::

           ┌──────────┐          ┌────────┐
           │ SPI+GPIO │          │  UART  │
           └─────┬────┘          └────┬───┘
                 │                    │
                 └──────────┬─────────┘
                            │ 2. Receive data
                            ▼
  ┌─3. Add Ready── RPMsg Port SPI/UART
  │                ▲
  │                │ 1. Get free buffer
  │                │
  │   ┌────────────│────────────────────────────────────────────┐
  │   │            │            RX Queue                        │
  │   │            │                                            │
  │   │  ┌───────────────────────────────────────────────────┐  │
  │   │  │                     Free List                     │<─────┐
  │   │  └───────────────────────────────────────────────────┘  │   │
  │   │                                                         │   │
  │   │  ┌───────────────────────────────────────────────────┐  │   │
  └─────>│                    Ready List                     │  │   │
      │  └───────────────────────────────────────────────────┘  │   │
      │            │                                            │   │
      └────────────┼────────────────────────────────────────────┘   │
                   │                                                │
                   │ 4. Get ready buffer                            │
                   ▼                                                │
          ┌──────────────────┐                                      │
          │    RX Thread     │                                      │
          │                  │                                      │
          │  - Find RPMsg    │                                      │
          │    Endpoint      │                                      │
     ┌─────> - ept->cb()     │                                      │
     │    │  - return buffer──────────5. Free───────────────────────┘
     │    └──────────────────┘
     │
     └─────> RPMsg Services

