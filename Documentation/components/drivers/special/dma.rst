===========
DMA Drivers
===========

Overview
========

The common DMA framework is a header-only interface defined in
``include/nuttx/dma/dma.h``. It was introduced in 2018 (``596f52c3ee``,
"Nuttx/dma: add dma framework for nuttx") with link transfers added in
2019 (``51475e4273``), per-channel DRQ selection in 2023 (``06ee8d9673``),
and source/destination address stepping in 2023 (``366628bf5a``).

There is no upper-half implementation file: ``drivers/dma/`` contains
only ``Kconfig`` and ``Make.defs``. The header defines the controller
and channel structures, the operation table, and thin macros through
which peripheral drivers (DMA clients) drive SoC-specific DMA
controller implementations. All hardware-specific functionality lives
in the controller implementation.

The framework is enabled with ``CONFIG_DMA`` (``drivers/dma/Kconfig``),
which selects ``ARCH_DMA``. Scatter/gather-style link transfers
additionally require ``CONFIG_DMA_LINK``.

Architecture
============

.. code-block:: text

   Peripheral driver (DMA client)
                |
                v
   DMA common interface (macros in include/nuttx/dma/dma.h)
                |
                v
   DMA controller implementation (struct dma_dev_s,
     per-channel struct dma_chan_s + struct dma_ops_s)
                |
                v
   DMA hardware

Terminology as used by the header:

- DMA controller: represented by ``struct dma_dev_s``. It owns a set of
  channels and hands them out on request.
- DMA channel: represented by ``struct dma_chan_s``. The client-visible
  portion is only an ``ops`` pointer; the controller implementation
  extends this structure with its own device-specific fields after it.
- DMA client: a peripheral driver that acquires a channel and starts
  transfers through the macros below (in-tree: ``drivers/audio/audio_dma.c``,
  ``drivers/serial/uart_16550.c``).
- DMA transfer: a single ``DMA_START*`` invocation described by a
  callback, a destination/source address pair, and lengths. There is no
  transfer descriptor structure in the framework.

Most DMA code under ``arch/`` (for example ``stm32_dma.c``,
``kinetis_edma.c``, ``cxd56_dmac.c``, ``rp2040_dmac.c``) does **not**
implement this framework; those drivers expose their own SoC-specific
APIs (for example ``stm32_dmastart()``, ``cxd56_dmastart()``). No
in-tree controller implements ``struct dma_dev_s`` yet, so a board
using the common framework must supply its own controller.

Controller Interface
====================

``struct dma_dev_s`` (``include/nuttx/dma/dma.h``) is the controller
vtable:

- ``get_chan(dev, ident)``: return the channel identified by ``ident``
  with mutually exclusive access. Per the header, the call waits until
  the current holder releases the channel with ``put_chan()``.
- ``put_chan(dev, chan)``: release a channel and wake any waiter. The
  released ``chan`` must not be used again until re-acquired.

``struct dma_ops_s`` is the per-channel operation table. Every entry is
invoked through a ``DMA_*`` macro taking the channel as first argument:

- ``config(chan, cfg)``: configure the channel from
  ``struct dma_config_s`` before use.
- ``start(chan, callback, arg, dst, src, len)``: one-shot transfer of
  ``len`` bytes from ``src`` to ``dst``; ``callback`` runs on completion.
- ``start_cyclic(chan, callback, arg, dst, src, len, period_len)``:
  cyclic transfer over ``len`` bytes; ``callback`` runs per
  ``period_len`` chunk.
- ``start_link(chan, callback, arg, work_mode, cfg)``
  (``CONFIG_DMA_LINK`` only): scatter/gather transfer described by
  ``struct dma_link_config_s``; ``callback`` runs when the link transfer
  finishes. ``work_mode`` is one of ``DMA_BLOCK_MODE``,
  ``DMA_SRC_LINK_MODE``, ``DMA_DST_LINK_MODE``, ``DMA_DUAL_LINK_MODE``.
- ``stop(chan)``: stop the transfer.
- ``pause(chan)`` / ``resume(chan)``: pause a transfer; ``resume()``
  must follow ``pause()`` to restart it.
- ``residual(chan)``: bytes remaining to be transferred.

``struct dma_config_s`` fields: ``direction`` (``DMA_MEM_TO_MEM``,
``DMA_MEM_TO_DEV``, ``DMA_DEV_TO_MEM``, ``DMA_DEV_TO_DEV``),
``priority``, ``timeout``, ``option``, ``dst_width`` / ``src_width``
(FIFO/register width in bytes: 1, 2, 4, 8), ``dst_drq`` / ``src_drq``
(physical DMA request IDs), ``dst_step`` / ``src_step`` (address shift
after each transfer; negative counts down). Zero means "keep the
current value".

``dma_callback_t`` (``void (*)(struct dma_chan_s *chan, void *arg,
ssize_t len)``): completion callback. ``chan`` is the finished channel,
``arg`` is the value passed to ``start*``, and ``len`` is the transfer
length on success or a negative error code on failure. The DMA module
performs no cache maintenance; on RX completion the client must
invalidate the DMA buffers.

Using DMA from a Driver
=======================

Client-facing macros (all in ``include/nuttx/dma/dma.h``):

- ``DMA_GET_CHAN(dev, ident)`` / ``DMA_PUT_CHAN(dev, chan)``: acquire /
  release a channel from a ``struct dma_dev_s *``.
- ``DMA_CONFIG(chan, cfg)``: apply a ``struct dma_config_s``.
- ``DMA_START(chan, callback, arg, dst, src, len)``: one-shot transfer.
- ``DMA_START_CYCLIC(chan, callback, arg, dst, src, len, period_len)``:
  cyclic transfer.
- ``DMA_START_LINK(chan, callback, arg, mode, link_cfg)``
  (``CONFIG_DMA_LINK`` only): link transfer.
- ``DMA_PAUSE(chan)`` / ``DMA_RESUME(chan)`` / ``DMA_STOP(chan)`` /
  ``DMA_RESIDUAL(chan)``: transfer control and status.

Address and cache rules, established by the header comments and both
in-tree clients:

- Addresses passed to ``start*`` are physical addresses. Both clients
  translate with ``up_addrenv_va_to_pa()`` (FIFO register addresses and
  DMA buffers alike).
- The DMA module performs no cache operations. The client cleans
  (``up_clean_dcache()``) TX buffers before ``DMA_START`` and
  invalidates (``up_invalidate_dcache()``) RX data after completion.
  See ``u16550_dmasend()`` / ``u16550_dmareceive()`` in
  ``drivers/serial/uart_16550.c`` and
  ``audio_dma_enqueuebuffer()`` / ``audio_dma_callback()`` in
  ``drivers/audio/audio_dma.c``.

Channel and Transfer Lifecycle
==============================

The framework defines no registration helper; the controller instance
reaches the client out of band (see the examples below). The observed
lifecycle is:

.. code-block:: text

   Acquire channel (DMA_GET_CHAN or equivalent)
                |
                v
   Configure channel (DMA_CONFIG)
                |
                v
   Start transfer (DMA_START / DMA_START_CYCLIC / DMA_START_LINK)
                |
                v
   Hardware execution, completion callback per transfer/period
                |
                v
   Pause/resume around flow control or underrun (optional)
                |
                v
   Stop (DMA_STOP) and/or release (DMA_PUT_CHAN)

Two in-tree patterns exist:

- One-shot TX: ``drivers/serial/uart_16550.c`` ``u16550_dmasend()``
  calls ``DMA_START`` once per UART TX buffer with completion callback
  ``u16550_dmasend_done()``, which reports the byte count via
  ``uart_xmitchars_done()`` and chains the next buffer, or retries the
  transfer if ``len`` indicates failure.
- Never-ending cyclic RX: ``u16550_dmarxfree()`` configures
  ``DMA_DEV_TO_MEM`` with a timeout derived from baud rate and starts
  ``DMA_START_CYCLIC`` once over the RX ring buffer with period
  ``dmarxsize / 4``; ``u16550_dmareceive_done()`` advances
  ``dmarxhead`` from the callback ``len`` and drains via
  ``uart_recvchars_dma()``. ``DMA_PAUSE()`` / ``DMA_RESUME()``
  implement RX flow control. ``drivers/audio/audio_dma.c`` follows the
  same cyclic model: ``audio_dma_configure()`` sets direction and FIFO
  width, ``audio_dma_start()`` calls ``DMA_START_CYCLIC`` over all
  audio buffers, ``audio_dma_callback()`` dequeues one finished buffer
  per period, and underrun pauses the channel until more buffers are
  enqueued. ``audio_dma_stop()`` calls ``DMA_STOP()`` and flushes the
  pending queue.

Gaps established from source (no in-tree use besides the header):

- ``DMA_PUT_CHAN()`` is never called in-tree; neither client releases
  its channel after acquisition.
- ``DMA_RESIDUAL()`` and ``DMA_START_LINK()`` have no in-tree callers.
- ``DMA_STOP()`` is used only by ``audio_dma``; the UART driver never
  stops its channels.

DMA Links
=========

When ``CONFIG_DMA_LINK`` is set, a transfer can walk arrays of
``struct dma_link_s`` (``addr`` plus ``link_num`` / ``link_size``)
bundled in ``struct dma_link_config_s`` (``dst_link_num``,
``src_link_num``, ``dst_link``, ``src_link``) instead of a single
contiguous buffer. The mode argument selects which side uses links:
``DMA_BLOCK_MODE``, ``DMA_SRC_LINK_MODE``, ``DMA_DST_LINK_MODE``,
``DMA_DUAL_LINK_MODE``.

This mechanism is optional and currently has no in-tree controller
implementation and no in-tree user; only ``CONFIG_DMA_LINK=y`` board
defconfigs (for example ``boards/xtensa/esp32/esp32-sparrow-kit``)
select the option. Controller authors should treat the link array
layout and ownership as defined solely by ``include/nuttx/dma/dma.h``
until an in-tree implementation exists.

Implementing a DMA Controller
=============================

A controller implementation must provide:

#. An instance of ``struct dma_dev_s`` with working ``get_chan`` /
   ``put_chan`` operations implementing the blocking-acquire,
   use-after-release forbidden semantics documented in the header.
#. Channel structures whose first member is ``struct dma_chan_s``
   (so a ``struct dma_chan_s *`` converts to the implementation
   structure), each carrying a populated ``struct dma_ops_s`` table.
#. At minimum the ``config`` / ``start`` / ``start_cyclic`` /
   ``stop`` / ``pause`` / ``resume`` operations used by the intended
   clients; ``residual`` and ``start_link`` are required only if
   clients use ``DMA_RESIDUAL()`` / ``DMA_START_LINK()`` (none do
   in-tree).

How the controller reaches the client depends on the client:

- Audio: ``audio_dma_initialize()``
  (``drivers/audio/audio_dma.c``, ``include/nuttx/audio/audio_dma.h``)
  takes a ``struct dma_dev_s *`` plus channel number directly and
  acquires the channel with ``DMA_GET_CHAN()``.
- 16550 UART: the UART obtains channels through the platform hook
  ``dmachan(priv, ident)`` in ``struct u16550_ops_s``
  (``include/nuttx/serial/uart_16550.h``), which is expected to return
  a ``struct dma_chan_s *``. The default hook, ``uart_dmachan()``, is
  declared but has no in-tree definition, so a platform using 16550
  DMA must either override ``ops->dmachan`` or provide its own
  ``uart_dmachan()`` (``drivers/serial/uart_pci_16550.c``
  shows the override pattern with a stub returning ``NULL``, meaning
  no DMA). Channel identities, RX buffer sizes, and RX timeouts come
  from ``CONFIG_16550_UARTn_DMA*`` options
  (``drivers/serial/Kconfig-16550``).

Existing Implementations
========================

- ``drivers/audio/audio_dma.c``: clearest end-to-end client. Shows
  acquire (``audio_dma_initialize()``), direction/width configuration,
  cyclic start over the buffer set, per-period dequeue in the callback,
  pause on underrun, resume on enqueue, and stop with queue flush.
- ``drivers/serial/uart_16550.c``: shows both one-shot TX chained from
  the completion callback and background cyclic RX with pause/resume
  flow control. Channel acquisition is delegated to the platform
  ``dmachan`` hook rather than ``DMA_GET_CHAN()``.
- ``arch/arm64/src/bcm2711/hardware/bcm2711_dma.h``: register
  definitions only, not a framework controller.
- Espressif ``lldesc_t`` / ``gdma_*`` code (for example
  ``arch/risc-v/src/common/espressif/esp_i2s.c``) implements its own
  descriptor chaining outside this framework and must not be confused
  with ``CONFIG_DMA_LINK`` link transfers.

Configuration
=============

- ``CONFIG_DMA``: enable the common DMA interface (selects
  ``ARCH_DMA``).
- ``CONFIG_DMA_LINK``: enable link-transfer structures and
  ``DMA_START_LINK()``.
- ``ARCH_DMA`` / ``ARCH_DMA_NO_FLASH_TRANSFER`` (``arch/Kconfig``): SoC
  capability selects.
- ``CONFIG_16550_UARTn_DMA``, ``CONFIG_16550_UARTn_DMA_TX``,
  ``CONFIG_16550_UARTn_DMA_RX``, ``CONFIG_16550_UARTn_DMA_RXBUFSIZE``,
  ``CONFIG_16550_UARTn_DMA_RXTIMEOUT``: 16550 UART DMA binding.
- ``CONFIG_AUDIO_BUFFER_NUMBYTES`` / ``CONFIG_AUDIO_NUM_BUFFERS``:
  default buffer geometry used by ``audio_dma_initialize()``.
