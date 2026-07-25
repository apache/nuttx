====================
STM32H7 DAC Driver
====================

.. tags:: chip:stm32h7, arch:arm, vendor:st, peripheral:dac

The STM32H7 DAC is a 12-bit, voltage-output, dual-channel digital-to-analog
converter embedded in the STM32H7 MCU. The driver supports two operating modes:

  *Basic (direct write) mode* — the application writes individual samples
  via the standard POSIX ``write()`` call.

  *DMA mode* — the DAC outputs samples from an internal DMA buffer at a
  rate determined by a timer. DMA mode itself has two sub-modes: circular
  (fixed buffer replayed periodically) and stream (double-buffered with
  half-transfer interrupts for real-time refill).

Hardware Features
=================

- 12-bit resolution
- 2 independent channels (DAC1_CH1 on PA4, DAC1_CH2 on PA5)
- Built-in output buffer for each channel
- DMA with timer-triggered transfers
- Configurable DMA buffer size per channel (``CONFIG_STM32_DACxCHy_DMA_BUFFER_SIZE``)
- Stream mode with half-transfer interrupts for double-buffering
- Timer range on H7: 1–15, with configurable output frequency

Driver Interface
================

The STM32H7 DAC driver follows the NuttX upper-half/lower-half DAC
architecture.  Board-level initialization calls ``stm32_dacinitialize()``
and registers the device with ``dac_register()``:

.. code-block:: c

   #include <nuttx/config.h>
   #include <nuttx/analog/dac.h>
   #include "stm32_dac.h"

   int board_dac_initialize(void)
   {
     struct dac_dev_s *dac;

   #ifdef CONFIG_STM32_DAC1CH1
     dac = stm32_dacinitialize(0);
     if (dac == NULL)
       return -ENODEV;

     ret = dac_register("/dev/dac0", dac);
     if (ret < 0)
       return ret;
   #endif

     return OK;
   }

The DAC is accessed through a character device in ``/dev`` (e.g. ``/dev/dac0``).
Standard file operations and driver-specific ``ioctl`` commands are used for
control and data transfer.

Operating Modes
===============

Basic Mode (direct write)
-------------------------

In basic mode the application calls ``write()`` with a ``dac_msg_s``
structure for each sample.  The upper-half driver queues the message
and the lower-half interrupt handler writes it to the DAC data register.

.. code-block:: c

   #include <nuttx/analog/dac.h>
   #include <fcntl.h>

   int fd = open("/dev/dac0", O_WRONLY);
   if (fd < 0)
     return ERROR;

   struct dac_msg_s msg;
   msg.am_channel = 0;          /* channel 1 */
   msg.am_data    = 2048;       /* mid-scale (0–4095) */

   write(fd, &msg, sizeof(msg));

   close(fd);

Each ``write()`` call delivers one sample.  For continuous waveforms a
timer or loop is needed; for higher update rates use DMA mode.

DMA Mode
--------

In DMA mode a timer periodically triggers the DAC to read the next sample
from the DMA buffer.  The buffer is split into two halves for stream mode
(double-buffering).  The ``ANIOC_DAC_INFO`` command returns the buffer
configuration:

.. code-block:: c

   #include <nuttx/analog/dac.h>
   #include <sys/ioctl.h>

   struct dac_info_s info;
   ioctl(fd, ANIOC_DAC_INFO, (unsigned long)&info);

   /* info.dma_buffer_size  — total DMA buffer in samples
    * info.dma_timer_frequency — timer frequency in Hz
    * The half length is dma_buffer_size / 2.
    * The per-sample rate is dma_timer_frequency / dma_buffer_size.
    */


Circular sub-mode (``halfint = 0``)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The DMA engine cycles through the entire buffer endlessly.  Only
transfer-complete interrupts are enabled; no events are delivered
to the application.  The application must pre-fill the **entire**
buffer before starting.

**Example:** fill the full DMA buffer with a sawtooth and start circular
output:

.. code-block:: c

   #include <nuttx/analog/dac.h>
   #include <sys/ioctl.h>

   struct dac_info_s info;
   ioctl(fd, ANIOC_DAC_INFO, (unsigned long)&info);

   uint16_t *buf = malloc(info.dma_buffer_size * sizeof(uint16_t));

   for (int i = 0; i < info.dma_buffer_size; i++)
     buf[i] = (uint16_t)((uint32_t)i * 4095 / info.dma_buffer_size);

   /* Copy the whole buffer into the internal DMA buffer */
   ioctl(fd, ANIOC_DAC_DMABUFF_INIT, (unsigned long)buf);

   /* Start circular output */
   struct dac_dma_start_s start;
   start.halfint = 0;
   ioctl(fd, ANIOC_DAC_DMA_START, (unsigned long)&start);

   /* Let it run ... */
   sleep(5);

   ioctl(fd, ANIOC_DAC_DMA_STOP, 0);
   close(fd);
   free(buf);

The total DMA buffer length is configured by
``CONFIG_STM32_DACxCHy_DMA_BUFFER_SIZE`` (default 256).  The
``DMAHBUF_WRITE`` command can also be used to write each half
separately instead of ``DMABUFF_INIT``:

.. code-block:: c

   /* Equivalent to one DMABUFF_INIT using two half writes */
   uint32_t half = info.dma_buffer_size / 2;

   struct dac_dma_event_s ev;
   ev.buffer = first_half_data;   /* half 0 */
   ev.half   = 0;
   ioctl(fd, ANIOC_DAC_DMAHBUF_WRITE, (unsigned long)&ev);

   ev.buffer = second_half_data;  /* half 1 */
   ev.half   = 1;
   ioctl(fd, ANIOC_DAC_DMAHBUF_WRITE, (unsigned long)&ev);

Stream sub-mode (``halfint = 1``)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Both half-transfer (HTIF) and transfer-complete (TCIF) interrupts are
enabled.  Each time a half-transfer finishes the driver posts an event
to a semaphore.  The application calls ``GET_EVENT`` (blocking) to
obtain the index of the completed half, writes new data into that half
via ``DMAHBUF_WRITE``, and the DMA engine continues filling the other
half — enabling continuous real-time waveform synthesis.

**Initial fill:** both halves must be written before starting the DMA.

**Refill:** after each ``GET_EVENT`` call, write **only one half**
(the half whose index was returned).

**Example:** DDS sine wave generation using stream mode:

.. code-block:: c

   #include <nuttx/analog/dac.h>
   #include <sys/ioctl.h>

   struct dac_info_s info;
   ioctl(fd, ANIOC_DAC_INFO, (unsigned long)&info);

   uint32_t half_len = info.dma_buffer_size / 2;
   uint16_t *workbuf = malloc(half_len * sizeof(uint16_t));
   int phase_acc = 0;

   /* Fill both halves with the initial sine data */

   for (int j = 0; j < 2; j++)
     {
       for (int i = 0; i < half_len; i++)
         {
           workbuf[i] = sine_table[phase_acc >> 12];
           phase_acc  = (phase_acc + phase_step) & (1024 * 4096 - 1);
         }
       struct dac_dma_event_s ev;
       ev.buffer = workbuf;
       ev.half   = j;
       ioctl(fd, ANIOC_DAC_DMAHBUF_WRITE, (unsigned long)&ev);
     }

   /* Start stream mode */
   struct dac_dma_start_s start;
   start.halfint = 1;
   ioctl(fd, ANIOC_DAC_DMA_START, (unsigned long)&start);

   /* Refill loop — write one half per event */
   volatile int stop = 0;

   while (!stop)
     {
       struct dac_dma_event_s ev;
       int ret = ioctl(fd, ANIOC_DAC_DMA_GET_EVENT,
                       (unsigned long)&ev);
       if (ret < 0)
         break;                 /* e.g. -EINTR from signal */

       /* Fill the completed half with new sine samples */
       for (int i = 0; i < half_len; i++)
         {
           workbuf[i] = sine_table[phase_acc >> 12];
           phase_acc  = (phase_acc + phase_step) & (1024 * 4096 - 1);
         }

       ev.buffer = workbuf;      /* write to the half returned by GET_EVENT */
       ioctl(fd, ANIOC_DAC_DMAHBUF_WRITE, (unsigned long)&ev);
     }

   ioctl(fd, ANIOC_DAC_DMA_STOP, 0);
   close(fd);
   free(workbuf);

DMA Structures
==============

These structures are defined in ``include/nuttx/analog/dac.h``.

.. c:struct:: dac_dma_start_s

Describes the DMA start configuration:

.. code-block:: c

   struct dac_dma_start_s
   {
     uint8_t halfint; /* 0 = circular (TC only),
                       * 1 = stream (HT + TC) */
   };

.. c:struct:: dac_dma_event_s

Carries a half-buffer pointer and half index for stream mode:

.. code-block:: c

   struct dac_dma_event_s
   {
     FAR uint16_t *buffer; /* data to write (DMAHBUF_WRITE) */
     int half;             /* 0 = first half, 1 = second half
                            * (returned by GET_EVENT,
                            *  set by caller for DMAHBUF_WRITE) */
   };

.. c:struct:: dac_info_s

Contains DAC capabilities and runtime state:

.. code-block:: c

   struct dac_info_s
   {
     uint8_t  sample_bits;          /* DAC resolution (12 bits) */
     uint8_t  dma_enabled;          /* 1 if DMA is currently running */
     uint8_t  halfint_enabled;      /* 1 if stream (HT) mode is active */
     uint32_t dma_buffer_size;      /* total DMA buffer length in samples */
     uint32_t dma_timer_frequency;  /* timer output frequency in Hz */
   };

DMA ioctl Commands
==================

These commands are defined in ``include/nuttx/analog/ioctl.h``.

The following commands are available when DMA is enabled
(``CONFIG_STM32_DAC1CH1_DMA`` or ``CONFIG_STM32_DAC1CH2_DMA``).
For the standard DAC ioctl commands see
:doc:`/components/drivers/character/analog/dac/index`.

.. c:macro:: ANIOC_DAC_DMABUFF_INIT

Copy the **entire** user buffer into the internal DMA buffer
(``dma_buffer_size`` samples).  The caller must provide a buffer
of exactly ``dma_buffer_size * sizeof(uint16_t)`` bytes.

   *IN*: ``uint16_t *`` — source buffer
   *OUT*: None

.. c:macro:: ANIOC_DAC_DMA_START

Start the DMA transfer.  The argument is a ``struct dac_dma_start_s *``.

   *IN*: ``struct dac_dma_start_s *``
   *OUT*: None

.. c:macro:: ANIOC_DAC_DMA_STOP

Stop the DMA transfer and the associated timer.

   *IN*: None
   *OUT*: None

.. c:macro:: ANIOC_DAC_DMA_GET_EVENT

Wait (blocking) for a half-transfer DMA event.  Returns the index of
the completed half (0 or 1) in ``dac_dma_event_s::half``.
Returns ``-EINTR`` if interrupted by a signal (e.g. Ctrl+C).

   *IN*: None
   *OUT*: ``struct dac_dma_event_s *`` — ``half`` set to the completed half

.. c:macro:: ANIOC_DAC_DMAHBUF_WRITE

Write data into **one half** of the internal DMA buffer.  The
``buffer`` field points to the user data; ``half`` selects which half
(0 = first, 1 = second) to overwrite.  The data length is always
``(dma_buffer_size / 2) * sizeof(uint16_t)`` bytes.

   *IN*: ``struct dac_dma_event_s *``
   *OUT*: None

.. c:macro:: ANIOC_DAC_INFO

Query DAC capabilities and current runtime state.  Returns the values
in a ``struct dac_info_s *``.  This command does not require DMA to be
enabled; it always reports the fixed resolution (12 bits) and,
depending on configuration, may return zero for DMA-related fields.

   *IN*: None
   *OUT*: ``struct dac_info_s *``

Configuration
=============

Each channel has its own Kconfig options:

.. list-table:: DAC Channel Configuration Options
   :widths: auto

   * - Name
     - Description
   * - ``CONFIG_STM32_DAC1CH1`` / ``CONFIG_STM32_DAC1CH2``
     - Enable DAC1 channel 1 / channel 2
   * - ``CONFIG_STM32_DAC1CH1_DMA`` / ``CONFIG_STM32_DAC1CH2_DMA``
     - Enable DMA for the channel
   * - ``CONFIG_STM32_DAC1CH1_DMA_BUFFER_SIZE``
     - DMA buffer size in samples (default 256)
   * - ``CONFIG_STM32_DAC1CH2_DMA_BUFFER_SIZE``
     - DMA buffer size in samples (default 256)
   * - ``CONFIG_STM32_DAC1CH1_DMA_PRIORITY_{LOW,MEDIUM,HIGH,VERYHIGH}``
     - DMA stream priority (default Medium)
   * - ``CONFIG_STM32_DAC1CH2_DMA_PRIORITY_{LOW,MEDIUM,HIGH,VERYHIGH}``
     - DMA stream priority (default Medium)
   * - ``CONFIG_STM32_DAC1CH1_TIMER``
     - Timer number for DMA trigger (range 1–15 on H7)
   * - ``CONFIG_STM32_DAC1CH2_TIMER``
     - Timer number for DMA trigger (range 1–15 on H7)
   * - ``CONFIG_STM32_DAC1CH1_TIMER_FREQUENCY``
     - Timer output frequency in Hz
   * - ``CONFIG_STM32_DAC1CH2_TIMER_FREQUENCY``
     - Timer output frequency in Hz

The corresponding timer peripheral must also be enabled:

.. code-block::

   CONFIG_STM32_TIM<n>=y

where ``<n>`` matches the timer number set in ``STM32_DAC1CH1_TIMER``
(or ``STM32_DAC1CH2_TIMER``).
