======================================
Coresight - HW Assisted Tracing on ARM
======================================

Overview
--------

Coresight is an umbrella of technologies allowing for the debugging of ARM
based SoC.  It includes solutions for JTAG and HW assisted tracing.  This
document is concerned with the latter.

HW assisted tracing is becoming increasingly useful when dealing with systems
that have many SoCs and other components like GPU and DMA engines. Developers
can monitor the behavior of their software as it runs on the device, view
real-time data about its execution, and identify and debug issues quickly.

Coresight omponents are generally categorised as source, link and sinks.
The source devices generats a compressed stream representing the processor
instruction path based on tracing scenarios. The link devices are responsible
for transferring the stream from the source device to the sink device. The sink
devices serve as as endpoints to the coresight implementation, either storing
the compressed stream in a memory buffer or creating an interface to the
outside world where data can be transferred to a host without fear of filling
up the onboard coresight memory buffer.

refer to the following document for more details:
https://developer.arm.com/documentation/102520/latest/


Acronyms and Classification
---------------------------

Acronyms:

PTM:
    Program Trace Macrocell
ETM:
    Embedded Trace Macrocell
STM:
    System trace Macrocell
ETB:
    Embedded Trace Buffer
ITM:
    Instrumentation Trace Macrocell
TPIU:
     Trace Port Interface Unit
TMC-ETR:
        Trace Memory Controller, configured as Embedded Trace Router
TMC-ETF:
        Trace Memory Controller, configured as Embedded Trace FIFO

Classification:

Source:
   ETM, STM, ITM
Link:
   Funnel, replicator, TMC-ETF
Sinks:
   ETB, TPIU, TMC-ETR

Framework and implementation
----------------------------

The coresight framework provides a central point to represent, configure and
manage coresight devices on a platform.  Any coresight compliant device can
register with the framework for as long as they use the right APIs:

.. c:function:: int coresight_register(FAR struct coresight_dev_s *csdev, FAR const struct coresight_desc_s *desc);
.. c:function:: void coresight_unregister(FAR struct coresight_dev_s *csdev);

``struct coresight_desc *desc`` describes the type of current coresight device
and where it connects to. When all the coresight devices are registered,
devices throught the tracing stream path can be enablea by calling:

.. c:function:: int coresight_enable(FAR struct coresight_dev_s *srcdev, FAR struct coresight_dev_s *destdev);

The ``coresight_enable`` function will build the path through srcdev and
destdev according the ``struct coresight_desc *desc``.
