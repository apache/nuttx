=======
TI AM67
=======

- **Processor** (`TI AM67A <https://www.ti.com/product/AM67A/>`__)
   - Quad-core 64-bit ARM Cortex-A53 @1.4 GHz for running high-level operating
     systems such as Linux
   - Dual single-core ARM Cortex-R5F @800 MHz for running real-time MCU
     applications
   - Dual 2 TOPS (4 TOPS total) deep learning accelerators for running vision
     applications
   - Advanced 50 GFLOPS GPU for high-performance graphics processing

The TI AM67 platform integrates a dual‑domain architecture comprising:

- **High‑Performance Domain –** Four Cortex‑A53 cores run a real‑time Linux
  operating system. These cores deliver the computational throughput required
  for intensive workloads such as image‑processing pipelines, computer‑vision
  algorithms, and other application‑level tasks.

- **Safety‑Critical Domain –** Dedicated Cortex‑R5F cores execute NuttX, a
  deterministic, low‑latency RTOS optimized for hard real‑time control. This
  domain handles safety‑critical functions, including the autopilot control
  loop.

.. warning::

   This chip currently only supports a basic implementation of NuttX with
   only UART console as a supported peripheral. Please see the contributing
   documentation if you would like to help contribute to the support.

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
