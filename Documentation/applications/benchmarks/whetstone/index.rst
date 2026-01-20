=========================================
``whetstone`` Whetstone FPU Benchmark
=========================================

Overview
========

Whetstone is a classic synthetic benchmark program designed to evaluate
processor arithmetic performance. By executing a series of standardized
computational tasks, it provides accurate assessment of floating-point
and integer operation capabilities.

This implementation is a C converted version of the Double Precision
Whetstone benchmark, based on the work by Rich Painter (Painter Engineering,
Inc.) from the netlib.org version.

The test results help developers:

- Quantify FPU (Floating-Point Unit) performance
- Analyze the impact of compiler optimization levels (-O2, -O3, etc.)
  on code execution efficiency
- Compare arithmetic performance across different hardware platforms
  or system configurations

Configuration
=============

Before running the test, enable the following Kconfig options:

.. code-block:: bash

   # Enable custom permissive license components
   CONFIG_ALLOW_CUSTOM_PERMISSIVE_COMPONENTS=y

   # Enable Whetstone benchmark
   CONFIG_BENCHMARK_WHETSTONE=y

   # Whetstone requires floating-point support in the C library
   CONFIG_LIBC_FLOATINGPOINT=y

Additional configuration options:

- ``CONFIG_BENCHMARK_WHETSTONE_PROGNAME`` - Program name (default: "whetstone")
- ``CONFIG_BENCHMARK_WHETSTONE_PRIORITY`` - Task priority (default: 100)
- ``CONFIG_BENCHMARK_WHETSTONE_STACKSIZE`` - Stack size (default: DEFAULT_TASK_STACKSIZE)

Usage
=====

Command Syntax
--------------

.. code-block:: bash

   whetstone [-c] [loops]

Parameters
----------

- ``[loops]`` - Module loop count. Sets the number of iterations for each
  internal test module. Increasing this value significantly increases
  computation and execution time. Default: 1000
- ``-c`` - Continuous mode. When specified, the benchmark repeats indefinitely
  until interrupted. Default: disabled

Examples
--------

Run a standard test with default parameters (1000 loops):

.. code-block:: bash

   nsh> whetstone

Increase computation load per module (100000 loops):

.. code-block:: bash

   nsh> whetstone 100000

Run in continuous mode with custom loop count:

.. code-block:: bash

   nsh> whetstone -c 100000

Output Interpretation
=====================

After completion, whetstone outputs test configuration, total duration,
and the final performance score.

Example Output
--------------

.. code-block:: bash

   nsh> whetstone 100000

   Loops: 100000, Iterations: 1, Duration: 5765 millisecond.
   C Converted Double Precision Whetstones: 1.7 MIPS

- **Loops: 100000** - Each module executed 100,000 loop iterations
- **Iterations: 1** - The test suite ran 1 round
- **Duration: 5765 millisecond** - Total execution time
- **1.7 MIPS** - Final performance score

Performance Metrics
-------------------

**MIPS / KIPS**

- **Definition**: Whetstone performance units - MIPS (Mega Whetstone
  Instructions Per Second) and KIPS (Kilo Whetstone Instructions Per Second)
- **Calculation**: ``KIPS = (100.0 * loops * iterations) / (duration_sec * 1000)``
- **Interpretation**: Higher scores indicate better processor arithmetic
  performance
- **Unit conversion**: Results display as KIPS when below 1000, otherwise
  as MIPS

Test Modules
============

The Whetstone benchmark consists of 11 carefully designed computational
modules covering different operation types:

.. list-table::
   :header-rows: 1

   * - Module
     - Description
   * - Module 1
     - Simple identifiers - basic floating-point operations
   * - Module 2
     - Array elements - array-based floating-point operations
   * - Module 3
     - Array as parameter - procedure calls with array arguments
   * - Module 4
     - Conditional jumps - branch operations
   * - Module 5
     - (Omitted)
   * - Module 6
     - Integer arithmetic - complex integer operations
   * - Module 7
     - Trigonometric functions - sin, cos, atan calculations
   * - Module 8
     - Procedure calls - function call overhead
   * - Module 9
     - Array references - array indexing operations
   * - Module 10
     - Integer arithmetic - simple integer operations
   * - Module 11
     - Standard functions - chained math functions (sqrt, exp, log)

Notes
=====

- This benchmark uses double-precision floating-point arithmetic
- For accurate measurements, ensure the system is not under heavy load
- If "Insufficient duration" is reported, increase the loop count
- Timing precision is in milliseconds, enabling quick and accurate results
  even with fewer loop iterations on high-performance embedded CPUs

References
==========

- Original Whetstone benchmark: H.J. Curnow and B.A. Wichmann,
  "A Synthetic Benchmark", The Computer Journal, Vol 19, No 1,
  February 1976, pp. 43-49
- netlib.org Whetstone: https://www.netlib.org/benchmark/whetstone.c
