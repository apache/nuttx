======================================
``tflite-micro`` TensorFlow Lite Micro
======================================

TensorFlow Lite for Microcontrollers (TFLM) is a C++ interpreter for
running ``.tflite`` models on memory-constrained targets. NuttX integrates
it from ``apps/mlearning/tflite-micro``.

The build downloads a pinned TFLM snapshot from
https://github.com/tensorflow/tflite-micro and applies NuttX patches that
add INT8-only operator registrations and an ``extern "C"`` entry point for
the hello-world example.

Dependencies
============

``CONFIG_TFLITEMICRO`` depends on all of:

- ``CONFIG_SYSTEM_FLATBUFFERS`` — FlatBuffers headers (``apps/system/flatbuffers``)
- ``CONFIG_MATH_GEMMLOWP`` — gemmlowp (``apps/math/gemmlowp``)
- ``CONFIG_MATH_KISSFFT`` — KissFFT (``apps/math/kissfft``)
- ``CONFIG_MATH_RUY`` — Ruy (``apps/math/ruy``)

TFLM is C++, so the configuration also needs C++ support (``CONFIG_HAVE_CXX``
and a C++ standard library such as ``CONFIG_LIBCXX``).

Optional acceleration:

- ``CONFIG_MLEARNING_CMSIS_NN`` — replace reference kernels with CMSIS-NN
  implementations. See :doc:`../cmsis-nn/index`.
- ``CONFIG_ARM_NEON`` — when CMSIS-NN is enabled, a few kernels are taken
  from ``apps/mlearning/tflite-micro/operators/neon`` instead of CMSIS-NN.

Configuration
=============

From :menuselection:`Application Configuration --> Machine Learning Support`:

``CONFIG_TFLITEMICRO``
  Enable the TFLM library.

``CONFIG_TFLITEMICRO_DEBUG``
  Keep TFLM error strings and print memory-use information
  (``TF_LITE_SHOW_MEMORY_USE``). When disabled, ``TF_LITE_STRIP_ERROR_STRINGS``
  is set to reduce code size.

``CONFIG_TFLITEMICRO_SYSLOG``
  Route TFLM logging through NuttX syslog via ``tflm_syslog.cc``.
  ``CONFIG_TFLITEMICRO_SYSLOG_LEVEL`` selects the syslog priority
  (default ``6``, ``LOG_INFO``). See ``include/syslog.h``.

``CONFIG_TFLITEMICRO_TOOL``
  Build the ``tflm`` NSH command from ``tflm_tool.cc``.
  ``CONFIG_TFLITEMICRO_TOOL_PRIORITY`` and
  ``CONFIG_TFLITEMICRO_TOOL_STACKSIZE`` set the task attributes
  (defaults: priority 100, stack 4096).

``CONFIG_TFLITEMICRO_HELLOWORLD``
  Build the ``tflm_hello`` example (CMake builds only). The example runs
  the upstream TFLM hello-world float and INT8 sine models.
  ``CONFIG_TFLITEMICRO_HELLOWORLD_PRIORITY`` and
  ``CONFIG_TFLITEMICRO_HELLOWORLD_STACKSIZE`` set the task attributes
  (defaults: priority 100, stack 4096).

Building
========

The ``sim:tflm`` defconfig enables TFLM, the ``tflm`` tool, the hello-world
example, libc++, FlatBuffers, gemmlowp, KissFFT, and Ruy.

Makefile build (produces the ``tflm`` tool; hello-world is CMake-only):

.. code-block:: console

   $ cd nuttx
   $ make distclean
   $ ./tools/configure.sh sim:tflm
   $ make -j$(nproc)
   $ ./nuttx

CMake build (produces both ``tflm`` and ``tflm_hello``):

.. code-block:: console

   $ cd nuttx
   $ cmake -B build -DBOARD_CONFIG=sim:tflm -GNinja
   $ cmake --build build
   $ ./build/nuttx

The first build downloads the TFLM sources. Subsequent builds reuse the
unpacked tree under ``apps/mlearning/tflite-micro/tflite-micro``.
``make distclean`` in the apps tree removes that snapshot.

Using the ``tflm`` tool
=======================

``tflm`` loads a ``.tflite`` file from the filesystem, constructs a
``tflite::MicroInterpreter``, and can invoke the model once for profiling
or emit compiled C++ (when TFLM was built with ``TFLITE_MODEL_COMPILER``).

.. code-block:: console

   nsh> tflm -h

   Utility to use tflite micro on nuttx.
   [ -C       ] Compile tflite model into c++ codes.
   [ -E       ] Do once evaluation (for profiling).
   [ -i <str> ] Readable model file path.
   [ -o <str> ] Writable c++ file path.
   [ -p <str> ] Prefix of compiled code.
   [ -a <int> ] Arena size (mempool).
   [ -h       ] Print this message.

Both ``-i`` and ``-o`` are required. Defaults are prefix ``NXAI`` and
arena size 8192 bytes.

The built-in operator resolver registers eight INT8-oriented ops:

- ``CONV_2D`` (INT8)
- ``MAX_POOL_2D`` (INT8)
- ``QUANTIZE`` (float32 to INT8)
- ``DEQUANTIZE`` (INT8)
- ``MEAN`` (INT8)
- ``RESHAPE``
- ``FULLY_CONNECTED`` (INT8)
- ``SOFTMAX`` (INT8)

Models that need other operators must change the resolver in
``apps/mlearning/tflite-micro/tflm_tool.cc``.

Hello-world example
===================

With a CMake ``sim:tflm`` image:

.. code-block:: console

   nsh> tflm_hello

This runs the upstream hello-world test: it profiles memory and latency,
then loads the float and INT8 sine models that are converted to C arrays
at build time with ``xxd``.

Embedding a model in an application
===================================

TFLM is designed for targets without a filesystem and without dynamic
allocation for the model itself. Typical NuttX applications compile the
``.tflite`` file into a C array and pass it to ``tflite::GetModel()``.

The CMake helper ``tflite_generate_data()`` in
``apps/mlearning/tflite-micro/CMakeLists.txt`` wraps ``xxd -i`` for that
purpose. The same conversion can be done manually::

  xxd -i model.tflite model_data.h

Provide a tensor arena (the ``-a`` size in ``tflm``, or a static buffer
in application code) large enough for the model's scratch tensors.
Insufficient arena size makes ``MicroInterpreter::AllocateTensors()``
fail.

Patches applied by NuttX
========================

- ``tflite-micro.patch`` — guard the ``ASSERT`` macro in the TFLM signal
  circular buffer.
- ``0001-dequantize-int8.patch`` — ``Register_DEQUANTIZE_INT8()``.
- ``0002-quantize-int8.patch`` — ``Register_QUANTIZE_FLOAT32_INT8()``.
- ``0003-mean-int8.patch`` — ``Register_MEAN_INT8()``.
- ``0004-tflite-add-extern-C-to-main-function-to-avoid-c-mang.patch`` —
  ``extern "C"`` on the hello-world ``main`` so NuttX can call it
  (applied by the CMake fetch).

See also
========

- Upstream TFLM: https://github.com/tensorflow/tflite-micro
- Hello-world example:
  https://github.com/tensorflow/tflite-micro/blob/main/tensorflow/lite/micro/examples/hello_world/README.md
- :doc:`../cmsis-nn/index` for ARM kernel acceleration
