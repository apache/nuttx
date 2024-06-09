==================================
``wamr`` Webassembly Micro Runtime
==================================

This guides explains how to try WAMR on NuttX.

Install WASI SDK
================

We need to be able to build WASM programs. This can be done by installing the ``wasi-sdk`` as shown in the `WAMR guide <https://wamr.gitbook.io/document/basics/getting-started/host_prerequsites>`_.

Make sure the installation is reachable via ``/opt/wasi-sdk`` on your host, as this is expected by the build script of the WASM hello world sample below.

Building NuttX
==============

We use the `sim/wamr` config to build WAMR with NuttX. Note that at this writing, the that CMake scripts don't work for this configuration. For now, please use the makefile build instead:

.. code:: console

   $ cd nuttx
   $ tools/configure.sh sim/wamr
   $ make -j4
   $ ls -l nuttx
   -rwxrwxr-x 1 yf yf 4176376 Jun 10 08:11 nuttx

This will generate a ``nutxx`` binary. Also a WASM sample is available in the folder ``apps/interpreters/wamr/wamr/product-mini/app-samples/hello-world`` folder. We build the sample as follows:

.. code:: console

   $ cd apps/interpreters/wamr/wamr/product-mini/app-samples/hello-world
   $ ./build.sh
   $ ls -l test.wasm
   -rwxrwxr-x 1 yf yf  413 Jun 10 08:18 test.wasm

The ``test.wasm`` program should be accessible from NuttX later.

Try WAMR in NSH
===============

Copy the ``test.wasm`` to the folder where the ``nuttx`` program lives, then run ``./nuttx`` to kick off NuttX and run ``iwasm /data/test.wasm`` from NSH:

.. code:: console

   $ ./nuttx

   NuttShell (NSH) NuttX-12.4.0
   nsh> iwasm /data/test.wasm
   Hello world!
   buf ptr: 0x1460
   buf: 1234
   nsh>

