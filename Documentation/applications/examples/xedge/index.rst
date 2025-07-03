``Xedge`` a lightweight Lua-based web framework for building secure, real-time IoT applications
================================================================================================

`Xedge <https://realtimelogic.com/products/xedge/>`_ is a robust IoT and web framework that is designed for microcontrollers. It is based on the industrial-grade Barracuda Application Server, designed for seamless OEM integration. Xedge accelerates embedded firmware development by providing a flexible, Lua-based environment and a full stack of industrial-strength protocols, including:

- OPC UA
- Modbus
- MQTT
- SMQ
- WebSockets
- HTTP/HTTPS

This Xedge port for NuttX comes pre-configured and requires:

- TCP/IP v4 and v6 support
- File System support
- 2 MB RAM allocated statically in ``xedge/xedge_main.c``
- QEMU
- Toolchain aarch64-none-elf (gcc-arm-11.2-2022.02 or higher)

.. note::
   These instructions set up Xedge in **development mode**. Xedge supports many configuration options that differ between development and production builds. For production settings and optimization, refer to the general Xedge build instructions (details below).


Why Use Lua and Xedge in Embedded Systems
------------------------------------------

Great Lua developers don't treat it as a "better C"; they treat it as a complement. Lua is an extension language, which means it's designed to work alongside C, not replace it. Smart embedded programmers use C for performance-critical, low-level code and Lua for high-level business logic, such as processing sensor data and managing secure cloud connectivity.

Writing embedded business logic purely in C often means hundreds of lines of boilerplate code to manage memory, handle complex APIs, and handle errors. Lua, especially when paired with a framework like Xedge, lifts that burden. It provides high-level libraries and modules out of the box for protocols, networking, file systems, and more.

This shift doesn't just make development easier; it makes it faster. What used to take weeks in C can now be done in days. Lua's simplicity encourages rapid prototyping and quick iteration, which is essential in modern IoT and embedded development, where both time-to-market and security are critical. For a conceptual overview of why this hybrid development model is so powerful, check out the tutorial `Why Smart C Coders Love Lua <https://realtimelogic.com/articles/Using-Lua-for-Embedded-Development-vs-Traditional-C-Code>`_.

.. figure:: https://realtimelogic.com/blogmedia/lots-of-embedded-c-code-cartoon-600.jpg
   :align: center
   :alt: Too much C code


Getting Started
---------------

To compile Xedge for NuttX, follow these steps:

**1. Configure NuttX for Xedge.**

For this example, we're using **QEMU** and **aarch64-none-elf-gcc** with the **qemu-armv8a** board. You can use any NuttX-supported board,
but Xedge requires boards with external RAM since it defines a 2MB static array in xedge_main.c. We recommend at least 2MB RAM for stable performance.

To install and configure **QEMU** and **aarch64-none-elf-gcc**, follow the instructions in the `NuttX Getting Started Guide <https://nuttx.apache.org/docs/latest/quickstart/quickstart.html>`_.

**2. Build NuttX.**

   Run xedge example with qemu-armv8a board ::

      $ ./tools/configure.sh qemu-armv8a:xedge
      $ make

   Running with QEMU::

      $ qemu-system-aarch64 -cpu cortex-a53 -smp 4 -nographic \
      -machine virt,virtualization=on,gic-version=3 \
      -chardev stdio,id=con,mux=on -serial chardev:con \
      -global virtio-mmio.force-legacy=false \
      -netdev user,id=u1,hostfwd=tcp:127.0.0.1:8080-10.0.2.15:80,hostfwd=tcp:127.0.0.1:8443-10.0.2.15:443,hostfwd=tcp:127.0.0.1:10023-10.0.2.15:23 \
      -device virtio-net-device,netdev=u1,bus=virtio-mmio-bus.0 \
      -mon chardev=con,mode=readline -kernel ./nuttx

Running Xedge
-------------

Running Xedge in NuttX terminal::

      nsh> xedge
      [    3.020000] [CPU0] Error: cannot set root to /mnt/lfs
      [    3.020000] [CPU0] Installing DiskIo failed!
      [    3.070000] [CPU0] Xedge: Server listening on IPv4 port 80
      [    3.080000] [CPU0] Xedge: SharkSSL server listening on IPv4 port 443
      [   13.720000] [CPU0] Xedge: Received invalid browser localStorage
      [   20.090000] [CPU1] 10.0.2.2 GET "rtl/assets/favicon.ico" t
      [   20.090000] [CPU1] Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/138
      [   20.090000] [CPU1] .0.0.0 Safari/537.36
      [   20.090000] [CPU1] Host: 127.0.0.1:8080
      [   20.090000] [CPU1] Connection: keep-alive
      [   20.090000] [CPU1] sec-ch-ua-platform: "Linux"
      [   20.090000] [CPU1] User-Agent: a
      [   20.090000] [CPU1] Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/138
      [   20.090000] [CPU1] .0.0.0 Safari/537.36
      [   20.090000] [CPU1] sec-ch-ua: "Not)A;Brand";v="8", "Chromium";v="138", "Google Chrome";v="138"
      [   20.090000] [CPU1] sec-ch-ua-mobile: ?0
      [   20.090000] [CPU1] Accept: image/avif,image/webp,image/apng,image/svg+xml,image/*,*/*;q=0.8
      [   20.090000] [CPU1] Sec-Fetch-Site: same-origin
      [   20.090000] [CPU1] Sec-Fetch-Mode: no-cors
      [   20.090000] [CPU1] Sec-Fetch-Dest: image
      [   20.090000] [CPU1] Referer: http://127.0.0.1:8080/rtl/
      [   20.090000] [CPU1] Accept-Encoding: gzip, deflate, br, zstd
      [   20.090000] [CPU1] Accept-Language: en-US,en;q=0.9,pt-BR;q=0.8,pt;q=0.7
      [   20.090000] [CPU1] If-None-Match: 6866b8fe
      [   20.090000] [CPU1] If-Modified-Since: Thu, 03 Jul 2025 17:08:14 GMT
      [   20.090000] [CPU1]
      [   20.090000] [CPU0] 10.0.2.2 Response:
      T
      [   20.090000] [CPU0] Thu, 03 Jul 2025 19:32:12 GMT
      Content-Encoding: gzip
      Vary: Accept-Encoding
      Con
      [   20.090000] [CPU0] tent-Length: 3436
      Keep-Alive: Keep-Alive

Launch your web browser and access 127.0.0.1:8080

You should see the Xedge IDE, which is enabled in developer mode:

.. figure:: xedge_example.png
   :align: center


Further Reading
===============

To learn more about Xedge and how to work with it effectively, see the following resources:

- **Using Xedge in Developer Mode:**
  Learn how to configure and work with Xedge during development, including dynamic Lua scripting, file system layout, and runtime behavior.
  
  `How to use Xedge when in developer mode <https://realtimelogic.com/ba/doc/en/Xedge.html>`_

- **Building Xedge for Developer or Production Mode:**
  Understand the differences between development and production builds, and follow detailed instructions for compiling Xedge appropriately for your target environment.
  
  `How to compile Xedge for developer or production mode <https://realtimelogic.com/ba/examples/xedge/readme.html>`_
