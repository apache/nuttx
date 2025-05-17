``Xedge`` a lightweight Lua-based web framework for building secure, real-time IoT applications
==============

Xedge is a robust IoT and web framework that is designed for microcontrollers. It is based on the industrial-grade Barracuda Application Server, designed for seamless OEM integration. Xedge accelerates embedded firmware development by providing a flexible, Lua-based environment and a full stack of industrial-strength protocols, including:

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

.. note::
   These instructions set up Xedge in **development mode**. Xedge supports many configuration options that differ between development and production builds. For production settings and optimization, refer to the general Xedge build instructions (details below).

Getting Started
---------------

To compile Xedge for NuttX, follow these steps:

**Note:** the included script ``prepare.sh`` performs step 1 to 4.

1. **Prepare the Xedge apps directory.**

   Navigate to the Xedge application folder:

   .. code-block:: bash

      cd nuttxspace/apps/examples/xedge

2. **Clone the required dependencies.**

   .. code-block:: bash

      git clone https://github.com/RealTimeLogic/BAS.git
      git clone https://github.com/RealTimeLogic/BAS-Resources.git

3. **Build the Xedge resources.**

   Move into the build directory:

   .. code-block:: bash

      cd BAS-Resources/build/

   Run the Xedge resource build script. The following command answers the script prompts automatically:

   .. code-block:: bash

      printf "n\nl\nn\n" | bash Xedge.sh

4. **Copy the generated file.**

   Copy ``XedgeZip.c`` to the Xedge example directory:

   .. code-block:: bash

      cp XedgeZip.c ../../BAS/examples/xedge/

5. **Configure NuttX for Xedge.**

   Return to the NuttX top-level directory and start the configuration tool:

   .. code-block:: bash

      make menuconfig

   In the configuration menu:

   - Enable the Xedge application:

     ``Application Configuration -> Examples -> Xedge``

   - Enable dynamic socket callback allocation:

     ``Networking Support -> Socket Support -> Enable dynamic socket callback allocation`` (set value to **1**)

   - Increase the number of dynamic TCP connections:

     ``Networking Support -> TCP/IP Networking -> Dynamic TCP/IP connections allocation`` (set value to **at least 20**)

   - Enable automatic time synchronization via SNTP:

     ``Application Configuration -> Network Utilities -> NTP Client``

   - Make sure syslog is enabled:

     ``Device Drivers  -> System Logging   -> Enable system logging``

     ``Device Drivers  -> System Logging   -> Log to /dev/console``

6. **Build NuttX.**

   Save your changes, exit ``menuconfig``, and build:

   .. code-block:: bash

      make

Running Xedge
-------------

Once NuttX is flashed and running, bring up the network interface and start Xedge:

.. code-block:: bash

   ifup eth0
   xedge

Use a browser and navigate to http://target-ip-address

Further Reading
===============

To learn more about Xedge and how to work with it effectively, see the following resources:

- **Using Xedge in Developer Mode:**
  Learn how to configure and work with Xedge during development, including dynamic Lua scripting, file system layout, and runtime behavior.
  
  `How to use Xedge when in developer mode <https://realtimelogic.com/ba/doc/en/Xedge.html>`_

- **Building Xedge for Developer or Production Mode:**
  Understand the differences between development and production builds, and follow detailed instructions for compiling Xedge appropriately for your target environment.
  
  `How to compile Xedge for developer or production mode <https://realtimelogic.com/ba/examples/xedge/readme.html>`_
