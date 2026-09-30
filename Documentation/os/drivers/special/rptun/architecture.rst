============
Architecture
============

RPTUN sits at the system's lower level, bridging the upper-level VirtIO/Vhost
drivers with the underlying OpenAMP framework.

Overview
========

The RPTUN architecture consists of the following layers:

::

  +------------------+     +------------------+
  |  VirtIO Drivers  |     |  Vhost Drivers   |
  +--------+---------+     +--------+---------+
           |                        |
           v                        v
  +----------------------------------------+
  |           VirtIO/Vhost Bus             |
  +----------------------------------------+
           |                        |
           v                        v
  +----------------------------------------+
  |              RPTUN Framework           |
  |  +----------------------------------+  |
  |  |       RPTUN Remoteproc           |  |
  |  |  (Lifecycle Management)          |  |
  |  +----------------------------------+  |
  |  +----------------------------------+  |
  |  |       VirtIO/Vhost RPTUN         |  |
  |  |  (Transport Layer)               |  |
  |  +----------------------------------+  |
  +----------------------------------------+
                    |
                    v
  +----------------------------------------+
  |           RPTUN Driver (BSP)           |
  |  (Platform-specific Implementation)    |
  +----------------------------------------+
                    |
                    v
  +----------------------------------------+
  |              OpenAMP                   |
  +----------------------------------------+

Component Description
=====================

RPTUN Remoteproc
----------------

- **Function**: Based on OpenAMP's Remoteproc functionality, implements
  lifecycle management of remote CPUs (start, stop, reset, etc.). It depends
  on the underlying driver (RPTUN Driver) to provide chip-specific
  implementations related to startup, shutdown, and interrupts.

- **User Interface**: Registers character devices through the Virtual File
  System (VFS), allowing user space to control remote CPUs using ``ioctl()``
  system calls or NuttShell (NSH) commands.

VirtIO/Vhost RPTUN
------------------

- **Function**: Utilizes shared memory and inter-core interrupts provided by
  Remoteproc to implement a VirtIO-standard compatible Remoteproc transport
  layer, enabling VirtIO/Vhost drivers to communicate across cores.

- **Interaction**: RPTUN associates with the VirtIO/Vhost framework through
  the ``virtio/vhost_register_device()`` interface.

RPTUN Driver (Adaptation Layer)
-------------------------------

- **Function**: Implements chip-specific functionality related to lifecycle
  management, shared memory, and inter-core interrupts.

- **Responsibilities**:

  - Remote CPU start/stop functionality
  - Provide shared memory regions (address and size)
  - Implement inter-core interrupt sending (notify) and reception handling
  - Provide physical-to-device address translation tables for shared memory

Initialization and Startup
==========================

Driver Initialization
---------------------

The platform driver calls ``rptun_initialize()`` to create and register an
RPTUN instance. Each instance manages a communication channel with a remote
CPU. The channel name can be specified through the ``rptun_ops->get_cpuname()``
interface. If communication with multiple remote cores is needed, multiple
RPTUN instances must be created.

Core Service Startup
--------------------

RPTUN's core service is started through ``rptun_dev_start()``. This function
creates a kernel thread to execute time-consuming tasks. There are two startup
modes:

- **Auto Start**: If the driver is configured for auto-start
  (``RPTUN_IS_AUTOSTART()`` returns true), ``rptun_initialize()`` will
  automatically call ``rptun_dev_start()``.

- **Manual Start**: After driver initialization, users can manually trigger
  the startup process through the NSH command
  ``rptun start /dev/rptun/<cpuname>`` or ``ioctl()`` system call.

Core Initialization Tasks
-------------------------

In the startup thread, RPTUN performs the following key operations:

1. Configure Remoteproc
2. Obtain and parse the Resource Table provided by the driver
3. Create VirtIO/Vhost devices based on information in the Resource Table
4. Register created devices to the VirtIO/Vhost bus
5. Register interrupt callback function ``rptun_callback()`` through
   ``ops->register_callback()`` to respond to interrupt notifications
   from the remote core

Remote CPU Lifecycle Management
===============================

RPTUN Remoteproc provides two ways for users and kernel code to manage
remote CPUs:

User Space Interface
--------------------

- **NSH Commands**: Use ``rptun start <dev>`` and ``rptun stop <dev>`` commands
- **Character Device**: Open device file ``/dev/rptun/<cpuname>``, then use
  ``ioctl()`` to send control codes like ``RPTUNIOC_START`` or ``RPTUNIOC_STOP``

Kernel Space API
----------------

Kernel modules can directly call API functions provided by RPTUN to control
remote CPUs. See :doc:`api_reference` for details.

VirtIO/Vhost Communication Flow
===============================

Device Initialization
---------------------

During RPTUN core service startup, it parses each VirtIO device defined in
the Resource Table, creates actual VirtIO devices, and registers them to
the VirtIO/Vhost bus. The main steps are:

1. **Parse Resources**: RPTUN framework parses the Resource Table, looking
   for all vdev and carveout resources.

2. **Create Device Instance**: Based on found vdev resource information,
   call OpenAMP interface ``remoteproc_create_virtio()`` to create VirtIO
   device instances.

3. **Initialize Shared Memory Heap**: Based on corresponding carveout
   resources, initialize an independent shared memory heap. This heap is
   dedicated to the VirtIO device for dynamic data buffer management.

4. **Register Device**: Register the initialized VirtIO/Vhost device to
   the system, making it visible to upper-layer applications.

5. **Loop Processing**: Repeat the above steps until all VirtIO devices
   in the Resource Table are created and registered.

Interrupt Handling
------------------

**Interrupt Callback (Remote to Local)**

When the RPTUN driver receives an inter-core interrupt from the remote core,
it calls the registered ``rptun_callback()`` function. This function internally
calls ``remoteproc_get_notification()``, ultimately triggering the callback
function of the VirtIO/Vhost driver associated with that Virtqueue to process
received data.

**Interrupt Notification (Local to Remote)**

When a VirtIO/Vhost driver needs to notify the remote end (e.g., data has
been placed in the Virtqueue), it calls ``virtqueue_kick()``. This call
ultimately invokes the RPTUN framework's ``rptun_notify()`` function, which
then calls the driver-implemented ``ops->notify()`` function to trigger
hardware to send an inter-core interrupt.
