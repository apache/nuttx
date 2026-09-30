================
VirtIO Framework
================

Introduction
============

NuttX implements a complete VirtIO framework based on OpenAMP. The framework
supports various VirtIO drivers compatible with the VirtIO standard (such as
VirtIO-Net, VirtIO-Block, etc.) at the upper layer, and different VirtIO
transport layer implementations (including VirtIO-MMIO, VirtIO-PCI,
VirtIO-Remoteproc, etc.) at the lower layer.

Architecture
============

Framework Overview
------------------

The VirtIO framework consists of three main layers:

.. code-block:: text

   +------------------------------------------------------------------+
   |                      VirtIO Driver Layer                         |
   |  +----------+ +----------+ +----------+ +-----+ +-------------+  |
   |  | VirtIO   | | VirtIO   | | VirtIO   | | ... | | VirtIO      |  |
   |  | Sock     | | Net      | | Blk      | |     | | Rpmsg       |  |
   |  +----------+ +----------+ +----------+ +-----+ +-------------+  |
   +------------------------------------------------------------------+
                                  |
                                  v
   +------------------------------------------------------------------+
   |                     VirtIO Framework Layer                       |
   |                       VirtIO Framework                           |
   +------------------------------------------------------------------+
                                  |
            +---------------------+---------------------+
            |                     |                     |
            v                     v                     v
   +----------------+    +----------------+    +----------------+
   |  VirtIO MMIO   |    |  VirtIO PCI    |    |  VirtIO Rptun  |
   +----------------+    +----------------+    +----------------+
   |  Memory Map    |    | PCI Framework  |    |     Rptun      |
   +----------------+    +----------------+    +----------------+
            |            | PCI Controller |    |  Rptun Driver  |
            |            +----------------+    +----------------+
            |                     |                     |                   +----------------------+
            |                     |                     +-----------------> | Remoteproc Processor |
            v                     v                                         +----------------------+
   +------------------------------------------------------------------+
   |                         Hypervisor                               |
   +------------------------------------------------------------------+

The framework can be divided into the following three parts:

1. **VirtIO Driver Layer**: The driver layer is responsible for interfacing
   VirtIO with the NuttX driver framework. The driver layer completes device
   initialization and data interaction by calling the unified interfaces
   provided by VirtIO.

2. **VirtIO Framework Layer**: The VirtIO layer provides unified interfaces
   for drivers, supporting registration, unregistration, and matching
   mechanisms for Drivers and Devices.

3. **VirtIO Transport Layer**: The transport layer provides support for
   different transport methods, including MMIO, RemoteProc, and PCI.

Workflow
--------

The following describes the matching process and calling relationship between
VirtIO Device and VirtIO Driver:

.. code-block:: text

   +------------------------------------------------------------------+
   | VirtIO Drivers                                                   |
   | +--------+ +--------+ +--------+ +--------+ +--------+ +-------+ |
   | |VirtIO  | |VirtIO  | |VirtIO  | |VirtIO  | |VirtIO  | |  ...  | |
   | |Net     | |Sock    | |Blk     | |Serial  | |Sound   | |       | |
   | +--------+ +--------+ +--------+ +--------+ +--------+ +-------+ |
   +------------------------------------------------------------------+
                              |
                              | virtio_register_driver()
                              v
   +------------------------------------------------------------------+
   |                        VirtIO Bus                                |
   +------------------------------------------------------------------+
                              ^
                              | virtio_register_device()
                              |
   +------------------------------------------------------------------+
   | Transport Layer                                                  |
   | +----------------+ +----------------+ +----------------+         |
   | | VirtIO-MMIO    | | VirtIO-PCI     | | VirtIO-Rptun   |         |
   | | Device Memory  | | PCI Framework  | | Resource Table |         |
   | | and IRQ        | | Device in PCI  | |                |         |
   | +----------------+ +----------------+ +----------------+         |
   +------------------------------------------------------------------+

                              ||
                              || Match Success
                              vv

   +------------------------------------------------------------------+
   |                    virtio_xxx_probe()                            |
   |    Reset, configure, feature negotiation, memory allocation      |
   +------------------------------------------------------------------+
                              |
                              | VirtIO API
                              v
   +------------------------------------------------------------------+
   |              Port to NuttX Various Frameworks                    |
   |           Normal Driver/Socket etc operations                    |
   +------------------------------------------------------------------+
                              |
                              v
   +------------------------------------------------------------------+
   |                           VFS                                    |
   +------------------------------------------------------------------+

1. **Driver Registration**:
   During NuttX initialization, ``virtio_register_drivers()`` is called to
   register all supported VirtIO Drivers to the VirtIO bus.

2. **Device Registration**:
   The registration process is initiated by the transport layer:

   - MMIO transport layer calls ``virtio_register_mmio_device()``
   - REMOTEPROC transport layer calls ``rptun_register_device()``
   - PCI transport layer calls ``virtio_pci_probe()``

   After the transport layer completes initialization, it calls
   ``virtio_register_device()`` to register the VirtIO Device to the VirtIO bus.

3. **Driver and Device Matching**:
   When a device is registered to the bus, the system attempts to match Driver
   and Device. If the match is successful, the ``probe`` function implemented
   by the Driver is executed. In the ``probe`` function, the driver initializes,
   configures, and performs feature negotiation on the VirtIO Device. Depending
   on the complexity and type of the device, it may also need to initialize
   private structures or perform additional operations.

4. **Register NuttX Driver**:
   Call the API provided by the NuttX driver framework to register the driver
   to the Virtual File System (VFS) for user access.

5. **Runtime**:
   During runtime, the Driver calls the ``virtqueue`` common interfaces provided
   by OpenAMP to exchange data and notifications in VirtIO standard format,
   thereby implementing driver functionality.

Source Code
===========

.. code-block:: text

   nuttx/
   ├── drivers/
   │   └── virtio/
   │       └── virtio.c       # VirtIO framework core implementation
   ├── include/
   │   └── nuttx/
   │       └── virtio/
   │           └── virtio.h   # VirtIO header file
   └── openamp/
       └── open-amp/          # OpenAMP repository

API Reference
=============

This section describes the interfaces that need to be called during VirtIO
driver adaptation.

NuttX Log Interfaces
--------------------

``vrtinfo(...)``
   INFO level VirtIO system log interface.

``vrtwarn(...)``
   WARNING level VirtIO system log interface.

``vrterr(...)``
   ERROR level VirtIO system log interface.

NuttX VirtIO Framework Interface
--------------------------------

.. c:function:: int virtio_register_driver(FAR struct virtio_driver *driver)

   Register a VirtIO Driver to the VirtIO bus. When a corresponding device
   already exists in the bus, it will immediately match and call the ``probe``
   function implemented by the driver. If there is no corresponding device in
   the bus, the driver's ``probe`` function will be called back after a
   corresponding VirtIO device is registered to the VirtIO bus to complete
   driver initialization.

   :param driver: Pointer to the virtio_driver structure
   :return: 0 on success, negative errno on failure

OpenAMP Interfaces
------------------

Prerequisites
~~~~~~~~~~~~~

- **Driver TX virtqueue**:
  The driver's transmit queue. Get a buffer from the ``used ring`` of ``txvq``,
  fill in the data to be sent, and then add it to the ``avail ring`` of ``txvq``
  to complete the data transmission process.

- **Driver RX virtqueue**:
  The driver's receive queue. Get a buffer from the ``used ring`` of ``rxvq``,
  read the data from it, and then return it to the ``avail ring`` of ``rxvq``
  to complete the data reception process.

Interface Description
~~~~~~~~~~~~~~~~~~~~~

.. c:function:: void *virtqueue_get_buffer(struct virtqueue *vq, uint32_t *len, uint16_t *idx)

   Get a buffer from the ``used ring`` of virtqueue.

   :param vq: Pointer to the virtqueue
   :param len: Length of the obtained buffer
   :param idx: Index of the obtained buffer in the ``used ring``
   :return: Pointer to the buffer, or NULL if no buffer available

.. c:function:: int virtqueue_add_buffer(struct virtqueue *vq, struct virtqueue_buf *buf_list, int readable, int writable, void *cookie)

   Add a buffer to the ``avail ring`` of virtqueue.

   :param vq: Pointer to the virtqueue
   :param buf_list: Array of buffers to be added
   :param readable: Number of readable buffers in ``buf_list``, indicating the
                    part that the Device should read
   :param writable: Number of writable buffers in ``buf_list``, indicating the
                    part that the Device should fill
   :param cookie: Cache pointer, this value will be returned when calling
                  ``virtqueue_get_buffer`` to get the buffer
   :return: 0 on success, negative errno on failure

.. c:function:: void virtqueue_kick(struct virtqueue *vq)

   Notify the Device. Usually called after sending data to the device side or
   returning a buffer to the device side to notify the device that it can
   proceed to the next operation.

   :param vq: Pointer to the virtqueue

.. c:function:: void virtqueue_enable_cb(struct virtqueue *vq)

   Enable virtqueue interrupt.

   :param vq: Pointer to the virtqueue

.. c:function:: void virtqueue_disable_cb(struct virtqueue *vq)

   Disable virtqueue interrupt.

   :param vq: Pointer to the virtqueue

References
==========

- `virtio: Towards a De-Facto Standard For Virtual I/O Devices <https://ozlabs.org/~rusty/virtio-spec/virtio-paper.pdf>`_
- `Virtual I/O Device (VIRTIO) Version 1.2 <https://docs.oasis-open.org/virtio/virtio/v1.2/csd01/virtio-v1.2-csd01.pdf>`_
