===============
Vhost Framework
===============

Introduction
============

The NuttX Vhost architecture is highly modular and forms a perfect symmetry
with the VirtIO frontend architecture. Vhost serves as the VirtIO backend
(Device side) implementation, enabling cross-core communication through
shared memory and inter-core interrupts.

Architecture
============

Framework Overview
------------------

The Vhost framework consists of three main layers:

.. code-block:: text

   +----------------------------------------------------------------+
   |                      Vhost Service Layer                       |
   |  +----------+ +----------+ +----------+                        |
   |  | Vhost    | | Vhost    | | ...      |                        |
   |  | Rpmsg    | | Rng      | |          |                        |
   |  +----------+ +----------+ +----------+                        |
   +----------------------------------------------------------------+
                                  |
                                  v
   +----------------------------------------------------------------+
   |                     Vhost Framework Layer                      |
   |                       Vhost Framework                          |
   +----------------------------------------------------------------+
                                  |
                                  v
   +----------------------------------------------------------------+
   |                     Vhost Transport Layer                      |
   |                      +----------------+                        |
   |                      |  Vhost Rptun   |                        |
   |                      +----------------+                        |
   |                      |     Rptun      |                        |
   |                      +----------------+                        |
   |                      |  Rptun Driver  |                        |
   |                      +----------------+                        |
   +----------------------------------------------------------------+
                                  |
                                  v
   +----------------------------------------------------------------+
   |                  Share Memory && Interrupt                     |
   +----------------------------------------------------------------+

The framework can be divided into the following three parts:

1. **Vhost Service Layer**: Located at the top layer, corresponding to
   ``Vhost Rpmsg``, ``Vhost Rng``, etc. This layer implements
   specific device logic conforming to the VirtIO standard. For example,
   ``Vhost Rng`` parses read requests from the frontend and converts
   them into operations on the underlying physical storage.

2. **Vhost Framework Layer**: This is the core middle layer that implements
   the Vhost bus logic and encapsulates an easy-to-use Virtqueue API. It
   manages device lifecycle, parses descriptor chains, and provides a unified
   data interface to the service layer.

3. **Vhost Transport Layer**: This layer handles low-level physical
   communication, particularly **Vhost Rptun** implemented for AMP scenarios.
   It uses shared memory and inter-core interrupts to establish physical
   connections with the peer VirtIO Rptun.

Workflow
--------

The following describes the matching process and calling relationship between
Vhost Device and Vhost Driver:

.. code-block:: text

   +------------------------------------------------------------------+
   | Vhost Drivers                                                    |
   | +--------+ +--------+  +-------+                                 |
   | | Vhost  | | Vhost  |  |  ...  |                                 |
   | | Rng    | | Rpmsg  |  |       |                                 |
   | +--------+ +--------+  +-------+                                 |
   +------------------------------------------------------------------+
                              |
                              | vhost_register_driver()
                              v
   +------------------------------------------------------------------+
   |                         Vhost Bus                                |
   +------------------------------------------------------------------+
                              ^
                              | vhost_register_device()
                              |
   +------------------------------------------------------------------+
   | Transport Layer                                                  |
   | +----------------+                                               |
   | | Vhost-Rptun    |                                               |
   | | Resource Table |                                               |
   | +----------------+                                               |
   +------------------------------------------------------------------+

                              ||
                              || Match Success
                              vv

   +------------------------------------------------------------------+
   |                     vhost_xxx_probe()                            |
   |    Reset, configure, feature negotiation, memory allocation      |
   +------------------------------------------------------------------+
                              |
                              | Vhost API
                              v
   +------------------------------------------------------------------+
   |              Port to NuttX Various Frameworks                    |
   |           Normal Driver/Socket etc operations                    |
   +------------------------------------------------------------------+
                              |
                              v
   +------------------------------------------------------------------+
   |                           VFS/Other Driver                       |
   +------------------------------------------------------------------+

1. **Driver Registration**:
   During NuttX initialization, ``vhost_register_drivers()`` is called to
   register all supported Vhost Drivers to the Vhost bus.

2. **Device Registration**:
   The registration process is initiated by the transport layer. The
   REMOTEPROC transport layer calls ``rptun_register_device()``.

3. **Driver and Device Matching**:
   When a device is registered to the bus, the system attempts to match Driver
   and Device. If the match is successful, the ``probe`` function implemented
   by the Driver is executed. In the ``probe`` function, the driver initializes,
   configures, and performs feature negotiation on the Vhost Device. Depending
   on the complexity and type of the device, it may also need to initialize
   private structures or perform additional operations.

4. **Register NuttX Driver**:
   Call the API provided by the NuttX driver framework to register the driver
   to the Virtual File System (VFS) for user access or access other system
   drivers through the VFS.

5. **Runtime**:
   During runtime, the Driver calls the ``virtqueue`` common interfaces provided
   by OpenAMP to exchange data and notifications in VirtIO standard format,
   thereby implementing driver functionality.

.. note::

   If the peer VirtIO Driver is not ready (status is not ``DRIVER_OK``), the
   device will be added to a deferred probe queue. The system will periodically
   check and complete matching when ready.

Source Code
===========

.. code-block:: text

   nuttx/
   ├── drivers/
   │   └── vhost/
   │       └── vhost.c        # Vhost framework core implementation
   ├── include/
   │   └── nuttx/
   │       └── vhost/
   │           └── vhost.h    # Vhost header file
   └── openamp/
       └── open-amp/          # OpenAMP repository

API Reference
=============

Data Structures
---------------

struct vhost_device
~~~~~~~~~~~~~~~~~~~

Vhost device structure, which is an alias for ``struct virtio_device``
(defined via macro ``#define vhost_device virtio_device``). Used to represent
a Vhost device instance, containing basic device information and state.

struct vhost_driver
~~~~~~~~~~~~~~~~~~~

Vhost driver structure, used to define a Vhost driver program.

.. code-block:: c

   struct vhost_driver
   {
     uint32_t           device;   /* Device ID for matching Vhost device */
     CODE int         (*probe)(FAR struct vhost_device *hdev);   /* Device probe callback */
     CODE void        (*remove)(FAR struct vhost_device *hdev);  /* Device remove callback */
     struct dq_entry_s  node;     /* List node for driver list */
   };

Core APIs
---------

vhost_register_device
~~~~~~~~~~~~~~~~~~~~~

.. c:function:: int vhost_register_device(FAR struct vhost_device *hdev)

   Register a Vhost device to the Vhost bus.

   :param hdev: Pointer to the Vhost device structure to register
   :return: 0 on success, negative errno on failure

   If the peer VirtIO Driver is not ready (status is not ``DRIVER_OK``), the
   device will be added to a deferred probe queue. The system will periodically
   check and complete matching when ready. If the peer is already ready, it
   will immediately attempt to match with registered drivers.

vhost_unregister_device
~~~~~~~~~~~~~~~~~~~~~~~

.. c:function:: int vhost_unregister_device(FAR struct vhost_device *hdev)

   Unregister a Vhost device from the Vhost bus.

   :param hdev: Pointer to the Vhost device structure to unregister
   :return: 0 on success, negative errno on failure

   If the device has been matched with a driver, the driver's ``remove``
   callback will be called first. Then the device is removed from the device
   list and related resources are released.

vhost_register_driver
~~~~~~~~~~~~~~~~~~~~~

.. c:function:: int vhost_register_driver(FAR struct vhost_driver *hdrv)

   Register a Vhost driver to the Vhost bus.

   :param hdrv: Pointer to the Vhost driver structure to register
   :return: 0 on success, negative errno on failure

   After driver registration, it will automatically attempt to match with
   registered devices. If a match is successful, the driver's ``probe``
   callback will be called.

vhost_unregister_driver
~~~~~~~~~~~~~~~~~~~~~~~

.. c:function:: int vhost_unregister_driver(FAR struct vhost_driver *hdrv)

   Unregister a Vhost driver from the Vhost bus.

   :param hdrv: Pointer to the Vhost driver structure to unregister
   :return: 0 on success, negative errno on failure

   This will iterate through all devices matched with this driver and call
   the driver's ``remove`` callback. Then the driver is removed from the
   driver list.

vhost_register_drivers
~~~~~~~~~~~~~~~~~~~~~~

.. c:function:: void vhost_register_drivers(void)

   Register all configured Vhost drivers.

   :return: None

   Called during system initialization. This will initialize the OpenAMP
   metal library and register corresponding Vhost drivers based on Kconfig
   configuration (such as ``vhost-rng``, ``vhost-rpmsg``, etc.).

vhost_get_vq_buffers
~~~~~~~~~~~~~~~~~~~~

.. c:function:: int vhost_get_vq_buffers(FAR struct virtqueue *vq, FAR struct virtqueue_buf *vb, size_t vbsize, FAR size_t *vbcnt)

   Get a set of available buffers from a Virtqueue.

   :param vq: Pointer to the Virtqueue
   :param vb: Output parameter, array for storing buffer information
   :param vbsize: Size of the ``vb`` array (maximum number of buffers)
   :param vbcnt: Output parameter, returns the actual number of buffers obtained
   :return: Head index of the descriptor chain on success, negative errno on failure

   Return values:

   - ``-ENOMEM``: No available buffers
   - ``-EINVAL``: ``vbsize`` is insufficient to hold all buffers

   This function automatically traverses the entire descriptor chain and fills
   all buffer information into the ``vb`` array. The returned head index can be
   used for subsequent calls to ``virtqueue_add_consumed_buffer`` to return
   buffers.

Macro APIs
----------

The following APIs are wrappers for VirtIO standard interfaces, used to
operate Vhost devices:

.. list-table::
   :header-rows: 1
   :widths: 35 35 30

   * - Macro
     - Corresponding VirtIO API
     - Description
   * - ``vhost_create_virtqueues``
     - ``virtio_create_virtqueues``
     - Create Virtqueue
   * - ``vhost_delete_virtqueues``
     - ``virtio_delete_virtqueues``
     - Delete Virtqueue
   * - ``vhost_set_status``
     - ``virtio_set_status``
     - Set device status
   * - ``vhost_get_status``
     - ``virtio_get_status``
     - Get device status
   * - ``vhost_set_features``
     - ``virtio_set_features``
     - Set device features
   * - ``vhost_get_features``
     - ``virtio_get_features``
     - Get device features
   * - ``vhost_read_config``
     - ``virtio_read_config``
     - Read device config
   * - ``vhost_write_config``
     - ``virtio_write_config``
     - Write device config
   * - ``vhost_has_feature``
     - ``virtio_has_feature``
     - Check feature support
   * - ``vhost_read_config_member``
     - ``virtio_read_config_member``
     - Read config struct member
   * - ``vhost_write_config_member``
     - ``virtio_write_config_member``
     - Write config struct member
