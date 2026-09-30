==============
Resource Table
==============

The Resource Table is a critical data structure defined in shared memory,
provided by the RPTUN driver to the RPTUN framework. It describes available
system resources.

Resource Table Contents
=======================

VirtIO Device (vdev resource)
-----------------------------

Defines the VirtIO device type, features, and Vring-related parameters.

Shared Memory Region (carveout resource)
----------------------------------------

Defines the physical address and size of shared memory used for Virtqueue,
data buffers, and special buffer regions.

Shared Memory Layout
====================

The shared memory used in the RPTUN framework is utilized by various elements
including Resource Table, Vrings, Vring Buffers, Carveout, etc. The layout
of this shared memory is crucial for the framework.

The RPTUN framework relies on the driver to declare a contiguous shared
memory block in the Resource Table. RPTUN builds the following structures
on this memory:

::

  +------------------------------------------+
  |           Resource Table (RSC)           |
  |  +------------------------------------+  |
  |  |  VirtIO Device 0 Description       |  |
  |  |  (vdev resource + carveout)        |  |
  |  +------------------------------------+  |
  |  |  VirtIO Device 1 Description       |  |
  |  |  (vdev resource + carveout)        |  |
  |  +------------------------------------+  |
  |  |  ...                               |  |
  |  +------------------------------------+  |
  +------------------------------------------+
  |              Vrings                      |
  |  +------------------------------------+  |
  |  |  Vring 0 (TX)                      |  |
  |  +------------------------------------+  |
  |  |  Vring 1 (RX)                      |  |
  |  +------------------------------------+  |
  +------------------------------------------+
  |         Carveout (Shared Heap)           |
  |  (Dynamic allocation for Vring Buffers)  |
  +------------------------------------------+

Resource Table (RSC)
--------------------

Describes all shared resources. Its address is directly provided by the
driver to the RPTUN framework. RSC internally contains descriptions of
one or more VirtIO devices. Each VirtIO device is represented by a vdev
resource and a carveout resource together.

Vrings
------

Stores the ring buffer data structures for Virtqueue. Each vdev resource
describes multiple vrings according to the standard. The address allocation
rules are as follows:

- **Dynamic Allocation**: If the ``vring.da`` (device address) field in the
  vdev resource of the Resource Table is 0 or ANY, the RPTUN framework will
  automatically allocate a memory block from the carveout region for Vrings.

- **Static Specification**: If ``vring.da`` is a valid address, it indicates
  that the driver has statically specified the memory location for Vrings,
  and the framework will use that address directly.

Carveout (Shared Memory Heap)
-----------------------------

Each VirtIO device must correspond to a carveout resource to provide
dynamically allocated shared memory. The RPTUN framework uses the carveout
region (minus the portion possibly already allocated to Vrings) to initialize
an independent shared memory heap.

Upper-layer VirtIO/Vhost drivers can dynamically allocate and free data
buffers (Vring Buffers) transmitted in Vrings through this heap, achieving
flexible memory management.

Address Translation
===================

In AMP architectures, different cores may access the same shared memory
block at different physical addresses. To ensure pointer validity on the
remote core, when passing a local core's pointer to the remote end, the
address needs to be converted to an address accessible by the remote end.

The RPTUN framework obtains the address translation table through the
``get_addrenv()`` interface in ``rptun_ops``. When adapting drivers, if
there is a situation where physical addresses are inconsistent between
the two cores, this interface must be implemented.

Resource Table Structure
========================

The standard resource table structure used by RPTUN:

.. code-block:: c

   struct aligned_data(8) rptun_rsc_s
   {
     struct resource_table    rsc_tbl_hdr;
     uint32_t                 offset[2];
     struct fw_rsc_trace      log_trace;
     struct fw_rsc_vdev       rpmsg_vdev;
     struct fw_rsc_vdev_vring rpmsg_vring0;
     struct fw_rsc_vdev_vring rpmsg_vring1;
     struct fw_rsc_config     config;
   };

For detailed information about Resource Table, please refer to the
OpenAMP documentation.
