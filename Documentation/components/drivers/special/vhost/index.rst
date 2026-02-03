=====
Vhost
=====

Vhost is the backend implementation of VirtIO in NuttX. While in standard
Linux kernel, Vhost is a technology for improving virtual machine I/O
performance by offloading VirtIO device data plane processing, NuttX extends
this concept for cross-core communication scenarios.

In NuttX, Vhost is defined as a complete VirtIO backend implementation. It
works with VirtIO frontend through Vhost-Rptun/PCI <-> VirtIO-Rptun/PCI to
achieve cross-core communication. Essentially, Vhost in NuttX is a complete
implementation of VirtIO Device, similar to VirtIO Device implementation in
QEMU.

Comparison
==========

The following table compares NuttX Vhost with Linux Vhost and QEMU VirtIO
Device:

.. list-table::
   :header-rows: 1
   :widths: 25 25 25 25

   * - Dimension
     - NuttX Vhost
     - Linux Vhost
     - QEMU VirtIO Device
   * - Use Case
     - Cross-core communication
     - Virtualization performance
     - Virtualization
   * - Transport Support
     - Rptun and PCI
     - MMIO and PCI
     - MMIO and PCI
   * - Data Plane
     - Included
     - Included
     - Included
   * - Control Plane
     - Included
     - Not included (by Hypervisor)
     - Included

Vhost and VirtIO are complementary concepts, representing the "backend" and
"frontend" relationship. In cross-core communication scenarios, one side acts
as the "frontend" (VirtIO Driver) and the other side acts as the "backend"
(Vhost Driver, i.e., VirtIO Device implementation).

.. toctree::
   :maxdepth: 1
   :caption: Contents

   framework
