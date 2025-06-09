==================
PCI(e) Bus Drivers
==================

PCI(e) bus driver can be found in ``drivers/pci``.

Supported PCI devices
=====================

PCI QEMU Test Device
--------------------

Test device provided by QEMU and enabled with ``-device pci-testdev``.

PCI QEMU EDU Device
-------------------

Test device provided by QEMU and enabled with ``-device edu``.

Inter-VM share memory Device (ivshmem)
--------------------------------------

Inter-VM shared memory support support can be found in ``drivers/pci/pci_ivshmem.c``.

This implementation is for ``ivshmem-v1`` which is compatible with QEMU and
ACRN hypervisor but won't work with Jailhouse hypervisor which uses ``ivshmem-v2``.

16550 Compatible Serial Card
----------------------------

UART 16550 compatible PCI serial card support can be found
in ``drivers/serial/uart_pci_16550.c``.

Supported devices:

- AX99100
- QEMU pci-serial device
- QEMU pci-serial-2x device
- QEMU pci-serial-4x device

Intel e1000
-----------

Intel e1000 compatible NIC support can be found in ``drivers/net/e1000.c``.

Supported devices:

- Intel I219
- Intel 82540EM
- Intel 82574L
- Intel 82574L

Intel igb
---------

Intel igb compatible NIC support can be found in ``drivers/net/igb.c``.

Supported devices:

- Intel 82576
- Intel I211

Intel igc
---------

Intel igc compatible NIC support can be found in ``drivers/net/igc.c``.

Supported devices:

- Intel I225LM
- Intel I226V

Kvaser PCI CAN card
-------------------

At the moment the card only works with QEMU.

The driver supports both SocketCAN interface and character driver.

The driver requires, ``vcan`` to run on the host:

.. code:: shell

   sudo ip link add dev can0 type vcan
   sudo ip link set can0 up

An example command to run the driver on ``x86_64`` looks like this:

.. code:: shell

   qemu-system-x86_64 -m 2G -cpu host -enable-kvm -kernel nuttx \
   -nographic -serial mon:stdio -object can-bus,id=canbus0 \
   -object can-host-socketcan,id=canhost0,if=can0,canbus=canbus0 \
   -device kvaser_pci,canbus=canbus0


CTUCANFD PCI CAN card
---------------------

At the moment the card only works with QEMU.

The driver supports both SocketCAN interface and character driver.

The driver requires, ``vcan`` to run on the host:

.. code:: shell

   sudo ip link add dev can0 type vcan
   sudo ip link set can0 up

An example command to run the driver on ``x86_64`` looks like this:

.. code:: shell

   qemu-system-x86_64 -m 2G -cpu host -enable-kvm -kernel nuttx \
   -nographic -serial mon:stdio -object can-bus,id=canbus0-bus \
   -object can-host-socketcan,if=can0,canbus=canbus0-bus,id=canbus0-socketcan \
   -device ctucan_pci,canbus0=canbus0-bus,canbus1=canbus0-bus

xHCI over PCI
-------------

xHCI support was tested on x86_64 target, both on QEMU and real hardware.

Known issues with this driver:

- Currently only USB2.0 is supported, there is no USB3.0 support in NuttX yet.

- USB HUB devices not supported yet

To enable xHCI support on QEMU, we have to add ``-device qemu-xhci`` argument.
All supported USB devices in QEMU can be found in
`QEMU documentation <https://qemu-project.gitlab.io/qemu/system/devices/usb.html>`_,
but not all classes are supported in NuttX.
