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

Intel igc
---------

Intel igc compatible NIC support can be found in ``drivers/net/igc.c``.

Supported devices:

- Intel I225LM
- Intel I226V
