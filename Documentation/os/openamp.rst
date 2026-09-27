===============
OpenAMP Support
===============

Asymmetric Multi Processing support in NuttX is implemented via the
`OpenAMP <https://www.openampproject.org/>`_ framework.

Asymmetric, as opposed to :doc:`SMP </os/scheduling/smp>`, means the cores are
not interchangeable and are not running one operating system between them.
Each has its own memory and its own idea of what it is doing, and they still
have to talk.

Two shapes of that appear in the tree, and it is worth knowing which one you
are in.  ``imx93-qsb`` has a single ``rpmsg`` configuration, for the case
people usually picture: a Cortex-A running Linux, a Cortex-M running NuttX.
But ``nucleo-h745zi`` ships ``nsh_cm7_rptun`` *and* ``nsh_cm4_rptun``, and
``nrf5340-dk`` ships ``rptun_cpuapp`` *and* ``rptun_cpunet`` -- NuttX on both
cores of one part, talking to itself across the divide.  That second shape is
the better represented one here.

What NuttX provides
===================

OpenAMP itself is imported rather than written here.  ``openamp/`` at the top
of the source tree is the recipe, not the code: ``open-amp.defs`` and
``libmetal.defs`` download two pinned upstream releases -- both
``2025.10.0`` as of this writing -- and 22 ``.patch`` files are applied on top.

What NuttX adds is a layer on each side of it, and the order is the opposite
of what the names suggest:

:doc:`RPMSG </os/drivers/special/rpmsg/index>`
   The layer everything else talks to: named channels, so a service on one
   core can be found and addressed from the other.  It lives in
   ``drivers/rpmsg/``, and ``CONFIG_RPMSG`` is what ``select``\ s
   ``CONFIG_OPENAMP``.

:doc:`RPTUN </os/drivers/special/rptun/index>`
   One way to carry RPMSG, *underneath* it rather than above --
   ``CONFIG_RPTUN`` ``select``\ s ``CONFIG_RPMSG_VIRTIO``.  It is the way that
   assumes shared memory: it declares which physical memory both cores see and
   at what address each of them sees it (``struct rptun_addrenv_s`` is a
   ``pa``/``da``/``size`` triple), hands over the resource table, and carries
   the interrupt each side uses to poke the other.

Porting AMP to a new part is mostly writing an RPTUN driver, and that is
because RPTUN does more than carry messages.  Besides ``notify()`` and
``get_resource()``, ``struct rptun_ops_s`` asks for ``get_firmware()``,
``start()``, ``stop()``, ``reset()`` and ``panic()``: the driver owns the
*life* of the remote core, not just the channel to it.

Not all of it is shared memory
==============================

If the two processors are not on the same die there is no shared memory to
declare and no remote core to boot.
:doc:`CONFIG_RPMSG_PORT </os/drivers/special/rpmsg/rpmsg_port>` covers that
case -- its Kconfig help calls it "cross chip communication" -- and it is
itself abstract, with SPI and
:doc:`UART </os/drivers/special/rpmsg/rpmsg_port_uart>` backends under it.
Each backend carries its own CRC option, since a wire can corrupt what shared
memory cannot.
``CONFIG_RPMSG_VIRTIO_LITE`` and ``CONFIG_RPMSG_VIRTIO_IVSHMEM`` are two more
carriers, the second over QEMU's inter-VM shared memory.  Every one of them
``select``\ s ``CONFIG_RPMSG``; none of them needs RPTUN.

What does *not* shrink is OpenAMP itself.  ``CONFIG_RPMSG`` ``select``\ s
``CONFIG_OPENAMP`` whatever the carrier, the generic layer is written against
``<openamp/rpmsg.h>``, and ``openamp/open-amp.defs`` adds all nine of its
sources unconditionally -- ``remoteproc.c``, ``remoteproc_virtio.c``,
``virtio.c``, ``virtqueue.c`` and ``rpmsg_virtio.c`` among them.  A UART link
references none of those, but they are still compiled;
``CONFIG_OPENAMP_VIRTIO_DEVICE_SUPPORT`` and
``CONFIG_OPENAMP_VIRTIO_DRIVER_SUPPORT`` only switch ``-D`` flags, they do not
drop files.  So the cheap part of choosing a wire over shared memory is the
porting work, not the image.

Almost everything is built on RPMSG
===================================

There are 96 ``*_RPMSG`` symbols across the tree, and they follow one idiom
worth learning before reading any of them: a feature arrives as a **pair**.
You enable the client on the core that wants the resource and the server on
the core that owns it -- ``CONFIG_FS_RPMSGFS`` and
``CONFIG_FS_RPMSGFS_SERVER``, ``CONFIG_NET_USRSOCK_RPMSG`` and
``CONFIG_NET_USRSOCK_RPMSG_SERVER``, ``CONFIG_RTC_RPMSG`` and
``CONFIG_RTC_RPMSG_SERVER``, and the same again for block devices, MTD,
syslog, note buffers, Bluetooth and network drivers.

The ``nrf5340-dk`` pair shows the idiom against real hardware.  Its radio
hangs off the network core, so ``rpmsghci_sdc_cpunet`` sets
``CONFIG_BLUETOOTH_RPMSG_SERVER`` and ``rpmsghci_bt_cpuapp`` sets
``CONFIG_BLUETOOTH_RPMSG``; the Kconfig calls them the HCI "server" and
"client" in those words.  The controller stays where the antenna is, the host
stack runs on the application core, and RPMSG is the HCI transport between
them.

That is why the same idea keeps reappearing across this documentation --
``rpmsgfs`` for a file system on the far side, ``CONFIG_CLK_RPMSG`` for clocks
owned by the other core, usrsock over RPMSG for a network stack that lives
elsewhere.  Each is the same shape: the resource is on one core, the caller is
on the other, and RPMSG carries the request.
