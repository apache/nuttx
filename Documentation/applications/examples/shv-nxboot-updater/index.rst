===========================================================
``shv-nxboot-updater`` Silicon Heaven firmware updater
===========================================================

This example implements firmware updates over the Silicon Heaven (SHV)
protocol for devices running the :doc:`NXBoot </applications/boot/nxboot/index>`
bootloader. It exposes the NXBoot update partition as an SHV file node,
so a new image can be uploaded through an SHV broker, verified by CRC,
rebooted and finally confirmed as stable.

Configuration
-------------

- ``CONFIG_EXAMPLES_SHV_NXBOOT_UPDATER`` – Enable the example. Depends on
  ``CONFIG_NETUTILS_LIBSHVC`` and ``CONFIG_ENABLE_ALL_SIGNALS``. NXBoot
  (``CONFIG_BOOT_NXBOOT``) must be configured as well.
- ``CONFIG_EXAMPLES_SHV_NXBOOT_UPDATER_PROGNAME`` – Program name. Default:
  ``shv_nxboot_updater``.
- ``CONFIG_EXAMPLES_SHV_NXBOOT_UPDATER_PRIORITY`` – Task priority. The SHV
  communication thread runs one priority level below. Default: ``100``.
- ``CONFIG_EXAMPLES_SHV_NXBOOT_UPDATER_STACKSIZE`` – Task stack size.
  Default: ``4096``.

Created nodes
-------------

All nodes are mounted under the mount point given by ``-m``. Every node also
provides the standard ``dir`` and ``ls`` methods.

=================  ============================================================
Node               Methods
=================  ============================================================
``.app``           ``shvVersionMajor``, ``shvVersionMinor``, ``name``,
                   ``version``, ``ping``, ``date``
``.device``        ``name``, ``version``, ``serialNumber``, ``uptime``,
                   ``reset``
``fwUpdate``       File node backed by the NXBoot update partition (MTD):
                   ``stat``, ``size``, ``write``, ``crc``
``fwStable``       ``get`` – return the confirmation state of the running
                   image (``nxboot_get_confirm()``),
                   ``confirm`` – mark the running image as stable
                   (``nxboot_confirm()``)
=================  ============================================================

Usage
-----

::

  shv_nxboot_updater -m <mount-point> -i <ip-addr> -t <port> -u <user> -p <passwd>

- ``-m`` – mount point in the SHV broker.
- ``-i`` – IPv4 address of the SHV broker.
- ``-t`` – TCP port of the SHV broker.
- ``-u`` / ``-p`` – user name and password for the broker.
- ``-h`` – print help.

All options except ``-h`` are mandatory. The application is typically started
at boot, e.g. from the NSH startup script::

  nsh> shv_nxboot_updater -m test/nuttxdevice -i 192.168.1.10 -t 3755 -u xyz -p xyz &

The application runs until it receives ``SIGTERM`` or until an error is
reported by the SHV connection.

A typical update cycle is:

1. Write the new NXBoot image (``nuttx.nximg``) by chunks to ``fwUpdate``
   using ``write``.
2. Compare the result of ``crc`` with the CRC32 of the uploaded image.
3. Call ``.device:reset``. NXBoot then boots the new image.
4. Once the new firmware is verified to work, call ``fwStable:confirm``.
   Otherwise NXBoot reverts to the previous image on the next reset.

Update script
-------------

The steps above are automated by a Python script located in
``apps/examples/shv-nxboot-updater/update-script``. Install its dependencies
first::

  $ cd apps/examples/shv-nxboot-updater/update-script
  $ pip install -r requirements.txt

Options of ``tui.py``:

- ``-s``, ``--server`` – SHV broker URL. Default:
  ``tcp://xyz@127.0.0.1:3755?password=xyz``.
- ``-m``, ``--mount`` – mount point of the device in the broker. Default:
  ``test/nuttxdevice``.
- ``-i``, ``--image`` – path to the NXBoot image. Default: ``nuttx.nximg``.
- ``-o``, ``--operation`` – ``flash`` (default) or ``confirm``.
- ``-v`` / ``-q`` – increase / decrease logging verbosity.

The ``flash`` operation uploads the image, checks the CRC and resets the
device. After the device has rebooted into the new firmware, the
``confirm`` operation marks it as stable::

  $ python3 tui.py -s "tcp://xyz@192.168.1.10:3755?password=xyz" \
      -m test/nuttxdevice -i nuttx.nximg -o flash
  $ python3 tui.py -s "tcp://xyz@192.168.1.10:3755?password=xyz" \
      -m test/nuttxdevice -o confirm
