=============
``select.py``
=============

This tool is written in Python and is intended to run as part of the CI
workflow. The primary purpose of this tool is to map a set of changed files to a
set of ``defconfig`` files (NuttX configurations) for build testing. The number
of selected ``defconfig`` files should be the minimum possible for full build
coverage.

Examples
========

For now, any files that are modified outside of the ``arch/`` and ``board/``
directories require a build of every in-tree configuration. This is because
there is currently no method of detecting which configurations are dependent on
which source files. A modified driver could be included anywhere (although in
practice, things like sensor drivers are probably in <10 configurations, so this
is wasteful).

.. code:: console

   $ tools/ci/build-selector/select.py drivers/sensors/lis2dh.c
   boards/x86/qemu/qemu-i486/configs/ostest/defconfig
   boards/x86/qemu/qemu-i486/configs/nsh/defconfig
   boards/or1k/mor1kx/or1k/configs/nsh/defconfig
   boards/x86_64/qemu/qemu-intel64/configs/ostest/defconfig
   boards/x86_64/qemu/qemu-intel64/configs/jumbo/defconfig
   boards/x86_64/qemu/qemu-intel64/configs/nsh_pci/defconfig
   boards/x86_64/qemu/qemu-intel64/configs/fb/defconfig
   boards/x86_64/qemu/qemu-intel64/configs/nsh_pci_smp/defconfig
   boards/x86_64/qemu/qemu-intel64/configs/nsh/defconfig
   boards/x86_64/qemu/qemu-intel64/configs/lvgl/defconfig
   ...
   # Full output omitted for brevity!

If only a single ``defconfig`` file is modified, it is the only file that should
be built!

.. code:: console

   $ tools/ci/build-selector/select.py boards/arm64/bcm2711/raspberrypi-4b/configs/nsh/defconfig
   boards/arm64/bcm2711/raspberrypi-4b/configs/nsh/defconfig

If only a single board has modifications, we should build only ``defconfig``
files associated with that board.

.. code:: console

   $ tools/ci/build-selector/select.py boards/arm64/bcm2711/raspberrypi-4b/src/bcm2711_i2cdev.c
   boards/arm64/bcm2711/raspberrypi-4b/configs/ostest/defconfig
   boards/arm64/bcm2711/raspberrypi-4b/configs/sd/defconfig
   boards/arm64/bcm2711/raspberrypi-4b/configs/fb/defconfig
   boards/arm64/bcm2711/raspberrypi-4b/configs/nsh/defconfig
   boards/arm64/bcm2711/raspberrypi-4b/configs/lvgl/defconfig
   boards/arm64/bcm2711/raspberrypi-4b/configs/cgol/defconfig
   boards/arm64/bcm2711/raspberrypi-4b/configs/coremark/defconfig

If only a single chip has modifications, then all ``defconfig`` files associated
with that chip should be built.

.. code:: console

   $ tools/ci/build-selector/select.py arch/arm64/src/bcm2711/bcm2711_mailbox.c
   boards/arm64/bcm2711/raspberrypi-4b/configs/ostest/defconfig
   boards/arm64/bcm2711/raspberrypi-4b/configs/sd/defconfig
   boards/arm64/bcm2711/raspberrypi-4b/configs/fb/defconfig
   boards/arm64/bcm2711/raspberrypi-4b/configs/nsh/defconfig
   boards/arm64/bcm2711/raspberrypi-4b/configs/lvgl/defconfig
   boards/arm64/bcm2711/raspberrypi-4b/configs/cgol/defconfig
   boards/arm64/bcm2711/raspberrypi-4b/configs/coremark/defconfig

And finally, if an architecture undergoes a modification, all ``defconfig``
files associated with that architecture should be built.

.. code:: console

   $ tools/ci/build-selector/select.py arch/arm64/Kconfig
   boards/arm64/bcm2711/raspberrypi-4b/configs/ostest/defconfig
   boards/arm64/bcm2711/raspberrypi-4b/configs/sd/defconfig
   boards/arm64/bcm2711/raspberrypi-4b/configs/fb/defconfig
   boards/arm64/bcm2711/raspberrypi-4b/configs/nsh/defconfig
   boards/arm64/bcm2711/raspberrypi-4b/configs/lvgl/defconfig
   boards/arm64/bcm2711/raspberrypi-4b/configs/cgol/defconfig
   boards/arm64/bcm2711/raspberrypi-4b/configs/coremark/defconfig
   boards/arm64/rk3399/pinephonepro/configs/nsh/defconfig
   boards/arm64/rk3399/nanopi_m4/configs/nsh/defconfig
   boards/arm64/a64/pinephone/configs/sensor/defconfig
   boards/arm64/a64/pinephone/configs/nsh/defconfig
   boards/arm64/a64/pinephone/configs/lvgl/defconfig
   boards/arm64/a64/pinephone/configs/lcd/defconfig
   boards/arm64/qemu/qemu-armv8a/configs/fastboot/defconfig
   boards/arm64/qemu/qemu-armv8a/configs/netnsh/defconfig
   boards/arm64/qemu/qemu-armv8a/configs/sotest/defconfig
   boards/arm64/qemu/qemu-armv8a/configs/citest_smp/defconfig
   boards/arm64/qemu/qemu-armv8a/configs/mte/defconfig
   boards/arm64/qemu/qemu-armv8a/configs/nsh_gicv2/defconfig
   boards/arm64/qemu/qemu-armv8a/configs/netnsh_hv/defconfig
   boards/arm64/qemu/qemu-armv8a/configs/sw_tags/defconfig
   boards/arm64/qemu/qemu-armv8a/configs/rpserver/defconfig
   boards/arm64/qemu/qemu-armv8a/configs/nsh_smp_tickless/defconfig
   boards/arm64/qemu/qemu-armv8a/configs/netnsh_smp_hv/defconfig
   boards/arm64/qemu/qemu-armv8a/configs/fb/defconfig
   boards/arm64/qemu/qemu-armv8a/configs/xedge_demo/defconfig
   boards/arm64/qemu/qemu-armv8a/configs/nsh/defconfig
   # remaining configurations omitted for brevity


This tool can also handle any combinations of the above; it always selects the
minimum defconfigs for the change set. For instance, modifying ``rp23xx`` common
logic and a Raspberry Pi 4B configuration:

.. code:: console

   $ tools/ci/build-selector/select.py arch/arm/src/rp23xx/rp23xx_idle.c boards/arm64/bcm2711/raspberrypi-4b/configs/sd/defconfig
   boards/arm/rp23xx/raspberrypi-pico-2/configs/spisd/defconfig
   boards/arm/rp23xx/raspberrypi-pico-2/configs/nsh/defconfig
   boards/arm/rp23xx/raspberrypi-pico-2/configs/userled/defconfig
   boards/arm/rp23xx/raspberrypi-pico-2/configs/usbnsh/defconfig
   boards/arm/rp23xx/raspberrypi-pico-2/configs/smp/defconfig
   boards/arm/rp23xx/pimoroni-pico-2-plus/configs/audiopack/defconfig
   boards/arm/rp23xx/pimoroni-pico-2-plus/configs/nsh/defconfig
   boards/arm/rp23xx/pimoroni-pico-2-plus/configs/nshsram/defconfig
   boards/arm/rp23xx/pimoroni-pico-2-plus/configs/userled/defconfig
   boards/arm/rp23xx/pimoroni-pico-2-plus/configs/composite/defconfig
   boards/arm/rp23xx/pimoroni-pico-2-plus/configs/usbmsc/defconfig
   boards/arm/rp23xx/pimoroni-pico-2-plus/configs/usbnsh/defconfig
   boards/arm/rp23xx/pimoroni-pico-2-plus/configs/smp/defconfig
   boards/arm/rp23xx/xiao-rp2350/configs/combo/defconfig
   boards/arm/rp23xx/xiao-rp2350/configs/nsh/defconfig
   boards/arm/rp23xx/xiao-rp2350/configs/usbnsh/defconfig
   boards/arm64/bcm2711/raspberrypi-4b/configs/sd/defconfig
