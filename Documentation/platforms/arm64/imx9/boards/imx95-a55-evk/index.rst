=====================
i.MX95 Evaluation Kit
=====================

.. tags:: chip:imx95, arch:arm64

The kit i.MX95 Evaluation Kit has a pre-installed Linux image which contains
u-boot and the i.MX95 reference Linux installation.

Below is a set of instructions on how to run NuttX on the i.MX95 EVK, on top
of the u-boot.

U-Boot configuration
--------------------

Two things need to be configured on u-boot before NuttX can be loaded:

* u-boot data cache must be turned off
* u-boot must stop to the u-boot console, i.e. the Linux payload must not be
  loaded

--------------
Manual option:
--------------

1. Disable u-boot autostart (needs to be done only once):

   .. code:: console

      Hit any key to stop autoboot:  0
      u-boot=> setenv bootdelay -1
      u-boot=> saveenv
      Saving Environment to MMC... Writing to MMC(0)... OK
      u-boot=> reset

2. On every boot, the data cache must be disabled for options 2 and 3 to work

   .. code:: console

      u-boot=> dcache off

-----------------
Automated option:
-----------------

Replace the default bootcmd to disable dcache automatically:

.. code:: console

   u-boot=> setenv bootdelay 0
   u-boot=> setenv bootcmd dcache off
   u-boot=> saveenv
   Saving Environment to MMC... Writing to MMC(0)... OK
   u-boot=> reset

To restore the default bootcmd which starts Linux automatically:

.. code:: console

    u-boot=> setenv bootcmd run distro_bootcmd;run bsp_bootcmd
    u-boot=> saveenv
    Saving Environment to MMC... Writing to MMC(0)... OK
    u-boot=> reset

The default bootcmd is:

.. code:: console

   u-boot=> env print bootcmd
   bootcmd=run distro_bootcmd;run bsp_bootcmd

Loading and running the NuttX image
===================================

You have four options:

1. Load via u-boot from SD-card
2. Run from SD-card, without u-boot


Option 1: load via u-boot from SD-card:
---------------------------------------

1. Build nuttx, and move ``nuttx.bin`` to MMC

2. Load from MMC and start nuttx payload

   .. code:: console

      u-boot=> dcache off; fatload mmc 0 0xa0100000 nuttx.bin; go 0xa0100000
