==================
TRIBOARD_TC4X9_COM
==================

This port should work on TRIBOARD_TC4X9_COM with a proper CPU.
The mandatory CPU features are:

* System Timer (STM)
* Asynchronous Serial Interface(ASCLIN) UART
* IRQs are managed by Interrupt Router(INT), IR Service Request Control Registers(SRC).

Toolchains
==========

Currently, only the Infineon’s AURIX™ GCC toolchain is tested.

Configurations
==============

Common Configuration Notes
--------------------------

1. Each TRIBOARD_TC4X9_COM configuration is maintained in a sub-directory
   and can be selected as follow::

     tools/configure.sh triboard_tc4x9_com:<subdir>

   Where ``<subdir>`` is one of the configuration sub-directories described in
   the following paragraph.

   NuttX Shell::

     tools/configure.sh triboard_tc4x9_com:nsh

2. These configurations use the mconf-based configuration tool.  To
   change a configurations using that tool, you should:

   a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
      see additional README.txt files in the NuttX tools repository.

   b. Execute ``make menuconfig`` in nuttx/ in order to start the
      reconfiguration process.

3. By default, all configurations assume the Linux.  This is easily
   reconfigured::

     CONFIG_HOST_LINUX=y

Configuration Sub-Directories
-----------------------------

ostest
------

The "standard" NuttX examples/ostest configuration.
