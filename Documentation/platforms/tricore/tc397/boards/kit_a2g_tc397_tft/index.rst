=================
KIT_A2G_TC397_TFT
=================

This page file describes the contents of the build configurations available
for the NuttX TriCore port.

Infineon’s AURIX™- TC3xx `KIT_A2G_TC397_TFT <https://www.infineon.com/cms/en/product/evaluation-boards/kit_a2g_tc397_5v_tft>`__ Board
=====================================================================================================================================

This port should work on KIT_A2G_TC397_TFT with a proper CPU.
The mandatory CPU features are:

* System Timer (STM)
* Asynchronous Serial Interface(ASCLIN) UART
* IRQs are managed by Interrupt Router(INT), IR Service Request Control Registers(SRC).

Toolchains
==========

Currently, only the Infineon’s AURIX™ Tasking toolchain is tested.

Configurations
==============

Common Configuration Notes
--------------------------

1. Each Tricore TC397 configuration is maintained in a sub-directory
   and can be selected as follow::

     tools/configure.sh tc397:<subdir>

   Where ``<subdir>`` is one of the configuration sub-directories described in
   the following paragraph.

   NuttX Shell::

     tools/configure.sh tc397:nsh

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
