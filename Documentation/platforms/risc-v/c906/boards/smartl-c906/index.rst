===========
smartl-c906
===========

.. tags:: arch:riscv, experimental

.. todo::

   There is currently no support for the FPU, ELF-based file applications,
   protected mode with PMP or RISC-V User mode.

Installation
============

Download and install the toolchain from
https://occ.t-head.cn/community/download.

If you're planning on using NuttX in QEMU for this board, install QEMU from
https://occ.t-head.cn/community/download.

Building NuttX
==============

.. note::

   If configuring to run on QEMU, enable the option ``CONFIG_C906_WITH_QEMU=y``
   in the Kconfig menu (``make menuconfig``).

Flashing
========

Run NuttX by downloading the produced ELF to RAM via a HW debugger.

If running on QEMU, first modify the config file ``smarth_906_cfg.xml`` to
enlarge the RAM size with this patch:

.. code:: diff

   -        <mem name="smart_inst_mem" addr="0x0" size ="0x00020000" attr ="MEM_RAM"></mem>
   +        <mem name="smart_inst_mem" addr="0x0" size ="0x00400000" attr ="MEM_RAM"></mem>
   ...
   -                smart_inst_mem, Start: 0x0, Length: 0x20000
   +                smart_inst_mem, Start: 0x0, Length: 0x400000

Then, you can launch QEMU using the following command:

.. code:: console

   $ ./cskysim -soc $PATH_TO_SOCCFG/smarth_906_cfg.xml -nographic -kernel $PATH_TO_NUTTX_BUILD_DIR/nuttx

Configurations
==============

You can select a configuration using the following command:

.. code:: console

   $ ./tools/configure.sh smartl-c906:<config>

Where ``<config>`` is one of the configurations listed below.

nsh
---

A simple configuration with the NSH shell.
