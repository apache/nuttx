=============================
Zynq UltraScale+ RFSoC ZCU111
=============================

The `ZCU111 <https://www.xilinx.com/products/boards-and-kits/zcu111.html>`_ is a
development board based on the Zynq UltraScale+ RFSoC(XCZU28DR) from XilinX(AMD).

Features
========

- **RF Data Converter**
    - **12-bit ADC:** 8, Max Rate 4.096G
    - **14-bit DAC:** 8, Max Rate 6.554G
    - **SD-FEC:** SD-FEC
- **Memory**
    - **PS DDR4:** 4GB 64-bit SODIMM
    - **SD-Card:** Yes
    - **M.2 SATA Connector:** Yes
    - **QSPI:** 2
- **Communications & Networking**
    - **USB UART/JTAG:** 1
    - **RJ45:** 1
    - **SFP+:** 4
    - **USB 3.0:** 1
- **Expansion Connectors**
    - **FMC-HPC Connector:** 2
    - **PMOD:** 2
    - **RFMC 1.0:** 2
    - **QSPI:** 2
- **Control & I/O**
    - **I2C:** Yes
    - **PMBUS:** Yes
    - **JTAG PC4 Header:** Yes
- **Boot Options**
    - **ISD Boot:** Yes
    - **QSPI Boot:** Yes
    - **JTAG Boot:** Yes
- **DDR4 SODIMM:** 4GB 64-bit, 2400MT/s, attached to Processor Subsystem (PS)

Serial Console
==============

Serial console for the PS:

===== ======== =============
Pin   Signal       Notes
===== ======== =============
MIO18 UART0 TX USB UART COM0
MIO19 UART0 RX USB UART COM0
===== ======== =============

PS-side UART interface and is connected to the FTDI U34 FT4232HL USB-to-Quad-UART
bridge port B Connect ZCU111 to our computer with the USB Cable. On our computer
start a Serial Terminal and connect to the USB Serial Port at **115200 bps**.
NuttX will appear in the Serial Console when it boots on zcu111.

LEDs and Buttons
================

The PS-side pushbutton SW19 is connected to MIO22 (pin U1.Y28). The PS-side LED DS50,
which is physically placed adjacent to the pushbutton, is connected to MIO23(pin U1.U29).

Configurations
==============

Each configuration is maintained in a sub-directory and can be selected as follow::

  tools/configure.sh zcu111:<subdir>

Where <subdir> is one of the following:

jtag
----

Basic NuttShell configuration for JTAG boot mode (nsh console enabled in UART0,
UART and JTAG exposed via FT4232HL USB-to-Quad-UART bridge port and USB cable).

nsh
---

Basic NuttShell configuration for Flash boot mode. We need create boot image with
zynqmp_fsbl.elf, zynqmp_pmufw.elf, bl31.elf and nuttx.elf in Vivado SDK or XSCT
shell. Also we need copy BOOT.BIN into SD Card(in SD card boot mode) or Flash it
into the QSPI FLASH(in QSPI boot mode).

ARM64 Toolchain
===============

There are two ways to install the toolchain for Zynq MPSoC:
The first way is download the ARM64 Toolchain ``aarch64-none-elf`` from
`Arm GNU Toolchain Downloads <https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads>`_.
Add the downloaded toolchain ``gcc-arm-...-aarch64-none-elf/bin``
to the ``PATH`` Environment Variable such as:

.. code-block:: console

  $ echo "export PATH=/home/username/tools/gcc-arm-11.2-2022.02-x86_64-aarch64-none-elf/bin:$PATH" >> ~/.profile

You can edit your .profile files if you don't use bash.

The second way is install Vivado SDK or Vitis development environment which included a complete
``aarch64-none-elf`` toolchain and we also add it to the ``PATH`` Environment Variable such as:

.. code-block:: console

  $ echo "export PATH=/home/username/tools/Xilinx/SDK/2018.3/gnu/aarch64/lin/aarch64-none/bin:$PATH" >> ~/.profile

You can edit your .profile files if you don't use bash.

Note: nuttx.elf build by toolchain install in first way can't be debuged by Vivado SDK which use
toolchain of second way for gdb version incompatibility.

Check the ARM64 Toolchain:

.. code:: console

   $ aarch64-none-elf-gcc -v

Building
========

There are two types of NuttX image for Zynq MPSoC: debug by JTAG and boot from FLASH.

debug by jtag
-------------

We just configure the NuttX project and build the project:

.. code:: console

   $ cd nuttx
   $ tools/configure.sh zcu111:jtag
   $ make

Set the Project to nuttx and Application to nuttx.elf for psu_cortexa53_0 in Vivado SDK Debug Configuration.
Just click Debug button then we can debug NuttX.

boot from flash
---------------

To boot from FLASH, we have to create BOOT.BIN image and flash it into QSPI FLASH or SD card. To create BOOT.BIN
in addition to building nuttx.elf, we also need to build zynqmp_fsbl.elf, zynqmp_pmufw.elf and bl31.elf
To build nuttx.elf we just configure the NuttX project and build the project:

.. code:: console

   $ cd nuttx
   $ tools/configure.sh zcu111:nsh
   $ make

build bl31.elf
--------------

To build bl31.elf we should fetch Fetch sources of ARM Trusted Firmware (ATF) and checkout the tags that
corresponding to the SDK version. Take Vivado 2018.3 for example:

.. code:: console

   $ git clone https://github.com/Xilinx/arm-trusted-firmware.git
   $ cd arm-trusted-firmware
   $ git checkout xilinx-v2018.3

By default, the Arm-trusted firmware builds for OCM space at address 0xFFFEA000, and ATF assume that UBoot
or nuttx.elf located at address 0x08000000. Then we just build bl31.elf with:

.. code:: console

   $ make CROSS_COMPILE=aarch64-none-elf- PLAT=zynqmp RESET_TO_BL31=1

But, with DEBUG flag set to 1, it can't fit in OCM, so by default with DEBUG=1, it builds for DDR location
0x1000 with build flag DEBUG=1 mentioned while building. Alternatively, user has always an option to build
for the location of their choice by specifying  the build flags ZYNQMP_ATF_MEM_BASE, ZYNQMP_ATF_MEM_SIZE while
building. The flag ZYNQMP_ATF_MEM_BASE specifies the base address of ATF and flag ZYNQMP_ATF_MEM_SIZE specifies
the maximum size the ATF image can be. what's more we can specifies the target address of Uboot or nuttx.elf
by PRELOADED_BL33_BASE. for zcu111:nsh configuration Example bl31 build command:

.. code:: console

   $ make CROSS_COMPILE=aarch64-none-elf- PLAT=zynqmp RESET_TO_BL31=1 ZYNQMP_ATF_MEM_BASE=0x10000 ZYNQMP_ATF_MEM_SIZE=0x40000 PRELOADED_BL33_BASE=0x100000

If we don't dubug bl31 we just build bl31 in following command:

.. code:: console

   $ make CROSS_COMPILE=aarch64-none-elf- PLAT=zynqmp RESET_TO_BL31=1 PRELOADED_BL33_BASE=0x100000

After the build process completes the bl31.elf binary is created within the /build/zynqmp/release/bl31 directory.

build zynqmp_pmufw.elf
----------------------

The Platform Management Unit (PMU) in Zynq MPSoC has a Microblaze with 32 KB of ROM and 128 KB of RAM. The ROM is
pre-loaded with PMU Boot ROM (PBR) which performs pre-boot tasks and enters a service mode. For more details on PMU,
PBR and PMUFW load sequence, refer to Platform Management Unit (Chapter-6) in Zynq MPSoC TRM (UG1085). PMU RAM can
be loaded with a firmware (PMU Firmware) at run-time and can be used to extend or customize the functionality of PMU.
Some part of the RAM is reserved for PBR, leaving around 125.7 KB for PMU Firmware.
There are usually two flows to create and build a PMU Firmware image for the target, Xilinx Vitis or Vivado SDK IDE or
hsi command line. The PMU Firmware is provided as a template application for the PMU processor for any hardware platform
including the Zynq MPSoC device. The steps required to create and build it can be applied by selecting the appropriate
platform, processor, and template to create zynqmp_pmufw.elf. We can also create PMU Firmware from system hardware
project hdf file by hsi command line:

.. code-block::

  proc generate_pmufw {} {
      if {[file exists pmu_fw/zynqmp_pmufw.elf] != 1} {
          set pmufw_design [hsi::create_sw_design pmu_1 -proc psu_pmu_0 -app zynqmp_pmufw]
          hsi::add_library libmetal
          hsi::generate_app -dir pmu_fw -compile
          return "pmu_fw/zynqmp_pmufw.elf"
      }
      return "pmu_fw/zynqmp_pmufw.elf"
  }

In order to call this procs, the user needs to open the hdf (hsi::open_hw_design):

.. code-block::

  proc create_pmufw {hdf} {
      hsi::open_hw_design $hdf
      set pmufw [generate_pmufw]
      hsi::close_hw_design [hsi::current_hw_design]
  }

Create a TCL script with HSI commands above -> Create a TCL script with HSI commands above ->
Launch XSCT 2018.3 -> Change directory to the zipped directory -> source xsct_script.tcl ->
create_pmufw design_1_wrapper.hdf

build zynqmp_fsbl.elf
---------------------

First Stage Bootloader (FSBL) for Zynq UltraScale+ MPSoC configures the FPGA with hardware bitstream (if it exists)
and loads the Operating System (OS) Image or Standalone (SA) Image or 2nd Stage Boot Loader image from the non-volatile
memory (SD/eMMC/QSPI) to Memory (DDR/TCM/OCM) and takes A53/R5 out of reset. It supports multiple partitions, and each
partition can be a code image or a bitstream. Each of these partitions, if required, will be authenticated and/or decrypted.
FSBL is loaded into OCM and handed off by CSU BootROM after authenticating and/or decrypting (as required) FSBL.
There are usually two flows to create and build a PMU Firmware image for the target, Xilinx Vitis or Vivado SDK IDE or
hsi command line. 
To create FSBL by Vitis or Vivado SDK IDE just launch VITIS or Vivado SDK and do following flow:

- Provide path where VITIS workspace and project need to be created. With this VITIS workspace will be created
- (Optional step) To work with local repos, Select "Xilinx" (ALT - x) -> Repositories. Against Local Repositories,
  click on "New..." and provide path of the local repo
- Select File-->New-->Application Project to open "New Project" window, provide name for FSBL project
- In the “Platform” section, click on “Create a new platform from hardware (XSA)” and select pre-defined hardware platform for ZynqMP.
    - Alternatively, to create a new/custom platform from a .xsa file, click on “+”, browse and select the XSA file and a new hardware platform is created.
- In the "Domain" window, select the processor psu_cortexa53_0/psu_cortexr5_0, OS as standalone and Language as C.
- Click Next and select "Zynq MP FSBL"
- Click "Finish" to generate the A53/R5 FSBL. This populates the FSBL code and also builds it (along with BSP)
- Debug prints in FSBL are now disabled by default. To enable debug prints, define symbol: FSBL_DEBUG_INFO.
    - In VITIS this can be done by: right click on FSBL application project -> select “C/C++ Build Settings” -> “Tool Settings” tab -> Symbols (under ARM v8 gcc compiler)
    - Click on Add (+) icon and Enter Value: FSBL_DEBUG_INFO, click on "OK" to close the "Enter Value" screen
- In case any of the source files (FSBL or BSP) need to be modified, browse the file, make the change and save the file,
  build the project. elf file will be present in the Debug/Release folder of FSBL project.

To create FSBL by XSCT command line just launch XSCT console and execute following TCL script with HSI commands:

.. code-block::

  proc generate_fsbl {} {
      if {[file exists zynqmp_fsbl/zynqmp_fsbl.elf] != 1} {
          set fsbl_design [hsi::create_sw_design fsbl_1 -proc psu_cortexa53_0 -app zynqmp_fsbl]
          common::set_property APP_COMPILER "aarch64-none-elf-gcc" $fsbl_design
          common::set_property -name APP_COMPILER_FLAGS -value "-DRSA_SUPPORT -DFSBL_DEBUG_INFO -DXPS_BOARD_ZCU111" -objects $fsbl_design
          hsi::add_library libmetal
          hsi::generate_app -dir zynqmp_fsbl -compile
      }
      return "zynqmp_fsbl/zynqmp_fsbl.elf"
  }

In order to call this procs, the user needs to open the hdf (hsi::open_hw_design):

.. code-block::

  proc create_fsbl {hdf} {
      hsi::open_hw_design $hdf
      set fsbl [generate_fsbl]
      hsi::close_hw_design [hsi::current_hw_design]
  }

Create a TCL script with HSI commands above -> Create a TCL script with HSI commands above ->
Launch XSCT 2018.3 -> Change directory to the zipped directory -> source xsct_script.tcl ->
create_fsbl design_1_wrapper.hdf

generate BOOT.bin image
-----------------------

You can create BOOT.bin images using the BIF attributes and the Bootgen command.
For this configuration, the BIF file(named fsbl.bif) contains the following attributes:

.. code-block::

  the_ROM_image:
  {
    [fsbl_config]a53_x64
    [bootloader]zynqmp_fsbl.elf
    [pmufw_image]zynqmp_pmufw.elf
    [destination_cpu = a53-0, exception_level = el-3, trustzone]bl31.elf
    [destination_cpu = a53-0, exception_level = el-1]nuttx.elf
  }

The Vitis IDE calls the following Bootgen command to generate the BOOT.bin image for this configuration:

.. code-block::

  bootgen -image fsbl.bif -arch zynqmp -o .\BOOT.bin

Flash BOOT.bin to QSPI FLASH
----------------------------

We can flash BOOT.bin into QSPI FLASH in following flow:

- In the Vivado SDK/Vitis IDE, select Xilinx -> Program Flash.
- In the Program Flash wizard, browse to and select the BOOT.bin image file that was created as a part of this example.
- Select **qspi-x8-dual_parallel** as the Flash type.
- Set the Offset as 0 and select the BOOT.bin file.
- Click Program to start the process of programming the QSPI flash with the BOOT.bin.
- Wait until you see the message “Flash Operation Successful” in the console.

Set mode switch SW6 to QSPI32, NuttX will appear in the Serial Console when we power on zcu111.
