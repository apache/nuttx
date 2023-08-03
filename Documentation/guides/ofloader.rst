=================
Open Flash Loader
=================

Introduce
=====================================

Open Flash loader is an implementation in NuttX that bridges the device
drivers in NuttX with the programming methods of a flash loader in J-Link.
It allows J-Link to program any supported NuttX device driver.

CONFIGURATION
===============

.. code-block:: c
  CONFIG_DISABLE_IDLE_LOOP=y
  CONFIG_SYSTEM_OFLOADER=y
  CONFIG_SYSTEM_OFLOADER_TABLE="/dev/flash,0x08000000,0x20000"，0

```CONFIG_DISABLE_IDLE_LOOP`` is used to disable the idle loop in NuttX.
```CONFIG_SYSTEM_OFLOADER``` is used to enable the Open Flash Loader.
```CONFIG_SYSTEM_OFLOADER_TABLE``` is used to configure the flash device
frist parameter is the device name, second parameter is the start address.

The reference configuration "stm32f429i-disco:ofloader" is designed
to be used with the STM32F429I-DISCO board in NuttX,
enabling the Open Flash loader functionality. This configuration allows
the J-Link flash loader to program the NuttX image onto the STM32F429I-DISCO board.

Usage
=====
1. Build NuttX with Open Flash Loader.
2. Configure the Jlink xml file. like

.. code-block::
  <Database>
      <Device>
          <ChipInfo Vendor="STM32NUTTX" Name="NuttX" Core="JLINK_CORE_CORTEX_M4" WorkRAMAddr="0x20000000" WorkRAMSize="0x10000000" />
          <FlashBankInfo Name="Storage" BaseAddr="0x00000000" MaxSize="0xffffffff" Loader="/home/ajh/work/vela_all/nuttx/nuttx" LoaderType="FLASH_ALGO_TYPE_OPEN" AlwaysPresent="1"/>
      </Device>
  </Database>

3. Use ``JLinkExe -if SWD -speed 4000 -device STM32NUTTX``,
   then ``loadbin /home/ajh/work/nuttx.bin 0x08000000``,
   we can flash nuttx.bin to /dev/flash

Precautions
===========

1.If you need to implement the ofloader on a different board,
you will need to read the `wiki <https://wiki.segger.com/SEGGER_Flash_Loader>`
and refer to the implementation of "ofloader.ld" linker script located
in the "boards/arm/stm32/stm32f429i-disco/scripts" directory.
This linker script defines how the different sections of the NuttX image are placed in memory.
You should configure the corresponding sections to be located in RAM,
where the J-Link can write the image correctly.

2.When running the ofloader, J-Link disables any interrupts by default.
Therefore, currently, it does not support programming devices with interrupt-driven drivers.

3.Due to limitations of J-Link, the ofloader image file cannot exceed 64KB.
It is essential to be mindful of this constraint during programming.

