==============
ET-STM32 Stamp
==============

This README discusses issues/thoughts unique to NuttX configuration(s) for the
ET-STM32 Stamp board from Futurlec (https://www.futurlec.com/ET-STM32_Stamp.shtml).

Microprocessor: 32-bit ARM Cortex M3 at 72MHz STM32F103RET6
Memory:         512 KB Flash and 64 KB SRAM
I/O Pins Out:   48
ADCs:           16 (at 12-bit resolution)
DACs:           2 (at 12-bit resolution)
Peripherals:    RTC, 4 timers, 2 I2Cs, 3 SPI ports, 1 on-board UART (upto 5 channels)
Other:          Sleep, stop, and standby modes; serial wire debug and JTAG interfaces

Please see link below for board specific details:

    https://www.futurlec.com/ET-STM32_Stamp_Technical.shtml

This configuration supports the ET-STM32 Stamp module.

Development Environment
=======================

Either Linux (recommended), Mac or Cygwin on Windows can be used for the development
environment.  The source has been built only using the GNU (Cortex M) toolchain.
Other toolchains will likely cause problems.

WSL (Windows Subsystem for Linux) was used to develop, compile and test the NuttX
build for the ET-STM32 Stamp platform.

Flashing/Programming
====================

Prerequisites:

1. The ET-STM32 Stamp module from Futurlec.

2. An RS232 connection cable such as the one in this link: (Part code: RS232CONN):
   https://www.futurlec.com/DevBoardAccessories.shtml

   It has a 4-pin connection header on one end and an RS-232 (DB9) female connector on
   the other. The 4-pin connector can be directly plugged onto the Stamp module.

3. An RS232 to USB converter cable. Ensure that a suitable driver is installed for
   the converter cable. When the cable is plugged in (for example), my PC lists the
   assigned port with this name: "USB-SERIAL CH340 (COM2)".

   Assuming Windows 10, navigate to: This PC -> Manage -> Device Manager -> Ports.

4. ST's Flash loader demonstrator tool. You can download it from here:
   https://www.st.com/en/development-tools/flasher-stm32.html

   To install the NuttX firmware (nuttx.bin) on the ET-STM32 Stamp:

1. First, power the Stamp module with a 3.3 VDC power supply. I made my own
   Stamp module fixture using a 3.3 VDC switching regulator, a prototype PCB card
   and some solder.

2. Insert the RS232CONN into the 4-pin on-board header. The other end should be
   connected to the USB port of the PC using the RS232-USB converter.

3. Set the BOOT1 jumper on your board to the ISP position.

4. Press the BOOT0 switch. The green "BOOT0=1" LED should light up.

5. Reset the board by pressing on the RESET button.

6. Using the ST Flash loader demonstrator to download the NuttX binary image.

7. Wait until programming is completed and press "Finish". Toggle the
   BOOT0 switch again. Reset the board.

You will now be presented with the NuttShell (NSH). Enjoy.

Configurations
==============

Information Common to All Configurations
----------------------------------------

The ET-STM32 Stamp configuration is maintained in a sub-directory and can be
selected as follow::

    tools/configure.sh et-stm32-stamp:<subdir>

Before building, make sure the PATH environment variable includes the
correct path to the directory than holds your toolchain binaries.

And then build NuttX by simply typing the following.  At the conclusion of
the make, the nuttx binary will reside in an ELF file called, simply, nuttx.::

    make

The <subdir> that is provided above as an argument to the tools/configure.sh
must be in one of the following.

NOTES:

1. These configurations use the mconf-based configuration tool.  To
   change any of these configurations using that tool, you should:

   a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
      see additional README.txt files in the NuttX tools repository.

   b. Execute 'make menuconfig' in nuttx/ in order to start the
      reconfiguration process.

Configuration Sub-directories
-----------------------------

nsh:
----

This configuration directory provide the basic NuttShell (NSH).
A serial console is provided on USART1.
