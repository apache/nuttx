README
======

  This is the README file for the port of NuttX to the Arduino Mega 2560 Rev3.

    https://www.arduino.cc/en/Main/ArduinoBoardMega2560

  The board is based on ATMega2560 chip from Atmel

  http://www.atmel.com/devices/atmega2560.aspx

Contents
========

  o Toolchain
  o Serial Console
  o Configurations

Toolchain
=========

  Right now only Atmel's AVR8 Toolchain is supported. You can get it from

    http://www.atmel.com/tools/atmelavrtoolchainforwindows.aspx

  It is basically WinAVR compatible so sub-projects may define WinAVR as a
  tool-chain but specify path to the Atmel AVR8 in path. See
  arduino-mega2560/hello for example.

Serial Console
==============

  The serial console is available on USART0.  You will need to connect an
  RS-232 shield or an external RS-232 driver as follows:

    TXD: TX0->1
    RXD: RX0->0
    GND: Power GND
    +5V: Power +5V

  You will then need to use a terminal program configured at 38400 8N1.

Configurations
==============

  Common Configuration Notes
  --------------------------
  1. Each Arduino MEGA2560 configuration is maintained in a sub-directory
     and can be selected as follow:

       tools/configure.sh arduino-mega2560:<subdir>

     Where <subdir> is one of the configuration sub-directories described in
     the following paragraph.

  2. These configurations use the mconf-based configuration tool.  To
     change a configurations using that tool, you should:

     a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
        see additional README.txt files in the NuttX tools repository.

     b. Execute 'make menuconfig' in nuttx/ in order to start the
        reconfiguration process.

  3. By default, all configurations assume the Atmel Studio AVR8 toolchain
     under Cygwin with Windows.  This is easily reconfigured:

        CONFIG_HOST_WINDOWS=y
        CONFIG_WINDOWS_CYGWIN=y
        CONFIG_AVR_BUILDROOT_TOOLCHAIN=y

  Configuration Sub-Directories
  -----------------------------
  hello:
    The simple apps/examples/hello "Hello, World!" example.

  nsh:
    This is a reduce NuttShell (NSH) configuration using apps/example/nsh.
    The serial console is provided on USART0 and can be accessed via
    an external RS-232 driver as described above under "Serial Console".

    NOTES:

    1. As of 2015-01-16, the default static memory usage is:

         $ size nuttx.elf
          text    data     bss     dec     hex filename
         34348    2117     944   37409    9221 nuttx.elf

       And dynamic, heap usage:

         nsh> free
                      total       used       free    largest
         Mem:          4736       1768       2968       2968

       There is plenty of FLASH space but not a lot of free SRAM. This
       SRAM usage is due primarily to constant strings defined by NSH.

       NOTE: I investigated the possibility of adding IPTR/IOBJ
       qualifiers in the NSH library and moving the strings to FLASH.
       This is still a possibility but would requires some extensive
       changes to NSH.
