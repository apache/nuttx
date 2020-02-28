README
======

  This README discusses issues unique to NuttX configurations for the
  STMicro NUCLEO-H743ZI development board featuring the STM32H743ZI
  MCU. The STM32H743ZI is a 400MHz Cortex-M7 operation with 2MBytes Flash
  memory and 1MByte SRAM. The board features:

  - On-board ST-LINK/V2-1 for programming and debugging,
  - Mbed-enabled (mbed.org)
  - 3 user LEDs
  - Two pushbuttons (user and reset)
  - 32.768 kHz crystal oscillator
  - USB OTG FS with Micro-AB connectors
  - Ethernet connector compliant with IEEE-802.3-2002
  - Board connectors:
    - USB with Micro-AB
    - SWD
    - Ethernet RJ45
    - ST Zio connector including Arduino Uno V3
    - ST morpho

  Refer to the http://www.st.com website for further information about this
  board (search keyword: NUCLEO-H743ZI)

Contents
========

  - STATUS
  - Serial Console
  - Configurations

STATUS
======

  2018-06-16:  This port is still incomplete!  It is lacking the following
    at a minimum:

    - The RCC register definitions must still be updated per the STM32H7x3
      user manual.
    - The RCC-based clock configuration logic must be redesigned to support
      the STM32H7's RCC.
    - There is no U[S]ART support in arch/arm/src/stm32h7.  For the most
      part the U[S]ART is compatible with the STM32 F7, however, updates
      will be required to support:  (1) the new STM32 H7 clock prescaler,
      and (2) the new STM32 H7 Rx and Tx FIFOs.
    - There is a fragmentary NSH here.  However, it does not have any serial
      console selected and is not expected to be usable.

    This logic has set on a branch for some time with little progress.  I
    have decided to bring it into the master but only with the option to
    select the STM32 H7 architecture dependent on CONFIG_EXPERMIMENTAL.
    That should eliminate the possibility of anyone accidentally enabling
    this know incomplete port.

  2018-07-12:  After several commits focused primarily on the H7 RCC and
    serial driver, Mateusz Szafoni states the basic NSH configuration is
    now functional.
  2018-09-16:  I2C Driver added by Mateusz Szafoni.
  2018-10-02:  SPI Driver added by Mateusz Szafoni.
  2019-03-02:  DMA support contributed by Mateusz Szafoni.

Serial Console
==============

  Many options are available for a serial console via the Morpho connector.
  Here two common serial console options are suggested:

  1. Arduino Serial Shield.

    If you are using a standard Arduino RS-232 shield with the serial
    interface with RX on pin D0 and TX on pin D1 from USART6:

      -------- ---------------
               STM32H7
      ARDUINO  FUNCTION  GPIO
      -- ----- --------- -----
      DO RX    USART6_RX PG9
      D1 TX    USART6_TX PG14
      -- ----- --------- -----

  2. Nucleo Virtual Console.

    The virtual console uses Serial Port 3 (USART3) with TX on PD8 and RX on
    PD9.

      ----------------- ---
      VCOM Signal       Pin
      ----------------- ---
      SERIAL_RX         PD9
      SERIAL_TX         PD8
      ----------------- ---

    These signals are internally connected to the on board ST-Link.

  The Nucleo virtual console is the default serial console in all
  configurations unless otherwise stated in the description of the
  configuration.

Configurations
==============

  Information Common to All Configurations
  ----------------------------------------
  Each Nucleo-H743ZI configuration is maintained in a sub-directory and
  can be selected as follow:

    tools/configure.sh [options] viewtool-stm32f107:<subdir>

  Where options should specify the host build platform (-l for Linux, -c for
  Cygwin under Windows, etc.).  Try 'tools/configure.sh -h' for the complete
  list of options.

  Before starting the build, make sure that (1) your PATH environment variable
  includes the correct path to your toolchain, and (2) you have the correct
  toolchain selected in the configuration.

  And then build NuttX by simply typing the following.  At the conclusion of
  the make, the nuttx binary will reside in an ELF file called, simply, nuttx.

    make

  The <subdir> that is provided above as an argument to the tools/configure.sh
  must be is one of the following.

  NOTES:

  1. These configurations use the mconf-based configuration tool.  To
     change any of these configurations using that tool, you should:

     a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
        see additional README.txt files in the NuttX tools repository.

     b. Execute 'make menuconfig' in nuttx/ in order to start the
        reconfiguration process.

  2. Unless stated otherwise, all configurations generate console
     output on the ST-Link VCOM, USART3.

  3. Unless otherwise stated, the configurations are setup for Linux by
     default:

     Build Setup:
       CONFIG_HOST_LINUX=y                     : Linux host operating system

  4. All of these configurations use the general arm-none-eabi toolchain for
     Linux  That toolchain selection can easily be reconfigured using 'make
     menuconfig'.

  5. These configurations all assume that you are loading code using
     something like the ST-Link v2 JTAG.  None of these configurations are
     setup to use the DFU bootloader but should be easily reconfigured to
     use that bootloader if so desired.

  Configuration Sub-directories
  -----------------------------

  nsh:

    This configuration provides a basic NuttShell configuration (NSH)
    for the Nucleo-H743ZI.  The default console is the VCOM on USART3.
