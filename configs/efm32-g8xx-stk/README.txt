README
======

  This README file discusses the port of NuttX to the Silicon Labs EFM32™ Gecko Starter Kit (EFM32-G8XX-STK). The Gecko Starter Kit features:

    • EFM32G890F128 MCU with 128 kB flash and 16 kB RAM
    •   32.768 kHz crystal
    •   32 MHz crystal
    • Advanced Energy Monitoring
    • Touch slider
    • 4x40 LCD
    • 4 User LEDs
    • 2 pushbutton switches
    • Reset button and a switch to disconnect the battery.
    • On-board SEGGER J-Link USB emulator
    •   ARM 20 pin JTAG/SWD standard Debug in/out connector

LEDs
====

  The EFM32 Gecko Start Kit has four yellow LEDs.  These LEDs are connected
  as follows:

  ------------------------------------- --------------------
  EFM32 PIN                             BOARD SIGNALS
  ------------------------------------- --------------------
  C0/USART1_TX#0/PCNT0_S0IN#2/ACMP0_CH0  MCU_PC0  UIF_LED0
  C1/USART1_RX#0/PCNT0_S1IN#2/ACMP0_CH1  MCU_PC1  UIF_LED1
  C2/USART2_TX#0/ACMP0_CH2               MCU_PC2  UIF_LED2
  C3/USART2_RX#0/ACMP0_CH3               MCU_PC3  UIF_LED3
  ------------------------------------- --------------------

  All LEDs are grounded and so are illuminated by outputting a high
  value to the LED.

  These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
  defined.  In that case, the usage by the board port is defined in
  include/board.h and src/efm32_autoleds.c.  The LEDs are used to
  encode OS-related events as follows:

    SYMBOL             Meaning                 LED1*  LED2   LED3   LED4
    ----------------- -----------------------  ------ -----  -----  ------
    LED_STARTED       NuttX has been started   ON     OFF    OFF    OFF
    LED_HEAPALLOCATE  Heap has been allocated  OFF    ON     OFF    OFF
    LED_IRQSENABLED   Interrupts enabled       ON     ON     OFF    OFF
    LED_STACKCREATED  Idle stack created       OFF    OFF    ON     OFF
    LED_INIRQ         In an interrupt**        ON     N/C    N/C    OFF
    LED_SIGNAL        In a signal handler***   N/C    ON     N/C    OFF
    LED_ASSERTION     An assertion failed      ON     ON     N/C    OFF
    LED_PANIC         The system has crashed   N/C    N/C    N/C    ON
    LED_IDLE          STM32 is is sleep mode   (Optional, not used)

  * If LED1, LED2, LED3 are statically on, then NuttX probably failed to boot
    and these LEDs will give you some indication of where the failure was
 ** The normal state is LED3 ON and LED1 faintly glowing.  This faint glow
    is because of timer interrupt that result in the LED being illuminated
    on a small proportion of the time.
*** LED2 may also flicker normally if signals are processed.

Configurations
==============
  Each EFM32 Gecko Starter Kit configuration is maintained in a sub-director
  and can be selected as follow:

    cd tools
    ./configure.sh efm32-g8xx-stk/<subdir>
    cd -
    . ./setenv.sh

  If this is a Windows native build, then configure.bat should be used
  instead of configure.sh:

    configure.bat STM32F4Discovery\<subdir>

  Where <subdir> is one of the following:

  nsh:
  ---
    Configures the NuttShell (nsh) located at apps/examples/nsh.  The
    Configuration enables the serial interfaces on USARTx.  Support for
    builtin applications is enabled, but in the base configuration no
    builtin applications are selected (see NOTES below).

    NOTES:

    1. This configuration uses the mconf-based configuration tool.  To
       change this configuration using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
          and misc/tools/

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. By default, this configuration uses the CodeSourcery toolchain
       for Windows and builds under Cygwin (or probably MSYS).  That
       can easily be reconfigured, of course.

       CONFIG_HOST_WINDOWS=y                   : Builds under Windows
       CONFIG_WINDOWS_CYGWIN=y                 : Using Cygwin
       CONFIG_ARMV7M_TOOLCHAIN_CODESOURCERYW=y : CodeSourcery for Windows
