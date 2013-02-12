README.txt
==========

This README file discusses the port of NuttX to the WaveShare Open1788 board:
See http://wvshare.com/product/Open1788-Standard.htm. This board features the
NXP LPC1788 MCU

CONTENTS
========

  o LEDs
  o Buttons
  o FPU
  o Configuration

LEDs
====

The Open1788 base board has four user LEDs

  LED1 : Connected to P1[14]
  LED2 : Connected to P0[16]
  LED3 : Connected to P1[13]
  LED4 : Connected to P4[27]

If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in
any way using the defitions provided in the board.h header file.

If CONFIG_ARCH_LEDs is defined, then NuttX will control the 3 LEDs on the
WaveShare Open1788K.  The following definitions describe how NuttX controls
the LEDs:
                             LED1 LED2 LED3 LED4
  LED_STARTED                OFF  OFF  OFF  OFF
  LED_HEAPALLOCATE           ON   OFF  OFF  OFF
  LED_IRQSENABLED            OFF   ON  OFF  OFF
  LED_STACKCREATED           ON    ON  OFF  OFF
  LED_INIRQ                  LED3 glows, on while in interupt
  LED_SIGNAL                 LED3 glows, on while in signal handler
  LED_ASSERTION              LED3 glows, on while in assertion
  LED_PANIC                  LED3 Flashes at 2Hz
  LED_IDLE                   LED glows, ON while sleeping

Buttons
=======

The Open1788K supports several buttons:

  USER1           : Connected to P4[26]
  USER2           : Connected to P2[22]
  USER3           : Connected to P0[10]

And a Joystick

  JOY_A           : Connected to P2[25]
  JOY_B           : Connected to P2[26]
  JOY_C           : Connected to P2[23]
  JOY_D           : Connected to P2[19]
  JOY_CTR         : Connected to P0[14]

These can be accessed using the definitions and interfaces defined in the
board.h header file.

FPU
===

FPU Configuration Options
-------------------------

There are two version of the FPU support built into the LPC17xx port.

1. Lazy Floating Point Register Save.

   This is an untested implementation that saves and restores FPU registers
   only on context switches.  This means: (1) floating point registers are
   not stored on each context switch and, hence, possibly better interrupt
   performance.  But, (2) since floating point registers are not saved,
   you cannot use floating point operations within interrupt handlers.

   This logic can be enabled by simply adding the following to your .config
   file:

   CONFIG_ARCH_FPU=y

2. Non-Lazy Floating Point Register Save

   Mike Smith has contributed an extensive re-write of the ARMv7-M exception
   handling logic. This includes verified support for the FPU.  These changes
   have not yet been incorporated into the mainline and are still considered
   experimental.  These FPU logic can be enabled with:

   CONFIG_ARCH_FPU=y
   CONFIG_ARMV7M_CMNVECTOR=y

   You will probably also changes to the ld.script in if this option is selected.
   This should work:

   -ENTRY(_stext)
   +ENTRY(__start)         /* Treat __start as the anchor for dead code stripping */
   +EXTERN(_vectors)       /* Force the vectors to be included in the output */

CFLAGS
------

Only the Atollic toolchain has built-in support for the Cortex-M4 FPU.  You will see
the following lines in each Make.defs file:

  ifeq ($(CONFIG_STM32_ATOLLIC_LITE),y)
    # Atollic toolchain under Windows
    ...
  ifeq ($(CONFIG_ARCH_FPU),y)
    ARCHCPUFLAGS = -mcpu=cortex-m4 -mthumb -march=armv7e-m -mfpu=fpv4-sp-d16 -mfloat-abi=hard
  else
    ARCHCPUFLAGS = -mcpu=cortex-m3 -mthumb -mfloat-abi=soft
  endif
  endif

If you are using a toolchain other than the Atollic toolchain, then to use the FPU
you will also have to modify the CFLAGS to enable compiler support for the ARMv7-M
FPU.  As of this writing, there are not many GCC toolchains that will support the
ARMv7-M FPU.  

As a minimum you will need to add CFLAG options to (1) enable hardware floating point
code generation, and to (2) select the FPU implementation.  You might try the same
options as used with the Atollic toolchain in the Make.defs file:

  ARCHCPUFLAGS = -mcpu=cortex-m4 -mthumb -march=armv7e-m -mfpu=fpv4-sp-d16 -mfloat-abi=hard

Configuration Changes
---------------------

Below are all of the configuration changes that I had to make to configs/stm3240g-eval/nsh2
in order to successfully build NuttX using the Atollic toolchain WITH FPU support:

  -CONFIG_ARCH_FPU=n              : Enable FPU support
  +CONFIG_ARCH_FPU=y

  -CONFIG_STM32_CODESOURCERYW=y   : Disable the CodeSourcery toolchain
  +CONFIG_STM32_CODESOURCERYW=n

  -CONFIG_STM32_ATOLLIC_LITE=n   : Enable *one* the Atollic toolchains
   CONFIG_STM32_ATOLLIC_PRO=n
  -CONFIG_STM32_ATOLLIC_LITE=y   : The "Lite" version
   CONFIG_STM32_ATOLLIC_PRO=n    : The "Pro" version

  -CONFIG_INTELHEX_BINARY=y       : Suppress generation FLASH download formats
  +CONFIG_INTELHEX_BINARY=n       : (Only necessary with the "Lite" version)

  -CONFIG_HAVE_CXX=y              : Suppress generation of C++ code
  +CONFIG_HAVE_CXX=n              : (Only necessary with the "Lite" version)

See the section above on Toolchains, NOTE 2, for explanations for some of
the configuration settings.  Some of the usual settings are just not supported
by the "Lite" version of the Atollic toolchain.

CONFIGURURATION
===============

  ostest
  ------ 
    This configuration directory, performs a simple OS test using
    apps/examples/ostest.

    NOTES:
 
    1. This configuration uses the mconf-based configuration tool.  To
       change this configuration using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
          and misc/tools/

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. Uses the older, OABI, buildroot toolchain.  But that is easily
       reconfigured:

       CONFIG_ARMV7M_TOOLCHAIN_BUILDROOT=y : Buildroot toolchain
       CONFIG_ARMV7M_OABI_TOOLCHAIN=y      : Older, OABI toolchain


