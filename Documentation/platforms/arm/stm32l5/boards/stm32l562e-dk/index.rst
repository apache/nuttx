================
ST STM32L562E-DK
================

This page discusses the port of NuttX to the STMicro STM32L562E-DK
board.  That board features the STM32L562QEI6QU MCU with 512KiB of FLASH
and 256KiB of SRAM.

This port is a proof-of-concept to demonstrate running NuttX in the Non-
Secure TrustZone domain as a companion to TrustedFirmware-M (TFM).  Running
NuttX on the STM32L562E-DK without TFM is currently not supported.

Status
======
2021-03-03: The board now boots and the basic NSH configuration works
    without problem.

LEDs
====

The Board provides a 2 user LEDs, LD9 and LD10
LED9   (Red)    PD_3
LED10  (Green)  PG_12

- When the I/O is LOW value, the LEDs are on.
- When the I/O is HIGH value, the LEDs are off.

These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
defined.  In that case, the usage by the board port is defined in
include/board.h and src/stm32_autoleds.c. The LEDs are used to encode OS
related events as follows when the LEDs are available:

  ===================  =======================   ====  ===== ============
  SYMBOL               Meaning                   RED   GREEN Note
  ===================  =======================   ====  ===== ============
  LED_STARTED          NuttX has been started    OFF   OFF  
  LED_HEAPALLOCATE     Heap has been allocated   ON    OFF
  LED_IRQSENABLED      Interrupts enabled        ON    ON
  LED_STACKCREATED     Idle stack created        OFF   ON
  LED_INIRQ            In an interrupt           GLOW  NC    momentary
  LED_SIGNAL           In a signal handler       GLOW  NC    momentary
  LED_ASSERTION        An assertion failed       GLOW  NC    momentary
  LED_PANIC            The system has crashed    ON    OFF   flashing 2Hz
  LED_IDLE             MCU is is sleep mode      NC    ON
  ===================  =======================   ====  ===== ============

  OFF -  means that the OS is still initializing. Initialization is very fast
         so if you see this at all, it probably means that the system is
         hanging up somewhere in the initialization phases.

  GREEN -  This means that the OS completed initialization.

  Flashing RED -  In the event of a fatal crash, all other LEDs will be
                  extinguished and RED LED will FLASH at a 2Hz rate.

Thus if the GREEN LED is lit, NuttX has successfully booted and is,
apparently, idleing normally.  If the RED LED is flashing at
approximately 2Hz, then a fatal error has been detected and the system has
halted.

Buttons
=======

B1 USER: the user button is connected to the I/O PC13 (pin 2) of the STM32
microcontroller.

Serial Consoles
===============

USART1 - Virtual COM Port
-------------------------

Default board is configured to use USART1 as console, which is wired to
the STLINK Virtual COM Port.

    ==== ====
    FUNC GPIO
    ==== ====
    TXD: PA9
    RXD: PA10
    ==== ====

Use make menuconfig to configure USART1 as the console::

    CONFIG_STM32L5_USART1=y
    CONFIG_USART1_SERIALDRIVER=y
    CONFIG_USART1_SERIAL_CONSOLE=y
    CONFIG_USART1_RXBUFSIZE=256
    CONFIG_USART1_TXBUFSIZE=256
    CONFIG_USART1_BAUD=115200
    CONFIG_USART1_BITS=8
    CONFIG_USART1_PARITY=0
    CONFIG_USART1_2STOP=0

Default
-------

As shipped, the virtual COM port is enabled.

TrustedFirmware-M
=================

You should study [UM2671] STMicroelectronics. UM2671: Getting started with
STM32CubeL5 TFM application, 3rd edition, June 2021.

I was using STM32CubeL5 v1.5.0
  (https://github.com/STMicroelectronics/STM32CubeL5/tree/v1.5.0).

Changes required to STM32CubeL5's TFM
-------------------------------------

The following three changes to TFM have to be applied to be able to run NuttX.

The first one is required since NuttX issues SVC instructions while interrupts
are disabled, which causes HardFaults.  NuttX then detects this situation in
the HardFault handler.  Per default HardFaults are taken to the secure domain,
though.  Thus, this change::

  --- a/Projects/STM32L562E-DK/Applications/TFM/TFM_SBSFU_Boot/Src/boot_hal.c
  +++ b/Projects/STM32L562E-DK/Applications/TFM/TFM_SBSFU_Boot/Src/boot_hal.c
  @@ -306,6 +306,9 @@ void jumper(struct arm_vector_table *vector)
     /* set the secure vector */
     SCB->VTOR = (uint32_t)vector;

  +  /* Stay in Non-Secure mode for BusFault, HardFault, and NMI exceptions */
  +  SCB->AIRCR = (SCB->AIRCR & 0x0000FFFF) | 0x05FA0000 | SCB_AIRCR_BFHFNMINS_Msk;
  +
     vt = (struct arm_vector_table *)vector;

  The second change is required to ensure all interrupts are taken to the
  non-secure domain:

  --- a/Projects/STM32L562E-DK/Applications/TFM/TFM_Appli/Secure/Src/spm_hal.c
  +++ b/Projects/STM32L562E-DK/Applications/TFM/TFM_Appli/Secure/Src/spm_hal.c
  @@ -535,6 +535,7 @@ enum tfm_plat_err_t tfm_spm_hal_system_reset_cfg(void)
   enum tfm_plat_err_t tfm_spm_hal_nvic_interrupt_target_state_cfg(void)
   {
     /*    return nvic_interrupt_target_state_cfg();*/
  +  nvic_interrupt_target_state_cfg();
     return TFM_PLAT_ERR_SUCCESS;
   }

The third chage is required, since current NuttX does not support lazy FPU
register stacking any longer.  Thus, this must be disabled for the TF-M secure
code as well::

  --- a/Projects/STM32L562E-DK/Applications/TFM/TFM_Appli/Secure/Src/target_cfg.c
  +++ b/Projects/STM32L562E-DK/Applications/TFM/TFM_Appli/Secure/Src/target_cfg.c
  @@ -134,7 +134,7 @@ void sau_and_idau_cfg(void)
     SCB->NSACR = (SCB->NSACR & ~(SCB_NSACR_CP10_Msk | SCB_NSACR_CP11_Msk)) |
                  ((SCB_NSACR_CP10_11_VAL << SCB_NSACR_CP10_Pos) & (SCB_NSACR_CP10_Msk | SCB_NSACR_CP11_Msk));

  -  FPU->FPCCR = (FPU->FPCCR & ~(FPU_FPCCR_TS_Msk | FPU_FPCCR_CLRONRETS_Msk | FPU_FPCCR_CLRONRET_Msk)) |
  +  FPU->FPCCR = (FPU->FPCCR & ~(FPU_FPCCR_TS_Msk | FPU_FPCCR_CLRONRETS_Msk | FPU_FPCCR_CLRONRET_Msk | FPU_FPCCR_LSPEN_Msk)) |
                  ((FPU_FPCCR_TS_VAL        << FPU_FPCCR_TS_Pos) & FPU_FPCCR_TS_Msk) |
                  ((FPU_FPCCR_CLRONRETS_VAL << FPU_FPCCR_CLRONRETS_Pos) & FPU_FPCCR_CLRONRETS_Msk) |
                  ((FPU_FPCCR_CLRONRET_VAL  << FPU_FPCCR_CLRONRET_Pos) & FPU_FPCCR_CLRONRET_Msk);

Encrypting and Signing the NuttX Binary
---------------------------------------

According to Figure 24 of [UM2671] and since we use the 'external flash
configuration', we must create the 'tfm_ns_enc_sign.bin' image from
'nuttx.bin'.  This is done with mcuboot's imgtool.py.::

  imgtool.py sign               \
        -k root-rsa-2048_1.pem  \  Key used for signing
        -E enc-rsa2048-pub.pem  \  Encrypt image using the provided public key
        -o 0x90000400           \  OTFDEC
        -e little               \  Little Endian
        -S 0x100000             \  Size of Flash Slot
        -H 0x400                \  Header Size
        --pad-header            \  Pad header with zeroes
        -v $version             \  Version (TEXT)
        -s auto                 \  Security Counter - auto: Create from Version
        --align 8               \  8 byte alignment
        nuttx.bin               \  input file
        tfm_ns_enc_sign.bin        output file

On my system the full command is as follows::

  $ python3 ~/compile/STM32CubeL5/Middlewares/Third_Party/mcuboot/scripts/imgtool.py sign -k ~/compile/STM32CubeL5/Middlewares/Third_Party/trustedfirmware/bl2/ext/mcuboot/root-rsa-2048_1.pem -E ~/compile/STM32CubeL5/Middlewares/Third_Party/trustedfirmware/bl2/ext/mcuboot/enc-rsa2048-pub.pem -o 0x90000400 -e little -S 0x100000 -H 0x400 --pad-header -v 1 -s auto --align 8 nuttx.bin tfm_ns_enc_sign.bin

Configurations
==============

Information Common to All Configurations
----------------------------------------
Each configuration is maintained in a sub-directory and can be
selected as follow::

    tools/configure.sh nucleo-l552ze:<subdir>

Before building, make sure the PATH environment variable includes the
correct path to the directory than holds your toolchain binaries.

And then build NuttX by simply typing the following.  At the conclusion of
the make, the nuttx binary will reside in an ELF file called, simply, nuttx::

    make oldconfig
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
   output on USART1, as described above under "Serial Console".  The
   elevant configuration settings are listed below::

         CONFIG_STM32L5_USART1=y
         CONFIG_STM32L5_USART1_SERIALDRIVER=y
         CONFIG_STM32L5_USART=y

         CONFIG_USART1_SERIALDRIVER=y
         CONFIG_USART1_SERIAL_CONSOLE=y

         CONFIG_USART1_RXBUFSIZE=256
         CONFIG_USART1_TXBUFSIZE=256
         CONFIG_USART1_BAUD=115200
         CONFIG_USART1_BITS=8
         CONFIG_USART1_PARITY=0
         CONFIG_USART1_2STOP=0

3. All of these configurations are set up to build under Linux using the
   "GNU Tools for ARM Embedded Processors" that is maintained by ARM
   (unless stated otherwise in the description of the configuration).

       https://developer.arm.com/open-source/gnu-toolchain/gnu-rm

   That toolchain selection can easily be reconfigured using
   'make menuconfig'.  Here are the relevant current settings:

Build Setup::
    CONFIG_HOST_LINUX=y                 : Linux environment

System Type -> Toolchain::
    CONFIG_ARM_TOOLCHAIN_GNU_EABI=y : GNU ARM EABI toolchain

Configuration sub-directories
=============================

nsh:
----

Configures the NuttShell (nsh) located at examples/nsh.  This
configuration is focused on low level, command-line driver testing.
