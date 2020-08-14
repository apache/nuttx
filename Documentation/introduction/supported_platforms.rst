.. include:: /substitutions.rst

===================
Supported Platforms
===================

**Supported Platforms by CPU core**. The number of ports to this
CPU follow in parentheses. The state of the various ports vary
from board-to-board. Follow the links for the details:

.. list-table::
   :widths: auto

   * -

       - `Linux/Cygwin user mode simulation <#linuxusermode>`__ (1)
       - ARM

         -  `ARM7TDMI <#arm7tdmi>`__ (4)
         -  `ARM920T <#arm920t>`__ (1)
         -  `ARM926EJS <#arm926ejs>`__ (4)
         -  `Other ARMv4 <#armv4>`__ (1)
         -  `ARM1176JZ <#arm1176jz>`__ (1)
         -  `ARM Cortex-A5 <#armcortexa5>`__ (3)
         -  `ARM Cortex-A8 <#armcortexa8>`__ (2)
         -  `ARM Cortex-A9 <#armcortexa9>`__ (1)
         -  `ARM Cortex-R4 <#armcortexr4>`__ (2)
         -  `ARM Cortex-M0/M0+ <#armcortexm0>`__ (13)
         -  `ARM Cortex-M3 <#armcortexm3>`__ (39)
         -  `ARM Cortex-M4 <#armcortexm4>`__ (59)
         -  `ARM Cortex-M7 <#armcortexm7>`__ (15)

       - Atmel AVR

         -  `Atmel 8-bit AVR <#atmelavr>`__ (5)
         -  `Atmel AVR32 <#atmelavr32>`__ (1)

       - Freescale

         -  `M68HCS12 <#m68hcs12>`__ (2)
     -

       - Intel

         -  `Intel 80x86 <#80x86>`__ (2)

       - Microchip

         -  `PIC32MX <#pic32mxmips>`__ (MIPS M4K) (4)
         -  `PIC32MZEC <#pic32mzmips>`__ (MIPS microAptive) (1)
         -  `PIC32MZEF <#pic32mzmips>`__ (MIPS M5150) (1)

       - Misoc

         -  `LM32 <#misoclm32>`__ (1)
         -  `Minerva <#minerva>`__ (1)

       - OpenRISC

         -  `mor1kx <#mor1kx>`__ (1)

       - Renesas/Hitachi:

         -  `Renesas/Hitachi SuperH <#superh>`__ (1/2)
         -  `Renesas M16C/26 <#m16c>`__ (1/2)
         -  `Renesas RX65N <#rx65n>`__ (2)
     -
       - `RISC-V <#riscv>`__ (2)

         -  `NEXT RISC-V NR5Mxx <#nr5mxx>`__ (1)
         -  `GreenWaves GAP8 (1) <#gwgap8>`__
         -  `Kendryte K210 (1) <#k210>`__
         -  `Litex (1) <#artya7>`__

       - Xtensa LX6:

         -  `ESP32 <#esp32>`__ (1)

       - ZiLOG

         -  `ZiLOG ZNEO Z16F <#zilogz16f>`__ (2)
         -  `ZiLOG eZ80 Acclaim! <#zilogez80acclaim>`__ (4)
         -  `ZiLOG Z8Encore! <#zilogz8encore>`__ (2)
         -  `ZiLOG Z180 <#zilogz180>`__ (1)
         -  `ZiLOG Z80 <#zilogz80>`__ (2)

**Supported Platforms by Manufacturer/MCU Family**. CPU core type
follows in parentheses. The state of the various ports vary from MCU to
MCU. Follow the links for the details:

.. list-table::

   * -

       - `Linux/Cygwin user mode simulation <#linuxusermode>`__ (1) ARM
       - Allwinner

         -  `A10 <#allwinnera10>`__ (Cortex-A8)

       - Broadcom

         -  `BCM2708 <#bcm2708>`__ (ARM1176JZ)

       - Espressif

         -  `ESP32 <#esp32>`__ (Dual Xtensa LX6)

       - GreenWaves

         -  `GAP8 <#gwgap8>`__ (RISC-V RV32IM)

       - Host PC based simulations

         -  `Linux/Cygwin user mode simulation <#linuxusermode>`__

       - Infineon

         -  `Infineon XMC45xx <#xmd45xx>`__

       - Intel

         -  `Intel 80x86 <#80x86>`__

       - Maxim Integrated

         -  `MAX32660 <#max3660>`__ (ARM Cortex-M3)

       - Microchip

         -  `PIC32MX2xx Family <#pic32mx2xx>`__ (MIPS32 M4K)
         -  `PIC32MX4xx Family <#pic32mx4xx>`__ (MIPS32 M4K)
         -  `PIC32MX7xx Family <#pic32mx7xx>`__ (MIPS32 M4K)
         -  `PIC32MZEC Family <#pic32mzec>`__ (MIPS32 |br|
            microAptiv)
         -  `PIC32MZEF Family <#pic32mzef>`__ (MIPS32 M5150)

       - Microchip (Formerly Atmel)

         -  `AVR ATMega128 <#avratmega128>`__ (8-bit AVR)
         -  `AVR ATMega1284p <#avratmega1284p>`__ (8-bit AVR)
         -  `AVR ATMega2560 <#avratmega2560>`__ (8-bit AVR)
         -  `AVR AT90USB64x and AT90USB6128x <#avrat90usbxxx>`__ |br|
            (8-bit AVR)
         -  `AVR32 AT32UC3BXXX <#at32uc3bxxx>`__ (32-bit AVR32)
         -  `Atmel SAMD20 <#at91samd20>`__ (ARM Cortex-M0+)
         -  `Atmel SAMD21 <#at91samd21>`__ (ARM Cortex-M0+)
         -  `Atmel SAML21 <#at91saml21>`__ (ARM Cortex-M0+)
         -  `Atmel SAM3U <#at91sam3u>`__ (ARM Cortex-M3)
         -  `Atmel SAM3X <#at91sam3x>`__ (ARM Cortex-M3)
         -  `Atmel SAM4C <#at91sam4c>`__ (ARM Cortex-M4)
         -  `Atmel SAM4E <#at91sam4e>`__ (ARM Cortex-M4)
         -  `Atmel SAM4L <#at91sam4l>`__ (ARM Cortex-M4)
         -  `Atmel SAM4S <#at91sam4s>`__ (ARM Cortex-M4)
         -  `Atmel SAMD5x/E5x <#at91samd5e5>`__ (ARM Cortex-M4)
         -  `Atmel SAME70 <#at91same70>`__ (ARM Cortex-M7)
         -  `Atmel SAMV71 <#at91samv71>`__ (ARM Cortex-M7)
         -  `Atmel SAMA5D2 <#at91sama5d2>`__ (ARM Cortex-A5)
         -  `Atmel SAMA5D3 <#at91sama5d3>`__ (ARM Cortex-A5)
         -  `Atmel SAMA5D4 <#at91sama5d4>`__ (ARM Cortex-A5)

       - Moxa

         -  `Moxa NP51x0 <#moxart>`__ (ARMv4)

       - nuvoTon

         -  `nuvoTon NUC120 <#nuvotonnu120>`__ (ARM Cortex-M0)

       - Nordic Semiconductor

         -  `NRF52xxx <#nrf52>`__ (ARM Cortex-M4)

       - NXP/Freescale

         -  `M68HCS12 <#m68hcs12>`__
         -  `NXP/Freescale i.MX1 <#freescaleimx1>`__ |br|
            (ARM920-T)
         -  `NXP/Freescale i.MX6 <#freescaleimx6>`__ |br|
            (ARM Cortex-A9)
         -  `NXP/Freescale i.MX RT <#freescaleimxrt>`__ |br|
            (ARM Cortex-M7)
         -  `NXP/FreeScale KL25Z <#freescalekl25z>`__ |br|
            (ARM Cortex-M0+)
         -  `NXP/FreeScale KL26Z <#freescalekl26z>`__ |br|
            (ARM Cortex-M0+)

         |br|
         |br|
         |br|
         |br|
         |br|
         |br|
         |br|
         |br|
         |br|
         |br|
         |br|
         |br|
         |br|
         |br|
         |br|
         |br|
         |br|
         |br|
         |br|
         |br|
     -

       - NXP/Freescale (Continued)

         -  `NXP/FreeScale Kinetis K20 <#kinetisk20>`__ (ARM |br|
            Cortex-M4)
         -  `NXP/FreeScale Kinetis K28 <#kinetisk28>`__ (ARM |br|
            Cortex-M4)
         -  `NXP/FreeScale Kinetis K40 <#kinetisk40>`__ (ARM |br|
            Cortex-M4)
         -  `NXP/FreeScale Kinetis K60 <#kinetisk60>`__ (ARM |br|
            Cortex-M4)
         -  `NXP/FreeScale Kinetis K64 <#kinetisk64>`__ (ARM |br|
            Cortex-M4)
         -  `NXP/FreeScale Kinetis K66 <#kinetisk66>`__ (ARM |br|
            Cortex-M4)

         -  `NXP LPC11xx <#nxplpc11xx>`__ (Cortex-M0)
         -  `NXP LPC214x <#nxplpc214x>`__ (ARM7TDMI)
         -  `NXP LPC2378 <#nxplpc2378>`__ (ARM7TDMI)
         -  `NXP LPC3131 <#nxplpc3131>`__ (ARM9E6JS)
         -  `NXP LPC315x <#nxplpc315x>`__ (ARM9E6JS)
         -  `NXP LPC176x <#nxplpc176x>`__ (ARM Cortex-M3)
         -  `NXP LPC178x <#nxplpc178x>`__ (ARM Cortex-M3)
         -  `NXP LPC40xx <#nxplpc40xx>`__ (ARM Cortex-M4)
         -  `NXP LPC43xx <#nxplpc43xx>`__ (ARM Cortex-M4)
         -  `NXP LPC54xx <#nxplpc54xx>`__ (ARM Cortex-M4)

         -  `NXP S32K11x <#nxps32k11x>`__ (Cortex-M0+)
         -  `NXP S32K14x <#nxps32k14x>`__ (Cortex-M4F)

       - ON Semiconductor:

         -  `LC823450 <#lc823450>`__ (Dual core ARM Cortex-M3)

       - Renesas/Hitachi:

         -  `Renesas/Hitachi SuperH <#superh>`__
         -  `Renesas M16C/26 <#m16c>`__
         -  `Renesas RX65N <#rx65n>`__

       - Silicon Laboratories, Inc.

         -  `EFM32 Gecko <#efm32g>`__ (ARM Cortex-M3)
         -  `EFM32 Giant Gecko <#efm32gg>`__ (ARM Cortex-M3)

       - Sony.

         -  `CXD56\ xx <#cxd56xx>`__ (6 x ARM Cortex-M4)

       - STMicroelectronics

         -  `STMicro STR71x <#str71x>`__ (ARM7TDMI)
         -  `STMicro STM32F0xx <#stm32f0xx>`__ (STM32 F0, |br|
            ARM Cortex-M0)
         -  `STMicro STM32L0xx <#stm32l0xx>`__ (STM32 L0, |br|
            ARM Cortex-M0)
         -  `STMicro STM32G0xx <#stm32g0xx>`__ (STM32 G0 |br|
            ARM Cortex-M0+)
         -  `STMicro STM32L152 <#stm32l152>`__ (STM32 L1 |br|
            "EnergyLite" Line, ARM Cortex-M3)
         -  `STMicro STM32L162 <#stm32l162>`__ (STM32 L1 |br|
            "EnergyLite" Medium+ Density, |br|
            ARM Cortex-M3)
         -  `STMicro STM32F100x <#stm32f100x>`__ (STM32 F1 |br|
            "Value Line" Family, ARM Cortex-M3)
         -  `STMicro STM32F102x <#stm32f102x>`__ (STM32 F1 |br|
            family, ARM Cortex-M3)
         -  `STMicro STM32F103C4/C8 <#stm32f103cx>`__ (STM32 F1 |br|
            "Low- and Medium-Density Line" |br|
            Family, ARM Cortex-M3)
         -  `STMicro STM32F103x <#stm32f103x>`__ (STM32 F1 |br|
            family, ARM Cortex-M3)
         -  `STMicro STM32F105x <#stm32f105x>`__ (ARM Cortex-M3)
         -  `STMicro STM32F107x <#stm32f107x>`__ (STM32 F1  |br|
            family, "Connectivity Line" |br|
            ARM Cortex-M3)
         -  `STMicro STM32F205x <#stm32f205x>`__ (STM32 F2 |br|
            family, ARM Cortex-M3)
         -  `STMicro STM32F207x <#stm32f207x>`__ (STM32 F2 |br|
            family, ARM Cortex-M3)
         -  `STMicro STM32F302x <#stm32f302x>`__ (STM32 F3 |br|
            family, ARM Cortex-M4)
         -  `STMicro STM32F303x <#stm32f303x>`__ (STM32 F3 |br|
            family, ARM Cortex-M4)
         -  `STMicro STM32F334 <#stm32f334x>`__ (STM32 F3 |br|
            family, ARM Cortex-M4)

         |br|
         |br|
         |br|
         |br|
         |br|
         |br|
         |br|
         |br|
         |br|
         |br|
         |br|
         |br|
         |br|
         |br|
         |br|
         |br|
         |br|
         |br|
         |br|
         |br|
         |br|
         |br|
         |br|

     -

       - STMicroelectronics (Continued)

         -  `STMicro STM32 F372/F373 <#stm32f372x>`__ |br|
            (ARM Cortex-M4)
         -  `STMicro STM32F4x1 <#stm32f4x1>`__ |br|
            (STM32 F4 family, ARM |br|
            Cortex-M4)
         -  `STMicro STM32F410 <#stm32f410>`__ |br|
            (STM32 F4 family, ARM |br|
            Cortex-M4)
         -  `STMicro STM32F405x/407x <#stm32f407x>`__ |br|
            (STM32 F4 family, ARM |br|
            Cortex-M4)
         -  `STMicro STM32 F427/F437 <#stm32f427x>`__ |br|
            (STM32 F4 family, ARM |br|
            Cortex-M4)
         -  `STMicro STM32 F429 <#stm32f429x>`__ |br|
            (STM32 FB family, ARM |br|
            Cortex-M4)
         -  `STMicro STM32 F433 <#stm32f433x>`__ |br|
            (STM32 F4 family, ARM |br|
            Cortex-M4)
         -  `STMicro STM32 F446 <#stm32f446x>`__ |br|
            (STM32 F4 family, ARM |br|
            Cortex-M4)
         -  `STMicro STM32 F46xx <#stm32f46xxx>`__ |br|
            (STM32 F4 family, ARM |br|
            Cortex-M4)
         -  `STMicro STM32 G474x <#stm32g474x>`__ |br|
            (STM32 G4 family, ARM |br|
            Cortex-M4)
         -  `STMicro STM32 L4x2 <#stm32l4x2>`__ |br|
            (STM32 L4 family, ARM |br|
            Cortex-M4)
         -  `STMicro STM32 L475 <#stm32l475>`__ |br|
            (STM32 L4 family, ARM |br|
            Cortex-M4)
         -  `STMicro STM32 L476 <#stm32l476>`__ |br|
            (STM32 L4 family, ARM |br|
            Cortex-M4)
         -  `STMicro STM32 L496 <#stm32l496>`__ |br|
            (STM32 L4 family, ARM |br|
            Cortex-M4)
         -  `STMicro STM32 L4Rx <#stm32l4rx>`__ |br|
            (STM32 LB family, ARM |br|
            Cortex-M4)
         -  `STMicro STM32 F72x/F73x <#stm32f72x3x>`__ |br|
            (STM32 F7 family, ARM |br|
            Cortex-M7)
         -  `STMicro STM32 F745/F746 <#stm32f74x>`__ |br|
            (STM32 F7 family, ARM |br|
            Cortex-M7)
         -  `STMicro STM32 F756 <#stm32f75x>`__ |br|
            (STM32 F7 family, ARM |br|
            Cortex-M7)
         -  `STMicro STM32 F76xx/F77xx <#stm32f76xx77xx>`__ |br|
            (STM32 F7 family, ARM |br|
            Cortex-M7)
         -  `STMicro STM32 H7x3 <#stm32h7x3>`__ |br|
            (STM32 H7 family, ARM |br|
            Cortex-M7)

       - Texas Instruments

         - (some formerly Luminary)
         -  `TI TMS320-C5471 <#tms320c5471>`__ |br|
            (ARM7TDMI)
         -  `TI TMS320-DM320 <#titms320dm320>`__ |br|
            (ARM9E6JS)
         -  `TI/Stellaris LM3S6432 <#tilms6432>`__ |br|
            (ARM Cortex-M3)
         -  `TI/Stellaris LM3S6432S2E <#tilm3s6432s2e>`__ |br|
            (ARM Cortex-M3)
         -  `TI/Stellaris LM3S6918 <#tilms6918>`__ |br|
            (ARM Cortex-M3)
         -  `TI/Stellaris LM3S6965 <#tilms6965>`__ |br|
            (ARM Cortex-M3)
         -  `TI/Stellaris LM3S8962 <#tilms8962>`__ |br|
            (ARM Cortex-M3)
         -  `TI/Stellaris LM3S9B92 <#tilms9b92>`__ |br|
            (ARM Cortex-M3)
         -  `TI/Stellaris LM3S9B96 <#tilms9b96>`__ |br|
            (ARM Cortex-M3)
         -  `TI/SimpleLink CC13x0 <#tilcc13x0>`__ |br|
            (ARM Cortex-M3)
         -  `TI/Stellaris LM4F120x <#tilm4f120x>`__ |br|
            (ARM Cortex-M4)
         -  `TI/Tiva TM4C123G <#titm4c123g>`__ |br|
            (ARM Cortex-M4)
         -  `TI/Tiva TM4C1294 <#titm4c1294>`__ |br|
            (ARM Cortex-M4)
         -  `TI/Tiva TM4C129X <#titm4c129x>`__ |br|
            (ARM Cortex-M4)
         -  `TI/SimpleLink CC13x2 <#tilcc13x2>`__ |br|
            (ARM Cortex-M4)
         -  `TI/Hercules TMS570LS04xx <#tms570ls04x>`__ |br|
            (ARM Cortex-R4)
         -  `TI/Hercules TMS570LS31xx <#tms570ls31x>`__ |br|
            (ARM Cortex-R4)
         -  `TI/Sitara AM335x <#tiam355x>`__ |br|
            (Cortex-A8)

       - ZiLOG

         -  `ZiLOG ZNEO Z16F <#zilogz16f>`__
         -  `ZiLOG eZ80 Acclaim! <#zilogez80acclaim>`__
         -  `ZiLOG Z8Encore! <#zilogz8encore>`__
         -  `ZiLOG Z180 <#zilogz180>`__
         -  `ZiLOG Z80 <#zilogz80>`__

