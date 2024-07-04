==================
Bouffalo Lab BL808
==================

`Bouffalo Lab BL808 <https://github.com/bouffalolab/bl_docs/tree/main/BL808_RM/en>`_ is a 64-bit / 32-bit RISC-V SoC with 3 RISC-V Cores:

- **D0 Multimedia Core:** T-Head C906 480 MHz 64-bit RISC-V CPU
    - RV64IMAFCV
    - Level 1 Instruction and Data Cache (Harvard architecture)
    - Sv39 Memory Management Unit
    - jTLB (128 entries)
    - AXI 4.0 128-bit master interface
    - Core Local Interrupt (CLINT) and Platform-Level Interrupt Controller (PLIC)
    - 80 External Interrupt Sources
    - BHT (8K) and BTB
    - RISC-V PMP (8 configurable areas)

- **M0 Wireless Core:** T-Head E907 320 MHz 32-bit RISC-V CPU
    - RV32IMAFCP
    - 32-bit / 16-bit Mixed Instruction Set
    - RISC-V Machine Mode and User Mode
    - 32 x 32-bit Integer General Purpose Registers (GPR)
    - 32 x 32-bit / 64-bit Floating-Point GPRs
    - AXI 4.0 main device interface and AHB 5.0 peripheral interface
    - Instruction and Data Cache

- **LP Low Power Core:** T-Head E902 150 MHz 32-bit RISC-V CPU
    - RV32E[M]C

- **RAM:** Embedded 64 MB PSRAM
- **Wireless:** 2.4 GHz 1T1R WiFi 802.11 b/g/n, Bluetooth 5.2, Zigbee
- **Ethernet:** 10 / 100 Mbps
- **USB:** USB 2.0 OTG
- **Audio:** Microphone and Speaker
- **Video Input:** Dual-lane MIPI CSI
- **Peripherals:** UART, SPI, I2C, PWM, SDH, EMAC, USB

Peripheral Support
==================

The following list indicates the state of peripherals' support in NuttX:

=========== ======= ====================
Peripheral  Support NOTES
=========== ======= ====================
GPDAC        No
DMA          No
EMAC         No
GPADC        Yes
GPIO         Yes
I2C          No
I2S          No
PWM          No
SPI          No
Timers       No
UART         Yes
USB          No
=========== ======= ====================

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
