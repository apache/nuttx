=============
SOPHGO SG2000
=============

`SOPHGO SG2000 <https://milkv.io/chips/sg2000>`_ is a 64-bit RISC-V SoC with 2 RISC-V Cores and 1 Arm Core:

- **Main Processor:** T-Head C906 1.0 GHz 64-bit RISC-V Core
    - 32 KB I-Cache, 64KB D-Cache
    - Integrated Vector and Floating-Point Operation Unit (FPU)

- **Co-Processor:** T-Head C906 700 MHz 64-bit RISC-V Core
    - Integrated Floating-Point Unit (FPU)

- **Alternate Main Processor:** Cortex-A53 1.0 GHz 64-bit Arm Core
    - 32 KB I-Cache, 32 KB D-Cache
    - 128 KB L2 Cache
    - Support Neon and Floating-Point Operation Unit (FPU)

- **MCU:** 8051 with 6 KB SRAM
- **TPU:** 0.5 TOPS INT8
- **Memory:** SIP DRAM 512 MB
- **Video Module:** ISP 5M @ 30 FPS, 2L MIPI DSI 5M @ 30 FPS, 4L or 2L+2L MIPI CSI 5M @ 30 FPS, H.265 / H.264 Decoding and Encoding 5M @ 30 FPS
- **Audio Module:** 16-bit Audio Codec, 2 x I2S / PCM, 1 x DMIC
- **Storage:** SPI-NOR, SPI-NAND, eMMC 5.0, 2 x SDIO 3.0
- **Network:** 10M / 100M MAC PHY
- **Security Module:** Crypto, Secure Boot, TRNG, Efuse
- **Peripherals:** 1 x USB 2.0 DRD, 5 x UART, 4 x SPI, 16 x PWM, 1 x IR, 6 x I2C, 6 x ADC, GPIOs

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
