=============
Kendryte K230
=============

The `Kendryte K230 <https://www.canaan.io/product/k230>`_ SoC contains two 64 bit RV64GC CPUs based on T-Head C908 IP core for embedded IoT applications.

- **CPU1:** RISC-V Core, 1.6GHz, 32KB I-cache, 32KB D-cache, 256KB L2 Cache, 128bit RVV 1.0 support
- **CPU0:** RISC-V Core, 0.8GHz, 32KB I-cache, 32KB D-cache, 256KB L2 Cache
- **KPU:** INT8 and INT16 Inference performance: Restnet-50>85fps@INT8, Mobilenet_v2 >670fps@INT8, YoloV5S>38fps@INT8
- **DPU:** for 3D structured light, resolution is up to 1920*1080
- **RAM:** 32-bit LPDDR4 / DDR4 / LPDDR3 / DDR3
- **Video Codec:** H.264, H.265 and JPEG support;
- **Video Input:** 3 x MIPI-CSI
- **Video Output:** 1 x MIPI DSI
- **USB:** USB-OTG 2.0 x 2
- **Security:** TRNG, OTP
- **Peripherals:** 5xUART, 5xI2C, 6xPWM, 64xGPIO+8xPMU, 2xSDxC:SD3.0, 3xSPI, WDT/RTC/Timer

See more details from above vendor's website.


Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
