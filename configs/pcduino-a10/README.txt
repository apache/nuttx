README
======

  This directory contains the port of NuttX to the pcDuino v1 board
  See http://www.pcduino.com/ for information about pcDuino Lite, v1,
  and v2.  These boards are based around the Allwinner A10 Cortex-A8 CPU.
  I have not compared these boards in detail, but I believe that the
  differences are cosmetic.  This port was developed on the v1 board, but
  the others may be compatible:
  
  pcDuino Lite (See http://www.pcduino.com/?page_id=1707)

    ITEMS                DETAILS
    -------------------- ---------------------------------------------------
    CPU                  1GHz ARM Cortex A8
    GPU                  OpenGL ES2.0, OpenVG 1.1 Mali 400 core
    DRAM                 512B
    Onboard Storage      NO Flash, microSD card (TF) slot for up to 32GB
    Video Output         HDMI
    Extension Interface  2.54mm Headers
    Network interface    10/100Mbps RJ45 and USB WiFi extension (not included)
    Power                5V, 2000mA
    Overall Size         125mm X 52mm

  pcDuino v1 (http://www.pcduino.com/?page_id=12)

    ITEMS                DETAILS
    -------------------- ---------------------------------------------------
    Items                Details
    CPU                  1GHz ARM Cortex A8
    GPU                  OpenGL ES2.0, OpenVG 1.1 Mali 400 core
 *  DRAM                 1GB
 *  Onboard Storage      2GB Flash, microSD card (TF) slot for up to 32GB
    Video Output         HDMI
    Extension Interface  2.54mm Headers
    Network interface    10/100Mbps RJ45 and USB WiFi extension (not included)
    Power                5V, 2000mA
    Overall Size         125mm X 52mm

  pcDuino v2 (http://www.pcduino.com/?page_id=1618)

    ITEMS                DETAILS
    -------------------- ---------------------------------------------------
    Items                Details
    CPU                  1GHz ARM Cortex A8
    GPU                  OpenGL ES2.0, OpenVG 1.1 Mali 400 core
    DRAM                 1GB
    Onboard Storage      2GB Flash, microSD card (TF) slot for up to 32GB
    Video Output         HDMI
 *  Extension Interface  Arduino Headers
 *  Network interface    10/100Mbps RJ45 and on-board WiFi module
    Power                5V, 2000mA
    Overall Size         125mm X 52mm

  Main features of the Allwinner A10
  (See http://www.allwinnertech.com/en/product/a10.html):

  CPU
    - ARM Cortex™-A8
    - 32KB I-Cache
    - 32KB D-Cache
    - 256KB L2 Cache

  GPU
    - ARM Mali-400

  Video
    - UHD 2160P video decoding
    - 3D video decoding
    - Support various video decoding formats, including VP8, AVS, H. 264
      MVC, VC-1, MPEG-1,2,4, etc
    - H.264 HP video encoding up to 1080p @ 30 fps or dual-channel 720p @ 30
      fps

  Display
    - Multi-channel HD display
    - Integrated HDMI 1.4
    - YPbPr, CVBS, VGA
    - Multiple LCD interfaces, including CPU, RGB, LVDS up to Full HD

  Memory
    - 32-bit DDR2/DDR3
    - Memory capacity up to 16G bits
    - SLC/MLC/TLC/DDR NAND
    - 8 flash chips, 64-bit ECC

        Memory capacity up to 64GB
        Support NAND of 5xnm, 4xnm, 3xnm, 2xnm, etc
        Support NAND of Samsung, Toshiba, Hynix, etc

  Boot Devices
    - NAND Flash
    - SPI NOR Flash
    - SD Card
    - USB

Contents
========

  - Serial Console
  - LEDs
  - Buttons

Serial Console
==============

  To be provided

LEDs
====

  To be provided

Buttons
=======

  To be provided
