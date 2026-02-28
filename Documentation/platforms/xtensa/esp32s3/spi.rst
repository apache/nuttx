.. _esp32s3_spi:

SPI Configuration for ESP32-S3
===============================

This guide explains how to set up SPI on the ESP32-S3. I wrote this after figuring out the configuration myself - hopefully it helps others avoid the same confusion.

Prerequisites
-------------
Before starting, make sure:
- You can already build NuttX for ESP32-S3
- You know how to use menuconfig (basic navigation is enough)

Configuration Steps
-------------------
Here's what I did to get SPI working:

1. **Enable SPI support**
   
   Run `make menuconfig` and go to:
   
```Device Drivers -> SPI Support```


Enable `[*] SPI`

2. **Select the SPI peripheral**

Go to:
```System Type -> ESP32-S3 Peripheral Selection```

Enable `[*] SPI2`

3. **Enable SPI Exchange**

Back in:
```Device Drivers -> SPI Support```

Enable `[*] SPI Exchange`

4. **Enable Chip Select (optional)**

In the same menu, enable:
[*] SPI Chip Select

You can change the CS pin from GPIO10 if needed.

Pin Mapping
-----------
Default pins for SPI2:

- SCK (Clock) → GPIO12
- MOSI → GPIO11
- MISO → GPIO13
- CS (Chip Select) → GPIO10

If these pins conflict with your board, you can change them using the GPIO matrix.

Chip Select Options
-------------------
ESP32-S3 gives you three ways to handle Chip Select:

**1. Hardware CS (easiest)**
- Just enable `CONFIG_ESP32S3_SPI2_CS_ENABLE=y`
- Hardware handles everything automatically
- Best for most projects

**2. Software CS**
- You control CS through the SPI driver
- More flexible if you need custom behavior

**3. Manual GPIO**
- You control CS yourself with gpio_write()
- Maximum control but more code to write

I'd suggest starting with option 1 (hardware CS) - it's simpler.

Common Problems
---------------
Issues I ran into:

- **SPI options missing in menuconfig** → Enable DMA first
Go to `Device Drivers -> DMA Support` and enable `[*] DMA`

- **SPI not working** → Check if pins are used by something else
Look at your board schematic to verify no conflicts

- **Build errors** → Make sure CONFIG_SPI_EXCHANGE is enabled
Check that you completed step 3 above

How to Test
-----------
**Method 1: Use the built-in test**

.. code-block:: bash

    cd apps/examples/spi
    make
    ./spi_test

You should see:

.. code-block::

    SPI Test Starting
    Initializing SPI bus 2
    SPI bus 2 initialized successfully
    Test PASSED

**Method 2: Quick code test**

Add this to your application:

.. code-block:: c

    #include <nuttx/spi/spi.h>

    void test_spi(void)
    {
        struct spi_dev_s *spi;
        uint8_t tx[4] = {0x55, 0xAA, 0x00, 0xFF};
        uint8_t rx[4] = {0};
    
        spi = esp32s3_spibus_initialize(2);
        if (!spi) {
            printf("Failed to initialize SPI\n");
            return;
        }
    
        SPI_LOCK(spi, true);
        SPI_SETFREQUENCY(spi, 1000000);
        SPI_EXCHANGE(spi, tx, rx, 4);
        SPI_LOCK(spi, false);
    
        printf("Sent: %02x %02x %02x %02x\n", tx[0], tx[1], tx[2], tx[3]);
        printf("Got:  %02x %02x %02x %02x\n", rx[0], rx[1], rx[2], rx[3]);
    }

**Still not working?**

Try these:
- Double-check DMA is enabled
- Try a slower speed (100kHz)
- Ask on the mailing list: https://lists.apache.org/list.html?dev@nuttx.apache.org
