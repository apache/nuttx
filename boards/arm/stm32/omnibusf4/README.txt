OMNIBUSF4 Target Support README
===============================

"OmnibusF4" is not a product name per se, but rather a design spec
that many product vendors within the drone flight management unit
(FMU) community adhere to. The spec defines the major components, and
how those components are wired into the STM32F405RGT6 microcontroller.

Airbot is one such vendor, and they publish a schematic here:

    http://bit.ly/obf4pro

Other software that supports the OmnibusF4 family include Betaflight,
iNAV, and many others. PX4 recently added support as well. No code
from any of those sources is included in this port.

Since OmnibusF4 is a drone FMU, most of its IO is already allocated to
FMU-specific tasks. As such, we don't need to make the board support
package as flexible as, say, an STM32F4 Discovery board.

The following are some of the committed IO pins. Most of the pins not
mentioned here are inaccessible, the details vary by board vendor:

io    peripheral   signal    notes
==================================
XIN                          8MHz crystal oscillator

PB0     TIM3 CH3   S1_OUT    motor 1 PWM output
PB1     TIM3 CH4   S2_OUT    motor 2 PWM output
PA3     TIM2 CH4   S3_OUT    motor 3 PWM output
PA2     TIM2 CH3   S4_OUT    motor 4 PWM output
PA1     TIM2 CH2   S5_OUT    motor 5 PWM output
PA8     TIM1 CH4   S6_OUT    motor 6 PWM output

PA4     SPI1      SPI1_NSS   mpu6000
PA5     SPI1      SPI1_SCL
PA6     SPI1      SPI1_MISO
PA7     SPI1      SPI1_MOSI

PC4     GPIO      GYRO_INT   mpu6000 EXTI

PB10    UART3/I2C UART3_TX   ttl UART tx or i2c_scl (used as console)
PB11    UART3/I2C UART3_RX   ttl UART rx or i2c_sda (used as console)

PB9                          RC_CH2 (rx pwm input)
PB8                          RC_CH1 (rx pwm input)
PC9                          RC_CH6 (rx pwm input)
PC8                          RC_CH5 (rx pwm input)
PC7                          RC_CH4 or USART6_RX (ttl)
PC6                          RC_CH3 or USART6_TX (ttl)

PB7     GPIO      SD_DET     SD card detection pin (low when card inserted)
PB5     GPIO      STAT       LED output (active low)
PB4     GPIO      BUZZER     buzzer output (active low)

PD2     GPIO      LED_STRIP  one-wire interface for LED strips

PC12    SPI3      SPI3_MOSI  bmp280 barometer (if populated) and/or max7456 OSD
PC11    SPI3      SPI3_MISO
PC10    SPI3      SPI3_SCL
PA15    GPIO      SPI3_NSS   OSD NSS
PB3     GPIO      BARO_CS    bmp280 NSS (if populated)

PA12    OTG       USB_DP
PA11    OTG       USB_DN

PA10    UART1     USART1_RX  SBUS_IN (through inverter) or PPM
PA9     UART1     USART1_TX

PB15    SPI2      SPI2_MOSI  sd/mmc card interface
PB14    SPI2      SPI2_MISO
PB13    SPI2      SPI2_SCLK
PB12    SPI2      SPI2_NSS

Build Instructions
==================

The boards/arm/stm32/omnibusf4/nsh/defconfig file creates a basic setup, and
includes drivers for all supported onboard chips. The console and
command prompt are sent to USART3.
