This is a port of NuttX to the STM32L4 Family
Used development board is the Nucleo L476RG

The status is HIGHLY EXPERIMENTAL.

OSTEST application works, but drivers are not complete.

Most code is copied and adapted from the STM32 Port.

TODO list
---------

Peripherals with equivalent implementation in STM32 port

IRQs     : OK
GPIO     : OK
EXTI     : OK, to be tested.
HSI      : OK
HSE      : To be tested
PLL      : Works @ 80 MHz
MSI      : TODO
LSE      : TODO, including calibration
RCC      : All registers defined, peripherals enabled, basic clock working
SYSCTL   : All registers defined
USART    : Working in normal mode (no DMA, to be tested, code is written)
DMA      : Ported from STM32, code written, to be tested
SRAM2    : Should work with enough MM regions
FIREWALL : Code written, to be tested, requires support from ldscript
SPI      : Code written, to be tested, including DMA
I2C      : Registers defined
RTC      : TODO
QSPI     : TODO (port from stm32f7)
CAN      : TODO
OTGFS    : TODO
Timers   : TODO
PM       : TODO, PWR registers defined
FSMC     : TODO
AES      : TODO
RNG      : TODO
CRC      : TODO (configurable polynomial)
WWDG     : TODO
IWDG     : TODO
MMCSD    : TODO
ADC      : TODO
DAC      : TODO

New peripherals with implementation to be written from scratch
These are Low Priority TODO items, unless someone requests or contributes it.

TSC      : TODO (Touch Screen Controller)
SWP      : TODO (Single wire protocol master, to connect with NFC enabled SIM cards)
LPUART   : TODO (Low power UART working with LSE at low baud rates)
LPTIMER  : TODO (Low power TIMER)
OPAMP    : TODO (Analog operational amplifier)
COMP     : TODO (Analog comparators)
DFSDM    : TODO (Digital Filter and Sigma-Delta Modulator)
LCD      : TODO (Segment LCD controller)
SAIPLL   : TODO (PLL For Digital Audio interfaces)
SAI      : TODO (Digital Audio interfaces, I2S, SPDIF, etc)

