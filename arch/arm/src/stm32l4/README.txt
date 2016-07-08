This is a port of NuttX to the STM32L4 Family
Used development board is the Nucleo L476RG, STM32L4VGDiscovery

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
HSE      : OK
PLL      : Works @ 80 MHz
MSI      : OK
LSE      : OK
RCC      : All registers defined, peripherals enabled, basic clock working
SYSCTL   : All registers defined
USART    : Working in normal mode (no DMA, to be tested, code is written)
DMA      : works; at least tested with QSPI
SRAM2    : OK; can be included in MM region or left separate for special app purposes
FIREWALL : Code written, to be tested, requires support from ldscript
SPI      : Code written, to be tested, including DMA
I2C      : Registers defined
RTC      : works
QSPI     : works in polling, interrupt, DMA, and also memory-mapped modes
CAN      : TODO
OTGFS    : dev implemented, tested, outstanding issue with CDCACM (ACM_SET_LINE_CODING, but otherwise works);
         : host implemented, only build smoke-tested (i.e. builds, but no functional testing yet)
Timers   : Implemented, with PWM oneshot and freerun, tickless OS support.  Limited testing (focused on tickless OS so far)
PM       : TODO, PWR registers defined
FSMC     : TODO
AES      : TODO
RNG      : works
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
SAIPLL   : works (PLL For Digital Audio interfaces, and other things)
SAI      : TODO (Digital Audio interfaces, I2S, SPDIF, etc)

