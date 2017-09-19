This is a port of NuttX to the STM32L4 Family

Used development boards are the Nucleo L476RG, Nucleo L496ZG,
Nucleo L452RE and STM32L4VGDiscovery.

Most code is copied and adapted from the STM32 Port.

TODO list
---------

Peripherals with implementation in STM32 port:

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
SRAM2    : OK; can be included in MM region or left separate for special app
         : purposes
SPI      : OK, tested (Including DMA)
I2C      : Code written, to be tested
RTC      : works
QSPI     : works in polling, interrupt, DMA, and also memory-mapped modes
CAN      : OK, tested
OTGFS    : dev implemented, tested, outstanding issue with CDCACM
         : (ACM_SET_LINE_CODING, but otherwise works); host implemented,
         : only build smoke-tested (i.e. builds, but no functional testing
         : yet)
Timers   : Implemented, with PWM oneshot and freerun, tickless OS support.
         : Limited testing (focused on tickless OS so far), PWM and QE tested OK.
PM       : TODO, PWR registers defined
FSMC     : TODO
AES      : TODO
RNG      : works
CRC      : TODO (configurable polynomial)
WWDG     : TODO
IWDG     : works
MMCSD    : TODO
ADC      : Code written, to be tested
DAC      : Code written, to be tested
DMA2D    : TODO (Chrom-Art Accelerator for image manipulation)

New peripherals with implementation to be written from scratch
These are Low Priority TODO items, unless someone requests or contributes
it.

FIREWALL : Code written, to be tested, requires support from ldscript
TSC      : TODO (Touch Screen Controller)
SWP      : TODO (Single wire protocol master, to connect with NFC enabled
         : SIM cards)
LPUART   : TODO (Low power UART working with LSE at low baud rates)
LPTIM    : Code written, to be tested (Low power TIMER)
OPAMP    : TODO (Analog operational amplifier)
COMP     : There is some code (Analog comparators)
DFSDM    : There is some code (Digital Filter for Sigma-Delta Modulators)
LCD      : TODO (Segment LCD controller)
SAIPLL   : works (PLL For Digital Audio interfaces, and other things)
SAI      : There is some code (Digital Audio interfaces, I2S, SPDIF, etc)
HASH     : TODO (SHA-1, SHA-224, SHA-256, HMAC)
DCMI     : TODO (Digital Camera interfaces)
