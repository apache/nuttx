This is a port of NuttX to the STM32L4 Family

Used development boards are the Nucleo L476RG, Nucleo L496ZG,
Nucleo L452RE, Nucleo L432KC, STM32L4VG Discovery and
Motorola MDK.

Most code is copied and adapted from the STM32 and STM32F7 ports.

The various supported STM32L4 families are:

-----------------------------------------------------------------
| NuttX config      | Manual | Chips
|
| Not supported     | RM0392 | STM32L471xx
|
| STM32L4_STM32L4X1 | RM0394 | Subset of STM32L4_STM32L4X3 [*]
|
| STM32L4_STM32L4X2 | RM0394 | Subset of STM32L4_STM32L4X3 [*]
|
| STM32L4_STM32L4X3 | RM0394 | STM32L43xxx/44xxx/45xxx/46xxx
|
| STM32L4_STM32L4X5 | RM0351 | STM32L475xx (was RM0395 in past)
|
| STM32L4_STM32L4X6 | RM0351 | STM32L476xx, STM32L486xx,
|                              STM32L496xx, STM32L4A6xx
|
| STM32L4_STM32L4XR | RM0432 | STM32L4Rxxx, STM32L4Sxxx (STM32L4+)
------------------------------------------------------------------

[*]: Please avoid depending on CONFIG_STM32L4_STM32L4X1 and
     CONFIG_STM32L4_STM32L4X2 as the MCUs are of the same subfamily
     as CONFIG_STM32L4_STM32L4X3.

TODO list
---------

Peripherals with implementation in STM32 port:

IRQs     : OK
GPIO     : OK
EXTI     : OK
HSI      : OK
HSE      : OK
PLL      : Works @ 80 MHz
MSI      : OK
LSE      : OK
RCC      : All registers defined, peripherals enabled, basic clock working
SYSCTL   : All registers defined
USART    : OK
DMA      : OK
SRAM2    : OK; can be included in MM region or left separate for special app
         : purposes
SPI      : OK, tested (including DMA)
I2C      : works
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
SDMMC    : works
ADC      : works
DAC      : works
DMA2D    : TODO (Chrom-Art Accelerator for image manipulation)

New peripherals with implementation to be written from scratch.
These are Low Priority TODO items, unless someone requests or contributes
it.

FIREWALL : Code written, to be tested, requires support from ldscript
TSC      : TODO (Touch Screen Controller)
SWP      : TODO (Single wire protocol master, to connect with NFC enabled
         : SIM cards)
LPUART   : Experimental support (Low power UART working with LSE at low
         : baud rates)
LPTIM    : Code written, to be tested (Low power TIMER)
OPAMP    : TODO (Analog operational amplifier)
COMP     : There is some code (Analog comparators)
DFSDM    : There is some code (Digital Filter for Sigma-Delta Modulators)
LCD      : TODO (Segment LCD controller)
SAIPLL   : works (PLL For Digital Audio interfaces, and other things)
SAI      : There is some code (Digital Audio interfaces, I2S, SPDIF, etc)
HASH     : TODO (SHA-1, SHA-224, SHA-256, HMAC)
DCMI     : TODO (Digital Camera interfaces)

New peripherals only in STM32L4+:

DMAMUX1    : OK
DSI        : TODO
GFXMMU     : TODO
LTDC       : TODO
OCTOSPI    : TODO
OCTOSPIIOM : TODO
