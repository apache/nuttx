This is a port of NuttX to the STM32WL5 Family.

Used development board is Nucleo WL55JC.

Most code is copied and adapted from STM32L4 and STM32L5 ports.

There are only two chips in family, STM32WL55 and STM32WL54. Only difference
between them is that STM32WL55 has LORA radio while WL54 does not.

STM32WL5 is a dual CPU (not core!) platform. Separate code must be generated
for both of them.

Only CPU0 has access to radio, but other peripherals are shared. CPU1 can
initialize all hardware (except for radio and CPU0 specific registers).

TODO list
---------

IRQs        : OK
GPIO        : OK
EXTI        : TODO
HSE         : OK
PLL         : OK @ 48MHz
HSI         : Not tested
MSI         : Not tested
LSE         : Not tested
RCC         : All registers defined, not all peripherals enabled
SYSCFG      : All registers defined, remapping not tested
USART       : OK
LPUART      : Partial OK
              OK   - full speed with HSE
              TODO - low power mode with LSE
DMA         : TODO
SRAM2       : TODO
SPI         : TODO
I2C         : TODO
RTC         : TODO
Timers      : TODO
PM          : TODO
AES         : TODO
RNG         : TODO
CRC         : TODO
WWDG        : TODO
IWDG        : TODO
ADC         : TODO
DAC         : TODO
CPU0<->CPU1 : TODO
Radio@CPU0  : TODO
