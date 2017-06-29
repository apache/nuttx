README
======

The Olimex STM32-H407 configuration is based on
stm32Fdiscovery and Olimex STM32-H405.

The H407 was programmed with ST-LINK/V2 from both Win8.1 and Ubuntu 14.04
This release provides baseline for H407 12MHZ clock in include/board.h

nsh - Only basic shell response tested on USART2

Development Environment
=======================

Either Linux or Cygwin on Windows can be used for the development environment.
The source has been built only using the GNU toolchain (see below).  Other
toolchains will likely cause problems.

LEDs
====

The H407 board has 1 Status LED;

SDCard
======

Expects to be plugged in else hangs. TODO: Shouldn't hang.
SDIO 4bit with pulls/downs on pins. Doesn't have a SD card detect pin.

UARTs
=====

On the H407 board, ?? all uarts are available for pin mappings
USART2 is typically used for nsh console

STM32-H407-specific Configuration Options
===============================================
