# HPM6750EVK2

## Overview

The HPM6750 is a dual-core flashless MCU running 816Mhz. It has a 2MB continuous on-chip ram. Also, it provides various memory interfaces, including SDRAM, Quad SPI NOR Flash, SD/eMMC. It integrates rich audio and video interfaces, including LCD, pixel DMA, camera， and I2S audio interfaces.

 ![hpm6750evk2](../../doc/images/boards/hpm6750evk2/hpm6750evk2.png "hpm6750evk2")

## Hardware

- HPM6750IVM MCU (816MHz, 2MB OCRAM)
- Onboard Memory
  - 256Mb SDRAM
  - 128Mb Quad SPI NOR Flash
- Display & Camera
  - LCD connector
  - Camera (DVP)
- Ethernet
  - 1000 Mbits PHY
  - 100 Mbits PHY
- USB
  - USB type C (USB 2.0 OTG) connector x3
- Audio
  - Line in
  - Mic
  - DAO
- Others
  - TF Slot
  - RGB LED
  - CAN
- Expansion port
  - Motor control

## DIP Switch S1

- Bit 1 and 2 controls boot mode

| bit[2:1] | Description                  |
| -------- | ---------------------------- |
| OFF, OFF | Boot from Quad SPI NOR flash |
| OFF, ON  | Serial boot                  |
| ON, OFF  | ISP                          |

(lab_hpm6750_evk2_board)=

## Button

(lab_hpm6750_evk2_board_buttons)=

| Name         | FUNCTIONS                                      |
| ------------ | ---------------------------------------------- |
| PBUTN (SW1)  | Power Button, TinyUF2 Boot Button, GPIO Button |
| WBUTN (SW2)  | WAKE UP Button                                 |
| RESETN (SW3) | Reset Button                                   |

## Plug-in

- `J108` ADC/DAC reference voltage selection
  | Connection | Description         |
  | ---------- | ------------------- |
  | 1, 2       | 3.3V                |
  | 2, 3       | High precision 3.3V |

## Resistor Switch

- GigE POP `R177-R182`
  | Status     | Description |
  | ---------- | ----------- |
  | Welding    | Network     |
  | Disconnect | Motor       |

## Pin Description

- UART0 Pin:

The UART0 pin leads to three positions:

| Function | Pin  | Position1 | Position2  | Position3 |
| -------- | ---- | --------- | ---------- | --------- |
| UART0.TX | PY06 | J109[1]   | JTAG P4[7] | USB2UART0 |
| UART0.RX | PY07 | J109[3]   | JTAG P4[9] | USB2UART0 |

Note: To avoid abnormal functions caused by multiple connections, please ensure that there is only one connection. For example, if the JTAG interface is connected to UART0, USB2UART0 interface send data to UART0 will be abnormal.

- SPI Pin：

| Function  | Pin  | Position |
| --------- | ---- | -------- |
| SPI2.CSN  | PE31 | J11[7]   |
| SPI2.SCLK | PE27 | J11[8]   |
| SPI2.MISO | PE28 | J11[9]   |
| SPI2.MOSI | PE30 | J11[10]  |

- I2C Pin：：

| Function | Pin  | Position |
| -------- | ---- | -------- |
| I2C0.SCL | PZ11 | J11[3]   |
| I2C0.SDA | PZ10 | J11[4]   |

- UART for core1 debug console：

| Function   | Pin  | Position |
| ---------- | ---- | -------- |
| UART13.TXD | PZ09 | J11[5]   |
| UART13.RXD | PZ08 | J11[6]   |

- PWM Pin:

| Function  | Pin  | Position |
| --------- | ---- | -------- |
| PWM2.P[0] | PD31 | J10[14]  |
| PWM2.P[1] | PD30 | J10[13]  |

- ACMP Pin

| Function   | Pin  | Position |
| ---------- | ---- | -------- |
| CMP.INN6   | PE21 | J10[8]   |
| CMP.COMP_1 | PE25 | J10[6]   |

- GPTMR Pin

| Function      | Pin  | Position |
| ------------- | ---- | -------- |
| GPTMR4.CAPT_1 | PE25 | J10[6]   |
| GPTMR3.COMP_1 | PE24 | J10[7]   |

- ADC12 Pin

| Function          | Pin   | Position |
| ----------------- | ----- | -------- |
| Reference Voltage | VREFH | J108[2]  |
| ADC0/1/2.VINP11   | PE25  | J10[6]   |
| ADC0/1/2.VINP10   | PE24  | J10[7]   |
| ADC0/1/2.VINP7    | PE21  | J10[8]   |

- ADC16 Pin

| Function          | Pin   | Position |
| ----------------- | ----- | -------- |
| Reference Voltage | VREFH | J108[2]  |
| ADC3.INA2         | PE29  | J10[5]   |

- headphone interface

| Function        | Position |
| --------------- | -------- |
| 3.5mm headphone | J13      |

- audio input interface

| Function   | Position |
| ---------- | -------- |
| microphone | P3       |

- DAO interface

| Function | Position |
| -------- | -------- |
| DAO-SPK  | J12      |
