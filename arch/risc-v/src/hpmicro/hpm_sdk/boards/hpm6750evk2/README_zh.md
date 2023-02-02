# HPM6750EVK2开发板

## 概述

HPM6750是一款主频达816MHz的双核微控制器。该芯片拥有最大2M字节的连续片上RAM，并集成了丰富的存储接口，如SDRAM，Quad SPI NOR flash， SD/eMMC卡。同时它也提供多种音视频接口包括LCD显示，像素DMA，摄像头以及I2S音频接口。

 ![hpm6750evk](../../doc/images/boards/hpm6750evk2/hpm6750evk2.png "hpm6750evk2")

## 板上硬件资源

- HPM6750IVM 微控制器 (主频816Mhz, 2MB片上内存)
- 板载存储
  - 256Mb SDRAM
  - 128Mb Quad SPI NOR Flash
- 显示/摄像头
  - LCD接口
  - 摄像头(DVP)接口
- 以太网
  - 1000 Mbits PHY
  - 100 Mbits PHY
- USB
  - USB type C (USB 2.0 OTG) connector x3
- 音频
  - Line in
  - Mic
  - Speaker
  - DAO
- 其他
  - TF卡槽
  - RGB LED
  - CAN
- 扩展口
  - 电机控制

## 拨码开关 S1

- Bit 1，2控制启动模式

| Bit[2:1] | 功能描述                |
| -------- | ----------------------- |
| OFF, OFF | Quad SPI NOR flash 启动 |
| OFF, ON  | 串行启动                |
| ON, OFF  | 在系统编程              |

(lab_hpm6750_evk2_board)=

## 按键

(**lab_hpm6750_evk2_board_buttons**)=

| 名称         | 功能                                  |
| ------------ | ------------------------------------- |
| PBUTN (SW1)  | 电源按键, TinyUF2 Boot按键, GPIO 按键 |
| WBUTN (SW2)  | WAKE UP 按键                          |
| RESETN (SW3) | Reset 按键                            |

## 插件

- `J108` 选择ADC/DAC参考电压| 连接 | 描述        |
  | ---- | ----------- |
  | 1, 2 | 3.3V        |
  | 2, 3 | 高精度 3.3V |

## 功能选择电阻

- GigE POP `R177-R182`| 状态 | 描述 |
  | ---- | ---- |
  | 焊接 | 网络 |
  | 断开 | 电机 |

## 引脚描述

- UART0串口引脚：

 UART0的引脚引出至三个位置：

| 功能     | 引脚 | 位置1   | 位置2      | 位置3     |
| -------- | ---- | ------- | ---------- | --------- |
| UART0.TX | PY06 | J109[1] | JTAG P4[7] | USB2UART0 |
| UART0.RX | PY07 | J109[3] | JTAG P4[9] | USB2UART0 |

注意：使用UART0时，请确保只有一处连接，避免多处连接造成功能不正常。例如，JTAG口连接了UART0，从USB2UART0接口发送数据给UART0就会异常。

- SPI引脚：

| 功能      | 引脚 | 位置    |
| --------- | ---- | ------- |
| SPI2.CSN  | PE31 | J11[7]  |
| SPI2.SCLK | PE27 | J11[8]  |
| SPI2.MISO | PE28 | J11[9]  |
| SPI2.MOSI | PE30 | J11[10] |

- I2C引脚：

| 功能     | 引脚 | 位置   |
| -------- | ---- | ------ |
| I2C0.SCL | PZ11 | J11[3] |
| I2C0.SDA | PZ10 | J11[4] |

- CORE1调试串口引脚：

| 功能       | 引脚 | 位置   |
| ---------- | ---- | ------ |
| UART13.TXD | PZ09 | J11[5] |
| UART13.RXD | PZ08 | J11[6] |

- PWM引脚:

| 功能      | 引脚 | 位置    |
| --------- | ---- | ------- |
| PWM2.P[0] | PD31 | J10[14] |
| PWM2.P[1] | PD30 | J10[13] |

- ACMP引脚

| 功能       | 引脚 | 位置   |
| ---------- | ---- | ------ |
| CMP.INN6   | PE21 | J10[8] |
| CMP.COMP_1 | PE25 | J10[6] |

- GPTMR引脚

| 功能          | 引脚 | 位置   |
| ------------- | ---- | ------ |
| GPTMR4.CAPT_1 | PE25 | J10[6] |
| GPTMR3.COMP_1 | PE24 | J10[7] |

- ADC12引脚

| 功能            | 引脚  | 位置    |
| --------------- | ----- | ------- |
| ADC12参考电压   | VREFH | J108[2] |
| ADC0/1/2.VINP11 | PE25  | J10[6]  |
| ADC0/1/2.VINP10 | PE24  | J10[7]  |
| ADC0/1/2.VINP7  | PE21  | J10[8]  |

- ADC16引脚

| 功能          | 引脚  | 位置    |
| ------------- | ----- | ------- |
| ADC16参考电压 | VREFH | J108[2] |
| ADC3.INA2     | PE29  | J10[5]  |

- 耳机接口

| 功能          | 位置 |
| ------------- | ---- |
| 3.5mm耳机接口 | J13  |

- 音频输入接口

| 功能   | 位置 |
| ------ | ---- |
| 麦克风 | P3   |

- DAO接口

| 功能     | 位置 |
| -------- | ---- |
| 喇叭接口 | J12  |
