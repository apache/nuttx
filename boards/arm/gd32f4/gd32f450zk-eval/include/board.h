/****************************************************************************
 * boards/arm/gd32f4/gd32f450zk-eval/include/board.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __BOARDS_ARM_GD32F450ZK_EVAL_INCLUDE_BOARD_H
#define __BOARDS_ARM_GD32F450ZK_EVAL_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

#define GD32_BOARD_SYSCLK_PLL_HXTAL

/* Do not include GD32F4 header files here */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The GD32F450ZK-EVAL board features a single 25MHz crystal.
 *
 * This is the default configuration:
 *   System clock source           : PLL (HXTAL)
 *   SYSCLK(Hz)                    : 200000000    Determined by PLL config
 *   HCLK(Hz)                      : 200000000    (GD32_SYSCLK_FREQUENCY)
 *   AHB Prescaler                 : 1            (GD32_RCU_CFG0_AHB_PSC)
 *   APB2 Prescaler                : 2            (GD32_RCU_CFG0_APB2_PSC)
 *   APB1 Prescaler                : 4            (GD32_RCU_CFG0_APB1_PSC)
 *   HXTAL value(Hz)               : 25000000     (GD32_BOARD_XTAL)
 *   PLLM                          : 25           (GD32_PLL_PLLM)
 *   PLLN                          : 400          (RCU_PLL_PLLN)
 *   PLLP                          : 2            (GD32_PLL_PLLP)
 *   PLLQ                          : 7            (GD32_PLL_PLLQ)
 */

/* IRC16M - 16 MHz RC factory-trimmed
 * IRC32K - 32 KHz RC
 * HXTAL  - On-board crystal frequency is 25MHz
 * LXTAL  - 32.768 kHz
 */

#ifndef CONFIG_GD32F4_BOARD_HXTAL_VALUE
#  define GD32_BOARD_HXTAL       25000000ul
#else
#  define GD32_BOARD_HXTAL       CONFIG_GD32F4_BOARD_HXTAL_VALUE
#endif

#define GD32_IRC16M_VALUE      16000000ul
#define GD32_IRC32K_VALUE      32000u
#define GD32_HXTAL_VALUE       GD32_BOARD_HXTAL
#define GD32_LXTAL_VALUE       32768u

#if defined(CONFIG_GD32F4_200MHZ)

/* Main PLL Configuration.
 *
 * PLL source is HXTAL
 * PLL_VCO = (GD32_HXTAL_VALUE / PLLM) * PLLN
 *         = (25,000,000 / 25) * 400
 *         = 400,000,000
 * SYSCLK  = PLL_VCO / PLLP
 *         = 400,000,000 / 2 = 168,000,000
 * USB, SDIO and RNG Clock
 *         =  PLL_VCO / PLLQ
 *         = 48,000,000
 */

#define GD32_PLL_PLLPSC            RCU_PLL_PLLPSC(25)
#define GD32_PLL_PLLN              RCU_PLL_PLLN(400)
#define GD32_PLL_PLLP              RCU_PLL_PLLP(2)
#define GD32_PLL_PLLQ              RCU_PLL_PLLQ(7)

#define GD32_SYSCLK_FREQUENCY      200000000ul

#elif defined(CONFIG_GD32F4_168MHZ)

/* Main PLL Configuration.
 *
 * PLL source is HXTAL
 * PLL_VCO = (GD32_HXTAL_VALUE / PLLM) * PLLN
 *         = (25,000,000 / 25) * 336
 *         = 336,000,000
 * SYSCLK  = PLL_VCO / PLLP
 *         = 336,000,000 / 2 = 168,000,000
 * USB, SDIO and RNG Clock
 *         =  PLL_VCO / PLLQ
 *         = 48,000,000
 */

#define GD32_PLL_PLLPSC            RCU_PLL_PLLPSC(25)
#define GD32_PLL_PLLN              RCU_PLL_PLLN(336)
#define GD32_PLL_PLLP              RCU_PLL_PLLP(2)
#define GD32_PLL_PLLQ              RCU_PLL_PLLQ(7)

#define GD32_SYSCLK_FREQUENCY      168000000ul

#endif

/* AHB clock (HCLK) is SYSCLK */

#define GD32_RCU_CFG0_AHB_PSC      RCU_CFG0_AHBPSC_CKSYS_DIV1  /* HCLK  = SYSCLK / 1 */
#define GD32_HCLK_FREQUENCY        GD32_SYSCLK_FREQUENCY

/* APB2 clock (PCLK2) is HCLK/2 */

#define GD32_RCU_CFG0_APB2_PSC     RCU_CFG0_APB2PSC_CKAHB_DIV2 /* PCLK2 = HCLK / 2 */
#define GD32_PCLK2_FREQUENCY       (GD32_HCLK_FREQUENCY/2)

/* APB1 clock (PCLK1) is HCLK/4 */

#define GD32_RCU_CFG0_APB1_PSC     RCU_CFG0_APB1PSC_CKAHB_DIV4 /* PCLK1 = HCLK / 4 */
#define GD32_PCLK1_FREQUENCY       (GD32_HCLK_FREQUENCY / 4)

/* Timers driven from APB1 will be twice PCLK1 */

#define GD32_APB1_TIMER2_CLKIN   (2*GD32_PCLK1_FREQUENCY)
#define GD32_APB1_TIMER3_CLKIN   (2*GD32_PCLK1_FREQUENCY)
#define GD32_APB1_TIMER4_CLKIN   (2*GD32_PCLK1_FREQUENCY)
#define GD32_APB1_TIMER5_CLKIN   (2*GD32_PCLK1_FREQUENCY)
#define GD32_APB1_TIMER6_CLKIN   (2*GD32_PCLK1_FREQUENCY)
#define GD32_APB1_TIMER7_CLKIN   (2*GD32_PCLK1_FREQUENCY)
#define GD32_APB1_TIMER12_CLKIN  (2*GD32_PCLK1_FREQUENCY)
#define GD32_APB1_TIMER13_CLKIN  (2*GD32_PCLK1_FREQUENCY)
#define GD32_APB1_TIMER14_CLKIN  (2*GD32_PCLK1_FREQUENCY)

/* Timers driven from APB2 will be twice PCLK2 */

#define GD32_APB2_TIMER1_CLKIN   (2*GD32_PCLK2_FREQUENCY)
#define GD32_APB2_TIMER8_CLKIN   (2*GD32_PCLK2_FREQUENCY)
#define GD32_APB2_TIMER9_CLKIN   (2*GD32_PCLK2_FREQUENCY)
#define GD32_APB2_TIMER10_CLKIN  (2*GD32_PCLK2_FREQUENCY)
#define GD32_APB2_TIMER11_CLKIN  (2*GD32_PCLK2_FREQUENCY)

/* Timer Frequencies, if APBx is set to 1, frequency is same to APBx
 * otherwise frequency is 2xAPBx.
 * Note: TIMER1,8 are on APB2, others on APB1
 */

#define BOARD_TIMER1_FREQUENCY    GD32_HCLK_FREQUENCY
#define BOARD_TIMER2_FREQUENCY    (GD32_HCLK_FREQUENCY/2)
#define BOARD_TIMER3_FREQUENCY    (GD32_HCLK_FREQUENCY/2)
#define BOARD_TIMER4_FREQUENCY    (GD32_HCLK_FREQUENCY/2)
#define BOARD_TIMER5_FREQUENCY    (GD32_HCLK_FREQUENCY/2)
#define BOARD_TIMER6_FREQUENCY    (GD32_HCLK_FREQUENCY/2)
#define BOARD_TIMER7_FREQUENCY    (GD32_HCLK_FREQUENCY/2)
#define BOARD_TIMER8_FREQUENCY    GD32_HCLK_FREQUENCY

/* LED definitions **********************************************************/

/* The GD32F450ZK_EVAL board has board has three LEDs. The LED1, LED2 and
 * LED3 are controlled by GPIO. LED1 is connected to PD4, LED2 is connected
 * to PD5, LED3 is connected to PG3
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs
 * in any way.
 * The following definitions are used to access individual LEDs.
 */

/* LED index values */

typedef enum
{
    BOARD_LED1 = 0,
    BOARD_LED2 = 1,
    BOARD_LED3 = 2,
    BOARD_LEDS
} led_typedef_enum;

/* LED bits */

#define BOARD_LED1_BIT    (1 << BOARD_LED1)
#define BOARD_LED2_BIT    (1 << BOARD_LED2)
#define BOARD_LED3_BIT    (1 << BOARD_LED3)

/* If CONFIG_ARCH_LEDS is defined, the usage by the board port is defined in
 * include/board.h and src/gd32f4xx_autoleds.c. The LEDs are used to encode
 * OS-related events as follows:
 *
 *
 *   SYMBOL                     Meaning                      LED state
 *                                                        LED1  LED2  LED3
 *   ----------------------  --------------------------  ------ ------ ---
 */

#define LED_STARTED        0 /* NuttX has been started   OFF    OFF   OFF  */
#define LED_HEAPALLOCATE   1 /* Heap has been allocated  ON     OFF   OFF  */
#define LED_IRQSENABLED    2 /* Interrupts enabled       OFF    ON    OFF  */
#define LED_STACKCREATED   3 /* Idle stack created       OFF    OFF   ON   */
#define LED_INIRQ          4 /* In an interrupt          ON     ON    OFF  */
#define LED_SIGNAL         5 /* In a signal handler      ON     OFF   ON   */
#define LED_ASSERTION      6 /* An assertion failed      OFF    ON    ON   */
#define LED_PANIC          7 /* The system has crashed   FLASH  ON    ON   */
#define LED_IDLE           8 /* MCU is is sleep mode     OFF    FLASH OFF  */

/* Button definitions *******************************************************/

/* The GD32F450Z Eval supports three user buttons:  Wakeup, Tamper and
 * User key, they are connected to GPIO PA0, PC13, PB14.
 * A low value will be sensed when the button is depressed.
 */

typedef enum
{
    BUTTON_WAKEUP = 0,
    BUTTON_TAMPER = 1,
    BUTTON_USER = 2,
    NUM_BUTTONS
} key_typedef_enum;

#define BUTTON_WAKEUP_BIT    (1 << BUTTON_WAKEUP)
#define BUTTON_TAMPER_BIT    (1 << BUTTON_TAMPER)
#define BUTTON_USER_BIT      (1 << BUTTON_USER)

/* Alternate function pin selections ****************************************/

#if defined(CONFIG_GD32F450ZK_EVAL_CONSOLE_BOARD)

/* USART0:
 *
 * These configurations assume that you are using a standard RS-232
 * shield with the serial interface with RX on PA10 and TX on PA10:
 *
 *   -------- ---------------
 *           GD32F450ZK-EVAL
 *   -- ----- --------- -----
 *   RX    USART0_RX PA10
 *   TX    USART0_TX PA9
 *   -- ----- --------- -----
 */

#  define GPIO_USART0_RX GPIO_USART0_RX_1
#  define GPIO_USART0_TX GPIO_USART0_TX_1

#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_USART0_IFLOWCONTROL)
#    define GPIO_USART0_RTS GPIO_USART0_RTS_1
#  endif
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_USART0_OFLOWCONTROL)
#    define GPIO_USART0_CTS GPIO_USART0_CTS_1
#  endif
#endif

#if CONFIG_GD32F4_USART0_TXDMA
#  define DMA_CHANNEL_USART0_TX    DMA_REQ_USART0_TX
#endif
#if CONFIG_GD32F4_USART0_RXDMA
#  define DMA_CHANNEL_USART0_RX    DMA_REQ_USART0_RX_1
#endif

#if defined(CONFIG_GD32F4_USART_RXDMA) || defined(CONFIG_GD32F4_USART_TXDMA)
#  define USART_DMA_INTEN          (DMA_CHXCTL_SDEIE | DMA_CHXCTL_TAEIE | DMA_CHXCTL_FTFIE)
#endif

/* USART3:
 * Use  USART3 and the USB virtual COM port
 */

#if defined(GD32F450ZK_EVAL_CONSOLE_VIRTUAL)
#  define GPIO_USART3_RX GPIO_USART3_RX_3
#  define GPIO_USART3_TX GPIO_USART3_TX_3
#endif

/* I2C0 gpios:
 *
 *   PB6      I2C0_SCL
 *   PB7      I2C0_SDA
 *
 */

#define GPIO_I2C0_SCL   GPIO_I2C0_SCL_1
#define GPIO_I2C0_SDA   GPIO_I2C0_SDA_1

/* SPI flash
 *
 *  PG12  SPI5_MISO
 *  PG14  SPI5_MOSI
 *  PG13  SPI5_SCK
 *
 *  PG9   SPI5_CS
 *
 */

#define GPIO_SPI5_CSPIN (GPIO_CFG_PORT_G | GPIO_PIN9_OUTPUT)

#define GPIO_SPI5_MISO_PIN  ((GPIO_SPI5_MISO & ~GPIO_CFG_SPEED_MASK) | GPIO_CFG_SPEED_25MHZ)
#define GPIO_SPI5_MOSI_PIN  ((GPIO_SPI5_MOSI & ~GPIO_CFG_SPEED_MASK) | GPIO_CFG_SPEED_25MHZ)
#define GPIO_SPI5_SCK_PIN   ((GPIO_SPI5_SCK & ~GPIO_CFG_SPEED_MASK) | GPIO_CFG_SPEED_25MHZ)

#define GPIO_SPI5_IO2_PIN  ((GPIO_SPI5_IO2 & ~GPIO_CFG_SPEED_MASK) | GPIO_CFG_SPEED_25MHZ)
#define GPIO_SPI5_IO3_PIN  ((GPIO_SPI5_IO3 & ~GPIO_CFG_SPEED_MASK) | GPIO_CFG_SPEED_25MHZ)

#ifdef CONFIG_GD32F4_SPI0
#  define GPIO_SPI0_CSPIN     (GPIO_CFG_PORT_B | GPIO_PIN9_OUTPUT)
#  define GPIO_SPI0_MISO_PIN  ((GPIO_SPI0_MISO_1 & ~GPIO_CFG_SPEED_MASK) | GPIO_CFG_SPEED_25MHZ)
#  define GPIO_SPI0_MOSI_PIN  ((GPIO_SPI0_MOSI_1 & ~GPIO_CFG_SPEED_MASK) | GPIO_CFG_SPEED_25MHZ)
#  define GPIO_SPI0_SCK_PIN   ((GPIO_SPI0_SCK_1 & ~GPIO_CFG_SPEED_MASK) | GPIO_CFG_SPEED_25MHZ)
#endif

#ifdef CONFIG_GD32F4_SPI0_DMA
#  define DMA_CHANNEL_SPI0_TX      DMA_REQ_SPI0_TX_1
#  define DMA_CHANNEL_SPI0_RX      DMA_REQ_SPI0_RX_1
#endif

#ifdef CONFIG_GD32F4_SPI_DMA
#  define SPI_DMA_INTEN            (DMA_CHXCTL_SDEIE | DMA_CHXCTL_TAEIE | DMA_CHXCTL_FTFIE)
#endif

/* The GD32 F4 connects to a DP83848 PHY using these pins:
 *
 *   GD32F450Z Eval BOARD      DP83848
 *   GPIO      SIGNAL          PIN NAME
 *   -------- ------------ -------------
 *   PB11      RMII_TX_EN      TXEN
 *   PB12      RMII_TXD0       TXD0
 *   PB13      RMII_TXD1       TXD1
 *   PC4       RMII_RXD0       RXD_0/PHYAD1
 *   PC5       RMII_RXD1       RXD_1/PHYAD2
 *   PA7       RMII_CRS_DV     RX_DV/MII_MODE
 *   PC1       RMII_MDC        MDC
 *   PA2       RMII_MDIO       MDIO
 *   NRST      NRST            RESET_N
 *   PA1       RMII_REF_CLK    X1
 *   PB15      RMII_INT        PWR_DOWN/INT
 *
 * The PHY address is 1.
 */

#define GPIO_ENET_RMII_TX_EN   GPIO_ENET_RMII_TX_EN_1
#define GPIO_ENET_RMII_TXD0    GPIO_ENET_RMII_TXD0_1
#define GPIO_ENET_RMII_TXD1    GPIO_ENET_RMII_TXD1_1

#ifdef CONFIG_GD32F4_ENET_PTP
  /* Enable pulse-per-second (PPS) output signal */

#  define GPIO_ENET_PPS_OUT    GPIO_ENET_PPS_OUT_1
#endif

#endif /* __BOARDS_ARM_GD32F450ZK_EVAL_INCLUDE_BOARD_H */
