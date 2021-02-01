/****************************************************************************
 * include/nuttx/analog/pga11x.h
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

#ifndef __INCLUDE_NUTTX_ANALOG_PGA11X_H
#define __INCLUDE_NUTTX_ANALOG_PGA11X_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/spi/spi.h>

#if defined(CONFIG_ADC) && defined(CONFIG_ADC_PGA11X)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Prerequisites:
 *  CONFIG_ADC=y is needed to enable support for analog input devices
 *
 * CONFIG_ADC_PGA11X
 *   Enables support for the PGA11X driver (Needs CONFIG_ADC)
 * CONFIG_PGA11X_SPIFREQUENCY
 *   SPI frequency.  Default 1MHz
 * CONFIG_PGA11X_DAISYCHAIN
 *   Use two PGA116/7's in Daisy Chain commands.
 * CONFIG_PGA11X_SPIMODE
 *   SPI Mode.  The specification says that the device operates in Mode 0 or
 *   Mode 3.  But sometimes you need to tinker with this to get things to
 *   work correctly. Default:  Mode 0
 * CONFIG_PGA11X_MULTIPLE
 *   Can be defined to support multiple PGA11X devices on board.  Each
 *   device will require a customized SPI interface to distinguish them
 *   When SPI_SELECT is called with devid=SPIDEV_MUX(n).
 *
 * Other settings that effect the driver:
 *   CONFIG_DEBUG_SPI_ERR/WARN/INFO -- This will enable debug output from
 *   the PGA117 driver.
 */

#ifndef CONFIG_PGA11X_SPIFREQUENCY
#  define CONFIG_PGA11X_SPIFREQUENCY 1000000
#endif

#ifndef CONFIG_PGA11X_SPIMODE
#  define CONFIG_PGA11X_SPIMODE SPIDEV_MODE0
#endif

/* PGA11x Commands **********************************************************/

/* Write command Gain Selection Bits (PGA112/PGA113)
 *
 * the PGA112 and PGA116 provide binary gain selections (1, 2, 4, 8, 16, 32,
 * 64, 128); the PGA113 and PGA117 provide scope gain selections (1, 2, 5,
 * 10, 20, 50, 100, 200).
 */

#define PGA11X_GAIN_1          (0) /* Gain=1:   Scope Gain=1 */
#define PGA11X_GAIN_2          (1) /* Gain=2:   Scope Gain=2 */
#define PGA11X_GAIN_4          (2) /* Gain=4:   Scope Gain=5 */
#define PGA11X_GAIN_5          (2) /* Gain=4:   Scope Gain=5 */
#define PGA11X_GAIN_8          (3) /* Gain=8:   Scope Gain=10 */
#define PGA11X_GAIN_10         (3) /* Gain=8:   Scope Gain=10 */
#define PGA11X_GAIN_16         (4) /* Gain=16:  Scope Gain=20 */
#define PGA11X_GAIN_20         (4) /* Gain=16:  Scope Gain=20 */
#define PGA11X_GAIN_32         (5) /* Gain=32:  Scope Gain=50 */
#define PGA11X_GAIN_50         (5) /* Gain=32:  Scope Gain=50 */
#define PGA11X_GAIN_64         (6) /* Gain=64:  Scope Gain=100 */
#define PGA11X_GAIN_100        (6) /* Gain=64:  Scope Gain=100 */
#define PGA11X_GAIN_128        (7) /* Gain=128: Scope Gain=200 */
#define PGA11X_GAIN_200        (7) /* Gain=128: Scope Gain=200 */

/* Write command Mux Channel Selection Bits
 *
 * The PGA112/PGA113 have a two-channel input MUX; the PGA116/PGA117 have a
 * 10-channel input MUX.
 */

#define PGA11X_CHAN_VCAL       (0)  /* VCAL/CH0 */
#define PGA11X_CHAN_CH0        (0)  /* VCAL/CH0 */
#define PGA11X_CHAN_CH1        (1)  /* CH1 */
#define PGA11X_CHAN_CH2        (2)  /* CH2 (PGA116/PGA117 only) */
#define PGA11X_CHAN_CH3        (3)  /* CH3 (PGA116/PGA117 only) */
#define PGA11X_CHAN_CH4        (4)  /* CH4 (PGA116/PGA117 only) */
#define PGA11X_CHAN_CH5        (5)  /* CH5 (PGA116/PGA117 only) */
#define PGA11X_CHAN_CH6        (6)  /* CH6 (PGA116/PGA117 only) */
#define PGA11X_CHAN_CH7        (7)  /* CH7 (PGA116/PGA117 only) */
#define PGA11X_CHAN_CH8        (8)  /* CH8 (PGA116/PGA117 only) */
#define PGA11X_CHAN_CH9        (9)  /* CH9 (PGA116/PGA117 only) */
#define PGA11X_CHAN_CAL1       (12) /* CAL1: connects to GND */
#define PGA11X_CHAN_CAL2       (13) /* CAL2: connects to 0.9VCAL */
#define PGA11X_CHAN_CAL3       (14) /* CAL3: connects to 0.1VCAL */
#define PGA11X_CHAN_CAL4       (15) /* CAL4: connects to VREF */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Handle used to reference the particular PGA11X instance */

typedef FAR void *PGA11X_HANDLE;

/* Settings for one device in a daisy-chain */

#ifdef CONFIG_PGA11X_DAISYCHAIN
struct pga11x_usettings_s
{
  uint8_t channel;    /* See PGA11X_CHAN_* definitions */
  uint8_t gain;       /* See PGA11X_GAIN_* definitions */
};
#endif

/* These structures are used to encode gain and channel settings.  This
 * includes both devices in the case of a daisy-chained configuration.
 * NOTE: This logic is currently limited to only 2 devices in the daisy-
 * chain.
 */

struct pga11x_settings_s
{
#ifndef CONFIG_PGA11X_DAISYCHAIN
  uint8_t channel;    /* See PGA11X_CHAN_* definitions */
  uint8_t gain;       /* See PGA11X_GAIN_* definitions */
#else
  struct
  {
    uint8_t channel;  /* See PGA11X_CHAN_* definitions */
    uint8_t gain;     /* See PGA11X_GAIN_* definitions */
  } u1;

  struct
  {
    uint8_t channel;  /* See PGA11X_CHAN_* definitions */
    uint8_t gain;     /* See PGA11X_GAIN_* definitions */
  } u2;
#endif
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: pga11x_initialize
 *
 * Description:
 *   Initialize the PGA117 amplifier/multiplexer(s).
 *
 * Input Parameters:
 *   spi - An SPI "bottom half" device driver instance
 *
 * Returned Value:
 *   On success, a non-NULL opaque handle is returned; a NULL is returned
 *   on any failure.  This handle may be used with the other PGA117 interface
 *   functions to control the multiplexer
 *
 ****************************************************************************/

PGA11X_HANDLE pga11x_initialize(FAR struct spi_dev_s *spi);

/****************************************************************************
 * Name: pga11x_select
 *
 * Description:
 *   Select an input channel and gain for all PGA11xs.
 *
 *   If CONFIG_PGA11X_DAISYCHAIN is defined, then pga11x_select() configures
 *   both chips in the daisy-chain.  pga11x_uselect() is provided to support
 *   configuring the parts in the daisychain independently.
 *
 * Input Parameters:
 *   spi      - An SPI "bottom half" device driver instance
 *   settings - New channel and gain settings
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int pga11x_select(PGA11X_HANDLE handle,
                  FAR const struct pga11x_settings_s *settings);

/****************************************************************************
 * Name: pga11x_uselect
 *
 * Description:
 *   Select an input channel and gain for one PGA11x.
 *
 *   If CONFIG_PGA11X_DAISYCHAIN is defined, then pga11x_uselect() configures
 *   one chips in the daisy-chain.
 *
 * Input Parameters:
 *   spi      - An SPI "bottom half" device driver instance
 *   pos      - Position of the chip in the daisy chain (0 or 1)
 *   settings - New channel and gain settings
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_PGA11X_DAISYCHAIN
int pga11x_uselect(PGA11X_HANDLE handle, int pos,
                   FAR const struct pga11x_usettings_s *settings);
#endif

/****************************************************************************
 * Name: pga11x_read
 *
 * Description:
 *   Read from all PGA117 amplifier/multiplexers.
 *
 *   If CONFIG_PGA11X_DAISYCHAIN is defined, then pga11x_read() reads from
 *   both chips in the daisy-chain.  pga11x_uread() is provided to support
 *   accessing the parts independently.
 *
 * Input Parameters:
 *   spi      - An SPI "bottom half" device driver instance
 *   settings - Returned channel and gain settings
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int pga11x_read(PGA11X_HANDLE handle,
                FAR struct pga11x_settings_s *settings);

/****************************************************************************
 * Name: pga11x_uread
 *
 * Description:
 *   Read from one PGA117 amplifier/multiplexer.
 *
 *   If CONFIG_PGA11X_DAISYCHAIN is defined, then pga11x_read() reads
 *   the parts independently.
 *
 * Input Parameters:
 *   spi      - An SPI "bottom half" device driver instance
 *   pos      - Position of the chip in the daisy chain (0 or 1)
 *   settings - Returned channel and gain settings
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_PGA11X_DAISYCHAIN
int pga11x_uread(PGA11X_HANDLE handle, int pos,
                 FAR struct pga11x_usettings_s *settings);
#endif

/****************************************************************************
 * Name: pga11x_shutdown
 *
 * Description:
 *   Put all PGA11x;'s in shutdown down mode.
 *
 *   If CONFIG_PGA11X_DAISYCHAIN is defined, then pga11x_shutdown() controls
 *   both chips in the daisy-chain.  pga11x_ushutdown() is provided to
 *   control the parts independently.
 *
 * Input Parameters:
 *   spi - An SPI "bottom half" device driver instance
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int pga11x_shutdown(PGA11X_HANDLE handle);

/****************************************************************************
 * Name: pga11x_ushutdown
 *
 * Description:
 *   Put one PGA11x in shutdown down mode.
 *
 *   If CONFIG_PGA11X_DAISYCHAIN is defined, then pga11x_ushutdown() is
 *   provided to shutdown the parts independently.
 *
 * Input Parameters:
 *   spi - An SPI "bottom half" device driver instance
 *   pos      - Position of the chip in the daisy chain (0 or 1)
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_PGA11X_DAISYCHAIN
int pga11x_ushutdown(PGA11X_HANDLE handle, int pos);
#endif

/****************************************************************************
 * Name: pga11x_enable
 *
 * Description:
 *   Take all PGA11x's out of shutdown down mode.
 *
 *   If CONFIG_PGA11X_DAISYCHAIN is defined, then pga11x_enable() controls
 *   both chips in the daisy-chain.  pga11x_uenable() is provided to
 *   control the parts independently.
 *
 * Input Parameters:
 *   spi - An SPI "bottom half" device driver instance
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int pga11x_enable(PGA11X_HANDLE handle);

/****************************************************************************
 * Name: pga11x_uenable
 *
 * Description:
 *   Take one PGA11x out of shutdown down mode.
 *
 *   If CONFIG_PGA11X_DAISYCHAIN is defined, then pga11x_uenable() is
 *   provided to enable the parts independently.
 *
 * Input Parameters:
 *   spi - An SPI "bottom half" device driver instance
 *   pos      - Position of the chip in the daisy chain (0 or 1)
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_PGA11X_DAISYCHAIN
int pga11x_uenable(PGA11X_HANDLE handle, int pos);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_ADC && CONFIG_ADC_PGA11X */
#endif /* __INCLUDE_NUTTX_ANALOG_PGA11X_H */
