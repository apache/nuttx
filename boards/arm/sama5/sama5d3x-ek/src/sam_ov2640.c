/****************************************************************************
 * boards/arm/sama5/sama5d3x-ek/src/sam_ov2640.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdlib.h>
#include <debug.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/video/fb.h>
#include <nuttx/video/ov2640.h>

#include "arm_internal.h"
#include "sam_periphclks.h"
#include "sam_lcd.h"
#include "sam_pck.h"
#include "sam_twi.h"
#include "sam_pio.h"
#include "hardware/sam_pinmap.h"

#include "sama5d3x-ek.h"

#ifdef HAVE_CAMERA

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Typical OV2640 XVCLK is 24MHz */

#define OV2640_FREQUENCY 24000000

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ov2640_lcd_initialize
 ****************************************************************************/

static inline struct fb_vtable_s *ov2640_lcd_initialize(void)
{
  struct fb_vtable_s *vplane;
  int ret;

  /* Initialize the frame buffer device */

  ret = up_fbinitialize(0);
  if (ret < 0)
    {
      gerr("ERROR: up_fbinitialize failed: %d\n", -ret);
      return NULL;
    }

  vplane = up_fbgetvplane(0, 0);
  if (!vplane)
    {
      gerr("ERROR: up_fbgetvplane failed\n");
    }

  return vplane;
}

/****************************************************************************
 * Name: ov2640_camera_initialize
 *
 * Description:
 *   Initialize the OV2640 camera in the correct mode of operation
 *
 * OV2640 Camera Interface
 *
 *   SAMA5D3x PIN             SAMA5D3x-EK    OV2640
 *   PIO  PER SIGNAL        ISI Socket J11
 *   ---- --- ------------- --- ------------ --------------------------------
 *   ---                     1  VDDISI       ---
 *   ---                     2  GND          ---
 *   ---                     3  VDDISI       ---
 *   ---                     4  GND          ---
 *   PE28  ?  ?              5  ZB_SLPTR     ???
 *   PE29  ?  ?              6  ZB_RST       C6 RESETB Reset mode (?)
 *   PC27  B  TWI1_CK        7  TWCK1        C2 SIO_C SCCB
 *                                           serial interface clock input
 *   PC26  B  TWI1_D         8  TWD1         C1 SIO_D SCCB
 *                                           serial interface data I/O
 *   ---                     9  GND          ---
 *   PD31  B  PCK1 (ISI_MCK) 10 ISI_MCK      C4 XVCLK
 *                                           System clock input (?)
 *   ---                     11 GND          ---
 *   PA30  C  ISI_VSYNC      12 ISI_VSYNC    D2 VSYNC
 *                                           Vertical synchronization
 *   ---                     13 GND          ---
 *   PA31  C  ISI_HSYNC      14 ISI_HSYNC    C3 HREF
 *                                           Horizontal reference output (?)
 *   ---                     15 GND          ---
 *   PC30  C  ISI_PCK        16 ISI_PCK      E3 PCLK Pixel clock output
 *   ---                     17 GND          ---
 *   PA16  C  ISI_D0         18 ISI_D0       E2 Y0 Video port output bit[0]
 *   PA17  C  ISI_D1         19 ISI_D1       E1 Y1 Video port output bit[1]
 *   PA18  C  ISI_D2         20 ISI_D2       F3 Y2 Video port output bit[2]
 *   PA19  C  ISI_D3         21 ISI_D3       G3 Y3 Video port output bit[3]
 *   PA20  C  ISI_D4         22 ISI_D4       F4 Y4 Video port output bit[4]
 *   PA21  C  ISI_D5         23 ISI_D5       G4 Y5 Video port output bit[5]
 *   PA22  C  ISI_D6         24 ISI_D6       E5 Y6 Video port output bit[6]
 *   PA23  C  ISI_D7         25 ISI_D7       G5 Y7 Video port output bit[7]
 *   PC29  C  ISI_D8         26 ISI_D8       F5 Y8 Video port output bit[8]
 *   PC28  C  ISI_D9         27 ISI_D9       G6 Y9 Video port output bit[9]
 *   PC27  C  ISI_D10        28 ISI_D10      ---
 *   PC26  C  ISI_D11        29 ISI_D11      ---
 *   ---                     30 GND          ---
 *
 *   ???                     ??              A2 EXPST_B
 *                                           Snapshot exposure start trigger
 *   ???                     ??              A6 STROBE  Flash control output
 *   ???                     ??              B2 FREX   Snapshot trigger
 *   ???                     ??              B6 PWDN   Power-down mode enable
 *
 ****************************************************************************/

static inline int ov2640_camera_initialize(void)
{
  struct i2c_master_s *i2c;
  uint32_t actual;
  int ret;

  /* Get the I2C driver that interfaces with the camers (OV2640_BUS) */

  i2c = sam_i2cbus_initialize(OV2640_BUS);
  if (!i2c)
    {
      gerr("ERROR: Failed to initialize TWI%d\n", OV2640_BUS);
      return EXIT_FAILURE;
    }

  /* Enable clocking to the ISI peripheral */

  sam_isi_enableclk();

  /* Configure OV2640 pins
   *
   * ISI:
   * - HSYNC, VSYNC, PCK
   * - 8 data bits for 8-bit color
   * PCK
   * - PCK1 provides OV2640 system clock
   */

  sam_configpio(PIO_ISI_HSYNC);
  sam_configpio(PIO_ISI_VSYNC);
  sam_configpio(PIO_ISI_PCK);

  sam_configpio(PIO_ISI_D0);
  sam_configpio(PIO_ISI_D1);
  sam_configpio(PIO_ISI_D2);
  sam_configpio(PIO_ISI_D3);
  sam_configpio(PIO_ISI_D4);
  sam_configpio(PIO_ISI_D5);
  sam_configpio(PIO_ISI_D6);
  sam_configpio(PIO_ISI_D7);

  sam_configpio(PIO_PMC_PCK1);

  /* Configure and enable the PCK1 output */

  actual = sam_pck_configure(PCK1, PCKSRC_MCK, OV2640_FREQUENCY);
  ginfo("Desired PCK1 frequency: %ld Actual: %ld\n",
        (long)OV2640_FREQUENCY, (long)actual);
  UNUSED(actual);

  sam_pck_enable(PCK1, true);

  /* Configure the ISI peripheral */

#warning Missing Logic

  /* Initialize the OV2640 camera */

  ret = ov2640_initialize(i2c);
  if (ret < 0)
    {
      gerr("ERROR: Failed to initialize the OV2640: %d\n", ret);
      return EXIT_FAILURE;
    }

  return EXIT_FAILURE;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ov2640_main
 *
 * Description:
 *   Entry point for the OV2640 Camera Demo
 *
 ****************************************************************************/

int ov2640_main(int argc, char *argv[])
{
  struct fb_vtable_s *vplane;
  int ret;

  /* First, initialize the display */

  vplane = ov2640_lcd_initialize();
  if (!vplane)
    {
      gerr("ERROR: ov2640_lcd_initialize failed\n");
      return  EXIT_FAILURE;
    }

  /* Then, initialize the camera */

  ret = ov2640_camera_initialize();
  if (ret != EXIT_SUCCESS)
    {
      gerr("ERROR: ov2640_camera_initialize failed\n");
      return  EXIT_FAILURE;
    }

  /* Now if everything is set up properly, the camera output should be
   * visible on the LCD.
   */

  return EXIT_SUCCESS;
}

#endif /* HAVE_CAMERA */
