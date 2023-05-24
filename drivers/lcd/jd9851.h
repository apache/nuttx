/****************************************************************************
 * drivers/lcd/jd9851.h
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

#ifndef __DRIVERS_LCD_JD9851_H
#define __DRIVERS_LCD_JD9851_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define JD9851_SLEEP_IN         0x10 /* Sleep in */
#define JD9851_SLEEP_OUT        0x11 /* Sleep out */

#define JD9851_INVERSION_ON     0x21 /* Display inversion on */
#define JD9851_DISOFF           0x28 /* Display off */
#define JD9851_DISON            0x29 /* Display on */
#define JD9851_CASET            0x2A /* Column address set */
#define JD9851_PASET            0x2B /* Page address set */
#define JD9851_RAMWR            0x2C /* Memory Write */
#define JD9851_RAMRD            0x2E /* Memory read */

#define JD9851_TEOFF            0x34 /* Tearing effect line off */
#define JD9851_TEON             0x35 /* Tearing effect line on */
#define JD9851_TE_MODE0         0x00
#define JD9851_TE_MODE1         0x01
#define JD9851_COLMOD           0x3A /* Pixel format set */
#define JD9851_COLMOD_16BIT     0x55

#define JD9851_MADCTL           0x36   /* Memory Data Access Control */
#define JD9851_MADCTL_MY        (1<<7) /* Page Address Order */
#define JD9851_MADCTL_MX        (1<<6) /* Column Address Order */
#define JD9851_MADCTL_MV        (1<<5) /* Page/Column Order */
#define JD9851_MADCTL_ML        (1<<4) /* Line Address Order */
#define JD9851_MADCTL_BGR       (1<<3) /* Set Panel Order BGR */

#define JD9851_SET_PAGE_CMD     0xDE /* Change page */
#define JD9851_PAGE_0           0x00
#define JD9851_PAGE_2           0x02

#define JD9851_PASSWORD         0xDF /* Set password to access inhouse reg */

/* PAGE_0 */
#define JD9851_GAMMA_SET_CMD    0xB7
#define JD9851_R_GAMMA_SET_CMD  0xC8

#define JD9851_POWER_CTRL       0xB9 /* Control engineer mode of power related setting */
#define JD9851_DCDC_SET         0xBB /* Set charge pump related setting */
#define JD9851_VDDD_CTRL        0xBC /* Control inrernal logic voltage setting */
#define JD9851_SETSTBA          0xC0 /* Set source output driving ability */
#define JD9851_SETPANEL         0xC1 /* Set panel related reg, page_0/2 */
#define JD9851_SETRGBCYC        0xC3 /* Set internal SD output timing */
#define JD9851_SETTCON          0xC4 /* Set timing control */
#define JD9851_SETGD            0xD0 /* Set Gate function */

#define JD9851_SETRGBIF         0xBD /* Set RGB interface related setting */
#define JD9851_RAM_CTRL         0xD7 /* Set interface related setting */

/* PAGE_2 */
#define JD9851_DCDC_SET2        0xB8 /* Set internal OTP program related setting */
#define JD9851_OSCM_SET         0xC5 /* Set oscillator M. OSCM */
#define JD9851_SETMIPI_2        0xCA

FAR struct lcd_dev_s *jd9851_lcdinitialize(FAR struct spi_dev_s *spi);

#endif /* __DRIVERS_LCD_JD9851_H */
