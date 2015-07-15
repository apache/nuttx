/********************************************************************************************
 * drivers/lcd/ssd1306_helpers.h
 *
 *   Copyright (C) 2015 Alan Carvalho de Assis
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ********************************************************************************************/

#include <nuttx/lcd/ssd1306.h>

//void ssd1306_sendbyte(FAR struct ssd1306_dev_s *priv, uint8_t regval);
//void ssd1306_sendblk(FAR struct ssd1306_dev_s *priv, uint8_t *data, uint8_t len);

#ifdef CONFIG_LCD_SSD1306_SPI
//  void ssd1306_select(FAR struct ssd1306_dev_s *priv, bool cs);
//  void ssd1306_cmddata(FAR struct ssd1306_dev_s *priv, bool cmd);

#  ifndef CONFIG_SPI_OWNBUS
//    static inline void ssd1306_configspi(FAR struct spi_dev_s *spi)
#  endif

#else
#  define ssd1306_select(priv, cs)
#  define ssd1306_cmddata(priv, cmd)
#endif 
