/******************************************************************************
 * drivers/wireless/spirit/lib/spirit_aes.c
 *
 *   Copyright(c) 2015 STMicroelectronics
 *   Author: VMA division - AMS
 *   Version 3.2.2 08-July-2015
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *   1. Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/

/******************************************************************************
 * Included Files
 ******************************************************************************/

#include <assert.h>

#include "spirit_aes.h"
#include "spirit_spi.h"

/******************************************************************************
 * Public Functions
 ******************************************************************************/

/******************************************************************************
 * Name: spirit_aes_enable
 *
 * Description:
 *   Enables or Disables the AES engine.
 *
 * Input Parameters:
 *   spirit  - Reference to a Spirit library state structure instance
 *  newstate new state for AES engine.
 *         This parameter can be: S_ENABLE or S_DISABLE.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on any failure.
 *
 ******************************************************************************/

int spirit_aes_enable(FAR struct spirit_library_s *spirit,
                      enum spirit_functional_state_e newstate)
{
  uint8_t regval = 0;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(newstate));

  /* Modifies the register value */

  ret = spirit_reg_read(spirit, ANA_FUNC_CONF0_BASE, &regval, 1);
  if (ret >= 0)
    {
      if (newstate == S_ENABLE)
        {
          regval |= AES_MASK;
        }
      else
        {
          regval &= ~AES_MASK;
        }

      /* Write to the ANA_FUNC_CONF0 register to enable or disable the AES
       * engine
       */

      ret = spirit_reg_write(spirit, ANA_FUNC_CONF0_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_aes_write_datain
 *
 * Description:
 *   Writes the data to encrypt or decrypt, or the encryption key for the
 *   derive decryption key operation into the AES_DATA_IN registers.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   buffer - Pointer to the user data buffer.  The first byte of the array
 *            must be the MSB byte and it will be put in the AES_DATA_IN[0]
 *            register, while the last one must be the LSB and it will be
 *            put in the AES_DATA_IN[buflen-1] register. If data to write
 *            are less than 16 bytes the remaining AES_DATA_IN registers
 *            will be filled with bytes equal to 0.
 *   buflen - Length of data in bytes.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on any failure.
 *
 ******************************************************************************/

int spirit_aes_write_datain(FAR struct spirit_library_s *spirit,
                            FAR const uint8_t *buffer, uint8_t buflen)
{
  uint8_t datain[16];
  uint8_t i;

  /* Verifies that there are no more than 16 bytes */

  (buflen > 16) ? (buflen = 16) : buflen;

  /* Fill the datain with the data buffer, using padding */

  for (i = 0; i < 16; i++)
    {
      if (i < (16 - buflen))
        {
          datain[i] = 0;
        }
      else
        {
          datain[i] = buffer[15 - i];
        }
    }

  /* Writes the AES_DATA_IN registers */

  return spirit_reg_write(spirit, AES_DATA_IN_15_BASE, datain, 16);
}

/******************************************************************************
 * Name: spirit_aes_read_dataout
 *
 * Description:
 *   Returns the encrypted or decrypted data or the description key from the
 *   AES_DATA_OUT register.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   buffer - pointer to the user data buffer. The AES_DATA_OUT[0]
 *            register value will be put as first element of the buffer
 *            (MSB), while the AES_DAT_OUT[buflen-1] register value will be
 *            put as last element of the buffer (LSB).
 *   buflen - Length of data to read in bytes.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on any failure.
 *
 ******************************************************************************/

int spirit_aes_read_dataout(FAR struct spirit_library_s *spirit,
                            FAR uint8_t *buffer, uint8_t buflen)
{
  uint8_t dataout[16];
  uint8_t address;
  int ret;

  /* Verifies that there are no more than 16 bytes */

  if (buflen > 16)
    {
      buflen = 16;
    }

  /* Evaluates the address of AES_DATA_OUT from which start to read */

  address = AES_DATA_OUT_15_BASE + 16 - buflen;

  /* Reads the exact number of AES_DATA_OUT registers */

  ret = spirit_reg_read(spirit, address, dataout, buflen);
  if (ret >= 0)
    {
      int i;

      /* Copy in the user buffer the read values changing the order */

      for (i = buflen - 1; i >= 0; i--)
        {
          *buffer++ = dataout[i];
        }
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_aes_write_key
 *
 * Description:
 *   Writes the encryption key into the AES_KEY_IN register.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   key    - Pointer to the buffer of 4 words containing the AES key.
 *            The first byte of the buffer must be the most significant byte
 *            AES_KEY_0 of the AES key.  The last byte of the buffer must be
 *            the less significant byte AES_KEY_15 of the AES key.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on any failure.
 *
 ******************************************************************************/

int spirit_aes_write_key(FAR struct spirit_library_s *spirit,
                         FAR const uint8_t *key)
{
  uint8_t tmp[16];
  int i;

  for (i = 0; i < 16; i++)
    {
      tmp[15 - i] = key[i];
    }

  /* Write to the AES_DATA_IN registers */

  return spirit_reg_write(spirit, AES_KEY_IN_15_BASE, tmp, 16);
}

/******************************************************************************
 * Name: spirit_aes_read_key
 *
 * Description:
 *   Returns the encryption/decryption key from the AES_KEY_IN register.
 *
 * Input Parameters:
 *   spirit  - Reference to a Spirit library state structure instance
 *   key     - Pointer to the buffer of 4 words (16 bytes) containing the AES
 *             key.  The first byte of the buffer shall be the most
 *             significant byte AES_KEY_0 of the AES key.
 *             The last byte of the buffer shall be the less significant byte
 *             AES_KEY_15 of the AES key.  This parameter is an uint8_t*.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on any failure.
 *
 ******************************************************************************/

int spirit_aes_read_key(FAR struct spirit_library_s *spirit, FAR uint8_t *key)
{
  uint8_t tmp[16];
  int ret;
  int i;

  /* Reads the AES_DATA_IN registers */

  ret = spirit_reg_read(spirit, AES_KEY_IN_15_BASE, tmp, 16);
  if (ret >= 0)
    {
      for (i = 0; i < 16; i++)
        {
          key[i] = tmp[15 - i];
        }
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_aes_enc2deckey
 *
 * Description:
 *   Derives the decryption key from a given encryption key.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on any failure.
 *
 ******************************************************************************/

int spirit_aes_enc2deckey(FAR struct spirit_library_s *spirit)
{
  /* Sends the COMMAND_AES_KEY command */

  return spirit_command(spirit, COMMAND_AES_KEY);
}

/******************************************************************************
 * Name: spirit_aes_encrypt
 *
 * Description:
 *   Executes the encryption operation.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on any failure.
 *
 ******************************************************************************/

int spirit_aes_encrypt(FAR struct spirit_library_s *spirit)
{
  /* Sends the COMMAND_AES_ENC command */

  return spirit_command(spirit, COMMAND_AES_ENC);
}

/******************************************************************************
 * Name: spirit_aes_decrypt
 *
 * Description:
 *   Executes the decryption operation.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on any failure.
 *
 ******************************************************************************/

int spirit_aes_decrypt(FAR struct spirit_library_s *spirit)
{
  /* Sends the COMMAND_AES_DEC command */

  return spirit_command(spirit, COMMAND_AES_DEC);
}

/******************************************************************************
 * Name: spirit_aes_derivekey_decrypt
 *
 * Description:
 *   Executes the key derivation and the decryption operation.
 *
 * Input Parameters:
 *   spirit  - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on any failure.
 *
 ******************************************************************************/

int spirit_aes_derivekey_decrypt(FAR struct spirit_library_s *spirit)
{
  /* Sends the COMMAND_AES_KEY_DEC command */

  return spirit_command(spirit, COMMAND_AES_KEY_DEC);
}
