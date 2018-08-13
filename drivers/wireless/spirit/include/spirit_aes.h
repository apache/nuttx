/*******************************************************************************
 * include/nuttx/wireless/spirit/include/spirit_aes.h
 * Configuration and management of SPIRIT AES Engine.
 *
 *   Copyright(c) 2015 STMicroelectronics
 *   Author: VMA division - AMS
 *   Version 3.2.2 08-July-2015
 *
 *   Adapted for NuttX by:
 *   Author:  Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/

#ifndef __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_AES_H
#define __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_AES_H

/* In order to encrypt data, the user must manage the AES_END IRQ. The data
 * must be split in blocks of 16 bytes and written into the AES DATA IN
 * registers. Then, after the key is written into the AES KEY registers, a
 * command of Execute encryption has to be sent.
 *
 * Example:
 *
 *     spirit_aes_write_datain(spirit, inbuffer , buflen);
 *     spirit_aes_encrypt(spirit);
 *
 *   Wait for encryption done (signalled via interrupt)
 *
 *     spirit_aes_read_dataout(espirit, outbuffer , buflen);
 *
 * In order to decrypt data, the user must manage the AES_END IRQ and have a
 * decryption key.  There are two operative modes to make the data
 * decryption:
 *
 * 1. Derive the decryption key from the encryption key and decrypt data
 *    directly using the spirit_aes_derivekey_decrypt() function
 *
 *    Example:
 *
 *      spirit_aes_write_datain(spirit, inbuffer , buflen);
 *      spirit_aes_derivekey_decrypt(spirit);
 *
 *    Wait for key derivation to complete (signalled via interrupt)
 *
 *      spirit_aes_read_dataout(spirit, outbuffer , buflen);
 *
 * 2  Derive the decryption key from the encryption key using the
 *    spirit_aes_enc2deckey() function, store it into the AES KEY
 *    registers and then decrypt data using the spirit_aes_decrypt()
 *    function
 *
 *    Example:
 *
 *      spirit_aes_write_datain(spirit, keyenc, 16);
 *      spirit_aes_enc2deckey(spirit);
 *
 *    Wait for key derivation to complete (signalled via interrupt)
 *
 *      spirit_aes_read_dataout(spirit, keydec, 16);
 *
 *      spirit_aes_write_key(key_dec);
 *      spirit_aes_write_datain(inbuffer , 16);
 *      spirit_aes_decrypt(spirit);
 *
 *    Wait for encryption done (signalled via interrupt)
 *
 *      spirit_aes_read_dataout(spirit, outbuffer, buflen);
 */

/******************************************************************************
 * Included Files
 ******************************************************************************/

#include "spirit_regs.h"
#include "spirit_types.h"

/******************************************************************************
 * Public Function Prototypes
 ******************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

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
                      enum spirit_functional_state_e newstate);

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
                            FAR const uint8_t *buffer, uint8_t buflen);

/******************************************************************************
 * Name: spirit_aes_read_dataout
 *
 * Description:
 *   Returns the encrypted or decrypted data or the decription key from the
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
                            FAR uint8_t *buffer, uint8_t buflen);

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
                         FAR const uint8_t *key);

/******************************************************************************
 * Name: spirit_aes_read_key
 *
 * Description:
 *   Returns the encryption/decryption key from the AES_KEY_IN register.
 *
 * Input Parameters:
 *   spirit  - Reference to a Spirit library state structure instance
 *  key  pointer to the buffer of 4 words (16 bytes) containing the AES key.
 *         The first byte of the buffer shall be the most significant byte AES_KEY_0 of the AES key.
 *         The last byte of the buffer shall be the less significant byte AES_KEY_15 of the AES key.
 *         This parameter is an uint8_t*.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on any failure.
 *
 ******************************************************************************/

int spirit_aes_read_key(FAR struct spirit_library_s *spirit, FAR uint8_t *key);

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

int spirit_aes_enc2deckey(FAR struct spirit_library_s *spirit);

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

int spirit_aes_encrypt(FAR struct spirit_library_s *spirit);

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

int spirit_aes_decrypt(FAR struct spirit_library_s *spirit);

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

int spirit_aes_derivekey_decrypt(FAR struct spirit_library_s *spirit);

#ifdef __cplusplus
}
#endif

#endif /* __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_AES_H */
