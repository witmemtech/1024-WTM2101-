/**
 * Copyright (c) 2015 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef SLIP_H__
#define SLIP_H__
#define ERR_BASE      (0)       ///< Global error base 

#define ERR_SUCCESS                     (0)  ///< Successful command
#define ERR_SVC_HANDLER_MISSING         (ERR_BASE + 1)  ///< SVC handler is missing
#define ERR_SOFTDEVICE_NOT_ENABLED      (ERR_BASE + 2)  ///< SoftDevice has not been enabled
#define ERR_INTERNAL                    (ERR_BASE + 3)  ///< Internal Error
#define ERR_NO_MEM                      (ERR_BASE + 4)  ///< No Memory for operation
#define ERR_NOT_FOUND                   (ERR_BASE + 5)  ///< Not found
#define ERR_NOT_SUPPORTED               (ERR_BASE + 6)  ///< Not supported
#define ERR_INVALID_PARAM               (ERR_BASE + 7)  ///< Invalid Parameter
#define ERR_INVALID_STATE               (ERR_BASE + 8)  ///< Invalid state, operation disallowed in this state
#define ERR_INVALID_LENGTH              (ERR_BASE + 9)  ///< Invalid Length
#define ERR_INVALID_FLAGS               (ERR_BASE + 10) ///< Invalid Flags
#define ERR_INVALID_DATA                (ERR_BASE + 11) ///< Invalid Data
#define ERR_DATA_SIZE                   (ERR_BASE + 12) ///< Invalid Data size
#define ERR_TIMEOUT                     (ERR_BASE + 13) ///< Operation timed out
#define ERR_NULL                        (ERR_BASE + 14) ///< Null Pointer
#define ERR_FORBIDDEN                   (ERR_BASE + 15) ///< Forbidden Operation
#define ERR_INVALID_ADDR                (ERR_BASE + 16) ///< Bad Memory Address
#define ERR_BUSY                        (ERR_BASE + 17) ///< Busy
#define ERR_CONN_COUNT                  (ERR_BASE + 18) ///< Maximum connection count exceeded.
#define ERR_RESOURCES                   (ERR_BASE + 19) ///< Not enough resources for operation
#define ERR_BT_OTA                      (ERR_BASE + 20) ///< Not enough resources for operation
#define ERR_NO_SPACE                    (ERR_BASE + 21) ///< Not enough space for operation
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @file
 *
 * @defgroup slip SLIP encoding and decoding
 * @{
 * @ingroup app_common
 *
 * @brief  This module encodes and decodes SLIP packages.
 *
 * @details The SLIP protocol is described in @linkSLIP.
 */

/** @brief Status information that is used while receiving and decoding a packet. */
typedef enum
{
  SLIP_STATE_DECODING = 0, //!< Ready to receive the next byte.
  SLIP_STATE_ESC_RECEIVED, //!< An ESC byte has been received and the next byte must be decoded differently.
  SLIP_STATE_CLEARING_INVALID_PACKET, //!< The received data is invalid and transfer must be restarted.
  SLIP_STATE_WAITING
} slip_read_state_t;

  /** @brief Representation of a SLIP packet. */
typedef struct
{
  slip_read_state_t   state; //!< Current state of the packet (see @ref slip_read_state_t).

  uint8_t             * p_buffer; //!< Decoded data.
  uint32_t            current_index; //!< Current length of the packet that has been received.
  uint32_t            buffer_len; //!< Size of the buffer that is available.
} slip_t;

/**@brief Function for encoding a SLIP packet.
 *
 * The maximum size of the output data is (2*input size + 1) bytes. Ensure that the provided buffer is large enough.
 *
 * @param[in,out]   p_output                The buffer where the encoded SLIP packet is stored. Ensure that it is large enough.
 * @param[in,out]   p_input                 The buffer to be encoded.
 * @param[in,out]   input_length            The length of the input buffer.
 * @param[out]      p_output_buffer_length  The length of the output buffer after the input has been encoded.
 *
 * @retval  NRF_SUCCESS         If the input was successfully encoded into output.
 * @retval  NRF_ERROR_NULL      If one of the provided parameters is NULL.
 */
uint32_t slip_encode(uint8_t * p_output,  uint8_t * p_input, uint32_t input_length, uint32_t * p_output_buffer_length);

/**@brief Function for decoding a SLIP packet.
 *
 * The decoded packet is put into @p p_slip::p_buffer. The index and buffer state is updated.
 *
 * Ensure that @p p_slip is properly initialized. The initial state must be set to @ref SLIP_STATE_DECODING.
 *
 * @param[in,out]   p_slip  State of the decoding process.
 * @param[in]       c       Byte to decode.
 *
 * @retval NRF_SUCCESS              If a packet has been parsed. The received packet can be retrieved from @p p_slip.
 * @retval NRF_ERROR_NULL           If @p p_slip is NULL.
 * @retval NRF_ERROR_NO_MEM         If there is no more room in the buffer provided by @p p_slip.
 * @retval NRF_ERROR_BUSY           If the packet has not been parsed completely yet.
 * @retval NRF_ERROR_INVALID_DATA   If the packet is encoded wrong. In this case, @p p_slip::state is set to @ref SLIP_STATE_CLEARING_INVALID_PACKET,
 *                                  and decoding will stay in this state until the END byte is received.
 */
uint32_t slip_decode_add_byte(slip_t * p_slip, uint8_t c);

#ifdef __cplusplus
}
#endif

#endif // SLIP_H__

/** @} */
