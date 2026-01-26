#pragma once

#include <cstddef>
#include <cstdint>

// Class to handle COBS decoding.
// 
// COBS (Consistent Overhead Byte Stuffing) means use 0 as the message delimiter
// and replace occurences of 0 with a counter inidicating the distance to the next 0.
struct COBS_Buffer {
  // Alias for a generic function that processes a buffer of some size.
  using BufferFunction = void (*)(const uint8_t * buffer, size_t size);

  static const size_t max_message_size = 512;
  static const size_t max_message_size_m1 = max_message_size - 1; // Reserve space for delimiter.

  // Buffer and state for decoding; because we may receive data in chunks
  // so we have to maintain state between calls.
  uint8_t decoded_data[max_message_size] = {};
  size_t decoded_length = 0;
  size_t decoded_length_until_zero = 0;

  // Buffer for encoding; we can reuse the same buffer for each message.
  uint8_t encoded_data[max_message_size] = {};
  size_t encoded_length = 0;

  // Reset the decoder state.
  void decode_reset() {
    decoded_length = 0;
    decoded_length_until_zero = 0;
  }

  // Check whether we are in the middle of decoding a message.
  bool decode_ongoing(){
    return decoded_length > 0;
  }

  // Decode a chunk of data framed using COBS.
  void decode_chunk(const uint8_t * chunk, size_t chunk_length, BufferFunction received_message) {
    for (size_t i = 0; i < chunk_length; ++i) {
      const uint8_t byte = chunk[i];

      // Check for zero byte which indicates end of message.
      if (byte == 0) {
        if (decoded_length != 0 and decoded_length_until_zero == 0) {
          // Process the complete message.
          received_message(decoded_data, decoded_length);
        }
        // Reset for the next message.
        decode_reset();
      // Otherwise we expect to read the legth till the next 0.
      } else if (decoded_length_until_zero == 0) {
        decoded_length_until_zero = byte - 1;
        // If this isn't the start, we need to add a zero when we expected a zero.
        if (decoded_length != 0) {
          decoded_data[decoded_length++] = 0;  // Insert zero byte for COBS decoding.
        }
      // Finally we read ordinary non-zero data.
      } else {
        decoded_data[decoded_length++] = byte;
        decoded_length_until_zero--;
      }
    }
  }

  // Reset the encoder state.
  void encode_reset() {
    encoded_length = 0;
  }

  // Check whether a message has been encoded.
  bool is_message_encoded() {
    return encoded_length > 0;
  }

  // Encode a message using COBS (Consistent Overhead Byte Stuffing).
  // 
  // Returns the length of the encoded message, or 0 on failure.
  bool encode_message(const uint8_t * input, size_t input_length) {
    // Nothing to encode.
    if (input_length == 0) return false;
    if (input == nullptr) return false;

    // Reset encode length to zero in case of errors.
    encoded_length = 0;
    
    size_t read_index = 0;
    size_t write_index = 1;  // Start writing after the first length byte.
    size_t code_index = 0;   // Index where the current length byte will go.
    uint8_t code = 1;        // Current length byte value.

    while (read_index < input_length) {
      if (input[read_index] == 0) {
        // Write the length byte.
        encoded_data[code_index] = code;
        code_index = write_index++;
        code = 1;  // Reset code for the next segment.
        read_index++;
      } else {
        // Copy non-zero byte to output.
        encoded_data[write_index++] = input[read_index++];
        code++;
        // If code reaches 0xFF, we need to start a new segment.
        if (code == 0xFF) {
          encoded_data[code_index] = code;
          code_index = write_index++;
          code = 1;
        }
      }
      // Check for output buffer overflow.
      if (write_index >= max_message_size_m1) {
        return false;  // Indicate failure due to insufficient buffer size.
      }
    }
    // Write the final length byte.
    encoded_data[code_index] = code;
    // Append the zero delimiter at the end.
    encoded_data[write_index++] = 0;

    encoded_length = write_index;

    // Yay.
    return true;
  }
};