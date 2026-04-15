#pragma once

#include <cstddef>
#include <cstdint>

namespace hex_mini_drive {

// Maximum message size (in bytes) for a message.
const size_t max_message_size = 256;

// A statically allocated buffer for messages.
struct MessageBuffer {
    uint8_t data[max_message_size];
    size_t size = 0;
};

// Class to handle COBS decoding.
// 
// COBS (Consistent Overhead Byte Stuffing) means use 0 as the message delimiter
// and replace occurences of 0 with a counter inidicating the distance to the next 0.
struct COBS_Buffer {
  
  // Buffer and state for decoding; because we may receive data in chunks
  // so we have to maintain state between calls.
  MessageBuffer decoding_buffer;

  // Number of bytes remaining until we have to insert a zero byte for COBS decoding.
  size_t decoding_length_until_zero = 0;

  // Whether we need to insert a 0 byte in the decoded output or not. COBS encoding
  // uses 255=0xFF as a special value to indicate that the next 255 bytes are all non-zero.
  // The next byte afterwards will be another length byte, but for the special case of 0xFF
  // we must not insert a zero byte in the decoded output.
  bool decoding_insert_zero_byte = false;

  // Buffer for encoding; we can reuse the same buffer for each message.
  MessageBuffer encoding_buffer;

  // Reset the decoder state.
  void decode_reset() {
    decoding_buffer.size = 0;
    decoding_length_until_zero = 0;
    decoding_insert_zero_byte = false;
  }

  // Check whether we are in the middle of decoding a message.
  bool decode_ongoing(){
    return decoding_buffer.size > 0;
  }

  // Decode a chunk of data framed using COBS; return the number of dropped bytes due to errors.
  size_t decode_chunk(const uint8_t * chunk, size_t chunk_length, void (*received_message)(uint8_t * buffer, size_t size)) {
    size_t dropped_bytes = 0;
    
    // Iterate over all bytes in the data chunk.
    for (size_t i = 0; i < chunk_length; ++i) {
      const uint8_t byte = chunk[i];

      // Check for zero byte which indicates end of message.
      if (byte == 0) {

        if (decoding_buffer.size != 0 and decoding_length_until_zero == 0) {
          // Process the complete message.
          received_message(decoding_buffer.data, decoding_buffer.size);
        } else {
          // We received a zero byte but we were still expecting more data for 
          // the current message, so this is an error.
          dropped_bytes += decoding_buffer.size;
        }
        // Reset for the next message.
        decode_reset();
        

      // Otherwise we expect to read the length till the next 0.
      } else if (decoding_length_until_zero == 0) {

        // The current byte represents the length until we need to insert a zero byte in the
        // decoded output. We then count this step, hence the -1.
        decoding_length_until_zero = byte - 1;

        // If this isn't the start, or 0xFF continuation, we need to add a zero to the
        // decoded output according to this segment's zero length byte.
        if (decoding_insert_zero_byte) {
          decoding_buffer.data[decoding_buffer.size++] = 0;  // Insert zero byte for COBS decoding.
        }

        // If this is a 0xFF continuation, we don't insert a zero byte for the next segment. 
        decoding_insert_zero_byte = (byte != 0xFF);
        

      // Finally we read ordinary non-zero data.
      } else {

        // Copy the current byte and count down the length until we need to insert a zero byte.
        decoding_buffer.data[decoding_buffer.size++] = byte;
        decoding_length_until_zero--;
      }
    }

    return dropped_bytes;
  }

  // Reset the encoder state.
  void encode_reset() {
    encoding_buffer.size = 0;
  }

  // Check whether a message has been encoded.
  bool is_message_encoded() {
    return encoding_buffer.size > 0;
  }

  // Encode a message using COBS (Consistent Overhead Byte Stuffing); returns true for success, false for failure/invalid input.
  bool encode_message(uint8_t const* input, size_t input_size) {
    // Nothing to encode.
    if (input_size == 0) return false;

    // Reset encode length to zero in case of errors.
    encoding_buffer.size = 0;
    
    // Current reading index from the input message.
    size_t read_index = 0;
    // Start writing after the first length byte.
    size_t write_index = 1;
    // Index where the current length byte will go.
    size_t code_index = 0;
    // Current length byte value.
    uint8_t code = 1;

    while (read_index < input_size) {
      if (input[read_index] == 0) {
        // Write the length byte.
        encoding_buffer.data[code_index] = code;
        code_index = write_index++;
        code = 1;  // Reset code for the next segment.
        read_index++;
      } else {
        // Copy non-zero byte to output.
        encoding_buffer.data[write_index++] = input[read_index++];
        code++;
        // If code reaches 0xFF, we need to start a new segment.
        if (code == 0xFF) {
          encoding_buffer.data[code_index] = code;
          code_index = write_index++;
          code = 1;
        }
      }
      // Check for output buffer overflow.
      if (write_index >= max_message_size) {
        return false;  // Indicate failure due to insufficient buffer size.
      }
    }
    // Write the final length byte.
    encoding_buffer.data[code_index] = code;
    // Append the zero delimiter at the end.
    encoding_buffer.data[write_index++] = 0;

    encoding_buffer.size = write_index;

    // Yay.
    return true;
  }
};

} // end namespace hex_mini_drive