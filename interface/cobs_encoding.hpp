#pragma once

#include <cstddef>
#include <cstdint>

namespace hex_mini_drive {

// Class to handle COBS decoding.
// 
// COBS (Consistent Overhead Byte Stuffing) means use 0 as the message delimiter
// and replace occurences of 0 with a counter inidicating the distance to the next 0.
template<size_t max_message_size>
struct ConsistentOverheadByteStuffing {
  
  // Buffer used to write decoded data.
  uint8_t decoding_buffer[max_message_size] = {0};
  // Amount of data currently in the decoding buffer.
  size_t decoding_buffer_size = 0;

  // Number of bytes remaining until we have to insert a zero byte for COBS decoding.
  size_t decoding_length_until_zero = 0;

  // Whether we need to insert a 0 byte in the decoded output or not. COBS encoding
  // uses 255=0xFF as a special value to indicate that the next 255 bytes are all non-zero.
  // The next byte afterwards will be another length byte, but for the special case of 0xFF
  // we must not insert a zero byte in the decoded output.
  bool decoding_insert_zero_byte = false;

  // Whether we have a complete message in the decoding_buffer.
  bool decoding_ready = false;

  // Buffer for encoding; we can reuse the same buffer for each message.
  uint8_t encoding_buffer[max_message_size] = {0};
  size_t encoding_buffer_size = 0;

  // Reset the decoder state.
  void decode_reset() {
    for (size_t i = 0; i < decoding_buffer_size; ++i) decoding_buffer[i] = 0;
    decoding_buffer_size = 0;
    decoding_length_until_zero = 0;
    decoding_insert_zero_byte = false;
    decoding_ready = false;
  }

  // Check if the decoder is empty.
  bool decode_empty() {
    return decoding_buffer_size == 0;
  }

  // Check whether we are in the middle of decoding a message.
  bool decode_ready(){
    return decoding_ready;
  }

  // Decode a chunk of data framed using COBS; return the number of bytes processed.
  // It may return early. A message is ready if decoding_ready is true, otherwise there's 
  // a partial message, other otherwise the bytes were dropped if decode_empty.
  size_t decode_chunk(const uint8_t * chunk, size_t chunk_length) {
    // Iterate over all bytes in the data chunk.
    for (size_t i = 0; i < chunk_length; ++i) {
      const uint8_t byte = chunk[i];

      // Check for zero byte which indicates end of message.
      if (byte == 0) {
        if (decoding_buffer_size == 0) {
          // Do nothing for an empty message.
        } else if (decoding_length_until_zero == 0) {
          // We're done reading the message by receiving a zero byte at the right time.
          decoding_ready = true;
          // Return the length of the processed chunk.
          return i + 1;
        } else {
          // We received a zero byte but we were still expecting more data for 
          // the current message, so this is an error.
          //
          // Reset for the next message; and return the data processed so far.
          decode_reset();
          return i + 1;
        }

      // Otherwise we expect to read the length till the next 0.
      } else if (decoding_length_until_zero == 0) {

        // The current byte represents the length until we need to insert a zero byte in the
        // decoded output. We then count this step, hence the -1.
        decoding_length_until_zero = byte - 1;

        // If this isn't the start, or 0xFF continuation, we need to add a zero to the
        // decoded output according to this segment's zero length byte.
        if (decoding_insert_zero_byte) {
          if (decoding_buffer_size >= max_message_size) {
            // We don't have space to insert the zero byte, so we drop the current message.
            decode_reset();
            return i + 1;
          } else {
            decoding_buffer[decoding_buffer_size++] = 0;  // Insert zero byte for COBS decoding.
          }
        }

        // If this is a 0xFF continuation, we don't insert a zero byte for the next segment. 
        decoding_insert_zero_byte = (byte != 0xFF);
        

      // Finally we read ordinary non-zero data.
      } else {

        // Copy the current byte and count down the length until we need to insert a zero byte.
        if (decoding_buffer_size >= max_message_size) {
          // We don't have space to copy the byte, so we drop the current message.
          decode_reset();
          return i + 1;
        } else {
          decoding_buffer[decoding_buffer_size++] = byte;
          decoding_length_until_zero--;
        }
      }
    }

    // We finished processing the whole chunk without an early return.
    return chunk_length;
  }

  // Reset the encoder state.
  void encode_reset() {
    for (size_t i = 0; i < encoding_buffer_size; ++i) encoding_buffer[i] = 0;
    encoding_buffer_size = 0;
  }

  // Check whether the encoding buffer is empty.
  bool encode_empty() {
    return encoding_buffer_size == 0;
  }

  // Calculate the expected output size for the current message in the encoding buffer.
  size_t encode_max_required_size() {
    // Each non-zero byte may require an additional length byte, plus one for the final zero delimiter.
    return encoding_buffer_size + (encoding_buffer_size / 254) + 1;
  }

  // Encode a message using COBS (Consistent Overhead Byte Stuffing).
  // Returns the amount of data written; 0 for errors.
  size_t encode_message(uint8_t * output, size_t max_output_size) {

    // Current reading index from the input message.
    size_t read_index = 0;
    // Start writing after the first length byte.
    size_t write_index = 1;
    // Index where the current length byte will go.
    size_t code_index = 0;
    // Current length byte value.
    uint8_t code = 1;

    if (write_index >= max_output_size) {
      return 0;  // Indicate failure due to insufficient buffer size.
    }

    while (read_index < encoding_buffer_size) {
      if (encoding_buffer[read_index] == 0) {
        // Write the length byte.
        output[code_index] = code;
        code_index = write_index++;
        code = 1;  // Reset code for the next segment.
        read_index++;
      } else {
        // Copy non-zero byte to output.
        output[write_index++] = encoding_buffer[read_index++];
        code++;
        // If code reaches 0xFF, we need to start a new segment.
        if (code == 0xFF) {
          output[code_index] = code;
          code_index = write_index++;
          code = 1;
        }
      }
      // Check for output buffer overflow.
      if (write_index >= max_output_size) {
        return 0;  // Indicate failure due to insufficient buffer size.
      }
    }
    // Write the final length byte.
    output[code_index] = code;
    // Append the zero delimiter at the end.
    output[write_index++] = 0;

    // Yay.
    return write_index;
  }
};

} // end namespace hex_mini_drive