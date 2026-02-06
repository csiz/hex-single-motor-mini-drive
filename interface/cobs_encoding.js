// COBS (Consistent Overhead Byte Stuffing) implementation in JavaScript
// COBS uses 0 as the message delimiter and replaces occurrences of 0 with a counter
// indicating the distance to the next 0.

const MAX_MESSAGE_SIZE = 512;
const MAX_MESSAGE_SIZE_M1 = MAX_MESSAGE_SIZE - 1; // Reserve space for delimiter.

/**
 * Class to handle COBS decoding and encoding.
 */
class COBS_Buffer {
  /**
   * Constructor for COBS_Buffer
   */
  constructor() {
    // Buffer and state for decoding; because we may receive data in chunks
    // so we have to maintain state between calls.
    this.decoded_data = new Uint8Array(MAX_MESSAGE_SIZE);
    this.decoded_length = 0;
    this.decoded_length_until_zero = 0;

    // Whether we need to insert a 0 byte in the decoded output or not. COBS encoding
    // uses 255=0xFF as a special value to indicate that the next 255 bytes are all non-zero.
    // The next byte afterwards will be another length byte, but for the special case of 0xFF
    // we must not insert a zero byte in the decoded output.
    this.decoding_insert_zero_byte = false;

    // Buffer for encoding; we can reuse the same buffer for each message.
    this.encoded_data = new Uint8Array(MAX_MESSAGE_SIZE);
  }

  /**
   * Reset the decoder state.
   */
  decode_reset() {
    this.decoded_length = 0;
    this.decoded_length_until_zero = 0;
    this.decoding_insert_zero_byte = false;
  }

  /**
   * Check whether we are in the middle of decoding a message.
   * @returns {boolean} True if decoding is ongoing
   */
  decode_ongoing() {
    return this.decoded_length > 0;
  }

  /**
   * Decode a chunk of data framed using COBS.
   * @param {Uint8Array} chunk - The chunk of data to decode
   * @param {Function} received_message - Callback function to process decoded messages
   */
  decode_chunk(chunk, received_message) {
    for (let i = 0; i < chunk.length; i++) {
      const byte = chunk[i];

      // Check for zero byte which indicates end of message
      if (byte === 0) {
        if (this.decoded_length !== 0 && this.decoded_length_until_zero === 0) {
          // Process the complete message
          received_message(this.decoded_data.slice(0, this.decoded_length));
        }
        // Reset for the next message
        this.decode_reset();
      }
      // Otherwise we expect to read the length till the next 0
      else if (this.decoded_length_until_zero === 0) {
        // The current byte represents the length until we need to insert a zero byte in the
        // decoded output. We then count this step, hence the -1.
        this.decoded_length_until_zero = byte - 1;

        // If this isn't the start, or 0xFF continuation, we need to add a zero to the
        // decoded output according to this segment's zero length byte.
        if (this.decoding_insert_zero_byte) {
          this.decoded_data[this.decoded_length++] = 0; // Insert zero byte for COBS decoding
        }

        // If this is a 0xFF continuation, we don't insert a zero byte for the next segment.
        this.decoding_insert_zero_byte = (byte !== 0xFF);
      }
      // Finally we read ordinary non-zero data
      else {
        this.decoded_data[this.decoded_length++] = byte;
        this.decoded_length_until_zero--;
      }
    }
  }

  /**
   * Reset the encoder state.
   */
  encode_reset() {
    this.encoded_length = 0;
  }

  /**
   * Check whether a message has been encoded.
   * @returns {boolean} True if a message has been encoded
   */
  is_message_encoded() {
    return this.encoded_length > 0;
  }

  /**
   * Encode a message using COBS (Consistent Overhead Byte Stuffing).
   * @param {Uint8Array} input - The input data to encode
   * @returns {Uint8Array|null} The encoded message or null on failure.
   */
  encode_message(input) {
    // Nothing to encode.
    if (input.length === 0) return null;
    if (input === null || input === undefined) return null;

    // Reset encode length to zero in case of errors.
    this.encoded_length = 0;

    let read_index = 0;
    let write_index = 1; // Start writing after the first length byte
    let code_index = 0; // Index where the current length byte will go
    let code = 1; // Current length byte value

    while (read_index < input.length) {
      if (input[read_index] === 0) {
        // Write the length byte
        this.encoded_data[code_index] = code;
        code_index = write_index++;
        code = 1; // Reset code for the next segment
        read_index++;
      } else {
        // Copy non-zero byte to output
        this.encoded_data[write_index++] = input[read_index++];
        code++;
        // If code reaches 0xFF, we need to start a new segment
        if (code === 0xFF) {
          this.encoded_data[code_index] = code;
          code_index = write_index++;
          code = 1;
        }
      }
      // Check for output buffer overflow
      if (write_index >= MAX_MESSAGE_SIZE_M1) {
        return null; // Indicate failure due to insufficient buffer size
      }
    }
    // Write the final length byte
    this.encoded_data[code_index] = code;
    // Append the zero delimiter at the end
    this.encoded_data[write_index++] = 0;

    // Return a copy of the encoded data with correct length.
    return this.encoded_data.slice(0, write_index);
  }
}

// Export the driver interface.
export { COBS_Buffer };