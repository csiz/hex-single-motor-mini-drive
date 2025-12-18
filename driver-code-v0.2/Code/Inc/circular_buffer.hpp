#pragma once

#include <cstddef>
#include <cstdint>


struct CircularBuffer {
  uint8_t * buffer;
  size_t max_size;
  volatile size_t write_head = 0;
  volatile size_t read_tail = 0;
  volatile size_t read_head = 0;
};

static inline void buffer_reset(CircularBuffer * buffer) {
  buffer->write_head = 0;
  buffer->read_tail = 0;
  buffer->read_head = 0;
}

static inline uint8_t * buffer_reserve_write_head(CircularBuffer * buffer, size_t size) {
  if (size == 0) return nullptr;


  if (buffer->write_head < buffer->read_tail) {
    // We are in wrap around mode.

    // Check if there's anything reserved for reading.
    if (buffer->read_head == buffer->read_tail) {
      // Reading was done, reset the read pointers.
      buffer->read_tail = buffer->write_head;
      buffer->read_head = 0;

      return (buffer->write_head + size < buffer->max_size) ? &buffer->buffer[buffer->write_head] : nullptr;
    } else {
    
      // We can only write if we have enough space before the read head.
      return (buffer->write_head + size < buffer->read_head) ? &buffer->buffer[buffer->write_head] : nullptr;
    }


  } else {
    // We are after the read area so there is space until the end of the buffer.

    if (buffer->write_head + size < buffer->max_size) {
      // We have enough space at the end of the buffer.
      return &buffer->buffer[buffer->write_head];

    } else if (size < buffer->read_head) {
      // We have enough space at the start of the buffer; but we need to mark the wrap around.
      buffer->write_head = 0;
      return &buffer->buffer[0];

    } else {
      // Not enough space to write the data.
      return nullptr;
    }
  }
}

static inline uint8_t buffer_mark_write(CircularBuffer * buffer, size_t size){
  // Compute the new write tail after writing.
  const size_t write_tail = buffer->write_head + size;

  // Check whether we are writing after the read area or in the wrap around area.
  if (buffer->write_head < buffer->read_tail) {
    // We are in the wrap around area.

    // Ensure we did not overwrite unread data.
    if (write_tail >= buffer->read_head) return 1;

    // Only advance the write head, leave the read tail to be reset by a complete read.
    buffer->write_head = write_tail;
    return 0;
  } else {
    // We are writing after the read area, no risk of overwriting unread data.

    // Ensure we do not exceed the buffer size.
    if (write_tail >= buffer->max_size) return 1;

    // Mark the data for reading.
    buffer->read_tail = write_tail;

    // Wrap around the write head if we have reached the end of the buffer.
    buffer->write_head = write_tail < buffer->max_size ? write_tail : 0;
    return 0;
  }
}

static inline size_t buffer_available_to_read(CircularBuffer * buffer) {
  // Since read tail is reset before the read head we should cap our result to 0 in case.
  const int readable_size = buffer->read_tail - buffer->read_head;
  return (readable_size > 0) ? readable_size : 0;
}

static inline uint8_t * buffer_get_read_head(CircularBuffer * buffer) {
  return &buffer->buffer[buffer->read_head];
}


static inline uint8_t buffer_mark_read(CircularBuffer * buffer, size_t size) {
  // Check how much data is available to read.
  const size_t available_size = buffer_available_to_read(buffer);

  // Ensure we are not trying to read more than available.
  if (size > available_size) return 1;

  // Advance the read head.
  buffer->read_head += size;

  // If we finished reading all data and the write head is wrapped around, we can reset the read head too.
  if (buffer->read_head == buffer->read_tail and buffer->write_head < buffer->read_tail) {

    // Set the new read tail to the current write head.
    buffer->read_tail = buffer->write_head;

    // Reset the read head to 0 for the next buffer_read call.
    buffer->read_head = 0;
  }

  return 0;
}