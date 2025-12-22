#pragma once

#include <cstddef>
#include <cstdint>


class CircularBuffer {
  uint8_t * buffer;
  size_t max_size;
  volatile size_t write_head = 0;
  volatile size_t read_tail = 0;
  volatile size_t read_head = 0;

public:
  CircularBuffer(uint8_t * buffer, size_t max_size)
    : buffer(buffer), max_size(max_size) {}

  void reset() {
    write_head = 0;
    read_tail = 0;
    read_head = 0;
  }

  uint8_t * reserve_write_head(size_t size) {
    if (write_head < read_tail) {
      // We are in wrap around mode.

      // Check if there's anything reserved for reading.
      if (read_head < read_tail) {
        // Reading is reserved between read_head and read_tail.
      
        // We can only write if we have enough space before the read head.
        return (write_head + size <= read_head) ? &buffer[write_head] : nullptr;
      } else {

        // Reading was done, reset the read pointers to what was written in the wrap around.
        read_tail = write_head;
        read_head = 0;

        return (write_head + size <= max_size) ? &buffer[write_head] : nullptr;
      }


    } else {
      // We are after the read area so there is space until the end of the buffer.
      // `write_head >= read_tail`

      // If reading was done, we can reset the whole buffer ahead of time.
      if (read_head >= read_tail) reset();

      // We have a choice to write at the end or wrap around to the start.
      if (write_head + size <= max_size) {
        // We have enough space at the end of the buffer.
        return &buffer[write_head];

      } else if (size <= read_head) {
        // We have enough space at the start of the buffer; but we need to mark the wrap around.
        write_head = 0;
        return &buffer[0];

      } else {

        // Not enough space to write the data.
        return nullptr;
      }
    }
  }

  uint8_t mark_write(size_t size){
    // Compute tail of the written data.
    const size_t write_tail = write_head + size;
  
    // Check whether we are writing after the read area or in the wrap around area.
    if (write_head < read_tail) {
      // We are in the wrap around area.
  
      // Ensure we did not overwrite unread data.
      if (write_tail > read_head) return 1;
  
      // At this point `write_tail <= read_head`.

      if (read_head >= read_tail) {
        // We have finished reading already, reset the read head to the data written so far.

        read_tail = write_tail;
        read_head = 0;
      }
  
      // Only advance the write head, leave the read tail to be reset by a complete read.
      write_head = write_tail;
      return 0;
    } else {
      // We are writing after the read area, no risk of overwriting unread data.
  
      // Ensure we do not exceed the buffer size.
      if (write_tail > max_size) return 1;
  
      // Mark the data for reading.
      read_tail = write_tail;
  
      // Wrap around the write head if we have reached the end of the buffer.
      write_head = (write_tail < max_size) ? write_tail : 0;
      return 0;
    }
  }
  
  size_t available_to_read() {
    // Since read tail is reset before the read head we should cap our result to 0 in case.
    const int readable_size = read_tail - read_head;
    return (readable_size > 0) ? readable_size : 0;
  }
  
  uint8_t * get_read_head() {
    return &buffer[read_head];
  }

  uint8_t mark_read(size_t size) {
    // Check how much data is available to read.
    const size_t available_size = available_to_read();
  
    // Ensure we are not trying to read more than available.
    if (size > available_size) return 1;
  
    // Advance the read head.
    read_head += size;
  
    return 0;
  }
};





