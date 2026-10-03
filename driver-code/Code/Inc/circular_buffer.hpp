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
    read_head = read_head + size;
  
    return 0;
  }
};


/* Circular buffer for sending generic data types between the main loop and an interrupt.

! The rule is that one side is always the reader and the other always the writer, each must
invoke only its own operations, never crossing the boundary.
*/
template<typename T, size_t N>
class CircularQueue{
  // The queue must have at least 2 elements to work.
  static_assert(N >= 2, "CircularQueue requires N > 2");

  // Data store; initialized to the default.
  T data[N] = {};

  // Index where we have to write, if there is space available.
  volatile size_t write_index = 0;
  // Index of data that we have finished reading.
  volatile size_t read_index = 0;

  public:

  /* Get the amount of space available for writing. */
  size_t write_available(){
    // We can write up to read_index - 1, only the reading loop can push read_index to be equal to write_index.
    return (read_index + N - 1 - write_index) % N;
  }

  /* Get the amount of contiguous space available for writing. */
  size_t contiguous_write_available() {
    // We must implement this operation with a single access to read_index.
    const size_t copy_of_read_index = read_index;
    if (write_index >= copy_of_read_index) {
      return N - write_index;
    } else {
      return copy_of_read_index - write_index;
    }
  }

  /* Get the pointer the the write_index. */
  T * get_write_pointer() {
    return &data[write_index];
  }

  /* Commit written data of the specified size. */
  void mark_written(size_t size) {
    write_index = (write_index + size) % N;
  }
  
  /* Write an element if space is available. */
  bool write_if_available(T const& element) {
    // Check if there is space available for writing.
    if (write_available() == 0) return false;
    // There is at least one space; write it there.
    data[write_index] = element;
    mark_written(1);
    return true;
  }

  /* Get active read element. */
  T const& peek() {
    return data[read_index];
  }

  /* Get modifiable pointer to the read buffer; use with contiguous_read_available() to get the size. */
  T * get_read_pointer() {
    return &data[read_index];
  }

  /* Get the amount of contiguous data available for reading. */
  size_t contiguous_read_available() {
    // We must write this operation with a single access to write_index.
    const size_t copy_of_write_index = write_index;
    // Note! Make this check identical to the one for writing.
    if (copy_of_write_index >= read_index) {
      return copy_of_write_index - read_index;
    } else {
      return N - read_index;
    }
  }

  size_t read_available() {
    return (write_index + N - read_index) % N;
  }

  /* Mark data as read. */
  void mark_read(size_t size = 1) {
    read_index = (read_index + size) % N;
  }

  bool read_if_available(T & element) {
    if (read_available() == 0) return false;
    element = data[read_index];
    mark_read(1);
    return true;
  }
};

