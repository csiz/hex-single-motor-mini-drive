#include "interface.hpp"

#include "constants.hpp"
#include "motor_control.hpp"
#include "byte_handling.hpp"
#include "interrupts.hpp"

#include "main.h"
#include "usbd_cdc_if.h"


uint32_t readouts_to_send = 0;
bool readouts_allow_sending = true;



// Time of last USB packet sent; used to detect timeouts.
uint32_t usb_last_send = 0;

// Maximum time to wait for a USB packet to be sent.
const uint32_t USB_TIMEOUT = 500;


// Receive data
// ------------

// Number of invalid commands received.
uint32_t invalid_commands = 0;

const size_t usb_min_command_size = 8;
const size_t usb_max_command_size = 64;
// Buffer for the USB command in case we receive it in chunks.
uint8_t usb_command[usb_max_command_size] = {};

size_t usb_command_index = 0;
size_t usb_bytes_expected = usb_min_command_size;




const size_t state_readout_size = 18;

void write_state_readout(uint8_t* buffer, const StateReadout& readout) {
    size_t offset = 0;
    write_uint16(buffer + offset, READOUT);
    offset += 2;
    write_uint16(buffer + offset, readout.readout_number);
    offset += 2;
    write_uint16(buffer + offset, readout.position);
    offset += 2;
    write_uint32(buffer + offset, readout.pwm_commands);
    offset += 4;
    write_uint16(buffer + offset, readout.u_readout);
    offset += 2;
    write_uint16(buffer + offset, readout.v_readout);
    offset += 2;
    write_uint16(buffer + offset, readout.w_readout);
    offset += 2;
    write_uint16(buffer + offset, readout.ref_readout);
    offset += 2;
}


static inline void usb_receive(){
    usb_command_index += usb_com_recv(usb_command + usb_command_index, usb_bytes_expected);

    // Wait for more data if we don't have a full command yet.
    if (usb_command_index < usb_bytes_expected) return;
        

    // The first number is the command code, the second is the data; if any.
    size_t offset = 0;
    const uint16_t command = read_uint16(&usb_command[offset]);
    offset += 2;
    const uint16_t timeout = read_uint16(&usb_command[offset]);
    offset += 2;
    const uint16_t pwm = read_uint16(&usb_command[offset]);
    offset += 2;
    const uint16_t leading_angle = read_uint16(&usb_command[offset]);
    offset += 2;



    switch (command) {
        // Send the whole history buffer over USB.
        case GET_READOUTS:
            // Clear the queue of older data.
            xQueueReset(readouts_queue);
            readouts_to_send = timeout;
            break;

        case GET_READOUTS_SNAPSHOT:
            xQueueReset(readouts_queue);
            // Dissalow sending until we fill the queue, so it doesn't interrupt commutation.
            readouts_allow_sending = false;
            readouts_to_send = HISTORY_SIZE;
            break;

        // Turn off the motor driver.
        case SET_STATE_OFF:
            motor_break();
            break;
            
        // Measure the motor phase currents.
        
        case SET_STATE_TEST_ALL_PERMUTATIONS:
            motor_start_test(test_all_permutations);
            break;

        case SET_STATE_TEST_GROUND_SHORT:
            motor_start_test(test_ground_short);
            break;

        case SET_STATE_TEST_POSITIVE_SHORT:
            motor_start_test(test_positive_short);
            break;

        case SET_STATE_TEST_U_DIRECTIONS:
            motor_start_test(test_u_directions);
            break;

        case SET_STATE_TEST_U_INCREASING:
            motor_start_test(test_u_increasing);
            break;
        case SET_STATE_TEST_U_DECREASING:
            motor_start_test(test_u_decreasing);
            break;
        case SET_STATE_TEST_V_INCREASING:
            motor_start_test(test_v_increasing);
            break;
        case SET_STATE_TEST_V_DECREASING:
            motor_start_test(test_v_decreasing);
            break;
        case SET_STATE_TEST_W_INCREASING:
            motor_start_test(test_w_increasing);
            break;
        case SET_STATE_TEST_W_DECREASING:
            motor_start_test(test_w_decreasing);
            break;

        // Drive the motor.
        case SET_STATE_DRIVE_POS:
            motor_drive_pos(pwm, timeout);
            break;
        case SET_STATE_DRIVE_NEG:
            motor_drive_neg(pwm, timeout);
            break;

        case SET_STATE_DRIVE_SMOOTH_POS:
            motor_drive_smooth_pos(pwm, timeout, leading_angle);
            break;
        case SET_STATE_DRIVE_SMOOTH_NEG:
            motor_drive_smooth_neg(pwm, timeout, leading_angle);
            break;

        // Freewheel the motor.
        case SET_STATE_FREEWHEEL:
            motor_freewheel();
            break;

        case SET_STATE_HOLD_U_POSITIVE:
            motor_hold(pwm, 0, 0, timeout);
            break;

        case SET_STATE_HOLD_V_POSITIVE:
            motor_hold(0, pwm, 0, timeout);
            break;

        case SET_STATE_HOLD_W_POSITIVE:
            motor_hold(0, 0, pwm, timeout);
            break;

        case SET_STATE_HOLD_U_NEGATIVE:
            motor_hold(0, pwm, pwm, timeout);
            break;

        case SET_STATE_HOLD_V_NEGATIVE:
            motor_hold(pwm, 0, pwm, timeout);
            break;

        case SET_STATE_HOLD_W_NEGATIVE:
            motor_hold(pwm, pwm, 0, timeout);
            break;

        default:
            // Invalid command; ignore it.
            invalid_commands += 1;
            // Flush the recv buffer once to remove any residual data; but ignore the data.
            usb_com_recv(usb_command, usb_min_command_size);
            break;
    }

    // Reset the end of the command buffer; to prepare for the next command.
    usb_command_index = 0;
    usb_bytes_expected = usb_min_command_size;
}

static inline void usb_queue_readouts(){
    // Send as many readouts as requested.
    for (size_t i = 0; readouts_to_send > 0 and i < HISTORY_SIZE; i++){
        StateReadout readout;

        // Skip if we have no data left to read.
        if (xQueuePeek(readouts_queue, &readout, 0) != pdTRUE) break;

        // Send the readout to the host.
        uint8_t readout_data[state_readout_size] = {0};
        write_state_readout(readout_data, readout);
        
        if(usb_com_queue_send(readout_data, state_readout_size) == 0){
            // We successfully added the readout to the USB buffer.
            readouts_to_send -= 1;
            usb_last_send = HAL_GetTick();
            // Discard the sent readout.
            xQueueReceive(readouts_queue, &readout, 0);

        } else {
            // The USB buffer is full; stop sending until there's space.
            if (HAL_GetTick() > usb_last_send + USB_TIMEOUT) {
                // The USB controller is not reading data; stop sending and clear the USB buffer.
                readouts_to_send = 0;
                usb_com_reset();
            }
            // Stop sending until the USB buffer is free again.
            break;
        }
    }
}

void usb_tick(){
    // Send USB data from the buffer.
    usb_com_send();

    usb_receive();

    // Queue the state readouts on the USB buffer.
    if (readouts_allow_sending) usb_queue_readouts();
    
    // Send USB data from the buffer, twice per loop, hopefully the USB module sends 64 byte packets while we compute.
    usb_com_send();
}

