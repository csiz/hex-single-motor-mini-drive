#pragma once

#include <cstdint>
#include <cstddef>


// Interface command codes
// -----------------------

const uint16_t READOUT = 0x2020;
const uint16_t GET_READOUTS = 0x2021;
const uint16_t GET_READOUTS_SNAPSHOT = 0x2022;
const uint16_t SET_STATE_OFF = 0x2030;
const uint16_t SET_STATE_DRIVE_POS = 0x2031;
const uint16_t SET_STATE_TEST_ALL_PERMUTATIONS = 0x2032;
const uint16_t SET_STATE_DRIVE_NEG = 0x2033;
const uint16_t SET_STATE_FREEWHEEL = 0x2034;

const uint16_t SET_STATE_TEST_GROUND_SHORT = 0x2036;
const uint16_t SET_STATE_TEST_POSITIVE_SHORT = 0x2037;

const uint16_t SET_STATE_TEST_U_DIRECTIONS = 0x2039;
const uint16_t SET_STATE_TEST_U_INCREASING = 0x203A;
const uint16_t SET_STATE_TEST_U_DECREASING = 0x203B;
const uint16_t SET_STATE_TEST_V_INCREASING = 0x203C;
const uint16_t SET_STATE_TEST_V_DECREASING = 0x203D;
const uint16_t SET_STATE_TEST_W_INCREASING = 0x203E;
const uint16_t SET_STATE_TEST_W_DECREASING = 0x203F;

const uint16_t SET_STATE_HOLD_U_POSITIVE = 0x3020;
const uint16_t SET_STATE_HOLD_V_POSITIVE = 0x3021;
const uint16_t SET_STATE_HOLD_W_POSITIVE = 0x3022;
const uint16_t SET_STATE_HOLD_U_NEGATIVE = 0x3023;
const uint16_t SET_STATE_HOLD_V_NEGATIVE = 0x3024;
const uint16_t SET_STATE_HOLD_W_NEGATIVE = 0x3025;

const uint16_t SET_STATE_DRIVE_SMOOTH_POS = 0x4030;
const uint16_t SET_STATE_DRIVE_SMOOTH_NEG = 0x4031;


// Comms functions
// ---------------

// Receive a command from the USB interface.
void usb_receive_command();

// Queue readouts to be sent to the USB interface.
void usb_queue_readouts();