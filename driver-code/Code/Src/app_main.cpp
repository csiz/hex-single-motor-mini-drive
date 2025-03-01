#include "app_main.hpp"
#include "utils.hpp"
#include "io.hpp"
#include "motor_control.hpp"
#include "data.hpp"

#include <stm32f1xx_ll_adc.h>
#include <stm32f1xx_ll_tim.h>
#include <usbd_cdc_if.h>

#include <stdlib.h>
#include <math.h>



uint32_t main_loop_update_number = 0;


float main_loop_update_rate = 0.0f;
float adc_update_rate = 0.0f;
float tim1_update_rate = 0.0f;
float tim2_update_rate = 0.0f;
float tim2_cc1_rate = 0.0f;

uint32_t last_usb_send = 0;
const uint32_t USB_TIMEOUT = 500;

// TODO: enable DMA channel 1 for ADC1 reading temperature and voltage.
// TODO: also need to set a minium on time for MOSFET driving, too little 
// won't spin the motor at all.


// Setup ADC for reading motor phase currents.
void adc_init(){
    // We use the ADC1 and ADC2 in simulatenous mode to read the motor phase currents
    // a short time after we turn on the mosfets during each PWM cycle (the motors
    // are driven by TIM1). When the ADC is triggered in injected mode both ADC modules 
    // read the current for U and V phases first then W and reference voltage second.
    // 
    // The ADC sample time is 20cycles, so the total sampling period is 20/12MHz = 1.67us.
    // 
    // Use the TIM1 channel 4 to generate an event a short time after the update event.
    // This event is used to trigger the ADC to read the current from the motor phases.
    // 
    // Delay the ADC sampling time by 16/72MHz = 222ns; the ADC will sample for 7.5 cycles
    // (and convert for 12.5); then finally sample W and the reference. 7.5/12MHz = 625ns.
    // For reference, the ADC sample time is 20*72MHz/12MHz = 120 ticks of TIM1.

    // When counting down, this is triggered 14/2 cycles before the counter reaches 0, sampling 
    // current symmetrically around 0 for the 2 consecutive readings.
    LL_TIM_OC_SetCompareCH4(TIM1, PWM_BASE - 42);


    // Enable the ADC interrupt for the end of the injected sequence which reads motor current.
    LL_ADC_EnableIT_JEOS(ADC1);

    // Enable the ADCs and wait for them to settle.
    LL_ADC_Enable(ADC1);
    LL_ADC_Enable(ADC2);
    HAL_Delay(100);

    // Calibrate the ADCs.
    LL_ADC_StartCalibration(ADC1);
    while (LL_ADC_IsCalibrationOnGoing(ADC1)) {}
    LL_ADC_StartCalibration(ADC2);
    while (LL_ADC_IsCalibrationOnGoing(ADC2)) {}

    // Start ADC conversions from the external trigger in TIM1.
    LL_ADC_INJ_StartConversionExtTrig(ADC1, LL_ADC_INJ_TRIG_EXT_RISING);
    LL_ADC_INJ_StartConversionExtTrig(ADC2, LL_ADC_INJ_TRIG_EXT_RISING);
}

// Select which interrupts to handle.
void interrupts_init(){

    // TIM1 is used to generate the PWM signals for the motor phases, and to trigger the ADC.
    // 
    // Note that we don't need to set dead-time at this point; the gate driver FD6288T&Q has a 
    // built-in dead-time of 100-300ns. The MOSFETs AO4266E have turn on rise time of 8ns and 
    // turn off fall time of 22ns so we should be covered.

    // Disable TIM1 update interrupt; occuring every PWM cycle. We will use the ADC interrupt to update the motor control.
    LL_TIM_DisableIT_UPDATE(TIM1);
    // Disable TIM1 trigger interrupt; it's only triggered during resets.
    LL_TIM_DisableIT_TRIG(TIM1);
    // Disable commutation interrupts; 
    LL_TIM_DisableIT_COM(TIM1);


    // Enable TIM2 update interrupt. We need to count updates of TIM2 to measure slow motor 
    // speed in case the timer overflows before a toggle in the hall sensor.
    LL_TIM_EnableIT_UPDATE(TIM2);
    // Disable TIM2 trigger interrupt. It is triggered every reset and hall sensor toggle.
    LL_TIM_DisableIT_TRIG(TIM2);
    // Enable TIM2 channel 1 interrupt. It is triggered every hall sensor toggle; we can 
    // read the time from the last time the input changed.
    LL_TIM_EnableIT_CC1(TIM2);
    // Disable TIM2 channel 2 interrupt. We are ignoring the delayed trigger output from 
    // TIM2 as we will commutate faster than the hall sensor toggles. To use this effectively
    // we need to use the prescaler to slow the timer down; but then we lose resolution.
    LL_TIM_DisableIT_CC2(TIM2);

    // Disable TIM1 capture/compare update selection preload. This is used to update the
    // output modes of all channels at the same time when a commutation event is triggered.
    // We don't need this as we always enable PWM mode and update the compare register to 
    // modulate the PWM duty cycle.
    // 
    // This setting is mostly here to document that in order to trigger commutations on TRGI 
    // we also need to enable preload for the motor ouput channel mode registers.
    LL_TIM_CC_DisablePreload(TIM1);
    // Don't allow TIM1 commutations to be triggered by TIM2 hall sensor toggles.
    LL_TIM_CC_SetUpdate(TIM1, LL_TIM_CCUPDATESOURCE_COMG_ONLY);
    
}

void enable_timers(){
       
    // Reinitialize the timers; reset the counters and update registers. Because the timers
    // are setup with preload registers the values we write to them are stored in shadow registers
    // and only applied when the update event occurs (to any individual PWM cycle consistent). We
    // need to generate an update event to force the timers to load the new values.
    LL_TIM_GenerateEvent_UPDATE(TIM1);
    LL_TIM_GenerateEvent_UPDATE(TIM2);
    LL_TIM_GenerateEvent_UPDATE(TIM3);

    // Start the timers.
    LL_TIM_EnableCounter(TIM1);
    LL_TIM_EnableCounter(TIM2);
    LL_TIM_EnableCounter(TIM3);

    // Enable TIM1 channels used to drive the motor phases.
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1N);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2N);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3N);

    // Enable TIM1 channel 4 used to trigger the ADC.
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH4);
    // Enable the TIM1 outputs following a break or clock issue event.
    LL_TIM_EnableAllOutputs(TIM1);

    // Enable TIM2 channels 1 as the hall sensor timer between commutations.
    LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1);
}

void app_init() {
    data_init();

    // Setup PWM settings.
    LL_TIM_SetAutoReload(TIM1, PWM_AUTORELOAD); // 72MHz / 1536 / 2 = 23.4KHz

    // Set which interrupts to handle.
    interrupts_init();
    
    // Setup the ADC for reading motor phase currents, temperature and voltage.
    // This function will run an initial calibration of the chip's ADCs (takes a few ms).
    adc_init();
    
    // Enable the timers: TIM1, TIM2, TIM3;
    // and ouputs: TIM1_CH1, TIM1_CH1N, TIM1_CH2, TIM1_CH2N, TIM1_CH3, TIM1_CH3N, TIM1_CH4, TIM2_CH1.
    enable_timers();

    // Enable LED outputs: TIM2_CH4, TIM3_CH1, TIM3_CH2.
    enable_LED_channels();

    // Get initial hall sensor state.
    read_hall_sensors();
}



void write_state_readout(uint8_t* buffer, const UpdateReadout& readout) {
    size_t offset = 0;
    write_uint32(buffer + offset, READOUT);
    offset += 4;
    write_uint32(buffer + offset, readout.readout_number);
    offset += 4;
    write_uint32(buffer + offset, readout.pwm_commands);
    offset += 4;
    write_uint16(buffer + offset, readout.u_readout);
    offset += 2;
    write_uint16(buffer + offset, readout.v_readout);
    offset += 2;
    write_uint16(buffer + offset, readout.w_readout);
    offset += 2;
    write_uint16(buffer + offset, readout.ref_readout);
}


void usb_tick(){
    // Receive data
    // ------------
    uint8_t usb_command[8] = {0};

    size_t bytes_received = usb_com_recv(usb_command, 8);

    // Check if we have received a full command; or any data at all.
    if (bytes_received == 8) {
        // The first number is the command code, the second is the data; if any.
        const uint32_t command = read_uint32(&usb_command[0]);
        const uint16_t data_0 = read_uint16(&usb_command[4]);
        const uint16_t data_1 = read_uint16(&usb_command[6]);
        
        const uint16_t pwm = data_1 > PWM_MAX ? PWM_MAX : data_1;
        const uint16_t timeout = data_0 > MAX_TIMEOUT ? MAX_TIMEOUT : data_0;


        switch (command) {
            // Send the whole history buffer over USB.
            case GET_READOUTS:
                readouts_to_send = timeout;
                break;

            case GET_READOUTS_SNAPSHOT:
                {
                    UpdateReadout discard_readout = {};
                    while(xQueueReceive(readouts_queue, &discard_readout, 0) == pdTRUE) /* Discard all past readouts. */;
                }
                readouts_allow_missing = false;
                readouts_allow_sending = false;
                readouts_to_send = HISTORY_SIZE;
                break;

            // Turn off the motor driver.
            case SET_STATE_OFF:
                turn_motor_off();
                break;
                
            // Measure the motor phase currents.
            
            case SET_STATE_TEST_ALL_PERMUTATIONS:
                start_test(test_all_permutations);
                break;

            case SET_STATE_TEST_GROUND_SHORT:
                start_test(test_ground_short);
                break;

            case SET_STATE_TEST_POSITIVE_SHORT:
                start_test(test_positive_short);
                break;

            case SET_STATE_TEST_U_DIRECTIONS:
                start_test(test_u_directions);
                break;

            case SET_STATE_TEST_U_INCREASING:
                start_test(test_u_increasing);
                break;
            case SET_STATE_TEST_U_DECREASING:
                start_test(test_u_decreasing);
                break;
            case SET_STATE_TEST_V_INCREASING:
                start_test(test_v_increasing);
                break;
            case SET_STATE_TEST_V_DECREASING:
                start_test(test_v_decreasing);
                break;
            case SET_STATE_TEST_W_INCREASING:
                start_test(test_w_increasing);
                break;
            case SET_STATE_TEST_W_DECREASING:
                start_test(test_w_decreasing);
                break;

            // Drive the motor.
            case SET_STATE_DRIVE:
                drive_motor_3phase(pwm, timeout);
                break;
            case SET_STATE_DRIVE_2PHASE:
                drive_motor_2phase(pwm, timeout);
                break;

            case SET_STATE_HOLD_U_POSITIVE:
                hold_motor(pwm, 0, 0, timeout);
                break;

            case SET_STATE_HOLD_V_POSITIVE:
                hold_motor(0, pwm, 0, timeout);
                break;

            case SET_STATE_HOLD_W_POSITIVE:
                hold_motor(0, 0, pwm, timeout);
                break;

            case SET_STATE_HOLD_U_NEGATIVE:
                hold_motor(0, pwm, pwm, timeout);
                break;

            case SET_STATE_HOLD_V_NEGATIVE:
                hold_motor(pwm, 0, pwm, timeout);
                break;

            case SET_STATE_HOLD_W_NEGATIVE:
                hold_motor(pwm, pwm, 0, timeout);
                break;
        }
    }

    // Send data
    // ---------

    // Queue the state readouts on the USB buffer.
    if (readouts_allow_sending){
        UpdateReadout readout;

        for (size_t i = 0; i < HISTORY_SIZE; i++){
            // Skip if we have no data left to read.
            if (xQueuePeek(readouts_queue, &readout, 0) != pdTRUE) break;

            // Send as many readouts as requested.
            if (readouts_to_send > 0) {
                // Send the readout to the host.
                uint8_t readout_data[20] = {0};
                write_state_readout(readout_data, readout);
                
                if(usb_com_queue_send(readout_data, 20) == 0){
                    // We successfully added the readout to the USB buffer.
                    readouts_to_send -= 1;
                    last_usb_send = HAL_GetTick();
                } else {
                    // The USB buffer is full; stop sending until there's space.
                    if (HAL_GetTick() > last_usb_send + USB_TIMEOUT) {
                        // The USB controller is not reading data; stop sending and clear the USB buffer.
                        readouts_to_send = 0;
                        usb_com_reset();
                    }
                    // Break before we consume the readout.
                    break;
                }

            } else {
                // Reset to 0 just in case we doubly subtracted.
                readouts_to_send = 0;
                // No readouts left to send, re-enable overwriting in case we came out of test mode.
                readouts_allow_missing = true;
            }
            
            // Either we've sent or want to discard the last readout.
            xQueueReceive(readouts_queue, &readout, 0);
        }
    }
    
    // Send USB data from the buffer.
    usb_com_send();
}

void app_tick() {
    // Send USB data from the buffer, twice per loop, hopefully the USB module sends 64 byte packets while we compute.
    usb_com_send();

    // Update timing information.
    uint32_t milliseconds = HAL_GetTick();
    float seconds = milliseconds / 1000.f;
    
    main_loop_update_number += 1;
    main_loop_update_rate = main_loop_update_number / seconds;

    adc_update_rate = adc_update_number / seconds;
    tim1_update_rate = tim1_update_number / seconds;
    tim2_update_rate = tim2_update_number / seconds;
    tim2_cc1_rate = tim2_cc1_number / seconds;

    // Process the latest ADC readouts.
    calculate_motor_phase_currents_gated();

    // Show the current hall sensor state on the LEDs.
    set_LED_RGB_colours(hall_1 ? 0x80 : 0, hall_2 ? 0x40 : 0, hall_3 ? 0x80 : 0);

    update_motor_control();

    // Handle USB communication.
    usb_tick();
}


void show_error(){
    set_RED_LED(0xFF);
    Error_Handler();
}