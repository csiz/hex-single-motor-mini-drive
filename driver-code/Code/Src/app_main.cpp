#include <stdlib.h>
#include <math.h>

#include <stm32f1xx_ll_adc.h>
#include <stm32f1xx_ll_tim.h>
#include <usbd_cdc_if.h>

#include "app_main.hpp"
#include "io.hpp"
#include "interrupts.hpp"
#include "utils.hpp"
#include "motor_control.hpp"

const uint32_t ADC_READOUT = 0x80202020;
const uint32_t GET_ADC_READOUTS = 0x80202021;



uint32_t main_loop_update_number = 0;


float main_loop_update_rate = 0.0f;
float adc_update_rate = 0.0f;
float tim1_update_rate = 0.0f;
float tim2_update_rate = 0.0f;
float tim2_cc1_rate = 0.0f;

uint32_t adc_updates_to_send = 0;
uint32_t adc_updates_index = 0;
size_t usb_misses = 0;




// TODO: enable DMA channel 1 for ADC1 reading temperature and voltage.
// TODO: also need to set a minium on time for MOSFET driving, too little 
// won't spin the motor at all.




void app_init() {
    interrupt_init();

    // Setup PWM settings.
    LL_TIM_SetAutoReload(TIM1, PWM_AUTORELOAD); // 72MHz / 1536 / 2 = 23.4KHz

    // ### Setup ADC for reading motor phase currents.
    //
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

    // When counting down, this is triggered 14cycles before the counter reaches 0, measuring 
    // current symmetrically around 0 for the 2 consecutive readings.
    LL_TIM_OC_SetCompareCH4(TIM1, 84); 


    // Enable the ADC interrupt for the end of the injected sequence which reads motor current.
    LL_ADC_EnableIT_JEOS(ADC1);

    // Enable the ADCs and wait for them to settle.
    LL_ADC_Enable(ADC1);
    LL_ADC_Enable(ADC2);
    HAL_Delay(1);

    // Calibrate the ADCs.
    LL_ADC_StartCalibration(ADC1);
    while (LL_ADC_IsCalibrationOnGoing(ADC1)) {}
    LL_ADC_StartCalibration(ADC2);
    while (LL_ADC_IsCalibrationOnGoing(ADC2)) {}

    // Start ADC conversions from the external trigger in TIM1.
    LL_ADC_INJ_StartConversionExtTrig(ADC1, LL_ADC_INJ_TRIG_EXT_RISING);
    LL_ADC_INJ_StartConversionExtTrig(ADC2, LL_ADC_INJ_TRIG_EXT_RISING);

    
    // ### Setup TIM1 for motor control.
    //
    // TIM1 is used to generate the PWM signals for the motor phases, and to trigger the ADC.
    // 
    // Note that we don't need to set dead-time at this point; the gate driver FD6288T&Q has a 
    // built-in dead-time of 100-300ns. The MOSFETs AO4266E have turn on rise time of 8ns and 
    // turn off fall time of 22ns so we should be covered.

    // Enable TIM1 update interrupt; occuring every PWM cycle.
    LL_TIM_EnableIT_UPDATE(TIM1);
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

    // Enable LED outputs: TIM2_CH4, TIM3_CH1, TIM3_CH2.
    enable_LED_channels();

    init_motor_position();
}



void write_adc_readout(uint8_t* buffer, const UpdateReadout& readout) {
    size_t offset = 0;
    write_uint32(buffer + offset, ADC_READOUT);
    offset += 4;
    write_uint32(buffer + offset, readout.readout_number);
    offset += 4;
    write_uint16(buffer + offset, readout.u_readout);
    offset += 2;
    write_uint16(buffer + offset, readout.v_readout);
    offset += 2;
    write_uint16(buffer + offset, readout.w_readout);
    offset += 2;
    write_uint16(buffer + offset, readout.ref_readout);
}

void app_tick() {
    uint32_t milliseconds = HAL_GetTick();
    float seconds = milliseconds / 1000.f;
    
    main_loop_update_number += 1;
    main_loop_update_rate = main_loop_update_number / seconds;

    adc_update_rate = adc_update_number / seconds;
    tim1_update_rate = tim1_update_number / seconds;
    tim2_update_rate = tim2_update_number / seconds;
    tim2_cc1_rate = tim2_cc1_number / seconds;

    // Overwrite history only when we're not sending data.
    bool overwrite_adc_history = adc_updates_to_send == 0;
    move_adc_readouts(overwrite_adc_history);

    update_motor_phase_currents();

    update_motor_control();

    set_LED_RGB_colours(hall_1 ? 0x80 : 0, hall_2 ? 0x40 : 0, hall_3 ? 0x80 : 0);

    uint8_t usb_command[8] = {0};

    size_t bytes_received = usb_com_recv(usb_command, 8);
    if (bytes_received == 8) {
        uint32_t command = usb_command[3] | (usb_command[2] << 8) | (usb_command[1] << 16) | (usb_command[0] << 24);
        uint32_t data = usb_command[7] | (usb_command[6] << 8) | (usb_command[5] << 16) | (usb_command[4] << 24);

        switch (command) {
            case GET_ADC_READOUTS:
                adc_updates_to_send = HISTORY_SIZE;
                UNUSED(data);
                break;
        }
    }

    while (adc_updates_to_send > 0) {
        const UpdateReadout readout = adc_readouts[adc_readouts_index];

        // Send the readout to the host.
        uint8_t readout_data[16] = {0};
        write_adc_readout(readout_data, readout);

        if(usb_com_queue_send(readout_data, 16) == 0){
            adc_updates_to_send -= 1;
            adc_readouts_index = (adc_readouts_index + 1) % HISTORY_SIZE;
        }else {
            break;
        }
    }

    // Send USB data from the buffer.
    usb_com_send();
}


void show_error(){
    set_RED_LED(0xFF);
}