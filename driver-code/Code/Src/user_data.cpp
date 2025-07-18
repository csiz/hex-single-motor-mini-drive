#include "user_data.hpp"

#include "interface.hpp"
#include "byte_handling.hpp"
#include "constants.hpp"
#include "error_handler.hpp"

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_flash.h"
#include "stm32f1xx_hal_flash_ex.h"

// Leave uninitialized! It references the user_data section in flash memory. The STM32
// automatically retrieves data from flash when these memory locations are accessed.
uint8_t user_data[user_data_size];

uint8_t page_buffer[FLASH_PAGE_SIZE] = {0};

const size_t current_calibration_offset = 0x00;
const size_t spare_offset = 0x20;
const size_t pid_parameters_offset = 0x80;
const size_t control_parameters_offset = 0xF0;

CurrentCalibration get_current_calibration(){
    const bool calibration_available = (CURRENT_FACTORS == read_uint16(user_data + current_calibration_offset));
    return calibration_available ? 
        parse_current_calibration(user_data + current_calibration_offset, current_calibration_size) :
        default_current_calibration;
}

ControlParameters get_control_parameters() {
    const bool parameters_available = (CONTROL_PARAMETERS == read_uint16(user_data + control_parameters_offset));
    return parameters_available ?
        parse_control_parameters(user_data + control_parameters_offset, control_parameters_size) :
        default_control_parameters;
}

uint32_t write_to_flash(uint32_t * flash_address, uint8_t * data, size_t len)
{
    if (len > FLASH_PAGE_SIZE) error();
    if (len % 8 != 0) error();
    if (len == 0) return HAL_OK;

    uint32_t return_error = HAL_OK;
	uint32_t page_error = 0;

    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase_init_struct = {
        .TypeErase = FLASH_TYPEERASE_PAGES,
        .PageAddress = reinterpret_cast<uint32_t>(flash_address),
        .NbPages = 1,
    };

    if (HAL_FLASHEx_Erase(&erase_init_struct, &page_error) != HAL_OK) {
        return_error = HAL_FLASH_GetError();
    }

    size_t offset = 0;

    while (return_error == HAL_OK && offset < len) {
        uint32_t address = reinterpret_cast<uint32_t>(flash_address) + offset;
        uint64_t data_doubleword = *reinterpret_cast<uint64_t *>(data + offset);

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, data_doubleword) == HAL_OK) {
            offset += 8;
        } else {
            return_error = HAL_FLASH_GetError();
        }
    }

    // Relock flash.
    HAL_FLASH_Lock();

    return return_error;
}

void save_settings_to_flash(
    CurrentCalibration const& current_calibration, 
    ControlParameters const& control_parameters
) {
    // Write the current calibration to the buffer.
    write_current_calibration(page_buffer + current_calibration_offset, current_calibration);

    // Write the observer parameters to the buffer.
    write_control_parameters(page_buffer + control_parameters_offset, control_parameters);

    // Write the buffer to flash memory.
    if(write_to_flash(reinterpret_cast<uint32_t *>(user_data), page_buffer, FLASH_PAGE_SIZE) != HAL_OK) {
        error();
    }
}
