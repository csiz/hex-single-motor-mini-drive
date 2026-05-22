#include "parameters_store.hpp"

#include "hex_mini_drive_interface.hpp"

#include "constants.hpp"
#include "error_handler.hpp"

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_flash.h"
#include "stm32g4xx_hal_flash_ex.h"
#include "type_definitions.hpp"

#include <cstring>

using namespace hex_mini_drive;

// Leave uninitialized! It references the user_data section in flash memory. The STM32
// automatically retrieves data from flash when these memory locations are accessed.
uint8_t user_data[user_data_size];

uint8_t page_buffer[FLASH_PAGE_SIZE] = {0};

const size_t current_calibration_offset = 0x00;
const size_t position_calibration_offset = 0x20;
const size_t control_parameters_offset = 0xF0;

const uint8_t * const current_calibration_address = user_data + current_calibration_offset;
const uint8_t * const position_calibration_address = user_data + position_calibration_offset;
const uint8_t * const control_parameters_address = user_data + control_parameters_offset;


CurrentCalibration get_current_calibration(){
    Message stored_data;
    read_message(stored_data, current_calibration_address, message_size(CURRENT_CALIBRATION));
    if (stored_data.message_code == MessageCode::CURRENT_CALIBRATION) {
        return std::get<CurrentCalibration>(stored_data.message_data);
    } else {
        return default_current_calibration;
    }
}
HallPositions get_position_calibration(){
    Message stored_data;
    read_message(stored_data, position_calibration_address, message_size(HALL_POSITIONS));
    if (stored_data.message_code == MessageCode::HALL_POSITIONS) {
        return std::get<HallPositions>(stored_data.message_data);
    } else {
        return default_position_calibration;
    }
}

ControlParameters get_control_parameters() {
    Message stored_data;
    read_message(stored_data, control_parameters_address, message_size(CONTROL_PARAMETERS));
    if (stored_data.message_code == MessageCode::CONTROL_PARAMETERS) {
        return std::get<ControlParameters>(stored_data.message_data);
    } else {
        return default_control_parameters;
    }
}

/**
  * @brief  Gets the page of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The page of a given address
  */
static uint32_t get_page(uint32_t Addr)
{
  uint32_t page = 0;

  if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
  {
    /* Bank 1 */
    page = (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;
  }
  else
  {
    /* Bank 2 */
    page = (Addr - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
  }

  return page;
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
        .Page = get_page(reinterpret_cast<uint32_t>(flash_address)),
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
    HallPositions const& position_calibration,
    ControlParameters const& control_parameters
) {
    write_message(page_buffer + current_calibration_offset, message_size(CURRENT_CALIBRATION), Message{
        .message_code = MessageCode::CURRENT_CALIBRATION,
        .message_data = current_calibration
    });

    write_message(page_buffer + position_calibration_offset, message_size(HALL_POSITIONS), Message{
        .message_code = MessageCode::HALL_POSITIONS,
        .message_data = position_calibration
    });

    write_message(page_buffer + control_parameters_offset, message_size(CONTROL_PARAMETERS), Message{
        .message_code = MessageCode::CONTROL_PARAMETERS,
        .message_data = control_parameters
    });

    // Write the buffer to flash memory.
    if(write_to_flash(reinterpret_cast<uint32_t *>(user_data), page_buffer, FLASH_PAGE_SIZE) != HAL_OK) {
        error();
    }
}
