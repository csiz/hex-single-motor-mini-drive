#include "parameters_store.hpp"

#include "interface.hpp"
#include "byte_handling.hpp"
#include "constants.hpp"
#include "error_handler.hpp"

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_flash.h"
#include "stm32g4xx_hal_flash_ex.h"

// Leave uninitialized! It references the user_data section in flash memory. The STM32
// automatically retrieves data from flash when these memory locations are accessed.
uint8_t user_data[user_data_size];

uint8_t page_buffer[FLASH_PAGE_SIZE] = {0};

const size_t current_calibration_offset = 0x00;
const size_t position_calibration_offset = 0x20;
const size_t control_parameters_offset = 0xF0;

const uint8_t * const current_calibration_address = user_data + current_calibration_offset;
const uint8_t * const control_parameters_address = user_data + control_parameters_offset;


CurrentCalibration get_current_calibration(){
    const bool missing_calibration = (CURRENT_FACTORS != read_uint16(current_calibration_address));
    return missing_calibration ? 
        default_current_calibration :
        parse_current_calibration(current_calibration_address, current_calibration_size);
}
PositionCalibration get_position_calibration(){
    const bool missing_calibration = (HALL_POSITIONS != read_uint16(user_data + position_calibration_offset));
    return missing_calibration ? 
        default_position_calibration :
        parse_position_calibration(user_data + position_calibration_offset, position_calibration_size);
}

ControlParameters get_control_parameters() {
    const bool missing_parameters = (CONTROL_PARAMETERS != read_uint16(control_parameters_address));
    return missing_parameters ? 
        default_control_parameters :
        parse_control_parameters(control_parameters_address, control_parameters_size);
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
    PositionCalibration const& position_calibration,
    ControlParameters const& control_parameters
) {
    // Write the current calibration to the buffer.
    write_current_calibration(page_buffer + current_calibration_offset, current_calibration);

    // Write the position calibration to the buffer.
    write_position_calibration(page_buffer + position_calibration_offset, position_calibration);

    // Write the observer parameters to the buffer.
    write_control_parameters(page_buffer + control_parameters_offset, control_parameters);

    // Write the buffer to flash memory.
    if(write_to_flash(reinterpret_cast<uint32_t *>(user_data), page_buffer, FLASH_PAGE_SIZE) != HAL_OK) {
        error();
    }
}
