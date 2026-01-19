FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)

idf_component_register(
  SRCS ${app_sources}
  INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/include"
  REQUIRES esp_lcd esp_wifi wpa_supplicant nvs_flash qrcode
  )
