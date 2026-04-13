FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)

idf_component_register(
  SRCS ${app_sources}
  INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/include"
  REQUIRES esp_lcd esp_wifi wpa_supplicant nvs_flash qrcode wifi_provisioning esp_http_server esp_https_server mdns lvgl
  EMBED_FILES "certs/cacert.pem" "certs/prvtkey.pem"
)

target_compile_options(${COMPONENT_LIB} PRIVATE 
  "-DLV_CONF_INCLUDE_SIMPLE"
  "-Wno-missing-field-initializers"
  "-I${CMAKE_SOURCE_DIR}/../interface/dist"
)

target_add_binary_data(${COMPONENT_TARGET} "certs/cacert.pem" TEXT)
target_add_binary_data(${COMPONENT_TARGET} "certs/prvtkey.pem" TEXT)