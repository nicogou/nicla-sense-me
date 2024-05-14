zephyr_library()
zephyr_include_directories(BHI260AP_Bosch)
zephyr_include_directories(BHI260AP_Bosch/firmware)

FILE(GLOB bhy2_sources BHI260AP_Bosch/*.c)
zephyr_library_sources(${bhy2_sources})
