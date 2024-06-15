FILE(GLOB sensors_sources *.c)
target_sources(app PRIVATE ${sensors_sources})

add_subdirectory(imu)
