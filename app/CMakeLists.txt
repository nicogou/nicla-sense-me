#-------------------------------------------------------------------------------
# Zephyr Example Application
#
# Copyright (c) 2021 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(app LANGUAGES C)

target_include_directories(app PRIVATE src)

add_subdirectory(src/app)
add_subdirectory(src/bluetooth)
add_subdirectory(src/bhi_common)
add_subdirectory(src/bhi_euler)
add_subdirectory(src/bhi_klio)
add_subdirectory(src/bhi_acc_gyro)
add_subdirectory(src/sensors)
add_subdirectory(src/nicla_zbus)
add_subdirectory(src/nicla_sd)

target_sources(app PRIVATE src/main.c)
