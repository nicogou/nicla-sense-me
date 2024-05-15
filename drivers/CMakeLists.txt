# Copyright (c) 2021 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

# Out-of-tree drivers for custom classes
add_subdirectory_ifdef(CONFIG_BLINK blink)

# Out-of-tree drivers for existing driver classes
add_subdirectory_ifdef(CONFIG_SENSOR sensor)

# Out-of-tree drivers for led drivers
add_subdirectory_ifdef(CONFIG_LED led)

# Out-of-tree drivers for charger drivers
add_subdirectory_ifdef(CONFIG_CHARGER charger)
