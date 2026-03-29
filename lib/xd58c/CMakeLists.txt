# Copyright (c) 2021, Legrand North America, LLC.
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_XD58C)
    zephyr_library()
    zephyr_library_sources(xd58c.c)
endif()