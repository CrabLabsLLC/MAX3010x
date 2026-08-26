# Copyright (c) 2025 Crab Labs LLC
# SPDX-License-Identifier: MIT

if(CONFIG_MAX3010X)
  # Export public headers for applications that include the module.
  zephyr_include_directories(include)
  if(TARGET drivers__sensor)
    target_sources(drivers__sensor PRIVATE src/max3010x.c)
    target_include_directories(drivers__sensor PRIVATE include)
  else()
    zephyr_library()
    zephyr_library_sources(src/max3010x.c)
    zephyr_library_include_directories(include)
  endif()
endif()
