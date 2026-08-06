# Sensor-reset behavior: compiled on every half that pulls in this module.
# It no-ops at runtime when the trackball node is absent (left peripheral half).
zephyr_library()
zephyr_library_sources(src/behavior_sensor_reset.c)
zephyr_include_directories(${APPLICATION_SOURCE_DIR}/include)

if(CONFIG_PMW3610_ALT)
  zephyr_library_sources(src/pmw3610.c)
endif()


