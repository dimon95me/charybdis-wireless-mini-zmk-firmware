/*
 * Copyright (c) 2026 The Keyboard Project
 *
 * SPDX-License-Identifier: MIT
 *
 * Custom ZMK behavior "sensor_reset": triggers a full PMW3610 power-up reset
 * + reconfigure on keypress (bound via combo 23+35 in combos.dtsi).
 *
 * Purpose: the trackball sensor is physically on the edge of its focus
 * envelope and its internal tracking state degrades over time (cursor speed
 * creeps up). This behavior gives the user an instant manual reset without
 * reflashing — the same async reinit path used by stuck-motion recovery and
 * adaptive reinit in pmw3610.c.
 *
 * Safe to call from the key event context: pmw3610_request_reset() only
 * schedules a k_work item and returns immediately.
 */

#define DT_DRV_COMPAT zmk_behavior_sensor_reset

#include <zephyr/device.h>
#include <drivers/behavior.h>
#include <zephyr/logging/log.h>

#include <zmk/behavior.h>

#include "pmw3610.h"

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

static int sensor_reset_binding_pressed(struct zmk_behavior_binding *binding,
                                        struct zmk_behavior_binding_event event)
{
#if DT_NODE_EXISTS(DT_NODELABEL(trackball))
    const struct device *sensor = DEVICE_DT_GET(DT_NODELABEL(trackball));
    if (device_is_ready(sensor)) {
        pmw3610_request_reset(sensor);
    } else {
        LOG_WRN("Trackball sensor not ready; sensor reset ignored");
    }
#else
    LOG_WRN("Sensor reset requested but no trackball in this build");
#endif
    return ZMK_BEHAVIOR_OPAQUE;
}

static int sensor_reset_binding_released(struct zmk_behavior_binding *binding,
                                         struct zmk_behavior_binding_event event)
{
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api sensor_reset_driver_api = {
    .binding_pressed = sensor_reset_binding_pressed,
    .binding_released = sensor_reset_binding_released,
};

BEHAVIOR_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL, POST_KERNEL,
                        CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &sensor_reset_driver_api);
