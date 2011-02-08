/*
 * Copyright (C) 2008 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <math.h>
#include <errno.h>
#include <string.h>
#include <poll.h>

#include <cutils/log.h>
#include <cutils/native_handle.h>
#include <cutils/sockets.h>

#include <hardware/sensors.h>
#include "ami602.h"
#include "poll_bc10.h"

/*****************************************************************************/

/*
 * The SENSORS Module
 */

static const struct sensor_t sSensorList[] = {
        { "bc10 3-axis Accelerometer",
                "BeatCraft, Inc.",
                1, SENSORS_HANDLE_BASE+ID_A,
                SENSOR_TYPE_ACCELEROMETER, 2048.0f, 1.0f, 1.0f, 0, { } },
        { "bc10 3-axis Magnetic field sensor",
                "BeatCraft, Inc.",
                1, SENSORS_HANDLE_BASE+ID_M,
                SENSOR_TYPE_MAGNETIC_FIELD, 2048.0f, 1.0f, 1.0f, 0, { } },
        { "bc10 Orientation sensor",
                "BeatCraft, Inc.",
                1, SENSORS_HANDLE_BASE+ID_O,
                SENSOR_TYPE_ORIENTATION, 360.0f, 1.0f, 1.0f, 0, { } },
};

static int open_sensors(const struct hw_module_t* module, const char* name,
    struct hw_device_t** device);

static int sensors__get_sensors_list(struct sensors_module_t* module,
    struct sensor_t const** list)
{
    *list = sSensorList;
    return MAX_NUM_SENSORS;
}

static struct hw_module_methods_t sensors_module_methods = {
    //.open = open_sensors
    open_sensors
};

const struct sensors_module_t HAL_MODULE_INFO_SYM = {
    .common = {
        .tag = HARDWARE_MODULE_TAG,
        .version_major = 1,
        .version_minor = 0,
        .id = SENSORS_HARDWARE_MODULE_ID,
        .name = "bc10 SENSORS Module",
        .author = "BeatCraft, Inc.",
        .methods = &sensors_module_methods,
    },
    .get_sensors_list = sensors__get_sensors_list
};

/*****************************************************************************/

static int open_sensors(const struct hw_module_t* module, const char* name,
        struct hw_device_t** device)
{
    return init_poll_bc10(module, name, device);
}
