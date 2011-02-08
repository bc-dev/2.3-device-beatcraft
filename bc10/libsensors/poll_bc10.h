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

#ifndef ANDROID_SENSORS_H
#define ANDROID_SENSORS_H

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

#define MAX_NUM_SENSORS 3
#define AMI602_DEV "/dev/ami602"

#define ID_A  (0)
#define ID_M  (1)
#define ID_O  (2)

__BEGIN_DECLS

/*****************************************************************************/

int init_poll_bc10(const struct hw_module_t* module, const char* name,
        struct hw_device_t** device);

__END_DECLS

#endif 
