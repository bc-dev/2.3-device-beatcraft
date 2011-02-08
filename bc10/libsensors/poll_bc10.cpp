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

static float one_rad = 180 / M_PI;

/*****************************************************************************/

/*
 * Poll context
 */

struct sensors_poll_context_t {
    struct sensors_poll_device_t device; // must be first

        sensors_poll_context_t();
        ~sensors_poll_context_t();
    int activate(int handle, int enabled);
    int setDelay(int handle, int64_t ns);
    int pollEvents(sensors_event_t* data, int count);

private:
    struct pollfd mPollFd;
    int stat;
    struct ami602_position pos;
    sensors_event_t event[MAX_NUM_SENSORS];

    int64_t getTimeNano();
};

sensors_poll_context_t::sensors_poll_context_t()
{
    mPollFd.fd = open(AMI602_DEV, O_RDWR);
    mPollFd.events = POLLIN;
    mPollFd.revents = 0;
    stat = 0;

    memset(event, 0x0, sizeof(event));

    event[0].version = sizeof(sensors_event_t);
    event[0].sensor = ID_A;
    event[0].type = SENSOR_TYPE_ACCELEROMETER;
    event[0].acceleration.status = SENSOR_STATUS_ACCURACY_HIGH;

    event[1].version = sizeof(sensors_event_t);
    event[1].sensor = ID_M;
    event[1].type = SENSOR_TYPE_MAGNETIC_FIELD;
    event[1].acceleration.status = SENSOR_STATUS_ACCURACY_HIGH;

    event[2].version = sizeof(sensors_event_t);
    event[2].sensor = ID_O;
    event[2].type = SENSOR_TYPE_ORIENTATION;
    event[2].acceleration.status = SENSOR_STATUS_ACCURACY_HIGH;
}

sensors_poll_context_t::~sensors_poll_context_t() {
    close(mPollFd.fd);
}

int sensors_poll_context_t::activate(int handle, int enabled) {
#if 0
    int err =  mSensors[index]->enable(handle, enabled);
    if (enabled && !err) {
        LOGE("error sending wake message (%s)", strerror(errno));
    }
    return err;
#else
    return 0;
#endif
}

int sensors_poll_context_t::setDelay(int handle, int64_t ns) {

    return 0;
}

int sensors_poll_context_t::pollEvents(sensors_event_t* data, int count)
{
    int cnt = count;
    int num = 0;
    int ret;

    while (cnt) {
        if (stat <= 0) {
            usleep(50000);
            ret = ioctl(mPollFd.fd, AMI602_IOCPOSITION, &pos);
            if (ret < 0) {
                LOGE("%s: ret=%d", __FUNCTION__, ret);
                return -1;
            }
            stat = MAX_NUM_SENSORS;
            //      ID_ACCELERATION

            // Original formula
            // sensors[0].acceleration.x = (((float)(pos.accel_x - 2048) * GRAVITY_EARTH * -1.0) / 800.0f );
            // sensors[0].acceleration.y = (((float)(pos.accel_y - 2048) * GRAVITY_EARTH ) / 800.0f );
            // sensors[0].acceleration.z = (((float)(pos.accel_z - 2048) * GRAVITY_EARTH * -1.0) / 800.0f );
            event[0].acceleration.x = 25.1f - (float)pos.accel_x * 0.01225f;
            event[0].acceleration.y = (float)pos.accel_y * 0.01225f - 25.1f;
            event[0].acceleration.z = 25.1f - (float)pos.accel_z * 0.01225f;

            //  ID_MAGNETIC_FIELD
            //  AMI602 value: 1gauss = 600, 1uT = 6
            event[1].magnetic.x = (((float)(pos.mag_x - 2048) / 6.0f));
            event[1].magnetic.y = (((float)(pos.mag_y - 2048) / 6.0f));
            event[1].magnetic.z = (((float)(pos.mag_z - 2048) / 6.0f));

            //  ID_ORIENTATION
            event[2].orientation.azimuth = atan2( (event[1].magnetic.y * -1),  event[1].magnetic.x) * one_rad + 180;
            event[2].orientation.pitch   = atan2( (event[0].acceleration.y * -1),  event[0].acceleration.z ) * one_rad;
            event[2].orientation.roll    = atan2( (event[0].acceleration.x * -1),  event[0].acceleration.z ) * one_rad;
        }

        if (stat > 0) {
            switch (stat) {
            case 1:
                *data = event[0];
                break;
            case 2:
                *data = event[1];
                break;
            case 3:
                *data = event[2];
                break;
            }
            data->timestamp = getTimeNano();
            data++;
            num++;
            stat--;
            cnt--;
        }
    }
    return num;
}

int64_t sensors_poll_context_t::getTimeNano()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000000 + ts.tv_nsec;
}

/*****************************************************************************/

static int poll__close(struct hw_device_t *dev)
{
    sensors_poll_context_t *ctx = (sensors_poll_context_t *)dev;
    if (ctx) {
        delete ctx;
    }
    return 0;
}

static int poll__activate(struct sensors_poll_device_t *dev,
        int handle, int enabled) {
    sensors_poll_context_t *ctx = (sensors_poll_context_t *)dev;
    return ctx->activate(handle, enabled);
}

static int poll__setDelay(struct sensors_poll_device_t *dev,
        int handle, int64_t ns) {
    sensors_poll_context_t *ctx = (sensors_poll_context_t *)dev;
    return ctx->setDelay(handle, ns);
}

static int poll__poll(struct sensors_poll_device_t *dev,
        sensors_event_t* data, int count) {
    sensors_poll_context_t *ctx = (sensors_poll_context_t *)dev;
    return ctx->pollEvents(data, count);
}

/*****************************************************************************/

int init_poll_bc10(const struct hw_module_t* module, const char* name,
        struct hw_device_t** device)
{
    int status = -EINVAL;

    sensors_poll_context_t *dev = new sensors_poll_context_t();
    memset(&dev->device, 0, sizeof(sensors_poll_device_t));

    dev->device.common.tag = HARDWARE_DEVICE_TAG;
    dev->device.common.version  = 0;
    dev->device.common.module   = const_cast<hw_module_t*>(module);
    dev->device.common.close    = poll__close;
    dev->device.activate        = poll__activate;
    dev->device.setDelay        = poll__setDelay;
    dev->device.poll            = poll__poll;

    *device = &dev->device.common;
    status = 0;
    return status;
}

