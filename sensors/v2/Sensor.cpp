/*
 * Copyright (C) 2019 The Android Open Source Project
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

#include "Sensor.h"

#include <hardware/sensors.h>
#include <log/log.h>
#include <utils/SystemClock.h>

#include <cmath>
#include <linux/input.h>
#include <cstring>
#include <dirent.h>
#include <unistd.h>

// Define missing key codes for fingerprint detection
#ifndef KEY_FINGERPRINT
#define KEY_FINGERPRINT 0x2A
#endif

#ifndef BTN_INFO
#define BTN_INFO 0x2A
#endif

namespace {

static bool readBool(int fd, bool seek) {
    char c;
    int rc;

    if (seek) {
        rc = lseek(fd, 0, SEEK_SET);
        if (rc) {
            ALOGE("failed to seek: %d", rc);
            return false;
        }
    }

    rc = read(fd, &c, sizeof(c));
    if (rc != 1) {
        ALOGE("failed to read bool: %d", rc);
        return false;
    }

    return c != '0';
}

static int openTouchInput() {
    int fd = -1;
    DIR* dir = opendir("/dev/input");

    if (dir != nullptr) {
        struct dirent* ent;

        while ((ent = readdir(dir)) != nullptr) {
            if (ent->d_type == DT_CHR) {
                std::string absolute_path = std::string("/dev/input/") + ent->d_name;
                char name[80] = {0};

                fd = open(absolute_path.c_str(), O_RDWR);
                if (fd < 0) {
                    continue;
                }

                if (ioctl(fd, EVIOCGNAME(sizeof(name) - 1), &name) > 0) {
                    if (strcmp(name, "fts_ts") == 0 || strcmp(name, "fts") == 0 || 
                        strcmp(name, "goodix_ts") == 0 || strcmp(name, "NVTCapacitiveTouchScreen") == 0 ||
                        strstr(name, "touch") != nullptr) {
                        ALOGI("Found touchscreen: %s at %s", name, absolute_path.c_str());
                        break;
                    }
                }

                close(fd);
                fd = -1;
            }
        }
        closedir(dir);
    }
    return fd;
}

}  // anonymous namespace

namespace android {
namespace hardware {
namespace sensors {
namespace V2_1 {
namespace subhal {
namespace implementation {

using ::android::hardware::sensors::V1_0::MetaDataEventType;
using ::android::hardware::sensors::V1_0::OperationMode;
using ::android::hardware::sensors::V1_0::Result;
using ::android::hardware::sensors::V1_0::SensorFlagBits;
using ::android::hardware::sensors::V1_0::SensorStatus;
using ::android::hardware::sensors::V2_1::Event;
using ::android::hardware::sensors::V2_1::SensorInfo;
using ::android::hardware::sensors::V2_1::SensorType;

Sensor::Sensor(int32_t sensorHandle, ISensorsEventCallback* callback)
    : mIsEnabled(false),
      mSamplingPeriodNs(0),
      mLastSampleTimeNs(0),
      mCallback(callback),
      mMode(OperationMode::NORMAL) {
    mSensorInfo.sensorHandle = sensorHandle;
    mSensorInfo.vendor = "The LineageOS Project";
    mSensorInfo.version = 1;
    constexpr float kDefaultMaxDelayUs = 1000 * 1000;
    mSensorInfo.maxDelay = kDefaultMaxDelayUs;
    mSensorInfo.fifoReservedEventCount = 0;
    mSensorInfo.fifoMaxEventCount = 0;
    mSensorInfo.requiredPermission = "";
    mSensorInfo.flags = 0;
    mRunThread = std::thread(startThread, this);
}

Sensor::~Sensor() {
    {
        std::unique_lock<std::mutex> lock(mRunMutex);
        mStopThread = true;
        mIsEnabled = false;
        mWaitCV.notify_all();
    }
    mRunThread.join();
}

const SensorInfo& Sensor::getSensorInfo() const {
    return mSensorInfo;
}

void Sensor::batch(int32_t samplingPeriodNs) {
    samplingPeriodNs =
            std::clamp(samplingPeriodNs, mSensorInfo.minDelay * 1000, mSensorInfo.maxDelay * 1000);

    if (mSamplingPeriodNs != samplingPeriodNs) {
        mSamplingPeriodNs = samplingPeriodNs;
        mWaitCV.notify_all();
    }
}

void Sensor::activate(bool enable) {
    std::lock_guard<std::mutex> lock(mRunMutex);
    if (mIsEnabled != enable) {
        mIsEnabled = enable;
        mWaitCV.notify_all();
    }
}

Result Sensor::flush() {
    if (!mIsEnabled) {
        return Result::BAD_VALUE;
    }

    Event ev;
    ev.sensorHandle = mSensorInfo.sensorHandle;
    ev.sensorType = SensorType::META_DATA;
    ev.u.meta.what = MetaDataEventType::META_DATA_FLUSH_COMPLETE;
    std::vector<Event> evs{ev};
    mCallback->postEvents(evs, isWakeUpSensor());

    return Result::OK;
}

void Sensor::startThread(Sensor* sensor) {
    sensor->run();
}

void Sensor::run() {
    std::unique_lock<std::mutex> runLock(mRunMutex);
    constexpr int64_t kNanosecondsInSeconds = 1000 * 1000 * 1000;

    while (!mStopThread) {
        if (!mIsEnabled || mMode == OperationMode::DATA_INJECTION) {
            mWaitCV.wait(runLock, [&] {
                return ((mIsEnabled && mMode == OperationMode::NORMAL) || mStopThread);
            });
        } else {
            timespec curTime;
            clock_gettime(CLOCK_REALTIME, &curTime);
            int64_t now = (curTime.tv_sec * kNanosecondsInSeconds) + curTime.tv_nsec;
            int64_t nextSampleTime = mLastSampleTimeNs + mSamplingPeriodNs;

            if (now >= nextSampleTime) {
                mLastSampleTimeNs = now;
                nextSampleTime = mLastSampleTimeNs + mSamplingPeriodNs;
                mCallback->postEvents(readEvents(), isWakeUpSensor());
            }

            mWaitCV.wait_for(runLock, std::chrono::nanoseconds(nextSampleTime - now));
        }
    }
}

bool Sensor::isWakeUpSensor() {
    return mSensorInfo.flags & static_cast<uint32_t>(SensorFlagBits::WAKE_UP);
}

std::vector<Event> Sensor::readEvents() {
    std::vector<Event> events;
    Event event;
    event.sensorHandle = mSensorInfo.sensorHandle;
    event.sensorType = mSensorInfo.type;
    event.timestamp = ::android::elapsedRealtimeNano();
    event.u.vec3.x = 0;
    event.u.vec3.y = 0;
    event.u.vec3.z = 0;
    event.u.vec3.status = SensorStatus::ACCURACY_HIGH;
    events.push_back(event);
    return events;
}

void Sensor::setOperationMode(OperationMode mode) {
    std::lock_guard<std::mutex> lock(mRunMutex);
    if (mMode != mode) {
        mMode = mode;
        mWaitCV.notify_all();
    }
}

bool Sensor::supportsDataInjection() const {
    return mSensorInfo.flags & static_cast<uint32_t>(SensorFlagBits::DATA_INJECTION);
}

Result Sensor::injectEvent(const Event& event) {
    Result result = Result::OK;
    if (event.sensorType == SensorType::ADDITIONAL_INFO) {
        // When in OperationMode::NORMAL, SensorType::ADDITIONAL_INFO is used to push operation
        // environment data into the device.
    } else if (!supportsDataInjection()) {
        result = Result::INVALID_OPERATION;
    } else if (mMode == OperationMode::DATA_INJECTION) {
        mCallback->postEvents(std::vector<Event>{event}, isWakeUpSensor());
    } else {
        result = Result::BAD_VALUE;
    }
    return result;
}

OneShotSensor::OneShotSensor(int32_t sensorHandle, ISensorsEventCallback* callback)
    : Sensor(sensorHandle, callback) {
    mSensorInfo.minDelay = -1;
    mSensorInfo.maxDelay = 0;
    mSensorInfo.flags |= SensorFlagBits::ONE_SHOT_MODE;
}

UdfpsSensor::UdfpsSensor(int32_t sensorHandle, ISensorsEventCallback* callback)
    : OneShotSensor(sensorHandle, callback), 
      mTouchFd(-1), 
      mScreenX(0), 
      mScreenY(0), 
      mFingerPressed(false),
      mShouldReportEvent(false) {
    
    mSensorInfo.name = "UDFPS Sensor";
    mSensorInfo.type = static_cast<SensorType>(static_cast<int32_t>(SensorType::DEVICE_PRIVATE_BASE) + 3);
    mSensorInfo.typeAsString = "org.lineageos.sensor.udfps";
    mSensorInfo.maxRange = 2048.0f;
    mSensorInfo.resolution = 1.0f;
    mSensorInfo.power = 0;
    mSensorInfo.flags |= SensorFlagBits::WAKE_UP;

    int rc = pipe(mWaitPipeFd);
    if (rc < 0) {
        mWaitPipeFd[0] = -1;
        mWaitPipeFd[1] = -1;
        ALOGE("failed to open wait pipe: %d", rc);
    }

    mPolls[0] = {
        .fd = mWaitPipeFd[0],
        .events = POLLIN,
    };

    mPolls[1] = {
        .fd = -1,
        .events = POLLIN,
    };
}

UdfpsSensor::~UdfpsSensor() {
    {
        std::unique_lock<std::mutex> lock(mRunMutex);
        mStopThread = true;
        mIsEnabled = false;
        mWaitCV.notify_all();
    }
    
    if (mRunThread.joinable()) {
        mRunThread.join();
    }
    
    if (mTouchFd >= 0) {
        close(mTouchFd);
        mTouchFd = -1;
    }
    
    if (mWaitPipeFd[0] >= 0) {
        close(mWaitPipeFd[0]);
    }
    if (mWaitPipeFd[1] >= 0) {
        close(mWaitPipeFd[1]);
    }
}

void UdfpsSensor::activate(bool enable) {
    std::lock_guard<std::mutex> lock(mRunMutex);
    if (mIsEnabled != enable) {
        mIsEnabled = enable;
        
        if (enable) {
            mTouchFd = openTouchInput();
            if (mTouchFd >= 0) {
                mPolls[1].fd = mTouchFd;
                ALOGI("UDFPS sensor enabled, touch device opened");
            } else {
                ALOGE("Failed to open touch device for UDFPS");
            }
        } else {
            if (mTouchFd >= 0) {
                close(mTouchFd);
                mTouchFd = -1;
                mPolls[1].fd = -1;
                ALOGI("UDFPS sensor disabled, touch device closed");
            }
        }
        
        mWaitCV.notify_all();
    }
}

void UdfpsSensor::setOperationMode(OperationMode mode) {
    Sensor::setOperationMode(mode);
    interruptPoll();
}

void UdfpsSensor::interruptPoll() {
    if (mWaitPipeFd[1] < 0) return;
    char c = '1';
    write(mWaitPipeFd[1], &c, sizeof(c));
}

void UdfpsSensor::run() {
    std::unique_lock<std::mutex> runLock(mRunMutex);
    
    while (!mStopThread) {
        if (!mIsEnabled || mMode == OperationMode::DATA_INJECTION || mTouchFd < 0) {
            mWaitCV.wait(runLock, [&] {
                return ((mIsEnabled && mMode == OperationMode::NORMAL && mTouchFd >= 0) || mStopThread);
            });
        } else {
            runLock.unlock();
            
            int rc = poll(mPolls, 2, -1);
            
            runLock.lock();
            
            if (rc < 0) {
                ALOGE("failed to poll: %d", rc);
                mStopThread = true;
                continue;
            }
            
            if (mPolls[1].revents & POLLIN) {
                struct input_event ev;
                ssize_t bytesRead = read(mTouchFd, &ev, sizeof(struct input_event));
                
                if (bytesRead == sizeof(struct input_event)) {
                    processInputEvent(ev);
                }
            } else if (mPolls[0].revents & POLLIN) {
                readBool(mWaitPipeFd[0], false);
            }
        }
    }
}

void UdfpsSensor::processInputEvent(const struct input_event& ev) {
    bool eventReported = false;
    
    // Handle fingerprint key events
    if (ev.type == EV_KEY) {
        bool isFingerprintKey = false;
        
        // Check for fingerprint-related key codes
        if (ev.code == KEY_FINGERPRINT) {
            isFingerprintKey = true;
        } else if (ev.code == BTN_INFO) {
            isFingerprintKey = true;
        } else if (ev.code >= 0x100 && ev.code <= 0x12F) {
            // BTN_* range
            isFingerprintKey = true;
        }
        
        if (isFingerprintKey) {
            bool pressed = (ev.value == 1);
            if (pressed != mFingerPressed) {
                mFingerPressed = pressed;
                sendFodEvent(pressed, mScreenX, mScreenY);
                eventReported = true;
            }
        }
    }
    
    // Handle touch coordinates
    if (ev.type == EV_ABS) {
        if (ev.code == ABS_MT_POSITION_X) {
            mScreenX = ev.value;
        } else if (ev.code == ABS_MT_POSITION_Y) {
            mScreenY = ev.value;
        } else if (ev.code == ABS_MT_TRACKING_ID) {
            if (ev.value >= 0 && !mFingerPressed) {
                mFingerPressed = true;
                sendFodEvent(true, mScreenX, mScreenY);
                eventReported = true;
            } else if (ev.value == -1 && mFingerPressed) {
                mFingerPressed = false;
                sendFodEvent(false, mScreenX, mScreenY);
                eventReported = true;
            }
        }
    }
    
    if (eventReported) {
        // Event stored, will be picked up by readEvents()
    }
}

void UdfpsSensor::sendFodEvent(bool pressed, int x, int y) {
    ALOGD("UDFPS %s at (%d, %d)", pressed ? "PRESS" : "RELEASE", x, y);
    
    mScreenX = x;
    mScreenY = y;
    mFingerPressed = pressed;
    mShouldReportEvent = true;
    
    interruptPoll();
}

std::vector<Event> UdfpsSensor::readEvents() {
    std::vector<Event> events;
    
    if (!mShouldReportEvent.load()) {
        return events;
    }
    
    Event event;
    event.sensorHandle = mSensorInfo.sensorHandle;
    event.sensorType = mSensorInfo.type;
    event.timestamp = ::android::elapsedRealtimeNano();
    
    event.u.data[0] = mScreenX;
    event.u.data[1] = mScreenY;
    event.u.data[2] = mFingerPressed ? 1 : 0;
    
    events.push_back(event);
    
    mShouldReportEvent = false;
    
    return events;
}

}  // namespace implementation
}  // namespace subhal
}  // namespace V2_1
}  // namespace sensors
}  // namespace hardware
}  // namespace android
