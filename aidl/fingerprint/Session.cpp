/*
 * Copyright (C) 2024-2025 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <thread>

#include "Legacy2Aidl.h"
#include "Session.h"

#include "CancellationSignal.h"

namespace aidl::android::hardware::biometrics::fingerprint {

void onClientDeath(void* cookie) {
    ALOGI("FingerprintService has died");
    Session* session = static_cast<Session*>(cookie);
    if (session && !session->isClosed()) {
        session->close();
    }
}

Session::Session(fingerprint_device_t* device, UdfpsHandler* udfpsHandler, int userId,
                 std::shared_ptr<ISessionCallback> cb, LockoutTracker lockoutTracker)
    : mDevice(device),
      mLockoutTracker(lockoutTracker),
      mUserId(userId),
      mCb(cb),
      mUdfpsHandler(udfpsHandler) {
    mDeathRecipient = AIBinder_DeathRecipient_new(onClientDeath);

    auto path = std::format("/data/vendor_de/{}/fpdata/", userId);
    mDevice->set_active_group(mDevice, mUserId, path.c_str());
}

// ... other Session methods ...

void Session::notify(const fingerprint_msg_t* msg) {
    // const uint64_t devId = reinterpret_cast<uint64_t>(mDevice);
    switch (msg->type) {
        case FINGERPRINT_ERROR: {
            int32_t vendorCode = 0;
            Error result = VendorErrorFilter(msg->data.error, &vendorCode);
            ALOGD("onError(%hhd, %d)", result, vendorCode);
            mCb->onError(result, vendorCode);
        } break;
        case FINGERPRINT_ACQUIRED: {
            int32_t vendorCode = 0;
            AcquiredInfo result =
                    VendorAcquiredFilter(msg->data.acquired.acquired_info, &vendorCode);
            ALOGD("onAcquired(%hhd, %d)", result, vendorCode);
            if (mUdfpsHandler) {
                mUdfpsHandler->onAcquired(static_cast<int32_t>(result), vendorCode);
            }
            // don't process vendor messages further since frameworks try to disable
            // udfps display mode on vendor acquired messages but our sensors send a
            // vendor message during processing...
            if (result != AcquiredInfo::VENDOR) {
                mCb->onAcquired(result, vendorCode);
            }
        } break;
        case FINGERPRINT_TEMPLATE_ENROLLING: {
            ALOGD("onEnrollResult(fid=%d, gid=%d, rem=%d)", msg->data.enroll.finger.fid,
                  msg->data.enroll.finger.gid, msg->data.enroll.samples_remaining);
            
            // ✅ ADD THIS: Notify UDFPS handler about enrollment progress
            if (mUdfpsHandler) {
                mUdfpsHandler->onEnrollmentProgress(
                    msg->data.enroll.finger.fid,
                    msg->data.enroll.samples_remaining
                );
            }
            
            mCb->onEnrollmentProgress(msg->data.enroll.finger.fid,
                                      msg->data.enroll.samples_remaining);
        } break;
        case FINGERPRINT_TEMPLATE_REMOVED: {
            ALOGD("onRemove(fid=%d, gid=%d, rem=%d)", msg->data.removed.finger.fid,
                  msg->data.removed.finger.gid, msg->data.removed.remaining_templates);
            std::vector<int> enrollments;
            enrollments.push_back(msg->data.removed.finger.fid);
            mCb->onEnrollmentsRemoved(enrollments);
        } break;
        case FINGERPRINT_AUTHENTICATED: {
            ALOGD("onAuthenticated(fid=%d, gid=%d)", msg->data.authenticated.finger.fid,
                  msg->data.authenticated.finger.gid);
            if (msg->data.authenticated.finger.fid != 0) {
                const hw_auth_token_t hat = msg->data.authenticated.hat;
                HardwareAuthToken authToken;
                translate(hat, authToken);

                if (mUdfpsHandler) {
                    mUdfpsHandler->onAuthenticationSucceeded();
                }
                mCb->onAuthenticationSucceeded(msg->data.authenticated.finger.fid, authToken);
                mLockoutTracker.reset(true);
            } else {
                if (mUdfpsHandler) {
                    mUdfpsHandler->onAuthenticationFailed();
                }
                mCb->onAuthenticationFailed();
                mLockoutTracker.addFailedAttempt();
                checkSensorLockout();
            }
        } break;
        case FINGERPRINT_TEMPLATE_ENUMERATING: {
            ALOGD("onEnumerate(fid=%d, gid=%d, rem=%d)", msg->data.enumerated.finger.fid,
                  msg->data.enumerated.finger.gid, msg->data.enumerated.remaining_templates);
            static std::vector<int> enrollments;
            enrollments.push_back(msg->data.enumerated.finger.fid);
            if (msg->data.enumerated.remaining_templates == 0) {
                mCb->onEnrollmentsEnumerated(enrollments);
                enrollments.clear();
            }
        } break;
    }
}

}  // namespace aidl::android::hardware::biometrics::fingerprint
