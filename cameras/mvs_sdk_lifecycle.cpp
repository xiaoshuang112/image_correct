#include "mvs_sdk_lifecycle.hpp"

#include "MvCameraControl.h"

#include <mutex>

namespace mvs_sdk {
namespace {

std::mutex g_mutex;
unsigned int g_refcount = 0;

}  // namespace

AcquireResult acquire() {
    std::lock_guard<std::mutex> lock(g_mutex);
    if (g_refcount == 0) {
        const int nRet = MV_CC_Initialize();
        if (MV_OK != nRet) {
            AcquireResult result;
            result.ok = false;
            result.sdk_code = static_cast<unsigned int>(nRet);
            result.message = "MV_CC_Initialize failed";
            return result;
        }
    }
    ++g_refcount;
    AcquireResult result;
    result.ok = true;
    return result;
}

void release() {
    std::lock_guard<std::mutex> lock(g_mutex);
    if (g_refcount == 0) return;
    --g_refcount;
    if (g_refcount == 0) MV_CC_Finalize();
}

unsigned int referenceCount() {
    std::lock_guard<std::mutex> lock(g_mutex);
    return g_refcount;
}

}  // namespace mvs_sdk
