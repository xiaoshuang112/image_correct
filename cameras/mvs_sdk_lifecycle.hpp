#ifndef MVS_SDK_LIFECYCLE_HPP
#define MVS_SDK_LIFECYCLE_HPP

#include <string>

// Process-wide MVS SDK ownership so multiple MVSCamera instances can share
// Initialize/Finalize. The last release calls Finalize.
namespace mvs_sdk {

struct AcquireResult {
    bool ok = false;
    unsigned int sdk_code = 0;
    std::string message;
};

AcquireResult acquire();
void release();

}  // namespace mvs_sdk

#endif  // MVS_SDK_LIFECYCLE_HPP
