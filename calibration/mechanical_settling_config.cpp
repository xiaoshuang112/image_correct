#include "mechanical_settling_config.hpp"

#include <atomic>

namespace {

std::atomic<int> g_mechanical_settling_ms{kDefaultMechanicalSettlingMs};

}  // namespace

int mechanicalSettlingWaitMs() {
    return g_mechanical_settling_ms.load(std::memory_order_relaxed);
}

bool setMechanicalSettlingWaitMs(int milliseconds) {
    if (milliseconds < kMinimumMechanicalSettlingMs ||
        milliseconds > kMaximumMechanicalSettlingMs) {
        return false;
    }
    g_mechanical_settling_ms.store(milliseconds, std::memory_order_relaxed);
    return true;
}
