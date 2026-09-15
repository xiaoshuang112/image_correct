#include "test_check.hpp"

#include "calibration/mechanical_settling_config.hpp"

namespace suite {

int test_mechanical_settling_config() {
    std::cout << "---- mechanical_settling_config ----\n";
    TEST_CHECK(mechanicalSettlingWaitMs() == kDefaultMechanicalSettlingMs,
               "settling wait starts with default");
    TEST_CHECK(setMechanicalSettlingWaitMs(2500) &&
                   mechanicalSettlingWaitMs() == 2500,
               "settling wait accepts runtime update");
    TEST_CHECK(!setMechanicalSettlingWaitMs(-1) &&
                   mechanicalSettlingWaitMs() == 2500,
               "settling wait rejects negative value");
    TEST_CHECK(!setMechanicalSettlingWaitMs(kMaximumMechanicalSettlingMs + 1) &&
                   mechanicalSettlingWaitMs() == 2500,
               "settling wait rejects excessive value");
    TEST_CHECK(setMechanicalSettlingWaitMs(kDefaultMechanicalSettlingMs),
               "settling wait restores default");
    return testutil::takeFailures();
}

}  // namespace suite
