#include "hand_eye_gui/ExposureList.hpp"
#include "test_check.hpp"

int main() {
    using collection::ExposureListError;
    std::vector<int> values{42};
    const auto accepts = [&](const QString& input, const std::vector<int>& expected) {
        TEST_CHECK(collection::parseExposureList(input, values) == ExposureListError::None &&
                   values == expected, "accepts_in_order: " + input.toStdString());
    };
    const auto rejects = [&](const QString& input, ExposureListError expected) {
        const auto before = values;
        TEST_CHECK(collection::parseExposureList(input, values) == expected && values == before,
                   "rejects_without_partial_output: " + input.toStdString());
    };

    accepts("1,1000000", {1, 1000000});
    accepts("1000000,1", {1000000, 1});
    accepts("25000,5000,14500,8500,1", {25000, 5000, 14500, 8500, 1});
    accepts(" ,，;\t\n5000，8500;14500\r\n25000\v\f1,，; ", {5000, 8500, 14500, 25000, 1});
    accepts("1,,;;，2,", {1, 2});
    accepts("+1,0002", {1, 2});

    for (const auto* input : {"", " ,，;\t\r\n", "1", "1,2,3,4,5,6"})
        rejects(QString::fromUtf8(input), ExposureListError::InvalidCount);
    for (const auto* input : {
             "0,2", "-1,2", "1000001,2", "2147483648,2", "-2147483649,2",
             "999999999999999999999999,2", "1,1", "01,+1", "1,2,3,4,1",
             "bad,2", "1.5,2", "1e3,2", "0x10,2", "NaN,2", "inf,2",
             "1:2", "bad", "1,2,3,4,5,6,bad", "1,2,3,4,5,1"})
        rejects(QString::fromUtf8(input), ExposureListError::InvalidValue);

    // A later successful parse replaces the previous sequence rather than appending.
    accepts("9,3", {9, 3});
    return testutil::takeFailures() == 0 ? 0 : 1;
}
