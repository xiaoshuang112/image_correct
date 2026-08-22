// MVS 错误码 → 诊断信息转换的无硬件测试。
#include "test_check.hpp"

#include "cameras/mvs_error.hpp"

#include <string>

using namespace mvsdiag;

namespace suite {

int test_mvs_error() {
    std::cout << "---- mvs_error ----\n";

    TEST_CHECK(std::string(errorName(0x00000000u)) == "MV_OK", "name_ok");
    TEST_CHECK(std::string(errorName(0x80000000u)) == "MV_E_HANDLE",
               "name_handle");
    TEST_CHECK(std::string(errorName(0x80000203u)) == "MV_E_ACCESS_DENIED",
               "name_access_denied");
    TEST_CHECK(std::string(errorName(0x80000007u)) == "MV_E_NODATA",
               "name_nodata");
    TEST_CHECK(std::string(errorName(0x80000006u)) == "MV_E_RESOURCE",
               "name_resource");
    TEST_CHECK(std::string(errorName(0x80000002u)) == "MV_E_BUFOVER",
               "name_bufover");
    TEST_CHECK(std::string(errorName(0x8000000Au)) == "MV_E_NOENOUGH_BUF",
               "name_noenough_buf");
    TEST_CHECK(std::string(errorName(0x80000107u)) == "MV_E_GC_TIMEOUT",
               "name_gc_timeout");
    TEST_CHECK(std::string(errorName(0xDEADBEEFu)) == "MV_E_UNKNOWN_CODE",
               "name_unknown");

    // 已知码必须有中文含义，未知码含义为空
    TEST_CHECK(std::string(errorMeaning(0x80000203u)).size() > 0,
               "meaning_known_non_empty");
    TEST_CHECK(std::string(errorMeaning(0xDEADBEEFu)).empty(),
               "meaning_unknown_empty");

    // 统一诊断串格式：stage=xxx code=0x........(NAME) meaning=...
    const std::string d = diagnostic("open_device", 0x80000203u);
    TEST_CHECK(d.find("stage=open_device") != std::string::npos &&
                   d.find("0x80000203") != std::string::npos &&
                   d.find("MV_E_ACCESS_DENIED") != std::string::npos &&
                   d.find("meaning=") != std::string::npos,
               "diagnostic_format");
    const std::string du = diagnostic("grab", 0xDEADBEEFu);
    TEST_CHECK(du.find("MV_E_UNKNOWN_CODE") != std::string::npos,
               "diagnostic_unknown_code");

    return testutil::takeFailures();
}

}  // namespace suite
