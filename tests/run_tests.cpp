// No-hardware MVS selection, diagnostics and collection configuration tests.
#include <iostream>
#include <string>

namespace suite {
int test_mvs_device_selector();
int test_mvs_error();
int test_config_resolver();
}  // namespace suite

int main() {
    int failures = 0;
    failures += suite::test_mvs_device_selector();
    failures += suite::test_mvs_error();
    failures += suite::test_config_resolver();
    if (failures == 0) {
        std::cout << "\n[caitu_tests] ALL PASS\n";
        return 0;
    }
    std::cout << "\n[caitu_tests] " << failures << " FAILURE(S)\n";
    return 1;
}
