#ifndef MECHANICAL_SETTLING_CONFIG_HPP
#define MECHANICAL_SETTLING_CONFIG_HPP

constexpr int kDefaultMechanicalSettlingMs = 1200;
constexpr int kMinimumMechanicalSettlingMs = 0;
constexpr int kMaximumMechanicalSettlingMs = 60000;

int mechanicalSettlingWaitMs();

// Returns false and leaves the active value unchanged when milliseconds is
// outside the supported range.
bool setMechanicalSettlingWaitMs(int milliseconds);

#endif
