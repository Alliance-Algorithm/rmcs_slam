#include "dedistortion.hpp"
#include "sophus/se3.hpp"
#include "util/imu.hpp"
#include "util/logger.hpp"

using namespace rmcs;

struct Dedistortion::Impl {
    RMCS_INITIALIZE_LOGGER("dedistortion");
};

Dedistortion::Dedistortion()
    : pimpl(std::make_unique<Impl>()) { }

Dedistortion::~Dedistortion() = default;
