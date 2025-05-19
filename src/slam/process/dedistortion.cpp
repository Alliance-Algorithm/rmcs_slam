#include "dedistortion.hpp"

using namespace rmcs;

struct Dedistortion::Impl { };

Dedistortion::Dedistortion()
    : pimpl(std::make_unique<Impl>()) { }

Dedistortion::~Dedistortion() = default;
