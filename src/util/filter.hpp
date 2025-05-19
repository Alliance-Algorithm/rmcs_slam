#pragma once

namespace rmcs::util {

inline double low_pass_filter(double current, double previous, double alpha) {
    return alpha * current + (1.0 - alpha) * previous;
}

}
