#pragma once

#include <std_srvs/srv/trigger.hpp>

namespace rmcs::util {

#define TRIGGER_CALLBACK                                                \
    [this](                                                             \
        const std_srvs::srv::Trigger::Request::ConstSharedPtr& request, \
        const std_srvs::srv::Trigger::Response::SharedPtr& response)

} // namespace rmcs::util
