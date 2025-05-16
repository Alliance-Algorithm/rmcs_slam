#pragma once
#include "util/pimpl.hpp"
#include <rclcpp/node.hpp>

class Node : public rclcpp::Node {
    RMCS_PIMPL_DEFINTION(Node);
};
