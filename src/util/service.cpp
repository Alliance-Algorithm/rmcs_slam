#include "service.hpp"
#include "util/node.hpp"

using namespace rmcs;

void just_use_some_service_and_make_lsp_working() {
    auto node = util::make_simple_node("something");
    util::service::rmcs_slam::switch_record(*node, false);
}
