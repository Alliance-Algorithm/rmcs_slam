#pragma once

#include <memory>

#define RMCS_PIMPL_DEFINTION(Class)          \
public:                                      \
    Class();                                 \
    ~Class();                                \
    Class(const Class&)            = delete; \
    Class& operator=(const Class&) = delete; \
                                             \
private:                                     \
    struct Impl;                             \
    std::unique_ptr<Impl> pimpl;

namespace internal {

inline auto _internal_use_include_library_to_remove_clangd_warning() noexcept -> void {
    std::unique_ptr<int> use_here;
}

}; // namespace internal
