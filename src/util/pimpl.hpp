#pragma once

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
