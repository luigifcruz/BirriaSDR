# SPDX-License-Identifier: MIT
# Copyright (c) 2026 The BirriaSDR Authors

# SoapySDR provides SOAPY_SDR_MODULE_UTIL, which builds the module with the
# right name, install location and ABI suffix. It links the in-tree birriasdr
# library, so the module does not need a separately installed libbirriasdr.
SOAPY_SDR_MODULE_UTIL(
    TARGET birriaSupport
    SOURCES Registration.cpp Device.cpp
    LIBRARIES birriasdr
)
target_include_directories(birriaSupport PRIVATE ${PROJECT_SOURCE_DIR}/include)
