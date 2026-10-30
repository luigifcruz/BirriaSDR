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
# The module uses std::clamp. Apple Clang defaults to an older standard.
target_compile_features(birriaSupport PRIVATE cxx_std_17)
# The installed module sits below the library directory, so it needs a runtime
# search path back to the installed libbirriasdr.
file(RELATIVE_PATH BIRRIA_MODULE_TO_LIB
     "${CMAKE_INSTALL_FULL_LIBDIR}/SoapySDR/modules${SOAPY_SDR_ABI_VERSION}"
     "${CMAKE_INSTALL_FULL_LIBDIR}")
if(APPLE)
    set_target_properties(birriaSupport PROPERTIES INSTALL_RPATH "@loader_path/${BIRRIA_MODULE_TO_LIB}")
elseif(UNIX)
    set_target_properties(birriaSupport PROPERTIES INSTALL_RPATH "$ORIGIN/${BIRRIA_MODULE_TO_LIB}")
endif()

# Explicit hardware test of the module. Deliberately not registered with CTest:
# it opens a physical dongle.
if(BUILD_TESTING)
    add_executable(test_soapy_stream tests/test_soapy_stream.cpp)
    target_compile_features(test_soapy_stream PRIVATE cxx_std_17)
    target_include_directories(test_soapy_stream PRIVATE ${SoapySDR_INCLUDE_DIRS})
    target_link_libraries(test_soapy_stream PRIVATE ${SoapySDR_LIBRARIES})
endif()
