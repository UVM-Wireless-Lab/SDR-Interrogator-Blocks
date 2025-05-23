# Copyright 2011,2012,2016,2018,2019 Free Software Foundation, Inc.
#
# This file was generated by gr_modtool, a tool from the GNU Radio framework
# This file is a part of gr-portable_interrogator_blocks
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

########################################################################
# Setup library
########################################################################
include(GrPlatform) #define LIB_SUFFIX
list(APPEND portable_interrogator_blocks_sources
    )

set(portable_interrogator_blocks_sources "${portable_interrogator_blocks_sources}" PARENT_SCOPE)
if(NOT portable_interrogator_blocks_sources)
    MESSAGE(STATUS "No C++ sources... skipping lib/")
    return()
endif(NOT portable_interrogator_blocks_sources)

add_library(gnuradio-portable_interrogator_blocks SHARED ${portable_interrogator_blocks_sources})
target_link_libraries(gnuradio-portable_interrogator_blocks gnuradio::gnuradio-runtime)
target_include_directories(gnuradio-portable_interrogator_blocks
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
    PUBLIC $<INSTALL_INTERFACE:include>
  )
set_target_properties(gnuradio-portable_interrogator_blocks PROPERTIES DEFINE_SYMBOL "gnuradio_portable_interrogator_blocks_EXPORTS")

if(APPLE)
    set_target_properties(gnuradio-portable_interrogator_blocks PROPERTIES
        INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib"
    )
endif(APPLE)

########################################################################
# Install built library files
########################################################################
include(GrMiscUtils)
GR_LIBRARY_FOO(gnuradio-portable_interrogator_blocks)

########################################################################
# Print summary
########################################################################
message(STATUS "Using install prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "Building for version: ${VERSION} / ${LIBVER}")

########################################################################
# Build and register unit test
########################################################################
include(GrTest)

# If your unit tests require special include paths, add them here
#include_directories()
# List all files that contain Boost.UTF unit tests here
list(APPEND test_portable_interrogator_blocks_sources
)
# Anything we need to link to for the unit tests go here
list(APPEND GR_TEST_TARGET_DEPS gnuradio-portable_interrogator_blocks)

if(NOT test_portable_interrogator_blocks_sources)
    MESSAGE(STATUS "No C++ unit tests... skipping")
    return()
endif(NOT test_portable_interrogator_blocks_sources)

foreach(qa_file ${test_portable_interrogator_blocks_sources})
    GR_ADD_CPP_TEST("portable_interrogator_blocks_${qa_file}"
        ${CMAKE_CURRENT_SOURCE_DIR}/${qa_file}
    )
endforeach(qa_file)
