cmake_minimum_required(VERSION 3.8)
project(qpoases_vendor)

find_package(ament_cmake REQUIRED)

# =========================================
# qpOASES vendor package
# Downloads and builds qpOASES as an
# ExternalProject, then installs headers
# and the shared library for downstream use.
# =========================================

include(ExternalProject)

set(QPOASES_VERSION "3.2.1")
set(QPOASES_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/qpOASES-${QPOASES_VERSION}")

ExternalProject_Add(qpOASES_ext
  URL "https://github.com/coin-or/qpOASES/archive/refs/tags/Version${QPOASES_VERSION}.tar.gz"
  PREFIX "${CMAKE_CURRENT_BINARY_DIR}/qpOASES"
  SOURCE_DIR "${CMAKE_CURRENT_BINARY_DIR}/qpOASES/src/qpOASES_ext"
  BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/qpOASES/build"
  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/qpOASES/install
    -DCMAKE_BUILD_TYPE=Release
    -DQPOASES_BUILD_EXAMPLES=OFF
  INSTALL_DIR "${CMAKE_CURRENT_BINARY_DIR}/qpOASES/install"
)

set(QPOASES_INSTALL_DIR "${CMAKE_CURRENT_BINARY_DIR}/qpOASES/install")

# Create imported target that downstream packages can link against
add_library(qpOASES SHARED IMPORTED GLOBAL)
add_dependencies(qpOASES qpOASES_ext)

set_target_properties(qpOASES PROPERTIES
  IMPORTED_LOCATION "${QPOASES_INSTALL_DIR}/lib/libqpOASES.so"
  INTERFACE_INCLUDE_DIRECTORIES "${QPOASES_INSTALL_DIR}/include"
)

# Install the built library and headers so they land in the ament prefix
install(
  DIRECTORY "${QPOASES_INSTALL_DIR}/include/"
  DESTINATION include
)

install(
  DIRECTORY "${QPOASES_INSTALL_DIR}/lib/"
  DESTINATION lib
  FILES_MATCHING PATTERN "*.so*"
)

# Export the package so find_package(qpoases_vendor) works
ament_export_include_directories("${QPOASES_INSTALL_DIR}/include")
ament_export_libraries(qpOASES)

ament_package()
