cmake_minimum_required(VERSION 3.8)
project(qpoases_vendor)

find_package(ament_cmake REQUIRED)

include(ExternalProject)

set(QPOASES_VERSION "3.2.1")

ExternalProject_Add(qpOASES_ext
  URL "https://github.com/coin-or/qpOASES/archive/refs/tags/releases/${QPOASES_VERSION}.tar.gz"
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

file(MAKE_DIRECTORY "${QPOASES_INSTALL_DIR}/include")
file(MAKE_DIRECTORY "${QPOASES_INSTALL_DIR}/lib")

# Install headers and library to ament install prefix
install(
  DIRECTORY "${QPOASES_INSTALL_DIR}/include/"
  DESTINATION include
)
install(
  FILES "${QPOASES_INSTALL_DIR}/lib/libqpOASES.a"
  DESTINATION lib
  OPTIONAL
)

# Write a cmake extras file that resolves the library path at find_package() time
# (CMAKE_CURRENT_LIST_DIR is evaluated when the extras file is included by downstream)
set(EXTRAS_FILE "${CMAKE_CURRENT_BINARY_DIR}/qpoases_vendor_extras.cmake")
file(WRITE "${EXTRAS_FILE}" [=[
# Resolve library relative to this cmake file's installed location:
#   <prefix>/share/qpoases_vendor/cmake/ -> <prefix>/lib/
get_filename_component(_qpoases_vendor_prefix
  "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)
set(qpoases_vendor_INCLUDE_DIRS "${_qpoases_vendor_prefix}/include")
set(qpoases_vendor_LIBRARIES    "${_qpoases_vendor_prefix}/lib/libqpOASES.a")
unset(_qpoases_vendor_prefix)
]=])

install(FILES "${EXTRAS_FILE}"
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_include_directories(include)
ament_package(CONFIG_EXTRAS "${EXTRAS_FILE}")
