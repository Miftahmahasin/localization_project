cmake_minimum_required(VERSION 3.8)
PROJECT(sbpl)

# version (e.g. for packaging)
set(SBPL_MAJOR_VERSION 1)
set(SBPL_MINOR_VERSION 3)
set(SBPL_PATCH_VERSION 1)
set(SBPL_VERSION ${SBPL_MAJOR_VERSION}.${SBPL_MINOR_VERSION}.${SBPL_PATCH_VERSION})

# Use C++14 standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set build type to Release
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(src/include)

add_library(${PROJECT_NAME} SHARED
  src/discrete_space_information/environment_nav2D.cpp
  src/discrete_space_information/environment_navxythetalat.cpp
  src/discrete_space_information/environment_navxythetamlevlat.cpp
  src/discrete_space_information/environment_nav2Duu.cpp
  src/discrete_space_information/environment_XXX.cpp
  src/discrete_space_information/environment_robarm.cpp
  src/heuristics/embedded_heuristic.cpp
  src/planners/adplanner.cpp
  src/planners/ANAplanner.cpp
  src/planners/araplanner.cpp
  src/planners/lazyARA.cpp
  src/planners/mhaplanner.cpp
  src/planners/ppcpplanner.cpp
  src/planners/rstarplanner.cpp
  src/planners/viplanner.cpp
  src/utils/heap.cpp
  src/utils/mdp.cpp
  src/utils/utils.cpp
  src/utils/2Dgridsearch.cpp
  src/utils/config.cpp
  )

# Make include directories public
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/include>
  $<INSTALL_INTERFACE:include>
)

set(SBPL_INCLUDE_DIR "${CMAKE_INSTALL_PREFIX}/include")
set(SBPL_LIB_DIR "${CMAKE_INSTALL_PREFIX}/lib")

# support for pkg-config
set(PKG_NAME "sbpl")
set(PKG_DESC "The Search Based Planning Library")
set(PKG_SBPL_LIBS "-lsbpl")
set(pkg_conf_file_in "${CMAKE_CURRENT_SOURCE_DIR}/sbpl.pc.in")
set(pkg_conf_file "${PROJECT_BINARY_DIR}/InstallFiles/sbpl.pc")
configure_file("${pkg_conf_file_in}" "${pkg_conf_file}" @ONLY)
install(FILES ${pkg_conf_file} DESTINATION lib/pkgconfig/ COMPONENT pkgconfig)

# support for cmake-config:
configure_file(sbpl-config.cmake.in
  "${PROJECT_BINARY_DIR}/InstallFiles/sbpl-config.cmake" @ONLY)
configure_file(sbpl-config-version.cmake.in
  "${PROJECT_BINARY_DIR}/InstallFiles/sbpl-config-version.cmake" @ONLY)
install(FILES
  "${PROJECT_BINARY_DIR}/InstallFiles/sbpl-config.cmake"
  "${PROJECT_BINARY_DIR}/InstallFiles/sbpl-config-version.cmake"
DESTINATION share/sbpl/)

# support for build tree export
option(SBPL_BUILD_TREE_EXPORT "Enable build tree exports" OFF)
if (SBPL_BUILD_TREE_EXPORT)
    export(TARGETS sbpl FILE sbpl-targets.cmake)
    set(CONF_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/src/include")
    configure_file(
        sbpl-config-tree-export.cmake.in
        "${PROJECT_BINARY_DIR}/sbpl-config.cmake"
        @ONLY)
    export(PACKAGE sbpl)
endif()

add_executable(test_sbpl src/test/main.cpp)
target_link_libraries(test_sbpl sbpl)

add_executable(test_adjacency_list src/test/test_adjacency_list.cpp)
target_link_libraries(test_adjacency_list sbpl)

# Install targets
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install include files
install(DIRECTORY src/include/
  DESTINATION include
)

# Export package
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(src/include)
ament_export_libraries(${PROJECT_NAME})

# Install test executables
install(TARGETS test_sbpl test_adjacency_list
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
