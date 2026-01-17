cmake_minimum_required(VERSION 3.20)

project(InfMVS
  VERSION 0.1.0
  DESCRIPTION "Modular C++ MVS reconstruction library"
  LANGUAGES CXX
)

# ============================================================================
# Options
# ============================================================================
option(INFMVS_BUILD_APPS   "Build CLI applications" ON)
option(INFMVS_BUILD_TESTS  "Build tests" ON)
option(INFMVS_BUILD_SHARED "Build shared libraries (DLL)" ON)
option(INFMVS_USE_OPENMP   "Enable OpenMP for parallel processing" ON)

# ============================================================================
# Global Settings
# ============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Shared library settings
if(INFMVS_BUILD_SHARED)
  set(BUILD_SHARED_LIBS ON)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS OFF)  # We use explicit export macros
endif()

# ============================================================================
# Find Dependencies (via vcpkg)
# ============================================================================
find_package(Eigen3 CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(fmt CONFIG REQUIRED)
find_package(magic_enum CONFIG REQUIRED)
find_package(assimp CONFIG REQUIRED)
find_package(CGAL CONFIG REQUIRED)
find_package(OpenMesh CONFIG REQUIRED)
find_package(PCL CONFIG REQUIRED)
find_package(OpenCV CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)

if(INFMVS_BUILD_TESTS)
  find_package(GTest CONFIG REQUIRED)
endif()

# ============================================================================
# Compiler Flags
# ============================================================================
if(MSVC)
  add_compile_options(/W4 /permissive- /utf-8)
  add_compile_definitions(_CRT_SECURE_NO_WARNINGS NOMINMAX)
else()
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================================
# Modules
# ============================================================================
add_subdirectory(modules/InfCore)
add_subdirectory(modules/InfIO)
add_subdirectory(modules/InfPointCloud)
add_subdirectory(modules/InfMesh)
add_subdirectory(modules/InfTexture)
add_subdirectory(modules/InfMeshPipeline)

# ============================================================================
# Applications
# ============================================================================
if(INFMVS_BUILD_APPS)
  add_subdirectory(apps)
endif()

# ============================================================================
# Tests
# ============================================================================
include(CTest)
if(INFMVS_BUILD_TESTS AND BUILD_TESTING)
  enable_testing()
  add_subdirectory(tests)
endif()

# ============================================================================
# Install
# ============================================================================
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# Export targets
install(EXPORT InfMVSTargets
  FILE InfMVSTargets.cmake
  NAMESPACE InfMVS::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/InfMVS
)

# Config file
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/InfMVSConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/InfMVSConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/InfMVS
)

# Version file
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/InfMVSConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/InfMVSConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/InfMVSConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/InfMVS
)
