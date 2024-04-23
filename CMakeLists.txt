cmake_minimum_required(VERSION 3.20)

project(ADCGWrapper VERSION 1.0.1)

add_library(adcg_wrapper INTERFACE)
add_library(adcg_wrapper::adcg_wrapper ALIAS adcg_wrapper)
target_compile_features(adcg_wrapper INTERFACE cxx_std_20)

# ##############################################################################
# Set up CMake options.
# ##############################################################################
include(CMakeDependentOption)

option(
  ADCG_WRAPPER_USE_SYSTEM_LIBRARIES
  "Prevent automatic download of missing external dependencies. To selectively use system-wide packages, provide their installation prefixes as CMake variables of the form '<PackageName>_ROOT', where <PackageName> is the (case-preserved) name given to the find_package() call and _ROOT is literal."
  OFF)
option(ADCG_WRAPPER_BUILD_TESTS "Add targets for unit testing." OFF)
option(ADCG_WRAPPER_BUILD_EXAMPLES
       "Add targets showing ADCGWrapper's features." OFF)
option(ADCG_WRAPPER_INSTALL "Install CMake targets during install step." OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

message(STATUS "--------------------------------------------------")
message(STATUS "Configuring ADCGWrapper with the following options:")
message(STATUS "    Build examples         : ${ADCG_WRAPPER_BUILD_EXAMPLES}")
message(STATUS "    Build tests            : ${ADCG_WRAPPER_BUILD_TESTS}")
message(STATUS "    Install                : ${ADCG_WRAPPER_INSTALL}")
message(
  STATUS "    Use system libraries   : ${ADCG_WRAPPER_USE_SYSTEM_LIBRARIES}")

# Add external dependencies.
set_property(GLOBAL PROPERTY CTEST_TARGETS_ADDED 1)
add_subdirectory(external)
message(STATUS "--------------------------------------------------")

# ##############################################################################
# Add compile options.
# ##############################################################################
if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang") # Clang.
  target_compile_options(adcg_wrapper INTERFACE "-pedantic"
                                                "$<$<CONFIG:RELEASE>:-O2>")
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU") # GNU Compiler Collection (GCC).
  target_compile_options(
    adcg_wrapper
    INTERFACE "-pedantic"
              "-ftemplate-backtrace-limit=1"
              "-fconstexpr-depth=2147483647"
              "-fconstexpr-loop-limit=2147483647"
              "-fconstexpr-cache-depth=2147483647"
              "-fconstexpr-ops-limit=2147483647"
              "$<$<CONFIG:RELEASE>:-O3>")
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Intel") # Intel C++ Compiler.
  message(WARNING "Intel C++ Compiler is not supported.")
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC") # MSVC.
  target_compile_options(adcg_wrapper INTERFACE "/W4"
                                                "$<$<CONFIG:RELEASE>:/O2>")
else()
  message(WARNING "${CMAKE_CXX_COMPILER_ID} is not supported.")
endif()

# ##############################################################################
# Set ADCGWrapper's include directories and link libraries.
# ##############################################################################
# Core modules.
list(APPEND ADCG_WRAPPER_INCLUDE_DIRECTORIES
     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
     $<INSTALL_INTERFACE:include>)
list(
  APPEND
  ADCG_WRAPPER_LINK_LIBRARIES
  Eigen3::Eigen
  spdlog::spdlog
  CppAD::CppAD
  CppADCodeGen::CppADCodeGen
  ${CMAKE_DL_LIBS}
  finitediff::finitediff)

target_include_directories(adcg_wrapper
                           INTERFACE ${ADCG_WRAPPER_INCLUDE_DIRECTORIES})
target_link_libraries(adcg_wrapper INTERFACE ${ADCG_WRAPPER_LINK_LIBRARIES})

# ##############################################################################
# Set ADCGWrapper's compile definitions.
# ##############################################################################
target_compile_definitions(
  adcg_wrapper
  INTERFACE $<$<CONFIG:Debug>:_GLIBCXX_ASSERTIONS>
            $<$<CONFIG:RelWithDebInfo>:_GLIBCXX_ASSERTIONS>
            $<$<CONFIG:MinSizeRel>:ADCG_WRAPPER_CONFIG_ENABLE_RELEASE_MODE>
            $<$<CONFIG:Release>:ADCG_WRAPPER_CONFIG_ENABLE_RELEASE_MODE>)

target_compile_definitions(
  adcg_wrapper
  INTERFACE -DADCG_WRAPPER_CONFIG_ENABLE_AUTODIFF
  INTERFACE
    "ADCG_WRAPPER_CODEGEN_FOLDER=\"${CMAKE_BINARY_DIR}/adcg_wrapper_codegen\"")

# ##############################################################################
# Add optional targets.
# ##############################################################################
if(ADCG_WRAPPER_BUILD_TESTS)
  enable_testing()
  add_subdirectory(test)
endif()

if(ADCG_WRAPPER_BUILD_EXAMPLES)
  add_subdirectory(example)
endif()

if(ADCG_WRAPPER_INSTALL)
  include(GNUInstallDirs)

  # Install ADCGWrapper headers.
  install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

  # Install ADCGWrapper binaries.
  install(
    TARGETS adcg_wrapper
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

  # Install ADCGWrapper configuration files.
  include(CMakePackageConfigHelpers)

  configure_package_config_file(
    ADCGWrapperConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/adcg_wrapper)

  write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    VERSION ${PACKAGE_VERSION}
    COMPATIBILITY SameMajorVersion)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
                ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
          DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/adcg_wrapper)

  # Perform installation.
  install(
    EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE adcg_wrapper::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/adcg_wrapper)
endif(ADCG_WRAPPER_INSTALL)
