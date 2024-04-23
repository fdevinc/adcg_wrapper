# ##############################################################################
# Define utility function.
# ##############################################################################
# Download and install an external project according to a provided configuration
# file.
function(download_external_project external_project_config_file
         external_project_name)
  # Create external project 'CMakeLists.txt' file inside the build folder.
  configure_file(
    ${external_project_config_file}
    "${CMAKE_CURRENT_BINARY_DIR}/${external_project_name}/CMakeLists.txt" @ONLY)

  # Create external project build directory 'ep-build'.
  file(MAKE_DIRECTORY
       ${CMAKE_CURRENT_BINARY_DIR}/${external_project_name}/ep-build)

  # Build external project.
  execute_process(
    COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" ..
    WORKING_DIRECTORY
      ${CMAKE_CURRENT_BINARY_DIR}/${external_project_name}/ep-build)
  execute_process(
    COMMAND ${CMAKE_COMMAND} --build .
    WORKING_DIRECTORY
      ${CMAKE_CURRENT_BINARY_DIR}/${external_project_name}/ep-build)
endfunction()

# ##############################################################################
# Define dependency directories.
# ##############################################################################
# Eigen.
if(DEFINED Eigen3_ROOT)
  set(EIGEN3_ROOT_DIRECTORY ${Eigen3_ROOT})
elseif(NOT ADCG_WRAPPER_USE_SYSTEM_LIBRARIES)
  set(USE_BUNDLED_EIGEN ON)
  set(BUNDLED_EIGEN_FILENAME
      ${CMAKE_CURRENT_LIST_DIR}/config/eigen/eigen-3.4.0.zip)
  set(EIGEN3_ROOT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/eigen/install)
endif()

# GoogleTest.
if(DEFINED GTest_ROOT)
  set(GTEST_ROOT_DIRECTORY ${GTest_ROOT})
elseif(NOT ADCG_WRAPPER_USE_SYSTEM_LIBRARIES)
  set(USE_DOWNLOADED_GOOGLETEST ON)
  set(GTEST_ROOT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest/install)
endif()

# spdlog.
if(DEFINED spdlog_ROOT)
  set(SPDLOG_ROOT_DIRECTORY ${spdlog_ROOT})
elseif(NOT ADCG_WRAPPER_USE_SYSTEM_LIBRARIES)
  set(USE_DOWNLOADED_SPDLOG ON)
  set(SPDLOG_ROOT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/spdlog/install)
endif()

# FiniteDiff.
if(DEFINED FiniteDiff_ROOT)
  set(FINITEDIFF_ROOT_DIRECTORY ${FiniteDiff_ROOT})
elseif(NOT ADCG_WRAPPER_USE_SYSTEM_LIBRARIES)
  set(USE_DOWNLOADED_FINITEDIFF ON)
  set(FINITEDIFF_ROOT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/finite-diff/install)
endif()

# CppAD.
if(DEFINED CppAD_ROOT)
  set(CPPAD_ROOT_DIRECTORY ${CppAD_ROOT})
elseif(NOT ADCG_WRAPPER_USE_SYSTEM_LIBRARIES)
  set(USE_DOWNLOADED_CPPAD ON)
  set(CPPAD_ROOT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/CppAD/install)
endif()

# CppADCodeGen.
if(DEFINED CppADCodeGen_ROOT)
  set(CPPADCODEGEN_ROOT_DIRECTORY ${CppADCodeGen_ROOT})
elseif(NOT ADCG_WRAPPER_USE_SYSTEM_LIBRARIES)
  set(USE_DOWNLOADED_CPPADCODEGEN ON)
  set(CPPADCODEGEN_ROOT_DIRECTORY
      ${CMAKE_CURRENT_BINARY_DIR}/CppADCodeGen/install)
endif()

# ===========================================================================
# 1) CORE DEPENDENCIES
# ##############################################################################
# Add Eigen.
# ##############################################################################
message(STATUS "--------------------------------------------------")
if(USE_BUNDLED_EIGEN)
  message(STATUS "Using bundled Eigen...")
  download_external_project(config/eigen/CMakeLists.txt.in eigen)
  find_package(Eigen3 REQUIRED PATHS
               ${EIGEN3_ROOT_DIRECTORY}/share/eigen3/cmake NO_DEFAULT_PATH)
else()
  message(STATUS "Using system-wide Eigen...")
  find_package(Eigen3 3.4.0 REQUIRED CONFIG)
endif()
set_target_properties(Eigen3::Eigen PROPERTIES IMPORTED_GLOBAL TRUE)

# If the Eigen package is found, it defines the variable 'EIGEN3_ROOT_DIR'.
# However, this feature is deprecated and 'EIGEN3_ROOT_DIR' may not be defined.
if(DEFINED EIGEN3_ROOT_DIR)
  message(STATUS "Eigen found at '${EIGEN3_ROOT_DIR}'.")
elseif(DEFINED EIGEN3_ROOT_DIRECTORY)
  message(STATUS "Eigen found at '${EIGEN3_ROOT_DIRECTORY}'.")
else()
  message(STATUS "Eigen found.")
endif()

# ===========================================================================
# 2) TESTING
if(ADCG_WRAPPER_BUILD_TESTS)
  # ############################################################################
  # Add GoogleTest.
  # ############################################################################
  message(STATUS "--------------------------------------------------")
  include(GoogleTest)

  if(USE_DOWNLOADED_GOOGLETEST)
    message(STATUS "Using locally downloaded GoogleTest...")
    download_external_project(config/googletest/CMakeLists.txt.in googletest)
    find_package(GTest CONFIG REQUIRED PATHS
                 ${GTEST_ROOT_DIRECTORY}/lib/cmake/GTest NO_DEFAULT_PATH)
  else()
    message(STATUS "Using system-wide GoogleTest...")
    find_package(GTest CONFIG REQUIRED)
  endif()
  set_target_properties(GTest::gtest PROPERTIES IMPORTED_GLOBAL TRUE)

  if(DEFINED GTEST_ROOT_DIRECTORY)
    message(STATUS "GoogleTest found at '${GTEST_ROOT_DIRECTORY}'.")
  else()
    message(STATUS "GoogleTest found.")
  endif()

endif(ADCG_WRAPPER_BUILD_TESTS)

# ===========================================================================
# 3) LOGGING
# ##############################################################################
# Add spdlog.
# ##############################################################################
message(STATUS "--------------------------------------------------")
if(USE_DOWNLOADED_SPDLOG)
  message(STATUS "Using locally downloaded spdlog...")
  download_external_project(config/spdlog/CMakeLists.txt.in spdlog)
  find_package(spdlog REQUIRED PATHS ${SPDLOG_ROOT_DIRECTORY}/lib/cmake/spdlog
               NO_DEFAULT_PATH)
else()
  message(STATUS "Using system-wide spdlog...")
  find_package(spdlog REQUIRED)
endif()
set_target_properties(spdlog::spdlog PROPERTIES IMPORTED_GLOBAL TRUE)
set_target_properties(spdlog::spdlog_header_only PROPERTIES IMPORTED_GLOBAL
                                                            TRUE)

if(DEFINED SPDLOG_ROOT_DIRECTORY)
  message(STATUS "spdlog found at '${SPDLOG_ROOT_DIRECTORY}'.")
else()
  message(STATUS "spdlog found.")
endif()

# ===========================================================================
# 4) AUTOMATIC DIFFERENTIATION
# ##############################################################################
# Add FiniteDiff.
# ##############################################################################
message(STATUS "--------------------------------------------------")
if(USE_DOWNLOADED_FINITEDIFF)
  message(STATUS "Using locally downloaded FiniteDiff...")
  download_external_project(config/finite-diff/CMakeLists.txt.in finite-diff)
  find_package(
    FiniteDiff REQUIRED PATHS ${FINITEDIFF_ROOT_DIRECTORY}/lib/cmake/finitediff
    NO_DEFAULT_PATH)
else()
  message(STATUS "Using system-wide FiniteDiff...")
  find_package(FiniteDiff REQUIRED)
endif()
set_target_properties(finitediff::finitediff PROPERTIES IMPORTED_GLOBAL TRUE)

if(DEFINED FINITEDIFF_ROOT_DIRECTORY)
  message(STATUS "FiniteDiff found at '${FINITEDIFF_ROOT_DIRECTORY}'.")
else()
  message(STATUS "FiniteDiff found.")
endif()

# ##############################################################################
# Add CppAD.
# ##############################################################################
message(STATUS "--------------------------------------------------")
if(USE_DOWNLOADED_CPPAD)
  message(STATUS "Using locally downloaded CppAD...")
  download_external_project(config/CppAD/CMakeLists.txt.in CppAD)
else()
  message(STATUS "Using system-wide CppAD...")
endif()

if(DEFINED CPPAD_ROOT_DIRECTORY)
  set(CPPAD_LIB_DIRECTORY ${CPPAD_ROOT_DIRECTORY}/lib)
  set(CPPAD_INCLUDE_DIRECTORY ${CPPAD_ROOT_DIRECTORY}/include)

  find_library(
    CppAD_LIBRARY cppad_lib
    PATHS ${CPPAD_LIB_DIRECTORY} REQUIRED
    NO_DEFAULT_PATH)
  find_path(
    CppAD_INCLUDE_DIR cppad/cppad.hpp
    PATHS ${CPPAD_INCLUDE_DIRECTORY} REQUIRED
    NO_DEFAULT_PATH)
else()
  find_library(CppAD_LIBRARY cppad_lib REQUIRED)
  find_path(CppAD_INCLUDE_DIR cppad/cppad.hpp REQUIRED)
endif()

if(DEFINED CPPAD_ROOT_DIRECTORY)
  message(STATUS "CppAD found at '${CPPAD_ROOT_DIRECTORY}'.")
else()
  message(STATUS "CppAD found.")
endif()

# Create target for CppAD.
add_library(CppAD SHARED IMPORTED GLOBAL)
add_library(CppAD::CppAD ALIAS CppAD)
target_include_directories(CppAD INTERFACE ${CppAD_INCLUDE_DIR})

get_filename_component(CppAD_LIBRARY_NAME ${CppAD_LIBRARY} NAME)
get_filename_component(CppAD_LIBRARY_REAL_PATH ${CppAD_LIBRARY} REALPATH)

set_target_properties(CppAD PROPERTIES IMPORTED_SONAME ${CppAD_LIBRARY_NAME})
set_target_properties(CppAD PROPERTIES IMPORTED_LOCATION
                                       ${CppAD_LIBRARY_REAL_PATH})

# ##############################################################################
# Add CppADCodeGen.
# ##############################################################################
message(STATUS "--------------------------------------------------")
if(USE_DOWNLOADED_CPPADCODEGEN)
  message(STATUS "Using locally downloaded CppADCodeGen...")
  download_external_project(config/CppADCodeGen/CMakeLists.txt.in CppADCodeGen)
else()
  message(STATUS "Using system-wide CppADCodeGen...")
endif()

if(DEFINED CPPADCODEGEN_ROOT_DIRECTORY)
  set(CPPADCODEGEN_INCLUDE_DIRECTORY ${CPPADCODEGEN_ROOT_DIRECTORY}/include)
  find_path(
    CppADCodeGen_INCLUDE_DIR cppad/cg.hpp
    PATHS ${CPPADCODEGEN_INCLUDE_DIRECTORY} REQUIRED
    NO_DEFAULT_PATH)
else()
  find_path(CppADCodeGen_INCLUDE_DIR cppad/cg.hpp REQUIRED)
endif()

if(DEFINED CPPADCODEGEN_ROOT_DIRECTORY)
  message(STATUS "CppADCodeGen found at '${CPPADCODEGEN_ROOT_DIRECTORY}'.")
else()
  message(STATUS "CppADCodeGen found.")
endif()

# Create target for CppADCodeGen.
add_library(CppADCodeGen INTERFACE IMPORTED GLOBAL)
add_library(CppADCodeGen::CppADCodeGen ALIAS CppADCodeGen)
target_include_directories(CppADCodeGen INTERFACE ${CppADCodeGen_INCLUDE_DIR})
