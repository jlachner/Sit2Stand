cmake_minimum_required(VERSION 3.2)
project(Sit2Stand)

# ----------------------------------------------------------------------------
# Dependencies
# ----------------------------------------------------------------------------

# Find Boost
#find_package(Boost REQUIRED COMPONENTS filesystem system thread date_time chrono)

# Find Eigen (header-only)
find_package(Eigen3 REQUIRED)

# ----------------------------------------------------------------------------
# Include Directories
# ----------------------------------------------------------------------------

set(includeDir
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../Explicit-FRI/Libraries/myFRIClient/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../Explicit-FRI/Libraries/FTSensor/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../Explicit-FRI/Libraries/Explicit-cpp/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../Explicit-FRI/Libraries/Explicit-cpp/Eigen
  ${EIGEN3_INCLUDE_DIR}
)

include_directories(${includeDir})
include_directories(${Boost_INCLUDE_DIRS})

# ----------------------------------------------------------------------------
# Source and Header Files
# ----------------------------------------------------------------------------

file(GLOB_RECURSE sourceFiles ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
file(GLOB_RECURSE headerFiles ${CMAKE_CURRENT_SOURCE_DIR}/*.h)

# ----------------------------------------------------------------------------
# Executable Target
# ----------------------------------------------------------------------------

add_executable(${PROJECT_NAME} ${sourceFiles} ${headerFiles})

# ----------------------------------------------------------------------------
# Link Libraries
# ----------------------------------------------------------------------------

# Link FRIClient static library
set(FRI_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}/../../../Explicit-FRI/Libraries/myFRIClient/lib/libFRIClient.a" CACHE STRING "FRI library")
set(SENSOR_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}/../../../Explicit-FRI/Libraries/FTSensor/lib/libFTSensor.so" CACHE STRING "FTSensor library")
set(EXPLICIT_CPP_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}/../../../Explicit-FRI/Libraries/Explicit-cpp/lib/libExplicit-cpp.so" CACHE STRING "Explicit-cpp library")


target_link_libraries(${PROJECT_NAME}
  ${FRI_LIBRARY}
  ${Boost_LIBRARIES}
  ${SENSOR_LIBRARY}
  ${EXPLICIT_CPP_LIBRARY}
  Eigen3::Eigen
)

# ----------------------------------------------------------------------------
# Output Directory
# ----------------------------------------------------------------------------

set_target_properties(${PROJECT_NAME} PROPERTIES
  DEBUG_POSTFIX "${CMAKE_DEBUG_POSTFIX}"
  RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_CURRENT_SOURCE_DIR}/../build/Debug"
  RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/../build/Release"
)

# ----------------------------------------------------------------------------
# Install Targets
# ----------------------------------------------------------------------------

# Install headers
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/ DESTINATION ${CMAKE_INSTALL_PREFIX}/include
  FILES_MATCHING PATTERN "*.h"
)

# Install binary
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/application
)