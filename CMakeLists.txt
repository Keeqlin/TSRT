cmake_minimum_required(VERSION 2.8) 

#Setting the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_CXX_EXTENSIONS OFF

# Define project name
PROJECT ("TSRT")

message(STATUS "OS NAME": ${CMAKE_SYSTEM_NAME})
if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    set(MACOSX TRUE)
endif()
if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    set(LINUX TRUE)
endif()

if (MACOSX)
    set(BOOST_ROOT /usr/local/include/Cellar/boost)
endif(MACOSX)

# Find OpenCV, you may need to set OpenCV_DIR variable
# to the absolute path to the directory containing OpenCVConfig.cmake file
# via the command line or GUI
find_package(OpenCV REQUIRED )
find_package(Eigen3 REQUIRED )
find_package(Boost REQUIRED COMPONENTS system)

# If the package has been found, several variables will
# be set, you can find the full list with descriptions
# in the OpenCVConfig.cmake file.
# Print some message showing some of them
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "Eigen3 library status:")
message(STATUS "    version: ${Eigen3_VERSION}")
message(STATUS "    include path: ${EIGEN3_INCLUDE_DIRS}")

if(Boost_FOUND)
    message(STATUS "Boost library status:")
    MESSAGE(STATUS "    Boost_INCLUDE_DIRS = ${Boost_INCLUDE_DIRS}")
    MESSAGE(STATUS "    Boost_LIBRARIES = ${Boost_LIBRARIES}")
    MESSAGE(STATUS "    Boost_LIB_VERSION = ${Boost_LIB_VERSION}")
endif(Boost_FOUND)

INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_SOURCE_DIR}/include  ${EIGEN3_INCLUDE_DIR} ${Boost_INCLUDE_DIRS}")
link_directories( ${CMAKE_BINARY_DIR})
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})

FILE(GLOB SOURCE
    "./src/*.cpp"
)

# Declare the executable target built from your sources
add_executable( ${PROJECT_NAME} ${SOURCE} )

# Link your application with OpenCV libraries
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} ${Boost_LIBRARIES}")

