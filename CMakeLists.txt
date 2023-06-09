cmake_minimum_required (VERSION 3.8)

project("RLReorientTesting")

# Add all headers and code files
file(GLOB_RECURSE FILES_CPP "src/*.cpp")
file(GLOB_RECURSE FILES_H "src/*.h")

add_executable(RLReorientTesting ${FILES_CPP} ${FILES_H})

# Set C++ version
set_target_properties(RLReorientTesting PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(RLReorientTesting PROPERTIES CXX_STANDARD 20)

# Include RocketSim
add_subdirectory(RocketSim)

# Include RLUtilities
add_subdirectory(RLUtilities)

target_link_libraries(RLReorientTesting RocketSim rlutilities_cpp)