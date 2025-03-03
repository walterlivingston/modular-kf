cmake_minimum_required(VERSION 3.28)
cmake_policy(SET CMP0167 NEW)  # Enable modern Boost behavior

project(mkf)
add_compile_options(-std=c++17) #optional to use c++11

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(Matplot++ REQUIRED)

include_directories(include
                    ${Boost_INCLUDE_DIRS})
link_directories(${Eigen_INCLUDE_DIRS})

# file(GLOB VENDORS_SOURCES dependencies/glad/src/glad.c
#                           dependencies/stb/stb.cpp)
file(GLOB PROJECT_HEADERS include/*.h include/mkf/*.h include/examples/mekf/*.h)
file(GLOB PROJECT_SOURCES src/*.cpp src/mkf/*.cpp src/examples/mekf/*.cpp)
file(GLOB PROJECT_CONFIGS CMakeLists.txt)

source_group("Headers" FILES ${PROJECT_HEADERS})
source_group("Sources" FILES ${PROJECT_SOURCES})
# source_group("Vendors" FILES ${VENDORS_SOURCES})

add_executable(${PROJECT_NAME} ${PROJECT_SOURCES} ${PROJECT_HEADERS}
                               ${PROJECT_CONFIGS} ${PROJECT_SHADERS}
                               ${VENDORS_SOURCES})

target_link_libraries(${PROJECT_NAME}
                      Eigen3::Eigen 
                      Boost::filesystem
                      Boost::system
                      Matplot++::matplot)