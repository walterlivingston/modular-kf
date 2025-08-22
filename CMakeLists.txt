cmake_minimum_required(VERSION 3.30)
project(mkf)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost 1.89 REQUIRED COMPONENTS filesystem)

include_directories(include dependencies/quaternions/include)

file(GLOB PROJECT_HEADERS include/*.hpp)
file(GLOB PROJECT_SOURCES src/*.cpp mkf/*.cpp)
# file(GLOB VENDORS_SOURCES )
file(GLOB PROJECT_CONFIGS CMakeLists.txt)

source_group("Headers" FILES ${PROJECT_HEADERS})
source_group("Sources" FILES ${PROJECT_SOURCES})
source_group("Vendors" FILES ${VENDORS_SOURCES})

add_executable(${PROJECT_NAME} ${PROJECT_SOURCES} ${PROJECT_HEADERS}
                               ${PROJECT_CONFIGS} )

target_link_libraries(${PROJECT_NAME} 
  PRIVATE Eigen3::Eigen
  Boost::filesystem)
