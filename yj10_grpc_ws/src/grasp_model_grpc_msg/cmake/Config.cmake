include(CMakeFindDependencyMacro)

# Same syntax as find_package
find_package(Protobuf CONFIG REQUIRED)
find_package(gRPC CONFIG REQUIRED)

# Any extra setup

# Add the targets file
include("${CMAKE_CURRENT_LIST_DIR}/@PROJECT_NAME@Targets.cmake")