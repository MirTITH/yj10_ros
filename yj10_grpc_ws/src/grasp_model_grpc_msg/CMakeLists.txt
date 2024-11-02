cmake_minimum_required(VERSION 3.12)
project(grasp_model_grpc_msg VERSION 0.4.1)

find_package(Protobuf CONFIG REQUIRED)
find_package(gRPC CONFIG REQUIRED)
find_package (Python REQUIRED)

set(PROTO_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/proto")
set(PROTO_IMPORT_DIRS "${CMAKE_CURRENT_LIST_DIR}/proto")
file(MAKE_DIRECTORY ${PROTO_BINARY_DIR})

configure_file(python/pyproject.toml.in ${PROTO_BINARY_DIR}/python/pyproject.toml)

# GLOB_RECURSE: 递归搜索
# CONFIGURE_DEPENDS: 每次执行 cmake 时都更新列表
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS
    ${PROTO_IMPORT_DIRS}/*.proto
)

add_library(${PROJECT_NAME}
    ${SOURCES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${PROTO_BINARY_DIR}/cpp>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC gRPC::grpc++)

# Generate proto for cpp
protobuf_generate(TARGET ${PROJECT_NAME}
    IMPORT_DIRS ${PROTO_IMPORT_DIRS}
    PROTOC_OUT_DIR ${PROTO_BINARY_DIR}/cpp
)

protobuf_generate(
    TARGET ${PROJECT_NAME}
    LANGUAGE grpc
    PLUGIN protoc-gen-grpc=$<TARGET_FILE:gRPC::grpc_cpp_plugin>
    # PLUGIN_OPTIONS generate_mock_code=true
    GENERATE_EXTENSIONS .grpc.pb.h .grpc.pb.cc
    IMPORT_DIRS ${PROTO_IMPORT_DIRS}
    PROTOC_OUT_DIR ${PROTO_BINARY_DIR}/cpp
)

# Install proto for cpp
install(
    DIRECTORY ${PROTO_BINARY_DIR}/cpp/
    DESTINATION include/
    FILES_MATCHING PATTERN "*.pb.h"
)

install(
    DIRECTORY ${PROTO_BINARY_DIR}/cpp/
    DESTINATION src/
    FILES_MATCHING PATTERN "*.pb.cc"
)

# Generate proto for python
protobuf_generate(TARGET ${PROJECT_NAME}
    LANGUAGE python
    IMPORT_DIRS ${PROTO_IMPORT_DIRS}
    PROTOC_OUT_DIR ${PROTO_BINARY_DIR}/python
    PROTOC_OPTIONS --pyi_out=${PROTO_BINARY_DIR}/python
)

protobuf_generate(
    TARGET ${PROJECT_NAME}
    LANGUAGE grpc
    PLUGIN protoc-gen-grpc=$<TARGET_FILE:gRPC::grpc_python_plugin>
    # PLUGIN_OPTIONS generate_mock_code=true
    GENERATE_EXTENSIONS _pb2_grpc.py
    IMPORT_DIRS ${PROTO_IMPORT_DIRS}
    PROTOC_OUT_DIR ${PROTO_BINARY_DIR}/python
)

# Install proto for python
install(
    DIRECTORY python/
    DESTINATION ${PROTO_BINARY_DIR}/python
)

# Install python package using pip. BEGIN
# add_custom_command(
#     OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/pip_install.stamp
#     COMMAND ${CMAKE_COMMAND} -E echo "Running ${Python_EXECUTABLE} -m pip install"
#     COMMAND ${CMAKE_COMMAND} -E env ${Python_EXECUTABLE} -m pip install ${PROTO_BINARY_DIR}/python
#     COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_CURRENT_BINARY_DIR}/pip_install.stamp
#     COMMENT "Installing Python package"
#     VERBATIM
# )

# add_custom_target(
#     install_python ALL
#     DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/pip_install.stamp
# )

# # 将自定义目标添加到安装步骤中
# install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --target install_python)")
# Install python package using pip. END

# Create a package configuration file
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    ${PROJECT_NAME}ConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

# Export the targets to a script
install(EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION lib/cmake/${PROJECT_NAME}
)


configure_file(cmake/Config.cmake
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    COPYONLY
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    DESTINATION lib/cmake/${PROJECT_NAME}
)
