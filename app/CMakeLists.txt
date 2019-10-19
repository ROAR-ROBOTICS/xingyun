add_executable(shell-app main.cpp)
include_directories(
    ${CMAKE_SOURCE_DIR}/include
)

find_package(PythonLibs)
target_include_directories(shell-app PRIVATE ${PYTHON_INCLUDE_DIR})
target_link_libraries(shell-app ${PYTHON_LIBRARY})
message(${PYTHON_INCLUDE_DIRS})


