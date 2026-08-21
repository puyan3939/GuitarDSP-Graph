cmake_minimum_required(VERSION 3.21)
project(GuitarDSPGraph VERSION 0.2.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(GUITARDSP_GRAPH_WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)

add_library(GuitarDSPGraphCore
    src/graph/Graph.cpp
    src/graph/CompiledAudioGraph.cpp
)

target_include_directories(GuitarDSPGraphCore
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(GuitarDSPGraphCore PUBLIC cxx_std_20)

if (MSVC)
    target_compile_options(GuitarDSPGraphCore PRIVATE /W4)
    if (GUITARDSP_GRAPH_WARNINGS_AS_ERRORS)
        target_compile_options(GuitarDSPGraphCore PRIVATE /WX)
    endif()
else()
    target_compile_options(GuitarDSPGraphCore PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
    if (GUITARDSP_GRAPH_WARNINGS_AS_ERRORS)
        target_compile_options(GuitarDSPGraphCore PRIVATE -Werror)
    endif()
endif()

enable_testing()

add_executable(GuitarDSPGraphTests
    tests/GraphTests.cpp
)
target_link_libraries(GuitarDSPGraphTests PRIVATE GuitarDSPGraphCore)
add_test(NAME graph_core COMMAND GuitarDSPGraphTests)

add_executable(GuitarDSPAudioRuntimeTests
    tests/AudioRuntimeTests.cpp
)
target_link_libraries(GuitarDSPAudioRuntimeTests PRIVATE GuitarDSPGraphCore)
add_test(NAME audio_runtime COMMAND GuitarDSPAudioRuntimeTests)
