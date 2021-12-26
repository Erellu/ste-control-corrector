cmake_minimum_required(VERSION 3.5)

project(ste-control-corrector-testbench LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

file(GLOB_RECURSE SRC "*.hpp")
target_sources(ste-control-corrector-testbench PRIVATE ${SRC})
