cmake_minimum_required(VERSION 3.19)

project(ste_control_corrector)
add_library(ste_control_corrector STATIC "corrector.hpp")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set_target_properties(ste_control_corrector PROPERTIES LINKER_LANGUAGE CXX)
