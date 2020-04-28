set(CurrentDirectory ${CMAKE_CURRENT_LIST_DIR})

add_library(Clipper "")

include(${CMAKE_CURRENT_LIST_DIR}/CMakeFiles.cmake)

target_include_directories(Clipper 
    PUBLIC
    ${CurrentDirectory}
)
Set_Common_TargetCompileOptions(Clipper)
target_link_libraries(Clipper
)
set_target_properties(Clipper PROPERTIES LINKER_LANGUAGE CXX)
