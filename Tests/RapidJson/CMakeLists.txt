set(CurrentDirectory ${CMAKE_CURRENT_LIST_DIR})

add_library(RapidJson "")

include(${CMAKE_CURRENT_LIST_DIR}/CMakeFiles.cmake)

target_include_directories(RapidJson 
    PUBLIC
    ${CurrentDirectory}
)
Set_Common_TargetCompileOptions(RapidJson)
target_link_libraries(RapidJson
                      PUBLIC
)

