set(CurrentDirectory ${CMAKE_CURRENT_LIST_DIR})

add_executable(Tests "")

include(${CMAKE_CURRENT_LIST_DIR}/CMakeFiles.cmake)

target_include_directories(Tests 
    PUBLIC
    ${CurrentDirectory}
)
Set_Common_TargetCompileOptions(Tests)
target_link_libraries(Tests
                      PUBLIC
                      Clipper
)

set_property(TARGET Tests PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:Tests>")
add_custom_command(TARGET Tests POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_LIST_DIR}/Data/ $<TARGET_FILE_DIR:Tests>/Data)
