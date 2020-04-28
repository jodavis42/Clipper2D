MACRO(ADD_MSVC_PRECOMPILED_HEADER ProjectName PrecompiledHeaderName PrecompiledSourceName)
  set_target_properties(${ProjectName} PROPERTIES COMPILE_FLAGS "/Yu\"${PrecompiledHeaderName}\"")
  set_source_files_properties("${PrecompiledSourceName}" PROPERTIES COMPILE_FLAGS "/Yc\"${PrecompiledHeaderName}\"")
ENDMACRO(ADD_MSVC_PRECOMPILED_HEADER)