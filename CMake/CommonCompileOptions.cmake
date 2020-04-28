MACRO(Set_Common_TargetCompileOptions ProjectName)
  target_compile_options(${ProjectName}
        PUBLIC
            -GS
        PRIVATE
            -MP
            -fp:fast 
            )
ENDMACRO(Set_Common_TargetCompileOptions)