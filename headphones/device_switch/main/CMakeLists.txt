file(GLOB SOURCES "./ble_hid/*.c" )
idf_component_register(SRCS "main.c" ${SOURCES} 
                        INCLUDE_DIRS "./ble_hid/include")
                    
# 在编译 ${COMPONENT_LIB} 时禁用有关未使用常量变量的警告
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-const-variable)
