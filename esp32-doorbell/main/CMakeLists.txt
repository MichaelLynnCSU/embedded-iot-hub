idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS ".")

if(DEFINED DOORBELL_ID)
    target_compile_definitions(__idf_main PUBLIC DOORBELL_ID=${DOORBELL_ID})
endif()
