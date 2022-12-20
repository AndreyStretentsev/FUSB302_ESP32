file(GLOB project_SRC
    "*.c"
    "core/*.c"
    "core/vdm/*.c"
    "core/modules/*.c"
)


idf_component_register(
    SRCS ${project_SRC}
    INCLUDE_DIRS "." "core" "core/modules" "core/vdm"
)
message("${project_SRC}")