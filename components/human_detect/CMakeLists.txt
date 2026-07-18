include(${COMPONENT_DIR}/cmake/utilities.cmake)

set(src_dirs     .)
set(include_dirs .)
set(requires     esp-dl led_buzzer)

set(packed_model ${CMAKE_BINARY_DIR}/espdl_models/human_face_detect.espdl)

idf_component_register(
    SRCS "human_detect.cpp" "human_face_detect.cpp"
    INCLUDE_DIRS ${include_dirs}
    REQUIRES ${requires}
)

# ── 模型嵌入 (flash RODATA) ──
if(IDF_TARGET STREQUAL "esp32p4")
    set(models_dir ${COMPONENT_DIR}/models/p4)
endif()

file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/espdl_models)
set(models
    ${models_dir}/human_face_detect_msr_s8_v1.espdl
    ${models_dir}/human_face_detect_mnp_s8_v1.espdl
)

set(pack_model_exe ${COMPONENT_DIR}/pack_model.py)
add_custom_command(
    OUTPUT ${packed_model}
    COMMENT "Move and Pack models..."
    COMMAND python ${pack_model_exe} --model_path ${models} --out_file ${packed_model}
    DEPENDS ${models}
    VERBATIM
)

target_add_aligned_binary_data(${COMPONENT_LIB} ${packed_model} BINARY)
